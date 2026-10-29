#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GP 转向角偏置补偿控制器 (Steering Bias Compensation)
目标：跟踪 y=1 直线
包含：Rviz 可视化 (实际路径, 期望路径, 理论积分路径)
"""

import rospy
import threading
import numpy as np
import torch
import gpytorch
import tf
import csv
import os
import math
from collections import deque

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32, Header
# 导入 yhs_can_msgs
from yhs_can_msgs.msg import ctrl_cmd, ctrl_fb

# -------------------------------
# 1. 执行器偏置估计器 (GP Core)
# -------------------------------
class ActuatorBiasEstimator:
    """
    使用高斯过程估计： Steering_Sent -> Steering_Error
    """
    def __init__(self, max_points=400, min_train=20, device='cpu'):
        self.likelihood = gpytorch.likelihoods.GaussianLikelihood()
        # 增加噪声约束以提高数值稳定性
        self.likelihood.noise_covar.noise = 0.1  # 初始噪声
        self.likelihood.noise_covar.register_constraint("raw_noise", gpytorch.constraints.GreaterThan(1e-4))
        self.gp_model = None
        self.train_t = []  
        self.train_inputs = []  # [time, steer_sent]
        self.train_errors = []  # steer_actual - steer_sent
        self.max_points = max_points
        self.min_train = min_train
        self.device = device
        self.count = 0
        self.recent_errors = deque(maxlen=50)
        self.bias_estimate = 0.0
        
        # 训练控制
        self.is_training = False
        self.training_lock = threading.Lock()
        self.model_lock = threading.RLock()  # 用于保护模型访问
        
        # 缓存归一化参数（训练后更新）
        self.train_x_mean = None
        self.train_x_std = None
        self.train_y_mean = None
        self.train_y_std = None

    class GPModel(gpytorch.models.ExactGP):
        def __init__(self, train_x, train_y, likelihood):
            super().__init__(train_x, train_y, likelihood)
            self.mean_module = gpytorch.means.ConstantMean()
            
            # 使用更稳定的kernel初始化
            rbf_kernel = gpytorch.kernels.RBFKernel(ard_num_dims=2)
            # 设置较大的lengthscale初始值以提高稳定性
            rbf_kernel.lengthscale = 1.0
            
            self.covar_module = gpytorch.kernels.ScaleKernel(rbf_kernel)
            # 初始化outputscale
            self.covar_module.outputscale = 1.0

        def forward(self, x):
            mean_x = self.mean_module(x)
            covar_x = self.covar_module(x)
            return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)

    def add_measurement(self, time_s, cmd_val, actual_val):
        """
        cmd_val: 发送的转向角 (度)
        actual_val: 实际执行的转向角 (度, 由 odom 反算得到)
        """
        error = actual_val - cmd_val
        
        self.train_t.append(float(time_s))
        self.train_inputs.append([float(time_s), float(cmd_val)])
        self.train_errors.append(float(error))
        self.recent_errors.append(float(error))
        
        if len(self.recent_errors) >= 10:
            self.bias_estimate = float(np.mean(list(self.recent_errors)[-20:]))

        if len(self.train_t) > self.max_points:
            self.train_t = self.train_t[-self.max_points:]
            self.train_inputs = self.train_inputs[-self.max_points:]
            self.train_errors = self.train_errors[-self.max_points:]

        self.count += 1
        # 更保守的训练频率：每10次测量训练一次，并且至少需要30个数据点
        if len(self.train_inputs) >= 30 and self.count % 10 == 0:
            # 检查数据是否有足够的多样性
            train_x_array = np.array(self.train_inputs)
            if train_x_array.shape[0] >= 30:
                # 检查输入是否有变化（避免所有数据点相同）
                x_std = np.std(train_x_array[:, 1])  # 检查转向指令的标准差
                if x_std > 0.1:  # 至少有0.1度的变化
                    # 在后台线程中训练，避免阻塞
                    with self.training_lock:
                        if not self.is_training:
                            self.is_training = True
                            train_thread = threading.Thread(target=self._train_gp_async)
                            train_thread.daemon = True
                            train_thread.start()
                else:
                    rospy.logwarn_throttle(10.0, "Insufficient data diversity (steer std: %.3f < 0.1), skipping GP training", x_std)

    def _train_gp_async(self):
        """异步训练GP，在后台线程执行"""
        try:
            # 复制训练数据，避免训练过程中数据被修改
            train_inputs_copy = list(self.train_inputs)
            train_errors_copy = list(self.train_errors)
            
            rospy.loginfo("Starting GP training in background thread with %d samples...", len(train_inputs_copy))
            self._train_gp(train_inputs_copy, train_errors_copy)
            rospy.loginfo("GP training completed successfully")
        except Exception as e:
            rospy.logerr(f"GP training thread failed: {e}")
        finally:
            with self.training_lock:
                self.is_training = False

    def _train_gp(self, train_inputs_data=None, train_errors_data=None):
        try:
            # 使用传入的数据或当前数据
            if train_inputs_data is None:
                train_inputs_data = self.train_inputs
            if train_errors_data is None:
                train_errors_data = self.train_errors
                
            train_x = torch.tensor(train_inputs_data, dtype=torch.float32, device=self.device)
            train_y = torch.tensor(train_errors_data, dtype=torch.float32, device=self.device)
            
            # 数据标准化以提高数值稳定性
            train_x_mean = train_x.mean(dim=0)
            train_x_std = train_x.std(dim=0) + 1e-6
            train_x_normalized = (train_x - train_x_mean) / train_x_std
            
            train_y_mean = train_y.mean()
            train_y_std = train_y.std() + 1e-6
            train_y_normalized = (train_y - train_y_mean) / train_y_std
            
            # 获取模型锁进行训练
            with self.model_lock:
                if self.gp_model is None:
                    self.gp_model = self.GPModel(train_x_normalized, train_y_normalized, self.likelihood).to(self.device)
                else:
                    self.gp_model.set_train_data(inputs=train_x_normalized, targets=train_y_normalized, strict=False)

                self.gp_model.train()
                self.likelihood.train()
                
                # 增加数值稳定性：更大的jitter和更多的重试次数
                with gpytorch.settings.cholesky_jitter(1e-1), \
                     gpytorch.settings.cholesky_max_tries(8), \
                     gpytorch.settings.max_cholesky_size(1000):
                    optimizer = torch.optim.Adam(self.gp_model.parameters(), lr=0.03)
                    mll = gpytorch.mlls.ExactMarginalLogLikelihood(self.likelihood, self.gp_model)

                    for i in range(15): # 进一步减少迭代次数
                        optimizer.zero_grad()
                        output = self.gp_model(train_x_normalized)
                        loss = -mll(output, train_y_normalized)
                        loss.backward()
                        optimizer.step()
                
                # 训练完成后更新归一化参数（在锁内）
                self.train_x_mean = train_x_mean
                self.train_x_std = train_x_std
                self.train_y_mean = train_y_mean
                self.train_y_std = train_y_std
                    
                rospy.loginfo("GP training completed. Current simple bias estimate: %.2f deg", self.bias_estimate)
        except Exception as e:
            # 如果GP训练失败，继续使用简单均值估计
            rospy.logwarn_throttle(5.0, f"GP training failed: {e}. Using simple bias estimate: %.2f deg", self.bias_estimate)

    def get_bias_estimate(self, cmd_query=None, t_query=None):
        # 使用缓存的归一化参数进行预测
        if self.gp_model is not None and self.train_x_mean is not None:
            # 获取模型锁保护预测过程
            with self.model_lock:
                try:
                    if t_query is None: t_query = rospy.Time.now().to_sec()
                    if cmd_query is None: cmd_query = 0.0
                    
                    self.gp_model.eval()
                    self.likelihood.eval()
                    
                    # 使用缓存的归一化参数
                    tx = torch.tensor([[float(t_query), float(cmd_query)]], dtype=torch.float32, device=self.device)
                    tx_normalized = (tx - self.train_x_mean) / self.train_x_std
                    
                    with torch.no_grad(), \
                         gpytorch.settings.cholesky_jitter(1e-1), \
                         gpytorch.settings.cholesky_max_tries(8):
                        pred = self.likelihood(self.gp_model(tx_normalized))
                        # 反归一化预测结果
                        pred_normalized = float(pred.mean.item())
                        pred_original = pred_normalized * float(self.train_y_std.item()) + float(self.train_y_mean.item())
                        return float(pred_original)
                except Exception:
                    return float(self.bias_estimate)
        return float(self.bias_estimate)

# -------------------------------
# 2. Path Controller (修改为 y=1)
# -------------------------------
class PathController:
    def __init__(self, gamma0=1.0, kp_path=0.5, max_vel=1.0, kp_steer=0.9, ki_steer=0.0):
        self.gamma0 = gamma0
        self.kp_path = kp_path
        self.max_vel = max_vel
        self.kp_steer = kp_steer # 转向比例增益
        self.ki_steer = ki_steer # 转向积分增益
        self.integral_ang_err = 0.0

    # 修改路径定义: y = 1
    def phi(self, position):
        return position[1] - 1.0

    def grad_phi(self, position):
        return torch.tensor([0.0, 1.0], dtype=torch.float32)

    def chiP(self, position):
        phi_val = self.phi(position)
        grad_phi_val = self.grad_phi(position)
        # E旋转矩阵，使向量场沿切线方向
        E = torch.tensor([[0.0, 1.0], [-1.0, 0.0]], dtype=torch.float32)
        term1 = self.gamma0 * torch.matmul(E, grad_phi_val)
        term2 = self.kp_path * phi_val * grad_phi_val
        return term1 - term2

    def desired_heading(self, position):
        chi = self.chiP(position)
        return np.arctan2(float(chi[1]), float(chi[0]))

    def compute_desired_steer(self, position, current_yaw, dt=0.05):
        """计算理想的转向角 (Degree)"""
        desired_yaw = self.desired_heading(position)
        # 航向误差
        ang_err = np.arctan2(np.sin(desired_yaw - current_yaw), 
                             np.cos(desired_yaw - current_yaw))
        
        # 误差积分
        self.integral_ang_err += ang_err * dt
        # 积分限幅 (防止积分饱和，设定抗饱和上限，例如 0.5 rad)
        self.integral_ang_err = np.clip(self.integral_ang_err, -2.5, 2.5)
        
        # 将航向误差转换为转向角 (PI控制)
        steer_rad = self.kp_steer * ang_err + self.ki_steer * self.integral_ang_err
        steer_deg = np.degrees(steer_rad)
        
        # 物理限制
        return np.clip(steer_deg, -30.0, 30.0)

    # 用于可视化：生成向量场积分路径
    def integrate_vector_field(self, start_pos, steps=400, dt=0.1):
        path_points = []
        current_pos = torch.tensor(start_pos, dtype=torch.float32)
        for _ in range(steps):
            path_points.append((float(current_pos[0]), float(current_pos[1])))
            chi = self.chiP(current_pos)
            norm = torch.norm(chi)
            vel = (chi / norm) * self.max_vel if norm > 1e-6 else torch.zeros_like(chi)
            current_pos = current_pos + vel * dt
        return path_points

# -------------------------------
# 3. ROS Node (整合 GP + 转向噪声 + 可视化)
# -------------------------------
class GPOdomControllerNode:
    def __init__(self):
        rospy.init_node('gp_steer_controller_node', anonymous=False)

        # === 参数 ===
        self.odom_topic = rospy.get_param('~odom_topic', '/AIMhc_UGV1/odom')
        # 修改控制话题
        self.cmd_topic = rospy.get_param('~cmd_topic', '/ctrl_cmd')
        
        self.v_fixed = rospy.get_param('~v_fixed', 0.8) # 固定速度 0.3 m/s
        self.control_hz = rospy.get_param('~control_hz', 20.0)
        self.kp_steer = rospy.get_param('~kp_steer', 0.5)
        self.ki_steer = rospy.get_param('~ki_steer', 0.1)  # 引入 PI 控制参数
        self.wheelbase = rospy.get_param('~wheelbase', 0.65) # 车辆轴距，用于转换 w -> steer
        self.device = rospy.get_param('~gp_device', 'cuda' if torch.cuda.is_available() else 'cpu')

        # === 偏置与噪声设置 ===
        # 这里的单位是 度 (Degree)
        self.actuator_bias_deg = rospy.get_param('~actuator_bias_deg', 5.0) # 假设车偏了3度
        self.actuator_noise_std = rospy.get_param('~actuator_noise_std', 1.0) # 噪声标准差
        
        self.enable_compensation = rospy.get_param('~enable_compensation', True)
        self.compensation_gain = rospy.get_param('~compensation_gain', 1.0)

        # === 模块初始化 ===
        self.bias_estimator = ActuatorBiasEstimator(device=self.device)
        self.controller = PathController(max_vel=self.v_fixed, kp_steer=self.kp_steer, ki_steer=self.ki_steer)

        # === Publishers ===
        self.cmd_pub = rospy.Publisher(self.cmd_topic, ctrl_cmd, queue_size=1)
        self.phi_pub = rospy.Publisher('/phi_error', Float32, queue_size=5)
        self.bias_pub = rospy.Publisher('/bias_est_deg', Float32, queue_size=5)
        self.bias_pub = rospy.Publisher('/bias_est_deg', Float32, queue_size=5)
        self.steer_ideal_pub = rospy.Publisher('/steer_ideal', Float32, queue_size=5)
        self.steer_sent_pub = rospy.Publisher('/steer_sent', Float32, queue_size=5)
        self.steer_measured_pub = rospy.Publisher('/steer_measured', Float32, queue_size=5)
        
        # 可视化 Publishers
        self.vis_actual_path_pub = rospy.Publisher('/vis/actual_path', Path, queue_size=1)
        self.vis_ref_path_pub = rospy.Publisher('/vis/ref_path', Path, queue_size=1, latch=True)
        self.vis_vec_path_pub = rospy.Publisher('/vis/vec_path', Path, queue_size=1, latch=True)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber('/ctrl_fb', ctrl_fb, self.ctrl_fb_callback, queue_size=10)

        # === 状态变量 ===
        self.lock = threading.Lock()
        self.latest_pos = None
        self.latest_yaw = 0.0
        self.latest_linear_v = 0.0
        self.latest_steer_fb = 0.0
        self.received_odom = False
        
        self.last_steer_sent = None # 记录上一次发送的命令（不含噪声，含补偿）
        self.last_control_time = None
        self.last_cmd_time = None
        
        # 诊断计数器
        self.control_count = 0

        # === 数据记录功能 ===
        # 全过程数据记录
        self.full_data = []  # [(timestamp, lateral_error), ...]
        
        # 稳态检测
        self.steady_state_detected = False
        self.steady_state_start_time = None
        self.steady_state_window = deque(maxlen=int(10.0 * 5))  # 10秒 * 5Hz = 50个数据点
        self.steady_state_data = []  # 稳态后的10秒数据
        
        # 数据记录定时器 (5Hz)
        self.data_log_timer = rospy.Timer(rospy.Duration(1.0 / 5.0), self.data_log_timer_cb)

        # 路径记录
        self.actual_path_msg = Path()
        self.actual_path_msg.header.frame_id = "odom"

        # 初始化可视化
        self.init_visualization()

        rospy.Timer(rospy.Duration(1.0 / self.control_hz), self.control_timer_cb)
        
        rospy.loginfo("Controller Ready. Target: y=1. Comp: %s. True Bias: %.1f deg", 
                      self.enable_compensation, self.actuator_bias_deg)

    def init_visualization(self):
        """生成静态的参考路径和理论积分路径"""
        # 1. 期望路径 y=1 (绿色)
        ref_msg = Path()
        ref_msg.header.frame_id = "odom"
        ref_msg.header.stamp = rospy.Time.now()
        for x in np.arange(0, 50, 0.5):
            pose = PoseStamped()
            pose.header = ref_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = 1.0
            pose.pose.orientation.w = 1.0
            ref_msg.poses.append(pose)
        self.vis_ref_path_pub.publish(ref_msg)

        # 2. 向量场积分路径 (蓝色) - 从(0,0)出发
        vec_msg = Path()
        vec_msg.header.frame_id = "odom"
        vec_msg.header.stamp = rospy.Time.now()
        points = self.controller.integrate_vector_field(start_pos=[0.0, 0.0])
        for p in points:
            pose = PoseStamped()
            pose.header = vec_msg.header
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.orientation.w = 1.0
            vec_msg.poses.append(pose)
        self.vis_vec_path_pub.publish(vec_msg)

    def ctrl_fb_callback(self, msg):
        with self.lock:
            # 记录最新的实际执行转向角度（度）
            self.latest_steer_fb = float(msg.ctrl_fb_steering)

    def odom_callback(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        # 记录实际路径
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.actual_path_msg.poses.append(pose_stamped)
        self.actual_path_msg.header.stamp = rospy.Time.now()

        q = msg.pose.pose.orientation
        quat = (q.x, q.y, q.z, q.w)
        yaw = tf.transformations.euler_from_quaternion(quat)[2]
        
        # 获取线速度和角速度，用于反算实际转向角
        v_linear = msg.twist.twist.linear.x
        w_angular = msg.twist.twist.angular.z

        with self.lock:
            self.latest_pos = torch.tensor([ox, oy], dtype=torch.float32)
            self.latest_yaw = float(yaw)
            self.latest_linear_v = v_linear
            self.received_odom = True

            # === GP 学习步骤 ===
            # 我们需要比较 "我们发出的命令" 和 "实际发生的转向"
            if self.last_steer_sent is not None and abs(v_linear) > 0.1:
                t_s = msg.header.stamp.to_sec()
                
                # 直接从 /ctrl_fb 话题获取实际的转向角，并在传感器读取上加入一半的偏置
                sensor_bias = 0.5 * self.actuator_bias_deg
                steer_measured_deg = self.latest_steer_fb + sensor_bias 

                # 添加数据到 GP (Sent vs Measured)
                # 这里的 measured 包含了我们注入的 bias + noise
                self.bias_estimator.add_measurement(t_s, self.last_steer_sent, steer_measured_deg)
                
                # 发布解算得到的实际转向角 (直接读取的反馈)
                self.steer_measured_pub.publish(Float32(data=steer_measured_deg))

                # 发布估计值用于调试
                bias_est = self.bias_estimator.get_bias_estimate(cmd_query=self.last_steer_sent)
                self.bias_pub.publish(Float32(data=bias_est))

    def control_timer_cb(self, event):
        with self.lock:
            if not self.received_odom: return
            pos = self.latest_pos.clone()
            yaw = self.latest_yaw
            # 发布路径
            self.vis_actual_path_pub.publish(self.actual_path_msg)

        # 计算时间步长 dt
        current_time = rospy.Time.now().to_sec()
        if self.last_control_time is None:
            dt = 1.0 / self.control_hz
        else:
            dt = current_time - self.last_control_time
        self.last_control_time = current_time

        # 1. 计算理想控制量 (Steer Cmd), 传入 dt
        steer_cmd = self.controller.compute_desired_steer(pos, yaw, dt=dt)
        
        # 2. 获取 GP 偏置估计
        bias_est = self.bias_estimator.get_bias_estimate(cmd_query=steer_cmd)

        # 3. 计算补偿后的发送量
        if self.enable_compensation:
            # Send = Ideal - Bias
            steer_sent = steer_cmd - self.compensation_gain * bias_est
        else:
            steer_sent = steer_cmd

        # 范围限制
        steer_sent = np.clip(steer_sent, -30.0, 30.0)
        
        # 发布诊断信息
        self.steer_ideal_pub.publish(Float32(data=steer_cmd))
        self.steer_sent_pub.publish(Float32(data=steer_sent))

        # 4. === 注入噪声和真实偏置 (模拟执行器) ===
        # 这是为了在仿真中制造问题让 GP 去学习
        noise = np.random.normal(0, self.actuator_noise_std)
        steer_final_simulated = steer_sent + self.actuator_bias_deg + noise
        
        # 再次截断，因为物理转向角有限制
        steer_final_simulated = np.clip(steer_final_simulated, -30.0, 30.0)

        # 5. 发布控制消息 (yhs_can_msgs)
        cmd_msg = ctrl_cmd()
        cmd_msg.ctrl_cmd_gear = 4       # 前进
        cmd_msg.ctrl_cmd_velocity = self.v_fixed
        cmd_msg.ctrl_cmd_steering = steer_final_simulated # 注入噪声后的值
        cmd_msg.ctrl_cmd_Brake = 0

        self.cmd_pub.publish(cmd_msg)
        
        # 发布误差数据
        phi = self.controller.phi(pos)
        self.phi_pub.publish(Float32(data=phi.item()))
        
        # 发布偏置估计值
        self.bias_pub.publish(Float32(data=bias_est))

        # 6. 记录发送数据 (用于下一帧的 GP 训练)
        with self.lock:
            self.last_steer_sent = steer_sent # 注意：GP学习的是 "发送值" vs "实际值" 之间的关系

    def data_log_timer_cb(self, event):
        """5Hz数据记录定时器回调"""
        with self.lock:
            if not self.received_odom:
                return
            
            # 计算当前横向误差
            pos = self.latest_pos.clone()
            lateral_error = float(self.controller.phi(pos).item())
            timestamp = rospy.Time.now().to_sec()
            
            # 记录全过程数据
            self.full_data.append((timestamp, lateral_error))
            
            # 稳态检测：检查误差是否连续10秒在0.5m范围内
            self.steady_state_window.append(abs(lateral_error))
            
            # 如果窗口满了且所有误差都在0.5m范围内，认为进入稳态
            if (len(self.steady_state_window) == self.steady_state_window.maxlen and
                all(err <= 0.5 for err in self.steady_state_window)):

                if not self.steady_state_detected:
                    self.steady_state_detected = True
                    self.steady_state_start_time = timestamp
                    # 在全过程数据中添加稳态开始标志
                    self.full_data.append((timestamp, 'STEADY_STATE_START'))
                    rospy.loginfo("Steady state detected at t=%.2f s", timestamp)
            
            # 如果已检测到稳态，开始记录10秒数据
            if self.steady_state_detected and self.steady_state_start_time is not None:
                if timestamp - self.steady_state_start_time <= 10.0:
                    self.steady_state_data.append((timestamp, lateral_error))
                else:
                    # 10秒数据收集完成，保存并计算指标
                    self.save_steady_state_data()
                    self.steady_state_detected = False  # 重置，避免重复保存

    def save_steady_state_data(self):
        """保存全过程数据并计算稳态指标，然后停止程序"""
        if len(self.steady_state_data) == 0:
            return

        # 计算稳态误差均值
        errors = [abs(err) for _, err in self.steady_state_data if isinstance(err, (int, float))]
        steady_state_error_mean = np.mean(errors)

        # 创建输出目录
        output_dir = os.path.join(os.path.dirname(__file__), "data")
        os.makedirs(output_dir, exist_ok=True)

        # 保存全过程数据为CSV，并在最后一行添加稳态误差均值
        csv_filename = os.path.join(output_dir, f"steady_state_analysis_{rospy.Time.now().to_sec():.0f}.csv")
        with open(csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'lateral_error'])
            writer.writerows(self.full_data)
            # 在最后一行添加稳态误差均值
            writer.writerow(['steady_state_error_mean', steady_state_error_mean])

        # 输出结果
        rospy.loginfo("Steady state analysis completed:")
        rospy.loginfo("  Total data points: %d", len(self.full_data))
        rospy.loginfo("  Steady state data points: %d", len(self.steady_state_data))
        rospy.loginfo("  Steady state duration: %.2f s", self.steady_state_data[-1][0] - self.steady_state_data[0][0])
        rospy.loginfo("  Steady state error mean: %.4f m", steady_state_error_mean)
        rospy.loginfo("  Data saved to: %s", csv_filename)

        # 停止程序
        rospy.loginfo("Analysis complete. Shutting down...")
        rospy.signal_shutdown("Steady state analysis completed")

    def spin(self):
        try:
            rospy.spin()
        finally:
            # 程序退出时的清理（如果不是正常分析完成导致的退出）
            if hasattr(self, 'steady_state_detected') and not self.steady_state_detected:
                if len(self.full_data) > 0:
                    output_dir = os.path.join(os.path.dirname(__file__), "data")
                    os.makedirs(output_dir, exist_ok=True)

                    full_csv_filename = os.path.join(output_dir, f"full_process_data_interrupted_{rospy.Time.now().to_sec():.0f}.csv")
                    with open(full_csv_filename, 'w', newline='') as csvfile:
                        writer = csv.writer(csvfile)
                        writer.writerow(['timestamp', 'lateral_error'])
                        writer.writerows(self.full_data)

                    rospy.loginfo("Program interrupted. Full process data saved to: %s", full_csv_filename)

if __name__ == '__main__':
    try:
        node = GPOdomControllerNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
