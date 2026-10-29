#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
稳定的 GP 转向角偏置补偿控制器 (Stable Steering Bias Compensation)
目标：跟踪 y=1 直线
核心修复：
1. 移除绝对时间作为GP输入（改为1D纯转角映射）。
2. 修复GP训练数据在稳态下的“方差坍缩”Bug（加入最小方差限制）。
3. 彻底移除控制器的积分项（回归纯P控制），完全依靠GP消除静差。
4. 在数据源头加入宏观运动学偏差的滑动平均滤波，压制高频观测噪声。
5. 放宽GP的噪声先验和长度尺度约束，强制输出平滑的宏观补偿信号。
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
import warnings

# 忽略 GPyTorch 数值近似矩阵时打印的警告
warnings.filterwarnings("ignore", message="A not p.d., added jitter")

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32, Header
from yhs_can_msgs.msg import ctrl_cmd

# -------------------------------
# 1. 稳定的执行器偏置估计器 (GP Core)
# -------------------------------
class ActuatorBiasEstimator:
    def __init__(self, max_points=200, min_train=20, device='cpu'):
        self.likelihood = gpytorch.likelihoods.GaussianLikelihood()
        # 【修改】大幅提高预期的测量噪声先验，防止GP过拟合瞬间的离群点
        self.likelihood.noise_covar.noise = 2.0 
        self.likelihood.noise_covar.register_constraint("raw_noise", gpytorch.constraints.GreaterThan(0.5))
        
        self.gp_model = None
        self.train_inputs = []  
        self.train_errors = []  
        self.max_points = max_points
        self.min_train = min_train
        self.device = device
        self.count = 0
        self.recent_errors = deque(maxlen=50)
        self.bias_estimate = 0.0
        
        self.is_training = False
        self.training_lock = threading.Lock()
        self.model_lock = threading.RLock()
        
        self.train_x_mean = None
        self.train_x_std = None
        self.train_y_mean = None
        self.train_y_std = None

    class GPModel(gpytorch.models.ExactGP):
        def __init__(self, train_x, train_y, likelihood):
            super().__init__(train_x, train_y, likelihood)
            self.mean_module = gpytorch.means.ConstantMean()
            
            rbf_kernel = gpytorch.kernels.RBFKernel(ard_num_dims=1)
            # 【修改】拉长核函数的长度尺度，强制使得指令映射保持极度平滑
            rbf_kernel.lengthscale = 5.0 
            
            self.covar_module = gpytorch.kernels.ScaleKernel(rbf_kernel)
            self.covar_module.outputscale = 2.0

        def forward(self, x):
            mean_x = self.mean_module(x)
            covar_x = self.covar_module(x)
            return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)

    def add_measurement(self, cmd_val, actual_val):
        error = actual_val - cmd_val
        
        self.train_inputs.append([float(cmd_val)])
        self.train_errors.append(float(error))
        self.recent_errors.append(float(error))
        
        if len(self.recent_errors) >= 10:
            self.bias_estimate = float(np.mean(list(self.recent_errors)[-20:]))

        if len(self.train_inputs) > self.max_points:
            self.train_inputs = self.train_inputs[-self.max_points:]
            self.train_errors = self.train_errors[-self.max_points:]

        self.count += 1
        if len(self.train_inputs) >= 30 and self.count % 10 == 0:
            with self.training_lock:
                if not self.is_training:
                    self.is_training = True
                    train_thread = threading.Thread(target=self._train_gp_async)
                    train_thread.daemon = True
                    train_thread.start()

    def _train_gp_async(self):
        try:
            train_inputs_copy = list(self.train_inputs)
            train_errors_copy = list(self.train_errors)
            self._train_gp(train_inputs_copy, train_errors_copy)
        except Exception as e:
            rospy.logerr(f"GP training thread failed: {e}")
        finally:
            with self.training_lock:
                self.is_training = False

    def _train_gp(self, train_inputs_data, train_errors_data):
        try:
            train_x = torch.tensor(train_inputs_data, dtype=torch.float32, device=self.device)
            train_y = torch.tensor(train_errors_data, dtype=torch.float32, device=self.device)
            
            # 防止稳态下的方差坍缩
            train_x_mean = train_x.mean(dim=0)
            train_x_std = torch.clamp(train_x.std(dim=0), min=1.0) 
            train_x_normalized = (train_x - train_x_mean) / train_x_std
            
            train_y_mean = train_y.mean()
            train_y_std = torch.clamp(train_y.std(), min=0.1)
            train_y_normalized = (train_y - train_y_mean) / train_y_std
            
            with self.model_lock:
                if self.gp_model is None:
                    self.gp_model = self.GPModel(train_x_normalized, train_y_normalized, self.likelihood).to(self.device)
                else:
                    self.gp_model.set_train_data(inputs=train_x_normalized, targets=train_y_normalized, strict=False)

                self.gp_model.train()
                self.likelihood.train()
                
                with gpytorch.settings.cholesky_jitter(1e-1), \
                     gpytorch.settings.max_cholesky_size(1000):
                    optimizer = torch.optim.Adam(self.gp_model.parameters(), lr=0.1)
                    mll = gpytorch.mlls.ExactMarginalLogLikelihood(self.likelihood, self.gp_model)
                    
                    for i in range(25): 
                        optimizer.zero_grad()
                        output = self.gp_model(train_x_normalized)
                        loss = -mll(output, train_y_normalized)
                        loss.backward()
                        optimizer.step()
                
                self.train_x_mean = train_x_mean
                self.train_x_std = train_x_std
                self.train_y_mean = train_y_mean
                self.train_y_std = train_y_std
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"GP training failed: {e}")

    def get_bias_estimate(self, cmd_query=0.0):
        if self.gp_model is not None and self.train_x_mean is not None:
            with self.model_lock:
                current_model = self.gp_model
                current_likelihood = self.likelihood
                mean = self.train_x_mean.clone()
                std = self.train_x_std.clone()
                y_mean = self.train_y_mean.clone()
                y_std = self.train_y_std.clone()

            try:
                current_model.eval()
                current_likelihood.eval()
                
                tx = torch.tensor([[float(cmd_query)]], dtype=torch.float32, device=self.device)
                tx_normalized = (tx - mean) / std
                
                with torch.no_grad():
                    pred = current_likelihood(current_model(tx_normalized))
                    pred_normalized = float(pred.mean.item())
                    pred_original = pred_normalized * float(y_std.item()) + float(y_mean.item())
                    return float(pred_original)
            except Exception:
                return float(self.bias_estimate)
        return float(self.bias_estimate)

# -------------------------------
# 2. 稳定的 Path Controller (纯P控制)
# -------------------------------
class PathController:
    def __init__(self, gamma0=1.0, kp_path=0.5, max_vel=1.0, kp_steer=0.9):
        self.gamma0 = gamma0
        self.kp_path = kp_path
        self.max_vel = max_vel
        self.kp_steer = kp_steer

    def phi(self, position):
        return position[1] - 1.0

    def grad_phi(self, position):
        return torch.tensor([0.0, 1.0], dtype=torch.float32)

    def chiP(self, position):
        phi_val = self.phi(position)
        grad_phi_val = self.grad_phi(position)
        E = torch.tensor([[0.0, 1.0], [-1.0, 0.0]], dtype=torch.float32)
        term1 = self.gamma0 * torch.matmul(E, grad_phi_val)
        term2 = self.kp_path * phi_val * grad_phi_val
        return term1 - term2

    def desired_heading(self, position):
        chi = self.chiP(position)
        return np.arctan2(float(chi[1]), float(chi[0]))

    def compute_desired_steer(self, position, current_yaw):
        desired_yaw = self.desired_heading(position)
        ang_err = np.arctan2(np.sin(desired_yaw - current_yaw), 
                             np.cos(desired_yaw - current_yaw))
        
        # 纯P控制，交由GP来处理宏观运动学偏差
        steer_rad = self.kp_steer * ang_err
        steer_deg = np.degrees(steer_rad)
        return steer_deg

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
# 3. ROS Node (整合并消解时序不对齐和抖动)
# -------------------------------
class GPOdomControllerNode:
    def __init__(self):
        rospy.init_node('gp_steer_controller_node', anonymous=False)

        self.odom_topic = rospy.get_param('~odom_topic', '/AIMhc_UGV1/odom')
        self.cmd_topic = rospy.get_param('~cmd_topic', '/ctrl_cmd')
        
        self.v_fixed = rospy.get_param('~v_fixed', 0.8) 
        self.control_hz = rospy.get_param('~control_hz', 20.0)
        self.kp_steer = rospy.get_param('~kp_steer', 0.5)
        self.wheelbase = rospy.get_param('~wheelbase', 0.65) 
        self.device = rospy.get_param('~gp_device', 'cuda' if torch.cuda.is_available() else 'cpu')

        self.actuator_bias_deg = rospy.get_param('~actuator_bias_deg', 5.0) 
        self.actuator_noise_std = rospy.get_param('~actuator_noise_std', 1.0) 
        
        self.enable_compensation = rospy.get_param('~enable_compensation', True)
        self.compensation_gain = rospy.get_param('~compensation_gain', 1.0)
        
        # 【修改】提高alpha值，由于前端已经平滑，这里减少相位滞后
        self.compensation_alpha = rospy.get_param('~compensation_alpha', 0.2) 
        self.filtered_bias_est = 0.0

        self.cmd_delay_s = rospy.get_param('~cmd_delay_s', 0.1) 
        self.cmd_history = deque(maxlen=200)
        
        # 【新增】运动学偏差观测的滑动平滑队列，过滤高频噪声
        self.measured_deviation_history = deque(maxlen=10)

        self.bias_estimator = ActuatorBiasEstimator(device=self.device)
        self.controller = PathController(max_vel=self.v_fixed, kp_steer=self.kp_steer)

        self.cmd_pub = rospy.Publisher(self.cmd_topic, ctrl_cmd, queue_size=1)
        self.phi_pub = rospy.Publisher('/phi_error', Float32, queue_size=5)
        self.bias_pub = rospy.Publisher('/bias_est_deg', Float32, queue_size=5)
        self.steer_ideal_pub = rospy.Publisher('/steer_ideal', Float32, queue_size=5)
        self.steer_sent_pub = rospy.Publisher('/steer_sent', Float32, queue_size=5)
        
        self.vis_actual_path_pub = rospy.Publisher('/vis/actual_path', Path, queue_size=1)
        self.vis_ref_path_pub = rospy.Publisher('/vis/ref_path', Path, queue_size=1, latch=True)
        self.vis_vec_path_pub = rospy.Publisher('/vis/vec_path', Path, queue_size=1, latch=True)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)

        self.lock = threading.Lock()
        self.latest_pos = None
        self.latest_yaw = 0.0
        self.latest_linear_v = 0.0
        self.received_odom = False
        
        self.full_data = []  
        self.steady_state_detected = False
        self.steady_state_start_time = None
        self.steady_state_window = deque(maxlen=int(10.0 * 5))
        self.steady_state_data = []
        
        self.actual_path_msg = Path()
        self.actual_path_msg.header.frame_id = "odom"
        
        self.init_visualization()
        rospy.Timer(rospy.Duration(1.0 / self.control_hz), self.control_timer_cb)
        rospy.Timer(rospy.Duration(1.0 / 5.0), self.data_log_timer_cb)

        rospy.loginfo("Stable Controller Ready. Target: y=1.")

    def init_visualization(self):
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

    def odom_callback(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.actual_path_msg.poses.append(pose_stamped)
        self.actual_path_msg.header.stamp = rospy.Time.now()

        q = msg.pose.pose.orientation
        quat = (q.x, q.y, q.z, q.w)
        yaw = tf.transformations.euler_from_quaternion(quat)[2]
        
        v_linear = msg.twist.twist.linear.x
        w_angular = msg.twist.twist.angular.z
        t_s = msg.header.stamp.to_sec()

        with self.lock:
            self.latest_pos = torch.tensor([ox, oy], dtype=torch.float32)
            self.latest_yaw = float(yaw)
            self.latest_linear_v = v_linear
            self.received_odom = True

            if len(self.cmd_history) > 0 and abs(v_linear) > 0.1:
                target_t = t_s - self.cmd_delay_s
                
                while len(self.cmd_history) > 1 and self.cmd_history[1][0] < target_t:
                    self.cmd_history.popleft()
                
                aligned_cmd = self.cmd_history[0][1]

                # 【修改】使用滑动平均获取宏观的运动学偏差，过滤除法带来的毛刺
                val = (w_angular * self.wheelbase) / v_linear
                instant_deviation_deg = math.degrees(math.atan(val))
                self.measured_deviation_history.append(instant_deviation_deg)
                
                smoothed_deviation_deg = sum(self.measured_deviation_history) / len(self.measured_deviation_history)

                self.bias_estimator.add_measurement(aligned_cmd, smoothed_deviation_deg)

    def control_timer_cb(self, event):
        with self.lock:
            if not self.received_odom: return
            pos = self.latest_pos.clone()
            yaw = self.latest_yaw
            self.vis_actual_path_pub.publish(self.actual_path_msg)

        current_time = rospy.Time.now().to_sec()
        
        steer_cmd = self.controller.compute_desired_steer(pos, yaw)
        raw_bias_est = self.bias_estimator.get_bias_estimate(cmd_query=steer_cmd)

        self.filtered_bias_est = (1.0 - self.compensation_alpha) * self.filtered_bias_est + self.compensation_alpha * raw_bias_est

        rospy.loginfo_throttle(1.0, "GP Predict Noise(Bias): raw=%.3f deg, filtered=%.3f deg", raw_bias_est, self.filtered_bias_est)

        if self.enable_compensation:
            steer_sent = steer_cmd - self.compensation_gain * self.filtered_bias_est
        else:
            steer_sent = steer_cmd

        steer_sent = np.clip(steer_sent, -30.0, 30.0)
        
        self.steer_ideal_pub.publish(Float32(data=steer_cmd))
        self.steer_sent_pub.publish(Float32(data=steer_sent))
        self.bias_pub.publish(Float32(data=self.filtered_bias_est))

        with self.lock:
            self.cmd_history.append((current_time, steer_sent))

        noise = self.actuator_bias_deg + np.random.normal(0, self.actuator_noise_std)
        steer_final_simulated = np.clip(steer_sent + noise, -30.0, 30.0)

        cmd_msg = ctrl_cmd()
        cmd_msg.ctrl_cmd_gear = 4
        cmd_msg.ctrl_cmd_velocity = self.v_fixed
        cmd_msg.ctrl_cmd_steering = steer_final_simulated
        cmd_msg.ctrl_cmd_Brake = 0

        self.cmd_pub.publish(cmd_msg)
        
        phi = self.controller.phi(pos)
        self.phi_pub.publish(Float32(data=phi.item()))

    def data_log_timer_cb(self, event):
        with self.lock:
            if not self.received_odom: return
            pos = self.latest_pos.clone()
            lateral_error = float(self.controller.phi(pos).item())
            timestamp = rospy.Time.now().to_sec()
            self.full_data.append((timestamp, lateral_error))
            
            self.steady_state_window.append(abs(lateral_error))
            
            if (len(self.steady_state_window) == self.steady_state_window.maxlen and
                all(err <= 0.5 for err in self.steady_state_window)):

                if not self.steady_state_detected:
                    self.steady_state_detected = True
                    self.steady_state_start_time = timestamp
                    self.full_data.append((timestamp, 'STEADY_STATE_START'))
                    rospy.loginfo("Steady state detected at t=%.2f s", timestamp)
            
            if self.steady_state_detected and self.steady_state_start_time is not None:
                if timestamp - self.steady_state_start_time <= 15.0:
                    self.steady_state_data.append((timestamp, lateral_error))
                else:
                    self.save_steady_state_data()
                    self.steady_state_detected = False

    def save_steady_state_data(self):
        if len(self.steady_state_data) == 0:
            return

        errors = [abs(err) for _, err in self.steady_state_data if isinstance(err, (int, float))]
        steady_state_error_mean = np.mean(errors)

        output_dir = os.path.join(os.path.dirname(__file__), "data")
        os.makedirs(output_dir, exist_ok=True)

        csv_filename = os.path.join(output_dir, f"steady_state_analysis_{rospy.Time.now().to_sec():.0f}.csv")
        with open(csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'lateral_error'])
            writer.writerows(self.full_data)
            writer.writerow(['steady_state_error_mean', steady_state_error_mean])

        rospy.loginfo("Steady state analysis completed:")
        rospy.loginfo("  Steady state error mean: %.4f m", steady_state_error_mean)
        rospy.loginfo("Analysis complete. Shutting down...")
        rospy.signal_shutdown("Steady state analysis completed")

    def spin(self):
        try:
            rospy.spin()
        finally:
            if hasattr(self, 'steady_state_detected') and not self.steady_state_detected:
                if len(self.full_data) > 0:
                    output_dir = os.path.join(os.path.dirname(__file__), "data")
                    os.makedirs(output_dir, exist_ok=True)
                    full_csv_filename = os.path.join(output_dir, f"stable_path_log_interrupted_{rospy.Time.now().to_sec():.0f}.csv")
                    with open(full_csv_filename, 'w', newline='') as csvfile:
                        writer = csv.writer(csvfile)
                        writer.writerow(['timestamp', 'lateral_error'])
                        writer.writerows(self.full_data)
                    rospy.loginfo("Program interrupted. Data saved to: %s", full_csv_filename)

if __name__ == '__main__':
    try:
        node = GPOdomControllerNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass