#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
import numpy as np
import torch
import tf
import csv
import os
import math
from collections import deque

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Header
# 导入 yhs_can_msgs (确保你的环境里编译了这个包)
from yhs_can_msgs.msg import ctrl_cmd

# -------------------------------
# Path Controller
# -------------------------------
class PathController:
    def __init__(self, gamma0=1.0, kp_path=0.5, max_vel=1.0, kp_steer=0.9, ki_steer=0.0):
        self.gamma0 = gamma0
        self.kp_path = kp_path
        self.max_vel = max_vel
        self.kp_steer = kp_steer # 转向比例增益
        self.ki_steer = ki_steer # 转向积分增益
        self.integral_ang_err = 0.0

    # 1. 修改路径定义：目标直线 y = 0
    def phi(self, position):
        # 目标是 y = 0, 所以误差是 y - 0
        return position[1] - 0.0

    # 2. 修改梯度：对应 y - 0 的梯度
    def grad_phi(self, position):
        # d(y-0)/dx = 0, d(y-0)/dy = 1
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
        dx = float(chi[0].item())
        dy = float(chi[1].item())
        return np.arctan2(dy, dx)

    def compute_cmd(self, pos, current_yaw, dt=0.05):
        desired_yaw = self.desired_heading(pos)
        
        # 计算角度误差 (-pi 到 pi)
        ang_err = np.arctan2(np.sin(desired_yaw - current_yaw), np.cos(desired_yaw - current_yaw))
        
        # 误差积分
        self.integral_ang_err += ang_err * dt
        # 积分限幅 (防止积分饱和，设定抗饱和上限，例如 0.5 rad)
        self.integral_ang_err = np.clip(self.integral_ang_err, -0.5, 0.5)

        # 将角度误差转换为转向角 (PI控制)
        raw_steering_rad = self.kp_steer * ang_err + self.ki_steer * self.integral_ang_err
        
        # 转换为角度 (Degree)
        steering_deg = np.degrees(raw_steering_rad)
        
        # 3. 添加噪声到转向角 (均值0，标准差2度，可根据需要调整)
        noise = np.random.normal(8.0, 0.5)
        # current_time = rospy.Time.now().to_sec()
        # noise = 5.0 +0.5* np.sin(2.0 * np.pi * 0.5 * current_time)
        steering_deg_noisy = steering_deg + noise

        # 限制范围 -30 到 30
        steering_final = np.clip(steering_deg_noisy, -30.0, 30.0)
        
        v = float(self.max_vel)
        
        return v, steering_final, float(self.phi(pos).item())

    # 辅助函数：生成向量场积分路径（用于可视化）
    def integrate_vector_field(self, start_pos, steps=200, dt=0.1):
        path_points = []
        current_pos = torch.tensor(start_pos, dtype=torch.float32)
        
        for _ in range(steps):
            path_points.append((float(current_pos[0]), float(current_pos[1])))
            
            # 获取向量场方向
            chi = self.chiP(current_pos)
            # 归一化速度向量，模拟恒定速度运动
            norm = torch.norm(chi)
            if norm > 1e-6:
                vel = (chi / norm) * self.max_vel
            else:
                vel = torch.zeros_like(chi)
            
            # 欧拉积分
            current_pos = current_pos + vel * dt
            
        return path_points

# -------------------------------
# ROS Node
# -------------------------------
class GPOdomControllerNode:
    def __init__(self):
        rospy.init_node('gp_odom_controller_node', anonymous=False)

        # 参数设置
        self.odom_topic = rospy.get_param('~odom_topic', '/AIMhc_UGV1/odom')
        # 修改默认发布话题名为 /ctrl_cmd
        self.cmd_topic = rospy.get_param('~cmd_topic', '/ctrl_cmd') 
        self.phi_topic = rospy.get_param('~phi_topic', '/phi_error')
        self.v_fixed = rospy.get_param('~v_fixed', 0.8) # 速度设为0.3                       
        self.control_hz = rospy.get_param('~control_hz', 20.0) # 提高一点频率
        self.kp_steer = rospy.get_param('~kp_steer', 0.5) # 转向增益
        self.ki_steer = rospy.get_param('~ki_steer', 0.1) # 转向积分增益

        self.controller = PathController(max_vel=self.v_fixed, kp_steer=self.kp_steer, ki_steer=self.ki_steer)

        # 4. 修改 Publisher 类型为 yhs_can_msgs/ctrl_cmd
        self.cmd_pub = rospy.Publisher(self.cmd_topic, ctrl_cmd, queue_size=1)
        self.phi_pub = rospy.Publisher(self.phi_topic, Float32, queue_size=5)
        
        # 5. 可视化 Publisher
        self.vis_actual_path_pub = rospy.Publisher('/vis/actual_path', Path, queue_size=1)
        self.vis_ref_path_pub = rospy.Publisher('/vis/ref_path', Path, queue_size=1, latch=True) # latch=True 保证只发一次也能看到
        self.vis_vec_path_pub = rospy.Publisher('/vis/vector_field_path', Path, queue_size=1, latch=True)

        # 可配置的定位噪声（用于在里程计位置上添加高斯噪声）
        # enable_pos_noise: if True, odom positions will be perturbed before used by controller
        self.enable_pos_noise = rospy.get_param('~enable_pos_noise', True)
        # mean and std are in meters
        self.pos_noise_mean = rospy.get_param('~pos_noise_mean', 0.0)
        self.pos_noise_std = rospy.get_param('~pos_noise_std', 0.05)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)

        self.lock = threading.Lock()
        # 保存真实与带噪位置（控制使用 noisy，误差统计使用 true）
        self.latest_true = None
        self.latest_noisy = None
        self.latest_yaw = 0.0
        self.received_odom = False
        self.last_control_time = None

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

        # 路径记录器
        self.actual_path_msg = Path()
        self.actual_path_msg.header.frame_id = "odom" # 假设 odom 坐标系存在

        # 初始化可视化 (参考路径和积分路径)
        self.init_visualization()

        rospy.Timer(rospy.Duration(1.0 / self.control_hz), self.control_timer_cb)

        rospy.loginfo("Controller initialized. Target Path: y=0. Output: /ctrl_cmd with Steering Noise.")

    def init_visualization(self):
        # 1. 生成参考路径 (直线 y=0)
        ref_msg = Path()
        ref_msg.header.frame_id = "odom"
        ref_msg.header.stamp = rospy.Time.now()
        
        for x in np.arange(0, 50, 0.5): # 画 50 米长
            pose = PoseStamped()
            pose.header = ref_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = 0.0 # y = 0
            pose.pose.orientation.w = 1.0
            ref_msg.poses.append(pose)
        self.vis_ref_path_pub.publish(ref_msg)

        # 2. 生成向量场积分路径 (假设从 (0,0) 出发)
        vec_msg = Path()
        vec_msg.header.frame_id = "odom"
        vec_msg.header.stamp = rospy.Time.now()
        
        # 模拟积分
        points = self.controller.integrate_vector_field(start_pos=[0.0, 0.0], steps=500, dt=0.1)
        
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
        
        q = msg.pose.pose.orientation
        quat = (q.x, q.y, q.z, q.w)
        yaw = tf.transformations.euler_from_quaternion(quat)[2]

        # noisy_pos = torch.tensor([ox, oy], dtype=torch.float32)
        # 选择性地在位置上添加高斯噪声（不改变可视化的实际路径）
        true_pos = torch.tensor([ox, oy], dtype=torch.float32)
        if self.enable_pos_noise:
            nx = ox + float(np.random.normal(self.pos_noise_mean, self.pos_noise_std))
            ny = oy + float(np.random.normal(self.pos_noise_mean, self.pos_noise_std))
            noisy_pos = torch.tensor([nx, ny], dtype=torch.float32)
        else:
            noisy_pos = true_pos.clone()

        with self.lock:
            self.latest_true = true_pos.clone()
            self.latest_noisy = noisy_pos.clone()
            self.latest_yaw = float(yaw)
            self.received_odom = True
            
            # 更新实际路径用于可视化
            pose = PoseStamped()
            pose.header = msg.header # 使用 odom 的 frame_id
            pose.pose.position.x = ox
            pose.pose.position.y = oy
            pose.pose.orientation = q
            self.actual_path_msg.poses.append(pose)
            self.actual_path_msg.header.stamp = rospy.Time.now() # 更新时间戳

    def control_timer_cb(self, event):
        with self.lock:
            if not self.received_odom:
                return
            noisy_pos = self.latest_noisy.clone()
            current_yaw = float(self.latest_yaw)
            # 发布实际路径
            if len(self.actual_path_msg.poses) > 0:
                self.actual_path_msg.header.frame_id = "odom" # 确保 frame 一致
                self.vis_actual_path_pub.publish(self.actual_path_msg)

        # 计算时间步长 dt
        current_time = rospy.Time.now().to_sec()
        if self.last_control_time is None:
            dt = 1.0 / self.control_hz
        else:
            dt = current_time - self.last_control_time
        self.last_control_time = current_time

        try:
            # 控制使用带噪声的位置（latest_noisy）
            with self.lock:
                pos_for_control = self.latest_noisy.clone()

            v, steering_deg, _phi_from_noisy = self.controller.compute_cmd(pos_for_control, current_yaw, dt=dt)

            # 发布并记录使用真实位置计算的误差 (phi)
            with self.lock:
                if self.latest_true is not None:
                    phi_val = float(self.controller.phi(self.latest_true).item())
                else:
                    phi_val = float(self.controller.phi(pos_for_control).item())

            phi_msg = Float32()
            phi_msg.data = float(phi_val)
            self.phi_pub.publish(phi_msg)

        except Exception as e:
            rospy.logwarn("Control error: %s", e)
            return

        # 6. 构造 yhs_can_msgs/ctrl_cmd 消息
        cmd_msg = ctrl_cmd()
        cmd_msg.ctrl_cmd_gear = 4       # 前进挡
        cmd_msg.ctrl_cmd_velocity = v   # 速度
        cmd_msg.ctrl_cmd_steering = steering_deg # 带噪声的转向角 (-30 ~ 30)
        cmd_msg.ctrl_cmd_Brake = 0      # 刹车

        self.cmd_pub.publish(cmd_msg)

    def data_log_timer_cb(self, event):
        """5Hz数据记录定时器回调"""
        with self.lock:
            if not self.received_odom:
                return
            
            # 计算当前横向误差 (使用真实位置计算误差)
            true_pos = self.latest_true.clone()
            lateral_error = float(self.controller.phi(true_pos).item())
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
        csv_filename = os.path.join(output_dir, "steady_state_analysis_no_comp_{:.0f}.csv".format(rospy.Time.now().to_sec()))
        with open(csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'lateral_error'])
            writer.writerows(self.full_data)
            # 在最后一行添加稳态误差均值
            writer.writerow(['steady_state_error_mean', steady_state_error_mean])
        
        # 输出结果
        rospy.loginfo("Steady state analysis completed (No Compensation):")
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
                    
                    full_csv_filename = os.path.join(output_dir, "full_process_data_no_comp_interrupted_{:.0f}.csv".format(rospy.Time.now().to_sec()))
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