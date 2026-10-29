#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import tf
import tf.transformations
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
from yhs_can_msgs.msg import ctrl_fb
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class FTC2VFPlanner:
    # --- 状态机定义 ---
    STATE_NAVIGATING = 0
    STATE_WAITING = 1
    STATE_FINISHED = 2

    def __init__(self):
        rospy.init_node('ft_c2vf_planner_node')

        # === 1. 参数加载 ===
        self.v_max = rospy.get_param('~max_speed', 1.0)
        self.v_min = rospy.get_param('~min_speed', 0.0)
        self.omega_max = rospy.get_param('~max_omega', 1.5)
        self.wheelbase = rospy.get_param('~wheelbase', 0.66)

        if rospy.has_param('~kappa_max'):
            self.kappa_max = rospy.get_param('~kappa_max')
            self.max_steer = math.atan(self.kappa_max * self.wheelbase)
        else:
            max_steer_deg = rospy.get_param('~max_steering_angle_deg', 30.0)
            self.max_steer = math.radians(max_steer_deg)
            self.kappa_max = math.tan(self.max_steer) / self.wheelbase 

        self.gamma = rospy.get_param('~gamma', 0.5)
        self.alpha = rospy.get_param('~alpha', 0.6)
        self.beta = rospy.get_param('~beta', 0.4)
        self.k_v = rospy.get_param('~k_v', 1.0)
        self.mu = rospy.get_param('~mu', 4.0)
        self.stop_tol = rospy.get_param('~stop_tolerance', 0.15)
        self.yaw_tol = math.radians(rospy.get_param('~yaw_tolerance_deg', 10.0))

        # === 2. 坐标映射与多目标点初始化 ===
        # 你的 odom_relative_node 会将起点归零
        # 为了与 MATLAB [5, -4, 0] 坐标系完全对齐，增加绝对坐标映射
        self.map_offset_x = 5.0
        self.map_offset_y = -4.0
        self.map_offset_yaw = 0.0

        # MATLAB 代码中的 7 个序列目标 (x, y, theta)
        self.waypoints = [
            (-1.0,   2.0,  math.pi/2),
            ( 3.0,  -1.0,  math.pi/4),
            ( 7.0,   7.0,  math.pi),
            (-10.0, -6.0, -math.pi/4),
            (-11.0,  1.0, -3*math.pi/4),
            ( 10.0, -2.0,  math.pi/2),
            ( 5.0,  -4.0,  0.0)
        ]
        
        self.current_wp_idx = 0
        self.state = self.STATE_NAVIGATING
        self.wait_start_time = rospy.Time(0)
        self.wait_duration = 2.0  # 停车等待时间 (秒)

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        
        # 多源数据反馈
        self.actual_v = 0.0    # 底盘线速度
        self.actual_w = 0.0    # 底盘打角反推角速度
        self.v_imu = 0.0       # 惯导(Odom)线速度
        self.w_imu = 0.0       # IMU原始角速度

        # 引导圆半径
        self.r = (3.0 + 2.0 * self.gamma) / self.kappa_max
        
        # 激活第一个目标点
        self.set_target(self.waypoints[self.current_wp_idx])

        # === 3. ROS 通信 ===
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb, queue_size=1)
        self.sub_fb = rospy.Subscriber('/ctrl_fb', ctrl_fb, self.fb_cb, queue_size=1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        self.sub_imu = rospy.Subscriber('/imu/data_raw', Imu, self.imu_cb, queue_size=1)
        
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_viz = rospy.Publisher('/ft_c2vf/markers', Marker, queue_size=10)
        self.pub_debug = rospy.Publisher('/ft_c2vf/debug_info', String, queue_size=1)

        self.tf_listener = tf.TransformListener()
        self.timer = rospy.Timer(rospy.Duration(0.01), self.control_loop)

    def set_target(self, wp):
        """设置当前目标点并更新流形参数"""
        self.target_x = wp[0]
        self.target_y = wp[1]
        self.target_theta = wp[2]
        self.update_manifold()
        rospy.loginfo("�� 正在前往目标 %d: (%.2f, %.2f, %.1f°)", 
                      self.current_wp_idx + 1, self.target_x, self.target_y, math.degrees(self.target_theta))

    def update_manifold(self):
        self.cx = self.target_x - self.r * math.sin(self.target_theta)
        self.cy = self.target_y + self.r * math.cos(self.target_theta)

    def fb_cb(self, msg):
        self.actual_v = msg.ctrl_fb_velocity
        steer_rad = math.radians(msg.ctrl_fb_steering)
        self.actual_w = self.actual_v * math.tan(steer_rad) / self.wheelbase

    def odom_cb(self, msg):
        self.v_imu = msg.twist.twist.linear.x

    def imu_cb(self, msg):
        self.w_imu = msg.angular_velocity.z

    def goal_cb(self, msg):
        """保留手动发布目标点的功能，作为调试干预使用"""
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, self.target_theta = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.state = self.STATE_NAVIGATING
        self.update_manifold()
        rospy.logwarn("⚠️ 检测到手动目标干预，已切换至新目标: (%.2f, %.2f)", self.target_x, self.target_y)

    def control_loop(self, event):
        # 1. 任务完成检测
        if self.state == self.STATE_FINISHED:
            self.pub_cmd.publish(Twist())  # 确保车辆静止
            return

        try:
            (trans, rot) = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
            sensor_x, sensor_y = trans[0], trans[1]
            _, _, raw_theta = tf.transformations.euler_from_quaternion(rot)

            # 杆臂修正 (天线在前 66cm, 偏左 20cm)
            offset_bx = -0.8675
            offset_by = -0.1
            world_dx = offset_bx * math.cos(raw_theta) - offset_by * math.sin(raw_theta)
            world_dy = offset_bx * math.sin(raw_theta) + offset_by * math.cos(raw_theta)
            
            # ===== 将 odom 的相对坐标映射到 MATLAB 全局绝对坐标 =====
            x = sensor_x + world_dx + self.map_offset_x
            y = sensor_y + world_dy + self.map_offset_y
            theta = raw_theta + self.map_offset_yaw
        except: 
            return

        # 核心参数预设
        kt = kn = kd = theta_e = psi = kappa_c = vr = v_cmd = w_cmd = 0.0
        dist_to_goal = math.hypot(x - self.target_x, y - self.target_y)
        yaw_diff = abs(math.atan2(math.sin(theta - self.target_theta), math.cos(theta - self.target_theta)))
        
        # 2. 状态机逻辑处理
        if self.state == self.STATE_NAVIGATING:
            # 抵达判定
            if dist_to_goal < self.stop_tol and yaw_diff < self.yaw_tol:
                rospy.loginfo("✅ 成功到达目标 %d！准备停车等待 %.1f 秒...", self.current_wp_idx + 1, self.wait_duration)
                self.state = self.STATE_WAITING
                self.wait_start_time = rospy.Time.now()
                self.pub_cmd.publish(Twist())  # 立即下发零速指令
                self.publish_debug(x, y, theta, dist_to_goal, 0, 0, 0, 0, 0, 0, 0, 0, 0)
                return

        elif self.state == self.STATE_WAITING:
            self.pub_cmd.publish(Twist())  # 持续保持驻车
            elapsed_time = (rospy.Time.now() - self.wait_start_time).to_sec()
            
            if elapsed_time >= self.wait_duration:
                self.current_wp_idx += 1
                if self.current_wp_idx >= len(self.waypoints):
                    rospy.loginfo("�� 所有目标点途径完毕，最终抵达 Target 7！实验结束。")
                    self.state = self.STATE_FINISHED
                else:       
                    self.set_target(self.waypoints[self.current_wp_idx])
                    self.state = self.STATE_NAVIGATING
            
            # 在等待期间仍然更新标记和调试信息
            self.publish_markers(x, y, theta)
            self.publish_debug(x, y, theta, dist_to_goal, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            return

        # 3. FT-C2VF 向量场解算 (仅在 NAVIGATING 状态下执行)
        xc, yc = x - self.cx, y - self.cy
        dist_c = math.hypot(xc, yc)
        if dist_c < 1e-4: return
        eta = dist_c / self.r
        g_phi_x, g_phi_y = 2 * xc, 2 * yc
        
        if eta <= 1.0:
            kt = 1.0 - (1.0 - eta)**(1.0 + self.gamma)
            kn = math.sqrt(max(0.0, 1.0 - kt**2))
            kd = (kt + eta * (1.0 + self.gamma) * (1.0 - eta)**self.gamma) / (self.r * eta)
        else:
            kt = 1.0 - (1.0 - 1.0 / eta)**(1.0 + self.gamma)
            kn = -math.sqrt(max(0.0, 1.0 - kt**2))
            kd = (kt - ((1.0 + self.gamma) / eta) * (1.0 - 1.0/eta)**self.gamma) / (self.r * eta)

        chi_ang = math.atan2(kt * g_phi_x + kn * g_phi_y, kt * (-g_phi_y) + kn * g_phi_x)
        
        # 误差计算
        theta_e = math.atan2(math.sin(theta - chi_ang), math.cos(theta - chi_ang))
        psi = (x - self.target_x)**2 + (y - self.target_y)**2 + self.mu * (1.0 - math.cos(theta_e))
        kappa_c = kd - (self.kappa_max - abs(kd)) * math.copysign(1.0, math.sin(theta_e)) * (abs(math.sin(theta_e)))**self.alpha

        # 控制指令
        vr = self.v_max if abs(kappa_c) < 1e-6 else self.v_min + 1.0 / (1.0/max(1e-6, self.v_max-self.v_min) + abs(kappa_c)/max(1e-6, self.omega_max - self.v_min * abs(kappa_c)))
        v_cmd = self.v_min + (vr - self.v_min) * (1.0 - math.exp(-self.k_v * psi**self.beta))
        w_cmd = max(-self.max_steer, min(self.max_steer, math.atan(kappa_c * self.wheelbase)))

        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v_cmd, w_cmd
        self.pub_cmd.publish(cmd)
        
        self.publish_markers(x, y, theta)
        self.publish_debug(x, y, theta, dist_to_goal, kt, kn, kd, theta_e, psi, kappa_c, vr, v_cmd, w_cmd)

    def publish_debug(self, x, y, theta, dist_to_goal, kt, kn, kd, theta_e, psi, kappa_c, vr, cmd_v, cmd_w):
        debug_dict = {
            "0_Task_Status": {
                "state": ["NAVIGATING", "WAITING", "FINISHED"][self.state],
                "current_target": self.current_wp_idx + 1
            },
            "1_Pose_Absolute": {"x": round(x, 4), "y": round(y, 4), "theta_deg": round(math.degrees(theta), 2)},
            "2_Velocity_FB": {"v_fb": round(self.actual_v, 4), "w_fb": round(self.actual_w, 4)},
            "3_Velocity_IMU": {"v_imu": round(self.v_imu, 4), "w_imu": round(self.w_imu, 4)},
            "4_Velocity_Cmd": {"v_cmd": round(cmd_v, 4), "w_cmd": round(cmd_w, 4), "vr": round(vr, 4)},
            "5_Errors": {
                "distance_to_target": round(dist_to_goal, 4), 
                "theta_e_rad": round(theta_e, 4),
                "theta_e_deg": round(math.degrees(theta_e), 2)
            },
            "6_Algorithm_Params": {
                "kappa_c": round(kappa_c, 4), "psi": round(psi, 4),
                "kt": round(kt, 4), "kn": round(kn, 4), "kd": round(kd, 4)
            }
        }
        self.pub_debug.publish(json.dumps(debug_dict, indent=2))

    def publish_markers(self, x, y, theta):
        t = rospy.Time.now()
        def create_marker(ns, id, mtype, color, scale):
            m = Marker(); m.header.frame_id = "odom"; m.header.stamp = t
            m.ns, m.id, m.type, m.action = ns, id, mtype, Marker.ADD
            m.color.r, m.color.g, m.color.b, m.color.a = color
            m.scale.x, m.scale.y, m.scale.z = scale
            return m
        
        # 为了让 RViz 中也正确显示，标记必须减去 offset
        # 因为 RViz 监听的是 odom 坐标系 (0,0 原点)，而上面记录的 target_x 等是绝对坐标
        viz_cx = self.cx - self.map_offset_x
        viz_cy = self.cy - self.map_offset_y
        viz_x = x - self.map_offset_x
        viz_y = y - self.map_offset_y
        viz_target_x = self.target_x - self.map_offset_x
        viz_target_y = self.target_y - self.map_offset_y

        # 引导圆
        m1 = create_marker("manifold", 0, Marker.CYLINDER, (0,1,0,0.3), (self.r*2, self.r*2, 0.02))
        m1.pose.position.x, m1.pose.position.y, m1.pose.orientation.w = viz_cx, viz_cy, 1.0
        self.pub_viz.publish(m1)
        
        # 车箭头
        m2 = create_marker("car", 1, Marker.ARROW, (0,0,1,1), (0.8, 0.15, 0.15))
        m2.pose.position.x, m2.pose.position.y = viz_x, viz_y
        m2.pose.orientation.x, m2.pose.orientation.y, m2.pose.orientation.z, m2.pose.orientation.w = tf.transformations.quaternion_from_euler(0, 0, theta)
        self.pub_viz.publish(m2)
        
        # 目标箭头
        m3 = create_marker("target", 2, Marker.ARROW, (1,0,0,1), (1.0, 0.2, 0.2))
        m3.pose.position.x, m3.pose.position.y = viz_target_x, viz_target_y
        m3.pose.orientation.x, m3.pose.orientation.y, m3.pose.orientation.z, m3.pose.orientation.w = tf.transformations.quaternion_from_euler(0, 0, self.target_theta)
        self.pub_viz.publish(m3)

if __name__ == '__main__':
    try: FTC2VFPlanner(); rospy.spin()
    except: pass