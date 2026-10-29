#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import tf
import tf.transformations
import json
import csv
import os
from datetime import datetime

from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
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

        # 注册 ROS 关闭回调以安全关闭 CSV 文件
        rospy.on_shutdown(self.shutdown_hook)

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
        
        self.soft_start_k = rospy.get_param('~soft_start_k', 1.0) 
        self.eps = rospy.get_param('~eps', 0.05)

        # === 多循环与 CSV 记录参数 ===
        self.max_loops = rospy.get_param('~num_loops', 1)  # 设定循环次数，默认为1
        self.current_loop_idx = 0  # 当前正在执行的循环索引
        
        # [修改] 将默认路径设置为你指定的 ROS 工作空间路径
        time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_csv_path = os.path.expanduser(f'~/workspace/ros1_vehicles/src/obs_void/bringup/data/ft_c2vf_debug_log_{time_str}.csv')
        self.csv_path = rospy.get_param('~csv_path', default_csv_path)
        self.csv_file = None
        self.csv_writer = None
        
        # 用于记录每到达一个目标点时的误差
        self.last_reached_dist_error = 0.0
        self.last_reached_yaw_error = 0.0

        # === 2. 状态与序列多目标初始化 ===
        self.is_initialized = False
        self.initial_theta = 0.0  
        self.init_x_raw = 0.0     
        self.init_y_raw = 0.0     
        
        self.nav_start_time = None 

        # 7 个标准目标点 (x, y, theta)
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
        self.wait_duration = 2.0

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        
        self.actual_v = 0.0
        self.actual_w = 0.0
        self.v_imu = 0.0
        self.w_imu = 0.0

        self.r = (3.0 + 2.0 * self.gamma) / self.kappa_max
        self.set_target(self.waypoints[self.current_wp_idx])

        # === 3. ROS 通信 ===
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb, queue_size=1)
        self.sub_fb = rospy.Subscriber('/ctrl_fb', ctrl_fb, self.fb_cb, queue_size=1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        self.sub_imu = rospy.Subscriber('/imu/data_raw', Imu, self.imu_cb, queue_size=1)
        
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_viz = rospy.Publisher('/ft_c2vf/markers', Marker, queue_size=10)
        self.pub_global_waypoints = rospy.Publisher('/ft_c2vf/global_waypoints', MarkerArray, queue_size=1, latch=True)
        
        # ================== 独立话题发布器 ==================
        self.debug_pubs = {}
        debug_topics = [
            'state', 'current_target',
            'x', 'y', 'theta_deg',
            'v_fb', 'w_fb',
            'v_imu', 'w_imu',
            'v_cmd', 'w_cmd', 'vr',
            'distance_to_target', 'theta_e_rad', 'theta_e_deg',
            'kappa_c', 'psi', 'kt', 'kn', 'kd',
            'reached_dist_error', 'reached_yaw_error_deg' 
        ]
        for topic in debug_topics:
            self.debug_pubs[topic] = rospy.Publisher(f'/ft_c2vf/debug/{topic}', Float64, queue_size=1)
        # ====================================================

        self.tf_listener = tf.TransformListener()
        self.timer = rospy.Timer(rospy.Duration(0.001), self.control_loop)

    def shutdown_hook(self):
        """ ROS 节点关闭时的回调，确保 CSV 正常保存关闭 """
        if self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.close()
            rospy.loginfo(f"�� 节点已终止，CSV 记录文件已安全保存至: {self.csv_path}")

    def pub_debug_f64(self, name, value):
        """ 发布指定的 Debug Float64 话题 """
        if name in self.debug_pubs:
            msg = Float64()
            msg.data = float(value)
            self.debug_pubs[name].publish(msg)

    def set_target(self, wp):
        self.target_x = wp[0]
        self.target_y = wp[1]
        self.target_theta = wp[2]
        self.update_manifold()
        rospy.loginfo("�� 正在前往 循环 %d 目标 %d: (%.2f, %.2f, %.1f°)", 
                      self.current_loop_idx + 1, self.current_wp_idx + 1, self.target_x, self.target_y, math.degrees(self.target_theta))

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
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, self.target_theta = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.state = self.STATE_NAVIGATING
        self.update_manifold()
        self.nav_start_time = rospy.Time.now()
        rospy.logwarn("⚠️ 检测到手动目标干预，已切换至新目标: (%.2f, %.2f)", self.target_x, self.target_y)

    def mat_to_odom(self, mx, my, mtheta):
        if not self.is_initialized:
            return mx, my, mtheta
        
        x_unoffset = mx - 5.0
        y_unoffset = my + 4.0
        
        cos_init = math.cos(self.initial_theta)
        sin_init = math.sin(self.initial_theta)
        
        dx_raw = x_unoffset * cos_init - y_unoffset * sin_init
        dy_raw = x_unoffset * sin_init + y_unoffset * cos_init
        
        ox = dx_raw + self.init_x_raw
        oy = dy_raw + self.init_y_raw
        otheta = mtheta + self.initial_theta
        
        return ox, oy, otheta

    def publish_global_waypoints(self):
        if not self.is_initialized: return
        
        marker_array = MarkerArray()
        t = rospy.Time.now()
        
        for i, wp in enumerate(self.waypoints):
            ox, oy, otheta = self.mat_to_odom(wp[0], wp[1], wp[2])
            
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = t
            m.ns = "global_waypoints"
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            
            if i == self.current_wp_idx:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 1.0 
                m.scale.x, m.scale.y, m.scale.z = 1.2, 0.25, 0.25
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.5, 0.0, 0.5, 0.6 
                m.scale.x, m.scale.y, m.scale.z = 0.8, 0.15, 0.15
                
            m.pose.position.x = ox
            m.pose.position.y = oy
            m.pose.position.z = 0.0
            
            q = tf.transformations.quaternion_from_euler(0, 0, otheta)
            m.pose.orientation.x = q[0]
            m.pose.orientation.y = q[1]
            m.pose.orientation.z = q[2]
            m.pose.orientation.w = q[3]
            
            marker_array.markers.append(m)
            
            text_m = Marker()
            text_m.header.frame_id = "odom"
            text_m.header.stamp = t
            text_m.ns = "global_waypoints_text"
            text_m.id = i + 100
            text_m.type = Marker.TEXT_VIEW_FACING
            text_m.action = Marker.ADD
            text_m.pose.position.x = ox
            text_m.pose.position.y = oy + 0.5
            text_m.pose.position.z = 0.5
            text_m.scale.z = 0.4
            text_m.color.r, text_m.color.g, text_m.color.b, text_m.color.a = 1.0, 1.0, 1.0, 1.0
            text_m.text = f"T{i+1}"
            
            marker_array.markers.append(text_m)

        self.pub_global_waypoints.publish(marker_array)

    def control_loop(self, event):
        if self.state == self.STATE_FINISHED:
            self.pub_cmd.publish(Twist())
            return

        try:
            (trans, rot) = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
            sensor_x, sensor_y = trans[0], trans[1]
            _, _, theta_raw = tf.transformations.euler_from_quaternion(rot)

            offset_bx = -0.8675
            offset_by = -0.1
            world_dx = offset_bx * math.cos(theta_raw) - offset_by * math.sin(theta_raw)
            world_dy = offset_bx * math.sin(theta_raw) + offset_by * math.cos(theta_raw)
            x_raw = sensor_x + world_dx
            y_raw = sensor_y + world_dy
        except: 
            return

        # ================= 初始化机制 =================
        if not self.is_initialized:
            self.initial_theta = theta_raw
            self.init_x_raw = x_raw  
            self.init_y_raw = y_raw
            self.is_initialized = True
            
            self.nav_start_time = rospy.Time.now() 
            
            rospy.loginfo("�� 规划器初始朝向已锁定: %.2f 度. 坐标系已整体旋转并平移对齐 MATLAB 空间.", math.degrees(self.initial_theta))
            self.publish_global_waypoints()
            
            # 启动并初始化 CSV 日志记录
            try:
                os.makedirs(os.path.dirname(os.path.abspath(self.csv_path)), exist_ok=True)
                self.csv_file = open(self.csv_path, 'w', newline='')
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow([
                    'timestamp', 'state', 'current_loop', 'current_target',
                    'x', 'y', 'theta_deg',
                    'v_fb', 'w_fb', 'v_imu', 'w_imu',
                    'v_cmd', 'w_cmd', 'vr',
                    'distance_to_target', 'theta_e_rad', 'theta_e_deg',
                    'kappa_c', 'psi', 'kt', 'kn', 'kd',
                    'reached_dist_error', 'reached_yaw_error_deg'
                ])
                rospy.loginfo(f"�� 开启 CSV 数据记录: {self.csv_path}")
            except Exception as e:
                rospy.logerr(f"❌ 无法创建 CSV 文件 {self.csv_path}: {e}")
                self.csv_writer = None

        theta_mat = theta_raw - self.initial_theta
        theta_mat = math.atan2(math.sin(theta_mat), math.cos(theta_mat))

        dx_raw = x_raw - self.init_x_raw
        dy_raw = y_raw - self.init_y_raw

        cos_init = math.cos(self.initial_theta)
        sin_init = math.sin(self.initial_theta)
        x_rotated = dx_raw * cos_init + dy_raw * sin_init
        y_rotated = -dx_raw * sin_init + dy_raw * cos_init

        x_mat = x_rotated + 5.0
        y_mat = y_rotated - 4.0

        dist_to_goal = math.hypot(x_mat - self.target_x, y_mat - self.target_y)
        yaw_diff = abs(math.atan2(math.sin(theta_mat - self.target_theta), math.cos(theta_mat - self.target_theta)))
        
        # ================= 状态机逻辑 =================
        if self.state == self.STATE_NAVIGATING:
            if dist_to_goal < self.stop_tol and yaw_diff < self.yaw_tol:
                # 记录到达时刻的误差
                self.last_reached_dist_error = dist_to_goal
                self.last_reached_yaw_error = math.degrees(yaw_diff)
                self.pub_debug_f64('reached_dist_error', self.last_reached_dist_error)
                self.pub_debug_f64('reached_yaw_error_deg', self.last_reached_yaw_error)
                
                rospy.loginfo("✅ 成功到达 循环 %d 目标 %d！位置误差: %.3f, 角度误差: %.2f°。准备停车等待 %.1f 秒...", 
                              self.current_loop_idx + 1, self.current_wp_idx + 1, 
                              self.last_reached_dist_error, self.last_reached_yaw_error, self.wait_duration)
                
                self.state = self.STATE_WAITING
                self.wait_start_time = rospy.Time.now()
                self.pub_cmd.publish(Twist())
                self.publish_debug(x_mat, y_mat, theta_mat, dist_to_goal, 0, 0, 0, 0, 0, 0, 0, 0, 0)
                return

        elif self.state == self.STATE_WAITING:
            self.pub_cmd.publish(Twist())
            elapsed_time = (rospy.Time.now() - self.wait_start_time).to_sec()
            
            if elapsed_time >= self.wait_duration:
                self.current_wp_idx += 1
                
                # 多循环控制逻辑
                if self.current_wp_idx >= len(self.waypoints):
                    self.current_loop_idx += 1
                    
                    if self.current_loop_idx >= self.max_loops:
                        rospy.loginfo(f"�� 预设的 {self.max_loops} 个循环已全部执行完毕！实验结束。")
                        self.state = self.STATE_FINISHED
                        if self.csv_file is not None and not self.csv_file.closed:
                            self.csv_file.close()
                            rospy.loginfo(f"�� CSV 记录已保存至: {self.csv_path}")
                    else:
                        rospy.loginfo(f"�� 正在开启第 {self.current_loop_idx + 1}/{self.max_loops} 个循环...")
                        self.current_wp_idx = 0  
                        self.set_target(self.waypoints[self.current_wp_idx])
                        self.state = self.STATE_NAVIGATING
                        self.nav_start_time = rospy.Time.now()
                        self.publish_global_waypoints()
                else:
                    self.set_target(self.waypoints[self.current_wp_idx])
                    self.state = self.STATE_NAVIGATING
                    self.nav_start_time = rospy.Time.now()
                    self.publish_global_waypoints()
            
            self.publish_markers(x_mat, y_mat, theta_mat)
            self.publish_debug(x_mat, y_mat, theta_mat, dist_to_goal, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            return

        # ================= 控制律计算 =================
        xc, yc = x_mat - self.cx, y_mat - self.cy
        dist_c = math.hypot(xc, yc)
        eta = dist_c / self.r
        g_phi_x, g_phi_y = 2 * xc, 2 * yc
        
        if eta <= 1.0:
            kt = 1.0 - (1.0 - eta)**(1.0 + self.gamma)
            kn = math.sqrt( 1.0 - kt**2)
            kd = (kt + eta * (1.0 + self.gamma) * (1.0 - eta)**self.gamma) / (self.r * eta)
        else:
            kt = 1.0 - (1.0 - 1.0 / eta)**(1.0 + self.gamma)
            kn = -math.sqrt( 1.0 - kt**2)
            kd = (kt - ((1.0 + self.gamma) / eta) * (1.0 - 1.0/eta)**self.gamma) / (self.r * eta)

        chi_ang = math.atan2(kt * g_phi_x + kn * g_phi_y, kt * (-g_phi_y) + kn * g_phi_x)
        
        theta_e = math.atan2(math.sin(theta_mat - chi_ang), math.cos(theta_mat - chi_ang))
        sin_te = math.sin(theta_e)
        if abs(sin_te) < self.eps:
            self.sign_term = (self.eps**(self.alpha - 1.0)) * sin_te 
        else:
            self.sign_term = math.copysign(1.0, sin_te) * (abs(sin_te)**self.alpha)
        psi = (x_mat - self.target_x)**2 + (y_mat - self.target_y)**2 + self.mu * (1.0 - math.cos(theta_e))
        kappa_c = kd - (self.kappa_max - abs(kd)) * self.sign_term

        vr = self.v_min + 1.0 / (1.0/(self.v_max-self.v_min) + abs(kappa_c)/ (self.omega_max - self.v_min * abs(kappa_c)))
        v_cmd = self.v_min + (vr - self.v_min) * (1.0 - math.exp(-self.k_v * psi**self.beta))
        
        if self.nav_start_time is not None:
            t_nav = (rospy.Time.now() - self.nav_start_time).to_sec()
            soft_start_factor = 1.0 - math.exp(-self.soft_start_k * t_nav)
            v_cmd = v_cmd * soft_start_factor
        
        w_cmd = math.atan(kappa_c * self.wheelbase)

        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v_cmd, w_cmd
        self.pub_cmd.publish(cmd)
        
        self.publish_markers(x_mat, y_mat, theta_mat)
        self.publish_debug(x_mat, y_mat, theta_mat, dist_to_goal, kt, kn, kd, theta_e, psi, kappa_c, vr, v_cmd, w_cmd)

    def publish_debug(self, x, y, theta, dist_to_goal, kt, kn, kd, theta_e, psi, kappa_c, vr, cmd_v, cmd_w):
        # 0. 任务状态
        self.pub_debug_f64('state', self.state)
        self.pub_debug_f64('current_target', self.current_wp_idx + 1)

        # 1. 位置
        self.pub_debug_f64('x', x)
        self.pub_debug_f64('y', y)
        self.pub_debug_f64('theta_deg', math.degrees(theta))

        # 2. 实际反馈速度
        self.pub_debug_f64('v_fb', self.actual_v)
        self.pub_debug_f64('w_fb', self.actual_w)

        # 3. IMU反馈速度
        self.pub_debug_f64('v_imu', self.v_imu)
        self.pub_debug_f64('w_imu', self.w_imu)

        # 4. 指令速度
        self.pub_debug_f64('v_cmd', cmd_v)
        self.pub_debug_f64('w_cmd', cmd_w)
        self.pub_debug_f64('vr', vr)

        # 5. 误差
        self.pub_debug_f64('distance_to_target', dist_to_goal)
        self.pub_debug_f64('theta_e_rad', theta_e)
        self.pub_debug_f64('theta_e_deg', math.degrees(theta_e))

        # 6. 算法参数
        self.pub_debug_f64('kappa_c', kappa_c)
        self.pub_debug_f64('psi', psi)
        self.pub_debug_f64('kt', kt)
        self.pub_debug_f64('kn', kn)
        self.pub_debug_f64('kd', kd)
        
        # 将所有当前数据作为一行写入 CSV
        if self.csv_writer is not None and self.state != self.STATE_FINISHED:
            row_data = [
                rospy.Time.now().to_sec(),      
                self.state,                     
                self.current_loop_idx + 1,      
                self.current_wp_idx + 1,        
                x, y, math.degrees(theta),
                self.actual_v, self.actual_w,
                self.v_imu, self.w_imu,
                cmd_v, cmd_w, vr,
                dist_to_goal, theta_e, math.degrees(theta_e),
                kappa_c, psi, kt, kn, kd,
                self.last_reached_dist_error,   
                self.last_reached_yaw_error     
            ]
            self.csv_writer.writerow(row_data)

    def publish_markers(self, x_mat, y_mat, theta_mat):
        t = rospy.Time.now()
        def create_marker(ns, id, mtype, color, scale):
            m = Marker(); m.header.frame_id = "odom"; m.header.stamp = t
            m.ns, m.id, m.type, m.action = ns, id, mtype, Marker.ADD
            m.color.r, m.color.g, m.color.b, m.color.a = color
            m.scale.x, m.scale.y, m.scale.z = scale
            return m
        
        viz_cx, viz_cy, _ = self.mat_to_odom(self.cx, self.cy, 0.0)
        viz_x, viz_y, viz_theta = self.mat_to_odom(x_mat, y_mat, theta_mat)

        m1 = create_marker("manifold", 0, Marker.CYLINDER, (0,1,0,0.3), (self.r*2, self.r*2, 0.02))
        m1.pose.position.x, m1.pose.position.y, m1.pose.orientation.w = viz_cx, viz_cy, 1.0
        self.pub_viz.publish(m1)
        
        m2 = create_marker("car", 1, Marker.ARROW, (0,0,1,1), (0.8, 0.15, 0.15))
        m2.pose.position.x, m2.pose.position.y = viz_x, viz_y
        m2.pose.orientation.x, m2.pose.orientation.y, m2.pose.orientation.z, m2.pose.orientation.w = tf.transformations.quaternion_from_euler(0, 0, viz_theta)
        self.pub_viz.publish(m2)

if __name__ == '__main__':
    try: 
        FTC2VFPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass