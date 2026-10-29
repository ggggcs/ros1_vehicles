#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
引导向量场局部路径规划器
替代TEB planner,基于论文的向量场避障方法
"""

import rospy
import numpy as np
import math
import re
import time
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Float32, Header
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from bringup.msg import ControlDebug
import threading
import tf

class EllipseObstacle:
    """椭圆障碍物数据结构"""
    def __init__(self, center_x, center_y, semi_major_axis, semi_minor_axis, orientation_rad):
        """
        Args:
            center_x, center_y: 椭圆中心坐标
            semi_major_axis: 长轴长度（直径）
            semi_minor_axis: 短轴长度（直径）
            orientation_rad: 椭圆旋转角度（弧度）
        """
        self.center_x = center_x
        self.center_y = center_y
        self.a = semi_major_axis / 2.0  # 半长轴
        self.b = semi_minor_axis / 2.0  # 半短轴
        self.theta = orientation_rad  # 直接使用弧度

class VectorFieldPlanner:
    def __init__(self):
        rospy.init_node('vector_field_planner', anonymous=True)
        
        # ========== 参数配置 ==========
        # 向量场增益参数
        self.k_p = rospy.get_param('~k_p', 1.0)  # 路径跟踪增益
        self.k_r = rospy.get_param('~k_r', 1.5)  # 反应向量场增益
        
        # Bump函数参数
        self.l1 = rospy.get_param('~l1', 0.3)
        self.l2 = rospy.get_param('~l2', 0.3)
        # 添加bump函数过渡区域参数
        self.bump_transition_width = rospy.get_param('~bump_transition_width', 10.0)
        
        # 边界偏移量
        self.repulsive_offset = rospy.get_param('~repulsive_offset', 0.6)
        self.reactive_offset = rospy.get_param('~reactive_offset', 1.0)
        
        # 数值稳定性参数
        self.epsilon = 1e-8
        self.max_vector_norm = rospy.get_param('~max_speed', 2.0)
        self.min_vector_norm = 1e-4
        self.E = np.array([[0.0, -1.0], [1.0, 0.0]])
        
        # 控制频率
        self.control_rate = rospy.get_param('~control_rate', 10.0)
        
        # 目标点容差
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.2)
        
        # 阿克曼参数
        self.wheelbase = rospy.get_param('~wheelbase', 0.335)
        self.max_steering_angle = rospy.get_param('~max_steering_angle', 2.0)
        self.steering_gain = rospy.get_param('~steering_gain', 10.0)
        
        # ========== 圆形路径参数 ==========
        # 路径类型: 'circle' (圆形)
        self.path_type = rospy.get_param('~path_type', 'circle')
        
        # 圆形路径参数: x^2 + y^2 = R^2
        self.circle_center_x = rospy.get_param('~circle_center_x', 0.0)
        self.circle_center_y = rospy.get_param('~circle_center_y', 0.0)
        self.circle_radius = rospy.get_param('~circle_radius', 5.0)
        self.circle_direction = rospy.get_param('~circle_direction', 'ccw')  # 'ccw' 或 'cw'
        
        # ========== 状态变量 ==========
        self.current_pose = None
        self.current_velocity = None
        self.obstacles = []  # 存储解析后的椭圆障碍物
        
        # TF监听器用于坐标转换
        self.tf_listener = tf.TransformListener()
        
        # 线程锁
        self.obstacle_lock = threading.Lock()
        self.pose_lock = threading.Lock()
        
        # ========== 障碍物管理模块 ==========
        # 数据关联距离阈值 (全局坐标系)
        self.obstacle_association_threshold = rospy.get_param('~obstacle_association_threshold', 1.0)
        
        # 全局障碍物地图 - 维护所有历史障碍物
        self.global_obstacle_map = []  # 列表，每个元素是一个EllipseObstacle对象
        
        rospy.loginfo("障碍物管理模块初始化: 数据关联阈值=%.2fm", self.obstacle_association_threshold)
        
        # ========== ROS话题订阅 ==========
        rospy.Subscriber('/lio_sam/mapping/odometry', Odometry, self.odom_callback)
        rospy.Subscriber('/odometry/imu', Odometry, self.imu_odom_callback)  # 用于获取速度
        # 订阅obs_fitting的椭圆方程话题
        rospy.Subscriber('/obs_fitting/ellipse_equations', String, self.obstacles_callback)
        
        # ========== ROS话题发布 ==========
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.local_plan_pub = rospy.Publisher('/vector_field_planner/local_plan', Path, queue_size=1)
        
        # 向量场可视化发布器 (使用MarkerArray以显示多个箭头)
        self.vector_field_pub = rospy.Publisher('/vector_field_planner/vector_field', MarkerArray, queue_size=1)
        self.path_field_pub = rospy.Publisher('/vector_field_planner/path_field', MarkerArray, queue_size=1)
        self.reactive_field_pub = rospy.Publisher('/vector_field_planner/reactive_field', MarkerArray, queue_size=1)
        
        # 障碍物边界可视化发布器
        self.obstacle_boundaries_pub = rospy.Publisher('/vector_field_planner/obstacle_boundaries', MarkerArray, queue_size=1)
        self.yaw_error_pub = rospy.Publisher('/vector_field_planner/yaw_error', Float32, queue_size=1)
        
        # 当前向量场可视化发布器
        self.current_vector_pub = rospy.Publisher('/vector_field_planner/current_vector', Marker, queue_size=1)
        
        # 圆形路径可视化发布器
        self.circle_path_pub = rospy.Publisher('/vector_field_planner/circle_path', Marker, queue_size=1, latch=True)
        
        # 性能统计发布器
        self.field_compute_time_pub = rospy.Publisher('/vector_field_planner/field_compute_time', Float32, queue_size=1)
        self.control_query_time_pub = rospy.Publisher('/vector_field_planner/control_query_time', Float32, queue_size=1)
        self.viz_time_pub = rospy.Publisher('/vector_field_planner/viz_time', Float32, queue_size=1)
        
        # 控制调试信息发布器
        self.control_debug_pub = rospy.Publisher('/vector_field_planner/control_debug', ControlDebug, queue_size=1)
        
        # 向量场计算和可视化参数
        self.field_center_x = rospy.get_param('~field_center_x', 0.0)  # 向量场固定中心X (map坐标系)
        self.field_center_y = rospy.get_param('~field_center_y', 0.0)  # 向量场固定中心Y (map坐标系)
        self.field_range = rospy.get_param('~field_range', 10.0)  # 向量场计算范围 (米)
        self.field_resolution = rospy.get_param('~field_resolution', 0.3)  # 向量场网格分辨率 (米)
        self.viz_frequency = rospy.get_param('~viz_frequency', 10.0)  # 可视化更新频率 (Hz)
        self.last_viz_time = rospy.Time(0)
        
        # 局部路径生成参数
        self.local_path_length = rospy.get_param('~local_path_length', 3.0)  # 局部路径长度 (米)
        self.local_path_step = rospy.get_param('~local_path_step', 0.1)  # 路径积分步长 (米)
        
        # 缓存的向量场数据
        self.cached_vector_field = {}  # 字典: (grid_x, grid_y) -> vector
        self.last_field_update_time = rospy.Time(0)
        self.field_update_interval = 1.0 / self.viz_frequency  # 向量场更新间隔
        
        rospy.loginfo("向量场规划器初始化完成")
        rospy.loginfo("参数: k_p=%.2f, k_r=%.2f, max_speed=%.2f", self.k_p, self.k_r, self.max_vector_norm)
        rospy.loginfo("路径类型: %s", self.path_type)
        if self.path_type == 'circle':
            rospy.loginfo("圆形路径: 圆心=(%.2f, %.2f), 半径=%.2f, 方向=%s", 
                         self.circle_center_x, self.circle_center_y, 
                         self.circle_radius, self.circle_direction)
        rospy.loginfo("向量场: 固定中心=(%.1f, %.1f), 范围=%.1fm, 分辨率=%.2fm, 频率=%.1fHz", 
                     self.field_center_x, self.field_center_y, self.field_range, self.field_resolution, self.viz_frequency)
        
        # 发布圆形路径可视化
        if self.path_type == 'circle':
            self.visualize_circle_path()
    
    # ========== 回调函数 ==========
    def odom_callback(self, msg):
        """接收里程计信息（位姿）"""
        with self.pose_lock:
            self.current_pose = msg.pose.pose
    
    def imu_odom_callback(self, msg):
        """接收IMU里程计信息（速度）"""
        with self.pose_lock:
            self.current_velocity = msg.twist.twist
    
    # ========== 障碍物管理模块函数 ==========
    def associate_obstacle(self, new_obstacle):
        """数据关联：在全局地图中查找与新障碍物匹配的历史障碍物
        
        Args:
            new_obstacle: 新检测到的椭圆障碍物 (EllipseObstacle)
            
        Returns:
            匹配的历史障碍物索引，如果没有匹配则返回None
        """
        for i, old_obstacle in enumerate(self.global_obstacle_map):
            # 计算两个椭圆中心之间的距离（全局坐标系）
            dx = new_obstacle.center_x - old_obstacle.center_x
            dy = new_obstacle.center_y - old_obstacle.center_y
            distance = np.sqrt(dx**2 + dy**2)
            
            # 如果距离小于阈值，认为是同一个障碍物
            if distance < self.obstacle_association_threshold:
                return i
        
        return None
    
    def update_global_obstacle_map(self, new_obstacles):
        """更新全局障碍物地图
        
        核心处理规则：
        1. 数据关联与更新：当新椭圆E_new被检测到时，在全局地图中搜索历史椭圆集合。
           如果找到E_old与E_new的中心距离小于D_threshold，则用E_new替换E_old。
        2. 盲区数据处理：对于历史椭圆E_old，如果没有任何新椭圆与其距离小于D_threshold，
           则保留E_old在其最后已知位置上。
        
        Args:
            new_obstacles: 当前周期检测到的椭圆障碍物列表
            
        Returns:
            更新后的全局障碍物地图
        """
        # 标记全局地图中哪些障碍物被更新了
        updated_indices = set()
        
        # 处理所有新检测到的障碍物
        for new_obstacle in new_obstacles:
            # 尝试在全局地图中找到匹配的历史障碍物
            matched_index = self.associate_obstacle(new_obstacle)
            
            if matched_index is not None:
                # 规则1：找到匹配的历史障碍物，用新数据替换
                self.global_obstacle_map[matched_index] = new_obstacle
                updated_indices.add(matched_index)
                rospy.logdebug("更新障碍物[%d]: 位置(%.2f, %.2f)", 
                             matched_index, new_obstacle.center_x, new_obstacle.center_y)
            else:
                # 新障碍物，添加到全局地图
                self.global_obstacle_map.append(new_obstacle)
                rospy.loginfo("新增障碍物: 位置(%.2f, %.2f)", 
                            new_obstacle.center_x, new_obstacle.center_y)
        
        # 规则2：保留所有未被更新的历史障碍物（处理盲区情况）
        # 这些障碍物将继续存在于地图中，维持其最后已知位置
        preserved_count = len(self.global_obstacle_map) - len(updated_indices) - len(new_obstacles) + len([i for i in updated_indices])
        if preserved_count > 0:
            rospy.logdebug("保留%d个历史障碍物（可能在盲区中）", preserved_count)
        
        return self.global_obstacle_map
    
    def obstacles_callback(self, msg):
        """接收obs_fitting发布的椭圆方程信息并解析，并通过障碍物管理模块处理"""
        new_obstacles = []  # 当前周期检测到的新障碍物
        
        try:
            # 解析椭圆方程字符串
            # 格式示例: "椭圆 ID:0 中心:(1.234,-2.567) 大小:2.0x1.5 角度:45.0° 面积:2.356"
            lines = msg.data.split('\n')
            
            for line in lines:
                if '中心:' in line and '大小:' in line and '角度:' in line:
                    try:
                        # 提取中心坐标 (velodyne坐标系)
                        center_match = re.search(r'中心:\(([^,]+),([^)]+)\)', line)
                        if not center_match:
                            continue
                        center_x_velodyne = float(center_match.group(1))
                        center_y_velodyne = float(center_match.group(2))
                        
                        # 提取大小 (width x height)
                        size_match = re.search(r'大小:([^x]+)x([^\s]+)', line)
                        if not size_match:
                            continue
                        width = float(size_match.group(1))
                        height = float(size_match.group(2))
                        
                        # 提取角度（velodyne坐标系中的角度，单位：度）
                        angle_match = re.search(r'角度:([^°]+)°', line)
                        if not angle_match:
                            continue
                        angle_velodyne_deg = float(angle_match.group(1))
                        angle_velodyne_rad = np.deg2rad(angle_velodyne_deg)
                        
                        # 坐标转换: velodyne -> map（包括位置和旋转）
                        try:
                            # 1. 转换中心点位置
                            point_velodyne = PointStamped()
                            point_velodyne.header.frame_id = "velodyne"
                            point_velodyne.header.stamp = rospy.Time(0)  # 使用最新的TF
                            point_velodyne.point.x = center_x_velodyne
                            point_velodyne.point.y = center_y_velodyne
                            point_velodyne.point.z = 0.0
                            
                            point_map = self.tf_listener.transformPoint("map", point_velodyne)
                            center_x = point_map.point.x
                            center_y = point_map.point.y
                            
                            # 2. 获取velodyne到map的旋转变换
                            # 查找变换并提取旋转角度
                            (trans, rot) = self.tf_listener.lookupTransform("map", "velodyne", rospy.Time(0))
                            
                            # 从四元数提取yaw角度
                            _, _, yaw_map_to_velodyne = euler_from_quaternion(rot)
                            
                            # 椭圆在map坐标系中的角度 = velodyne系中的角度 + 坐标系旋转角度
                            angle_map_rad = angle_velodyne_rad + yaw_map_to_velodyne
                            
                            # 归一化到 [-pi, pi]
                            angle_map_rad = np.arctan2(np.sin(angle_map_rad), np.cos(angle_map_rad))
                            
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                            rospy.logwarn_throttle(5.0, "TF转换失败 (velodyne -> map): %s", str(e))
                            continue
                        
                        # 创建椭圆障碍物对象 (map坐标系，角度单位：弧度)
                        obstacle = EllipseObstacle(center_x, center_y, width, height, angle_map_rad)
                        new_obstacles.append(obstacle)
                        
                    except Exception as e:
                        rospy.logwarn("解析椭圆数据失败: %s, line: %s", str(e), line)
                        continue
            
            # 使用障碍物管理模块更新全局地图
            with self.obstacle_lock:
                self.global_obstacle_map = self.update_global_obstacle_map(new_obstacles)
                
                # 将全局地图中的所有障碍物作为当前障碍物列表
                # 这样既包含新检测到的障碍物，也包含历史保留的障碍物
                self.obstacles = list(self.global_obstacle_map)
                
                rospy.loginfo_throttle(2.0, "障碍物管理: 当前检测=%d, 全局地图=%d", 
                                     len(new_obstacles), len(self.obstacles))
                
        except Exception as e:
            rospy.logerr("处理障碍物消息失败: %s", str(e))
    
    # ========== 路径隐式函数定义 ==========
    def circle_path_function(self, x, y):
        """圆形路径的隐式函数及梯度
        
        圆形路径: (x-cx)^2 + (y-cy)^2 = R^2
        隐式函数: phi(x,y) = (x-cx)^2 + (y-cy)^2 - R^2
        
        Args:
            x, y: 当前位置
            
        Returns:
            phi_val: 路径函数值 (=0在路径上, >0在圆外, <0在圆内)
            grad_phi: 梯度 [grad_x, grad_y]
        """
        # 相对于圆心的坐标
        dx = x - self.circle_center_x
        dy = y - self.circle_center_y
        
        # 隐式函数值
        phi_val = dx**2 + dy**2 - self.circle_radius**2
        
        # 梯度: ∇phi = [2*dx, 2*dy]
        grad_phi = np.array([2*dx, 2*dy])
        
        return phi_val, grad_phi
    
    # ========== 向量场计算函数 ==========
    def safe_U_Q(self, phi_val):
        # """安全的排斥bump函数 U_Q
        # 在排斥边界内(φ<0)趋近0，在排斥边界外(φ>0)趋近1
        # """
        if phi_val >= 0:  # 排斥边界外
            return 1.0
        elif phi_val < -self.bump_transition_width:  # 远离排斥边界
            return 0.0
        
        # 平滑过渡区域
        denominator = max(-phi_val, self.epsilon)  # 正值
        f1 = np.exp(self.l1 / denominator)
        f2 = np.exp(self.l2 / (phi_val + self.bump_transition_width))  # 避免除零，使用可配置参数
        
        val = f1 / max(f1 + f2, self.epsilon)
        return np.clip(val, 0.0, 1.0)

    def safe_T_R(self, phi_val):
        # """安全的反应bump函数 T_R  
        # 在反应边界内(φ<0)趋近1，在反应边界外(φ>0)趋近0
        # """
        if phi_val <= 0:  # 反应边界内
            return 1.0
        elif phi_val > self.bump_transition_width:  # 远离反应边界
            return 0.0
        
        # 平滑过渡区域
        denominator = max(phi_val, self.epsilon)  # 正值
        f1 = np.exp(self.l1 / (-phi_val + self.bump_transition_width))  # 使用可配置参数确保分母为正
        f2 = np.exp(self.l2 / denominator)
        
        val = f2 / max(f1 + f2, self.epsilon)
        return np.clip(val, 0.0, 1.0)

    def _default_min_vector(self):
        """提供一个最小安全向量，避免返回全零导致的数值问题"""
        return np.array([self.min_vector_norm, 0.0])

    def _normalize_with_limits(self, vec, zero_default=False):
        """对向量执行归一化和幅值限制，必要时返回默认值"""
        default = np.zeros(2) if zero_default else self._default_min_vector()

        if vec is None:
            return default

        vec = np.asarray(vec, dtype=float)
        if vec.shape != (2,) or np.any(~np.isfinite(vec)):
            return default

        norm = np.linalg.norm(vec)
        if norm < self.epsilon:
            return default

        if norm > self.max_vector_norm:
            return vec * (self.max_vector_norm / norm)

        return vec / norm
    
    def ellipse_function_and_gradient(self, x, y, ellipse, offset=0.0):
        """计算椭圆函数值和梯度
        
        Args:
            x, y: 当前位置
            ellipse: EllipseObstacle对象
            offset: 边界偏移量（米），用于计算扩大的椭圆
        
        Returns:
            phi_val: 椭圆函数值
            grad: 梯度 [grad_x, grad_y]
        """
        cx = ellipse.center_x
        cy = ellipse.center_y
        # 应用偏移量到半轴
        a = ellipse.a + offset
        b = ellipse.b + offset
        theta = ellipse.theta
        
        # 旋转变换
        x_rot = (x - cx) * np.cos(theta) + (y - cy) * np.sin(theta)
        y_rot = -(x - cx) * np.sin(theta) + (y - cy) * np.cos(theta)
        
        # 椭圆函数值
        phi_val = (x_rot / a)**2 + (y_rot / b)**2 - 1
        
        # 梯度计算
        dval_dx_rot = 2 * x_rot / (a**2)
        dval_dy_rot = 2 * y_rot / (b**2)
        
        # 旋转回原坐标系
        grad_x = dval_dx_rot * np.cos(theta) - dval_dy_rot * np.sin(theta)
        grad_y = dval_dx_rot * np.sin(theta) + dval_dy_rot * np.cos(theta)
        
        return phi_val, np.array([grad_x, grad_y])
    

    
    def path_tracking_vector_field(self, x, y):
        """路径跟踪向量场 χ_P (基于隐式函数)
        
        基于论文和MATLAB代码的实现:
        χ_P = direction_sign * E * ∇φ - k_p * φ * ∇φ
        
        其中:
        - E = [0, -1; 1, 0] 是旋转矩阵,将梯度旋转90度得到切向
        - φ(x,y) 是路径的隐式函数 (圆形: x^2 + y^2 - R^2)
        - ∇φ 是路径函数的梯度 (圆形: [2x, 2y])
        - direction_sign 确定运动方向 (顺时针或逆时针)
        - k_p 是路径跟踪增益,控制收敛速度
        """
        if self.path_type == 'circle':
            # 使用圆形路径的隐式函数
            phi, grad_phi = self.circle_path_function(x, y)
        else:
            rospy.logwarn_throttle(5.0, "不支持的路径类型: %s", self.path_type)
            return self._default_min_vector()
        
        # 检查梯度有效性
        grad_norm = np.linalg.norm(grad_phi)
        if grad_norm < self.epsilon:
            return self._default_min_vector()
        
        # 确定运动方向
        # 'ccw' (逆时针) -> direction_sign = 1
        # 'cw' (顺时针) -> direction_sign = -1
        direction_sign = 1.0 if self.circle_direction == 'ccw' else -1.0
        
        # 路径跟踪向量场: χ_P = direction_sign * E * ∇φ - k_p * φ * ∇φ
        # 第一项: 切向分量,驱动沿路径前进
        # 第二项: 法向分量,驱动收敛到路径
        chi_P = direction_sign * (self.E @ grad_phi) - self.k_p * phi * grad_phi
        
        if np.any(~np.isfinite(chi_P)):
            return self._default_min_vector()
        
        return chi_P
    
    def reactive_vector_field(self, x, y):
    # """障碍物反应向量场 χ_R - 智能转向版本
    
    # 基于论文公式(4)改进: χ_R = K * E * ∇φ - k_r * φ * ∇φ
    # 其中K决定绕行方向（+1或-1），选择转角最小的方向
    # """
        with self.obstacle_lock:
            obstacles = list(self.obstacles)
        
        if len(obstacles) == 0:
            return np.zeros(2)
        
        # 找到最近的障碍物
        min_distance = float('inf')
        nearest_obstacle = None
        
        for obstacle in obstacles:
            phi_reactive, _ = self.ellipse_function_and_gradient(x, y, obstacle, self.reactive_offset)
            if not np.isfinite(phi_reactive):
                continue
            if phi_reactive < min_distance:
                min_distance = phi_reactive
                nearest_obstacle = obstacle
        
        if nearest_obstacle is None:
            return np.zeros(2)
        
        # 计算反应向量场
        phi_reactive, grad_phi_reactive = self.ellipse_function_and_gradient(x, y, nearest_obstacle, self.reactive_offset)
        
        if grad_phi_reactive is None or np.any(~np.isfinite(grad_phi_reactive)):
            return np.zeros(2)
            
        grad_norm = np.linalg.norm(grad_phi_reactive)
        if grad_norm < self.epsilon:
            return np.zeros(2)
        
        # ========== 智能转向方向选择 ==========
        # 计算路径跟踪向量（期望前进方向）
        chi_P = self.path_tracking_vector_field(x, y)
        chi_P_norm = np.linalg.norm(chi_P)
        
        # 默认K=1（逆时针绕行）
        K = 1.0
        
        if chi_P_norm > self.epsilon:
            # 归一化路径跟踪向量
            chi_P_normalized = chi_P / chi_P_norm
            
            # 计算两种绕行方向的切向量
            tangent_ccw = self.E @ grad_phi_reactive  # 逆时针（K=+1）
            tangent_cw = -self.E @ grad_phi_reactive  # 顺时针（K=-1）
            
            # 归一化切向量
            tangent_ccw_norm = np.linalg.norm(tangent_ccw)
            tangent_cw_norm = np.linalg.norm(tangent_cw)
            
            if tangent_ccw_norm > self.epsilon and tangent_cw_norm > self.epsilon:
                tangent_ccw_normalized = tangent_ccw / tangent_ccw_norm
                tangent_cw_normalized = tangent_cw / tangent_cw_norm
                
                # 计算两种方向与路径跟踪方向的点积（余弦相似度）
                # 点积越大，方向越接近，转角越小
                dot_ccw = np.dot(tangent_ccw_normalized, chi_P_normalized)
                dot_cw = np.dot(tangent_cw_normalized, chi_P_normalized)
                
                # 选择点积更大的方向（转角更小）
                if dot_cw > dot_ccw:
                    K = -1.0  # 顺时针绕行转角更小
                    rospy.logdebug_throttle(1.0, "智能转向：选择顺时针绕行 (K=-1)")
                else:
                    K = 1.0   # 逆时针绕行转角更小
                    rospy.logdebug_throttle(1.0, "智能转向：选择逆时针绕行 (K=+1)")
        
        # χ_R = K * E * ∇φ - k_r * φ * ∇φ
        chi_R = K * (self.E @ grad_phi_reactive) - self.k_r * phi_reactive * grad_phi_reactive
        
        if np.any(~np.isfinite(chi_R)):
            return np.zeros(2)
        
        return chi_R
    
    def composite_vector_field(self, x, y):
    # """复合向量场 χ_c - 按照论文正确定义
    
    # 三个区域：
    # 1. 排斥区域内 (φ_repulsive < 0): χ_c = χ̂_R
    # 2. 混合区域 (φ_repulsive > 0 且 φ_reactive < 0): χ_c = S·χ̂_P + Z·χ̂_R  
    # 3. 非反应区域 (φ_reactive > 0): χ_c = χ̂_P
    # """
        # 单点计算计时（仅用于调试，正常情况不输出）
        point_start = time.time()
        
        if not np.isfinite(x) or not np.isfinite(y):
            return self._default_min_vector()

        # 预计算路径跟踪向量场（归一化）
        chi_P = self.path_tracking_vector_field(x, y)
        chi_P_hat = self._normalize_with_limits(chi_P)

        with self.obstacle_lock:
            obstacles = list(self.obstacles)

        # 如果没有障碍物，直接返回路径跟踪向量场
        if len(obstacles) == 0:
            return chi_P_hat

        # 检查是否在任意障碍物的排斥区域内
        in_repulsive_zone = False
        nearest_obstacle = None
        min_phi_repulsive = float('inf')
        
        for obstacle in obstacles:
            phi_repulsive, grad_phi_repulsive = self.ellipse_function_and_gradient(x, y, obstacle, self.repulsive_offset)
            
            if not np.isfinite(phi_repulsive):
                continue
                
            if phi_repulsive < min_phi_repulsive:
                min_phi_repulsive = phi_repulsive
                nearest_obstacle = obstacle
                
            # 如果在任意障碍物的排斥区域内
            if phi_repulsive < 0:
                in_repulsive_zone = True
                break
        
        # 1. 排斥区域内：只使用反应向量场（带智能转向）
        if in_repulsive_zone and nearest_obstacle is not None:
            phi_reactive, grad_phi_reactive = self.ellipse_function_and_gradient(x, y, nearest_obstacle, self.reactive_offset)
            
            if grad_phi_reactive is not None and np.all(np.isfinite(grad_phi_reactive)):
                grad_norm = np.linalg.norm(grad_phi_reactive)
                if grad_norm >= self.epsilon:
                    # 智能选择绕行方向
                    chi_P_norm = np.linalg.norm(chi_P)
                    K = 1.0
                    if chi_P_norm > self.epsilon:
                        chi_P_normalized = chi_P / chi_P_norm
                        tangent_ccw = self.E @ grad_phi_reactive
                        tangent_cw = -self.E @ grad_phi_reactive
                        
                        tangent_ccw_norm = np.linalg.norm(tangent_ccw)
                        tangent_cw_norm = np.linalg.norm(tangent_cw)
                        
                        if tangent_ccw_norm > self.epsilon and tangent_cw_norm > self.epsilon:
                            dot_ccw = np.dot(tangent_ccw / tangent_ccw_norm, chi_P_normalized)
                            dot_cw = np.dot(tangent_cw / tangent_cw_norm, chi_P_normalized)
                            K = -1.0 if dot_cw > dot_ccw else 1.0
                    
                    # χ_R = K * E * ∇φ_reactive - k_r * φ_reactive * ∇φ_reactive
                    chi_R = K * (self.E @ grad_phi_reactive) - self.k_r * phi_reactive * grad_phi_reactive
                    chi_R_hat = self._normalize_with_limits(chi_R, zero_default=True)
                    return chi_R_hat
        
        # 2. 混合区域和非反应区域处理
        U_Q_product = 1.0
        T_R_sum = np.zeros(2)
        
        for obstacle in obstacles:
            phi_repulsive, _ = self.ellipse_function_and_gradient(x, y, obstacle, self.repulsive_offset)
            phi_reactive, grad_phi_reactive = self.ellipse_function_and_gradient(x, y, obstacle, self.reactive_offset)
            
            if not np.isfinite(phi_repulsive) or not np.isfinite(phi_reactive):
                continue

            # 计算bump函数
            U_Q_val = self.safe_U_Q(phi_repulsive)  # 排斥边界bump函数
            T_R_val = self.safe_T_R(phi_reactive)   # 反应边界bump函数

            # 计算反应向量场（带智能转向）
            chi_R = np.zeros(2)
            if grad_phi_reactive is not None and np.all(np.isfinite(grad_phi_reactive)):
                grad_norm = np.linalg.norm(grad_phi_reactive)
                if grad_norm >= self.epsilon:
                    # 智能选择绕行方向
                    chi_P_norm = np.linalg.norm(chi_P)
                    K = 1.0
                    if chi_P_norm > self.epsilon:
                        chi_P_normalized = chi_P / chi_P_norm
                        tangent_ccw = self.E @ grad_phi_reactive
                        tangent_cw = -self.E @ grad_phi_reactive
                        
                        tangent_ccw_norm = np.linalg.norm(tangent_ccw)
                        tangent_cw_norm = np.linalg.norm(tangent_cw)
                        
                        if tangent_ccw_norm > self.epsilon and tangent_cw_norm > self.epsilon:
                            dot_ccw = np.dot(tangent_ccw / tangent_ccw_norm, chi_P_normalized)
                            dot_cw = np.dot(tangent_cw / tangent_cw_norm, chi_P_normalized)
                            K = -1.0 if dot_cw > dot_ccw else 1.0
                    
                    chi_R = K * (self.E @ grad_phi_reactive) - self.k_r * phi_reactive * grad_phi_reactive
            
            chi_R_hat = self._normalize_with_limits(chi_R, zero_default=True)
            
            # 累积
            U_Q_product *= U_Q_val
            T_R_sum += T_R_val * chi_R_hat

        # 确保数值稳定性
        if not np.isfinite(U_Q_product):
            U_Q_product = 1.0

        # 复合向量场：χ_c = (∏U_Q)χ̂_P + ∑(T_R·χ̂_R)
        chi_c = U_Q_product * chi_P_hat + T_R_sum
        
        # 检查结果有效性并限制幅值
        if np.any(~np.isfinite(chi_c)):
            return self._default_min_vector()

        chi_norm = np.linalg.norm(chi_c)
        if chi_norm > self.max_vector_norm:
            chi_c = chi_c * (self.max_vector_norm / chi_norm)
        elif chi_norm < self.min_vector_norm:
            chi_c = self._default_min_vector()

        # 记录单点计算时间（调试用）
        point_elapsed = (time.time() - point_start) * 1000
        if point_elapsed > 1.0:  # 只记录超过1ms的计算
            rospy.logdebug("单点向量场计算耗时: %.2fms at (%.2f, %.2f)", point_elapsed, x, y)

        return chi_c
    
    def vector_to_ackermann(self, chi_c, current_yaw):
        """将向量场转换为Ackermann控制命令
        
        Args:
            chi_c: 复合向量场
            current_yaw: 当前朝向角
        
        Returns:
            Twist消息
        """
        # 检查向量场是否有效
        chi_norm = np.linalg.norm(chi_c)
        if np.isnan(chi_norm) or chi_norm < 1e-6:  # 放宽阈值，只要不是NaN且不是完全为0
            rospy.logwarn_throttle(2.0, "向量场无效或太小: norm=%.6f", chi_norm)
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd
        
        # 归一化向量场，只保留方向信息
        chi_c_normalized = chi_c / chi_norm
        
        # 期望方向
        desired_yaw = np.arctan2(chi_c_normalized[1], chi_c_normalized[0])
        
        # 角度差（归一化到[-pi, pi]）
        yaw_error = desired_yaw - current_yaw
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))
        if self.yaw_error_pub.get_num_connections() > 0:
            self.yaw_error_pub.publish(Float32(data=yaw_error))
        
        # 线速度使用配置的max_speed参数（不受向量大小影响）
        linear_vel = self.max_vector_norm
        
        # 计算Ackermann舵机角度
        # 使用比例控制：steering_angle = k * yaw_error
        steering_angle = self.steering_gain * yaw_error
        
        # 限制舵机角度
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        # 发布控制调试信息
        if self.control_debug_pub.get_num_connections() > 0:
            debug_msg = ControlDebug()
            debug_msg.header = Header()
            debug_msg.header.stamp = rospy.Time.now()
            debug_msg.header.frame_id = "map"
            
            # 向量场信息
            debug_msg.vector_x = float(chi_c[0])
            debug_msg.vector_y = float(chi_c[1])
            debug_msg.vector_norm = float(chi_norm)
            
            # 角度信息（弧度）
            debug_msg.current_yaw = float(current_yaw)
            debug_msg.desired_yaw = float(desired_yaw)
            debug_msg.yaw_error = float(yaw_error)
            
            # 控制输出
            debug_msg.steering_angle = float(steering_angle)
            debug_msg.linear_velocity = float(linear_vel)
            
            # 角度信息（度）
            debug_msg.current_yaw_deg = float(np.degrees(current_yaw))
            debug_msg.desired_yaw_deg = float(np.degrees(desired_yaw))
            debug_msg.yaw_error_deg = float(np.degrees(yaw_error))
            debug_msg.steering_angle_deg = float(np.degrees(steering_angle))
            
            # 额外调试信息
            if self.current_velocity is not None:
                vx = self.current_velocity.linear.x
                vy = self.current_velocity.linear.y
                debug_msg.speed_magnitude = float(np.sqrt(vx**2 + vy**2))
                debug_msg.speed_direction = float(np.arctan2(vy, vx))
            else:
                debug_msg.speed_magnitude = 0.0
                debug_msg.speed_direction = 0.0
            
            self.control_debug_pub.publish(debug_msg)
        
        # 详细调试输出
        rospy.loginfo_throttle(0.5, 
            "向量=[%.3f,%.3f], 向量场方向=%.1f°, 当前朝向=%.1f°, 角度误差=%.1f°, 舵机角度=%.1f°",
            chi_c[0], chi_c[1], np.degrees(desired_yaw), np.degrees(current_yaw), 
            np.degrees(yaw_error), np.degrees(steering_angle))
        
        # 创建Twist消息
        # 注意: nav_sim.py期望angular.z是舵机角度（不是角速度）
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = steering_angle  # 发送舵机角度
        
        return cmd
    
    def get_current_yaw(self):
        """获取当前朝向角 - 使用车体位姿朝向"""
        with self.pose_lock:
            # 使用车体位姿朝向（从LIO-SAM里程计获取）
            if self.current_pose is None:
                return 0.0
            
            orientation = self.current_pose.orientation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, pose_yaw = euler_from_quaternion(quaternion)
            
            # 调试信息：同时显示速度方向用于对比
            if self.current_velocity is not None:
                vx = self.current_velocity.linear.x
                vy = self.current_velocity.linear.y
                v_speed = np.sqrt(vx**2 + vy**2)
                
                if v_speed > 0.05:  # 速度大于0.05m/s
                    v_yaw = np.arctan2(vy, vx)
                    rospy.logdebug_throttle(1.0, "🔍 车体朝向=%.1f°, 速度方向=%.1f°, 速度大小=%.2fm/s", 
                                          np.degrees(pose_yaw), np.degrees(v_yaw), v_speed)
            
            return pose_yaw
    
    def check_on_path(self):
        """检查是否在路径上"""
        if self.current_pose is None:
            return False
        
        with self.pose_lock:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
        
        if self.path_type == 'circle':
            phi, _ = self.circle_path_function(x, y)
            # 路径误差小于阈值认为在路径上
            return abs(phi) < 0.5
        
        return False
    
    def visualize_circle_path(self):
        """可视化圆形路径"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "circle_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # 生成圆形路径点
        num_points = 100
        angles = np.linspace(0, 2*np.pi, num_points)
        
        from geometry_msgs.msg import Point
        for angle in angles:
            point = Point()
            point.x = self.circle_center_x + self.circle_radius * np.cos(angle)
            point.y = self.circle_center_y + self.circle_radius * np.sin(angle)
            point.z = 0.0
            marker.points.append(point)
        
        # 闭合圆形
        if len(marker.points) > 0:
            marker.points.append(marker.points[0])
        
        # 线条样式
        marker.scale.x = 0.1  # 线宽
        
        # 颜色：红色
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # 完全不透明
        
        marker.pose.orientation.w = 1.0
        
        # 发布
        self.circle_path_pub.publish(marker)
        rospy.loginfo("圆形路径可视化已发布到 /vector_field_planner/circle_path")
    
    def visualize_obstacle_boundaries(self):
        """可视化障碍物的排斥边界和反应边界（椭圆形状）"""
        marker_array = MarkerArray()
        
        with self.obstacle_lock:
            if len(self.obstacles) == 0:
                # 发布空的MarkerArray清除旧的标记
                self.obstacle_boundaries_pub.publish(marker_array)
                return
            
            marker_id = 0
            
            for obstacle in self.obstacles:
                # 原始椭圆（黄色，实线）- 障碍物本体
                marker_original = Marker()
                marker_original.header.frame_id = "map"
                marker_original.header.stamp = rospy.Time.now()
                marker_original.ns = "obstacle_original"
                marker_original.id = marker_id
                marker_original.type = Marker.CYLINDER  # 用圆柱体近似椭圆
                marker_original.action = Marker.ADD
                
                marker_original.pose.position.x = obstacle.center_x
                marker_original.pose.position.y = obstacle.center_y
                marker_original.pose.position.z = 0.05
                
                # 设置朝向
                quat = quaternion_from_euler(0, 0, obstacle.theta)
                marker_original.pose.orientation.x = quat[0]
                marker_original.pose.orientation.y = quat[1]
                marker_original.pose.orientation.z = quat[2]
                marker_original.pose.orientation.w = quat[3]
                
                # 椭圆大小
                marker_original.scale.x = obstacle.a * 2  # 长轴直径
                marker_original.scale.y = obstacle.b * 2  # 短轴直径
                marker_original.scale.z = 0.1  # 高度
                
                marker_original.color.r = 1.0
                marker_original.color.g = 1.0
                marker_original.color.b = 0.0
                marker_original.color.a = 0.3
                
                marker_array.markers.append(marker_original)
                marker_id += 1
                
                # 排斥边界（红色，半透明）
                repulsive_radius = obstacle.a + self.repulsive_offset
                repulsive_radius_b = obstacle.b + self.repulsive_offset
                
                marker_repulsive = Marker()
                marker_repulsive.header.frame_id = "map"
                marker_repulsive.header.stamp = rospy.Time.now()
                marker_repulsive.ns = "repulsive_boundary"
                marker_repulsive.id = marker_id
                marker_repulsive.type = Marker.CYLINDER
                marker_repulsive.action = Marker.ADD
                
                marker_repulsive.pose.position.x = obstacle.center_x
                marker_repulsive.pose.position.y = obstacle.center_y
                marker_repulsive.pose.position.z = 0.03
                
                marker_repulsive.pose.orientation.x = quat[0]
                marker_repulsive.pose.orientation.y = quat[1]
                marker_repulsive.pose.orientation.z = quat[2]
                marker_repulsive.pose.orientation.w = quat[3]
                
                marker_repulsive.scale.x = repulsive_radius * 2
                marker_repulsive.scale.y = repulsive_radius_b * 2
                marker_repulsive.scale.z = 0.05
                
                marker_repulsive.color.r = 1.0
                marker_repulsive.color.g = 0.0
                marker_repulsive.color.b = 0.0
                marker_repulsive.color.a = 0.2
                
                marker_array.markers.append(marker_repulsive)
                marker_id += 1
                
                # 反应边界（橙色，半透明）
                reactive_radius = obstacle.a + self.reactive_offset
                reactive_radius_b = obstacle.b + self.reactive_offset
                
                marker_reactive = Marker()
                marker_reactive.header.frame_id = "map"
                marker_reactive.header.stamp = rospy.Time.now()
                marker_reactive.ns = "reactive_boundary"
                marker_reactive.id = marker_id
                marker_reactive.type = Marker.CYLINDER
                marker_reactive.action = Marker.ADD
                
                marker_reactive.pose.position.x = obstacle.center_x
                marker_reactive.pose.position.y = obstacle.center_y
                marker_reactive.pose.position.z = 0.01
                
                marker_reactive.pose.orientation.x = quat[0]
                marker_reactive.pose.orientation.y = quat[1]
                marker_reactive.pose.orientation.z = quat[2]
                marker_reactive.pose.orientation.w = quat[3]
                
                marker_reactive.scale.x = reactive_radius * 2
                marker_reactive.scale.y = reactive_radius_b * 2
                marker_reactive.scale.z = 0.02
                
                marker_reactive.color.r = 1.0
                marker_reactive.color.g = 0.5
                marker_reactive.color.b = 0.0
                marker_reactive.color.a = 0.15
                
                marker_array.markers.append(marker_reactive)
                marker_id += 1
        
        # 发布可视化
        self.obstacle_boundaries_pub.publish(marker_array)
    
    def visualize_current_vector(self, x, y, chi_c, current_yaw):
        """可视化当前位置的向量场向量（大箭头）
        
        Args:
            x, y: 当前车辆位置
            chi_c: 当前的复合向量场
            current_yaw: 当前朝向角
        """
        from geometry_msgs.msg import Point
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "current_vector"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # 检查向量是否有效
        chi_norm = np.linalg.norm(chi_c)
        if chi_norm < self.epsilon:
            # 如果向量为0，不显示
            marker.action = Marker.DELETE
            self.current_vector_pub.publish(marker)
            return
        
        # 归一化并放大显示
        chi_c_normalized = chi_c / chi_norm
        arrow_length = 1.5  # 箭头长度（米）
        
        # 起点：车辆当前位置
        start = Point()
        start.x = x
        start.y = y
        start.z = 0.5  # 高度设置为0.5米，更显眼
        
        # 终点：沿向量方向
        end = Point()
        end.x = x + chi_c_normalized[0] * arrow_length
        end.y = y + chi_c_normalized[1] * arrow_length
        end.z = 0.5
        
        marker.points = [start, end]
        
        # 箭头样式：更粗更大
        marker.scale.x = 0.15  # 箭杆直径
        marker.scale.y = 0.25  # 箭头宽度
        marker.scale.z = 0.3   # 箭头高度
        
        # 颜色：亮紫色，非常显眼
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # 完全不透明
        
        marker.pose.orientation.w = 1.0
        
        self.current_vector_pub.publish(marker)
    
    def compute_local_vector_field(self):
        """计算并缓存固定区域的向量场网格
        
        向量场在map坐标系下固定，不随车辆移动
        范围: [center_x - range/2, center_x + range/2] x [center_y - range/2, center_y + range/2]
            
        Returns:
            dict: 网格坐标到向量的映射 {(x, y): [vx, vy]}
        """
        current_time = rospy.Time.now()
        
        # 检查是否需要更新向量场
        if (current_time - self.last_field_update_time).to_sec() < self.field_update_interval:
            return self.cached_vector_field
        
        self.last_field_update_time = current_time
        
        # 开始计时
        start_time = time.time()
        
        # 生成固定区域的网格 (map坐标系)
        half_range = self.field_range / 2.0
        x_values = np.arange(self.field_center_x - half_range, self.field_center_x + half_range, self.field_resolution)
        y_values = np.arange(self.field_center_y - half_range, self.field_center_y + half_range, self.field_resolution)
        
        # 计算整个网格的向量场
        vector_field = {}
        grid_count = 0
        
        for x in x_values:
            for y in y_values:
                try:
                    # 计算该点的复合向量场
                    chi_c = self.composite_vector_field(x, y)
                    
                    # 存储到字典中
                    grid_key = (round(x / self.field_resolution) * self.field_resolution,
                               round(y / self.field_resolution) * self.field_resolution)
                    vector_field[grid_key] = chi_c
                    grid_count += 1
                    
                except Exception as e:
                    rospy.logdebug("向量场计算失败 at (%.2f, %.2f): %s", x, y, str(e))
                    continue
        
        # 计算耗时
        elapsed_time = (time.time() - start_time) * 1000  # 转换为毫秒
        
        # 发布性能统计到话题
        if self.field_compute_time_pub.get_num_connections() > 0:
            self.field_compute_time_pub.publish(Float32(data=elapsed_time))
        
        # 更新缓存
        self.cached_vector_field = vector_field
        
        return vector_field
    
    def get_vector_at_position(self, x, y):
        """从缓存的向量场中获取指定位置的向量（使用最近邻插值）
        
        Args:
            x, y: 查询位置
            
        Returns:
            numpy array: 该位置的向量
        """
        # 检查输入是否有效
        if np.isnan(x) or np.isnan(y) or np.isinf(x) or np.isinf(y):
            rospy.logwarn("无效的位置输入: x=%.2f, y=%.2f", x, y)
            return np.array([0.0, 0.0])
        
        # 如果缓存为空，直接计算
        if not self.cached_vector_field:
            chi = self.composite_vector_field(x, y)
            # 检查结果是否有效
            if np.any(np.isnan(chi)) or np.any(np.isinf(chi)):
                return np.array([0.0, 0.0])
            return chi
        
        # 找到最近的网格点
        try:
            grid_x = round(x / self.field_resolution) * self.field_resolution
            grid_y = round(y / self.field_resolution) * self.field_resolution
            grid_key = (grid_x, grid_y)
            
            # 如果网格点存在，返回缓存的向量
            if grid_key in self.cached_vector_field:
                chi = self.cached_vector_field[grid_key]
                # 检查缓存的向量是否有效
                if np.any(np.isnan(chi)) or np.any(np.isinf(chi)):
                    return np.array([0.0, 0.0])
                return chi
            else:
                # 否则实时计算
                chi = self.composite_vector_field(x, y)
                if np.any(np.isnan(chi)) or np.any(np.isinf(chi)):
                    return np.array([0.0, 0.0])
                return chi
        except Exception as e:
            rospy.logwarn_throttle(1.0, "获取向量失败: %s", str(e))
            return np.array([0.0, 0.0])
    
    def generate_local_path(self, start_x, start_y):
        """通过向量场积分生成局部路径修改：
        
        
        
        Args:
            start_x, start_y: 起始位置（当前车辆位置）
            
        Returns:
            Path消息
        """
        local_path = Path()
        local_path.header.frame_id = "map"
        local_path.header.stamp = rospy.Time.now()
        
        # 从当前位置开始沿向量场积分
        x, y = start_x, start_y
        accumulated_distance = 0.0
        
        while accumulated_distance < self.local_path_length:
            # 创建路径点
            pose = PoseStamped()
            pose.header = local_path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # 获取该点的向量
            chi = self.get_vector_at_position(x, y)
            chi_norm = np.linalg.norm(chi)
            
            # 如果向量太小，停止积分
            if chi_norm < self.min_vector_norm:
                break
            
            # 向量方向作为朝向
            yaw = np.arctan2(chi[1], chi[0])
            quaternion = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            
            local_path.poses.append(pose)
            
            # 沿向量前进一步
            step = min(self.local_path_step, self.local_path_length - accumulated_distance)
            direction = chi / chi_norm
            x += direction[0] * step
            y += direction[1] * step
            accumulated_distance += step
        
        return local_path
    
    def visualize_vector_field(self):
    # """可视化固定区域的向量场 - map坐标系固定位置"""
        current_time = rospy.Time.now()
        if (current_time - self.last_viz_time).to_sec() < 1.0 / self.viz_frequency:
            return
        self.last_viz_time = current_time
        
        # 开始计时可视化
        viz_start = time.time()
        
        from geometry_msgs.msg import Point
        
        composite_array = MarkerArray()
        path_array = MarkerArray()
        reactive_array = MarkerArray()
        
        arrow_id = 0
        
        # 使用固定中心的网格范围
        half_range = self.field_range / 2.0
        x_values = np.arange(self.field_center_x - half_range, self.field_center_x + half_range, self.field_resolution)
        y_values = np.arange(self.field_center_y - half_range, self.field_center_y + half_range, self.field_resolution)
        
        for x in x_values:
            for y in y_values:
                try:
                    # 计算所有向量场
                    chi_c = self.composite_vector_field(x, y)
                    chi_p = self.path_tracking_vector_field(x, y)
                    chi_r = self.reactive_vector_field(x, y)
                    
                    scale = self.field_resolution * 0.8
                    
                    # 1. 复合向量场（橙色）
                    chi_c_norm = np.linalg.norm(chi_c)
                    if chi_c_norm > self.min_vector_norm:
                        chi_c_dir = chi_c / chi_c_norm
                        
                        arrow = Marker()
                        arrow.header.frame_id = "map"
                        arrow.header.stamp = current_time
                        arrow.ns = "composite_arrows"
                        arrow.id = arrow_id
                        arrow.type = Marker.ARROW
                        arrow.action = Marker.ADD
                        
                        start = Point(x, y, 0.1)
                        end = Point(x + chi_c_dir[0] * scale, y + chi_c_dir[1] * scale, 0.1)
                        arrow.points = [start, end]
                        
                        arrow.scale.x = 0.05
                        arrow.scale.y = 0.08
                        arrow.scale.z = 0.08
                        arrow.color.r = 1.0
                        arrow.color.g = 0.5
                        arrow.color.b = 0.0
                        arrow.color.a = 0.8
                        arrow.pose.orientation.w = 1.0  # 初始化四元数，避免警告
                        
                        composite_array.markers.append(arrow)
                        arrow_id += 1
                    
                    # 2. 路径跟踪向量场（绿色）
                    chi_p_norm = np.linalg.norm(chi_p)
                    if chi_p_norm > self.min_vector_norm:
                        chi_p_dir = chi_p / chi_p_norm
                        
                        arrow_p = Marker()
                        arrow_p.header.frame_id = "map"
                        arrow_p.header.stamp = current_time
                        arrow_p.ns = "path_arrows"  
                        arrow_p.id = arrow_id
                        arrow_p.type = Marker.ARROW
                        arrow_p.action = Marker.ADD
                        
                        start_p = Point(x, y, 0.05)
                        end_p = Point(x + chi_p_dir[0] * scale * 0.6, y + chi_p_dir[1] * scale * 0.6, 0.05)
                        arrow_p.points = [start_p, end_p]
                        
                        arrow_p.scale.x = 0.03
                        arrow_p.scale.y = 0.05
                        arrow_p.scale.z = 0.05
                        arrow_p.color.r = 0.0
                        arrow_p.color.g = 1.0
                        arrow_p.color.b = 0.0
                        arrow_p.color.a = 0.6
                        arrow_p.pose.orientation.w = 1.0  # 初始化四元数，避免警告
                        
                        path_array.markers.append(arrow_p)
                        arrow_id += 1
                    
                    # 3. 反应向量场（红色）
                    chi_r_norm = np.linalg.norm(chi_r)
                    if chi_r_norm > self.min_vector_norm:
                        chi_r_dir = chi_r / chi_r_norm
                        
                        arrow_r = Marker()
                        arrow_r.header.frame_id = "map"
                        arrow_r.header.stamp = current_time
                        arrow_r.ns = "reactive_arrows"
                        arrow_r.id = arrow_id
                        arrow_r.type = Marker.ARROW
                        arrow_r.action = Marker.ADD
                        
                        start_r = Point(x, y, 0.15)
                        end_r = Point(x + chi_r_dir[0] * scale * 0.6, y + chi_r_dir[1] * scale * 0.6, 0.15)
                        arrow_r.points = [start_r, end_r]
                        
                        arrow_r.scale.x = 0.03
                        arrow_r.scale.y = 0.05
                        arrow_r.scale.z = 0.05
                        arrow_r.color.r = 1.0
                        arrow_r.color.g = 0.0
                        arrow_r.color.b = 0.0
                        arrow_r.color.a = 0.6
                        arrow_r.pose.orientation.w = 1.0  # 初始化四元数，避免警告
                        
                        reactive_array.markers.append(arrow_r)
                        arrow_id += 1
                        
                except Exception as e:
                    rospy.logdebug("可视化失败 at (%.2f, %.2f): %s", x, y, str(e))
                    continue
        
        # 发布可视化
        if len(composite_array.markers) > 0:
            self.vector_field_pub.publish(composite_array)
        
        if len(path_array.markers) > 0:
            self.path_field_pub.publish(path_array)
        
        if len(reactive_array.markers) > 0:
            self.reactive_field_pub.publish(reactive_array)
        
        # 可视化耗时统计
        viz_elapsed = (time.time() - viz_start) * 1000
        if self.viz_time_pub.get_num_connections() > 0:
            self.viz_time_pub.publish(Float32(data=viz_elapsed))
    
    def run(self):
        """主控制循环"""
        rate = rospy.Rate(self.control_rate)
        
        rospy.loginfo("向量场规划器开始运行...")
        rospy.loginfo("使用隐式函数路径: %s", self.path_type)
        
        while not rospy.is_shutdown():
            # 检查是否有有效数据
            with self.pose_lock:
                if self.current_pose is None:
                    rospy.loginfo_throttle(5.0, "等待里程计数据...")
                    rate.sleep()
                    continue
                
                x = self.current_pose.position.x
                y = self.current_pose.position.y
            
            # 1. 计算固定区域的向量场网格 (map坐标系固定)
            self.compute_local_vector_field()
            
            # 2. 通过向量场积分生成局部路径
            local_path = self.generate_local_path(x, y)
            if len(local_path.poses) > 0:
                self.local_plan_pub.publish(local_path)
            
            # 3. 从缓存的向量场中获取当前位置的控制向量
            control_start = time.time()
            chi_c = self.get_vector_at_position(x, y)
            control_elapsed = (time.time() - control_start) * 1000
            
            # 发布控制查询时间
            if self.control_query_time_pub.get_num_connections() > 0:
                self.control_query_time_pub.publish(Float32(data=control_elapsed))
            
            # 直接计算当前位置的向量场（不用缓存）来调试
            chi_c_direct = self.composite_vector_field(x, y)
            chi_P_direct = self.path_tracking_vector_field(x, y)
            chi_P_norm = np.linalg.norm(chi_P_direct)
            
            # 4. 转换为控制命令
            current_yaw = self.get_current_yaw()
            cmd = self.vector_to_ackermann(chi_c, current_yaw)
            
            # 详细调试输出
            # rospy.loginfo_throttle(1.0, "位置: (%.2f,%.2f), chi_P原始norm=%.3f, chi_c=(%.3f,%.3f) norm=%.3f, vel=%.3f, steer=%.3f", 
            #                       x, y, chi_P_norm, chi_c[0], chi_c[1], np.linalg.norm(chi_c), cmd.linear.x, cmd.angular.z)
            
            # 5. 发布控制命令
            self.cmd_vel_pub.publish(cmd)
            
            # 6. 发布向量场可视化 (固定在map坐标系)
            self.visualize_vector_field()
            
            # 7. 发布障碍物边界可视化
            self.visualize_obstacle_boundaries()
            
            # 8. 可视化当前位置的向量场向量
            self.visualize_current_vector(x, y, chi_c, current_yaw)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        planner = VectorFieldPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
