#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 LIO-SAM 的里程计适配成下游规划/跟踪能直接吃的形式。干三件事:
  1. 姿态: 从「雷达自身坐标轴」旋到「车体坐标轴(x 朝车头)」
  2. 位置: 可选地平移参考点 (rear_axle_x, 默认 0 = 不平移, 参考点就是雷达)
  3. 速度: 输入里程计的 twist 不可信(见 twist_source), 默认用底盘 /ctrl_fb 反馈重建;
     并**同时发车体系和世界系两个版本**, 因为下游两个消费者要的坐标系不一样(见下)

为什么需要这个节点
------------------
实车 LIO-SAM 的配置是:
    config/params.yaml:   lidarFrame: "base_link"   baselinkFrame: "base_link"
    robot.urdf.xacro:     chassis_link -> velodyne 是单位阵(全零)
所以 LIO-SAM 给出的位姿就是**激光雷达物理安装点**的位姿, 姿态就是**雷达自身坐标
轴的朝向**。而 TEB 的阿克曼模型假设"车沿自身 +x 前进", 不修正朝向的话 TEB 认为
的车头方向偏 90 度, 会朝侧向猛打方向。

*** 输入必须用 /odometry/imu, 不能用 /lio_sam/mapping/odometry ***
------------------------------------------------------------------
LIO-SAM 有两路里程计输出, 用途完全不同 —— 实测 (2026-08-02, 本车 Jetson):

    话题                              header 延迟   频率     用途
    /lidar_points (驱动)                 158 ms      -       原始点云
    /lio_sam/deskew/cloud_deskewed       480 ms      -       imageProjection 去畸变
    /lio_sam/mapping/odometry            768 ms    4.7 Hz    **建图/回环用**
    /odometry/imu (TransformFusion)     19.8 ms    100 Hz    **控制用**
    TF odom->base_link                    12 ms    214 Hz    (RViz 画车用的就是它)

mapOptimization 要做 scan-to-map 匹配, 在 Jetson 上跑 92 万点的稠密云需要 768ms,
出位姿只有 4.7Hz。imuPreintegration 里的 TransformFusion 正是为此存在的:
它把最近一帧 768ms 前的雷达位姿, 用 IMU 预积分外推到当前时刻, 以 IMU 频率发布
/odometry/imu, 坐标系与原点和 mapping/odometry 完全一致(frame_id 都是 odom)。

【踩过的坑 —— "车明明在路径上却莫名其妙开始转弯"】
RViz 画车身和点云用的是 TF(12ms 延迟), 而 EGO/TEB 当时吃的是 mapping/odometry
(768ms 延迟)。于是**人眼看到的车和控制器看到的车根本不是同一个车**:
    0.3 m/s 下 768ms = 位置落后 23cm
    车一旦有横摆角速度 w, 航向落后 w*0.768s; w=0.2rad/s 时**航向差 8.8 度**
TEB 每周期用这个落后的位姿当固定起点(buildInitialPlan), 于是:
  - 车已经修正回路径了, TEB 还看到"车偏在一边", 继续打方向 -> 过冲
  - 过冲后又反向重复 -> 纯延迟环典型的极限环, 直线上表现为**画龙**
  - 摆动周期约 4*总延迟 ~ 4s, 0.3m/s 下波长约 1.2m
换成 /odometry/imu 后感知延迟 768ms -> 20ms, 采样 4.7Hz -> 100Hz, 相位裕度提高
一个数量级, 同样的增益下环路从振荡变稳定。

注意: 只改本节点的输入话题, **LIO-SAM 本身一行不动**。

*** 两个输出话题, 区别只在 twist 的坐标系 ***
------------------------------------------------
/legOdom        twist = **车体系** (ROS 标准: twist 表达在 child_frame 里)
                -> teb_tracking.cpp:271-272 读 twist.linear.x / angular.z 当作
                   "前进速度 / 横摆角速度", 正是车体系语义。
/legOdom_world  twist = **世界系**
                -> ego_replan_fsm.cpp:199-201 把 twist.linear 整个当成**世界系
                   速度向量**存进 odom_vel_, 再作为 B 样条的起始速度边界条件
                   (planner_manager.cpp:93 one_segment_traj_gen(start_pt,
                   start_vel,...) 建的是世界系多项式)。

【踩过的坑】曾经只发一个车体系 twist 给两边吃, 结果 EGO 不管车头朝哪, 都以为
"我正以 v m/s 朝世界 +X 运动"。车头朝世界 -Y 时, 样条必须先往 +X 出发再拐回目标
-> 规划出 S 弯; 车头一变弯就重算 -> 路径一直在变。全局轨迹(planGlobalTraj)吃的
是同一个错向量, 所以连全局参考路径本身都是弯的。
排查判据: 车朝世界 +X 开时路径正常、朝其它方向开就弯, 就是这个问题。

坐标定义
--------
lidar_yaw_offset_deg: 车头方向在雷达坐标系里的朝向(度)。
    -90  表示「车头 = 雷达 -y 轴」(本车实测值)
      0  表示「车头 = 雷达 +x 轴」(雷达与车体同向, 无需旋转)
    车体 yaw = 雷达 yaw + lidar_yaw_offset_deg

rear_axle_x: 参考点在**车体坐标系**下相对雷达的纵向位置(米), 负值 = 车后方。
    **默认 0, 即参考点就是雷达安装点。** 本车雷达正好装在前轴上(后轴在雷达后
    0.66m = 轴距), 所以参考点 = 前轴中心, 转角换算走前轴自行车模型
    delta = asin(w*L/v) (见 nav_ackermann.py)。
    只有当你想让 TEB 用严格的后轴模型 delta=atan(w*L/v) 时才填 -0.66, 但那样
    EGO 的规划起点和 grid_map 的参考点也会一起退到车后 0.66m ——
    路径起点看上去落在车已经走过的地方, 且到点时车会越过目标点 0.66m。

twist_source: twist 从哪里来 —— **换输入话题后这个参数必须配对改, 否则速度全错**
--------------------------------------------------------------------------
两路 LIO-SAM 里程计的 twist 语义完全不同, 而且都不能直接用:

  /lio_sam/mapping/odometry : mapOptmization 根本不填 twist, 恒为 0。
  /odometry/imu             : imuPreintegration 填了, 但是**混合坐标系**——
        twist.linear  = GTSAM NavState 的速度, 在**世界系(导航系)**
        twist.angular = 陀螺原始角速度, 在**车体系**
      而且积分有偏置: 实测车静止时 linear = (-0.0104, -0.0039) m/s。

  ctrl_fb (默认, 推荐): 完全忽略输入 twist, 用底盘 /ctrl_fb 的真实轮速 + 真实
      前轮转角按自行车模型重建。100Hz, 无偏置, 语义明确。
  odom_body:        输入 twist 按「表达在输入话题的 child_frame」处理(ROS 标准),
                    只适用于确实按标准填 twist 的里程计源。
  odom_world_linear:输入 linear 按世界系、angular 按车体系处理(/odometry/imu 的语义)。

为什么必须显式给这个参数: 老版本靠「twist 全零就说明没人填」来自动判断。换到
/odometry/imu 之后 twist **非零但语义不同**, 那个自动判断会静默地走错分支, 把
世界系速度当车体系速度转一遍 —— 车头朝哪速度就错成哪样, 极难查。

输出
----
/legOdom       (nav_msgs/Odometry) teb_tracking、waypoint_generator、grid_map
/legOdom_world (nav_msgs/Odometry) ego_planner_node (launch 里 remap 过去)
两者 pose 完全相同, 只有 twist 的坐标系不同。frame_id 保持 odom。
发布频率 = 输入频率, 但受 max_output_rate 限幅(默认 50Hz): teb_tracking 只有
20Hz 控制环, 50Hz 已是 2.5 倍过采样; 限幅带来的零阶保持延迟 10ms, 相对省下的
768ms 可忽略, 而 Python 端 100Hz 做四元数运算在 Jetson 上没必要。

注意: grid_map 的点云是靠 TF(velodyne->odom) 自己变换的, 本节点**不动 TF 树**,
LIO-SAM 原有的 odom->base_link 保持不变, 所以点云仍然落在正确的世界位置。
本节点只额外发一个 odom-><rear_frame> 的 TF 供 RViz 观察(可关)。
"""

import math
import threading

import numpy as np
import rospy
import tf
import tf.transformations as tft
from nav_msgs.msg import Odometry

try:
    from yhs_can_msgs.msg import ctrl_fb
    HAVE_CTRL_FB = True
except ImportError:          # 仿真机上没有该消息包, 自动降级
    HAVE_CTRL_FB = False


class OdomRearAxle(object):
    def __init__(self):
        # 默认 /odometry/imu 而**不是** /lio_sam/mapping/odometry: 后者实测 768ms
        # 延迟 / 4.7Hz, 拿来闭环会画龙。详见模块头注释。
        self.in_topic = rospy.get_param('~input_topic', '/odometry/imu')
        self.out_topic = rospy.get_param('~output_topic', '/legOdom')
        # 同样的 pose, 但 twist 表达在世界系 —— 专供 ego_planner_node, 见模块头注释
        self.world_topic = rospy.get_param('~world_twist_topic', '/legOdom_world')
        # 输出限频(Hz), <=0 表示不限。默认 50: teb_tracking 控制环只有 20Hz。
        self.max_rate = float(rospy.get_param('~max_output_rate', 50.0))
        # 抽稀门限留 10% 容差: 输入 100Hz(10ms) 抽到 50Hz(20ms) 时, 源的时间戳抖动会
        # 让 dt 落在 19.x ms 上被误判成"太密"而丢弃, 下一帧要等到 30ms ——
        # 实测这样只能出 38.8Hz 而不是 50Hz, 白白多 15ms 延迟。
        self._min_period = (0.9 / self.max_rate) if self.max_rate > 0 else 0.0
        self._last_pub = None

        # 雷达 -> 车体 的偏航修正
        self.yaw_offset = math.radians(float(rospy.get_param('~lidar_yaw_offset_deg', -90.0)))
        # 参考点在车体系下相对雷达的位置。默认 0 = 参考点就是雷达(本车雷达在前轴上)
        self.rear_x = float(rospy.get_param('~rear_axle_x', 0.0))
        self.rear_y = float(rospy.get_param('~rear_axle_y', 0.0))
        self.rear_z = float(rospy.get_param('~rear_axle_z', 0.0))

        self.publish_tf = bool(rospy.get_param('~publish_tf', True))
        self.rear_frame = rospy.get_param('~rear_frame', 'base_rear_axle')
        # 是否把 z/roll/pitch 压平(2D 导航常用, 避免坡度让 B 样条跑到空中)
        self.flatten_2d = bool(rospy.get_param('~flatten_2d', False))

        # 车体相对雷达的固定旋转 q_LB (绕 z 转 yaw_offset)
        self.q_lb = tft.quaternion_from_euler(0.0, 0.0, self.yaw_offset)
        # 后轴在车体系下的位置向量
        self.r_b = np.array([self.rear_x, self.rear_y, self.rear_z])

        self.br = tf.TransformBroadcaster() if self.publish_tf else None
        self.pub = rospy.Publisher(self.out_topic, Odometry, queue_size=10)
        self.pub_world = rospy.Publisher(self.world_topic, Odometry, queue_size=10)

        # ---- twist 来源 ----
        # 为什么必须重建: teb_tracking.cpp:271-272 从 odom 的 twist 取 TEB 的**起始
        # 速度**。若恒为 0, TEB 每个 50ms 周期都以为"车此刻静止", 配合 acc_lim_theta
        # 限制会认为角速度必须从 0 爬升 -> 系统性少打方向、转弯偏晚、切内道。
        # 而 /odometry/imu 虽然填了 twist, 却是「linear 世界系 + angular 车体系」的
        # 混合语义且带偏置, 直接用会错得很隐蔽。默认一律用底盘 /ctrl_fb 重建。
        self.twist_source = str(rospy.get_param('~twist_source', 'ctrl_fb')).strip().lower()
        if self.twist_source not in ('ctrl_fb', 'odom_body', 'odom_world_linear'):
            rospy.logwarn('[odom_rear_axle] twist_source="%s" 无法识别, 按 ctrl_fb 处理',
                          self.twist_source)
            self.twist_source = 'ctrl_fb'
        # 兼容老参数名: use_ctrl_fb=false 等价于 twist_source=odom_body
        if not bool(rospy.get_param('~use_ctrl_fb', True)) and self.twist_source == 'ctrl_fb':
            rospy.logwarn('[odom_rear_axle] use_ctrl_fb=false 已废弃, '
                          '按 twist_source=odom_body 处理')
            self.twist_source = 'odom_body'
        if self.twist_source == 'ctrl_fb' and not HAVE_CTRL_FB:
            rospy.logwarn('[odom_rear_axle] 本机没有 yhs_can_msgs, '
                          'twist_source 降级为 odom_body')
            self.twist_source = 'odom_body'
        self.use_ctrl_fb = (self.twist_source == 'ctrl_fb')
        self.ctrl_fb_topic = rospy.get_param('~ctrl_fb_topic', '/ctrl_fb')
        # twist_source=ctrl_fb 但底盘反馈断了时的降级来源。
        # 'zero' = 退 0 (旧行为); 'odom_body'/'odom_world_linear' = 用输入里程计的
        # twist, 语义与 twist_source 里同名取值一致。**实车上 /ctrl_fb 会真的消失**
        # (yhs_can_control 退出/CAN 断), 此时退 0 让 TEB 以为车静止会少打方向,
        # 降级到输入 twist 比退 0 强。
        self.fallback = str(rospy.get_param('~fallback_twist_source', 'zero')).strip().lower()
        if self.fallback not in ('zero', 'odom_body', 'odom_world_linear'):
            rospy.logwarn('[odom_rear_axle] fallback_twist_source="%s" 无法识别, 按 zero 处理',
                          self.fallback)
            self.fallback = 'zero'
        self.wheelbase = float(rospy.get_param('~wheelbase', 0.66))
        # 参考点在哪个轴上 —— 决定 ctrl_fb 反馈换算出的**线速度**归算到哪个点。
        # 必须与 nav_ackermann.ref_point 一致(统一由 yaml 的 *ref_point 锚点给)。
        self.front_ref = str(rospy.get_param('~ref_point', 'front_axle')).strip().lower() \
            == 'front_axle'
        self.gear_reverse = int(rospy.get_param('~gear_reverse', 2))
        self.fb_timeout = float(rospy.get_param('~ctrl_fb_timeout', 0.5))
        self.fb_v = 0.0
        self.fb_w = 0.0
        self.fb_time = rospy.Time(0)
        if self.use_ctrl_fb:
            rospy.Subscriber(self.ctrl_fb_topic, ctrl_fb, self.fb_cb, queue_size=5)

        self._warned_twist = False
        self._lock = threading.Lock()
        self._last_log = rospy.Time(0)

        self.sub = rospy.Subscriber(self.in_topic, Odometry, self.cb, queue_size=10)

        rospy.loginfo('[odom_rear_axle] %s -> %s (twist 车体系, 供 teb_tracking)',
                      self.in_topic, self.out_topic)
        rospy.loginfo('[odom_rear_axle]            -> %s (twist 世界系, 供 ego_planner_node)',
                      self.world_topic)
        rospy.loginfo('[odom_rear_axle] twist_source = %s, 输出限频 %s',
                      self.twist_source,
                      ('%.0f Hz' % self.max_rate) if self.max_rate > 0 else '不限')
        if 'mapping/odometry' in self.in_topic:
            rospy.logwarn('[odom_rear_axle] 输入用的是 mapOptimization 的建图里程计, '
                          '实测 768ms 延迟 / 4.7Hz —— 闭环跟踪会画龙! '
                          '控制请改用 /odometry/imu (TransformFusion, 20ms / 100Hz)')
        _axis = {0: '+x', 90: '+y', -90: '-y', 180: '-x', -180: '-x'}.get(
            int(round(math.degrees(self.yaw_offset))), '自定义方向')
        rospy.loginfo('[odom_rear_axle] lidar_yaw_offset = %.1f deg (车头 = 雷达 %s 方向)',
                      math.degrees(self.yaw_offset), _axis)
        if abs(self.rear_x) < 1e-6 and abs(self.rear_y) < 1e-6:
            rospy.loginfo('[odom_rear_axle] 参考点 = 雷达安装点(不平移)。'
                          '本车雷达在前轴上, 故 nav_ackermann 需用前轴模型 delta=asin(w*L/v)')
        else:
            rospy.logwarn('[odom_rear_axle] 参考点已平移到车体系 (%.3f, %.3f, %.3f) m。'
                          '注意 EGO 的规划起点和 grid_map 参考点会一起移动, '
                          'nav_ackermann 需改回后轴模型 delta=atan(w*L/v)',
                          self.rear_x, self.rear_y, self.rear_z)
        if self.flatten_2d:
            rospy.loginfo('[odom_rear_axle] flatten_2d=true: 输出 z=0, roll=pitch=0')

    def fb_cb(self, msg):
        """底盘反馈 -> (v, omega)。ctrl_fb_velocity 是速度大小(底盘按后轴/驱动轮计),
        方向看挡位; ctrl_fb_steering 是前轮转角(度)。

        横摆角速度与参考点无关: omega = v_rear * tan(delta) / L
        线速度**与参考点有关**: 前轴点速度 v_front = v_rear / cos(delta)
        (前轴沿前轮方向走, 转弯半径比后轴大, 走得更快; 满舵 30 度时高 15%)
        """
        v = abs(msg.ctrl_fb_velocity)
        if msg.ctrl_fb_gear == self.gear_reverse:
            v = -v
        delta = math.radians(msg.ctrl_fb_steering)
        self.fb_w = v * math.tan(delta) / self.wheelbase
        self.fb_v = v / math.cos(delta) if self.front_ref else v
        self.fb_time = rospy.Time.now()

    def _twist_from_odom(self, msg, R_v, R_lb, source):
        """从输入里程计提取车体系 (v_r, w_r)。source 语义见模块头注释 twist_source 节。

        w_r 与参考点无关: 只需从雷达系旋到车体系。
        v_r 与语义有关:
          odom_world_linear (即 /odometry/imu 的语义)
              twist.linear 是 GTSAM NavState 速度, 在**世界系**:
                  车体系 v = R_wb^T * v_world + omega x r_ref   (牵连速度)
          odom_body (标准 ROS 语义)
              twist 表达在输入话题的 child frame(雷达系):
                  v_ref = R_LB^T * (v_lidar + omega_lidar x r_l)
        """
        v_l = np.array([msg.twist.twist.linear.x,
                        msg.twist.twist.linear.y,
                        msg.twist.twist.linear.z])
        w_l = np.array([msg.twist.twist.angular.x,
                        msg.twist.twist.angular.y,
                        msg.twist.twist.angular.z])
        w_r = R_lb.T.dot(w_l)
        if source == 'odom_world_linear':
            v_r = R_v.T.dot(v_l) + np.cross(w_r, self.r_b)
        else:  # odom_body
            r_l = R_lb.dot(self.r_b)            # 参考点在雷达系下的位置
            v_r = R_lb.T.dot(v_l + np.cross(w_l, r_l))
        return v_r, w_r

    def cb(self, msg):
        # ---- 输出限频 ----
        # /odometry/imu 是 100Hz, 而 teb_tracking 控制环只有 20Hz。按源时间戳抽稀,
        # 省掉 Jetson 上 Python 端一半的四元数运算。max_output_rate<=0 则不限频。
        if self._min_period > 0.0:
            t = msg.header.stamp.to_sec()
            if self._last_pub is not None:
                dt = t - self._last_pub
                if 0.0 <= dt < self._min_period:
                    return                      # 太密, 丢弃
                if dt < 0.0:
                    self._last_pub = None       # 时间回跳(重放/重启), 重新计时
            self._last_pub = t

        # ---- 姿态: q_vehicle = q_lidar * q_LB ----
        q_l = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        n = np.linalg.norm(q_l)
        if n < 1e-9 or not np.isfinite(n):
            rospy.logwarn_throttle(5.0, '[odom_rear_axle] 收到非法四元数, 丢弃该帧')
            return
        q_l = q_l / n
        q_v = tft.quaternion_multiply(q_l, self.q_lb)

        # ---- 位置: p_rear = p_lidar + R_vehicle * r_b ----
        R_v = tft.quaternion_matrix(q_v)[:3, :3]
        p_l = np.array([msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z])
        p_r = p_l + R_v.dot(self.r_b)

        if self.flatten_2d:
            p_r[2] = 0.0
            yaw = tft.euler_from_quaternion(q_v)[2]
            q_v = tft.quaternion_from_euler(0.0, 0.0, yaw)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id      # odom
        out.child_frame_id = self.rear_frame
        out.pose.pose.position.x = p_r[0]
        out.pose.pose.position.y = p_r[1]
        out.pose.pose.position.z = p_r[2]
        out.pose.pose.orientation.x = q_v[0]
        out.pose.pose.orientation.y = q_v[1]
        out.pose.pose.orientation.z = q_v[2]
        out.pose.pose.orientation.w = q_v[3]
        out.pose.covariance = msg.pose.covariance

        # ---- 速度(twist) ----
        # 输出按 ROS 约定表达在 child frame(车体系)里, 供 teb_tracking 当 TEB 起始速度。
        # 输入 twist 的语义由 twist_source 显式指定 —— 绝不做"全零就当没填"的自动
        # 猜测, 因为 /odometry/imu 的 twist 非零但是混合坐标系(见模块头注释)。
        R_lb = tft.quaternion_matrix(self.q_lb)[:3, :3]

        if self.twist_source == 'ctrl_fb':
            fb_fresh = (self.fb_time != rospy.Time(0) and
                        (rospy.Time.now() - self.fb_time).to_sec() < self.fb_timeout)
            if fb_fresh:
                v_r = np.array([self.fb_v, 0.0, 0.0])   # 车体系, x 朝车头
                w_r = np.array([0.0, 0.0, self.fb_w])
            else:
                # 底盘反馈断了。**实车上真的会发生**(yhs_can_control 退出/CAN 断),
                # 此时退回 0 会让 TEB 每周期都以为车静止 -> 系统性少打方向。
                # 降级到输入里程计的 twist 比退 0 强, 语义由 fallback_twist_source
                # 显式给定, 仍然不做猜测。
                if self.fallback == 'zero':
                    v_r = np.zeros(3)
                    w_r = np.zeros(3)
                else:
                    v_r, w_r = self._twist_from_odom(msg, R_v, R_lb, self.fallback)
                if not self._warned_twist:
                    self._warned_twist = True
                    rospy.logwarn('[odom_rear_axle] twist_source=ctrl_fb 但 %s 无数据 -> '
                                  '降级为 %s。底盘驱动(yhs_can_control)起了吗?',
                                  self.ctrl_fb_topic, self.fallback)
                else:
                    rospy.logwarn_throttle(5.0,
                                           '[odom_rear_axle] %s 数据超时(>%.1fs), 降级为 %s',
                                           self.ctrl_fb_topic, self.fb_timeout, self.fallback)
        else:
            v_r, w_r = self._twist_from_odom(msg, R_v, R_lb, self.twist_source)

        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = v_r
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = w_r
        out.twist.covariance = msg.twist.covariance

        self.pub.publish(out)

        # 世界系版本: 同一个 pose, twist 旋到世界系。
        # ego_replan_fsm.cpp:199-201 把 twist.linear 整个当世界系速度向量存进
        # odom_vel_, 再当 B 样条起始速度边界条件 —— 喂车体系会让 EGO 以为车永远
        # 朝世界 +X 走, 规划出 S 弯(详见模块头注释)。
        out_w = Odometry()
        out_w.header = out.header
        out_w.child_frame_id = out.child_frame_id
        out_w.pose = out.pose                   # pose 与车体系版本完全相同
        v_w = R_v.dot(v_r)
        w_w = R_v.dot(w_r)
        out_w.twist.twist.linear.x, out_w.twist.twist.linear.y, out_w.twist.twist.linear.z = v_w
        out_w.twist.twist.angular.x, out_w.twist.twist.angular.y, out_w.twist.twist.angular.z = w_w
        out_w.twist.covariance = msg.twist.covariance
        self.pub_world.publish(out_w)

        if self.br is not None:
            self.br.sendTransform((p_r[0], p_r[1], p_r[2]),
                                  (q_v[0], q_v[1], q_v[2], q_v[3]),
                                  msg.header.stamp,
                                  self.rear_frame,
                                  msg.header.frame_id)

        now = rospy.Time.now()
        if (now - self._last_log).to_sec() > 2.0:
            self._last_log = now
            rospy.loginfo_throttle(
                2.0, '[odom_rear_axle] lidar(%.2f, %.2f, yaw=%.1f) -> rear(%.2f, %.2f, yaw=%.1f)',
                p_l[0], p_l[1], math.degrees(tft.euler_from_quaternion(q_l)[2]),
                p_r[0], p_r[1], math.degrees(tft.euler_from_quaternion(q_v)[2]))


if __name__ == '__main__':
    rospy.init_node('odom_rear_axle')
    OdomRearAxle()
    rospy.spin()
