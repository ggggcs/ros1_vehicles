#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT-C2VF 有限时间曲率约束向量场控制器 (ROS1)
============================================
论文: Finite-Time Curvature-Constrained Vector Fields for
      Saturation-Free Motion Planning of Nonholonomic Robots

功能: 引导非完整机器人以指定朝向在有限时间内到达目标配置,
      同时满足曲率约束且控制无饱和。

话题 (与参考程序 w_GP_actual_guassian_5_1.py 保持一致):
  订阅: /AIMhc_UGV1/odom  (Odometry)
        /ctrl_fb           (ctrl_fb, 可选)
  发布: /ctrl_cmd          (ctrl_cmd, 控制指令)
        /ft_c2vf/theta_e   (Float32, 朝向误差)
        /ft_c2vf/eta       (Float32, 径向比)
        /ft_c2vf/kappa_c   (Float32, 命令曲率)
        /ft_c2vf/v         (Float32, 线速度)
        /ft_c2vf/omega     (Float32, 角速度)
        /vis/actual_path   (Path, 实际轨迹)
        /vis/desired_manifold (Path, 期望圆环)
        /vis/ref_trajectory   (Path, 参考向量场流线)

使用方法:
  # 使用默认参数启动
  rosrun your_package ft_c2vf_controller.py

  # 通过参数服务器配置 (launch file 示例)
  roslaunch your_package ft_c2vf.launch

  # 离线测试 (不依赖 ROS)
  python ft_c2vf_controller.py --test
"""

import sys
import threading
import math
import numpy as np
from collections import deque

# ── ROS1 imports ──
try:
    import rospy
    import tf
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped, Twist
    from std_msgs.msg import Float32
    from visualization_msgs.msg import Marker
    from yhs_can_msgs.msg import ctrl_cmd, ctrl_fb
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


# ╔══════════════════════════════════════════════════════════════════════╗
# ║                   1. FT-C2VF 核心规划器 (纯算法)                      ║
# ╚══════════════════════════════════════════════════════════════════════╝

class FTC2VFPlanner:
    """
    FT-C2VF 算法核心实现，无 ROS 依赖，可直接单独测试。

    参数说明 (与论文一致):
        kappa_max : 最大允许曲率 κ̄
        gamma     : 收敛形状参数 γ ∈ (0, 1)
        alpha     : 朝向误差指数 α, 需满足 α ≤ 2γ/(1+γ)
        beta      : 速度收敛指数 β ∈ (0, 0.5)
        mu        : 位置-朝向误差权重
        kv        : 速度响应增益
        v_max     : 最大线速度
        v_min     : 最小线速度 (0=可停止, >0=必须前进如固定翼)
        omega_max : 最大角速度
        r         : 期望圆环半径 (为 None 则自动计算)
    """

    def __init__(self, kappa_max=1.0, gamma=0.5, alpha=0.6, beta=0.4,
                 mu=4.0, kv=1.0, v_max=1.0, v_min=0.0, omega_max=1.0,
                 r=None):
        self.kappa_max = float(kappa_max)
        self.gamma = float(gamma)
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.mu = float(mu)
        self.kv = float(kv)
        self.v_max = float(v_max)
        self.v_min = float(v_min)
        self.omega_max = float(omega_max)

        # 自动计算或使用指定的半径
        if r is None:
            self.r = (3.0 + 2.0 * self.gamma) / self.kappa_max
        else:
            self.r = float(r)

        # 目标配置
        self.xd = 0.0
        self.yd = 0.0
        self.thetad = 0.0
        self.vd = 0.0
        self.cx = 0.0
        self.cy = 0.0

        # 验证参数
        self._validate_params()

        # 调试数据 (最近一次控制计算的内部状态)
        self.debug = {}

    def _validate_params(self):
        """验证参数约束 (Theorem 3 & 4)，返回警告列表"""
        warnings = []
        r_min = (3.0 + 2.0 * self.gamma) / self.kappa_max
        if self.r < r_min:
            warnings.append(
                "r=%.3f < (3+2γ)/κ̄=%.3f, 闭环曲率约束可能不满足" % (self.r, r_min))
        alpha_max = 2.0 * self.gamma / (1.0 + self.gamma)
        if self.alpha > alpha_max:
            warnings.append(
                "α=%.3f > 2γ/(1+γ)=%.3f, 有限时间收敛条件不满足" % (self.alpha, alpha_max))
        if self.beta >= 0.5:
            warnings.append(
                "β=%.3f ≥ 0.5, 有限时间速度匹配条件不满足" % self.beta)
        if self.v_min > 0 and self.kappa_max * self.v_min >= self.omega_max:
            warnings.append(
                "κ̄·v₋=%.3f ≥ ω̄=%.3f, 控制输入集可能为空" % (
                    self.kappa_max * self.v_min, self.omega_max))
        return warnings

    def set_target(self, xd, yd, thetad, vd=0.0):
        """
        设定目标配置并计算期望圆环圆心。

        参数:
            xd, yd : 目标位置
            thetad : 目标朝向 (rad)
            vd     : 目标速度 (0=停稳, >0=巡航经过)
        """
        self.xd = float(xd)
        self.yd = float(yd)
        self.thetad = float(thetad)
        self.vd = float(vd)

        # 公式: c = ξd + σ·r·n(θd)
        sigma = 1.0  # σ ∈ {-1, 1}, 选择期望圆环在哪一侧
        self.cx = self.xd + sigma * self.r * (-math.sin(self.thetad))
        self.cy = self.yd + sigma * self.r * math.cos(self.thetad)

    def compute_control(self, x, y, theta):
        """
        计算控制指令 (v, omega)。

        参数:
            x, y  : 当前位置
            theta : 当前朝向 (rad)

        返回:
            (v, omega, debug_dict)
        """
        # ── 1. 计算相对圆心位置与径向比 η ──
        xc = x - self.cx
        yc = y - self.cy
        norm_xi = math.sqrt(xc * xc + yc * yc)
        eta = norm_xi / self.r if self.r > 0 else 1.0

        # ── 2. 计算增益函数 ψτ, ψν (公式 8, 9) ──
        if eta <= 1.0:
            # S₁ 区域 (圆内)
            psi_tau = 1.0 - (1.0 - eta) ** (1.0 + self.gamma)
            psi_nu = math.sqrt(max(0.0, 1.0 - psi_tau * psi_tau))
            # 曲率 κχ (公式 34)
            if eta > 1e-10:
                kappa_chi = (
                    1.0 - (1.0 - eta) ** (1.0 + self.gamma)
                    + eta * (1.0 + self.gamma) * (1.0 - eta) ** self.gamma
                ) / (self.r * eta)
            else:
                kappa_chi = 2.0 * (1.0 + self.gamma) / self.r
        else:
            # S₂ 区域 (圆外)
            inv_eta = 1.0 / eta
            psi_tau = 1.0 - (1.0 - inv_eta) ** (1.0 + self.gamma)
            psi_nu = -math.sqrt(max(0.0, 1.0 - psi_tau * psi_tau))
            # 曲率 κχ (公式 35)
            one_m_inv = 1.0 - inv_eta
            kappa_chi = (
                1.0 - one_m_inv ** (1.0 + self.gamma)
                - (1.0 + self.gamma) * inv_eta * one_m_inv ** self.gamma
            ) / (self.r * eta)

        # ── 3. 计算向量场方向 (公式 4) ──
        grad_phi_x = 2.0 * xc
        grad_phi_y = 2.0 * yc
        # χ = ψτ·E·∇ϕ + ψν·∇ϕ,  E = [[0,-1],[1,0]]
        chi_x = psi_tau * (-grad_phi_y) + psi_nu * grad_phi_x
        chi_y = psi_tau * grad_phi_x + psi_nu * grad_phi_y
        chi_angle = math.atan2(chi_y, chi_x)

        # ── 4. 朝向误差 θe ──
        theta_e = math.atan2(math.sin(theta - chi_angle),
                             math.cos(theta - chi_angle))

        # ── 5. 构型度量 Ξ (公式 13) ──
        dist_to_target_sq = (x - self.xd) ** 2 + (y - self.yd) ** 2
        psi_val = dist_to_target_sq + self.mu * (1.0 - math.cos(theta_e))

        # ── 6. 命令曲率 κc (公式 14) ──
        k_omega = self.kappa_max - kappa_chi
        sin_theta_e_alpha = (abs(math.sin(theta_e)) ** self.alpha
                             * (1.0 if math.sin(theta_e) >= 0 else -1.0))
        kappa_c = kappa_chi - k_omega * sin_theta_e_alpha

        # ── 7. 自适应速度上界 vr (公式 16) ──
        abs_kc = abs(kappa_c)
        omega_margin = self.omega_max - self.v_min * abs_kc
        if omega_margin <= 1e-10:
            v_r = self.v_min
        else:
            v_r = self.v_min + 1.0 / (
                1.0 / (self.v_max - self.v_min) + abs_kc / omega_margin
            )

        # ── 8. 线速度 v (公式 17a) ──
        v = self.v_min + (v_r - self.v_min) * (
            1.0 - math.exp(-self.kv * psi_val ** self.beta)
        )

        # ── 9. 角速度 ω (公式 17b) ──
        omega = v * kappa_c

        # ── 存储调试信息 ──
        self.debug = {
            'eta': eta, 'theta_e': theta_e,
            'kappa_chi': kappa_chi, 'kappa_c': kappa_c,
            'k_omega': k_omega, 'v_r': v_r,
            'psi': psi_val, 'chi_angle': chi_angle,
            'dist_to_target': math.sqrt(dist_to_target_sq),
        }

        return v, omega, self.debug

    def get_vector_field_at(self, x, y):
        """
        返回 (x, y) 位置的向量场方向角 (用于可视化)。
        若位置恰好在奇异点 (圆心)，返回 None。
        """
        xc = x - self.cx
        yc = y - self.cy
        norm_xi = math.sqrt(xc * xc + yc * yc)
        if norm_xi < 1e-10:
            return None
        eta = norm_xi / self.r

        if eta <= 1.0:
            psi_tau = 1.0 - (1.0 - eta) ** (1.0 + self.gamma)
            psi_nu = math.sqrt(max(0.0, 1.0 - psi_tau * psi_tau))
        else:
            inv_eta = 1.0 / eta
            psi_tau = 1.0 - (1.0 - inv_eta) ** (1.0 + self.gamma)
            psi_nu = -math.sqrt(max(0.0, 1.0 - psi_tau * psi_tau))

        grad_phi_x = 2.0 * xc
        grad_phi_y = 2.0 * yc
        chi_x = psi_tau * (-grad_phi_y) + psi_nu * grad_phi_x
        chi_y = psi_tau * grad_phi_x + psi_nu * grad_phi_y

        return math.atan2(chi_y, chi_x)

    def integrate_trajectory(self, start_x, start_y, steps=400, dt=0.1):
        """
        沿向量场积分生成参考轨迹 (用于 Rviz 可视化)。
        返回 [(x, y), ...] 列表。
        """
        path = []
        x, y = float(start_x), float(start_y)
        for _ in range(steps):
            path.append((x, y))
            dir_angle = self.get_vector_field_at(x, y)
            if dir_angle is None:
                break
            x += self.v_max * math.cos(dir_angle) * dt
            y += self.v_max * math.sin(dir_angle) * dt
        return path

    def get_desired_manifold_points(self, n_points=200):
        """生成期望圆环 Mr 的离散点 (用于 Rviz 可视化)。"""
        points = []
        for i in range(n_points):
            angle = 2.0 * math.pi * i / n_points
            points.append((
                self.cx + self.r * math.cos(angle),
                self.cy + self.r * math.sin(angle),
            ))
        return points


# ╔══════════════════════════════════════════════════════════════════════╗
# ║              2. FT-C2VF ROS1 控制器节点                              ║
# ╚══════════════════════════════════════════════════════════════════════╝

class FTC2VFControllerNode:
    """
    ROS1 控制器节点，集成 FT-C2VF 规划器。

    核心流程:
      1. 订阅 odom 获取 [x, y, θ]
      2. 定时器调用 FT-C2VF 计算 [v, ω]
      3. 将 [v, ω] 转为 ctrl_cmd 格式 (velocity + steering_angle)
      4. 发布控制指令和可视化数据
    """

    def __init__(self):
        if not ROS_AVAILABLE:
            raise RuntimeError(
                "ROS 环境不可用。请 source ROS setup.bash 后运行。\n"
                "或使用 --test 模式进行离线测试: "
                "python ft_c2vf_controller.py --test"
            )

        rospy.init_node('ft_c2vf_controller_node', anonymous=False)

        # ── 读取参数 ──
        # 话题
        self.odom_topic = rospy.get_param('~odom_topic', '/AIMhc_UGV1/odom')
        self.cmd_topic = rospy.get_param('~cmd_topic', '/ctrl_cmd')
        self.ctrl_fb_topic = rospy.get_param('~ctrl_fb_topic', '/ctrl_fb')

        # 物理约束
        v_max = rospy.get_param('~v_max', 1.0)
        v_min = rospy.get_param('~v_min', 0.0)
        omega_max = rospy.get_param('~omega_max', 1.0)
        kappa_max = rospy.get_param('~kappa_max', 0.77)

        # 算法参数
        gamma = rospy.get_param('~gamma', 0.5)
        alpha = rospy.get_param('~alpha', 0.6)
        beta = rospy.get_param('~beta', 0.4)
        mu = rospy.get_param('~mu', 4.0)
        kv = rospy.get_param('~kv', 1.0)
        r = rospy.get_param('~r', None)  # None = 自动计算

        # 目标配置
        target_x = rospy.get_param('~target_x', 2.0)
        target_y = rospy.get_param('~target_y', 0.0)
        target_theta = rospy.get_param('~target_theta', math.pi)
        target_vd = rospy.get_param('~target_vd', 0.0)

        # 车辆参数
        self.wheelbase = rospy.get_param('~wheelbase', 0.65)
        self.max_steer_deg = rospy.get_param('~max_steer_deg', 30.0)
        self.control_hz = rospy.get_param('~control_hz', 20.0)
        self.enable_viz = rospy.get_param('~enable_visualization', True)
        # 可视化参考坐标系（Fixed Frame），可通过参数覆盖
        # 默认为 'odom'，也会在接收到里程计后用里程计 header.frame_id 覆盖实际轨迹的 frame
        self.vis_frame = rospy.get_param('~visualization_frame', 'odom')

        # ── 初始化 FT-C2VF 规划器 ──
        self.planner = FTC2VFPlanner(
            kappa_max=kappa_max, gamma=gamma, alpha=alpha, beta=beta,
            mu=mu, kv=kv, v_max=v_max, v_min=v_min, omega_max=omega_max, r=r,
        )
        self.planner.set_target(target_x, target_y, target_theta, target_vd)

        # 参数验证
        for w in self.planner._validate_params():
            rospy.logwarn("FT-C2VF: %s", w)

        # ── Publishers ──
        self.cmd_pub = rospy.Publisher(self.cmd_topic, ctrl_cmd, queue_size=1)

        # 诊断话题 (命名空间 /ft_c2vf/)
        self.diag_pub = {
            'theta_e': rospy.Publisher(
                '/ft_c2vf/theta_e', Float32, queue_size=5),
            'eta': rospy.Publisher(
                '/ft_c2vf/eta', Float32, queue_size=5),
            'kappa_c': rospy.Publisher(
                '/ft_c2vf/kappa_c', Float32, queue_size=5),
            'v': rospy.Publisher(
                '/ft_c2vf/v', Float32, queue_size=5),
            'omega': rospy.Publisher(
                '/ft_c2vf/omega', Float32, queue_size=5),
            'kappa_chi': rospy.Publisher(
                '/ft_c2vf/kappa_chi', Float32, queue_size=5),
        }

        # 可视化话题 (/vis/)
        if self.enable_viz:
            self.vis_actual_path_pub = rospy.Publisher(
                '/vis/actual_path', Path, queue_size=1)
            self.vis_manifold_pub = rospy.Publisher(
                '/vis/desired_manifold', Path, queue_size=1, latch=True)
            self.vis_ref_path_pub = rospy.Publisher(
                '/vis/ref_trajectory', Path, queue_size=1, latch=True)
            self.vis_target_pub = rospy.Publisher(
                '/vis/target_pose', Marker, queue_size=1, latch=True)

        # ── Subscribers ──
        rospy.Subscriber(self.odom_topic, Odometry,
                         self._odom_callback, queue_size=10)
        rospy.Subscriber(self.ctrl_fb_topic, ctrl_fb,
                         self._ctrl_fb_callback, queue_size=10)

        # ── 状态 ──
        self.lock = threading.Lock()
        self.latest_x = 0.0
        self.latest_y = 0.0
        self.latest_theta = 0.0
        self.latest_linear_v = 0.0
        self.latest_steer_fb = 0.0
        self.received_odom = False

        # 轨迹记录
        self.actual_path_msg = Path()
        self.actual_path_msg.header.frame_id = self.vis_frame

        # 控制诊断
        self.control_count = 0
        self.last_cmd_time = None

        # 收敛监测
        self.convergence_history = deque(maxlen=50)

        # ── 初始化可视化 ──
        if self.enable_viz:
            self._publish_static_visualizations()

        # ── 控制定时器 ──
        rospy.Timer(rospy.Duration(1.0 / self.control_hz),
                     self._control_timer_cb)

        # 打印启动信息
        rospy.loginfo("=" * 60)
        rospy.loginfo("FT-C2VF Controller Ready")
        rospy.loginfo("  Target: (%.2f, %.2f, %.2f rad), vd=%.2f",
                      target_x, target_y, target_theta, target_vd)
        rospy.loginfo("  Manifold: center=(%.2f, %.2f), r=%.2f",
                      self.planner.cx, self.planner.cy, self.planner.r)
        rospy.loginfo("  Constraints: v∈[%.2f,%.2f], |ω|≤%.2f, |κ|≤%.2f",
                      v_min, v_max, omega_max, kappa_max)
        rospy.loginfo("  Params: γ=%.2f, α=%.2f, β=%.2f, μ=%.1f, kv=%.1f",
                      gamma, alpha, beta, mu, kv)
        rospy.loginfo("  Odom: %s", self.odom_topic)
        rospy.loginfo("  Control: %s @ %.1f Hz",
                      self.cmd_topic, self.control_hz)
        rospy.loginfo("=" * 60)

    # ──────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────

    def _odom_callback(self, msg):
        """里程计回调: 提取 [x, y, θ]"""
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        quat = (q.x, q.y, q.z, q.w)
        yaw = tf.transformations.euler_from_quaternion(quat)[2]

        v_linear = msg.twist.twist.linear.x

        # 记录路径
        if self.enable_viz:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose
            self.actual_path_msg.poses.append(pose_stamped)
            # 确保 Path.header.frame_id 与里程计使用的 frame 一致，避免 RViz 中的 frame mismatch
            # 如果里程计使用了不同的 frame（例如 'base_link'），将 actual_path 的 frame 与之保持一致
            try:
                self.actual_path_msg.header.frame_id = msg.header.frame_id
            except Exception:
                # 保持默认 self.vis_frame
                pass
            self.actual_path_msg.header.stamp = rospy.Time.now()

        with self.lock:
            self.latest_x = float(ox)
            self.latest_y = float(oy)
            self.latest_theta = float(yaw)
            self.latest_linear_v = v_linear
            self.received_odom = True

    def _ctrl_fb_callback(self, msg):
        """控制反馈回调: 记录实际执行转向角 (可选监控)"""
        with self.lock:
            self.latest_steer_fb = float(msg.ctrl_fb_steering)

    # ──────────────────────────────────────────────────────────────
    # 控制定时器
    # ──────────────────────────────────────────────────────────────

    def _control_timer_cb(self, event):
        """控制定时器回调: 核心控制循环"""
        with self.lock:
            if not self.received_odom:
                return
            x = self.latest_x
            y = self.latest_y
            theta = self.latest_theta

        # 计算控制指令
        v, omega, debug = self.planner.compute_control(x, y, theta)
        self.control_count += 1

        # ── 将 [v, ω] 转为 ctrl_cmd 消息 ──
        # ω → steering angle (阿克曼转向): δ = atan(ω·L / v)
        if abs(v) > 1e-6:
            steer_rad = math.atan2(omega * self.wheelbase, v)
        else:
            steer_rad = 0.0
        steer_deg = math.degrees(steer_rad)
        steer_deg = np.clip(steer_deg, -self.max_steer_deg,
                            self.max_steer_deg)

        cmd_msg = ctrl_cmd()
        cmd_msg.ctrl_cmd_gear = 4            # 前进档
        cmd_msg.ctrl_cmd_velocity = float(v)
        cmd_msg.ctrl_cmd_steering = float(steer_deg)
        cmd_msg.ctrl_cmd_Brake = 0

        self.cmd_pub.publish(cmd_msg)

        # ── 发布诊断数据 ──
        self.diag_pub['theta_e'].publish(
            Float32(data=float(debug['theta_e'])))
        self.diag_pub['eta'].publish(Float32(data=float(debug['eta'])))
        self.diag_pub['kappa_c'].publish(
            Float32(data=float(debug['kappa_c'])))
        self.diag_pub['v'].publish(Float32(data=float(v)))
        self.diag_pub['omega'].publish(Float32(data=float(omega)))
        self.diag_pub['kappa_chi'].publish(
            Float32(data=float(debug['kappa_chi'])))

        # ── 发布可视化路径 ──
        if self.enable_viz and self.control_count % 5 == 0:
            self.vis_actual_path_pub.publish(self.actual_path_msg)

        # ── 收敛监测 ──
        self.convergence_history.append(abs(debug['theta_e']))
        if (self.control_count % (int(self.control_hz) * 5) == 0
                and len(self.convergence_history) >= 20):
            avg_te = sum(list(self.convergence_history)[-20:]) / 20.0
            dist = debug['dist_to_target']
            rospy.loginfo(
                "FT-C2VF | t=%.1fs | avg|θe|=%.4f rad | dist=%.3f m | "
                "v=%.3f | ω=%.3f | κc=%.3f",
                rospy.Time.now().to_sec(), avg_te, dist,
                v, omega, debug['kappa_c']
            )

    # ──────────────────────────────────────────────────────────────
    # 静态可视化
    # ──────────────────────────────────────────────────────────────

    def _publish_static_visualizations(self):
        """发布不随时间变化的静态可视化数据 (latch=True)"""
        header = rospy.Header()
        header.frame_id = "odom"
        header.stamp = rospy.Time.now()

        # 1. 期望圆环 Mr
        manifold_msg = Path()
        manifold_msg.header = header
        for (px, py) in self.planner.get_desired_manifold_points():
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation.w = 1.0
            manifold_msg.poses.append(pose)
        self.vis_manifold_pub.publish(manifold_msg)

        # 2. 参考向量场流线 (从多个起点积分)
        ref_msg = Path()
        ref_msg.header = header
        r_offsets = [0.3, 1.5, 3.0]
        angles = [0.0, math.pi / 2, math.pi, -math.pi / 2]
        for r_off in r_offsets:
            for ang in angles:
                sx = self.planner.cx + r_off * self.planner.r * math.cos(ang)
                sy = self.planner.cy + r_off * self.planner.r * math.sin(ang)
                traj = self.planner.integrate_trajectory(
                    sx, sy, steps=300, dt=0.05)
                for (px, py) in traj:
                    pose = PoseStamped()
                    pose.header = header
                    pose.pose.position.x = px
                    pose.pose.position.y = py
                    pose.pose.orientation.w = 1.0
                    ref_msg.poses.append(pose)
        self.vis_ref_path_pub.publish(ref_msg)

        # 3. 目标配置 ARROW Marker
        target_msg = Marker()
        target_msg.header = header
        target_msg.ns = "target"
        target_msg.id = 0
        target_msg.type = Marker.ARROW
        target_msg.action = Marker.ADD
        target_msg.pose.position.x = self.planner.xd
        target_msg.pose.position.y = self.planner.yd
        target_msg.pose.position.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, self.planner.thetad)
        target_msg.pose.orientation.x = q[0]
        target_msg.pose.orientation.y = q[1]
        target_msg.pose.orientation.z = q[2]
        target_msg.pose.orientation.w = q[3]
        target_msg.scale.x = 1.0  # 箭头长度
        target_msg.scale.y = 0.15 # 箭头宽度
        target_msg.scale.z = 0.15 # 箭头高度
        target_msg.color.r = 1.0  # 黄色
        target_msg.color.g = 1.0
        target_msg.color.b = 0.0
        target_msg.color.a = 1.0
        
        self.vis_target_pub.publish(target_msg)

        rospy.loginfo("Static visualizations published (latch=True)")

    # ──────────────────────────────────────────────────────────────
    # 主循环
    # ──────────────────────────────────────────────────────────────

    def spin(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass


# ╔══════════════════════════════════════════════════════════════════════╗
# ║                    3. 离线测试模式 (--test)                          ║
# ╚══════════════════════════════════════════════════════════════════════╝

def run_offline_test():
    """
    离线测试 FT-C2VF 算法核心，不依赖 ROS。
    使用与 MATLAB simulation_fig1_v7.m 相同的参数和初值,
    验证算法的正确性和约束满足情况。
    """
    print("=" * 65)
    print("  FT-C2VF 离线测试 (与 MATLAB simulation_fig1_v7.m 对齐)")
    print("=" * 65)

    planner = FTC2VFPlanner(
        kappa_max=1.0, gamma=0.5, alpha=0.6, beta=0.4,
        mu=4.0, kv=1.0, v_max=1.0, v_min=0.0, omega_max=1.0,
    )

    # 测试场景 (论文 Group 2, Rob. 5-8)
    sqrt2 = math.sqrt(2.0)
    test_cases = [
        {'name': 'Rob.5', 'init': (0.8, 0.7, -5*math.pi/6),
         'target': (-2*sqrt2, 2*sqrt2, -3*math.pi/4)},
        {'name': 'Rob.6', 'init': (-0.5, 0.7, -math.pi/5),
         'target': (-2*sqrt2, -2*sqrt2, -math.pi/4)},
        {'name': 'Rob.7', 'init': (-1.2, -0.9, math.pi/4),
         'target': (2*sqrt2, -2*sqrt2, math.pi/4)},
        {'name': 'Rob.8', 'init': (0.4, -0.7, 5*math.pi/8),
         'target': (2*sqrt2, 2*sqrt2, 3*math.pi/4)},
    ]

    dt = 0.005
    T_end = 30.0
    total_steps = int(T_end / dt)

    all_cases_pass = True

    for tc in test_cases:
        planner.set_target(*tc['target'])
        x, y, theta = tc['init']

        converged_at = None
        heading_aligned_at = None

        # 记录用于约束验证
        kc_history = []
        omega_history = []
        v_history = []

        for step in range(total_steps):
            t = step * dt
            v, omega, debug = planner.compute_control(x, y, theta)

            # 欧拉积分
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += omega * dt
            theta = math.atan2(math.sin(theta), math.cos(theta))

            kc_history.append(debug['kappa_c'])
            omega_history.append(omega)
            v_history.append(v)

            if heading_aligned_at is None and abs(debug['theta_e']) < 0.01:
                heading_aligned_at = t

            if converged_at is None and debug['psi'] < 1e-6:
                converged_at = t

        final_dist = math.sqrt((x - planner.xd)**2 + (y - planner.yd)**2)
        final_te = abs(math.atan2(
            math.sin(theta - planner.thetad),
            math.cos(theta - planner.thetad)))

        max_abs_kc = max(abs(k) for k in kc_history)
        max_abs_omega = max(abs(w) for w in omega_history)
        min_v = min(v_history)
        max_v = max(v_history)

        print("\n%s:" % tc['name'])
        print("  朝向对齐时间 Tc: %s" % (
            "%.2f s" % heading_aligned_at if heading_aligned_at else "未对齐"))
        print("  目标到达时间 Ta: %s" % (
            "%.2f s" % converged_at if converged_at else "未到达"))
        print("  最终位置误差: %.4f m" % final_dist)
        print("  最终朝向误差: %.4f rad" % final_te)

        # 约束检查 (上界用 <=, 下界用 >=)
        upper_checks = [
            ("|κc| ≤ κ̄=1.0", max_abs_kc, 1.01),
            ("|ω| ≤ ω̄=1.0", max_abs_omega, 1.01),
            ("v ≤ v_max=1.0", max_v, 1.01),
        ]
        for name, actual, limit in upper_checks:
            passed = actual <= limit
            status = "PASS" if passed else "FAIL"
            if not passed:
                all_cases_pass = False
            print("  [%s] %s (%.4f)" % (status, name, actual))

        # 下界检查
        passed_lower = min_v >= -0.01
        status = "PASS" if passed_lower else "FAIL"
        if not passed_lower:
            all_cases_pass = False
        print("  [%s] v ≥ v_min=0.0 (min=%.4f)" % (status, min_v))

    print("\n" + "=" * 65)
    if all_cases_pass:
        print("  全部测试通过。所有约束满足，可在 ROS 中使用。")
    else:
        print("  *** 存在约束违反! 请检查参数设置 ***")
    print("=" * 65)


# ╔══════════════════════════════════════════════════════════════════════╗
# ║                           入口                                      ║
# ╚══════════════════════════════════════════════════════════════════════╝

if __name__ == '__main__':
    if '--test' in sys.argv:
        run_offline_test()
    elif not ROS_AVAILABLE:
        print("ERROR: ROS 环境不可用。")
        print("  - 使用 --test 进行离线测试:")
        print("    python ft_c2vf_controller.py --test")
        print("  - 或 source ROS 环境后通过 rosrun 启动")
        sys.exit(1)
    else:
        try:
            node = FTC2VFControllerNode()
            node.spin()
        except rospy.ROSInterruptException:
            pass
