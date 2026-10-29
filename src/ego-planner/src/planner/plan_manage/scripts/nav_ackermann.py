#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nav_ackermann.py —— ego-planner/teb_tracking 的实车底盘桥接节点

    /cmd_vel (geometry_msgs/Twist)  ->  /ctrl_cmd (yhs_can_msgs/ctrl_cmd)

与 racecar_description/scripts/nav_sim.py 的区别(务必分清, 两者不能混用)
--------------------------------------------------------------------
nav_sim.py:  把 Twist.angular.z 当作**前轮转向角(rad)**直接转成度, 且写死 gear=4。
             它配的是 ft_c2vf / move_base+TEB(cmd_angle_instead_rotvel:=true) 那种
             「已经在上游算好转向角」的规划器。
本节点:      把 Twist.angular.z 当作**角速度 omega(rad/s)**, 因为 teb_tracking.cpp
             的 publishCmd() 发的就是 (v, w):
                 msg.linear.x = sm_v;  msg.angular.z = sm_w;
             所以这里必须自己按自行车模型解算前轮转向角。
             直接把 omega 当角度发给底盘会得到完全错误的转向量。

转向角公式取决于「定位参考点在哪个轴上」(ref_point 参数)
-------------------------------------------------------
自行车模型里前后轴两个参考点的公式**不一样**, 用错会有系统性偏差:

  ref_point = rear_axle   参考点在后轴中心
      后轴点速度 v_r 沿车体 x 轴, 横摆 w = v_r*tan(delta)/L
      ->  delta = atan(w * L / v)
      最小转弯半径 R = L/tan(delta_max) = 0.66/tan30 = 1.14 m

  ref_point = front_axle  参考点在前轴中心 (**本车默认**)
      前轴点速度 v_f 沿**前轮**方向(车体 x 轴转过 delta), 横摆 w = v_f*sin(delta)/L
      ->  delta = asin(w * L / v)
      最小转弯半径 R = L/sin(delta_max) = 0.66/sin30 = 1.32 m

本车雷达正好装在前轴正上方(后轴在雷达后 0.66m = 轴距), 而 odom_rear_axle.py
默认不平移参考点(rear_axle_x=0), 所以 EGO / grid_map / TEB 的参考点都是前轴,
必须用 asin。用 atan 会**系统性少打方向**: 例如 w*L/v = 0.5 时
    asin -> 30.0 deg (正确)      atan -> 26.6 deg (少 3.4 deg)
转弯偏晚、切内道。

用 asin 换算后精度如何: 车实际画出的**前轴轨迹半径 = v/w**, 正是 TEB 规划的
半径; **横摆角速度也精确等于 w**。唯一残差是前轴点的瞬时速度方向是 theta+delta
而非 theta(TEB 的 carlike 约束假设沿 theta), 满打方向时每个 50ms 周期约 7.5mm,
而 TEB 每周期从实测位姿重解, 不累积。

两处必须跟着一起改的参数(否则 TEB 会规划出底盘打不出来的弯):
    teb_tracking.teb/min_turning_radius  前轴 1.32 -> 填 1.4 留余量
    tuning_base.min_turning_radius       同上

底盘约束(来源: 程控说明.txt + drivers/yhs_can_control/src/yhs_can_control.cpp)
---------------------------------------------------------------------------
  * ctrl_cmd_velocity 必须 >= 0。驱动第 92 行 `if(velocity < 0) vel = 0;`
    会把负速度直接钳成 0 —— 所以**倒车不能靠负速度, 只能靠挡位**。
  * ctrl_cmd_gear: 4 = 前进挡(D), 2 = 倒挡(R)
  * ctrl_cmd_steering: 前轮转向角, 单位**度**, 范围 -30 ~ +30, 负数向右转
    (与 ROS 约定一致: 正 = 左转)
  * ctrl_cmd_velocity 量程 0 ~ 2 m/s

安全设计
--------
  * 定频发布(默认 50Hz), 不依赖 /cmd_vel 的到达节奏; teb_tracking 只有 20Hz。
  * 看门狗: 超过 cmd_timeout 收不到 /cmd_vel 就发 0 速(保持当前挡位与转角)。
  * 换向停顿: 前进<->倒车切换时先发 0 速 direction_switch_dwell 秒, 等挡位啮合,
    避免带速换挡冲击传动。
  * 低速保持转角: |v| 很小时 w*L/v 会发散并打满方向, 此时保持上一次转角不动。
  * asin 定义域保护: |w*L/v| > 1 表示要求的半径小于 L(物理上不可能), 钳到 ±1
    即满舵, 再由 max_steer_deg 限幅。
  * 转角限速: 保护转向机构, 默认 120 deg/s(远高于正常需求, 不干扰跟踪)。
  * 退出时连发 0 速。
"""

import math

import rospy
from geometry_msgs.msg import Twist
from yhs_can_msgs.msg import ctrl_cmd

# 底盘硬限制, 与 params 无关的兜底
CHASSIS_MAX_SPEED = 2.0     # m/s, 程控说明标注量程
CHASSIS_MAX_STEER_DEG = 30.0


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class NavAckermann(object):
    def __init__(self):
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.ctrl_cmd_topic = rospy.get_param('~ctrl_cmd_topic', '/ctrl_cmd')

        self.wheelbase = float(rospy.get_param('~wheelbase', 0.66))
        self.max_speed = clamp(float(rospy.get_param('~max_speed', 0.3)),
                               0.0, CHASSIS_MAX_SPEED)
        self.max_steer_deg = clamp(float(rospy.get_param('~max_steer_deg', 30.0)),
                                   0.0, CHASSIS_MAX_STEER_DEG)

        # 定位参考点在哪个轴上, 决定转角公式(见模块头注释)。必须与
        # odom_rear_axle.rear_axle_x 一致: 0 -> front_axle, -0.66 -> rear_axle
        self.ref_point = str(rospy.get_param('~ref_point', 'front_axle')).strip().lower()
        if self.ref_point not in ('front_axle', 'rear_axle'):
            rospy.logwarn('[nav_ackermann] ref_point="%s" 无法识别, 按 front_axle 处理',
                          self.ref_point)
            self.ref_point = 'front_axle'
        self.front_ref = (self.ref_point == 'front_axle')

        self.gear_forward = int(rospy.get_param('~gear_forward', 4))
        self.gear_reverse = int(rospy.get_param('~gear_reverse', 2))

        self.enable_reverse = bool(rospy.get_param('~enable_reverse', True))
        self.dwell = float(rospy.get_param('~direction_switch_dwell', 0.4))
        self.rate_hz = float(rospy.get_param('~publish_rate', 50.0))
        self.cmd_timeout = float(rospy.get_param('~cmd_timeout', 0.5))

        self.v_deadband = float(rospy.get_param('~v_deadband', 0.02))
        # |v| 低于此值不再更新转角(避免 atan(wL/v) 发散打死方向)
        self.v_min_for_steer = float(rospy.get_param('~v_min_for_steer', 0.05))
        self.steer_rate_deg_s = float(rospy.get_param('~steer_rate_deg_s', 120.0))

        # ---- 转向死区(带滞回) —— 治直线摆轮残留 ----
        # 实测(2026-08-10)直线段是 ~0.6Hz 的持续极限环: steer ±10° 正弦、odom_w ±0.09,
        # 环路延迟 402ms(转向伺服 261ms + 转向->偏航 141ms)下连续高增益校正自激。
        # 死区把 |δ|<steer_deadband_deg 的校正压成 0(滞回避免在阈值边沿抖动), 修正变成
        # 有界的阈值脉冲 -> 极限环被打破, 只留死区宽度内的微摆(±1.5° 不可见)。
        # 弯道/倒车时 δ 大, 不触发生效。0 = 关闭。
        self.steer_db_low = float(rospy.get_param('~steer_deadband_deg', 0.0))
        self.steer_db_high = float(rospy.get_param('~steer_deadband_exit_deg',
                                                   max(0.0, self.steer_db_low)))
        if self.steer_db_high < self.steer_db_low:
            self.steer_db_high = self.steer_db_low
        self._in_db = False

        # ---- 运行状态 ----
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = rospy.Time(0)
        self.cur_gear = self.gear_forward
        self.cur_steer_deg = 0.0
        self.dwell_until = rospy.Time(0)
        self.last_tick = None

        self.pub = rospy.Publisher(self.ctrl_cmd_topic, ctrl_cmd, queue_size=1)
        self.sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_cb,
                                    queue_size=1, buff_size=2 ** 20)

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo('[nav_ackermann] %s -> %s', self.cmd_vel_topic, self.ctrl_cmd_topic)
        _rad = math.radians(self.max_steer_deg)
        if self.front_ref:
            rospy.loginfo('[nav_ackermann] angular.z 按**角速度(rad/s)**解释, 参考点=前轴, '
                          'delta = asin(w*L/v), L = %.3f m', self.wheelbase)
            _rmin = self.wheelbase / math.sin(_rad)
        else:
            rospy.loginfo('[nav_ackermann] angular.z 按**角速度(rad/s)**解释, 参考点=后轴, '
                          'delta = atan(w*L/v), L = %.3f m', self.wheelbase)
            _rmin = self.wheelbase / math.tan(_rad)
        rospy.loginfo('[nav_ackermann] 限速 %.2f m/s, 转角上限 %.1f deg '
                      '(最小转弯半径 %.2f m, 请确认 teb/min_turning_radius >= 此值)',
                      self.max_speed, self.max_steer_deg, _rmin)
        rospy.loginfo('[nav_ackermann] 倒车 %s (gear %d=前进 / %d=倒挡, 换向停顿 %.2fs)',
                      '启用' if self.enable_reverse else '禁用',
                      self.gear_forward, self.gear_reverse, self.dwell)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.tick)

    def cmd_cb(self, msg):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self.last_cmd_time = rospy.Time.now()

    # ---------------- 主循环 ----------------
    def tick(self, _evt):
        now = rospy.Time.now()
        dt = 1.0 / self.rate_hz
        if self.last_tick is not None:
            dt = max(1e-3, (now - self.last_tick).to_sec())
        self.last_tick = now

        # 看门狗: 没有新指令 -> 停车, 保持挡位和转角
        if self.last_cmd_time == rospy.Time(0) or \
                (now - self.last_cmd_time).to_sec() > self.cmd_timeout:
            if self.last_cmd_time != rospy.Time(0):
                rospy.logwarn_throttle(2.0,
                                       '[nav_ackermann] %s 超过 %.2fs 无消息, 停车',
                                       self.cmd_vel_topic, self.cmd_timeout)
            self.publish(0.0, self.cur_steer_deg, self.cur_gear)
            return

        v_in = self.cmd_v
        w_in = self.cmd_w

        # ---- 方向判定 ----
        if abs(v_in) < self.v_deadband:
            direction = 0
        else:
            direction = 1 if v_in > 0.0 else -1

        if direction < 0 and not self.enable_reverse:
            rospy.logwarn_throttle(2.0, '[nav_ackermann] 收到倒车指令但 enable_reverse=false, 停车')
            self.publish(0.0, self.cur_steer_deg, self.cur_gear)
            return

        # ---- 挡位切换(带停顿) ----
        want_gear = self.cur_gear
        if direction > 0:
            want_gear = self.gear_forward
        elif direction < 0:
            want_gear = self.gear_reverse

        if want_gear != self.cur_gear:
            rospy.loginfo('[nav_ackermann] 换挡 %d -> %d, 先停 %.2fs',
                          self.cur_gear, want_gear, self.dwell)
            self.cur_gear = want_gear
            self.dwell_until = now + rospy.Duration(self.dwell)

        in_dwell = now < self.dwell_until

        # ---- 转向角: 自行车模型, v 带符号 ----
        if abs(v_in) >= self.v_min_for_steer:
            # 自行车模型。参考点在前轴还是后轴, 公式不同(见模块头注释):
            #   后轴: w = v*tan(delta)/L  ->  delta = atan(w*L/v)
            #   前轴: w = v*sin(delta)/L  ->  delta = asin(w*L/v)
            ratio = w_in * self.wheelbase / v_in
            if self.front_ref:
                # |ratio|>1 表示要求的前轴半径 < L, 物理上不可能 -> 钳成满舵
                delta = math.asin(clamp(ratio, -1.0, 1.0))
            else:
                delta = math.atan(ratio)
            target_steer_deg = clamp(math.degrees(delta),
                                     -self.max_steer_deg, self.max_steer_deg)
        else:
            # 低速/停车: 保持当前转角, 不打死方向
            target_steer_deg = self.cur_steer_deg

        # ---- 转向死区(带滞回) ----
        # 只压零位附近的小指令(直线段极限环的根源), 弯道大转角不受影响。
        if self.steer_db_low > 0.0:
            if abs(target_steer_deg) < self.steer_db_low:
                self._in_db = True
            elif abs(target_steer_deg) > self.steer_db_high:
                self._in_db = False
            if self._in_db:
                target_steer_deg = 0.0

        # 转角限速
        max_step = self.steer_rate_deg_s * dt
        step = clamp(target_steer_deg - self.cur_steer_deg, -max_step, max_step)
        self.cur_steer_deg = clamp(self.cur_steer_deg + step,
                                   -self.max_steer_deg, self.max_steer_deg)

        # ---- 速度: 底盘只收正值, 方向由挡位决定 ----
        speed = 0.0 if (direction == 0 or in_dwell) else min(abs(v_in), self.max_speed)

        self.publish(speed, self.cur_steer_deg, self.cur_gear)

        rospy.loginfo_throttle(
            1.0, '[nav_ackermann] in(v=%.2f w=%.2f) -> gear=%d speed=%.2f steer=%.1fdeg%s',
            v_in, w_in, self.cur_gear, speed, self.cur_steer_deg,
            ' [换挡停顿]' if in_dwell else '')

    def publish(self, speed, steer_deg, gear):
        msg = ctrl_cmd()
        msg.ctrl_cmd_gear = gear
        msg.ctrl_cmd_velocity = clamp(speed, 0.0, CHASSIS_MAX_SPEED)
        msg.ctrl_cmd_steering = clamp(steer_deg, -CHASSIS_MAX_STEER_DEG, CHASSIS_MAX_STEER_DEG)
        msg.ctrl_cmd_Brake = 0
        self.pub.publish(msg)

    def on_shutdown(self):
        try:
            for _ in range(10):
                self.publish(0.0, self.cur_steer_deg, self.cur_gear)
                rospy.sleep(0.02)
            rospy.loginfo('[nav_ackermann] 已发送停车指令')
        except Exception:
            pass


if __name__ == '__main__':
    rospy.init_node('nav_ackermann')
    NavAckermann()
    rospy.spin()
