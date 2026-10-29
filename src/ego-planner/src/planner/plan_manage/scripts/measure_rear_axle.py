#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
measure_rear_axle.py —— 实测「定位参考点到后轴中心的距离」

原理
----
后轴中心是阿克曼车辆上**横向速度恒为零**的那个点。
若被跟踪的点 P 位于后轴前方 d 处, 刚体运动学给出(车体系):
    v_P = v_rear + omega × r,   r = (d, 0, 0)
    => v_P = (v, omega*d, 0)
即 P 的**横向速度分量 = d * omega**。

所以只要让车走一段稳定转弯的弧线, 同时记录里程计, 就能拟合出 d:
    v_lat = d * omega        ->  最小二乘  d = Σ(v_lat*omega) / Σ(omega^2)
其中 v_lat / omega 全部由里程计自身算出, 不需要任何外部测量。

用法
----
  1. LIO-SAM 正常运行
  2. rosrun ego_planner measure_rear_axle.py
  3. 按提示让车**打死一侧方向、稳定走一段弧**(3~5 秒即可, 低速无所谓),
     最好左右各走一段以抵消系统偏差
  4. 回车结束, 脚本给出 d, 直接填进 real_vehicle_params.yaml 的 rear_axle_x

注意
----
本脚本订阅的是 **LIO-SAM 原始话题**, 量出来的 d 是「雷达安装点到后轴」的距离。
参数 rear_axle_x 要填 **-d** (车体系, 负值表示后轴在参考点后方)。
"""

import math
import sys

import numpy as np
import rospy
import tf.transformations as tft
from nav_msgs.msg import Odometry


class RearAxleMeasurer(object):
    def __init__(self):
        self.topic = rospy.get_param('~odom_topic', '/lio_sam/mapping/odometry')
        # 车头方向在雷达系里的朝向, 必须与 real_vehicle_params.yaml 中一致
        self.yaw_off = math.radians(float(rospy.get_param('~lidar_yaw_offset_deg', -90.0)))
        # 只用转弯足够明显的样本, 否则 omega≈0 时 d 无法辨识(除以接近 0 的数)
        self.min_omega = float(rospy.get_param('~min_omega', 0.05))   # rad/s
        self.min_speed = float(rospy.get_param('~min_speed', 0.05))   # m/s

        self.samples = []   # (t, x, y, yaw_vehicle)
        rospy.Subscriber(self.topic, Odometry, self.cb, queue_size=100)

    def cb(self, msg):
        q = msg.pose.pose.orientation
        yaw_lidar = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        yaw_veh = yaw_lidar + self.yaw_off     # 雷达系 -> 车体系
        self.samples.append((msg.header.stamp.to_sec(),
                             msg.pose.pose.position.x,
                             msg.pose.pose.position.y,
                             yaw_veh))

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    def report(self):
        n = len(self.samples)
        if n < 10:
            print('\n[失败] 只收到 %d 帧 %s, 数据太少。LIO-SAM 在跑吗?' % (n, self.topic))
            return

        s = self.samples
        lat, om, lon = [], [], []
        for i in range(1, n):
            t0, x0, y0, th0 = s[i - 1]
            t1, x1, y1, th1 = s[i]
            dt = t1 - t0
            if dt <= 1e-3:
                continue
            # 世界系速度
            vx, vy = (x1 - x0) / dt, (y1 - y0) / dt
            # 取中点姿态, 减小离散化误差
            th = th0 + 0.5 * self._wrap(th1 - th0)
            w = self._wrap(th1 - th0) / dt
            # world -> 车体系
            v_lon = vx * math.cos(th) + vy * math.sin(th)
            v_lat = -vx * math.sin(th) + vy * math.cos(th)
            if abs(w) < self.min_omega or abs(v_lon) < self.min_speed:
                continue
            lat.append(v_lat); om.append(w); lon.append(v_lon)

        if len(om) < 5:
            print('\n[失败] 有效转弯样本只有 %d 个。' % len(om))
            print('       需要车**明显转弯**(|omega| > %.2f rad/s)且**在走**(|v| > %.2f m/s)。'
                  % (self.min_omega, self.min_speed))
            print('       请打方向盘走一段弧线再测。')
            return

        lat = np.array(lat); om = np.array(om); lon = np.array(lon)
        # 最小二乘 v_lat = d * omega (过原点)
        d = float(np.dot(lat, om) / np.dot(om, om))
        resid = lat - d * om
        # 拟合优度: 1 - 残差方差/原方差
        r2 = 1.0 - np.var(resid) / np.var(lat) if np.var(lat) > 1e-12 else 0.0

        print('\n===== 结果 =====')
        print('总帧数 %d, 有效转弯样本 %d' % (n, len(om)))
        print('里程计频率 ≈ %.1f Hz' % ((n - 1) / (s[-1][0] - s[0][0])))
        print('转弯范围 |omega| = %.3f ~ %.3f rad/s, 速度 %.2f ~ %.2f m/s'
              % (np.abs(om).min(), np.abs(om).max(), np.abs(lon).min(), np.abs(lon).max()))
        print('横向速度 v_lat = %.3f ~ %.3f m/s' % (lat.min(), lat.max()))
        print('\n>>> 参考点位于后轴前方  d = %.3f m   (拟合优度 R^2 = %.2f) <<<' % (d, r2))

        if r2 < 0.5:
            print('\n[注意] R^2 偏低, 说明数据里噪声占主导。可能原因:')
            print('       - 转弯幅度太小 -> 打死方向再走一遍')
            print('       - 里程计频率太低/有跳变 -> 走慢一点、走久一点')
            print('       结果仅供参考, 建议重测。')

        print('\n填法: 在 config/real_vehicle_params.yaml 里')
        print('        odom_rear_axle:')
        print('          rear_axle_x: %.2f          # = -d, 负值表示后轴在参考点后方' % (-d))
        if abs(d) < 0.1:
            print('\n结论: d 接近 0, 参考点本来就在后轴附近, 改这个不会明显改善跟踪。')
            print('      跟踪差应另找原因(见 teb_tracking 起始速度恒为 0 的问题)。')
        else:
            err = abs(d) * np.abs(om).max()
            print('\n影响量级: 最大转弯时该点横向速度误差 = d*omega_max = %.2f m/s,' % err)
            print('          与前进速度 %.2f m/s 相比, 运动方向偏离车头 %.1f 度。'
                  % (np.abs(lon).max(), math.degrees(math.atan2(err, np.abs(lon).max()))))


if __name__ == '__main__':
    rospy.init_node('measure_rear_axle')
    m = RearAxleMeasurer()
    print('订阅 %s ...' % m.topic)
    rospy.sleep(1.0)
    if not m.samples:
        print('[警告] 还没收到里程计数据, 确认 LIO-SAM 已启动并初始化完成。')
    print('\n请让车**打一侧方向走一段弧线**(3~5 秒, 低速即可),')
    print('最好左右各走一段。走完回到这里按回车。')
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass
    m.report()
    sys.exit(0)
