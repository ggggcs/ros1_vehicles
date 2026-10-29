#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
capture_tracking.py —— 抓取跟踪抖动数据, 用于区分「画龙/摆轮」是指令侧还是执行侧。

用法 (实车沿直线走 5~10s, 最好是包含一次倒车的完整往返):
    rosrun ego_planner capture_tracking.py _sec:=10
    # 结束后在终端看统计, 数据存到 /tmp/tracking_capture.csv

输出的关键指标:
    steer_cmd 零交叉率 (次/秒) : 转角指令跨过 0 的频率 —— 高 = 控制器在零位附近左右摆
    steer_fb  零交叉率        : 实际前轮跨过 0 的频率 —— 高 = 底盘在摆
    |steer_cmd| 均值 / 峰值   : 指令振幅
    判断:
        steer_cmd 零交叉率高 且 |cmd| 小(±2°内) -> 控制器抖动, 可调
            (TEB weight_viapoint 调低 / lookahead_dist 调大 / 或加转向死区)
        steer_cmd 稳(≈0) 但 steer_fb 在摆        -> 底盘死区/迟滞, 软件无解
        w_cmd 抖且与 steer 同步                  -> TEB 起始速度噪声, 查 /ctrl_fb
"""

import csv
import math
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from yhs_can_msgs.msg import ctrl_cmd, ctrl_fb


class Capture(object):
    def __init__(self):
        self.sec = float(rospy.get_param('~sec', 10.0))
        self.t0 = None
        self.rows = []          # (t, cmd_v, cmd_w, steer_cmd, fb_v, fb_steer, odom_w)
        rospy.Subscriber('/cmd_vel', Twist, self.cb_cmd, queue_size=200)
        rospy.Subscriber('/ctrl_cmd', ctrl_cmd, self.cb_ctrl, queue_size=200)
        rospy.Subscriber('/ctrl_fb', ctrl_fb, self.cb_fb, queue_size=200)
        rospy.Subscriber('/legOdom', Odometry, self.cb_odom, queue_size=200)
        # 每个话题缓存最近一条, cb 主循环统一取, 保证对齐
        self._cmd = (0.0, 0.0)
        self._steer = 0.0
        self._fb = (0.0, 0.0)
        self._ow = 0.0

    def cb_cmd(self, m):
        self._cmd = (m.linear.x, m.angular.z)

    def cb_ctrl(self, m):
        self._steer = m.ctrl_cmd_steering

    def cb_fb(self, m):
        self._fb = (m.ctrl_fb_velocity, m.ctrl_fb_steering)

    def cb_odom(self, m):
        self._ow = m.twist.twist.angular.z

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.t0 is None:
                self.t0 = rospy.Time.now()
            t = (rospy.Time.now() - self.t0).to_sec()
            self.rows.append((t, self._cmd[0], self._cmd[1], self._steer,
                              self._fb[0], self._fb[1], self._ow))
            if t >= self.sec:
                break
            rate.sleep()
        self.report()

    def zc_rate(self, a):
        """零交叉率: 符号变化次数 / 时长"""
        s = np.sign(a)
        n = int(np.sum(s[1:] != s[:-1]))
        return n / self.sec

    def report(self):
        a = np.array(self.rows)
        t, cmd_v, cmd_w, steer, fb_v, fb_s, ow = a.T
        with open('/tmp/tracking_capture.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'cmd_v', 'cmd_w', 'steer_cmd', 'fb_v', 'fb_steer', 'odom_w'])
            w.writerows(self.rows)
        print('\n' + '=' * 64)
        print('实车跟踪抖动采集  %.0f s  (已存 /tmp/tracking_capture.csv)' % self.sec)
        print('=' * 64)
        print('%-24s %10s %10s %10s' % ('', '均值|x|', '峰值', '零交叉/秒'))
        print('-' * 64)
        for name, x in [('cmd_w (TEB omega)', cmd_w),
                        ('steer_cmd (指令角)', steer),
                        ('steer_fb (实际角)', fb_s),
                        ('odom_w (实测 yaw率)', ow)]:
            print('%-24s %10.3f %10.3f %10.2f' % (name, np.mean(np.abs(x)), np.max(np.abs(x)),
                                                 self.zc_rate(x)))
        # 只看运动段(速度>0.05)
        mov = np.abs(cmd_v) > 0.05
        if mov.sum() > 10:
            print('\n>>> 运动段(共 %.1fs):' % (mov.sum() / 50.0))
            sc = steer[mov]
            print('    steer_cmd 零交叉 %.2f 次/s, 振幅 |x|%.3f / max %.3f deg'
                  % (self.zc_rate(sc), np.mean(np.abs(sc)), np.max(np.abs(sc))))
            if self.zc_rate(sc) > 1.0 and np.max(np.abs(sc)) < 3.0:
                print('    -> 指令侧在零位附近抖动(控制器), 可调: weight_viapoint↓ / lookahead_dist↑ / 转向死区')
            elif self.zc_rate(sc) <= 0.5:
                print('    -> 指令基本稳定; 若前轮仍肉眼摆, 多半是底盘死区/迟滞(执行侧)')
        print('')


if __name__ == '__main__':
    rospy.init_node('capture_tracking', anonymous=True)
    Capture().run()
