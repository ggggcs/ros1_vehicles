#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
capture_reverse.py —— 倒车"退一点就停"诊断

实车做几次倒车机动(倒车-打方向-再倒), 本脚本把 teb_tracking 的内部日志(/rosout)
和 /cmd_vel、reversing/stuck 标志、位姿按时间线录下来, 区分"停"是哪种:

    日志                         含义
    rear obstacle within ..m   后向安全门触发 -> vx 被强制 0 (车后方 0.6m 内有障碍点)
    TEB plan failed -> stop    TEB 优化器找不到可行解 -> vx=0
    STUCK(blocked)             blocked_timeout 3s 到 -> 请求 FSM 重规划
    STUCK(reverse-timeout)     reverse_timeout 8s 到 -> 请求 FSM 重规划
    v=0 但无上述日志             TEB 自己输出的解就是停(可能优化器在局部最优间跳)

用法 (起导航链后):
    rosrun ego_planner capture_reverse.py _sec:=40
结束后把打印的时间线发我。建议: 40s 里做 3~4 次倒车机动。
"""

import csv
import math
import time

import rospy
import tf.transformations as tft
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from std_msgs.msg import Bool

LEVEL = {Log.DEBUG: 'D', Log.INFO: 'I', Log.WARN: 'W', Log.ERROR: 'E', Log.FATAL: 'F'}


class CaptureReverse(object):
    def __init__(self):
        self.sec = float(rospy.get_param('~sec', 40.0))
        self.t0 = time.time()
        self.events = []            # (t, tag, detail)
        self.v = 0.0
        self.w = 0.0
        self.reversing = False
        self.stuck = False
        self.pos = (0.0, 0.0)
        self.yaw = 0.0
        self.last_state = None
        rospy.Subscriber('/cmd_vel', Twist, self.cb_cmd, queue_size=200)
        rospy.Subscriber('/teb_tracking/reversing', Bool, self.cb_rev, queue_size=100)
        rospy.Subscriber('/teb_tracking/stuck', Bool, self.cb_stuck, queue_size=100)
        rospy.Subscriber('/legOdom', Odometry, self.cb_odom, queue_size=200)
        rospy.Subscriber('/rosout', Log, self.cb_rosout, queue_size=500)

    def now(self):
        return time.time() - self.t0

    def cb_cmd(self, m):
        self.v, self.w = m.linear.x, m.angular.z
        # 状态迁移检测: 前进->倒车 / 倒车->停 / 倒车恢复
        st = ('rev' if self.v < -0.02 else ('fwd' if self.v > 0.02 else 'zero'))
        if st != self.last_state:
            self.events.append((self.now(), 'cmd->%s' % st, 'v=%.2f w=%.3f' % (self.v, self.w)))
            self.last_state = st

    def cb_rev(self, m):
        if m.data != self.reversing:
            self.reversing = m.data
            self.events.append((self.now(), 'reversing=%s' % m.data, ''))

    def cb_stuck(self, m):
        if m.data != self.stuck:
            self.stuck = m.data
            self.events.append((self.now(), 'STUCK=%s' % m.data, ''))

    def cb_odom(self, m):
        self.pos = (m.pose.pose.position.x, m.pose.pose.position.y)
        q = m.pose.pose.orientation
        self.yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def cb_rosout(self, m):
        if 'teb_tracking' not in m.name:
            return
        # 只记录有诊断价值的信息: WARN/ERROR 全部, INFO 里 v/w 和 nearest
        if m.level >= Log.WARN or 'v=' in m.msg or 'nearest=' in m.msg:
            self.events.append((self.now(), 'log[%s]' % LEVEL.get(m.level, '?'),
                                m.msg.strip()[:130]))

    def run(self):
        rate = rospy.Rate(50)
        last_sample = 0.0
        while not rospy.is_shutdown() and self.now() < self.sec:
            # 每 2s 记一个位姿样点
            if self.now() - last_sample >= 2.0:
                last_sample = self.now()
                self.events.append((self.now(), 'odom', 'pos=(%.2f,%.2f) yaw=%.1f'
                                   % (self.pos[0], self.pos[1], math.degrees(self.yaw))))
            rate.sleep()
        self.report()

    def report(self):
        ev = sorted(self.events, key=lambda e: e[0])
        print('\n' + '=' * 78)
        print('倒车机动时间线  (%.0fs)' % self.sec)
        print('=' * 78)
        for (t, tag, detail) in ev:
            print('%6.1fs  %-18s  %s' % (t, tag, detail))
        print('')


if __name__ == '__main__':
    rospy.init_node('capture_reverse', anonymous=True)
    CaptureReverse().run()
