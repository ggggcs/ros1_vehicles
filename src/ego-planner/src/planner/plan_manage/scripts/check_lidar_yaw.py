#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_lidar_yaw.py —— 实测「雷达坐标系朝向」标定脚本 (首次实车跑之前必须先做这一步)

要解决什么问题
--------------
实车 LIO-SAM 配置是 lidarFrame = baselinkFrame = base_link, 且
LIO-SAM/launch/include/config/robot.urdf.xacro 里 chassis_link -> velodyne 是单位阵,
所以 /lio_sam/mapping/odometry 报出来的 yaw = **激光雷达自身 x 轴的指向**,
不一定是车头指向。

TEB 的阿克曼模型假设「车沿着自己的 +x 轴前进」。如果雷达装歪了(比如 x 轴朝车
左侧、y 轴朝车头), 那么 TEB 认为的车头方向就和真实车头差 90 度 ——
规划出的路径车根本跟不了, 会朝着侧向使劲打方向。这是首跑最大的坑, 必须先测。

怎么用
------
  1. 正常启动 LIO-SAM (roslaunch lio_sam run.launch), 确认 /lio_sam/mapping/odometry 有数据
  2. 跑本脚本:   rosrun ego_planner check_lidar_yaw.py
  3. 按提示, **手动推车 / 遥控让车直线前进 3~5 米**(不要转弯, 不要倒车)
  4. 回车结束, 脚本给出 lidar_yaw_offset_deg

原理
----
车直行时, 世界系下的位移方向 = 真实车头方向。
把它和里程计报的 yaw 相减, 差值就是「车头方向在雷达系里的朝向」, 即 yaw offset:
      lidar_yaw_offset_deg = atan2(dy, dx) - yaw_odom
结果约等于:
        0 度  -> 雷达 +x 朝车头, 不需要任何修正
      +90 度  -> 雷达 +y 朝车头 (你怀疑的这种)
      -90 度  -> 雷达 -y 朝车头
      180 度  -> 雷达装反了, +x 朝车尾
"""

import math
import sys

import rospy
from nav_msgs.msg import Odometry
import tf.transformations as tft


class YawChecker(object):
    def __init__(self):
        self.topic = rospy.get_param('~odom_topic', '/lio_sam/mapping/odometry')
        # 只统计位移足够大的样本, 避免原地噪声主导
        self.min_dist = float(rospy.get_param('~min_dist', 1.0))
        self.samples = []
        self.sub = rospy.Subscriber(self.topic, Odometry, self.cb, queue_size=50)

    def cb(self, msg):
        yaw = tft.euler_from_quaternion([
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        self.samples.append((msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))

    def report(self):
        if len(self.samples) < 2:
            print('\n[失败] 只收到 %d 帧 %s —— LIO-SAM 在跑吗?' % (len(self.samples), self.topic))
            return

        x0, y0, _ = self.samples[0]
        x1, y1, _ = self.samples[-1]
        dx, dy = x1 - x0, y1 - y0
        dist = math.hypot(dx, dy)

        print('\n===== 结果 =====')
        print('采样帧数        : %d' % len(self.samples))
        print('起点 -> 终点    : (%.2f, %.2f) -> (%.2f, %.2f)' % (x0, y0, x1, y1))
        print('直线位移        : %.2f m' % dist)

        if dist < self.min_dist:
            print('\n[失败] 位移只有 %.2f m, 不足 %.1f m, 噪声会淹没结果。请多走几米重测。'
                  % (dist, self.min_dist))
            return

        # 位移方向 = 真实车头方向
        heading_true = math.atan2(dy, dx)
        # 行驶过程中里程计报的平均 yaw (用向量平均, 避免 ±pi 环绕问题)
        sy = sum(math.sin(s[2]) for s in self.samples)
        cy = sum(math.cos(s[2]) for s in self.samples)
        yaw_odom = math.atan2(sy, cy)

        offset = math.degrees(math.atan2(math.sin(heading_true - yaw_odom),
                                         math.cos(heading_true - yaw_odom)))

        print('真实车头方向    : %.1f deg  (由位移方向得到)' % math.degrees(heading_true))
        print('里程计报的 yaw  : %.1f deg  (行驶期间平均)' % math.degrees(yaw_odom))
        print('\n>>> lidar_yaw_offset_deg = %.1f  <<<' % offset)

        # 就近归到 0/90/180/-90
        cands = [0.0, 90.0, 180.0, -90.0]
        near = min(cands, key=lambda c: abs((offset - c + 180) % 360 - 180))
        err = abs((offset - near + 180) % 360 - 180)
        desc = {0.0: '雷达 +x 朝车头 —— 不需要任何修正',
                90.0: '雷达 +y 朝车头 —— 需要 +90 度修正',
                180.0: '雷达 +x 朝车尾(装反) —— 需要 180 度修正',
                -90.0: '雷达 -y 朝车头 —— 需要 -90 度修正'}[near]
        print('最接近的标准角  : %+.0f deg (偏差 %.1f deg)' % (near, err))
        print('含义            : %s' % desc)

        if err > 15.0:
            print('\n[注意] 偏离标准角 %.1f 度, 偏大。可能是没走直线, 或者雷达确实是斜装的。'
                  '建议重测一次确认。' % err)

        if abs(near) < 1e-6:
            print('\n结论: 不用改, real_navigation.launch 保持现状直接跑。')
        else:
            print('\n结论: 必须修正, 否则 TEB 认为的车头方向差 %.0f 度, 车会朝侧向打方向。' % near)
            print('      在 real_navigation.launch 里启用 odom_rear_axle 节点:')
            print('        1) 取消该节点的注释')
            print('        2) lidar_yaw_offset_deg 设为 %.1f' % near)
            print('        3) rear_axle_x 先保持 0.0 (按你的要求仍用雷达位置代替车辆位置)')
            print('        4) 把 arg odom_topic 改成 /legOdom')
            print('        5) teb_tracking_params_real.yaml 里 odom_topic 也改成 /legOdom')


if __name__ == '__main__':
    rospy.init_node('check_lidar_yaw')
    c = YawChecker()
    print('订阅 %s ...' % c.topic)
    rospy.sleep(1.0)
    if not c.samples:
        print('[警告] 还没收到里程计数据, 确认 LIO-SAM 已启动并已初始化。')
    print('\n请让车**直线前进 3~5 米**(不要转弯/倒车), 走完后回到这里按回车。')
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass
    c.report()
    sys.exit(0)
