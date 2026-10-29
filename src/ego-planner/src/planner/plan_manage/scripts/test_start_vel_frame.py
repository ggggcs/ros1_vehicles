#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_start_vel_frame.py —— 复现并验证「EGO 起始速度坐标系」这个 bug

背景
----
ego_replan_fsm.cpp:199-201 把 odom 的 twist.linear 整个当成**世界系速度向量**
存进 odom_vel_, 再作为 B 样条起始速度边界条件。而 odom_rear_axle.py 按 ROS
标准发的是**车体系** twist (teb_tracking 要的正是车体系)。两者混用时 EGO 会
以为车永远朝世界 +X 运动 -> 规划出 S 弯。

本脚本让车"停在原点、车头朝世界 -Y"、目标点在正前方 5m 处 (0,-5),
分别用两种 twist 喂 EGO, 对比规划出来的 B 样条控制点形状:

    模式 body  : twist=(0.3,0,0) 车体系  -> **有 bug 时的输入**
                 EGO 会以为在朝世界 +X 走, 必须先往 +X 出发再拐回来
    模式 world : twist=(0,-0.3,0) 世界系 -> **修好后的输入**
                 与车头方向一致, 应规划出直线

判据: 控制点相对"起点->终点"直线的最大横向偏离。
      前方无障碍时, world 模式应接近 0; body 模式应明显 > 0。

用法 (需要先起 ego_planner_node, 见文件末尾说明)
    rosrun ego_planner test_start_vel_frame.py _mode:=body
    rosrun ego_planner test_start_vel_frame.py _mode:=world
"""

import math
import sys

import numpy as np
import rospy
import tf
import tf.transformations as tft
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

from ego_planner.msg import Bspline

# 车停在原点, 车头朝世界 -Y (yaw = -90 deg)。选这个朝向是因为它与世界 +X 正交,
# 两种 twist 的差异最大, 最容易看出来。
YAW = -math.pi / 2
SPEED = 0.3
GOAL = (0.0, -5.0)          # 车头正前方 5m


class Tester(object):
    def __init__(self, mode):
        self.mode = mode
        self.pub_odom = rospy.Publisher('/legOdom', Odometry, queue_size=10)
        self.pub_odom_w = rospy.Publisher('/legOdom_world', Odometry, queue_size=10)
        self.pub_cloud = rospy.Publisher('/patchworkpp/nonground', PointCloud2, queue_size=10)
        self.pub_wp = rospy.Publisher('/waypoint_generator/waypoints', Path, queue_size=10)
        self.br = tf.TransformBroadcaster()
        self.splines = []
        rospy.Subscriber('/planning/bspline', Bspline, self.bspline_cb, queue_size=50)

    def bspline_cb(self, msg):
        pts = np.array([[p.x, p.y, p.z] for p in msg.pos_pts])
        self.splines.append(pts)

    def make_odom(self, world_twist):
        q = tft.quaternion_from_euler(0, 0, YAW)
        o = Odometry()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.orientation.x, o.pose.pose.orientation.y = q[0], q[1]
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = q[2], q[3]
        if world_twist:
            # 世界系: 车头朝 -Y, 所以速度向量是 (0, -0.3, 0)
            o.twist.twist.linear.x = SPEED * math.cos(YAW)
            o.twist.twist.linear.y = SPEED * math.sin(YAW)
        else:
            # 车体系: 恒为 (v, 0, 0), 与朝向无关 —— 这正是 bug 的来源
            o.twist.twist.linear.x = SPEED
            o.twist.twist.linear.y = 0.0
        return o

    def make_cloud(self):
        """空旷场地: 只在 8m 外放一圈点, 保证 grid_map 有输入但走廊里没有障碍。
        frame_id 用 odom, 这样 grid_map 的 TF 查找是恒等变换。"""
        pts = []
        for a in np.linspace(0, 2 * math.pi, 120, endpoint=False):
            pts.append((8.0 * math.cos(a), 8.0 * math.sin(a), 0.3))
        h = Header(stamp=rospy.Time.now(), frame_id='odom')
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]
        return pc2.create_cloud(h, fields, pts)

    def run(self):
        rate = rospy.Rate(20)
        t0 = rospy.Time.now()
        sent_goal = False
        while not rospy.is_shutdown():
            el = (rospy.Time.now() - t0).to_sec()
            now = rospy.Time.now()

            self.pub_odom.publish(self.make_odom(world_twist=False))   # TEB 那份, 恒车体系
            self.pub_odom_w.publish(self.make_odom(world_twist=(self.mode == 'world')))
            self.pub_cloud.publish(self.make_cloud())
            self.br.sendTransform((0, 0, 0), tft.quaternion_from_euler(0, 0, YAW),
                                  now, 'base_link', 'odom')

            if el > 3.0 and not sent_goal:
                p = Path()
                p.header.stamp = now
                p.header.frame_id = 'odom'
                ps = PoseStamped()
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = GOAL[0], GOAL[1], 0.0
                ps.pose.orientation.w = 1.0
                p.poses.append(ps)
                self.pub_wp.publish(p)
                sent_goal = True
                rospy.loginfo('[test] 目标点已发布 (%.1f, %.1f)', *GOAL)

            if el > 10.0:
                break
            rate.sleep()
        self.report()

    def report(self):
        print('\n' + '=' * 62)
        print('模式 %s   (车头朝世界 -Y, 目标在正前方 5m, 前方无障碍)' % self.mode)
        print('=' * 62)
        if not self.splines:
            print('没收到任何 /planning/bspline —— ego_planner_node 起了吗?')
            return
        print('共收到 %d 条轨迹' % len(self.splines))
        devs = []
        for k, pts in enumerate(self.splines):
            a, b = pts[0, :2], pts[-1, :2]
            d = b - a
            n = np.linalg.norm(d)
            if n < 1e-6:
                continue
            d = d / n
            rel = pts[:, :2] - a
            lat = np.abs(rel[:, 0] * (-d[1]) + rel[:, 1] * d[0])   # 到 a->b 直线的横向距离
            arc = np.sum(np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1))
            devs.append(lat.max())
            print('  轨迹%d: 控制点%2d个  起点(%.2f,%.2f) 终点(%.2f,%.2f)  '
                  '最大横向偏离 %.3f m  控制多边形长/弦长 %.3f'
                  % (k, len(pts), a[0], a[1], b[0], b[1], lat.max(), arc / n))
        if devs:
            print('\n>>> 最大横向偏离: 均值 %.3f m, 最大 %.3f m' % (np.mean(devs), np.max(devs)))
            print('    world 模式应接近 0 (直线); body 模式应明显 > 0 (S 弯)')


if __name__ == '__main__':
    rospy.init_node('test_start_vel_frame')
    mode = rospy.get_param('~mode', 'world')
    if mode not in ('body', 'world'):
        print('mode 必须是 body 或 world')
        sys.exit(1)
    t = Tester(mode)
    rospy.sleep(1.0)
    t.run()

# 起 ego_planner_node 的最小命令 (不要起 teb_tracking/nav_ackermann, 否则车会动):
#   rosparam load $(rospack find ego_planner)/config/real_vehicle_params.yaml
#   rosrun ego_planner ego_planner_node /grid_map/cloud:=/patchworkpp/nonground
