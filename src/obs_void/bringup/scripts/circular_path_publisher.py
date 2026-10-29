#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
圆形路径发布器
生成一条圆形路径并发布到/move_base/NavfnROS/plan话题
用于替代全局规划器(A*)，简化导航架构
"""

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf

class CircularPathPublisher:
    def __init__(self):
        rospy.init_node('circular_path_publisher', anonymous=True)
        
        # ========== 参数配置 ==========
        # 圆形路径参数
        self.center_x = rospy.get_param('~center_x', 0.0)  # 圆心X坐标
        self.center_y = rospy.get_param('~center_y', 0.0)  # 圆心Y坐标
        self.radius = rospy.get_param('~radius', 5.0)      # 圆半径(米)
        self.num_points = rospy.get_param('~num_points', 100)  # 路径点数量
        self.direction = rospy.get_param('~direction', 'ccw')  # 方向: 'cw'(顺时针) 或 'ccw'(逆时针)
        
        # 发布频率
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)  # Hz
        
        # 坐标系
        self.frame_id = rospy.get_param('~frame_id', 'map')
        
        # ========== ROS发布器 ==========
        # 发布到向量场规划器订阅的话题
        self.path_pub = rospy.Publisher('/move_base/NavfnROS/plan', Path, queue_size=1, latch=True)
        
        rospy.loginfo("圆形路径发布器初始化完成")
        rospy.loginfo("参数: 圆心=(%.2f, %.2f), 半径=%.2f, 点数=%d, 方向=%s",
                     self.center_x, self.center_y, self.radius, self.num_points, self.direction)
        
        # 生成并发布路径
        self.generate_and_publish_path()
        
    def generate_circular_path(self):
        """生成圆形路径
        
        Returns:
            Path消息
        """
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time.now()
        
        # 生成圆形路径点
        if self.direction == 'cw':
            # 顺时针: 从0到-2π
            angles = np.linspace(0, -2*np.pi, self.num_points, endpoint=False)
        else:
            # 逆时针: 从0到2π
            angles = np.linspace(0, 2*np.pi, self.num_points, endpoint=False)
        
        for angle in angles:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = rospy.Time.now()
            
            # 位置
            pose.pose.position.x = self.center_x + self.radius * np.cos(angle)
            pose.pose.position.y = self.center_y + self.radius * np.sin(angle)
            pose.pose.position.z = 0.0
            
            # 朝向: 切向方向
            if self.direction == 'cw':
                tangent_angle = angle - np.pi/2  # 顺时针切向
            else:
                tangent_angle = angle + np.pi/2  # 逆时针切向
            
            # 转换为四元数
            quaternion = tf.transformations.quaternion_from_euler(0, 0, tangent_angle)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            
            path.poses.append(pose)
        
        # 闭合路径: 添加起点作为终点
        if len(path.poses) > 0:
            path.poses.append(path.poses[0])
        
        rospy.loginfo("生成圆形路径: %d 个点", len(path.poses))
        
        return path
    
    def generate_and_publish_path(self):
        """生成并持续发布路径"""
        rate = rospy.Rate(self.publish_rate)
        
        # 生成路径
        path = self.generate_circular_path()
        
        # 持续发布
        while not rospy.is_shutdown():
            # 更新时间戳
            path.header.stamp = rospy.Time.now()
            for pose in path.poses:
                pose.header.stamp = rospy.Time.now()
            
            # 发布
            self.path_pub.publish(path)
            rospy.loginfo_throttle(5.0, "发布圆形路径 (半径=%.2fm, %d点)", 
                                  self.radius, len(path.poses))
            
            rate.sleep()

def main():
    try:
        publisher = CircularPathPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("圆形路径发布器节点关闭")

if __name__ == '__main__':
    main()
