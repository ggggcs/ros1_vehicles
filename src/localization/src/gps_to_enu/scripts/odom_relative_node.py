#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
import tf
import tf2_ros

class OdomRelative:
    def __init__(self):
        rospy.init_node('odom_relative_node', anonymous=True)
        
        # 原点标志和存储
        self.origin_set = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_z = 0.0
        
        # 行驶轨迹存储
        self.trajectory = Path()
        self.trajectory.header.frame_id = "odom"
        self.last_pose_time = rospy.Time(0)
        self.pose_interval = 0.1  # 每0.1秒记录一次位置（10Hz）
        
        # 订阅原始odom话题 (GPS绝对坐标)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # 发布相对odom话题
        self.relative_odom_pub = rospy.Publisher('/AIMhc_UGV1/odom', Odometry, queue_size=10)
        
        # 发布行驶轨迹
        self.trajectory_pub = rospy.Publisher('/odom_trajectory', Path, queue_size=1)
        
        # TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        rospy.loginfo("Odom relative converter initialized")
        rospy.loginfo("Publishing trajectory to /odom_trajectory")
        rospy.loginfo("Waiting for first odom message to set origin...")
    
    def odom_callback(self, msg):
        """处理odom消息"""
        # 如果还没有设置原点，使用第一个odom位置作为原点
        if not self.origin_set:
            self.origin_x = msg.pose.pose.position.x
            self.origin_y = msg.pose.pose.position.y
            self.origin_z = msg.pose.pose.position.z
            
            self.origin_set = True
            rospy.loginfo("Origin set to: x={:.3f}, y={:.3f}, z={:.3f}".format(
                self.origin_x,
                self.origin_y,
                self.origin_z
            ))
            return
        
        # 计算相对于原点的位置
        relative_odom = Odometry()
        relative_odom.header = msg.header
        relative_odom.header.frame_id = msg.header.frame_id  # 保持原frame_id或可修改为"map"
        relative_odom.child_frame_id = msg.child_frame_id
        
        # 计算相对位置：当前位置减去原点位置
        relative_odom.pose.pose.position.x = msg.pose.pose.position.x - self.origin_x
        relative_odom.pose.pose.position.y = msg.pose.pose.position.y - self.origin_y
        relative_odom.pose.pose.position.z = msg.pose.pose.position.z - self.origin_z
        
        # 姿态保持不变，直接复制原始角度
        relative_odom.pose.pose.orientation = msg.pose.pose.orientation
        
        # 复制协方差
        relative_odom.pose.covariance = msg.pose.covariance
        
        # 复制速度信息（速度在body frame，不需要转换）
        relative_odom.twist = msg.twist
        
        # 发布相对odom
        self.relative_odom_pub.publish(relative_odom)
        
        # 发布TF变换 (odom -> base_link)
        transform = TransformStamped()
        # 使用当前时间而不是消息时间戳，确保TF始终是最新的
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = relative_odom.header.frame_id  # odom
        transform.child_frame_id = relative_odom.child_frame_id   # base_link
        
        # 设置变换位置
        transform.transform.translation.x = relative_odom.pose.pose.position.x
        transform.transform.translation.y = relative_odom.pose.pose.position.y
        transform.transform.translation.z = relative_odom.pose.pose.position.z
        
        # 设置变换姿态
        transform.transform.rotation = relative_odom.pose.pose.orientation
        
        # 发布TF变换
        self.tf_broadcaster.sendTransform(transform)
        
        # 更新并发布行驶轨迹
        current_time = rospy.Time.now()
        if (current_time - self.last_pose_time).to_sec() >= self.pose_interval:
            pose_stamped = PoseStamped()
            pose_stamped.header = relative_odom.header
            pose_stamped.pose = relative_odom.pose.pose
            
            self.trajectory.poses.append(pose_stamped)
            self.trajectory.header.stamp = current_time
            
            # 限制轨迹点数量（保留最近1000个点，约200秒的轨迹）
            if len(self.trajectory.poses) > 1000:
                self.trajectory.poses.pop(0)
            
            self.trajectory_pub.publish(self.trajectory)
            self.last_pose_time = current_time
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        converter = OdomRelative()
        converter.run()
    except rospy.ROSInterruptException:
        pass
