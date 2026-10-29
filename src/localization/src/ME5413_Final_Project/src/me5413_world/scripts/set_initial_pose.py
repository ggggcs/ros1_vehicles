#!/usr/bin/env python3
"""
AMCL初始位置设置工具
用于在RViz中手动设置机器人初始位置，帮助AMCL快速收敛
"""

import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

def set_initial_pose(x, y, yaw):
    """
    设置AMCL初始位置
    
    参数:
        x: 地图坐标系下的X坐标 (米)
        y: 地图坐标系下的Y坐标 (米)
        yaw: 航向角 (弧度)
    """
    rospy.init_node('set_amcl_pose', anonymous=True)
    
    # 创建发布器
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    
    # 等待发布器连接
    rospy.sleep(1.0)
    
    # 创建消息
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "map"
    
    # 设置位置
    pose_msg.pose.pose.position.x = x
    pose_msg.pose.pose.position.y = y
    pose_msg.pose.pose.position.z = 0.0
    
    # 设置姿态（从yaw转换为四元数）
    pose_msg.pose.pose.orientation.x = 0.0
    pose_msg.pose.pose.orientation.y = 0.0
    pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    
    # 设置协方差（表示初始位置的不确定性）
    pose_msg.pose.covariance = [
        0.25, 0.0, 0.0, 0.0, 0.0, 0.0,    # x的不确定性
        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,    # y的不确定性
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.068    # yaw的不确定性（约15度）
    ]
    
    # 发布消息
    rospy.loginfo("=" * 50)
    rospy.loginfo("设置AMCL初始位置:")
    rospy.loginfo("  位置: (%.2f, %.2f) m", x, y)
    rospy.loginfo("  航向: %.2f 度", math.degrees(yaw))
    rospy.loginfo("=" * 50)
    
    pub.publish(pose_msg)
    rospy.sleep(0.5)
    pub.publish(pose_msg)  # 发送两次确保接收
    
    rospy.loginfo("初始位置已设置！")
    rospy.loginfo("提示：你也可以在RViz中使用'2D Pose Estimate'工具手动设置")

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("用法: rosrun me5413_world set_initial_pose.py <x> <y> <yaw_degrees>")
        print("示例: rosrun me5413_world set_initial_pose.py 0.0 0.0 0.0")
        print("      rosrun me5413_world set_initial_pose.py -1.7 -15.8 90.0")
        sys.exit(1)
    
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw_deg = float(sys.argv[3])
        yaw_rad = math.radians(yaw_deg)
        
        set_initial_pose(x, y, yaw_rad)
    except ValueError as e:
        print("错误：参数必须是数字")
        print("用法: rosrun me5413_world set_initial_pose.py <x> <y> <yaw_degrees>")
        sys.exit(1)
    except rospy.ROSInterruptException:
        pass
