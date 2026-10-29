#!/usr/bin/env python3
"""朝向检查工具 - 检查RTK朝向与地图朝向是否一致"""
import rospy
from nav_msgs.msg import Odometry
import math

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def odom_callback(msg):
    yaw_rad = quaternion_to_yaw(msg.pose.pose.orientation)
    yaw_deg = math.degrees(yaw_rad)
    
    print("\n" + "="*60)
    print("当前RTK航向: %.2f度 (%.4f弧度)" % (yaw_deg, yaw_rad))
    print("="*60)
    print("判断:")
    if abs(yaw_deg) < 5:
        print("  ✅ 接近0度 → map_origin_yaw=0.0")
    elif abs(yaw_deg - 90) < 5:
        print("  ⚠️ 接近90度 → map_origin_yaw=-1.5708 (-90度)")
    elif abs(yaw_deg + 90) < 5:
        print("  ⚠️ 接近-90度 → map_origin_yaw=1.5708 (+90度)")
    elif abs(abs(yaw_deg) - 180) < 5:
        print("  ⚠️ 接近180度 → map_origin_yaw=3.1416 (180度)")
    else:
        print("  📐 自定义角度 → map_origin_yaw=%.4f (%.2f度)" % (-yaw_rad, -yaw_deg))
    print("="*60)

if __name__ == '__main__':
    rospy.init_node('check_yaw')
    print("\n让机器人朝向地图的正北方向，然后观察输出...\n")
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()
