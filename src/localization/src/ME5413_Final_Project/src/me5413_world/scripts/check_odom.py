#!/usr/bin/env python3
"""检查RTK odom数据格式"""
import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    
    print("\n" + "="*60)
    print("RTK Odom 数据检查")
    print("="*60)
    print("坐标: (%.3f, %.3f)" % (x, y))
    print("坐标系: %s -> %s" % (msg.header.frame_id, msg.child_frame_id))
    
    if abs(x) > 10000 or abs(y) > 10000:
        print("\n⚠️  检测到大数值坐标（可能是UTM/地心坐标）")
        print("    这会导致TF偏移过大，地图无法显示")
        print("\n建议：")
        print("1. 检查RTK驱动是否可以输出局部坐标")
        print("2. 或者记录建图起点坐标作为原点：")
        print("   map_origin_x = %.3f" % x)
        print("   map_origin_y = %.3f" % y)
        print("3. 使用动态坐标转换节点处理大数值")
    else:
        print("\n✅ 坐标数值合理（局部坐标）")
        print("   可以直接使用零偏移：map_origin_x=0, map_origin_y=0")
    print("="*60)

if __name__ == '__main__':
    rospy.init_node('check_odom')
    print("\n监听 /odom 话题...\n")
    rospy.Subscriber('/odom', Odometry, callback)
    rospy.spin()
