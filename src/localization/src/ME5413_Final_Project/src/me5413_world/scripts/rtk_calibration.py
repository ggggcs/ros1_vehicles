#!/usr/bin/env python3
"""
RTK地图标定工具
用于确定map→odom的静态变换参数
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math

class RTKCalibration:
    def __init__(self):
        rospy.init_node('rtk_calibration', anonymous=True)
        
        self.current_odom = None
        
        # 订阅RTK odom
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("RTK地图标定工具")
        rospy.loginfo("=" * 60)
        rospy.loginfo("使用说明：")
        rospy.loginfo("1. 将机器人移动到地图上的已知位置")
        rospy.loginfo("2. 记录RViz中该位置的地图坐标 (map_x, map_y)")
        rospy.loginfo("3. 本工具会显示当前RTK坐标 (odom_x, odom_y)")
        rospy.loginfo("4. 计算偏移: map_origin = odom - map")
        rospy.loginfo("=" * 60)
        
    def odom_callback(self, msg):
        """接收RTK odom数据"""
        self.current_odom = msg
    
    def quaternion_to_yaw(self, q):
        """四元数转yaw角"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def print_current_pose(self):
        """打印当前RTK位置"""
        if self.current_odom is None:
            rospy.logwarn("等待RTK数据...")
            return
        
        pose = self.current_odom.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)
        
        print("\n" + "=" * 60)
        print("【当前RTK位置（odom坐标系）】")
        print("  X: %.3f m" % x)
        print("  Y: %.3f m" % y)
        print("  Yaw: %.2f 度" % math.degrees(yaw))
        print("=" * 60)
        print("\n标定步骤：")
        print("1. 在RViz中查看机器人在地图上的位置 (map_x, map_y)")
        print("2. 如果地图显示位置不对，计算偏移：")
        print("   map_origin_x = %.3f - map_x" % x)
        print("   map_origin_y = %.3f - map_y" % y)
        print("\n示例：")
        print("  如果RViz显示机器人在地图(5.0, 3.0)位置")
        print("  而RTK显示(%.3f, %.3f)" % (x, y))
        print("  则map_origin_x = %.3f - 5.0 = %.3f" % (x, x - 5.0))
        print("  则map_origin_y = %.3f - 3.0 = %.3f" % (y, y - 3.0))
        print("\n3. 在rtk_navigation.launch中设置：")
        print("   <arg name=\"map_origin_x\" default=\"%.3f\"/>" % (x - 5.0))
        print("   <arg name=\"map_origin_y\" default=\"%.3f\"/>" % (y - 3.0))
        print("\n【朝向校验】")
        print("1. 让机器人朝向地图的正北（Y轴正方向）")
        print("2. 查看当前RTK航向角：%.2f度" % math.degrees(yaw))
        print("3. 如果RTK显示不是0度或360度，则需要设置朝向偏移：")
        print("   map_origin_yaw = %.4f 弧度 (%.2f度)" % (-yaw, -math.degrees(yaw)))
        print("\n说明：")
        print("  - 如果机器人朝北时RTK显示90度，说明地图相对RTK逆时针旋转了90度")
        print("  - 需要设置 map_origin_yaw = -1.5708 (即-90度)")
        print("=" * 60 + "\n")
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(0.5)  # 每2秒更新一次
        
        rospy.sleep(2.0)  # 等待数据
        
        while not rospy.is_shutdown():
            self.print_current_pose()
            rate.sleep()

if __name__ == '__main__':
    try:
        calibration = RTKCalibration()
        calibration.run()
    except rospy.ROSInterruptException:
        pass
