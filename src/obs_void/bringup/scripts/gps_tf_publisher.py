#!/usr/bin/env python3
"""
GPS TF Publisher
订阅GPS输出的odom消息，发布对应的TF变换
不做任何坐标转换，只是桥接odom消息到TF树
"""
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class GPSTFPublisher:
    def __init__(self):
        rospy.init_node('gps_tf_publisher', anonymous=True)
        
        # TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 订阅GPS的odom话题
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        rospy.loginfo("GPS TF Publisher initialized")
        rospy.loginfo("Subscribing to /odom and publishing TF: odom -> base_link")
    
    def odom_callback(self, msg):
        """
        接收odom消息并发布对应的TF
        """
        # 创建TF消息
        transform = TransformStamped()
        
        # 使用消息时间戳保持时间一致性，但如果太旧则使用当前时间
        msg_time = msg.header.stamp
        current_time = rospy.Time.now()
        time_diff = (current_time - msg_time).to_sec()
        
        # 如果消息时间戳太旧（超过1秒），使用当前时间
        if time_diff > 1.0 or time_diff < -0.1:
            transform.header.stamp = current_time
            rospy.logwarn_throttle(5.0, "GPS odom timestamp is %.2fs old, using current time", time_diff)
        else:
            transform.header.stamp = msg_time
            
        transform.header.frame_id = msg.header.frame_id  # 应该是 "odom"
        transform.child_frame_id = msg.child_frame_id    # 应该是 "base_link"
        
        # 位置
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        
        # 姿态
        transform.transform.rotation = msg.pose.pose.orientation
        
        # 发布TF
        self.tf_broadcaster.sendTransform(transform)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher = GPSTFPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
