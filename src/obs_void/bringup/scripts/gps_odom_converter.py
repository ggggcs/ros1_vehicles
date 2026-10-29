#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import math

class GPSOdomConverter:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('gps_odom_converter', anonymous=True)

        # 订阅GPS odom话题（已经是标准ROS odom格式）
        self.gps_sub = rospy.Subscriber('/AIMhc_UGV1/odom', Odometry, self.gps_callback)

        # 发布odom话题
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        # TF广播器
        self.tf_broadcaster = tf.TransformBroadcaster()

        # 初始位置标志和偏移
        self.initialized = False
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_yaw = 0.0

        rospy.loginfo("GPS Odometry Converter initialized (passthrough mode)")

    def gps_callback(self, gps_msg):
        # 如果还没初始化，以当前GPS位置为原点
        if not self.initialized:
            self.offset_x = gps_msg.pose.pose.position.x
            self.offset_y = gps_msg.pose.pose.position.y

            # 获取初始朝向
            orientation_q = gps_msg.pose.pose.orientation
            self.offset_yaw = tf.transformations.euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            ])[2]  # 只取yaw角度

            self.initialized = True
            rospy.loginfo("GPS Odometry Converter initialized with origin offset: (%.6f, %.6f, %.1f°)",
                         self.offset_x, self.offset_y, math.degrees(self.offset_yaw))

        # 创建新的odom消息
        odom_msg = Odometry()
        odom_msg.header = gps_msg.header
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # 应用位置偏移（以初始位置为原点）
        odom_msg.pose.pose.position.x = gps_msg.pose.pose.position.x - self.offset_x
        odom_msg.pose.pose.position.y = gps_msg.pose.pose.position.y - self.offset_y
        odom_msg.pose.pose.position.z = gps_msg.pose.pose.position.z

        # 应用朝向偏移（以初始朝向为0°）
        original_yaw = tf.transformations.euler_from_quaternion([
            gps_msg.pose.pose.orientation.x,
            gps_msg.pose.pose.orientation.y,
            gps_msg.pose.pose.orientation.z,
            gps_msg.pose.pose.orientation.w
        ])[2]

        relative_yaw = original_yaw - self.offset_yaw
        # 标准化角度到[-pi, pi]
        relative_yaw = math.atan2(math.sin(relative_yaw), math.cos(relative_yaw))

        # 设置新的朝向
        odom_quaternion = tf.transformations.quaternion_from_euler(0, 0, relative_yaw)
        odom_msg.pose.pose.orientation.x = odom_quaternion[0]
        odom_msg.pose.pose.orientation.y = odom_quaternion[1]
        odom_msg.pose.pose.orientation.z = odom_quaternion[2]
        odom_msg.pose.pose.orientation.w = odom_quaternion[3]

        # 直接复制速度信息（GPS可能没有速度信息，这里保持原样）
        odom_msg.twist = gps_msg.twist

        # 设置协方差（如果GPS消息中有的话）
        if hasattr(gps_msg.pose, 'covariance') and gps_msg.pose.covariance:
            odom_msg.pose.covariance = gps_msg.pose.covariance
        if hasattr(gps_msg.twist, 'covariance') and gps_msg.twist.covariance:
            odom_msg.twist.covariance = gps_msg.twist.covariance

        # 发布odom消息
        self.odom_pub.publish(odom_msg)

        # 发布TF变换 (odom -> base_link)
        self.tf_broadcaster.sendTransform(
            (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z),
            odom_quaternion,
            odom_msg.header.stamp,
            "base_link",
            "odom"
        )

if __name__ == '__main__':
    try:
        converter = GPSOdomConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass