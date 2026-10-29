#!/usr/bin/env python3
"""
Odom TF Publisher - 实车导航用
从/odom话题读取里程计数据，发布正确的 odom->base_link TF变换
用于GPS驱动不发布TF的情况
"""

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class OdomTFPublisher:
    def __init__(self):
        rospy.init_node('odom_tf_publisher', anonymous=False)
        
        # ========== UTM坐标转换配置 ========== 
        # 如果RTK输出UTM/地心坐标（大数值），需要设置原点进行转换
        self.use_utm_conversion = rospy.get_param('~use_utm_conversion', True)
        self.utm_origin_x = rospy.get_param('~utm_origin_x', 683576.5653)  # 建图起点X
        self.utm_origin_y = rospy.get_param('~utm_origin_y', 3123046.3857)  # 建图起点Y
        
        # TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 订阅里程计话题
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        
        if self.use_utm_conversion:
            rospy.loginfo("=" * 60)
            rospy.loginfo("Odom TF Publisher - UTM坐标转换模式")
            rospy.loginfo("UTM原点: (%.3f, %.3f)", self.utm_origin_x, self.utm_origin_y)
            rospy.loginfo("将发布转换后的局部坐标 odom -> base_link")
            rospy.loginfo("=" * 60)
        else:
            rospy.loginfo("Odom TF Publisher started - Publishing odom -> base_link transform")
    
    def odom_callback(self, msg):
        """处理里程计消息并发布TF"""
        
        # ⚠️ 检查数据有效性（防止NaN值）
        import math
        if (math.isnan(msg.pose.pose.position.x) or 
            math.isnan(msg.pose.pose.position.y) or 
            math.isnan(msg.pose.pose.position.z) or
            math.isnan(msg.pose.pose.orientation.x) or
            math.isnan(msg.pose.pose.orientation.y) or
            math.isnan(msg.pose.pose.orientation.z) or
            math.isnan(msg.pose.pose.orientation.w)):
            rospy.logwarn_throttle(5.0, "检测到NaN值在odom数据中，跳过发布TF")
            return
        
        # 检查四元数是否归一化
        quat_norm = math.sqrt(
            msg.pose.pose.orientation.x**2 + 
            msg.pose.pose.orientation.y**2 + 
            msg.pose.pose.orientation.z**2 + 
            msg.pose.pose.orientation.w**2
        )
        if abs(quat_norm - 1.0) > 0.1:  # 允许10%误差
            rospy.logwarn_throttle(5.0, "四元数未归一化 (norm=%.3f)，跳过发布TF", quat_norm)
            return
        
        # 创建TF消息
        t = TransformStamped()
        
        # 时间戳
        t.header.stamp = msg.header.stamp
        
        # 坐标系 (正确的方向！)
        t.header.frame_id = msg.header.frame_id  # odom
        t.child_frame_id = msg.child_frame_id     # base_link
        
        # ========== 坐标转换：UTM绝对坐标 -> 局部相对坐标 ==========
        if self.use_utm_conversion:
            # 减去原点，转换为相对坐标
            local_x = msg.pose.pose.position.x - self.utm_origin_x
            local_y = msg.pose.pose.position.y - self.utm_origin_y
            local_z = msg.pose.pose.position.z  # Z通常不需要转换
            
            # 位置（转换后的局部坐标）
            t.transform.translation.x = local_x
            t.transform.translation.y = local_y
            t.transform.translation.z = local_z
            
            # 调试输出（每5秒一次）
            if rospy.get_time() % 5 < 0.1:
                rospy.loginfo_throttle(5.0, 
                    "UTM(%.2f, %.2f) -> 局部(%.2f, %.2f)", 
                    msg.pose.pose.position.x, msg.pose.pose.position.y,
                    local_x, local_y)
        else:
            # 直接使用原始坐标（适用于已经是局部坐标的情况）
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
        
        # 姿态
        t.transform.rotation = msg.pose.pose.orientation
        
        # 发布TF
        self.tf_broadcaster.sendTransform(t)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher = OdomTFPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
