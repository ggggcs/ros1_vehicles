#!/usr/bin/env python3
"""
RTK定位节点 - 替代AMCL
直接使用RTK提供的高精度位置进行地图定位
发布 map->odom 变换
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
import numpy as np

class RTKLocalization:
    def __init__(self):
        rospy.init_node('rtk_localization', anonymous=False)
        
        # 参数配置
        self.rtk_topic = rospy.get_param('~rtk_topic', '/odom')  # RTK里程计话题
        self.use_enu_offset = rospy.get_param('~use_enu_offset', True)  # 是否使用ENU偏移
        self.map_origin_lat = rospy.get_param('~map_origin_lat', 0.0)  # 地图原点纬度
        self.map_origin_lon = rospy.get_param('~map_origin_lon', 0.0)  # 地图原点经度
        self.map_origin_x = rospy.get_param('~map_origin_x', 0.0)  # 地图原点X（如果已经是ENU）
        self.map_origin_y = rospy.get_param('~map_origin_y', 0.0)  # 地图原点Y
        
        # TF相关
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 存储最新的odom->base_link变换
        self.last_odom_pose = None
        
        # 订阅RTK数据
        self.rtk_sub = rospy.Subscriber(self.rtk_topic, Odometry, self.rtk_callback, queue_size=10)
        
        # 发布定位结果（用于RViz显示）
        self.pose_pub = rospy.Publisher('/rtk_pose', PoseWithCovarianceStamped, queue_size=10)
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("RTK定位节点启动")
        rospy.loginfo("RTK话题: %s", self.rtk_topic)
        rospy.loginfo("地图原点偏移: (%.2f, %.2f)", self.map_origin_x, self.map_origin_y)
        rospy.loginfo("发布 map -> odom 变换")
        rospy.loginfo("=" * 50)
    
    def rtk_callback(self, msg):
        """处理RTK数据并发布map->odom变换"""
        
        try:
            # 1. 获取RTK在odom坐标系下的位置（这就是机器人的绝对位置）
            rtk_x = msg.pose.pose.position.x
            rtk_y = msg.pose.pose.position.y
            rtk_z = msg.pose.pose.position.z
            
            # 检查是否有NaN值
            if np.isnan(rtk_x) or np.isnan(rtk_y) or np.isnan(rtk_z):
                rospy.logwarn_throttle(5.0, "RTK数据包含NaN值，跳过本次更新")
                return
            
            # 2. 应用地图原点偏移（将RTK坐标转换到地图坐标系）
            map_x = rtk_x - self.map_origin_x
            map_y = rtk_y - self.map_origin_y
            
            # 3. 获取当前odom->base_link变换（如果需要）
            # 由于RTK直接给出了机器人在全局坐标系的位置
            # map->odom = map->base_link - odom->base_link
            # 但如果RTK话题就是odom，那么odom坐标系就是全局坐标系
            # 此时只需要发布map->odom的固定偏移
            
            # 4. 构建map->odom变换
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = "map"
            t.child_frame_id = "odom"
            
            # map到odom的平移（地图原点偏移的负值）
            t.transform.translation.x = -self.map_origin_x
            t.transform.translation.y = -self.map_origin_y
            t.transform.translation.z = 0.0
            
            # 无旋转（假设地图坐标系与RTK坐标系对齐）
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            # 发布TF变换
            self.tf_broadcaster.sendTransform(t)
            
            # 5. 发布定位结果用于可视化
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = "map"
            
            pose_msg.pose.pose.position.x = map_x
            pose_msg.pose.pose.position.y = map_y
            pose_msg.pose.pose.position.z = map_z = rtk_z
            pose_msg.pose.pose.orientation = msg.pose.pose.orientation
            
            # RTK协方差（假设RTK精度很高）
            pose_msg.pose.covariance = list(msg.pose.covariance)
            
            self.pose_pub.publish(pose_msg)
            
            # 日志输出（降低频率）
            if rospy.get_time() % 2 < 0.1:  # 每2秒输出一次
                rospy.loginfo_throttle(2.0, "RTK定位: map(%.2f, %.2f) | odom(%.2f, %.2f)", 
                                      map_x, map_y, rtk_x, rtk_y)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "TF查询失败: %s", str(e))
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        localization = RTKLocalization()
        localization.run()
    except rospy.ROSInterruptException:
        pass
