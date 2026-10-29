#!/usr/bin/env python3
"""
AMCL诊断工具
实时监控AMCL定位质量和粒子云状态
"""

import rospy
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class AMCLDiagnostics:
    def __init__(self):
        rospy.init_node('amcl_diagnostics', anonymous=True)
        
        # 数据存储
        self.particle_cloud = None
        self.amcl_pose = None
        self.odom_pose = None
        self.scan_data = None
        
        self.last_particle_count = 0
        self.last_report_time = rospy.Time.now()
        
        # 订阅话题
        rospy.Subscriber('/particlecloud', PoseArray, self.particle_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("AMCL诊断工具已启动")
        rospy.loginfo("监控话题:")
        rospy.loginfo("  - /particlecloud (粒子云)")
        rospy.loginfo("  - /amcl_pose (定位结果)")
        rospy.loginfo("  - /odom (里程计)")
        rospy.loginfo("  - /scan (激光雷达)")
        rospy.loginfo("=" * 60)
    
    def particle_callback(self, msg):
        """处理粒子云数据"""
        self.particle_cloud = msg
        self.last_particle_count = len(msg.poses)
    
    def amcl_pose_callback(self, msg):
        """处理AMCL定位结果"""
        self.amcl_pose = msg
    
    def odom_callback(self, msg):
        """处理里程计数据"""
        self.odom_pose = msg
    
    def scan_callback(self, msg):
        """处理激光雷达数据"""
        self.scan_data = msg
    
    def quaternion_to_yaw(self, q):
        """四元数转yaw角"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def compute_particle_spread(self):
        """计算粒子分散程度"""
        if not self.particle_cloud or len(self.particle_cloud.poses) == 0:
            return None, None, None
        
        positions = np.array([[p.position.x, p.position.y] 
                             for p in self.particle_cloud.poses])
        
        mean_pos = np.mean(positions, axis=0)
        std_x = np.std(positions[:, 0])
        std_y = np.std(positions[:, 1])
        
        return mean_pos, std_x, std_y
    
    def analyze_scan_quality(self):
        """分析激光雷达数据质量"""
        if not self.scan_data:
            return None
        
        valid_ranges = [r for r in self.scan_data.ranges 
                       if self.scan_data.range_min < r < self.scan_data.range_max]
        
        if len(valid_ranges) == 0:
            return {
                'total_points': 0,
                'valid_points': 0,
                'valid_ratio': 0.0,
                'mean_range': 0.0
            }
        
        return {
            'total_points': len(self.scan_data.ranges),
            'valid_points': len(valid_ranges),
            'valid_ratio': len(valid_ranges) / len(self.scan_data.ranges),
            'mean_range': np.mean(valid_ranges)
        }
    
    def print_diagnostics(self):
        """打印诊断信息"""
        current_time = rospy.Time.now()
        if (current_time - self.last_report_time).to_sec() < 2.0:
            return
        
        self.last_report_time = current_time
        
        print("\n" + "=" * 60)
        print("AMCL诊断报告 - %s" % rospy.Time.now())
        print("=" * 60)
        
        # 1. 粒子云状态
        print("\n【粒子云状态】")
        if self.particle_cloud:
            mean_pos, std_x, std_y = self.compute_particle_spread()
            print("  粒子数量: %d" % self.last_particle_count)
            if mean_pos is not None:
                print("  粒子中心: (%.2f, %.2f)" % (mean_pos[0], mean_pos[1]))
                print("  X轴分散: %.3f m" % std_x)
                print("  Y轴分散: %.3f m" % std_y)
                
                # 判断收敛状态
                if std_x < 0.5 and std_y < 0.5:
                    print("  状态: ✓ 已收敛")
                elif std_x < 2.0 and std_y < 2.0:
                    print("  状态: ⚠ 收敛中...")
                else:
                    print("  状态: ✗ 未收敛（粒子过于分散）")
        else:
            print("  ✗ 未接收到粒子云数据")
        
        # 2. AMCL定位结果
        print("\n【AMCL定位】")
        if self.amcl_pose:
            pose = self.amcl_pose.pose.pose
            yaw = self.quaternion_to_yaw(pose.orientation)
            cov = self.amcl_pose.pose.covariance
            print("  位置: (%.2f, %.2f) m" % (pose.position.x, pose.position.y))
            print("  航向: %.2f 度" % math.degrees(yaw))
            print("  位置不确定性: X=%.3f, Y=%.3f" % (math.sqrt(cov[0]), math.sqrt(cov[7])))
            print("  航向不确定性: %.2f 度" % math.degrees(math.sqrt(cov[35])))
            
            # 判断定位质量
            if math.sqrt(cov[0]) < 0.3 and math.sqrt(cov[7]) < 0.3:
                print("  质量: ✓ 高精度定位")
            elif math.sqrt(cov[0]) < 1.0 and math.sqrt(cov[7]) < 1.0:
                print("  质量: ⚠ 中等精度")
            else:
                print("  质量: ✗ 低精度（协方差过大）")
        else:
            print("  ✗ 未接收到AMCL位姿")
        
        # 3. 里程计状态
        print("\n【里程计】")
        if self.odom_pose:
            pose = self.odom_pose.pose.pose
            yaw = self.quaternion_to_yaw(pose.orientation)
            print("  位置: (%.2f, %.2f) m" % (pose.position.x, pose.position.y))
            print("  航向: %.2f 度" % math.degrees(yaw))
            print("  状态: ✓ 正常")
        else:
            print("  ✗ 未接收到里程计数据")
        
        # 4. 激光雷达数据
        print("\n【激光雷达】")
        scan_quality = self.analyze_scan_quality()
        if scan_quality:
            print("  总点数: %d" % scan_quality['total_points'])
            print("  有效点: %d (%.1f%%)" % (
                scan_quality['valid_points'], 
                scan_quality['valid_ratio'] * 100
            ))
            print("  平均距离: %.2f m" % scan_quality['mean_range'])
            
            if scan_quality['valid_ratio'] > 0.7:
                print("  质量: ✓ 良好")
            elif scan_quality['valid_ratio'] > 0.4:
                print("  质量: ⚠ 一般")
            else:
                print("  质量: ✗ 差（有效点太少）")
        else:
            print("  ✗ 未接收到激光数据")
        
        # 5. 建议
        print("\n【诊断建议】")
        if self.particle_cloud and self.amcl_pose and self.scan_data:
            mean_pos, std_x, std_y = self.compute_particle_spread()
            scan_quality = self.analyze_scan_quality()
            
            if std_x > 2.0 or std_y > 2.0:
                print("  • 粒子过于分散，建议:")
                print("    1. 在RViz中使用'2D Pose Estimate'设置初始位置")
                print("    2. 或使用: rosrun me5413_world set_amcl_pose.py <x> <y> <yaw>")
            
            if scan_quality['valid_ratio'] < 0.4:
                print("  • 激光数据质量差，建议:")
                print("    1. 检查激光雷达是否正常工作")
                print("    2. 检查/scan话题是否正确发布")
                print("    3. 调整AMCL的laser_min_range和laser_max_range参数")
            
            if not self.odom_pose:
                print("  • 里程计数据缺失，建议:")
                print("    1. 检查/odom话题是否正确发布")
                print("    2. 检查odom_tf_publisher是否运行")
        else:
            print("  • 等待数据接收中...")
        
        print("=" * 60)
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(0.5)  # 0.5 Hz (每2秒一次)
        
        while not rospy.is_shutdown():
            self.print_diagnostics()
            rate.sleep()

if __name__ == '__main__':
    try:
        diagnostics = AMCLDiagnostics()
        diagnostics.run()
    except rospy.ROSInterruptException:
        pass
