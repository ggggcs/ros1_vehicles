#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
向量场规划器测试脚本 - 验证obs_fitting数据接收
"""

import rospy
from std_msgs.msg import String
import re

def ellipse_callback(msg):
    """测试解析obs_fitting的椭圆数据"""
    rospy.loginfo("=" * 50)
    rospy.loginfo("收到椭圆方程消息")
    rospy.loginfo("=" * 50)
    
    obstacles = []
    lines = msg.data.split('\n')
    
    for line in lines:
        if '中心:' in line and '大小:' in line and '角度:' in line:
            try:
                # 提取中心坐标
                center_match = re.search(r'中心:\(([^,]+),([^)]+)\)', line)
                if not center_match:
                    continue
                center_x = float(center_match.group(1))
                center_y = float(center_match.group(2))
                
                # 提取大小
                size_match = re.search(r'大小:([^x]+)x([^\s]+)', line)
                if not size_match:
                    continue
                width = float(size_match.group(1))
                height = float(size_match.group(2))
                
                # 提取角度
                angle_match = re.search(r'角度:([^°]+)°', line)
                if not angle_match:
                    continue
                angle = float(angle_match.group(1))
                
                obstacles.append({
                    'center': (center_x, center_y),
                    'size': (width, height),
                    'angle': angle
                })
                
                rospy.loginfo("解析椭圆:")
                rospy.loginfo("  中心: (%.3f, %.3f)", center_x, center_y)
                rospy.loginfo("  尺寸: %.3f x %.3f", width, height)
                rospy.loginfo("  角度: %.1f°", angle)
                rospy.loginfo("  半轴: a=%.3f, b=%.3f", width/2.0, height/2.0)
                rospy.loginfo("-" * 30)
                
            except Exception as e:
                rospy.logwarn("解析失败: %s", str(e))
    
    rospy.loginfo("总共解析出 %d 个椭圆障碍物", len(obstacles))
    rospy.loginfo("=" * 50)

if __name__ == '__main__':
    try:
        rospy.init_node('test_ellipse_parser', anonymous=True)
        rospy.loginfo("启动椭圆数据解析测试...")
        rospy.loginfo("订阅话题: /obs_fitting/ellipse_equations")
        rospy.loginfo("等待数据...")
        
        rospy.Subscriber('/obs_fitting/ellipse_equations', String, ellipse_callback)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
