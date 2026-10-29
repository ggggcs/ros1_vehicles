#!/usr/bin/env python3
"""
Joint States Publisher for Real Robot
发布机器人关节状态，用于robot_state_publisher生成TF树
用于实车没有硬件驱动发布joint_states的情况
"""

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def publish_joint_states():
    rospy.init_node('joint_state_publisher', anonymous=False)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(50)  # 50Hz
    
    rospy.loginfo("Joint State Publisher started - Publishing static joint states for wheels")
    
    # Jackal的四个轮子
    joint_names = [
        'front_left_wheel',
        'front_right_wheel', 
        'rear_left_wheel',
        'rear_right_wheel'
    ]
    
    while not rospy.is_shutdown():
        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.name = joint_names
        
        # 位置设为0（静态，或者可以根据里程计计算）
        js.position = [0.0, 0.0, 0.0, 0.0]
        js.velocity = [0.0, 0.0, 0.0, 0.0]
        js.effort = [0.0, 0.0, 0.0, 0.0]
        
        pub.publish(js)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
