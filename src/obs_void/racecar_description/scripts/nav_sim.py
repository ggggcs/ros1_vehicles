#!/usr/bin/env python3
import rospy
import std_msgs.msg
from yhs_can_msgs.msg import ctrl_cmd
from geometry_msgs.msg import Twist
import math

import time 
import threading
pub = rospy.Publisher("/ctrl_cmd", ctrl_cmd, queue_size=1)

# Ackermann parameters - must match URDF and TEB config
WHEELBASE = 0.66  # meters, from racecar.urdf.xacro

def thread_job():
    rospy.spin()

def callback(data):
    speed = data.linear.x 
    turn = data.angular.z  # This is steering angle from TEB (cmd_angle_instead_rotvel: True) in radians

    msg = ctrl_cmd()
    
    # Convert radians to degrees for steering angle
    steering_angle_degrees = math.degrees(turn)
    
    # Set control command parameters
    msg.ctrl_cmd_gear = 4  # Drive gear
    msg.ctrl_cmd_velocity = speed  # Velocity in m/s
    msg.ctrl_cmd_steering = steering_angle_degrees  # Steering angle in degrees
    msg.ctrl_cmd_Brake = 0  # No brake

    pub.publish(msg)

def SubscribeAndPublish():
    rospy.init_node('nav_sim', anonymous=True)
    rospy.Subscriber('cmd_vel', Twist, callback, queue_size=1, buff_size=52428800)
    #rospy.Subscriber('cmd_vel', Twist, callback,queue_size=1,buff_size=52428800)
    rospy.spin()


if __name__ == '__main__':
    try:
        SubscribeAndPublish()
    except rospy.ROSInterruptException:
        pass


########################
