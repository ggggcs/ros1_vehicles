#!/usr/bin/env python3
import rospy
import std_msgs.msg
from yhs_can_msgs.msg import ctrl_cmd
from geometry_msgs.msg import Twist
from bringup.msg import UltrasonicControl
import math

import time 
import threading
pub = rospy.Publisher("/ctrl_cmd", ctrl_cmd, queue_size=1)

# Ackermann parameters - must match URDF and TEB config
WHEELBASE = 0.66  # meters, from racecar.urdf.xacro

# Base speed for fuzzy control
BASE_SPEED = 1.0  # m/s, base speed to be multiplied by speed coefficient

def thread_job():
    rospy.spin()

def callback(data):
    # Read fuzzy control outputsz
    delta_angle = data.delta_angle  # Steering angle correction in degrees
    speed_coefficient = data.speed_coefficient  # Speed coefficient (0-1)
    
    # Calculate final speed and steering
    final_speed = BASE_SPEED * speed_coefficient
    final_steering = delta_angle  # Use delta_angle directly as steering angle in degrees

    msg = ctrl_cmd()
    
    # Set control command parameters
    msg.ctrl_cmd_gear = 4  # Drive gear
    msg.ctrl_cmd_velocity = 0.3  # Final velocity in m/s
    msg.ctrl_cmd_steering = final_steering  # Final steering angle in degrees
    msg.ctrl_cmd_Brake = 0  # No brake

    pub.publish(msg)
    # rospy.loginfo("Published control: speed=%.2f m/s, steering=%.2f deg", final_speed, final_steering)

def SubscribeAndPublish():
    rospy.init_node('nav_sim_fuzzy', anonymous=True)
    rospy.Subscriber('/ultrasonic_fuzzy_control', UltrasonicControl, callback, queue_size=1, buff_size=52428800)
    rospy.spin()


if __name__ == '__main__':
    try:
        SubscribeAndPublish()
    except rospy.ROSInterruptException:
        pass
