#!/usr/bin/env python3
import rospy
import std_msgs.msg
from yhs_can_msgs.msg import ctrl_cmd
from geometry_msgs.msg import Twist
from bringup.msg import UltrasonicControl
import math

import time
import threading

# Global variables to store latest messages
latest_cmd_vel = None
latest_fuzzy_control = None

pub = rospy.Publisher("/ctrl_cmd", ctrl_cmd, queue_size=1)

# Ackermann parameters - must match URDF and TEB config
WHEELBASE = 0.66  # meters, from racecar.urdf.xacro

def cmd_vel_callback(data):
    global latest_cmd_vel
    latest_cmd_vel = data
    publish_fused_control()

def fuzzy_control_callback(data):
    global latest_fuzzy_control
    latest_fuzzy_control = data
    publish_fused_control()

def publish_fused_control():
    global latest_cmd_vel, latest_fuzzy_control

    if latest_cmd_vel is None or latest_fuzzy_control is None:
        return

    # Speed control: fixed value like in sim_fuzzy
    final_speed = 0.6  # Fixed speed

    # Angle control: sim angle + fuzzy angle
    sim_angle_degrees = math.degrees(latest_cmd_vel.angular.z)
    fuzzy_angle_degrees = latest_fuzzy_control.delta_angle
    final_steering = sim_angle_degrees + fuzzy_angle_degrees

    msg = ctrl_cmd()

    # Set control command parameters
    msg.ctrl_cmd_gear = 4  # Drive gear
    msg.ctrl_cmd_velocity = final_speed  # Fixed velocity in m/s
    msg.ctrl_cmd_steering = final_steering  # Combined steering angle in degrees
    msg.ctrl_cmd_Brake = 0  # No brake

    pub.publish(msg)
    rospy.loginfo("Published fused control: speed=%.2f m/s, steering=%.2f deg (sim: %.2f + fuzzy: %.2f)",
                  final_speed, final_steering, sim_angle_degrees, fuzzy_angle_degrees)

def SubscribeAndPublish():
    rospy.init_node('nav_fusion', anonymous=True)
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback, queue_size=1, buff_size=52428800)
    rospy.Subscriber('/ultrasonic_fuzzy_control', UltrasonicControl, fuzzy_control_callback, queue_size=1, buff_size=52428800)
    rospy.spin()

if __name__ == '__main__':
    try:
        SubscribeAndPublish()
    except rospy.ROSInterruptException:
        pass