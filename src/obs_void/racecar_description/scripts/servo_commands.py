#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
import math

# Ackermann steering geometry parameters (must match URDF)
WHEELBASE = 0.335  # meters (L), from racecar.urdf.xacro
TRACK_WIDTH = 0.365  # meters (W), distance between left and right wheels (hex_hub_dist)
MIN_TURN_RADIUS = 0.01  # meters, minimum turn radius to avoid division by zero

class ServoCommands:
    def __init__(self):
        rospy.init_node('servo_commands', anonymous=True)
        
        # Initialize publishers (create once, not in callback!)
        self.pub_vel_left_rear_wheel = rospy.Publisher(
            '/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher(
            '/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher(
            '/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher(
            '/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_pos_left_steering_hinge = rospy.Publisher(
            '/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.pub_pos_right_steering_hinge = rospy.Publisher(
            '/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)
        
        # Subscribe to Ackermann command
        rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped, self.set_throttle_steer)
        
        rospy.loginfo("Servo Commands initialized with Ackermann geometry")
        rospy.loginfo(f"  Wheelbase: {WHEELBASE}m, Track width: {TRACK_WIDTH}m")
    
    def calculate_ackermann_angles(self, center_angle):
        """
        Calculate left and right steering angles using Ackermann geometry.
        
        For Ackermann steering, both wheels must turn in the SAME direction:
        - Left turn (φ > 0): Both wheels turn left (positive angles)
          - Inner wheel (left) has LARGER angle
          - Outer wheel (right) has SMALLER angle
        - Right turn (φ < 0): Both wheels turn right (negative angles)
          - Inner wheel (right) has LARGER magnitude
          - Outer wheel (left) has SMALLER magnitude
        
        Standard Ackermann formula:
        cot(φ_inner) = cot(φ_center) - W/(2L)
        cot(φ_outer) = cot(φ_center) + W/(2L)
        
        Args:
            center_angle: Virtual center steering angle (radians)
        
        Returns:
            (left_angle, right_angle): Steering angles for left and right wheels
        """
        if abs(center_angle) < 1e-6:
            # Straight driving
            return 0.0, 0.0
        
        # Use cotangent formula for numerical stability
        # This ensures both wheels always turn in the same direction
        cot_center = 1.0 / math.tan(center_angle)
        offset = TRACK_WIDTH / (2.0 * WHEELBASE)
        
        # Calculate left and right wheel angles
        # The wheel on the inside of the turn gets the larger angle
        if center_angle > 0:  # Left turn
            # Left wheel is inner (larger angle)
            cot_left = cot_center - offset
            left_angle = math.atan(1.0 / cot_left)
            
            # Right wheel is outer (smaller angle)
            cot_right = cot_center + offset
            right_angle = math.atan(1.0 / cot_right)
        else:  # Right turn
            # Left wheel is outer (smaller magnitude)
            cot_left = cot_center - offset
            left_angle = math.atan(1.0 / cot_left)
            
            # Right wheel is inner (larger magnitude)
            cot_right = cot_center + offset
            right_angle = math.atan(1.0 / cot_right)
        
        return left_angle, right_angle
    
    def calculate_wheel_speeds(self, speed, center_angle):
        """
        Calculate differential wheel speeds for Ackermann steering.
        
        When turning, inner wheels travel shorter distance than outer wheels.
        Speed distribution based on turn radius:
        V_left  = V * (R - W/2) / R
        V_right = V * (R + W/2) / R
        
        Args:
            speed: Center velocity (from drive command)
            center_angle: Virtual center steering angle (radians)
        
        Returns:
            (V_left, V_right): Velocities for left and right wheels
        """
        if abs(center_angle) < 1e-6:
            # Straight driving - same speed for both sides
            return speed, speed
        
        # Calculate turn radius: R = L / tan(φ)
        R = WHEELBASE / math.tan(center_angle)
        
        # Apply minimum radius constraint to avoid extreme speeds
        sign = 1 if R >= 0 else -1
        R = sign * max(abs(R), MIN_TURN_RADIUS)
        
        # Differential speed calculation
        V_left = speed * (R - TRACK_WIDTH/2) / R
        V_right = speed * (R + TRACK_WIDTH/2) / R
        
        return V_left, V_right
    
    def set_throttle_steer(self, data):
        """
        Callback for Ackermann drive commands.
        Converts center steering angle and speed to individual wheel commands.
        """
        # Get command values
        speed = data.drive.speed * 13.95348  # Convert to wheel velocity
        center_steer = data.drive.steering_angle
        
        # Calculate Ackermann steering angles for left and right wheels
        left_steer, right_steer = self.calculate_ackermann_angles(center_steer)
        
        # Calculate differential wheel speeds
        V_left, V_right = self.calculate_wheel_speeds(speed, center_steer)
        
        # Publish wheel velocities with differential speeds
        self.pub_vel_left_rear_wheel.publish(V_left)
        self.pub_vel_right_rear_wheel.publish(V_right)
        self.pub_vel_left_front_wheel.publish(V_left)
        self.pub_vel_right_front_wheel.publish(V_right)
        
        # Publish steering angles (different for left and right)
        self.pub_pos_left_steering_hinge.publish(left_steer)
        self.pub_pos_right_steering_hinge.publish(right_steer)
    
    def spin(self):
        """Keep node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ServoCommands()
        node.spin()
    except rospy.ROSInterruptException:
        pass
