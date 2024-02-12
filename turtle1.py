# -*- coding: utf-8 -*-
"""
Created on Sat Jan  3 23:20:47 2024

@author: mrslk
"""

import rospy
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import turtlesim.srv
import numpy as np
import random
import json

# Load data from the JSON file
with open('odev.json') as file:
    data = json.load(file)

# Initialize ROS node
rospy.init_node('robot1', anonymous=True) 

# Initialize pose variables for Robot-1 and Robot-2
pose1 = Pose()
pose2 = Pose()

# Initialize Twist message for velocity commands
vel_msg = Twist()

# Flags to check if pose information has been received for Robot-1 and Robot-2
poseflag1 = False
poseflag2 = False

# Callback function for Robot-1
def callback1(data):
    global pose1, poseflag1
    pose1 = data
    poseflag1 = True

# Callback function for Robot-2
def callback2(data):
    global pose2, poseflag2
    pose2 = data
    poseflag2 = True

# Movement function for Robot-1
def move_robot(distance, error_sum):
    """
    Movement function for Robot-1
    """
    # Wait until pose information for both robots is received
    while not (poseflag1 and poseflag2):
        rospy.sleep(0.01)
    
    while True:
        # Calculate errors
        v_error, theta_error = calculate_error(distance)

        # Calculate velocity commands using PID controller
        error_sum, linear_vel, angle_vel = calculate_pid(error_sum, v_error, theta_error)
            
        # Set velocity commands
        vel_msg.linear.x = linear_vel
        vel_msg.angular.z = angle_vel

        # Publish velocity commands
        vel_pub.publish(vel_msg) 
                
        # Sleep to maintain a consistent loop rate
        loop_rate.sleep() 

# Function to calculate error from the distance between Robot-1 and Robot-2
def calculate_error(distance):
    """
    Function to calculate the error from the distance between Robot 1 and Robot 2
    """
    u1 = pose2.x - pose1.x
    u2 = pose2.y - pose1.y
    v_error = np.sqrt(u1**2 + u2**2) - distance
    theta_goal = np.arctan2(u2, u1)
    u3 = theta_goal - pose1.theta
    theta_error = np.arctan2(np.sin(u3), np.cos(u3))
    return v_error, theta_error

# Function to calculate velocity commands using a PID controller
def calculate_pid(error_sum, v_error, theta_error):
    """
    Calculate the speed of the robot using a PID controller.
    """
    # PID controller gains
    Kp = 0.5
    Ki = 0.01

    # Proportional and integral terms for linear velocity
    p = v_error * Kp 
    I = v_error * Ki    

    # Calculate linear velocity
    linear_vel = p + I

    # Update the error sum for the integral term
    error_sum += v_error

    # Gain for angular velocity
    Kh = 0.7

    # Calculate angular velocity
    angle_vel = Kh * theta_error

    return error_sum, linear_vel, angle_vel

# Subscribe to pose topics for Robot-1 and Robot-2
rospy.Subscriber('/turtle1/pose', Pose, callback1)
rospy.Subscriber('/turtle2/pose', Pose, callback2) 

# Publisher for velocity commands for Robot-1
vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=5)

# Rate at which the main loop runs
loop_rate = rospy.Rate(5)

# Wait for the "spawn" service to be available
rospy.wait_for_service('spawn')
spawner = rospy.ServiceProxy("spawn", turtlesim.srv.Spawn)
spawner(0, 0, 0, 'turtle1')  # Spawn Robot-1

# Start the movement of Robot-1
move_robot(data[0]["Following_distance"], 0)

# Keep the node running
rospy.spin()
