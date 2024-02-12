# -*- coding: utf-8 -*-
"""
Created on Sat Jan  4 23:25:17 2024

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

# Read data from JSON file
with open('Odev6.json') as f:
    veri = json.load(f)

# Initialize pose variable for Robot-2
pose = Pose()

# Flag to check if pose information has been received for Robot-2
poseflag = False

# Callback function for Robot-2's pose
def callback(data):
    global pose, poseflag
    pose = data
    poseflag = True

# Move Robot-2 with specified linear displacement and angular velocity
def move_turtlebot2(speed, angle_speed):
    """
    Robot 2's linear displacement function
    """
    while not poseflag:  # This loop works until the pose info arrives.
        rospy.sleep(0.01)

    vel_msg = Twist()   
    while True:
        vel_msg.linear.x = speed
        vel_pub.publish(vel_msg)
        rospy.sleep(1)

        direction_control(angle_speed)

        loop_rate.sleep()

# Rotate Robot-2 to a specific angle with given angular velocity
def rotate(angle_speed, theta):
    """
    Robot 2's angular displacement function
    """
    vel_msg = Twist()
    while True:
        diff = abs(pose.theta - theta)
        
        if diff > 0.1:
            vel_msg.angular.z = angle_speed
            vel_pub.publish(vel_msg)
        else:
            vel_msg.angular.z = 0
            vel_pub.publish(vel_msg)
            break

# Adjust Robot-2's direction when hitting a wall
def direction_control(angle_speed):
    """
    Angle change when Robot-2 hits a wall
    """
    vel_msg = Twist()
    while not poseflag:
        rospy.sleep(0.01)
    
    # Hit to left wall and theta 90-180
    if pose.x < 0.1 and pose.theta > 0:
        theta = np.pi - pose.theta
        rotate(angle_speed, theta)

    # Hit to left wall and theta 180-270    
    elif pose.x < 0.1 and pose.theta < 0:
        theta = -np.pi - pose.theta
        rotate(angle_speed, theta)

    # Hit to right wall and theta 0-90
    elif pose.x > 11 and pose.theta > 0:
        theta = np.pi - pose.theta
        rotate(angle_speed, theta)

    # Hit to right wall and theta 270-360 
    elif pose.x > 11 and pose.theta < 0:
        theta = -np.pi - pose.theta
        rotate(angle_speed, theta)

    # Hit to upper wall  
    elif pose.y > 11:
        theta = -pose.theta
        rotate(angle_speed, theta)

    # Hit to bottom wall
    elif pose.y < 0.1:
        theta = -pose.theta
        rotate(angle_speed, theta)

if __name__ == '__main__':
    rospy.init_node('robot2', anonymous=True) 
    
    rospy.Subscriber('/turtle2/pose', Pose, callback) 

    vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=5)

    