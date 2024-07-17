#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from math import sin, cos, sqrt, atan, acos, pi


# Robot geometry constants
e = 0.03 # end effector radius
f = 0.3  # base radius
re = 0.3  # forearm length
rf = 0.51  # arm length

# Trigonometric constants
sqrt3 = sqrt(3.0)
sin120 = sqrt3 / 2.0
cos120 = -0.5
tan60 = sqrt3
sin30 = 0.5
tan30 = 1 / sqrt3

def delta_calcAngleYZ(x0, y0, z0):
    y1 = -0.5 * tan30 * f
    y0 -= 0.5 * tan30 * e
    a = (x0 ** 2 + y0 ** 2 + z0 ** 2 + rf ** 2 - re ** 2 - y1 ** 2) / (2 * z0)
    b = (y1 - y0) / z0
    d = -(a + b * y1) ** 2 + rf * (b ** 2 * rf + rf)
    if d < 0:
        d= -d  # Non-existing point
        yj = (y1 - a * b - sqrt(d)) / (b ** 2 + 1)
        zj = a + b * yj
        theta = 180.0 * atan(-zj / (y1 - yj)) / pi + ((yj > y1) * 180.0)
    else :

        yj = (y1 - a * b - sqrt(d)) / (b ** 2 + 1)
        zj = a + b * yj
        theta = 180.0 * atan(-zj / (y1 - yj)) / pi + ((yj > y1) * 180.0) 
    return 0, theta

def delta_calcInverse(x0, y0, z0):
    status, theta1 = delta_calcAngleYZ(x0, y0, z0)
    if status == 0:
        status, theta2 = delta_calcAngleYZ(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0)
    if status == 0:
        status, theta3 = delta_calcAngleYZ(x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0)
    if status == 0:
        return 0, theta1, theta2, theta3
    return -1, 0, 0, 0  # Error or non-existing position

def calculate_torque(x, y, z):
    status, theta1, theta2, theta3 = delta_calcInverse(x, y, z)
    if status == 0:
        # Placeholder for actual torque calculation
        # For now, just send the angles as torque values as an example
        return theta1, theta2, theta3
    return 0, 0, 0  # If error, return zero torques

# Initialize ROS node
rospy.init_node('delta_control', anonymous=True)


# Initialize ROS publishers for the three main joints of the delta robot
pub_joint1 = rospy.Publisher('/delta_robot/delta_joint1_effort_controller/command', Float64, queue_size=10)
pub_joint2 = rospy.Publisher('/delta_robot/delta_joint2_effort_controller/command', Float64, queue_size=10)
pub_joint3 = rospy.Publisher('/delta_robot/delta_joint3_effort_controller/command', Float64, queue_size=10)

while not rospy.is_shutdown():
    # Desired XYZ position of the end effector
    desired_x, desired_y, desired_z = -0.5, 0.5, 0.5
    
    # Calculate torques (or angles in this example)
    torque1, torque2, torque3 = calculate_torque(desired_x, desired_y, desired_z)
    
    #torque1, torque2, torque3 = 1 , 0.5 , -1
    
    # Publish torques
    pub_joint1.publish(Float64(torque1))
    pub_joint2.publish(Float64(torque2))
    pub_joint3.publish(Float64(torque3))

    rospy.sleep(0.1)  # Adjust frequency accordingly


