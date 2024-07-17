#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

def callback(data):
    rospy.loginfo("Received data: %s", data.data)

def listener():
    rospy.init_node('test_listener', anonymous=True)
    rospy.Subscriber("/delta_robot/delta_joint1_effort_controller/command", Float64, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
