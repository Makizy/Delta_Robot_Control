#!/usr/bin/env python3
import rospy
import tf

def print_end_effector_position():
    rospy.init_node('end_effector_listener')
    
    listener = tf.TransformListener()
    print(1)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            
            # Replace 'base_link' with your robot's base link
            # and 'end_effector_link' with the actual name of your end-effector link
            (trans, rot) = listener.lookupTransform('link_0', 'low_base', rospy.Time(0))
            print(2)
            print("End-effector position: x={0:.2f}, y={1:.2f}, z={2:.2f}".format(trans[0], trans[1], trans[2]))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        rate.sleep()

if __name__ == '__main__':
    print_end_effector_position()
