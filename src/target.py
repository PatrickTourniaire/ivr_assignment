#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64


def target_publisher():

    # Defines publisher and subscriber
    # initialize the node named
    rospy.init_node('target_publisher', anonymous=True)
    rate = rospy.Rate(50)  # 50hz
    # initialize a publisher for end effector target positions
    target_pos_pub = rospy.Publisher("target_pos", Float64MultiArray, queue_size=10)

    t0 = rospy.get_time()
    while not rospy.is_shutdown():
        cur_time = np.array([rospy.get_time()]) - t0
        #y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
        tx = 3.0 * np.cos(cur_time * np.pi / 20)
        ty = 4.0 * np.sin(cur_time * np.pi / 14) + 0.5
        tz = 1.0 * np.sin(cur_time * np.pi / 18) + 4.5
        target_pos = Float64MultiArray()
        target_pos.data = np.array([tx, ty, tz])
        target_pos_pub.publish(target_pos)
        rate.sleep()


# run the code if the node is called
if __name__ == '__main__':
    try:
        target_publisher()
    except rospy.ROSInterruptException:
        pass
