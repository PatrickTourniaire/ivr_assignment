#!/usr/bin/env python3

import rospy
import numpy as np
import time
from math import pi
from std_msgs.msg import Float64, String, Float64MultiArray

class robot_motion:

    def __init__(self):
        rospy.init_node('vision_1_movement', anonymous=True)

        self.joint1_publisher = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.joint3_publisher = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.joint4_publisher = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        self.rate = rospy.Rate(100)
        self.initial_time = rospy.get_time()

    def callback(self):
        while not rospy.is_shutdown():
            t = rospy.get_time() - self.initial_time

            self.joint1_package = Float64()
            self.joint1_package.data = pi * np.sin((pi / 28) * t)
            # self.joint1_package.data = pi/2
            self.joint3_package = Float64()
            self.joint3_package.data = (pi / 2) * np.sin((pi / 20) * t)
            # self.joint3_package.data = -pi/4
            self.joint4_package = Float64()
            self.joint4_package.data = (pi / 2) * np.sin((pi / 18) * t)

            self.joint1_publisher.publish(self.joint1_package)
            self.joint3_publisher.publish(self.joint3_package)
            self.joint4_publisher.publish(self.joint4_package)

if __name__ == '__main__':
    try:
        p = robot_motion()
        p.callback()
    except rospy.ROSInterruptException:
        pass