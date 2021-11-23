#!/usr/bin/env python3

from genpy import message
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import message_filters


class control:
    def __init__(self):
        rospy.init_node('control', anonymous=True)

        self.link_lengths = {
            "link_1": 4.0,
            "link_2": 0.0,
            "link_3": 3.2,
            "link_4": 2.8           
        }

        self.ja1_data = None
        self.ja3_data = None
        self.ja4_data = None

        self.ja1_data_updated = False
        self.ja3_data_updated = False
        self.ja4_data_updated = False

        self.ja1 = rospy.Subscriber('joint_angle_1', Float64, self.ja1_callback)
        self.ja3 = rospy.Subscriber('joint_angle_3', Float64, self.ja2_callback)
        self.ja4 = rospy.Subscriber('joint_angle_4', Float64, self.ja3_callback)
        
        def ja1_callback(self, ja1):
            self.ja1_data = ja1.data
            self.ja1_data_updated = True

            self.callback()

        def ja3_callback(self, ja3):
            self.ja3_data = ja3.data
            self.ja3_data_updated = True

            self.callback()

        def ja4_callback(self, ja4):
            self.ja4_data = ja4.data
            self.ja4_data_updated = True

            self.callback()

        def callback(self):
            if (self.ja1_data_updated and self.ja2_data_updated and self.ja3_data_updated):
                return None

    

    
# call the class
def main(args):
    control = control()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)