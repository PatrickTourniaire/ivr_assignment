#!/usr/bin/env python3

from numpy.core.numeric import NaN
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


class joint_estimation_1:
    def __init__(self):
        rospy.init_node('vision_1', anonymous=True)

        self.link_lengths = {
            "link_1": 4.0,
            "link_2": 0.0,
            "link_3": 3.2,
            "link_4": 2.8           
        }

        self.errors_2 = []
        self.errors_3 = []
        self.errors_4 = []

        self.joint_angles_sub = message_filters.Subscriber('joint_states_1', Float64MultiArray)
        self.joint_angle_2_sub = message_filters.Subscriber('/robot/joint2_position_controller/command', Float64)
        self.joint_angle_3_sub = message_filters.Subscriber('/robot/joint3_position_controller/command', Float64)
        self.joint_angle_4_sub = message_filters.Subscriber('/robot/joint4_position_controller/command', Float64)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.joint_angles_sub, self.joint_angle_2_sub, self.joint_angle_3_sub, self.joint_angle_4_sub], queue_size=10, slop=0.1, allow_headerless=True)
        self.ts.registerCallback(self.callback)
        self.bridge = CvBridge()

    def callback(self, pred_angles, angle2, angle3, angle4):
        if pred_angles.data[0] != None or  angle2.data != None:
            self.errors_2.append(abs(pred_angles.data[0] - angle2.data))
        
        if pred_angles.data[1] != None or  angle3.data != None:
            self.errors_3.append(abs(pred_angles.data[1] - angle3.data))

        if pred_angles.data[2] != None or angle4.data != None:
            self.errors_4.append(abs(pred_angles.data[2] - angle4.data))

        print("Joint2 average error: " + str(sum(self.errors_2) / float(len(self.errors_2))))
        print("Joint3 average error: " + str(sum(self.errors_3) / float(len(self.errors_3))))
        print("Joint4 average error: " + str(sum(self.errors_4) / float(len(self.errors_4))))


    
# call the class
def main(args):
    joint_estimation = joint_estimation_1()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)