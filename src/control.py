#!/usr/bin/env python3

from genpy import message
import roslib
import sys
import rospy
import cv2
import math
import numpy as np
from scipy.spatial.transform import Rotation
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
        self.ja3 = rospy.Subscriber('joint_angle_3', Float64, self.ja3_callback)
        self.ja4 = rospy.Subscriber('joint_angle_4', Float64, self.ja4_callback)

        self.ja_test = rospy.Subscriber('/robot/joint_states/position', Float64MultiArray, self.callback_test)

        self.end_effector_publisher = rospy.Publisher("/end_effector", Float64MultiArray, queue_size=10)
    
    def callback_test(self, jas):
        ja1, _, ja3, ja4 = jas.data

        dh_params = [
            [math.radians(90), 0, self.link_lengths["link_1"], ja1],
            [math.radians(90), 0, 0, math.radians(90)],
            [-math.radians(90), self.link_lengths["link_3"], 0, ja3],
            [0, self.link_lengths["link_4"], 0, ja4]
        ]

        A_1_0 = np.array(self.calculate_transformation(dh_params[0]))
        A_2_1 = np.array(self.calculate_transformation(dh_params[1]))
        A_3_2 = np.array(self.calculate_transformation(dh_params[2]))
        A_4_3 = np.array(self.calculate_transformation(dh_params[3]))

        K = A_1_0 @ A_2_1 @ A_3_2 @ A_4_3
        end_effector_pos = [K[0][3], K[1][3], K[2][3]]

        print("true states: " + end_effector_pos)


    def ja1_callback(self, ja1):
        if math.isnan(ja1.data):
            ja1.data = 0
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
    
    def matprint(self, mat, fmt="g"):
        col_maxes = [max([len(("{:"+fmt+"}").format(x)) for x in col]) for col in mat.T]
        for x in mat:
            for i, y in enumerate(x):
                print(("{:"+str(col_maxes[i])+fmt+"}").format(y), end="  ")
            print("")

    def callback(self):
        if (self.ja1_data_updated and self.ja3_data_updated and self.ja4_data_updated):
            dh_params = [
                [math.radians(90), 0, self.link_lengths["link_1"], self.ja1_data],
                [math.radians(90), 0, 0, math.radians(90)],
                [-math.radians(90), self.link_lengths["link_3"], 0, self.ja3_data],
                [0, self.link_lengths["link_4"], 0, self.ja4_data]
            ]

            print(dh_params)

            A_1_0 = np.array(self.calculate_transformation(dh_params[0]))
            A_2_1 = np.array(self.calculate_transformation(dh_params[1]))
            A_3_2 = np.array(self.calculate_transformation(dh_params[2]))
            A_4_3 = np.array(self.calculate_transformation(dh_params[3]))

            K = A_1_0 @ A_2_1 @ A_3_2 @ A_4_3
            end_effector_pos = [K[0][3], K[1][3], K[2][3]]

            print(end_effector_pos)

            # print(self.matprint(temp_c))
            self.end_effector_pos = Float64MultiArray()
            self.end_effector_pos.data = end_effector_pos
            self.end_effector_publisher.publish(self.end_effector_pos)

            self.ja1_data_updated = False
            self.ja3_data_updated = False
            self.ja4_data_updated = False
        

    def calculate_transformation(self, dh_param_row):
        alpha, a, d, theta = dh_param_row

        rot_theta = np.array([
            [np.cos(theta), -np.sin(theta), 0, 0],
            [np.sin(theta), np.cos(theta), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        trans_d = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, d],
            [0, 0, 0, 1]
        ])

        trans_a = np.array([
            [1, 0, 0, a],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        rot_alpha = np.array([
            [1, 0, 0, 0],
            [0, np.cos(alpha), -np.sin(alpha), 0],
            [0, np.sin(alpha), np.cos(alpha), 0],
            [0, 0, 0, 1]
        ])

        return rot_theta @ trans_d @ trans_a @ rot_alpha
            
# call the class
def main(args):
    control_1 = control()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)