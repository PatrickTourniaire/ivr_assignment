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

        self.previous_step_time = np.array([rospy.get_time()], dtype='float64')
        self.previous_step_time2 = np.array([rospy.get_time()], dtype='float64')

        self.error = np.array([0.0, 0.0, 0.0], dtype="float64")

        self.target_pos = None

        self.end_effector_pos = np.array([0.0, 0.0, 10.0], dtype='float64')

        self.ja1 = rospy.Subscriber('joint_angle_1', Float64, self.ja1_callback)
        self.ja3 = rospy.Subscriber('joint_angle_3', Float64, self.ja3_callback)
        self.ja4 = rospy.Subscriber('joint_angle_4', Float64, self.ja4_callback)

        self.target_pos_sub = rospy.Subscriber("target_pos", Float64MultiArray, self.target_callback)

        self.end_effector_pub = rospy.Publisher("/end_effector", Float64MultiArray, queue_size=10)

        self.joint_1_pub = rospy.Publisher('robot/joint1_position_controller/command', Float64, queue_size=10)
        self.joint_3_pub = rospy.Publisher('robot/joint3_position_controller/command', Float64, queue_size=10)
        self.joint_4_pub = rospy.Publisher('robot/joint4_position_controller/command', Float64, queue_size=10)


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

    def target_callback(self, target):
        self.target_pos = target.data

    def callback(self):
        if (self.ja1_data_updated and self.ja3_data_updated and self.ja4_data_updated):
            end_effector_pos = self.calculate_end_effector_position()

            if (self.target_pos != None):
                q_d = self.control_open(self.target_pos)
                

                q1 = Float64()
                q1.data = q_d[0]
                self.joint_1_pub.publish(q1)

                q3 = Float64()
                q3.data = q_d[2]
                self.joint_3_pub.publish(q3)

                q4 = Float64()
                q4.data = q_d[3]
                self.joint_4_pub.publish(q4)

            end_effector = Float64MultiArray()
            end_effector.data = end_effector_pos
            self.end_effector_pub.publish(end_effector)

            self.ja1_data_updated = False
            self.ja3_data_updated = False
            self.ja4_data_updated = False
    
    def calculate_end_effector_position(self):
        self.end_effector_pos = self.forward_kinematics(self.ja1_data, self.ja3_data, self.ja4_data)

        print("=== FK END EFFECTOR ===")
        print(self.end_effector_pos)
        print("=======================")

        return self.end_effector_pos

    def forward_kinematics(self, theta1, theta3, theta4):
        dh_params = [
            [math.radians(90), 0, self.link_lengths["link_1"], theta1],
            [math.radians(90), 0, 0, math.radians(90)],
            [math.radians(90), self.link_lengths["link_3"], 0, theta3],
            [0, self.link_lengths["link_4"], 0, theta4]
        ]

        A_1_0 = np.array(self.calculate_transformation(dh_params[0]))
        A_2_1 = np.array(self.calculate_transformation(dh_params[1]))
        A_3_2 = np.array(self.calculate_transformation(dh_params[2]))
        A_4_3 = np.array(self.calculate_transformation(dh_params[3]))

        K = A_1_0 @ A_2_1 @ A_3_2 @ A_4_3
        end_effector_pos = [-K[0][3], -K[1][3], K[2][3]]

        return end_effector_pos

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

    def calc_jacobian(self, ja1, ja2, ja3, ja4):

        a1 = self.link_lengths["link_1"]
        a2 = self.link_lengths["link_2"]
        a3 = self.link_lengths["link_3"]
        a4 = self.link_lengths["link_4"]

        jacobian_matrix = np.array([
            [-a4*np.sin(ja4)*np.sin(ja1)-a4*np.cos(ja3)*np.cos(ja4)*np.cos(ja1)-a3*np.cos(ja3)*np.cos(ja1), 0, a4*np.sin(ja1)*np.cos(ja4)*np.sin(ja3) + a3*np.sin(ja1)*np.sin(ja3), a4*np.cos(ja1)*np.cos(ja4)+a4*np.sin(ja1)*np.sin(ja4)*np.cos(ja3)],
            [-a4*np.cos(ja3)*np.cos(ja4)*np.sin(ja1) + a4*np.sin(ja4)*np.cos(ja1) - a3*np.sin(ja1) - a3*np.cos(ja3)*np.sin(ja1), 0, -a4*np.cos(ja1)*np.sin(ja3)*np.cos(ja4) - a3*np.cos(ja1)*np.sin(ja3), a4*np.sin(ja1)*np.cos(ja4)-a4*np.cos(ja1)*np.cos(ja3)*np.sin(ja4)],
            [0, 0, a4*np.cos(ja3)*np.cos(ja4) + a3*np.cos(ja3), -a4*np.sin(ja3)*np.sin(ja4)]
        ])

        return jacobian_matrix

    def control_open(self, target):
        current_time = rospy.get_time()
        
        dt = current_time - self.previous_step_time2
        self.previous_step_time2 = dt

        J_inv = np.linalg.pinv(self.calc_jacobian(self.ja1_data, 0, self.ja3_data, self.ja4_data))
        pos_d = np.array(target)
        q = [self.ja1_data, 0, self.ja3_data, self.ja4_data]

        self.error = (pos_d - self.end_effector_pos) / dt
        q_d = q + (dt * np.dot(J_inv, self.error.transpose()))

        return q_d

            
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