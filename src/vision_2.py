#!/usr/bin/env python3

import re
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
import math


class joint_estimation_2:
    def __init__(self):
        rospy.init_node('vision_2', anonymous=True)

        self.link_lengths = {
            "link_1": 4.0,
            "link_2": 0.0,
            "link_3": 3.2,
            "link_4": 2.8           
        }

        self.x_axis = np.array([1, 0, 0])
        self.y_axis = np.array([0, 1, 0])
        self.z_axis = np.array([0, 0, 1])

        # Last known configuration of the system. Initially empty
        # index 0: ja1, index 1: ja3, index2: ja4
        self.last_known_ja = []
        
        # signs for joint angle 1, 3, 4.
        self.ja_signs = [1, 1, 1]

        # number of times we have not allowed the angle to change
        # index 0: ja1, index1: ja3, and index 2: ja4
        self.buffer_graph_smoothing = [0, 0, 0]

        #blue blob is directly over yellow blob. (OR circle3 over circle 2)
        self.blue_blob_in_transition = True

        # Keeps record of last known position of blue blob OR circle 3
        self.link2_prev_pos = []

        # Colour Ranges to be used for thresholding
        # RED
        self.RED_BLOB_HSV_COLOR_RANGE_BELOW = (0,50,50)
        self.RED_BLOB_HSV_COLOR_RANGE_UPPER = (20,255,255)
        # GREEN
        self.GREEN_BLOB_HSV_COLOR_RANGE_BELOW = (50,50,50)
        self.GREEN_BLOB_HSV_COLOR_RANGE_UPPER = (70,255,255)
        # BLUE
        self.BLUE_BLOB_HSV_COLOR_RANGE_UPPER = (130,255,255)
        self.BLUE_BLOB_HSV_COLOR_RANGE_BELOW = (110,50,50)
        # YELLOW
        self.YELLOW_BLOB_HSV_COLOR_RANGE_BELOW = (20,50,50)
        self.YELLOW_BLOB_HSV_COLOR_RANGE_UPPER = (40,255,255)

        # Topics on which JAs will be published
        self.joint_angle_1 = rospy.Publisher('joint_angle_1', Float64, queue_size=10)
        self.joint_angle_3 = rospy.Publisher('joint_angle_3', Float64, queue_size=10)
        self.joint_angle_4 = rospy.Publisher('joint_angle_4', Float64, queue_size=10)

        self.image_sub1 = message_filters.Subscriber('/camera1/robot/image_raw', Image)
        self.image_sub2 = message_filters.Subscriber('/camera2/robot/image_raw', Image)
        self.ts = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2], 100)
        self.ts.registerCallback(self.callback)
        self.bridge = CvBridge()


    def publish_angle_estimations(self, angles):
        self.joint_1 = Float64()
        self.joint_1.data = angles[0]

        self.joint_3 = Float64()
        self.joint_3.data = angles[2]

        self.joint_4 = Float64()
        self.joint_4.data = angles[3]

        self.joint_angle_1.publish(self.joint_1)
        self.joint_angle_3.publish(self.joint_3)
        self.joint_angle_4.publish(self.joint_4)


    def callback(self, image1, image2):
        try:
            self.yz_image = self.bridge.imgmsg_to_cv2(image1, "bgr8")
            self.xz_image = self.bridge.imgmsg_to_cv2(image2, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.waitKey(1)
    
        angles = self.detect_joint_angles(self.yz_image, self.xz_image)
        self.publish_angle_estimations(angles)
    

    # Calculate the conversion from pixel to meter
    def pixel2meter(self,image):
        # Obtain the centre of each coloured blob
        circle1Pos, _ = self.detect_color(self.yz_image, self.xz_image, "yellow")
        circle2Pos, _ = self.detect_color(self.yz_image, self.xz_image, "green")
        # find the distance between two circles
        dist = np.sum((circle1Pos[0:3] - circle2Pos[0:3])**2)
        return self.link_lengths["link_1"] / np.sqrt(dist)
        

      # Returns the position and visibility of a given coloured blob from given two images
    def detect_color(self,yz_image, xz_image, color):
        color_range_upper = 0
        color_range_below = 0
        if (color == "red"):
            color_range_upper = self.RED_BLOB_HSV_COLOR_RANGE_UPPER
            color_range_below = self.RED_BLOB_HSV_COLOR_RANGE_BELOW
        elif (color == "blue"):
            color_range_upper = self.BLUE_BLOB_HSV_COLOR_RANGE_UPPER
            color_range_below = self.BLUE_BLOB_HSV_COLOR_RANGE_BELOW
        elif(color == "yellow"):
            color_range_upper = self.YELLOW_BLOB_HSV_COLOR_RANGE_UPPER
            color_range_below = self.YELLOW_BLOB_HSV_COLOR_RANGE_BELOW
        elif (color == 'green'):
            color_range_upper = self.GREEN_BLOB_HSV_COLOR_RANGE_UPPER
            color_range_below = self.GREEN_BLOB_HSV_COLOR_RANGE_BELOW

    
        (cy, cz_yz), small_area_yz = self.find_moments(yz_image, color_range_below, color_range_upper)
        (cx, cz_xz), small_area_xz = self.find_moments(xz_image, color_range_below, color_range_upper)

        return np.array([cx, cy, cz_xz, cz_yz]), (small_area_yz, small_area_xz)
    

    # Calculates and returns the position and visibility of a coloured blob in a given image based on the given threshold
    # Makes use of hue, saturation, and value in hsv space to calculate required properties.
    # Appropriate values of hue, saturation, and value for different coloured blobs are determined using the 
    # python file hsv_color_finder.py in this package.
    def find_moments(self, image, color_range_below, color_range_upper):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        yz_mask = cv2.inRange(hsv_image, color_range_below, color_range_upper)
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        yz_mask = cv2.dilate(yz_mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(yz_mask)
        # Calculate pixel coordinates for the centre of the blob
        m10 = M['m10']
        m00 = M['m00']
        m01 = M['m01']
        # small area determines if this blob is visible
        small_area = False
        if m00 < 10000:
            # print("Small moment!")
            small_area = True

        if m00 == 0:
            m00 = 0.000001

        return ((int(m10 / m00), int(m01 / m00)), small_area)


    # Calculate the relevant joint angles from the image
    def detect_joint_angles(self, yz_image, xz_image):
        circle1Pos_img, circle1SmallAreas_img = self.detect_color(yz_image, xz_image, "green")
        circle2Pos_img, circle2SmallAreas_img = self.detect_color(yz_image, xz_image, "yellow") 
        circle3Pos_img, circle3SmallAreas_img = self.detect_color(yz_image, xz_image, "blue") 
        circle4Pos_img, circle4SmallAreas_img = self.detect_color(yz_image, xz_image, "red")

        # When one blob is behind another in one image
        # We take the appropriate coordinate from the blob that is infront
        if (circle3Pos_img[2] > circle2Pos_img[2]):
            circle3Pos_img[2] = circle2Pos_img[2]

        if circle2SmallAreas_img[0]:
            circle2Pos_img[2] = circle3Pos_img[2]

        if circle2SmallAreas_img[1]:
            circle2Pos_img[0] = circle3Pos_img[0]

        if circle3SmallAreas_img[0]:
            circle3Pos_img[1] = circle2Pos_img[1]

        if circle3SmallAreas_img[1]:
            circle3Pos_img[0] = circle2Pos_img[0]

        if circle4SmallAreas_img[0]:
            circle4Pos_img[1] = circle3Pos_img[1]

        if circle4SmallAreas_img[1]:
            circle4Pos_img[0] = circle3Pos_img[0]

        self.draw_circles_on_blobs(circle1Pos_img, circle2Pos_img, circle3Pos_img, circle4Pos_img)


        a = self.pixel2meter(yz_image)
        
        print("=== END EFFECTOR POSITION ===")
        print('end_effector_pos: ' + str(a * (circle1Pos_img - circle4Pos_img)))
        print("=============================")
        
        # Obtain the centre and visibility of each coloured blob 
        circle1Pos, circle1SmallAreas = self.detect_color(yz_image, xz_image, "green")[0], self.detect_color(yz_image, xz_image, "green")[1]
        circle2Pos, circle2SmallAreas = self.detect_color(yz_image, xz_image, "yellow")[0], self.detect_color(yz_image, xz_image, "yellow")[1]
        circle3Pos, circle3SmallAreas = self.detect_color(yz_image, xz_image, "blue")[0], self.detect_color(yz_image, xz_image, "blue")[1] 
        circle4Pos, circle4SmallAreas = self.detect_color(yz_image, xz_image, "red")[0], self.detect_color(yz_image, xz_image, "red")[1]
       
        # circle1Pos = a * circle1Pos
        # circle2Pos = a * circle2Pos
        # circle3Pos = a * circle3Pos
        # circle4Pos = a * circle4Pos
        
        # When one blob is behind another in one image
        # 1. We take the appropriate coordinate from the blob that is infront
        # 2. We take Z-coordinate of the blob that is behind from the other image. (Because
        #   for every blob we are calculating two z coordinates from two images)
        if (circle3Pos[2] > circle2Pos[2]):
            circle3Pos[2] = circle2Pos[2]

        if circle2SmallAreas[0]:
            circle2Pos[1] = circle3Pos[1]
            # circle2Pos[2] = circle2Pos[3]
        if circle2SmallAreas[1]:
            circle2Pos[0] = circle3Pos[0]
            

        if circle3SmallAreas[0]:
            circle3Pos[1] = circle2Pos[1]
            # circle3Pos[2] = circle3Pos[3]
        if circle3SmallAreas[1]:
            circle3Pos[0] = circle2Pos[0]
            

        if circle4SmallAreas[0]:
            circle4Pos[1] = circle3Pos[1]
            # circle4Pos[2] = circle4Pos[3]
        if circle4SmallAreas[1]:
            circle4Pos[0] = circle3Pos[0]
            

        link1 = (circle2Pos - circle1Pos)
        # bec z points upwards in the Robot's Axis but points downwards in the image.
        link1[2] = -link1[2]
        link1[3] = -link1[3]
        link2 = (circle3Pos - circle2Pos)
        link2[2] = -link2[2]
        link2[3] = -link2[3]
        link3 = (circle4Pos - circle3Pos)
        link3[2] = -link3[2]
        link3[3] = -link3[3]


        # Checks if blue blob is over yellow
        blue_over_yellow = self.blue_over_yellow(circle2Pos, circle3Pos)
        if blue_over_yellow and not(self.blue_blob_in_transition):
            self.blue_blob_in_transition = True
            # print('blue blob entered transition mode. sign of ja3: {}'.format(self.ja_signs[1]))
        # if it was over yellow previously and is not anymore, this means that sign of ja3 has changed.
        elif not blue_over_yellow and self.blue_blob_in_transition:
            self.blue_blob_in_transition = False
            self.ja_signs[1] = self.ja_signs[1] * -1
            # print('blue blob leaving transition mode. sign of ja3: {}'.format(self.ja_signs[1]))

        # which of the z co-ordinates from two images to use
        z_to_use = 2

        #unit vector of link2
        unit_link2 = link2[[0,1,z_to_use]] / np.linalg.norm(link2[[0,1,z_to_use]])
        
        if self.ja_signs[1] == -1:
            #cross vector between link 1 and z axis
            cross_link1_z = np.cross(unit_link2, self.z_axis)
        else:
            # cross vector between link1 and z axis
            cross_link1_z = np.cross(self.z_axis, unit_link2)
        # unit vector of cross_link1_z
        unit_cross_link1_z = cross_link1_z / np.linalg.norm(cross_link1_z)
        dot_x_cross_link1_z = np.dot(unit_cross_link1_z, self.x_axis)
        ja1 = np.arccos(dot_x_cross_link1_z)
        
        # Update the sign of ja1 based on which quadrant blue blob is in right now
        self.update_sign_of_ja1(self.quadrant(link2))


        #finding ja3
        unit_link2 = link2[[0,1,z_to_use]] / np.linalg.norm(link2[[0,1,z_to_use]])
        dot_link2_z = np.dot(self.z_axis, unit_link2)
        ja3 = np.arccos(dot_link2_z)
        

        # finding ja4
        unit_link3 = link3[[0, 1, z_to_use]] / np.linalg.norm(link3[[0, 1, z_to_use]])
        dot_link3_link2 = np.dot(unit_link2, unit_link3)
        cross_link2_link3 = np.cross(unit_link2, unit_link3)
        ja4 = np.arccos(dot_link3_link2)
    
        # sign of ja4
        # as long as blue blob is in positive y, and the cross vector points downwards, or the converse, the angle is positive
        if (cross_link2_link3[2] > 0 and self.ja_signs[1] == -1) or (cross_link2_link3[2] < 0 and self.ja_signs[1] == 1):
            self.ja_signs[2] = -1
            print('condition executed')
        else:
            self.ja_signs[2] = 1

        # assign proper signs to every angle. 
        # The signs have been determined earlier in this method.
        ja1, ja3, ja4 = self.sign_correction(ja1, ja3, ja4)

        # Smooth the graph and get rid of random spikes.
        ja4 = self.smooth_angle(ja4, 2)
        ja3 = self.smooth_angle(ja3, 1)
        ja1 = self.smooth_angle(ja1, 0)

        # Update last known Joint angles with angles calculated in this iteration
        self.last_known_ja = [ja1, ja3, ja4]
        # Update last known position of blob2 with position calculated in this iteration
        self.link2_prev_pos = link2

        return np.array([ja1, 0, ja3, ja4])
    

    # Takes joint angles and assigns signs which have been determined in the given iteration
    def sign_correction(self, ja1, ja3, ja4):
        return ja1 * self.ja_signs[0], ja3 * self.ja_signs[1], ja4 * self.ja_signs[2]


    # Decides if the new calculated value of a given angle should be used, 
    # or the value calculated from the previous iteration should be used
    def smooth_angle(self, angle, angle_index):
        buffer_val = self.buffer_graph_smoothing[angle_index]
        if buffer_val > 0:
            thresh = buffer_val * 0.3
        else:
            thresh = 0.2
        if len(self.last_known_ja) == 0:
            return angle
        if abs(angle - self.last_known_ja[angle_index]) > thresh:
            self.buffer_graph_smoothing[angle_index] += 1
            return self.last_known_ja[angle_index]
        else:
            self.buffer_graph_smoothing[angle_index] = 0
            return angle
    

    # Determines if the blue blob is over yellow blob in any given iteration
    def blue_over_yellow(self, yellow_blob_pos, blue_blob_pos):
        threshold = 10
        if (abs(blue_blob_pos[0] - yellow_blob_pos[0]) < threshold) and (abs(blue_blob_pos[1] - yellow_blob_pos[1]) < threshold):
            return True
        return False
    

    # Assign a sign to joint angle 1 based on the position of 
    # blue blob and the sign of joint angle 3
    def update_sign_of_ja1(self, quadrant):
        if quadrant == 1:
            self.ja_signs[0] = self.ja_signs[1]
        elif quadrant == 2:
            self.ja_signs[0] =  -1 * self.ja_signs[1]
        elif quadrant == 3:
            self.ja_signs[0] =  -1 * self.ja_signs[1]
        elif quadrant == 4:
            self.ja_signs[0] = self.ja_signs[1]
    

    # Determine the quadrant blue blob is in in xy plane
    def quadrant(self, blue_blob_pos):
        # where x and y are positive
        if blue_blob_pos[0] > 0 and blue_blob_pos[1] > 0:
            return 1
        elif blue_blob_pos[0] < 0 and blue_blob_pos[1] > 0:
            return 2
        elif blue_blob_pos[0] < 0 and blue_blob_pos[1] < 0:
            return 3
        elif blue_blob_pos[0] > 0 and blue_blob_pos[1] < 0:
            return 4
        else:
            # //TODO: remove this else statement
            print('quandrant can not be correctly determined.')


    # Draws white circles on the blobs which reftlect the corresponding position of each blob calculated by the algorithm. 
    # Mainly used for debugging
    def draw_circles_on_blobs(self, circle1Pos_img, circle2Pos_img, circle3Pos_img, circle4Pos_img):
        image_with_centers = cv2.circle(self.xz_image, (int(circle1Pos_img[0]), int(circle1Pos_img[2])), 2, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (int(circle2Pos_img[0]), int(circle2Pos_img[2])), 2, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (int(circle3Pos_img[0]), int(circle3Pos_img[2])), 2, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (int(circle4Pos_img[0]), int(circle4Pos_img[2])), 2, (255, 255, 255), cv2.FILLED)

        cv2.imshow('Images with blob centers XZ', cv2.resize(image_with_centers, (400,400)))
        cv2.imwrite('robot_xz.jpg', image_with_centers)

        image_with_centers = cv2.circle(self.yz_image, (int(circle1Pos_img[1]), int(circle1Pos_img[3])), 2, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (int(circle2Pos_img[1]), int(circle2Pos_img[3])), 2, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (int(circle3Pos_img[1]), int(circle3Pos_img[3])), 2, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (int(circle4Pos_img[1]), int(circle4Pos_img[3])), 2, (255, 255, 255), cv2.FILLED)

        cv2.imshow('Images with blob centers YZ', cv2.resize(image_with_centers, (400,400)))
    
# call the class
def main(args):
    joint_estimation = joint_estimation_2()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)