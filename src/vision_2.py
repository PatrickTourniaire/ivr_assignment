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

        self.previous_angles = []

        self.RED_BLOB_HSV_COLOR_RANGE_BELOW = (0,50,50)
        self.RED_BLOB_HSV_COLOR_RANGE_UPPER = (20,255,255)

        self.GREEN_BLOB_HSV_COLOR_RANGE_BELOW = (50,50,50)
        self.GREEN_BLOB_HSV_COLOR_RANGE_UPPER = (70,255,255)

        self.BLUE_BLOB_HSV_COLOR_RANGE_UPPER = (130,255,255)
        self.BLUE_BLOB_HSV_COLOR_RANGE_BELOW = (110,50,50)

        self.YELLOW_BLOB_HSV_COLOR_RANGE_BELOW = (20,50,50)
        self.YELLOW_BLOB_HSV_COLOR_RANGE_UPPER = (40,255,255)

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
        dist = np.sum((circle1Pos[0:2] - circle2Pos[0:2])**2)
        return self.link_lengths["link_1"] / np.sqrt(dist)
        

      # In this method you can focus on detecting the centre of the red circle
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

        small_area = False
        if m00 < 10000:
            print("Small moment!")
            small_area = True

        if m00 == 0:
            m00 = 0.000001
        
        cy = int(m10 / m00)
        cz_yz = int(m01 / m00)
        return ((int(m10 / m00), int(m01 / m00)), small_area)



    # Calculate the relevant joint angles from the image
    def detect_joint_angles(self, yz_image, xz_image):
        circle1Pos_img, circle1SmallAreas_img = self.detect_color(yz_image, xz_image, "green")
        circle2Pos_img, circle2SmallAreas_img = self.detect_color(yz_image, xz_image, "yellow") 
        circle3Pos_img, circle3SmallAreas_img = self.detect_color(yz_image, xz_image, "blue") 
        circle4Pos_img, circle4SmallAreas_img = self.detect_color(yz_image, xz_image, "red")
        #print(circle1Pos_img)

        if (circle3Pos_img[2] > circle2Pos_img[2]):
            circle3Pos_img[2] = circle2Pos_img[2]

        # if circle2SmallAreas_img[0]:
        #     circle2Pos_img[2] = circle3Pos_img[2]

        # if circle2SmallAreas_img[1]:
        #     circle2Pos_img[0] = circle3Pos_img[0]

        # if circle3SmallAreas_img[0]:
        #     circle3Pos_img[1] = circle2Pos_img[1]

        # if circle3SmallAreas_img[1]:
        #     circle3Pos_img[0] = circle2Pos_img[0]

        # if circle4SmallAreas_img[0]:
        #     circle4Pos_img[1] = circle3Pos_img[1]

        # if circle4SmallAreas_img[1]:
        #     circle4Pos_img[0] = circle3Pos_img[0]

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


        a = self.pixel2meter(yz_image)
        
        # Obtain the centre of each coloured blob 
        circle1Pos, circle1SmallAreas = self.detect_color(yz_image, xz_image, "green")[0], self.detect_color(yz_image, xz_image, "green")[1]
        circle2Pos, circle2SmallAreas = self.detect_color(yz_image, xz_image, "yellow")[0], self.detect_color(yz_image, xz_image, "yellow")[1]
        circle3Pos, circle3SmallAreas = self.detect_color(yz_image, xz_image, "blue")[0], self.detect_color(yz_image, xz_image, "blue")[1] 
        circle4Pos, circle4SmallAreas = self.detect_color(yz_image, xz_image, "red")[0], self.detect_color(yz_image, xz_image, "red")[1]
        
        if (circle3Pos[2] > circle2Pos[2]):
            circle3Pos[2] = circle2Pos[2]

        if circle2SmallAreas[0]:
            circle2Pos[1] = circle3Pos[1]

        if circle2SmallAreas[1]:
            circle2Pos[0] = circle3Pos[0]

        if circle3SmallAreas[0]:
            circle3Pos[1] = circle2Pos[1]

        if circle3SmallAreas[1]:
            circle3Pos[0] = circle2Pos[0]

        if circle4SmallAreas[0]:
            circle4Pos[1] = circle3Pos[1]

        if circle4SmallAreas[1]:
            circle4Pos[0] = circle3Pos[0]

        link2_vec = [circle3Pos[0] - circle2Pos[0], circle3Pos[2] - circle2Pos[2]]
        link3_vec = [circle4Pos[0] - circle3Pos[0], circle4Pos[2] - circle3Pos[2]]

        ja1 = - np.arctan2(circle3Pos[0] - circle1Pos[0], circle3Pos[1] - circle1Pos[1]) + 0.07
        ja3 = - np.arctan2(circle3Pos[1] - circle2Pos[1], circle3Pos[3] - circle2Pos[3]) - 0.07
        ja4 = - np.arctan2(circle3Pos[0] - circle4Pos[0], circle3Pos[2] - circle4Pos[2])

        if len(self.previous_angles) == 0:
            self.previous_angles = [ja1, ja3, ja4]

        if ja3 > 0:
            ja3 -= 3.14
        else:
            ja3 += 3.14

        if (abs(self.previous_angles[1] - ja3) > 0.2):
            ja3 = self.previous_angles[1]

        if (abs(self.previous_angles[0] - ja1) > 1 and self.previous_angles[0] < ja1 and ja1 > 0):
            ja1 -= 3.14
        else:
            ja1 += 3.14

        
        self.previous_angles = [ja1, ja3, ja4]
        return np.array([ja1, 0, ja3, ja4])
    
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