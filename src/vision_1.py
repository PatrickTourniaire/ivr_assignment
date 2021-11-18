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


class joint_estimation_1:
    def __init__(self):
        rospy.init_node('vision_1', anonymous=True)

        self.link_lengths = {
            "link_1": 4.0,
            "link_2": 0.0,
            "link_3": 3.2,
            "link_4": 2.8           
        }

        self.RED_BLOB_HSV_COLOR_RANGE_BELOW = (0,50,50)
        self.RED_BLOB_HSV_COLOR_RANGE_UPPER = (20,255,255)

        self.GREEN_BLOB_HSV_COLOR_RANGE_BELOW = (50,50,50)
        self.GREEN_BLOB_HSV_COLOR_RANGE_UPPER = (70,255,255)

        self.BLUE_BLOB_HSV_COLOR_RANGE_UPPER = (130,255,255)
        self.BLUE_BLOB_HSV_COLOR_RANGE_BELOW = (110,50,50)

        self.YELLOW_BLOB_HSV_COLOR_RANGE_BELOW = (20,50,50)
        self.YELLOW_BLOB_HSV_COLOR_RANGE_UPPER = (40,255,255)


        self.image_sub1 = message_filters.Subscriber('/camera1/robot/image_raw', Image)
        self.image_sub2 = message_filters.Subscriber('/camera2/robot/image_raw', Image)
        self.joint_angles_pub = rospy.Publisher('joint_states_1', Float64MultiArray, queue_size=10)
        self.ts = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2], 100)
        self.ts.registerCallback(self.callback)
        self.bridge = CvBridge()

    def callback(self, image1, image2):
        try:
            self.yz_image = self.bridge.imgmsg_to_cv2(image1, "bgr8")
            self.xz_image = self.bridge.imgmsg_to_cv2(image2, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # cv2.imshow('YZ Image', self.yz_image)
        # cv2.imshow('XZ Image', self.xz_image)

        
        cv2.waitKey(1)
        self.joints = Float64MultiArray()
        x = self.detect_joint_angles(self.yz_image, self.xz_image)
        self.joints.data = x
        self.joint_angles_pub.publish(self.joints)
    
    # Calculate the conversion from pixel to meter
    def pixel2meter(self,image):
        # Obtain the centre of each coloured blob
        circle1Pos = self.detect_color(self.yz_image, self.xz_image, "yellow")
        circle2Pos = self.detect_color(self.yz_image, self.xz_image, "green")
        # find the distance between two circles
        dist = np.sum((circle1Pos[0:2] - circle2Pos[0:2])**2)
        return 4 / np.sqrt(dist)
        

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
        else:
            print('WRONG COLOR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

    
        cy, cz_yz = self.find_moments(yz_image, color_range_below, color_range_upper)
        cx, cz_xz = self.find_moments(xz_image, color_range_below, color_range_upper)

        return np.array([cx, cy, cz_yz, cz_xz])

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
        if m00 == 0:
            m00 = 0.000001
        cy = int(m10 / m00)
        cz_yz = int(m01 / m00)
        return (int(m10 / m00), int(m01 / m00))



    # Calculate the relevant joint angles from the image
    def detect_joint_angles(self, yz_image, xz_image):
        # hsv_image = cv2.cvtColor(self.yz_image, cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(hsv_image, (110,50,50), (130,255,255)) # blue
        # mask = cv2.inRange(hsv_image, (50,50,50), (70,255,255)) # green
        # mask = cv2.inRange(hsv_image, (20,50,50), (40,255,255)) # yellow
        # mask = cv2.inRange(hsv_image, (0,50,50), (20,255,255)) # red
        # cv2.imshow('HSV Image', mask)
    

        a = self.pixel2meter(yz_image)
        # Obtain the centre of each coloured blob 
        circle1Pos = a * self.detect_color(yz_image, xz_image, "green")
        circle2Pos = a * self.detect_color(yz_image, xz_image, "yellow") 
        circle3Pos = a * self.detect_color(yz_image, xz_image, "blue") 
        circle4Pos = a * self.detect_color(yz_image, xz_image, "red")

        vector_circle1_circle2 = (circle2Pos - circle1Pos)
        vector_circle2_circle3 = (circle3Pos - circle2Pos)
        vector_circle3_circle4 = (circle4Pos - circle3Pos)

        print('=== Circle Positions ===\nCircle1Pos: {}\nCircle2Pos: {}\nCircle3Pos: {}\nCircle4Pos: {}\n=== ==='.format(
            circle1Pos, circle2Pos, circle3Pos, circle4Pos))

        print('=== Vector Positions ===\nCircle1 to Circle2: {}\nCircle2 to Circle3: {}\nCircle3 to Circle4: {}\n===   ==='.format(
            vector_circle1_circle2, vector_circle2_circle3, vector_circle3_circle4))

        # ===== for drawing detecting blob centers on image ====

        circle1Pos_img = self.detect_color(yz_image, xz_image, "green")
        circle2Pos_img = self.detect_color(yz_image, xz_image, "yellow") 
        circle3Pos_img = self.detect_color(yz_image, xz_image, "blue") 
        circle4Pos_img = self.detect_color(yz_image, xz_image, "red")
        print(circle1Pos_img)

        image_with_centers = cv2.circle(self.xz_image, (int(circle1Pos_img[0]), int(circle1Pos_img[3])), 2, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (int(circle2Pos_img[0]), int(circle2Pos_img[3])), 2, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (int(circle3Pos_img[0]), int(circle3Pos_img[3])), 2, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (int(circle4Pos_img[0]), int(circle4Pos_img[3])), 2, (255, 255, 255), cv2.FILLED)

        cv2.imshow('Images with blob centers', cv2.resize(image_with_centers, (400,400)))
        cv2.imwrite('robot_xz.jpg', image_with_centers)

        image_with_centers = cv2.circle(self.yz_image, (int(circle1Pos_img[1]), int(circle1Pos_img[2])), 2, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (int(circle2Pos_img[1]), int(circle2Pos_img[2])), 2, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (int(circle3Pos_img[1]), int(circle3Pos_img[2])), 2, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (int(circle4Pos_img[1]), int(circle4Pos_img[2])), 2, (255, 255, 255), cv2.FILLED)

        cv2.imshow('Images with blob centers YZ', cv2.resize(image_with_centers, (400,400)))
        cv2.imwrite('robot_yz.jpg', image_with_centers)

        # ===== Angles =====

        print('=== ANGLES ===')
        ja2 = - np.arctan2(circle2Pos[0] - circle3Pos[0], circle2Pos[3] - circle3Pos[3])
        ja3 = np.arctan2(circle2Pos[1] - circle3Pos[1], circle2Pos[2] - circle3Pos[2])
        unit_vector1 = vector_circle2_circle3 / np.linalg.norm(vector_circle2_circle3)
        unit_vector2 = vector_circle3_circle4 / np.linalg.norm(vector_circle3_circle4)
        dot_1_2 = np.dot(unit_vector1, unit_vector2)
        ja4 = np.arccos(dot_1_2)

        print([ja2, ja3, ja4])
        print("\n")

        # theta2 = np.arctan2(vector_circle1_circle2[0], vector_circle1_circle2[2])

        # print(np.arctan(np.array(vector_circle1_circle2[0], vector_circle1_circle2[2])))
        # print(np.arctan(np.array(vector_circle1_circle2[1], vector_circle1_circle2[2])))

        # print(np.arccos(np.array(np.dot(vector_circle1_circle2, np.array([0, 1, 0]) / ()))))

        return np.array([ja2, ja3, ja4])
    
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