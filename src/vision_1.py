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
        self.image_sub1 = message_filters.Subscriber('/camera1/robot/image_raw', Image)
        self.image_sub2 = message_filters.Subscriber('/camera2/robot/image_raw', Image)
        self.ts = message_filters.TimeSynchronizer([self.image_sub1, self.image_sub2], 10)
        self.ts.registerCallback(self.callback)
        self.bridge = CvBridge()

    def callback(self, image1, image2):
        try:
            self.yz_image = self.bridge.imgmsg_to_cv2(image1, "bgr8")
            self.xz_image = self.bridge.imgmsg_to_cv2(image2, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow('YZ Image', self.yz_image)
        cv2.imshow('XZ Image', self.xz_image)
        self.detect_green(self.yz_image, self.xz_image)
        cv2.waitKey(1)
    
    # Calculate the conversion from pixel to meter
    def pixel2meter(self,image):
        # Obtain the centre of each coloured blob
        circle1Pos = self.detect_blue(image)
        circle2Pos = self.detect_green(image)
        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos)**2)
        return 3 / np.sqrt(dist)
    
      # In this method you can focus on detecting the centre of the red circle
    def detect_red(self,yz_image, xz_image):
        # Isolate the blue colour in the image as a binary image
        yz_mask = cv2.inRange(yz_image, (0, 0, 100), (0, 0, 255))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        yz_mask = cv2.dilate(yz_mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(yz_mask)
        # Calculate pixel coordinates for the centre of the blob
        cy = int(M['m10'] / M['m00'])
        cz = int(M['m01'] / M['m00'])

        xz_mask = cv2.inRange(xz_image, (0, 0, 100), (0, 0, 255))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        xz_mask = cv2.dilate(xz_mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(xz_mask)
        # Calculate pixel coordinates for the centre of the blob
        cx = int(M['m10'] / M['m00'])
        cz2 = int(M['m01'] / M['m00'])
        cv2.imshow('asas', yz_mask)
        cv2.imshow('asasasas', xz_mask)
        cv2.waitKey(1)
        print(np.array([cx, cy, cz]))
        print(cz, cz2)
        return np.array([cx, cy, cz])


    # Detecting the centre of the green circle
    def detect_green(self,yz_image, xz_image):
        yz_mask = cv2.inRange(yz_image, (0, 100, 0), (0, 255, 0))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        yz_mask = cv2.dilate(yz_mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(yz_mask)
        # Calculate pixel coordinates for the centre of the blob
        cy = int(M['m10'] / M['m00'])
        cz = int(M['m01'] / M['m00'])

        xz_mask = cv2.inRange(xz_image, (0, 100, 0), (0, 255, 100))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        xz_mask = cv2.dilate(xz_mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(xz_mask)
        # Calculate pixel coordinates for the centre of the blob
        cx = int(M['m10'] / M['m00'])
        cz2 = int(M['m01'] / M['m00'])
        cv2.imshow('asas', yz_mask)
        cv2.imshow('asasasas', xz_mask)
        cv2.waitKey(1)
        print(np.array([cx, cy, cz]))
        print(cz, cz2)
        return np.array([cx, cy, cz])

    # Detecting the centre of the blue circle
    def detect_blue(self,yz_image, xz_image):
        yz_mask = cv2.inRange(yz_image, (100, 0, 0), (255, 0, 0))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        yz_mask = cv2.dilate(yz_mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(yz_mask)
        # Calculate pixel coordinates for the centre of the blob
        cy = int(M['m10'] / M['m00'])
        cz = int(M['m01'] / M['m00'])

        xz_mask = cv2.inRange(xz_image, (0, 100, 100), (0, 255, 255))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        xz_mask = cv2.dilate(xz_mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(xz_mask)
        # Calculate pixel coordinates for the centre of the blob
        cx = int(M['m10'] / M['m00'])
        cz2 = int(M['m01'] / M['m00'])
        cv2.imshow('asas', yz_mask)
        cv2.imshow('asasasas', xz_mask)
        cv2.waitKey(1)
        print(np.array([cx, cy, cz]))
        print(cz, cz2)
        return np.array([cx, cy, cz])    

    # Detecting the centre of the yellow circle
    def detect_yellow(self,yz_image, xz_image):
        yz_mask = cv2.inRange(yz_image, (0, 100, 100), (0, 255, 255))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        yz_mask = cv2.dilate(yz_mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(yz_mask)
        # Calculate pixel coordinates for the centre of the blob
        cy = int(M['m10'] / M['m00'])
        cz = int(M['m01'] / M['m00'])

        xz_mask = cv2.inRange(xz_image, (0, 100, 100), (0, 255, 255))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        xz_mask = cv2.dilate(xz_mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(xz_mask)
        # Calculate pixel coordinates for the centre of the blob
        cx = int(M['m10'] / M['m00'])
        cz2 = int(M['m01'] / M['m00'])
        cv2.imshow('asas', yz_mask)
        cv2.imshow('asasasas', xz_mask)
        cv2.waitKey(1)
        print(np.array([cx, cy, cz]))
        print(cz, cz2)
        return np.array([cx, cy, cz])

    # Calculate the relevant joint angles from the image
    def detect_joint_angles(self, yz_image, xz_image):
        a = self.pixel2meter(yz_image)
        # Obtain the centre of each coloured blob 
        center = a * self.detect_yellow(yz_image, xz_image)
        circle1Pos = a * self.detect_blue(yz_image, xz_image) 
        circle2Pos = a * self.detect_green(yz_image, xz_image) 
        circle3Pos = a * self.detect_red(yz_image, xz_image)

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