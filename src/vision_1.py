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
        circle1Pos = self.detect_yellow(self.yz_image, self.xz_image)
        circle2Pos = self.detect_green(self.yz_image, self.xz_image)
        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos)**2)
        return 4 / np.sqrt(dist)
    

      # In this method you can focus on detecting the centre of the red circle
    def detect_red(self,yz_image, xz_image):
        # Isolate the blue colour in the image as a binary image
        yz_mask = cv2.inRange(yz_image, (0, 0, 1), (0, 0, 255))
        cv2.imshow('ass', 255 - self.yz_image[:,:,0])
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
        # cv2.imshow('asas', yz_mask)
        # cv2.imshow('asasasas', xz_mask)
        cv2.waitKey(1)
        # print(np.array([cx, cy, cz]))
        # print(cz, cz2)
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
        # cv2.imshow('asas', yz_mask)
        # cv2.imshow('asasasas', xz_mask)
        cv2.waitKey(1)
        # print(np.array([cx, cy, cz]))
        # print(cz, cz2)
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

        xz_mask = cv2.inRange(xz_image, (100, 0, 0), (255, 0, 0))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        xz_mask = cv2.dilate(xz_mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(xz_mask)
        # Calculate pixel coordinates for the centre of the blob
        cx = int(M['m10'] / M['m00'])
        cz2 = int(M['m01'] / M['m00'])
        # cv2.imshow('asas', yz_mask)
        # cv2.imshow('asasasas', xz_mask)
        cv2.waitKey(1)
        # print(np.array([cx, cy, cz]))
        # print(cz, cz2)
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
        # cv2.imshow('asas', yz_mask)
        # cv2.imshow('asasasas', xz_mask)
        cv2.waitKey(1)
        # print(np.array([cx, cy, cz]))
        # print(cz, cz2)
        return np.array([cx, cy, cz])

    # Calculate the relevant joint angles from the image
    def detect_joint_angles(self, yz_image, xz_image):
        a = self.pixel2meter(yz_image)
        # Obtain the centre of each coloured blob 
        circle1Pos = a * self.detect_green(yz_image, xz_image)
        circle2Pos = a * self.detect_yellow(yz_image, xz_image) 
        circle3Pos = a * self.detect_blue(yz_image, xz_image) 
        circle4Pos = a * self.detect_red(yz_image, xz_image)

        vector_circle1_circle2 = (circle2Pos - circle1Pos)
        vector_circle2_circle3 = (circle3Pos - circle2Pos)
        vector_circle3_circle4 = (circle4Pos - circle3Pos)

        print('=== Circle Positions ===\nCircle1Pos: {}\nCircle2Pos: {}\nCircle3Pos: {}\nCircle4Pos: {}\n=== ==='.format(
            circle1Pos, circle2Pos, circle3Pos, circle4Pos))

        print('=== Vector Positions ===\nCircle1 to Circle2: {}\nCircle2 to Circle3: {}\nCircle3 to Circle4: {}\n===   ==='.format(
            vector_circle1_circle2, vector_circle2_circle3, vector_circle3_circle4))

        # ===== for drawing detecting blob centers on image ====

        circle1Pos_img = self.detect_green(yz_image, xz_image)
        circle2Pos_img = self.detect_yellow(yz_image, xz_image) 
        circle3Pos_img = self.detect_blue(yz_image, xz_image) 
        circle4Pos_img = self.detect_red(yz_image, xz_image)

        image_with_centers = cv2.circle(self.xz_image, (circle1Pos_img[0], circle1Pos_img[2]), 2, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (circle2Pos_img[0], circle2Pos_img[2]), 2, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (circle3Pos_img[0], circle3Pos_img[2]), 2, (255, 255, 255), cv2.FILLED)
        image_with_centers = cv2.circle(image_with_centers, (circle4Pos_img[0], circle4Pos_img[2]), 2, (255, 255, 255), cv2.FILLED)

        cv2.imshow('Images with blob centers', image_with_centers)

        # ===== Angles =====

        print('=== ANGLES ===')
        ja2 = - np.arctan2(circle2Pos[0] - circle3Pos[0], circle2Pos[2] - circle3Pos[2])
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