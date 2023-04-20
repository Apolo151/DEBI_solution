#!/usr/bin/env python3
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


def image_callback(ros_image):
    # Convert the ROS message to an image
    try:
        # Load the image
        img = CvBridge().imgmsg_to_cv2(ros_image, "bgr8")
        # Convert to GrayScale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Define coordinates of the 4 corners of the mapping from front view to bird's eye view
        # from top left to top right anti clockwise
        pts1 = np.float32([[0, 255], [0, 600],
                       [500, 600], [500, 470]])
        pts2 = np.float32([[0, 0], [0, 600],
                       [500, 600], [500, 600]])
        

        # Apply Perspective Transform Algorithm
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        result = cv2.warpPerspective(img, matrix, (500, 600))

        # Display the images
        cv2.imshow('Original', img)
        cv2.imshow('Ball Localization', result)
        cv2.waitKey(3)
        #cv2.destroyAllWindows()

    except Exception as e:
        print(e)


if __name__ == '__main__':
    # Initialize the ROS node and the subscriber
    rospy.init_node("ip_camera_subscriber", anonymous=True)
    sub = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    # Keep the ROS node running
    rospy.spin()