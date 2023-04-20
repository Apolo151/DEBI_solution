#!/usr/bin/env python3
import sys
# Import the necessary libraries
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# Define a callback function to convert the ROS message to an image and display it
def image_callback(ros_image):

    # Convert the ROS message to an image
    try:
        main_img = CvBridge().imgmsg_to_cv2(ros_image, "bgr8")
        #blur1= cv2.GaussianBlur(main_img,(5,5),0)


        # hsv to segment background
        hsv = cv2.cvtColor(main_img, cv2.COLOR_BGR2HSV)
        hsv_gray = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)
        #lower = np.array([0, 0, 0])
        #upper = np.array([255, 60, 255])
        #mask = cv2.inRange(hsv, lower, upper)
        #res = cv2.bitwise_and(main_img, main_img, mask=mask)
        #res = cv2.bitwise_not(res)
        #res = cv2.bitwise_and(res, edges)


        gray=cv2.cvtColor(main_img,cv2.COLOR_BGR2GRAY)
        # Perform histogram equalization using cv2.equalizeHist()
        eq_img = cv2.equalizeHist(gray)

        # Blur the image using a median filter
        img=cv2.medianBlur(eq_img,3)

        # declare the gray img as the equalized gray image
        gray=eq_img
        # Perform Hough Circle Transform
        circles=cv2.HoughCircles(hsv_gray,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
        circles=np.uint16(np.around(circles))

        # Detect edges using Canny
        edges = cv2.Canny(hsv_gray, 0, 180, apertureSize=3)
        
        # Loop over the detected circles and draw them on the original image
        for i in circles[0,:]:
            cv2.circle(main_img,(i[0],i[1]),i[2],(0,255,0),2)
            cv2.circle(main_img,(i[0],i[1]),2,(0,0,255),3)

        print(circles[0])

        # Apply HoughLines function to detect lines
        lines = cv2.HoughLines(edges, 1, np.pi/180, 200)

        # Loop over the detected lines and draw them on the original image
        for line in lines:
            rho, theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(main_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
        
        cv2.imshow("Camera hist", main_img)
        cv2.waitKey(3)

    except Exception as e:
        print(e)
    # Display the image
    cv2.imshow("Camera Stream", gray)
    

def get_circles():
    # Initialize the ROS node and the subscriber
    sub = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

if __name__=="__main__":
    rospy.init_node("ip_camera_subscriber", anonymous=True)
    # Keep the ros node running
    rospy.spin()