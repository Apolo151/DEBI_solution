#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

global circles, circle_msg
circles = None
circle_msg = PointStamped()

BALL_RADIUS = 5.5*(10**(-2))*0.5 # meter
FOCAL_LENGTH = 0.00304 # meter
SENSOR_WIDTH = 2.813*(10**(-3)) # meter

# Define a callback function to convert the ROS message to an image and display it
def image_callback(ros_image):
    #black_bg = np.zeros((480,640,1), np.uint8)
    global circle_msg, circles
    try:
        # Convert the ROS message to an image
        main_img = CvBridge().imgmsg_to_cv2(ros_image, "bgr8")
        ## Blur img
        main_img=cv2.medianBlur(main_img, 5)
        #main_img= cv2.GaussianBlur(main_img,(5,5),0)

        # Convert to HSV and get gray hsv
        hsv = cv2.cvtColor(main_img, cv2.COLOR_BGR2HSV)
        hsv_gray = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)

        ''' Filter background color
        walls_lower = np.array([5, 60, 61])
        walls_upper = np.array([20, 210, 96])
        side_walls_lower = np.array([3, 0, 0])
        side_walls_upper = np.array([19, 60, 255])
        ground_lower = np.array([0, 0, 10])
        ground_upper = np.array([0, 0, 200])
        walls_mask = cv2.inRange(hsv, walls_lower, walls_upper)
        ground_mask = cv2.inRange(hsv, ground_lower, ground_upper)
        side_walls_mask = cv2.inRange(hsv, side_walls_lower, side_walls_upper)
        res = cv2.bitwise_and(hsv, hsv, mask=walls_mask)
        res = cv2.bitwise_not(res)
        res2 = cv2.bitwise_and(hsv, hsv, mask=ground_mask)
        res2 = cv2.bitwise_not(res2)
        res3 = cv2.bitwise_and(hsv, hsv, mask=side_walls_mask)
        res3 = cv2.bitwise_not(res3)
        res_f = cv2.bitwise_and(res, res2)
        res_f = cv2.bitwise_and(res_f, res3)
        #res = cv2.bitwise_and(res, res, mask=white_mask)
        #res = cv2.bitwise_not(res)
        #res = cv2.bitwise_and(res, edges)
        res_f = cv2.cvtColor(res_f, cv2.COLOR_BGR2GRAY)'''

        edges = cv2.Canny(hsv_gray, 0, 180, apertureSize=3)
        
        #contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        #cnt = sorted(cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2], key=cv2.contourArea)[-1]
        # Draw Contours on Black Background
        #cv2.drawContours(black_bg, [cnt], -1, (255,255,255), -1)
        #result_img = cv2.bitwise_and(main_img, main_img, mask=black_bg)
        #result_img = cv2.cvtColor(result_img, cv2.COLOR_BGR2GRAY)
        #black_bg = cv2.cvtColor(black_bg, cv2.COLOR_BGR2GRAY)
        #edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        #edges2 = cv2.dilate(edges, None, iterations=2)
        #edges = edges2-edges
        #edges = cv2.dilate(edges, None, iterations=1)
        #edges = cv2.erode(edges, None, iterations=1)

        # Perform Hough Circle Transform
        circles=cv2.HoughCircles(hsv_gray,cv2.HOUGH_GRADIENT,1,400,param1=45,param2=19,minRadius=5,maxRadius=80)

        # Define circles msg
        circle_msg.header = ros_image.header
        circle_msg.point.x = -1
        circle_msg.point.y = -1
        circle_msg.point.z = -1 # Radius
        # Loop over the detected circles and draw them on the original image
        if circles is not None:
            # set msg parameters to first circle in list
            # maybe sort circles and get (bigger radius - closest to center - most consistent) (TODO)
            circle_msg.point.x = circles[0][0][0]
            circle_msg.point.y = circles[0][0][1]
            circle_msg.point.z = circles[0][0][2] # Radius
            ## Draw circles
            circles=np.uint16(np.around(circles))
            for i in circles[0,:]:
                cv2.circle(main_img,(i[0],i[1]),i[2],(0,255,200),1)
                cv2.circle(main_img,(i[0],i[1]),2,(0,0,255),1)
        
        # Publish circles coordinates and radius
        pub.publish(circle_msg)
        #print(circle_msg.point.z)
        # Display Images
        cv2.imshow("Main Image", main_img)
        #cv2.imshow("Canny", edges)
        #cv2.imshow("HSV gray", hsv_gray)
        cv2.waitKey(3)
    except Exception as e:
        print(e)
    
# Define Subs and Pubs
rospy.init_node("circles_detector", anonymous=True)
pub = rospy.Publisher("circles_coors", PointStamped, queue_size=1)
sub = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

if __name__=="__main__":
    # Keep the ros node running
    rospy.spin()