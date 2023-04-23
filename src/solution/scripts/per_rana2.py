#!/usr/bin/env python3
# Import the necessary libraries
#from Import import *
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


# Define the messgae 
FeedBackMsg=Float32MultiArray()
last_error=[0,0]
def image_callback(ros_image):
    global FeedBackMsg,last_error
    # Convert the ROS message to an image
    main_img = CvBridge().imgmsg_to_cv2(ros_image, "bgr8")
    # Apply Gaussian blur for filtering noise
    main_img = cv2.GaussianBlur(main_img, (5, 5), 0)
    # # crop image to its half
    # x = int(main_img.shape[1]-(main_img.shape[1]*0.65))   # Starting point from the left end
    # y = int(main_img.shape[0]-(main_img.shape[0]*0.5))  # Starting point from the bottom end
    # w = int(main_img.shape[1]*0.65)  # Width of the crop
    # h = int(main_img.shape[0]*0.45)  # Height of the crop
    # main_img = main_img[y:y+h, x:x+w]
    # main_img = main_img[0:main_img.shape[0], 0:int(main_img.shape[1]*0.40)]
    # hsv to segment background
    hsv = cv2.cvtColor(main_img, cv2.COLOR_BGR2HSV)
    hsv_gray = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)
    # Perform Hough Circle Transform
    circles=cv2.HoughCircles(hsv_gray,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
    if circles is not None:
        #publish that ball is detected
        BallFlagMsg=Bool()
        BallFlagMsg.data=True
        BallFlag.publish(BallFlagMsg)
        #end of publish
        #largest[0]-->x    largest[1]--->y
        circles = np.round(circles[0, :]).astype("int")
        largest_circle = max(circles, key=lambda circle: circle[2])
        center = (largest_circle[0], largest_circle[1])
        radius = largest_circle[2]
        # draw the circle around the ball on the frame
        cv2.circle(main_img, center, radius, (0,255,0), 2)
        cv2.circle(main_img, center, 2, (0,0,255), 3)
        error = (center[0] - (main_img.shape[1] / 2))
        last_error=[radius,error]
        FeedBackMsg.data=last_error
    else:
        #publish that ball is not detected
        BallFlagMsg=Bool()
        BallFlagMsg.data=False
        BallFlag.publish(BallFlagMsg)
    pub.publish(FeedBackMsg)
    # Display the image
    cv2.imshow("CameraStream", main_img)
    cv2.waitKey(3)
# Initialize the ROS node, the subscriber and the publisher
rospy.init_node("TurtleCamera", anonymous=True)
sub = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
pub = rospy.Publisher("/Ball_info", Float32MultiArray, queue_size=0)
BallFlag=rospy.Publisher("/BallFlag",Bool,queue_size=0)
# Keep the ROS node running
rospy.spin()