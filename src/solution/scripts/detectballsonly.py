#!/usr/bin/env python3
import sys
# Import the necessary libraries
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from tf.transformations import euler_from_quaternion

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

# def image_callback(ros_image):
#     try:
#         # Convert the ROS message to an image
#         try:
#            r_and_error = []
#            main_img = CvBridge().imgmsg_to_cv2(ros_image, "bgr8")
#            mask = np.zeros_like(main_img)

#            gray = cv2.cvtColor(main_img, cv2.COLOR_BGR2GRAY)
#            gray_blurred = cv2.blur(gray, (5, 5))
#            detected_circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, 1, 10, param1=60,
#                                                param2=30, minRadius=1, maxRadius=40)

#            # Draw circles that are detected.
#            if detected_circles is not None:
#                # Convert the circle parameters a, b and r to integers.
#                detected_circles = np.uint16(np.around(detected_circles))

#                for pt in detected_circles[0, :]:
#                    a, b, r = pt[0], pt[1], pt[2]
#                    error = (a - (main_img.shape[1] / 2))
#                    r_and_error.append([error, r])
#                    print(r_and_error)

#                    # Draw the circumference of the circle.
#                    cv2.circle(main_img, (a, b), r, (0, 255, 0), 2)

          
#            # Publish the r_and_error list as a list of lists
#            pub.publish(Float32MultiArray(data=r_and_error))
           

#         except Exception as e:
#             print(f"Exception caught: {e}")


#         # Display the image
#         cv2.imshow("IP Camera Stream", main_img)
#         cv2.waitKey(1)

#     except CvBridgeError as e:
#         print(e)

import math
import numpy as np
import matplotlib.pyplot as plt


x_pos=0
y_pos=0
theta=0

# Known ball size in meters
known_ball_size = 0.09

# Focal length of Raspberry Pi Camera V2 lens in meters
focal_length = 0.00036

# Pixel size of the camera sensor in meters
pixel_size = 0.00000112



def robot_and_curve_par(threshed):
    # Get x and y coordinates of the robot
    x_pixel, y_pixel = robot_coords(threshed)

    # Fit a polynomial curve to the robot's path
    coefficients = np.polyfit(x_pixel, y_pixel, 1)

    # Generate x values for the polynomial fit
    x_fit = np.linspace(x_pixel.min(), x_pixel.max(), len(x_pixel))

    # Predict y values using the polynomial fit
    y_fit = np.polyval(coefficients, x_fit)

    # Do some plotting
    fig = plt.figure(figsize=(12,9))
    plt.subplot(221)
    # Convert x and y coordinates to polar coordinates
    points, angles = to_polar_coords(x_fit, y_fit)
    arrow_length = 100
    x_arrow = arrow_length * np.cos(math.atan(coefficients[0]))
    y_arrow = arrow_length * np.sin(math.atan(coefficients[0]))
    #plt.arrow(0, 0, x_arrow, y_arrow) # , color='red', zorder=2, head_width=10, width=2
    #plt.show()

    # Return the slope and intercept of the linear term
    return coefficients[0], coefficients[1]

def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)

    return dist, angles
# Define a function to convert from image coords to rover coords
def robot_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos, _ = binary_img.nonzero()

    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2).astype(np.float)
    return x_pixel, y_pixel

def odom_callback(odom_msg):
        global x_pos 
        global y_pos
        # Update current position and orientation of the robot
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        x_pos = position.x
        y_pos = position.y
        theta = yaw


def perspect_transform(img, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))  # keep same size as input image
    return warped


def color_thresh_red(img):
    # convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # mask of red (0,50,50) ~ (10, 255, 255) + (170,50,50) ~ (180, 255, 255)
    mask1 = cv2.inRange(hsv, (0, 50, 50), (10, 255, 255))
    mask2 = cv2.inRange(hsv, (170, 50, 50), (180, 255, 255))
    mask = cv2.bitwise_or(mask1, mask2)

    # slice the red
    imask = mask > 0
    red = np.zeros_like(img, np.uint8)
    red[imask] = img[imask]

    return red
def color_thresh_blue(img):
    # convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # mask of blue (100,50,50) ~ (130, 255, 255)
    mask = cv2.inRange(hsv, (100, 50, 50), (130, 255, 255))

    # slice the blue
    imask = mask > 0
    blue = np.zeros_like(img, np.uint8)
    blue[imask] = img[imask]

    return blue


def color_thresh_green(img):
## convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


    mask = cv2.inRange(hsv, (36, 25, 25), (70, 255,255))

## slice the green
    imask = mask>0
    green = np.zeros_like(img, np.uint8)
    green[imask] = img[imask]
    return green




def locate_ball(a, b, r, main_img):
    dst_size = 40
    bottom_offset = 7
    scale=2*dst_size
    # Define color ranges for green, blue, and red
    green_lower = np.array([36, 25, 25])
    green_upper = np.array([70, 255,255])

    blue_lower = np.array([100, 50, 50])
    blue_upper = np.array([130, 255, 255])

    red_lower = np.array([150, 0, 0])
    red_upper = np.array([255, 100, 100])

# Check if the pixel color is within any of the color ranges
    if tuple(main_img[b,a]) == (0, 102, 0):
        color='green'
    if tuple(main_img[b,a]) == (102, 0, 0):
        color='blue'
    if tuple(main_img[b,a]) == (0, 0,102):
        color='red'

    # Use the mask to copy the circle pixels from the original image to the black image
    
    source = np.float32([[45,305], [600 ,305],[459,273], [182,273]])
    destination = np.float32([[main_img.shape[1]/2 - dst_size, main_img.shape[0] - bottom_offset],
                          [main_img.shape[1]/2 + dst_size, main_img.shape[0] - bottom_offset],
                          [main_img.shape[1]/2 + dst_size, main_img.shape[0] - 2*dst_size - bottom_offset], 
                          [main_img.shape[1]/2 - dst_size, main_img.shape[0] - 2*dst_size - bottom_offset],
                          ])
    warped = perspect_transform(main_img, source, destination)
    
    
    cirmap_green= color_thresh_green(warped)
    cv2.imshow("www",warped)
    cirmap_red=color_thresh_red(warped)
    cirmap_blue=color_thresh_blue(warped)
    if cirmap_green.any() and color=='green':   
            print('hi green ')
            # rock_x_pixel,rock_y_pixel=robot_coords(cirmap_green)
            # dist, angles = to_polar_coords(rock_x_pixel, rock_y_pixel)
            angle,c=robot_and_curve_par(cirmap_green)
    elif cirmap_red.any() and color=='red':
            print('hi red') 
            cv2.imshow("ssssssssss",cirmap_red)
            # rock_x_pixel,rock_y_pixel=robot_coords(cirmap_red)
            # dist, angles = to_polar_coords(rock_x_pixel, rock_y_pixel)
            angle,c=robot_and_curve_par(cirmap_red)
    elif cirmap_blue.any()and color=='blue': 
            print('hi bule')  
            # rock_x_pixel,rock_y_pixel=robot_coords(cirmap_blue)
            
            # dist, angles = to_polar_coords(rock_x_pixel, rock_y_pixel)
            angle,c=robot_and_curve_par(cirmap_blue)

    
    angle=math.atan(angle)
    # Convert the radius from pixels to meters
    apparent_size = 2.0 * r * pixel_size

    # Calculate the distance between the camera and the ball
    distance = (known_ball_size * focal_length) / apparent_size

    # # Calculate the angle of the ball from the center of the image
    # angle = math.atan2(b - main_img.shape[0], a - (main_img.shape[1]/2))
    #angle=np.median(angles)
    # Calculate the distance in the x and y directions
    distance_x = (distance * math.cos(angle)+x_pos)+0.09
    distance_y = distance * math.sin(angle)+y_pos
    

    # Calculate the x and y positions of the ball relative to the center of the camera
    x = distance_x
    y = distance_y

    return x, y, distance


def image_callback(ros_image):
    try:
        # Convert the ROS message to an image
        try:
            r_and_error = []
            main_img = CvBridge().imgmsg_to_cv2(ros_image, "bgr8")
            kernel = np.ones((7,7), np.uint8)
            dilation = cv2.dilate(main_img, kernel, iterations=1)
            erosion = cv2.erode(dilation, kernel, iterations=1)
            
            mask = np.zeros_like(main_img)

            gray = cv2.cvtColor(erosion, cv2.COLOR_BGR2GRAY)
            
            gray_blurred = cv2.blur(gray, (5, 5))
            detected_circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, 1, 10, param1=60,
                                               param2=30, minRadius=1, maxRadius=40)

            # Draw circles that are detected.
            if detected_circles is not None:
                # Convert the circle parameters a, b and r to integers.
                detected_circles = np.uint16(np.around(detected_circles))

                for pt in detected_circles[0, :]:
                    a, b, r = pt[0], pt[1], pt[2]
                    error = (a - (main_img.shape[1] / 2))
                    r_and_error.append([error, r])

                    # # Draw the circumference of the circle.
                    #cv2.circle(main_img, (a, b), r, (0, 255, 0), 2)


                    # Calculate the distances between the detected ball and your current position in the x and y directions
                    distance_x, distance_y ,distance= locate_ball(a, b, r,erosion)
                    print("Distance_x: {:.2f} m, Distance_y: {:.2f} m".format(distance_x, distance_y))

                    # Publish the distances as a ROS message
                    pub.publish(Float32MultiArray(data=[error, r, distance_x, distance_y]))

            # Display the image
            cv2.imshow("IP Camera Stream", gray_blurred)
            cv2.waitKey(1)

        except Exception as e:
            print(f"Exception caught: {e}")

    except CvBridgeError as e:
        print(e)




# Initialize the ROS node, the subscriber and the publisher
rospy.init_node("ip_camera_subscriber", anonymous=True)
sub = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
pub = rospy.Publisher("r_and_error_", Float32MultiArray, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)

# Keep the ROS node running
rospy.spin()


