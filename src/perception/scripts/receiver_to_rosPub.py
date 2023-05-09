#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import socket
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

global ip_address, port, server_socket

def setup_connection(Aip_address="0.0.0.0", Aport=1234):
    global ip_address, port, img
    # Define the local address and port to bind to
    # Define the IP address and port to listen on
    ip_address = Aip_address  # Listen on all available network interfaces
    port = Aport  # Use the same port number as the sender

def close_connection():
    global server_socket
    #Close the socket
    server_socket.close()
    cv2.destroyAllWindows()

def get_and_pub_image():
    global ip_address, port, server_socket
    # Create a socket to receive data
    server_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    server_socket.bind((ip_address, port))
    try:
        # Receive the number of frames sent by the sender
        n_frames, _ = server_socket.recvfrom(1024)
        n_frames = int(n_frames.decode())
        print(n_frames)
        #if(n_frames < 43) : continue

        # Receive all the frames sent by the sender
        frames = []
        for i in range(n_frames):
            frame, _ = server_socket.recvfrom(1024)
            frames.append(frame)

        # Concatenate the received frames into a single byte array
        img_bytes = b''.join(frames)

        # Decode the byte array into an image using OpenCV  
        img = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_COLOR)
        # convert image to ROS message
        ros_img = CvBridge().cv2_to_imgmsg(img, "bgr8")
        # Publish image
        pub.publish(ros_img)
        # Show the image
        #cv2.imshow("Received Image", img)
        # Check for a key press and terminate if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            check=True
    except:
        pass

# Init ROS node
rospy.init_node("camera_pub", anonymous=True)
pub = rospy.Publisher("/camera/img", Image, queue_size=2)

if __name__=="__main__":
    setup_connection()
    while not rospy.is_shutdown():
        try:
            # Get image
            img = get_and_pub_image()
            #rospy.spin()
        except KeyboardInterrupt:
            break   
    ## Close connection
    close_connection()
