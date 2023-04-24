#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Save image
        #filename = rospy.get_param(, 'image.jpg')
        cv2.imwrite(str(rospy.get_time())+"img.jpg", cv_image)
        rospy.loginfo("Saved image to {}".format("NAME"))
        rospy.sleep(2)

if __name__ == '__main__':
    image_saver = ImageSaver()
    rospy.spin()
