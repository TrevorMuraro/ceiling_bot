#! /usr/bin/env python

import rospy
import cv_bridge
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image



# Launch file needs to launch usb_cam_node
# Subscribe to /usb_cam/image_raw, which publishes sensor_msgs/Image

class ArucoNode():

    def __init__(self):

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, callback=self.image_callback)

        self.bridge = cv_bridge.CvBridge()

    def image_callback(self, img_msg):

        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")