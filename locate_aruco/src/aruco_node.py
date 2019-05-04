#! /usr/bin/env python

import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import Image




# Subscribe to /usb_cam/image_raw, which publishes sensor_msgs/Image