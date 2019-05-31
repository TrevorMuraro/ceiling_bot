#! /usr/bin/env python

import rospy
import cv_bridge
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image
import numpy as np



# Launch file needs to launch usb_cam_node: rosrun usb_cam usb_cam_node
# Subscribe to /usb_cam/image_raw, which publishes sensor_msgs/Image

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_250)

# The grid board type we're looking for
board = aruco.GridBoard_create(
        markersX = 1,
        markersY = 1,
        markerLength = 0.168, # In meters
        markerSeparation = 0.1, # In meters
        dictionary = ARUCO_DICT
    )

rvecs, tvecs = None, None

CAMERA_MATRIX = 

DISTORTION_COEFFICIENTS = 

class ArucoNode():

    def __init__(self):

        #rospy.loginfo("Starting node")

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, callback=self.image_callback)
        self.debug_image_pub_1 = rospy.Publisher('/locate_aruco/aruco_debug_stage_1', Image, queue_size=10)
        self.debug_image_pub_2 = rospy.Publisher('/locate_aruco/aruco_debug_stage_2', Image, queue_size=10)

        self.bridge = cv_bridge.CvBridge()

        rospy.spin()

    def image_callback(self, img_msg):

        #rospy.loginfo("Received image")

        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")

        gray_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

        """
        test_img = aruco.drawDetectedMarkers(cv_img, corners, borderColor=(0, 0, 255))
        rosified_test_img = self.bridge.cv2_to_imgmsg(test_img, encoding="rgb8")

        self.debug_image_pub_1.publish(rosified_test_img)
        """

        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.168, CAMERA_MATRIX, DISTORTION_COEFFICIENTS)


        rospy.loginfo("rvecs: {rvecs}, tvecs: {tvecs}, _objPoints: {points}".format(rvecs=rvecs, tvecs=tvecs, points=_objPoints))

        if rvecs is not None:
            test_img_2 = aruco.drawAxis(cv_img, CAMERA_MATRIX, DISTORTION_COEFFICIENTS, rvecs[0], tvecs[0], 0.168)
            rosified_test_img_2 = self.bridge.cv2_to_imgmsg(test_img_2, encoding="rgb8")

            self.debug_image_pub_2.publish(rosified_test_img_2)



if __name__ == '__main__':
    rospy.init_node('aruco_node')

    try:
        n = ArucoNode()
    except rospy.ROSInterruptException:
        pass