#! /usr/bin/env python

import rospy
import cv_bridge
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image



# Launch file needs to launch usb_cam_node
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

class ArucoNode():

    def __init__(self):

        #rospy.loginfo("Starting node")

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, callback=self.image_callback)

        self.bridge = cv_bridge.CvBridge()

        rospy.spin()

    def image_callback(self, img_msg):

        #rospy.loginfo("Received image")

        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")

        gray_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

        rospy.loginfo("corners: {corners}, ids: {ids}, rejectedImgPoints: {rejects}".format(corners=corners, ids=ids, rejects=rejectedImgPoints))

        """
        if ids is not None and len(ids) == 5:
            for i, corner in zip(ids, corners):
                rospy.loginfo("id: {id}, corner: {corner}".format(id=i, corner=corner))
        """

        # aruco.estimatePoseSingleMarkers

if __name__ == '__main__':
    rospy.init_node('aruco_node')

    try:
        n = ArucoNode()
    except rospy.ROSInterruptException:
        pass