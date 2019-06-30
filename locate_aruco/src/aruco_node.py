#! /usr/bin/env python

import rospy
import cv_bridge
import cv2
from cv2 import aruco #pylint:disable=no-name-in-module
from sensor_msgs.msg import Image, CameraInfo
from aruco_msgs.msg import ArucoTransform, ArucoTransformArray #pylint:disable=import-error
import numpy as np
from math import sin, cos


# Launch file needs to launch usb_cam_node: rosrun usb_cam usb_cam_node
# To get around the error, first roslaunch usb_cam usb_cam-test.launch
# Subscribe to /usb_cam/image_raw, which publishes sensor_msgs/Image

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_250)

ARUCO_SQUARE_SIZE = 0.097

# The grid board type we're looking for
board = aruco.GridBoard_create(
        markersX = 1,
        markersY = 1,
        markerLength = ARUCO_SQUARE_SIZE, # In meters
        markerSeparation = 0.1, # In meters
        dictionary = ARUCO_DICT
    )

rvecs, tvecs = None, None



class ArucoNode():

    def __init__(self):

        #rospy.loginfo("Starting node")

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, callback=self.image_callback)
        self.caminfo_sub = rospy.Subscriber('/usb_cam/camera_info', CameraInfo, callback=self.caminfo_callback)
        self.debug_image_pub = rospy.Publisher('/locate_aruco/aruco_debug', Image, queue_size=10)
        self.markers_pub = rospy.Publisher('/locate_aruco/aruco_transforms', ArucoTransformArray, queue_size=10)

        self.bridge = cv_bridge.CvBridge()

        self.CAMERA_MATRIX = None
        self.DISTORTION_COEFFICIENTS = None
        self.have_cam_info = False

        rospy.spin()

    def caminfo_callback(self, caminfo_msg):

        if self.have_cam_info:
            pass
        else:
            if caminfo_msg.K == [0, 0, 0, 0, 0, 0, 0, 0, 0]:
                rospy.logwarn("CameraInfo message has K matrix all zeros")
            else:
                self.DISTORTION_COEFFICIENTS = caminfo_msg.D
                self.CAMERA_MATRIX = np.zeros((3, 3))

                for i in range(0, 3):
                    for j in range(0, 3):
                        self.CAMERA_MATRIX[i][j] = caminfo_msg.K[i * 3 + j]

                self.have_cam_info = True


    def image_callback(self, img_msg):

        if self.have_cam_info:

            output_msg = ArucoTransformArray()
            output_msg.header.seq = img_msg.header.seq
            output_msg.header.stamp = img_msg.header.stamp
            output_msg.header.frame_id = img_msg.header.frame_id

            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")

            gray_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, ARUCO_SQUARE_SIZE, self.CAMERA_MATRIX, self.DISTORTION_COEFFICIENTS)


            #Tvecs: +x is to the right (as seen by camera), +y is down (as seen by camera), +z is further from camera
            #Rvecs: 

            rospy.loginfo("ids: {ids}, rvecs: {rvecs}, tvecs: {tvecs}".format(ids=ids, rvecs=rvecs, tvecs=tvecs))


            test_img = cv_img
            if ids is not None:
                for i in range(len(ids)):
                    test_img = aruco.drawAxis(test_img, self.CAMERA_MATRIX, self.DISTORTION_COEFFICIENTS, rvecs[i], tvecs[i], ARUCO_SQUARE_SIZE)
            rosified_test_img = self.bridge.cv2_to_imgmsg(test_img, encoding="rgb8")
            self.debug_image_pub.publish(rosified_test_img)

            if ids is not None:
                for i in range(len(ids)):
                    marker_pose = ArucoTransform()
                    marker_pose.id = ids[i][0]

                    marker_pose.transform.translation.x = tvecs[i][0][0]
                    marker_pose.transform.translation.y = tvecs[i][0][1]
                    marker_pose.transform.translation.z = tvecs[i][0][2]

                    #Conversion from axis-angle to quaternion
                    #Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/

                    #Get total angle of rotation from axis-angle
                    angle = cv2.norm(rvecs[i][0])
                    #Normalize axis-angle vector
                    axis = (rvecs[i][0][0] / angle, rvecs[i][0][1] / angle, rvecs[i][0][2] / angle)

                    x = axis[0] * sin(angle / 2)
                    y = axis[1] * sin(angle / 2)
                    z = axis[2] * sin(angle / 2)
                    w = cos(angle / 2)

                    marker_pose.transform.rotation.x = x
                    marker_pose.transform.rotation.y = y
                    marker_pose.transform.rotation.z = z
                    marker_pose.transform.rotation.w = w

                    output_msg.transforms.append(marker_pose)

            self.markers_pub.publish(output_msg)

            





if __name__ == '__main__':
    rospy.init_node('aruco_node')

    try:
        n = ArucoNode()
    except rospy.ROSInterruptException:
        pass