from cv2 import aruco
import cv2

if __name__ == '__main__':
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    

    img = aruco.drawMarker(aruco_dict, 1, 700)
    cv2.imwrite('1_marker.jpg', img)