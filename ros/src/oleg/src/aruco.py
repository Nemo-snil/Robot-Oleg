#!/usr/bin/env python
# license removed for brevity
import rospy
from robot.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge
import numpy as np
import cv2

def aruco():
    camMatrix = np.array([
        [378.38115568, 0, 344.14743078],
        [0, 379.21272761, 222.24596086],
        [0, 0, 1]
    ], dtype=np.float32)

    distCoeffs = np.array([0.14644642, -0.16629221, 0.00098132, -0.00050964,  0.06482885], dtype=np.float32)

    marker_length = 190

    obj_points = np.array([
        [-marker_length / 2.0, marker_length / 2.0, 0],
        [marker_length / 2.0, marker_length / 2.0, 0],
        [marker_length / 2.0, -marker_length / 2.0, 0],
        [-marker_length / 2.0, -marker_length / 2.0, 0]
    ], dtype=np.float32)

    # Create the ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    rospy.init_node('aruco', anonymous=True)

    def video_callback(ros_image):
        try:
            image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"Ошибка конвертации: {e}")
            return
        
        corners, ids, rejected = detector.detectMarkers(image)
        
        if ids is not None:
            # cv2.aruco.drawDetectedMarkers(image, corners, _id)
            msg_array = MarkerArray()
            for i, _id in enumerate(ids):
                msg = Marker()
                
                _, rvec, tvec = cv2.solvePnP(obj_points, corners[i], camMatrix, distCoeffs)
                rvec = rvec[0], -rvec[2] if rvec[0] < 0 else rvec[2], -rvec[1] if rvec[0] < 0 else rvec[1]

                msg.id = int(_id)
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = tvec
                msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = quaternion_from_euler(rvec[0], rvec[1], rvec[2])
                msg_array.markers.append(msg)

            pub_aruco.publish(msg_array)

    bridge = CvBridge()
    sub_image = rospy.Subscriber('/image', Image, video_callback)
    pub_aruco = rospy.Publisher('/markers', MarkerArray, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    aruco()
