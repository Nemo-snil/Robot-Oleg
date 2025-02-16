#!/usr/bin/env python
# license removed for brevity
import rospy
from robot.msg import MarkerArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker as VisualMarker
from cv_bridge import CvBridge

def visual_marker():
    rospy.init_node('visual_marker', anonymous=True)

    def aruco_callback(markers: MarkerArray):
        for marker in markers.markers:
            visual_msg = VisualMarker()
            visual_msg.header.frame_id = "base_camera"
            visual_msg.header.stamp = rospy.Time.now()
            visual_msg.ns = "chairs"
            visual_msg.id = int(marker.id)
            visual_msg.type = VisualMarker.CUBE
            visual_msg.action = VisualMarker.ADD

            visual_msg.pose.position.x = marker.pose.position.z/1000 + 0.24
            visual_msg.pose.position.y = -marker.pose.position.x/1000
            visual_msg.pose.position.z = 0.385

            visual_msg.pose.orientation.w, visual_msg.pose.orientation.x, visual_msg.pose.orientation.y, visual_msg.pose.orientation.z = quaternion_from_euler(-euler_from_quaternion([marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z])[1], 0, 3.14)

            visual_msg.scale.x = 0.48
            visual_msg.scale.y = 0.49
            visual_msg.scale.z = 0.77

            visual_msg.color.a = 1.0
            visual_msg.color.r = 156
            visual_msg.color.g = 156
            visual_msg.color.b = 156

            pub_visual_aruco.publish(visual_msg)

    sub_aruco = rospy.Subscriber('/markers', MarkerArray, aruco_callback)
    pub_visual_aruco = rospy.Publisher('/visualization_marker', VisualMarker, queue_size=10)

    bridge = CvBridge()

    rospy.spin()

if __name__ == '__main__':
    visual_marker()