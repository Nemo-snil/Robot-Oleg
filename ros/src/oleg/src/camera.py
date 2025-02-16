#!/usr/bin/env python
# license removed for brevity
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

def camera():
    rospy.init_node('camera', anonymous=True)

    pub_image = rospy.Publisher('/image', Image, queue_size=1)

    bridge = CvBridge()
    
    rate = rospy.Rate(30)

    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        ret, image = cap.read()
        pub_image.publish(bridge.cv2_to_imgmsg(image, 'bgr8'))

        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    camera()
