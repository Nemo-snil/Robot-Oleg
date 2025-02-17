#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

def bluetooth_listener():
    rospy.init_node('robot', anonymous=True)

    def get_msg(data):
        try:
            speed = Twist()
            text = data.data.split(',')

            speed.linear.x = float(text[1]) / 2
            speed.linear.y = -float(text[0]) / 2
            speed.linear.z = float(text[2]) 
            speed.angular.x = float(text[3])
            speed.angular.y = float(text[4])
            speed.angular.z = -float(text[5])

            speed_pub.publish(speed)
        except IndexError:
            if len(data.data) > 4: return
            rospy.loginfo(data.data)
            if data.data == "UP":
                rospy.wait_for_service('lift_up')
                try:
                    service = rospy.ServiceProxy('lift_up', Empty)
                    out = service()
                except rospy.ServiceException:
                    print("Error")
            else:
                rospy.wait_for_service('lift_down')
                try:
                    service = rospy.ServiceProxy('lift_down', Empty)
                    out = service()
                except rospy.ServiceException:
                    print("Error")

    speed_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    bluetooth_sub = rospy.Subscriber('/bluetooth', String, get_msg)

    rospy.spin()
 
if __name__ == '__main__':
    bluetooth_listener()