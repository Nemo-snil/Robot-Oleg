#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from robot.srv import *
from simple_pid import PID
from robot.msg import MarkerArray, ChairLegs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Bool

RADIAN_TO_ANGLE = 57.29577951308232

class Transporter:
    def __init__(self):
        rospy.init_node('robot', anonymous=True)
        rospy.loginfo("run")

        self.speed_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.marker_sub = rospy.Subscriber('/markers', MarkerArray, self.get_marker)
        self.leg_sub = rospy.Subscriber('/legs', ChairLegs, self.get_leg)

        self.marker_position: Point = Point()
        self.marker_position.x = 0
        self.marker_position.z = 1500
        self.marker_orientation = [0, 0, 0]
        self.is_new_marker = False

        self.leg_position: ChairLegs = ChairLegs()

    def get_leg(self, legs: ChairLegs):
        self.leg_position = legs

    def get_marker(self, markers: MarkerArray):
        for marker in markers.markers:
            if marker.id != 1: continue
            # Получаем положение с камеры
            self.marker_position: Point = markers.markers[0].pose.position
            self.marker_orientation = euler_from_quaternion([markers.markers[0].pose.orientation.w, markers.markers[0].pose.orientation.x, markers.markers[0].pose.orientation.y, markers.markers[0].pose.orientation.z])
            self.marker_orientation = self.marker_orientation[0], self.marker_orientation[1], self.marker_orientation[2]
            # Положение для центра робота
            self.marker_position.x += math.sin(self.marker_orientation[1]) * 360
            self.marker_position.z += math.cos(self.marker_orientation[1]) * 360

            self.is_new_marker = True
            #rospy.loginfo(self.marker_orientation)

        #rospy.loginfo(f'{self.marker_orientation[0]*RADIAN_TO_ANGLE} {self.marker_orientation[1]*RADIAN_TO_ANGLE} {self.marker_orientation[2]*RADIAN_TO_ANGLE}')

    def marker_distance(self, position: Point):
        return ( (position.x)**2 + (position.z)**2 )**0.5

    def move_to_next_marker(self, direct):
        speed = Twist()
        if direct == 0:
            speed.linear.x = 1
        elif direct == 1:
            speed.linear.y = 1
        elif direct == 2:
            speed.linear.x = -1
        elif direct == 3:
            speed.linear.y = -1
        
        self.speed_pub.publish(speed)
        rospy.sleep(2)
        self.is_new_marker = False

        while not self.is_new_marker: pass

        while self.marker_distance(self.marker_position) > 0.5:
            speed.linear.x = self.pid_posx(self.marker_position.x)
            speed.linear.y = self.pid_posy(self.marker_position.y)
            
            self.speed_pub.publish(speed)
            rospy.loginfo(f"{speed.linear.x}, {speed.linear.y}")
        self.speed_pub.publish(Twist())

        rospy.loginfo("Приехали")

    def move_with_angle(self, x: float, y: float, angle: float, z: float = 0):
        a = angle * RADIAN_TO_ANGLE
        b = 90 - a

        speed = Twist()

        speed.linear.x = x * math.cos(a / RADIAN_TO_ANGLE) - y * math.cos(b / RADIAN_TO_ANGLE)
        speed.linear.y = x * math.sin(a / RADIAN_TO_ANGLE) + y * math.sin(b / RADIAN_TO_ANGLE) 
        speed.angular.z = z

        self.speed_pub.publish(speed)

    def move_to_chair(self):
        pid_x = PID(.001, 0, 0, setpoint=1500, output_limits=(-.2, .2))
        pid_y = PID(.001, 0, 0, setpoint=0, output_limits=(-.2, .2))
        pid_z = PID(.4, 0, 0, setpoint=0, output_limits=(-.2, .2))

        #while not rospy.is_shutdown() and not (990 < self.marker_distance(self.marker_position) < 1010 and -0.005 < self.marker_orientation[1] < 0.005):
        while not rospy.is_shutdown():
            angle = self.marker_orientation[1]

            x = pid_x(self.marker_position.z)
            y = pid_y(self.marker_position.x)
            z = -pid_z(angle)

            self.move_with_angle(-x, y, angle, z)

            rospy.sleep(0.1)
        
        self.speed_pub.publish(Twist())

    def move_under_chair(self):
        pid_y = PID(.004, 0, 0, setpoint=-60, output_limits=(-0.2, 0.2))

        speed = Twist()
        speed.linear.x = 0.2
        while not rospy.is_shutdown():
            speed.linear.y = pid_y(self.leg_position.left - self.leg_position.right)

            self.speed_pub.publish(speed)
            rospy.sleep(0.1)


    def run(self):
        self.move_to_next_marker(0)


if __name__ == '__main__':
    robot = Transporter()
    robot.move_to_chair()
    #robot.move_under_chair()
    rospy.loginfo("приехали")
    rospy.spin()