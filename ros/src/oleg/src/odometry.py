#!/usr/bin/env python
# license removed for brevity
import rospy
from math import sin, cos, pi
from tf import TransformBroadcaster, transformations
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import TransformStamped, Quaternion

TICK_TO_METR = pi/10/1440
TICK_TO_RADIAN = 2*pi/9080 #TODO Сделать превращение тики в радианы

rospy.init_node('odometry_publisher', anonymous=True)

def get_encoder(encoder: Int32MultiArray):
    global previous_time, previous_x, previous_y, previous_th, x, y, th

    current_time = rospy.Time.now()
    dt = (current_time-previous_time).to_sec()

    x1 = y1 = (-encoder.data[0] + encoder.data[3]) / 2 * 2**0.5 / 2
    x2 = y2 = (encoder.data[1] - encoder.data[2]) / 2 * 2**0.5 / 2

    current_x = (x1+x2)*TICK_TO_METR
    current_y = -(y1-y2)*TICK_TO_METR
    current_th = (encoder.data[0]+encoder.data[1]+encoder.data[2]+encoder.data[3]) / 4 * TICK_TO_RADIAN

    vx = (current_x - previous_x) / dt
    vy = (current_y - previous_y) / dt
    vth = (current_th - previous_th) / dt

    previous_x = current_x
    previous_y = current_y
    previous_th = current_th

    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x+=delta_x
    y+=delta_y
    th+=delta_th

    
    odom_quat = transformations.quaternion_from_euler(0, 0, th)
    
    odom_trans = TransformStamped()
    odom_trans.header.stamp = current_time
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "base_link"

    odom_trans.transform.translation.x = x
    odom_trans.transform.translation.y = y
    odom_trans.transform.translation.z = 0
    
    odom_trans.transform.rotation.x = odom_quat[0]
    odom_trans.transform.rotation.y = odom_quat[1]
    odom_trans.transform.rotation.z = odom_quat[2]
    odom_trans.transform.rotation.w = odom_quat[3]


    odom_broadcaster.sendTransform((x, y, 0), odom_quat, current_time, "base_link", "odom")

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0
    
    odom.pose.pose.orientation.x = odom_quat[0]
    odom.pose.pose.orientation.y = odom_quat[1]
    odom.pose.pose.orientation.z = odom_quat[2]
    odom.pose.pose.orientation.w = odom_quat[3]

    odom.child_frame_id = "base_link"
    odom.twist.twist.linear.x = vx
    odom.twist.twist.linear.y = vy
    odom.twist.twist.angular.z = vth

    odom_pub.publish(odom)

    previous_time = current_time


encoder_sub = rospy.Subscriber('/encoder', Int32MultiArray, get_encoder)
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
odom_broadcaster = TransformBroadcaster()

current_time = rospy.Time.now()
previous_time = rospy.Time.now()

previous_x = 0.0
previous_y = 0.0
previous_th = 0.0

x = 0.0
y = 0.0
th = 0.0

rospy.spin()
