#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Twist
import tf
import numpy as np
global wheel_radius ,wheel_base, last_time, last_speed_left, last_speed_right, x,y,theta ,vx ,vy, vtheta
wheel_radius = 0.154  # in meters
wheel_base = 0.56  # in meters
last_speed_left = 0
last_speed_right = 0
x = 0.0
y = 0.0
theta = 0.0
vx = 0.0
vy = 0.0
vtheta = 0.0

def left_speed_callback( msg):
    last_time = rospy.Time.now()

    speed_left = msg.data
    elapsed_time = (rospy.Time.now() - last_time).to_sec()
    vx = (speed_left + vy * theta) * np.cos(theta)
    vy = (speed_left + vy * theta) * np.sin(theta)
    vtheta = (speed_left / wheel_base) * np.cos(theta)
    last_time = rospy.Time.now()
def right_speed_callback( msg):
    speed_right = msg.data
    elapsed_time = (rospy.Time.now() - last_time).to_sec()
    vx = (speed_right + vy * theta) * np.cos(theta)
    vy = (speed_right + vy * theta) * np.sin(theta)
    vtheta = (speed_right / wheel_base) * np.sin(theta)
    last_time = rospy.Time.now()
    

def __init__():
    rospy.init_node('odometer', anonymous=True)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    speed_left_sub = rospy.Subscriber("/feedback", Float32, left_speed_callback)
    # speed_right_sub = rospy.Subscriber("right_speed", Float32, right_speed_callback)
    rospy.loginfo(speed_left_sub[0])



def publish_odometry():
    odom_broadcaster = tf.TransformBroadcaster()
    current_time = rospy.Time.now()
    elapsed_time = (current_time - last_time).to_sec()
    x += vx * elapsed_time
    y += vy * elapsed_time
    theta += vtheta * elapsed_time
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    odom_broadcaster.sendTransform(
        (x, y, 0),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    # set the position
    odom.pose.pose = (Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    # odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vtheta))

    # publish the message
while True:
    publish_odometry()