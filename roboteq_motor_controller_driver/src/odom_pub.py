#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Twist
import tf
import numpy as np

class Odometer:
    def __init__(self):
        rospy.init_node('odometer', anonymous=True)
        self.wheel_radius = 0.154  # in meters
        self.wheel_base = 0.56  # in meters
        self.last_time = rospy.Time.now()
        self.last_speed_left = 0
        self.last_speed_right = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.speed_left_sub = rospy.Subscriber("/hall_speed", Float32, self.speed_callback)
        # self.speed_right_sub = rospy.Subscriber("right_speed", Float32, self.right_speed_callback)
        self.odom_broadcaster = tf.TransformBroadcaster()

    def speed_callback(self, msg):
        speed_left = msg.data
        elapsed_time = (rospy.Time.now() - self.last_time).to_sec()
        self.vx = (speed_left + self.vy * self.theta) * np.cos(self.theta)
        self.vy = (speed_left + self.vy * self.theta) * np.sin(self.theta)
        self.vtheta = (speed_left / self.wheel_base) * np.cos(self.theta)
        self.last_time = rospy.Time.now()

    def right_speed_callback(self, msg):
        speed_right = msg.data
        elapsed_time = (rospy.Time.now() - self.last_time).to_sec()
        self.vx = (speed_right + self.vy * self.theta) * np.cos(self.theta)
        self.vy = (speed_right + self.vy * self.theta) * np.sin(self.theta)
        self.vtheta = (speed_right / self.wheel_base) * np.sin(self.theta)
        self.last_time = rospy.Time.now()

    def publish_odometry(self):
        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.last_time).to_sec()
        self.x += self.vx * elapsed_time
        self.y += self.vy * elapsed_time
        self.theta += self.vtheta * elapsed_time
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
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
        odom.pose.pose = (Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        # odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vtheta))

        # publish the message
       
