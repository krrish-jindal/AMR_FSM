#!/usr/bin/env python

import math
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, LaserScan
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf2_ros import TransformListener, filter_failure_reasons
from message_filters import Subscriber, TimeSynchronizer

class PointCloudToLaserScanNode:
    def __init__(self):
        self.target_frame = ""
        self.tolerance = 0.01
        self.min_height = -0.2
        self.max_height = 0.15
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.pi / 180.0
        self.scan_time = 1.0 / 30.0
        self.range_min = 0.0
        self.range_max = float('inf')
        self.inf_epsilon = 1.0
        self.use_inf = True

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub = Subscriber("cloud_in", PointCloud2)
        self.pub = rospy.Publisher("scan", LaserScan, queue_size=10)

        self.sub.registerCallback(self.cloud_cb)

    def cloud_cb(self, cloud_msg):
        output = LaserScan()
        output.header = cloud_msg.header
        if self.target_frame != "":
            output.header.frame_id = self.target_frame

        output.angle_min = self.angle_min
        output.angle_max = self.angle_max
        output.angle_increment = self.angle_increment
        output.time_increment = 0.0
        output.scan_time = self.scan_time
        output.range_min = self.range_min
        output.range_max = self.range_max

        ranges_size = int(math.ceil((output.angle_max - output.angle_min) / output.angle_increment))

        if self.use_inf:
            output.ranges = [float('inf')] * ranges_size
        else:
            output.ranges = [output.range_max + self.inf_epsilon] * ranges_size

        try:
            if output.header.frame_id != cloud_msg.header.frame_id:
                cloud_out = do_transform_cloud(cloud_msg, self.tf_buffer.lookup_transform(output.header.frame_id, cloud_msg.header.frame_id, cloud_msg.header.stamp, rospy.Duration(self.tolerance)))
            else:
                cloud_out = cloud_msg

            for point in pc2.read_points(cloud_out, field_names=("x", "y", "z"), skip_nans=True):
                if math.isnan(point[0]) or math.isnan(point[1]) or math.isnan(point[2]):
                    continue

                if point[1] > self.max_height or point[1] < self.min_height:
                    continue

                range_val = math.hypot(point[2], point[1])
                if range_val < self.range_min:
                    continue
                if range_val > self.range_max:
                    continue

                angle = math.atan2(point[1], point[2])
                if angle < output.angle_min or angle > output.angle_max:
                    continue

                index = int((angle - output.angle_min) / output.angle_increment)
                if range_val < output.ranges[index]:
                    output.ranges[index] = range_val

            self.pub.publish(output)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform exception occurred")

if __name__ == '__main__':
    rospy.init_node('pointcloud_to_laserscan')
    node = PointCloudToLaserScanNode()
    rospy.spin()
