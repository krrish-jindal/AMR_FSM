#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def callback(scan_data):
    # Process the received laser scan data
    rospy.loginfo("Received laser scan data with %d ranges", len(scan_data.ranges))

    # Create new laser scan messages for different angular ranges
    scan_data_range1 = LaserScan()
    scan_data_range1.header = scan_data.header
    scan_data_range1.header.frame_id = "sensor_laser"
    
    scan_data_range1.angle_min = 0.0
    scan_data_range1.angle_max = 1.0
    scan_data_range1.ranges = scan_data.ranges[0:200]  # Example range subset
    scan_data_range1.intensities = scan_data.intensities[200:300]  # Example intensity subset

    scan_data_range2 = LaserScan()
    scan_data_range2.header = scan_data.header
    scan_data_range2.header.frame_id = "sensor_laser"

    scan_data_range2.angle_min = -3.14
    scan_data_range2.angle_max = 3.14
    scan_data_range2.ranges = scan_data.ranges[0:300]  # Example range subset
    scan_data_range2.intensities = scan_data.intensities[200:300]  # Example intensity subset

    # Publish the laser scan data to multiple topics
    publisher1.publish(scan_data_range1)
    publisher2.publish(scan_data_range2)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('scan_topic_subscriber_publisher', anonymous=True)

    # Define the topic to subscribe and publish to
    subscribe_topic = '/scan'
    publish_topic1 = '/scan_range1'
    publish_topic2 = '/scan_range2'

    # Create the subscriber object
    subscriber = rospy.Subscriber(subscribe_topic, LaserScan, callback)

    # Create the publisher objects
    publisher1 = rospy.Publisher(publish_topic1, LaserScan, queue_size=100)
    publisher2 = rospy.Publisher(publish_topic2, LaserScan, queue_size=100)

    # Spin and wait for incoming messages
    rospy.spin()
