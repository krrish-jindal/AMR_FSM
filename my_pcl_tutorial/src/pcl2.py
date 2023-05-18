import rospy
from sensor_msgs.msg import PointCloud2
import pcl
import pcl_ros
from pcl_msgs.msg import PointCloud2 as PCL2Msg
from pcl_filters.msg import FilterLimits

def cloud_callback(cloud_msg):
    # Convert ROS PointCloud2 message to PCL point cloud
    cloud = pcl.PointCloud()
    pcl_conversions.fromROSMsg(cloud_msg, cloud)

    # Perform pass-through filtering
    pass_through = cloud.make_passthrough_filter()
    pass_through.set_filter_field_name("z")
    pass_through.set_filter_limits(0.0, 1.0)  # Specify the range along the Z-axis to keep points
    cloud_filtered = pass_through.filter()

    # Convert filtered point cloud back to ROS message
    filtered_cloud_msg = pcl2.create_cloud_xyz32(cloud_msg.header, cloud_filtered.to_array())

    # Publish the filtered point cloud
    pub.publish(filtered_cloud_msg)

if __name__ == '__main__':
    rospy.init_node('pass_through_filter')

    # Create a publisher to publish the filtered point cloud
    pub = rospy.Publisher('filtered_cloud', PCL2Msg, queue_size=1)

    # Subscribe to the input point cloud topic
    rospy.Subscriber('input_cloud', PointCloud2, cloud_callback)

    rospy.spin()
