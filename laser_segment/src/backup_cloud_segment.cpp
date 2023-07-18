//        Created For Linear Segmentation in Point Cloud Data



#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub1;
ros::Publisher pub2;

void cloudCallback1(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Convert ROS PointCloud2 message to PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  pcl::PassThrough<pcl::PointXYZ> passY;
  passY.setInputCloud(cloud);
  passY.setFilterFieldName("y");
  passY.setFilterLimits(0.35, 4);
  passY.filter(*cloud);

  // Convert filtered point cloud back to ROS message
  sensor_msgs::PointCloud2 filtered_cloud_msg;
  pcl::toROSMsg(*cloud, filtered_cloud_msg);

  // Publish the filtered point cloud
  pub1.publish(filtered_cloud_msg);
}

void cloudCallback2(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Convert ROS PointCloud2 message to PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  pcl::PassThrough<pcl::PointXYZ> passY;
  passY.setInputCloud(cloud);
  passY.setFilterFieldName("y");
  passY.setFilterLimits(-4, -0.35);
  passY.filter(*cloud);

  // Convert filtered point cloud back to ROS message
  sensor_msgs::PointCloud2 filtered_cloud_msg;
  pcl::toROSMsg(*cloud, filtered_cloud_msg);

  // Publish the filtered point cloud
  pub2.publish(filtered_cloud_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pass_through_filter");
  ros::NodeHandle nh;

  // Create publishers to publish the filtered point clouds
  pub1 = nh.advertise<sensor_msgs::PointCloud2>("output1", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2>("output2", 1);

  // Subscribe to the input point cloud topics
  ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2>("cloud", 1, cloudCallback1);
  ros::Subscriber sub2 = nh.subscribe<sensor_msgs::PointCloud2>("cloud", 1, cloudCallback2);

  ros::spin();

  return 0;
}
