#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Convert ROS PointCloud2 message to PCL point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  // Create a voxel grid filter
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
  voxelGrid.setInputCloud(cloud);
  voxelGrid.setLeafSize(0.02, 0.02, 0.02);  // Specify the size of the voxels
  voxelGrid.filter(*cloud);

  // Perform pass-through filtering 
  pcl::PassThrough<pcl::PointXYZRGB> passZ;
  passZ.setInputCloud(cloud);
  passZ.setFilterFieldName("z");
  passZ.setFilterLimits(-1, 5.0);  // FRONT X AXIS Specify the range along the Z-axis to keep points 
  passZ.filter(*cloud);
  pcl::PassThrough<pcl::PointXYZRGB> passX;
  passX.setInputCloud(cloud);
  passX.setFilterFieldName("x");
  passX.setFilterLimits(-5, 5.0);  // Specify the range along the X-axis to keep points
  passX.filter(*cloud);
  pcl::PassThrough<pcl::PointXYZRGB> passY;
  passY.setInputCloud(cloud);
  passY.setFilterFieldName("y");
  passY.setFilterLimits(-0.3, 0.1);  // FRONT Z AXIS Specify the range along the Y-axis to keep points
  passY.filter(*cloud);

  // Convert filtered point cloud back to ROS message
  sensor_msgs::PointCloud2 filtered_cloud_msg;
  pcl::toROSMsg(*cloud, filtered_cloud_msg);

  // Publish the filtered point cloud
  pub.publish(filtered_cloud_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "voxel_grid_filter");
  ros::NodeHandle nh;

  // Create a publisher to publish the filtered point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Subscribe to the input point cloud topic
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, cloudCallback);

  ros::spin();

  return 0;
}
