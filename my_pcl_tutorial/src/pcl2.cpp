// Tried pointcloud to scan fail

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

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
  passZ.setFilterLimits(-1, 5.0);  // Specify the range along the Z-axis to keep points
  passZ.filter(*cloud);
  pcl::PassThrough<pcl::PointXYZRGB> passX;
  passX.setInputCloud(cloud);
  passX.setFilterFieldName("x");
  passX.setFilterLimits(-5, 5.0);  // Specify the range along the X-axis to keep points
  passX.filter(*cloud);
  pcl::PassThrough<pcl::PointXYZRGB> passY;
  passY.setInputCloud(cloud);
  passY.setFilterFieldName("y");
  passY.setFilterLimits(-0.1, 0.15);  // Specify the range along the Y-axis to keep points
  passY.filter(*cloud);

  // Convert filtered point cloud to laser scan
  sensor_msgs::LaserScan laser_scan_msg;
  laser_scan_msg.header = cloud_msg->header;
  laser_scan_msg.angle_min = -M_PI / 2.0;  // Start angle of the laser scan
  laser_scan_msg.angle_max = M_PI / 2.0;   // End angle of the laser scan
  laser_scan_msg.angle_increment = M_PI / 180.0;  // Angular resolution of the laser scan
  laser_scan_msg.time_increment = 0.0;
  laser_scan_msg.scan_time = 1.0 / 30.0;  // Time between scans (assuming 30Hz)
  laser_scan_msg.range_min = 0.0;         // Minimum range of the laser scan
  laser_scan_msg.range_max = 10.0;        // Maximum range of the laser scan

  // Transform point cloud to laser scanner frame
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud(*cloud, *transformed_cloud, Eigen::Affine3f::Identity());

  // Extract points within a specific range
  pcl::IndicesPtr indices(new std::vector<int>());
  pcl::PassThrough<pcl::PointXYZRGB> passThrough;
  passThrough.setInputCloud(transformed_cloud);
  passThrough.setFilterFieldName("x");
  passThrough.setFilterLimits(0.0, 10.0);  // Specify the range along the X-axis to keep points
  passThrough.filter(*indices);

  // Create the laser scan data
  laser_scan_msg.ranges.resize(transformed_cloud->size(), std::numeric_limits<float>::infinity());
  for (size_t i = 0; i < indices->size(); ++i) {
    int index = indices->at(i);
    float range = std::sqrt(std::pow(transformed_cloud->points[index].x, 2) +
                            std::pow(transformed_cloud->points[index].y, 2) +
                            std::pow(transformed_cloud->points[index].z, 2));
    if (range >= laser_scan_msg.range_min && range <= laser_scan_msg.range_max) {
      int angle_index = (int)((std::atan2(transformed_cloud->points[index].y,
                                          transformed_cloud->points[index].x) - laser_scan_msg.angle_min) /
                              laser_scan_msg.angle_increment);
      laser_scan_msg.ranges[angle_index] = range;
    }
  }

  // Publish the laser scan
  pub.publish(laser_scan_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "voxel_grid_filter");
  ros::NodeHandle nh;

  // Create a publisher to publish the laser scan
  pub = nh.advertise<sensor_msgs::LaserScan>("laser_scan", 1);

  // Subscribe to the input point cloud topic
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, cloudCallback);

  ros::spin();

  return 0;
}
