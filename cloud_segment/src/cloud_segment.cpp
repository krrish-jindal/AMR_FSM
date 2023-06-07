#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
ros::Publisher publish1;
ros::Publisher publish2;
ros::Publisher publish3;
ros::Publisher publish4;
ros::Publisher publish5;
ros::Publisher publish6;


//  WORKING CODE TO REMOVE LASER SEGMENTATION 



//   FOR FRONT OF AMR

void scanCallback1(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  sensor_msgs::LaserScan filtered_scan_msg = *scan_msg;  // Make a copy of the input scan

  // Apply filtering logic based on range of scan angles
  for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
    float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

    if (angle <= -1 || angle >= 1) {
      // Set the filtered range value to a specific value or NaN
      filtered_scan_msg.ranges[i] = 0.0;  // Set to 0.0 for example
    }
  }

  // Publish the filtered laser scan
  publish1.publish(filtered_scan_msg);
}


//  LEFT FORNT AMR 

void scanCallback2(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  sensor_msgs::LaserScan filtered_scan_msg = *scan_msg;  // Make a copy of the input scan

  // Apply filtering logic based on range of scan angles
  for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
    float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

    if (angle <= 1.5 || angle >= 2.23) {
      // Set the filtered range value to a specific value or NaN
      filtered_scan_msg.ranges[i] = 0.0;  // Set to 0.0 for example
    }
  }

  // Publish the filtered laser scan
  publish2.publish(filtered_scan_msg);
}


//  FOR BACK OF AMR

void scanCallback3(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  sensor_msgs::LaserScan filtered_scan_msg = *scan_msg;  // Make a copy of the input scan

  // Apply filtering logic based on range of scan angles
  for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
    float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

    if (angle >= -2.82 && angle <= 2.82) {
      // Set the filtered range value to a specific value or NaN
      filtered_scan_msg.ranges[i] = 0.0;  // Set to 0.0 for example
    }
  }

  // Publish the filtered laser scan
  publish3.publish(filtered_scan_msg);
}


//    LEFT BACK AMR

void scanCallback4(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  sensor_msgs::LaserScan filtered_scan_msg = *scan_msg;  // Make a copy of the input scan

  // Apply filtering logic based on range of scan angles
  for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
    float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

    if (angle <= 2.46 || angle >= 2.67) {
      // Set the filtered range value to a specific value or NaN
      filtered_scan_msg.ranges[i] = 0.0;  // Set to 0.0 for example
    }
  }

  // Publish the filtered laser scan
  publish4.publish(filtered_scan_msg);
}


// RIGHT BACK AMR

void scanCallback5(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  sensor_msgs::LaserScan filtered_scan_msg = *scan_msg;  // Make a copy of the input scan

  // Apply filtering logic based on range of scan angles
  for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
    float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

    if (angle >= -2.44 || angle <= -2.57) {
      // Set the filtered range value to a specific value or NaN
      filtered_scan_msg.ranges[i] = 0.0;  // Set to 0.0 for example
    }
  }

  // Publish the filtered laser scan
  publish5.publish(filtered_scan_msg);
}



//     RIGHT FRONT AMR

void scanCallback6(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  sensor_msgs::LaserScan filtered_scan_msg = *scan_msg;  // Make a copy of the input scan

  // Apply filtering logic based on range of scan angles
  for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
    float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

    if (angle <= -2.2 || angle >= -1.43) {
      // Set the filtered range value to a specific value or NaN
      filtered_scan_msg.ranges[i] = 0.0;  // Set to 0.0 for example
    }
  }

  // Publish the filtered laser scan
  publish6.publish(filtered_scan_msg);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_node");
  ros::NodeHandle nh;

  // Create publishers to publish the filtered point clouds
  publish1 = nh.advertise<sensor_msgs::LaserScan>("out1", 1);
  publish2 = nh.advertise<sensor_msgs::LaserScan>("out2", 1);
  publish3 = nh.advertise<sensor_msgs::LaserScan>("out3", 1);
  publish4 = nh.advertise<sensor_msgs::LaserScan>("out4", 1);
  publish5 = nh.advertise<sensor_msgs::LaserScan>("out5", 1);
  publish6 = nh.advertise<sensor_msgs::LaserScan>("out6", 1);

  ros::Subscriber sub1 = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback1);
  ros::Subscriber sub2 = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback2);
  ros::Subscriber sub3 = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback3);
  ros::Subscriber sub4 = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback4);
  ros::Subscriber sub5 = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback5);
  ros::Subscriber sub6 = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback6);


  ros::spin();

  return 0;
}
