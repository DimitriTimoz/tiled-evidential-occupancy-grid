/**
**  Simple ROS Node
**/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LaserScanToGrid
{
public:
  LaserScanToGrid(ros::NodeHandle& nh)
  {
    ar_sub_ = nh.subscribe<sensor_msgs::LaserScan>("laser_scan", 1, &LaserScanToGrid::visionCallback, this);
  }

  void visionCallback(const sensor_msgs::LaserScanConstPtr& msg)
  {
    ROS_INFO_STREAM(msg);
  }

  ros::Subscriber ar_sub_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "laser_scan_to_grid_node");

  // Create a ROS node handle
  ros::NodeHandle nh;

  LaserScanToGrid laser_scan_to_grid(nh);
  ROS_INFO("Hello, World!");

  // Don't exit the program.
  ros::spin();
}
