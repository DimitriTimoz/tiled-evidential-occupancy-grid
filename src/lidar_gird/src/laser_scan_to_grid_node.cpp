/**
**  Simple ROS Node
**/
#include <ros/ros.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "laser_scan_to_grid_node");

  // Create a ROS node handle
  ros::NodeHandle nh;

  ROS_INFO("Hello, World!");

  // Don't exit the program.
  ros::spin();
}
