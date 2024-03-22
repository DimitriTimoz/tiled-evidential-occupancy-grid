#ifndef TALKER_H
#define TALKER_H

/**
**  Simple ROS Node
**/
#include <ros/ros.h>
#include <sstream>
#include "LaserScanToGrid.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "laser_scan_to_grid_node");

  // Create a ROS node handle
  ros::NodeHandle nh;

  LaserScanToGrid laser_scan_to_grid(nh);
  ROS_INFO("Hello, I am a laser_scan_to_grid_node.");

  // Don't exit the program.
  ros::spin();
}
#endif
