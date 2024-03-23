#pragma once

#include "ros/ros.h"
#include "Talker.h"
#include "Point2D.h"
#include <sensor_msgs/LaserScan.h>

class LaserScanToGrid
{
public:
  LaserScanToGrid(ros::NodeHandle& nh);
  void visionCallback(const sensor_msgs::LaserScanConstPtr& msg);

private:
  ros::Subscriber ar_sub_;
  Talker talker_;
  std::vector<Point2D> points;
  // Grid 
  std::vector<std::vector<unsigned int>> occupied;
  std::vector<std::vector<unsigned int>> free;
  float resolution = 0.05;
};
