#pragma once

#include "ros/ros.h"
#include "Talker.h"
#include "common/Point2D.h"
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

class LaserScanToGrid
{
public:
  // - Methods
  // - - Constructor
  LaserScanToGrid(ros::NodeHandle& nh);
  // - - Callbacks
  void visionCallback(const sensor_msgs::LaserScanConstPtr& msg);

private:
  // - Methods
  nav_msgs::OccupancyGrid getOccupancyGrid();
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  double getZRotation();
  std::tuple<double, double> getXYTranslation();



  // - Attributes
  nav_msgs::Odometry current_odometry;

  ros::Subscriber ar_sub_;
  ros::Subscriber odometry_subscriber;
  ros::Publisher local_occupancy_publisher, local_free_publisher;
  // Grid 
  std::vector<std::vector<float>> occupied;
  std::vector<std::vector<float>> free;
  float resolution = 0.05;
};
