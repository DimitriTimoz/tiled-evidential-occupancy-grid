#pragma once

#include <vector>
#include "ros/ros.h"
#include "common/Point2D.h"
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

class LaserScanToGrid
{
public:
  // - Methods
  // - - Constructor
  LaserScanToGrid(ros::NodeHandle &);
  // - - Callbacks
  void visionCallback(const sensor_msgs::LaserScanConstPtr &);

private:
  // - Methods
  nav_msgs::OccupancyGrid getOccupancyGrid();
  void odometryCallback(const nav_msgs::OdometryConstPtr &);
  double getZRotation();
  std::tuple<double, double> getXYTranslation();
  void publishGrid();

  // - Attributes
  ros::NodeHandle& node_handle;
  ros::Subscriber ar_sub_;
  ros::Subscriber odometry_subscriber;
  ros::Publisher local_occupancy_publisher, local_free_publisher;
  // Grid
  nav_msgs::Odometry current_odometry;
  std::vector<std::vector<int8_t>> occupied;
  std::vector<std::vector<int8_t>> free;
  const float resolution = 0.05;
};

template <typename T>
std::vector<T> flattenMatrix(const std::vector<std::vector<T>> &matrix);