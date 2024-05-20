#pragma once

// - Libraries
// - - Local
#include "common/EOGM.h"
// - - ROS
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
// - - Std
#include <mutex>
#include <tuple>
#include <string>

class EvidentialGrid
{
public:
  EvidentialGrid() = delete;
  EvidentialGrid(ros::NodeHandle &, unsigned int, unsigned int, float, const char *, const char *);

  void visionCallback(const sensor_msgs::LaserScanConstPtr &);
  void odometryCallback(const nav_msgs::OdometryConstPtr &);

private:
  // - Methods

  void main(const sensor_msgs::LaserScanConstPtr &);

  // - - Laser scan to grid
  nav_msgs::OccupancyGrid getOccupancyGrid();

  double getZRotation();
  std::tuple<double, double> getXYTranslation();

  void laserScanToGrid(const sensor_msgs::LaserScanConstPtr &);

  // - - Map fusion

  void fuse();
  void publish();

  // - Attributes

  ros::Publisher global_eogm_publisher;

  /// @brief The laser scan subscriber
  ros::Subscriber laser_scan_subscriber;
  /// @brief The odometry subscriber
  ros::Subscriber odometry_subscriber;
  
  std::mutex current_odometry_mutex;
  nav_msgs::Odometry current_odometry;

  ros::NodeHandle &node_handle;

  EOGM local_eogm, global_eogm;
  double origin_x, origin_y;

  const float resolution;

  std::string laser_scan_topic;
};