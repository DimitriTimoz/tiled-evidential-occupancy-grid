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

/// @brief The main class for the Evidential Grid
class EvidentialGrid
{
public:
  EvidentialGrid() = delete; // Remove default constructor

  /// @brief Construct a new Evidential Grid object
  ///
  /// @param node_handle The ROS node handle
  /// @param width The width of the grid
  /// @param height The height of the grid
  /// @param resolution The resolution of the grid
  /// @param odometry_topic The topic to subscribe to for odometry
  /// @param laser_scan_topic The topic to subscribe to for laser scans
  EvidentialGrid(ros::NodeHandle &node_handle, unsigned int width, unsigned int height, float resolution, const char *odometry_topic, const char *laser_scan_topic);

  ///@brief Callback for the odometry topic
  ///
  ///@param msg The odometry message
  void visionCallback(const sensor_msgs::LaserScanConstPtr &);

  ///@brief Callback for the odometry topic
  ///
  ///@param msg The odometry message
  void odometryCallback(const nav_msgs::OdometryConstPtr &);

private:
  // - Methods

  /// @brief Main function to process the laser scan
  /// @param  msg The laser scan message
  void main(const sensor_msgs::LaserScanConstPtr &);

  // - - Laser scan to grid

  /// @brief Get the rotation of the robot
  ///
  /// @return The rotation of the robot
  double getZRotation();

  /// @brief Convert a laser scan to a grid
  /// @param  msg The laser scan message
  void laserScanToGrid(const sensor_msgs::LaserScanConstPtr &);

  // - - Map fusion

  /// @brief Fuse the local EOGM with the global EOGM
  void fuse();

  /// @brief Publish the global EOGM
  void publish();

  // - Attributes

  /// @brief The global EOGM publisher
  ros::Publisher global_eogm_publisher;

  /// @brief The laser scan subscriber
  ros::Subscriber laser_scan_subscriber;
  /// @brief The odometry subscriber
  ros::Subscriber odometry_subscriber;

  /// @brief Odometry mutex to protect the odometry message
  std::mutex current_odometry_mutex;
  /// @brief The current odometry message
  nav_msgs::Odometry current_odometry;

  /// @brief The ROS node handle
  ros::NodeHandle &node_handle;

  /// @brief The local and global EOGMs
  EOGM local_eogm, global_eogm;

  /// @brief Resolution of the grid
  const float resolution;

  /// @brief Laser scan topic string
  std::string laser_scan_topic;
};
