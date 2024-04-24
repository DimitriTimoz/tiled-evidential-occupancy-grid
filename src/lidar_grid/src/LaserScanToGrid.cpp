#include "LaserScanToGrid.h"
#include "common/EOGM.h"
#include <cmath>
#include <chrono>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

LaserScanToGrid::LaserScanToGrid(ros::NodeHandle &node_handle) : node_handle(node_handle)
{
  this->odometry_subscriber = node_handle.subscribe("/robot/throttle/odom", 1, &LaserScanToGrid::odometryCallback, this);

  this->local_occupancy_publisher = node_handle.advertise<nav_msgs::OccupancyGrid>("local_occupancy", 5);
  this->local_free_publisher = node_handle.advertise<nav_msgs::OccupancyGrid>("local_free", 5);
}

double LaserScanToGrid::getZRotation()
{
  using namespace tf2;

  Quaternion q;
  convert(this->current_odometry.pose.pose.orientation, q);

  return tf2::getYaw(q);
}

void LaserScanToGrid::visionCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
  ROS_INFO("Laser scan : %d", msg->header.stamp);
  ROS_INFO("Odom : %d", this->current_odometry.header.stamp);

  auto total_local = std::chrono::high_resolution_clock::now();

  #define MAX_RANGE 10
  // Initialize the grid
  // Compute the width and height of the grid based on the data
  int range_max = 0;
  for (int i = 0; i < msg->ranges.size(); i++) {
    if (msg->intensities[i] < 500 || msg->ranges[i] > MAX_RANGE) {
      continue;
    }
    int range = (int)msg->ranges[i] / this->resolution;
    if (range > range_max) {
      range_max = range;
    }
  }

  int height = (range_max) * 2 + 1;
  int width = (range_max) * 2 + 1;
  int origin = range_max;

  float start_angle = msg->angle_min + this->getZRotation();

  this->free.resize(width, std::vector<int8_t>(height, 0));
  this->occupied.resize(width, std::vector<int8_t>(height, 0));
// Compute cartesian position of all the points
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < msg->ranges.size(); i++)
  {
    if (msg->intensities[i] < 500 || msg->ranges[i] > MAX_RANGE) { 
      continue;
    }

    float angle = start_angle + i * msg->angle_increment;
    float range = msg->ranges[i] / this->resolution;

    if (range > range_max)
    {
      range = range_max - this->resolution;
    } 
    int x0 = origin; // Position x du Lidar
    int y0 = origin; // Position y du Lidar
    int x1 = x0 + (int)(range * cos(angle));
    int y1 = y0 + (int)(range * sin(angle));


    if (x1 >= this->occupied.size() || y1 >= this->occupied[0].size()){
      ROS_ERROR("Error %d %d", x1, y1);
      continue;
    }

#pragma omp critical
    this->occupied[x1][y1] = EOGM::fromFloatToByte(0.7);

    // Bresenham's algorithm
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (true)
    {
      // Mark the cell as free
      if (x0 != x1 || y0 != y1)
      {
#pragma omp critical
        this->free[x0][y0] = EOGM::fromFloatToByte(0.7);
      }
      if (x0 == x1 && y0 == y1)
        break; // End of the line
      e2 = 2 * err;
      if (e2 >= dy)
      {
        err += dy;
        x0 += sx;
      }
      if (e2 <= dx)
      {
        err += dx;
        y0 += sy;
      }
    }
  }

  ROS_INFO_STREAM("Local EOGM generated in " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - total_local).count() << "ms");
  this->publishGrid();

  this->free.clear();
  this->occupied.clear();

  ROS_INFO_STREAM(msg);
}

void LaserScanToGrid::publishGrid()
{

  nav_msgs::OccupancyGrid grid;

  grid.info.resolution = (float)this->resolution;
  grid.info.width = this->occupied.size();
  grid.info.height = this->occupied[0].size();
  grid.info.origin.position.x = this->current_odometry.pose.pose.position.x;
  grid.info.origin.position.y = this->current_odometry.pose.pose.position.y;
  grid.info.origin.position.z = 0;
  grid.info.origin.orientation.x = 0;
  grid.info.origin.orientation.y = 0;
  grid.info.origin.orientation.z = 0;
  grid.info.origin.orientation.w = 1;
  grid.header.frame_id = "robot_odom";

  grid.data = flattenMatrix(this->occupied);
  this->local_occupancy_publisher.publish(grid);

  grid.data = flattenMatrix(this->free);
  this->local_free_publisher.publish(grid);
}

template <typename T>
std::vector<T> flattenMatrix(const std::vector<std::vector<T>> &matrix)
{
  std::vector<T> result;
  result.reserve(matrix.size() * matrix[0].size()); // Preallocate memory

  // - Flatten the matrix
  for (const auto &row : matrix)
    for (T element : row)
      result.push_back(element);
  return result;
}

void LaserScanToGrid::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
  static bool first_time = true;

  this->current_odometry = *msg;

  if (first_time)
  {
    this->ar_sub_ = this->node_handle.subscribe<sensor_msgs::LaserScan>("/robot/throttle/scan", 1, &LaserScanToGrid::visionCallback, this);
  }
}
