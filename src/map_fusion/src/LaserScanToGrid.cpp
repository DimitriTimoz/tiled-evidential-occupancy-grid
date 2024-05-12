#include "common/EOGM.h"
#include <cmath>
#include <chrono>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "EvidentialGrid.hpp"

double EvidentialGrid::getZRotation()
{
  using namespace tf2;

  Quaternion q;
  convert(this->current_odometry.pose.pose.orientation, q);

  return tf2::getYaw(q);
}

void EvidentialGrid::laserScanToGrid(const sensor_msgs::LaserScanConstPtr &msg)
{
  ROS_INFO("Laser scan : %d", msg->header.stamp);
  ROS_INFO("Odom : %d", this->current_odometry.header.stamp);

  auto total_local = std::chrono::high_resolution_clock::now();

#define MAX_RANGE 10
  // Initialize the grid
  // Compute the width and height of the grid based on the data
  int range_max = 0;
  for (int i = 0; i < msg->ranges.size(); i++)
  {
    if (msg->intensities[i] < 500 || msg->ranges[i] > MAX_RANGE)
    {
      continue;
    }
    int range = (int)msg->ranges[i] / this->resolution;
    if (range > range_max)
    {
      range_max = range;
    }
  }

  int height = (range_max) * 2 + 1;
  int width = (range_max) * 2 + 1;
  int origin = range_max;

  float start_angle = msg->angle_min + this->getZRotation();

  this->free.clear();
  this->occupied.clear();

  this->free.resize(width, std::vector<float>(height, 0));
  this->occupied.resize(width, std::vector<float>(height, 0));
// Compute cartesian position of all the points
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < msg->ranges.size(); i++)
  {
    if (msg->intensities[i] < 500 || msg->ranges[i] > MAX_RANGE)
    {
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

    if (x1 >= occupied.size() || y1 >= occupied[0].size())
    {
      ROS_ERROR("Error %d %d", x1, y1);
      continue;
    }

#pragma omp critical
    occupied[x1][y1] = 0.7;

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
        free[x0][y0] = 0.7;
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
}