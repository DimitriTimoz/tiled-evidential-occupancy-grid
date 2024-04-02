#include "LaserScanToGrid.h"
#include "common/EOGM.h"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

LaserScanToGrid::LaserScanToGrid(ros::NodeHandle &nodehandle)
{
  this->ar_sub_ = nodehandle.subscribe<sensor_msgs::LaserScan>("/scan", 1, &LaserScanToGrid::visionCallback, this);

  this->odometry_subscriber = nodehandle.subscribe("/odom", 1, &LaserScanToGrid::odometryCallback, this);

  this->local_occupancy_publisher = nodehandle.advertise<nav_msgs::OccupancyGrid>("local_occupancy_publisher", 5);
  this->local_free_publisher = nodehandle.advertise<nav_msgs::OccupancyGrid>("local_free_publisher", 5);
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
  // Initialize the grid
  // Compute the width and height of the grid based on the data
  int range_max = (int)msg->range_max / this->resolution;
  int height = (range_max) * 2;
  int width = (range_max) * 2;
  int origin = width / 2;

  float start_angle = msg->angle_min + this->getZRotation();

  this->free.resize(width, std::vector<float>(height, 0.0));
  this->occupied.resize(width, std::vector<float>(height, 0.0));
// Compute cartesian position of all the points
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < msg->ranges.size(); i++)
  {
    float angle = start_angle + i * msg->angle_increment;
    float range = msg->ranges[i];

    if (range > msg->range_max)
    {
      range = msg->range_max - this->resolution;
    }
    range /= this->resolution;
    int x0 = origin; // Position x du Lidar
    int y0 = origin; // Position y du Lidar
    int x1 = x0 + (int)(range * cos(angle));
    int y1 = y0 + (int)(range * sin(angle));
#pragma omp critical
    this->occupied[x1][y1] = 0.9;

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
        this->free[x0][y0] = 0.9;
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

  EOGM eogm(this->occupied, this->free, this->resolution);
  nav_msgs::OccupancyGrid occupancy_grid = eogm.getOccupancyGrid();
  occupancy_grid.info.origin.position.x = this->current_odometry.pose.pose.position.y;
  occupancy_grid.info.origin.position.y = this->current_odometry.pose.pose.position.x;

  nav_msgs::OccupancyGrid free_grid = eogm.getFreeGrid();
  free_grid.info.origin.position.x = this->current_odometry.pose.pose.position.y;
  free_grid.info.origin.position.y = this->current_odometry.pose.pose.position.x;

  this->local_occupancy_publisher.publish(occupancy_grid);
  this->local_free_publisher.publish(free_grid);

  this->free.clear();
  this->occupied.clear();

  ROS_INFO_STREAM(msg);
}

void LaserScanToGrid::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
  this->current_odometry = *msg;
}

