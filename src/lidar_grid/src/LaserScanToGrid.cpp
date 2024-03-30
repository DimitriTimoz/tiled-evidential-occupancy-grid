#include "LaserScanToGrid.h"
#include "common/EOGM.h"

LaserScanToGrid::LaserScanToGrid(ros::NodeHandle& nodehandle) : talker_(nodehandle)
{
    this->ar_sub_ = nodehandle.subscribe<sensor_msgs::LaserScan>("/scan", 1, &LaserScanToGrid::visionCallback, this);
}

void LaserScanToGrid::visionCallback(const sensor_msgs::LaserScanConstPtr& msg)
  {
    // Clear points
    points.clear();

    // Initialize the grid
    // Compute the width and height of the grid based on the data
    int range_max = (int)msg->range_max / this->resolution;
    int height = (range_max)*2;
    int width = (range_max)*2;
    int origin = width/2;

    this->free.resize(width, std::vector<unsigned int>(height, 0));
    this->occupied.resize(width, std::vector<unsigned int>(height, 0));
    // Compute cartesian position of all the points
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < msg->ranges.size(); i++)
    {
      float angle = msg->angle_min + i * msg->angle_increment;
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
      this->occupied[x1][y1] += 1;

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
          this->free[x0][y0] += 1;
        }
        if (x0 == x1 && y0 == y1) break; // End of the line
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
      }
    }
    EOGM eogm(occupied, free, this->resolution);
    nav_msgs::OccupancyGrid grid = eogm.getOccupancyGrid();
    this->talker_.publishMessage(grid);
    this->free.clear();
    this->occupied.clear();
    ROS_INFO_STREAM(msg);
  }
  