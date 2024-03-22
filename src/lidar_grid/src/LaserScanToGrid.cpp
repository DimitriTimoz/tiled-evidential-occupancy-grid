#include "LaserScanToGrid.h"
#include "common/EOGM.h"

LaserScanToGrid::LaserScanToGrid(ros::NodeHandle& nodehandle) : talker_(nodehandle)
{
    this->ar_sub_ = nodehandle.subscribe<sensor_msgs::LaserScan>("laser_scan", 1, &LaserScanToGrid::visionCallback, this);
}

void LaserScanToGrid::visionCallback(const sensor_msgs::LaserScanConstPtr& msg)
  {
    // Clear points
    points.clear();

    // Compute cartesian position of all the points
    for (int i = 0; i < msg->ranges.size(); i++)
    {
      float angle = msg->angle_min + i * msg->angle_increment;
      float range = msg->ranges[i];

      if (range < msg->range_min || range > msg->range_max)
      {
        continue;
      }

      int x = 0;
      int y = 0;
      float step = 0;
      while (step < range)
      {
        free[x][y] += 1;
        step += resolution;
        x = (int)(step * cos(angle))/resolution;
        y = (int)(step * sin(angle))/resolution;
      }
      occupied[x][y] += 1;
    }
    EOGM eogm(occupied, free, 5, 5, resolution);
    // TODO: Publish the EOGM
    this->free.clear();
    this->occupied.clear();
    ROS_INFO_STREAM(msg);
  }