#ifndef TALKER_H
#define TALKER_H

/**
**  Simple ROS Node
**/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include "Point2D.h"
#include "Talker.h"

class LaserScanToGrid
{
public:
  LaserScanToGrid(ros::NodeHandle& nh) : talker_(&nh)
  {
    this->ar_sub_ = nh.subscribe<sensor_msgs::LaserScan>("laser_scan", 1, &LaserScanToGrid::visionCallback, this);
  }

  void visionCallback(const sensor_msgs::LaserScanConstPtr& msg)
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
        step += resolution;
        x = (int)(step * cos(angle))/resolution;
        y = (int)(step * sin(angle))/resolution;
        free[x][y] += 1;
      }
      occupied[x][y] += 1;
    }
    EOGM eogm(occupied, free, 100, 100, resolution);
    // TODO: Publish the EOGM
    this->free.clear();
    this->occupied.clear();
    ROS_INFO_STREAM(msg);
  }

private:
  ros::Subscriber ar_sub_;
  Talker talker_;
  std::vector<Point2D> points;
  // Grid 
  std::vector<std::vector<int>> occupied;
  std::vector<std::vector<int>> free;
  float resolution = 0.1;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "laser_scan_to_grid_node");

  // Create a ROS node handle
  ros::NodeHandle nh;

  LaserScanToGrid laser_scan_to_grid(nh);
  ROS_INFO("Hello, I am a laser_scan_to_grid_node.");

  // Don't exit the program.
  ros::spin();
}
#endif
