#pragma once

#include "LaserScanToGrid.hpp"
#include "MapFusion.hpp"
#include "ros/ros.h"

class EvidentialGrid
{
public:
    EvidentialGrid(ros::NodeHandle &);

    void LaserScanToGrid::visionCallback(const sensor_msgs::LaserScanConstPtr &);


private:
    MapFusion map_fusion;
    LaserScanToGrid laser_scan;
    // - Attributes
    ros::Publisher global_eogm_publisher;

    ros::Subscriber ar_sub_;
    ros::Subscriber odometry_subscriber;

    std::vector<std::vector<int8_t>> occupied;
    std::vector<std::vector<int8_t>> free;

    nav_msgs::Odometry current_odometry;
}