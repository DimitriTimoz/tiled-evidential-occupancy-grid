#include "EvidentialGrid.hpp"

#include <nav_msgs/OccupancyGrid.h>

#include <octomap_msgs/conversions.h>

// - Std
#include <chrono>

EvidentialGrid::EvidentialGrid(ros::NodeHandle &node_handle, float resolution) : node_handle(node_handle),
                                                                                 global_eogm(20, 20, resolution),
                                                                                 resolution(resolution)

{

    this->odometry_subscriber = node_handle.subscribe("/robot/throttle/odom", 1, &EvidentialGrid::odometryCallback, this);

    this->global_eogm_publisher = node_handle.advertise<octomap_msgs::Octomap>("global_eogm", 1);
}

void EvidentialGrid::main(const sensor_msgs::LaserScanConstPtr &msg)
{
    this->laserScanToGrid(msg);

    this->fuse();

    this->publish();
}