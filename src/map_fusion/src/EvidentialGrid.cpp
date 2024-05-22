#include "EvidentialGrid.hpp"

#include <nav_msgs/OccupancyGrid.h>

#include <octomap_msgs/conversions.h>

// - Std
#include <chrono>

EvidentialGrid::EvidentialGrid(ros::NodeHandle &node_handle, unsigned int width, unsigned int height, float resolution, const char *odometry_topic, const char *laser_scan_topic, bool local_grid_realignment) : node_handle(node_handle),
                                                                                                                                                                                                                 global_eogm(width, height, resolution),
                                                                                                                                                                                                                 resolution(resolution),
                                                                                                                                                                                                                 laser_scan_topic(laser_scan_topic),
                                                                                                                                                                                                                 local_eogm(resolution),
                                                                                                                                                                                                                 local_grid_realignment(local_grid_realignment)

{

    this->odometry_subscriber = node_handle.subscribe(odometry_topic, 1, &EvidentialGrid::odometryCallback, this);

    this->global_eogm_publisher = node_handle.advertise<octomap_msgs::Octomap>("global_eogm", 1);
}

void EvidentialGrid::main(const sensor_msgs::LaserScanConstPtr &msg)
{
    auto start = std::chrono::high_resolution_clock::now();
    this->laserScanToGrid(msg);
    ROS_INFO("Laser scan to grid took %li ms", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count());

    start = std::chrono::high_resolution_clock::now();
    this->fuse();
    ROS_INFO("Fusion took %li ms", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count());

    start = std::chrono::high_resolution_clock::now();
    this->publish();
    ROS_INFO("Publishing took %li ms", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count());
}
