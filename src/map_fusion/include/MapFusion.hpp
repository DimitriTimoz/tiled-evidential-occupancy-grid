#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tuple>
#include "common/EOGM.h"

class MapFusion {
public:
    // - Methods
    // - - Constructor
    MapFusion(ros::NodeHandle& node_handle);
    // - - Callbacks
    void localOccupancyCallback(const nav_msgs::OccupancyGridConstPtr&);
    void localFreeCallback(const nav_msgs::OccupancyGridConstPtr&);

private:
    // - Methods
    // - - Getters
    double getZRotation();
    std::tuple<double, double> getXYTranslation();

    void fuse();

    // - Attributes
    double origin_x, origin_y;
    nav_msgs::OccupancyGrid occupancy_grid, free_grid;
    EOGM global_eogm;

    ros::Publisher global_eogm_publisher;   
    ros::NodeHandle node_handle;
    ros::Subscriber local_occupancy_subscriber, local_free_subscriber;

    const float resolution = 0.05;
};