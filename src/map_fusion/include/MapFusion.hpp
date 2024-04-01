#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tuple>

class MapFusion {
public:
    // - Methods
    // - - Constructor
    MapFusion(ros::NodeHandle& node_handle);
    // - - Callbacks
    void eogmCallback(const nav_msgs::OccupancyGridConstPtr& msg);

private:
    // - Methods
    // - - Getters
    double getZRotation();
    std::tuple<double, double> getXYTranslation();

    

    // - Attributes

    ros::Publisher global_eogm_publisher;   
    ros::NodeHandle node_handle;
    ros::Subscriber eogm_subscriber;
};