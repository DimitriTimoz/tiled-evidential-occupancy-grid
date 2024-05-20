#include "EvidentialGrid.hpp"

void EvidentialGrid::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    static bool first_time = true;

    this->current_odometry_mutex.lock();
    current_odometry = *msg;
    this->current_odometry_mutex.unlock();

    // Wait for the first odometry message to arrive
    if (first_time)
    {
        ROS_INFO("Subscribing to laser scan topic : %s", this->laser_scan_topic.c_str());
        this->laser_scan_subscriber = this->node_handle.subscribe<sensor_msgs::LaserScan>(this->laser_scan_topic, 1, &EvidentialGrid::visionCallback, this);
        first_time = false;
    }
}

void EvidentialGrid::visionCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    this->current_odometry_mutex.lock();
    this->main(msg);
    this->current_odometry_mutex.unlock();
}