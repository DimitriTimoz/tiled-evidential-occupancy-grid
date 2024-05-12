#include "EvidentialGrid.hpp"

void EvidentialGrid::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    static bool first_time = true;

    this->current_odometry_mutex.lock();
    current_odometry = *msg;
    this->current_odometry_mutex.unlock();

    // Wait for the first odometry message to arrive
    if (first_time)
        this->ar_sub_ = this->node_handle.subscribe<sensor_msgs::LaserScan>("/robot/throttle/scan", 1, &EvidentialGrid::visionCallback, this);
}

void EvidentialGrid::visionCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    this->current_odometry_mutex.lock();
    this->main(msg);
    this->current_odometry_mutex.unlock();
}