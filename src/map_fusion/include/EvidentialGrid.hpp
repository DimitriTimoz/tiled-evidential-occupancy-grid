#pragma once

// - Libraries
// - - Local
#include "common/EOGM.h"
// - - ROS
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
// - - Std
#include <mutex>
#include <tuple>

class EvidentialGrid
{
public:
    EvidentialGrid(ros::NodeHandle &, float resolution = 0.05);

    void visionCallback(const sensor_msgs::LaserScanConstPtr &);
    void odometryCallback(const nav_msgs::OdometryConstPtr &);

private:
    // - Methods

    void main(const sensor_msgs::LaserScanConstPtr &);

    // - - Laser scan to grid
    nav_msgs::OccupancyGrid getOccupancyGrid();

    double getZRotation();
    std::tuple<double, double> getXYTranslation();
    
    void laserScanToGrid(const sensor_msgs::LaserScanConstPtr &);

    // - - Map fusion

    void fuse();
    void publish();

      // - Attributes

    ros::Publisher global_eogm_publisher;

    ros::Subscriber ar_sub_;
    ros::Subscriber odometry_subscriber;

    std::vector<std::vector<float>> occupied, free;

    std::mutex current_odometry_mutex;
    nav_msgs::Odometry current_odometry;

    ros::NodeHandle &node_handle;

    EOGM global_eogm;
    double origin_x, origin_y;

    const float resolution;
};