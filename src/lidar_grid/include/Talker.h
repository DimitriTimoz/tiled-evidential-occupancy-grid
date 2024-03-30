#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/OccupancyGrid.h>
class Talker {
public:
    Talker(ros::NodeHandle& nodehandle);
    void publishMessage(nav_msgs::OccupancyGrid& msg);

private:
    ros::NodeHandle nh_;
    ros::Publisher chatter_publisher_;
    void initializePublisher();
};
