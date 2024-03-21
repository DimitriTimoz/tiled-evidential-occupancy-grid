#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"

class Talker {
public:
    Talker(ros::NodeHandle* nodehandle);
    void publishMessage(std_msgs::String& msg);

private:
    ros::NodeHandle nh_;
    ros::Publisher chatter_publisher_;
    void initializePublisher();
};
