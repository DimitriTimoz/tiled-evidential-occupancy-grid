#include "Talker.h"

Talker::Talker(ros::NodeHandle& nodehandle): nh_(nodehandle) {
    this->initializePublisher();
}

void Talker::initializePublisher() {
    this->chatter_publisher_ = this->nh_.advertise<nav_msgs::OccupancyGrid>("local_eogm", 5);
}

void Talker::publishMessage(nav_msgs::OccupancyGrid& msg) {
    chatter_publisher_.publish(msg);
    ROS_INFO_STREAM("Published message");
}
