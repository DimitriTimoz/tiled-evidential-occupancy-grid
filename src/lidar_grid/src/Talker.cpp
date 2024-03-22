#include "Talker.h"

Talker::Talker(ros::NodeHandle& nodehandle): nh_(nodehandle) {
    this->initializePublisher();
}

void Talker::initializePublisher() {
    this->chatter_publisher_ = this->nh_.advertise<std_msgs::String>("eogm", 1000);
}

void Talker::publishMessage(std_msgs::String& msg) {
    ROS_INFO("%s", msg.data.c_str());
    chatter_publisher_.publish(msg);
}
