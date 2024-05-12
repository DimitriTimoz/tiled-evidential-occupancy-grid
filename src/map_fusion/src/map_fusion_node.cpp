#include <ros/ros.h>

#include "EvidentialGrid.hpp"
#include <mutex>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "map_fusion_node");

    ros::NodeHandle node_handle;

    EvidentialGrid evidential_grid(node_handle, 0.05);

    ROS_INFO("Hello, I am a map_fusion_node.");

    ros::spin();
}