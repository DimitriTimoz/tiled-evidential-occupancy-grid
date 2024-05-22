#include <ros/ros.h>

#include "EvidentialGrid.hpp"
#include <mutex>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "map_fusion_node");

    ros::NodeHandle node_handle;

    // EvidentialGrid evidential_grid(node_handle, 0.05, "/rmp440le/odom", "/jaguar_robot/scan");
    EvidentialGrid evidential_grid(node_handle, 40, 40, 0.125, "/robot/throttle/odom", "/robot/throttle/scan");  // For the Robotnik Vogui
    // EvidentialGrid evidential_grid(node_handle, 80, 80, 0.05, "/rmp440le/odom", "/jaguar_robot/scan", false);
    // EvidentialGrid evidential_grid(node_handle, 40, 40, 0.05, "/robot/throttle/odom", "/robot/throttle/scan"); // For the Robotnik Vogui
    //EvidentialGrid evidential_grid(node_handle, 60, 60, 0.10, "/odom", "/scan", true);    // For the ilab

    ROS_INFO("Hello, I am a map_fusion_node.");

    ros::spin();
}
