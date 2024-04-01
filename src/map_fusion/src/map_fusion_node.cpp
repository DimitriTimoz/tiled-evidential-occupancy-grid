#include <ros/ros.h>

#include "MapFusion.hpp"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "map_fusion_node");   

    ros::NodeHandle node_handle;

   // MapFusion map_fusion(node_handle);

    ROS_INFO("Hello, I am a map_fusion_node.");

    ros::spin();
}