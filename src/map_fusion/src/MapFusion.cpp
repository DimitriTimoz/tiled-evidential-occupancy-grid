#include "EvidentialGrid.hpp"
#include <octomap_msgs/conversions.h>

void EvidentialGrid::publish()
{
    octomap_msgs::Octomap global_eogm_msg;
    if (!octomap_msgs::fullMapToMsg(this->global_eogm.getOctomap(), global_eogm_msg))
    {
        ROS_ERROR("Failed to serialize octomap");
    }
    ROS_INFO_STREAM("Frame id: " << global_eogm_msg.header.frame_id);
    global_eogm_msg.header.frame_id = "map";

    this->global_eogm_publisher.publish(global_eogm_msg);
}

void EvidentialGrid::fuse()
{
    static bool first_time = true;

    ROS_INFO_STREAM("Fusing Local Grids");

    auto total_fuse = std::chrono::high_resolution_clock::now();

    // Set the origin of the global EOGM
    if (first_time)
    {
        auto origin_x = this->current_odometry.pose.pose.position.x - (20 / 2);
        auto origin_y = this->current_odometry.pose.pose.position.y - (20 / 2);
        this->global_eogm.setOrigin(origin_x, origin_y);
        first_time = false;
    }

    auto start_2 = std::chrono::high_resolution_clock::now();
    EOGM local_eogm(this->occupied, this->free, this->resolution);
    ROS_INFO_STREAM("Created Local EOGM in " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_2).count() << "ms");

    auto start = std::chrono::high_resolution_clock::now();
    this->global_eogm.fuse(local_eogm);

    ROS_INFO_STREAM("Fused Local Grids in " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms");

    ROS_INFO_STREAM("Fused total in " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - total_fuse).count() << "ms");
}
