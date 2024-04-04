#include "MapFusion.hpp"

#include <chrono>

MapFusion::MapFusion(ros::NodeHandle &node_handle) : node_handle(node_handle), global_eogm(60, 60, 0.05)
{
    this->global_eogm_publisher = this->node_handle.advertise<nav_msgs::OccupancyGrid>("global_eogm", 1);

    this->local_occupancy_subscriber = this->node_handle.subscribe("/local_occupancy", 5, &MapFusion::localOccupancyCallback, this);
    this->local_free_subscriber = this->node_handle.subscribe("/local_free", 5, &MapFusion::localFreeCallback, this);
}

void MapFusion::localOccupancyCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    ROS_INFO_STREAM("Received Local Occupancy Grid");

    this->occupancy_grid = *msg;

    this->fuse();
}

void MapFusion::localFreeCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    ROS_INFO_STREAM("Received Local Free Grid");

    this->free_grid = *msg;

    this->fuse();
}

void MapFusion::fuse()
{
    static bool first_time = true;

    if (this->occupancy_grid.info.origin.position != this->free_grid.info.origin.position)
        return;

    ROS_INFO_STREAM("Fusing Local Grids");

    auto total_fuse = std::chrono::high_resolution_clock::now();

    // Set the origin of the global EOGM
    if (first_time)
    {
        auto origin_x = this->occupancy_grid.info.origin.position.y - (60 / 2);
        auto origin_y = this->occupancy_grid.info.origin.position.x - (60 / 2);
        this->global_eogm.setOrigin(origin_x, origin_y);
        first_time = false;
    }

    auto start_2 = std::chrono::high_resolution_clock::now();
    EOGM local_eogm(this->occupancy_grid, this->free_grid, 0.05);
    ROS_INFO_STREAM("Created Local EOGM in " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_2).count() << "ms");

    auto start = std::chrono::high_resolution_clock::now();
    this->global_eogm.fuse(local_eogm);
    ROS_INFO_STREAM("Fused Local Grids in " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms");

    this->global_eogm_publisher.publish(this->global_eogm.getOccupancyGrid());

    ROS_INFO_STREAM("Fused total in " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - total_fuse).count() << "ms");
}