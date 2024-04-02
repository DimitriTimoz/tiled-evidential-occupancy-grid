#include "MapFusion.hpp"


MapFusion::MapFusion(ros::NodeHandle &node_handle) : node_handle(node_handle)
{
    this->eogm_subscriber = this->node_handle.subscribe("/local_eogm", 5, &MapFusion::eogmCallback, this);

    this->global_eogm_publisher = this->node_handle.advertise<nav_msgs::OccupancyGrid>("global_eogm", 5);

}

void MapFusion::eogmCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    static bool first_time = true;

    ROS_INFO_STREAM("Received EO Grid Map");

    if (first_time)
    {
      
        first_time = false;
    }
      
}

/*


std::tuple<double, double> MapFusion::getXYTranslation()
{
  double x_translation = this->current_odometry.pose.pose.position.x - this->reference_odometry.pose.pose.position.x;
  double y_translation = this->current_odometry.pose.pose.position.y - this->reference_odometry.pose.pose.position.y;

  return std::make_tuple(x_translation, y_translation);
}
*/