/**
**  Simple ROS Node
**/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class Point2D 
{
public:
  float x, y;

  Point2D(float x, float y) : x(x), y(y) {}
};

class LaserScanToGrid
{
public:
  LaserScanToGrid(ros::NodeHandle& nh)
  {
    this->ar_sub_ = nh.subscribe<sensor_msgs::LaserScan>("laser_scan", 1, &LaserScanToGrid::visionCallback, this);
  }

  void visionCallback(const sensor_msgs::LaserScanConstPtr& msg)
  {
    // Clear points
    points.clear();

    // Compute cartesian position of all the points
    for (int i = 0; i < msg->ranges.size(); i++)
    {
      float angle = msg->angle_min + i * msg->angle_increment;
      float range = msg->ranges[i];

      if (range < msg->range_min || range > msg->range_max)
      {
        continue;
      }

      float x = range * cos(angle);
      float y = range * sin(angle);

      points.push_back(Point2D(x, y));
    }

    ROS_INFO_STREAM(msg);
  }

private:
  ros::Subscriber ar_sub_;
  std::vector<Point2D> points;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "laser_scan_to_grid_node");

  // Create a ROS node handle
  ros::NodeHandle nh;

  LaserScanToGrid laser_scan_to_grid(nh);
  ROS_INFO("Hello, I am a laser_scan_to_grid_node.");

  // Don't exit the program.
  ros::spin();
}
