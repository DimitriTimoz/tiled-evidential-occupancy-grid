#include "common/EOGM.h"
#include <cmath>
#include <ros/ros.h>

EOGM::EOGM(unsigned int width, unsigned int height, float resolution) : resolution(resolution), origin_x(0), origin_y(0)
{
    this->grid = vector<std::vector<BeliefMassFunction>>(width / resolution, vector<BeliefMassFunction>(height / resolution, BeliefMassFunction()));
}

EOGM::EOGM(vector<std::vector<float>> occupied, vector<std::vector<float>> free, float resolution) : resolution(resolution)
{
    this->grid = vector<std::vector<BeliefMassFunction>>(occupied.size(), vector<BeliefMassFunction>(occupied[0].size(), BeliefMassFunction()));

    for (int x = 0; x < occupied.size(); x++)
    {
        for (int y = 0; y < occupied[0].size(); y++)
        {
            if (occupied[x][y] > free[x][y])
                this->grid[x][y] = BeliefMassFunction(BeliefMassFunction::State::OCCUPIED, occupied[x][y]);
            else
                this->grid[x][y] = BeliefMassFunction(BeliefMassFunction::State::FREE, free[x][y]);
        }
    }
}

EOGM::EOGM(nav_msgs::OccupancyGrid occupancy_grid, nav_msgs::OccupancyGrid free_grid, float resolution)
    : EOGM(occupancy_grid.info.width, occupancy_grid.info.height, resolution)
{
    if (occupancy_grid.info.width != free_grid.info.width || occupancy_grid.info.height != free_grid.info.height)
    {
        ROS_ERROR("Occupancy grid and free grid have different dimensions");
        return;
    }

    if (occupancy_grid.info.origin.position.x != free_grid.info.origin.position.x || occupancy_grid.info.origin.position.y != free_grid.info.origin.position.y)
    {
        ROS_ERROR("Occupancy grid and free grid have different origins");
        return;
    }

    this->setOrigin(occupancy_grid.info.origin.position.x, occupancy_grid.info.origin.position.y);

    size_t width = occupancy_grid.info.width;

    for (int x = 0; x < occupancy_grid.info.width; x++)
    {
        for (int y = 0; y < occupancy_grid.info.height; y++)
        {
            if (occupancy_grid.data[x + (y * width)] > free_grid.data[x + (y * width)])
                this->grid[x][y] = BeliefMassFunction(BeliefMassFunction::State::OCCUPIED, EOGM::fromByteToFloat(occupancy_grid.data[x + y * width]));
            else
                this->grid[x][y] = BeliefMassFunction(BeliefMassFunction::State::FREE, EOGM::fromByteToFloat(free_grid.data[x + y * width]));
        }
    }
}

map<Point2D, BeliefMassFunction> EOGM::getGrid(float rotation_matrix[2][2], float translation_vector[2])
{
    map<Point2D, BeliefMassFunction> grid_map;
    Point2D translation((int)round(translation_vector[0]), (int)round(translation_vector[1]));

    for (int x = 0; x < this->grid.size(); x++)
    {
        for (int y = 0; y < this->grid[0].size(); y++)
        {
            Point2D point(x, y);
            point = point.rotate(rotation_matrix);
            grid_map[point + translation] = this->grid[x][y];
        }
    }
}

nav_msgs::OccupancyGrid EOGM::getOccupancyGrid()
{
    nav_msgs::OccupancyGrid grid_msg;
    grid_msg.info.resolution = (float)this->resolution;
    grid_msg.info.width = this->grid.size();
    grid_msg.info.height = this->grid[0].size();
    grid_msg.info.origin.position.x = -(float)this->grid.size() / 2.0;
    grid_msg.info.origin.position.y = -(float)this->grid[0].size() / 2.0;
    grid_msg.info.origin.position.z = 0;
    grid_msg.info.origin.orientation.x = 0;
    grid_msg.info.origin.orientation.y = 0;
    grid_msg.info.origin.orientation.z = 0;
    grid_msg.info.origin.orientation.w = 1;
    // grid_msg.header.frame_id = "occupancy";

    vector<int8_t> data;
    data.reserve(this->grid.size() * this->grid[0].size());

    for (int x = 0; x < this->grid.size(); x++)
        for (int y = 0; y < this->grid[0].size(); y++)
            data.push_back(int(this->grid[x][y].getMass(BeliefMassFunction::State::OCCUPIED)*100));

    grid_msg.data = data;

    return grid_msg;
}

nav_msgs::OccupancyGrid EOGM::getFreeGrid()
{
    nav_msgs::OccupancyGrid grid_msg;
    grid_msg.info.resolution = (float)this->resolution;
    grid_msg.info.width = this->grid.size();
    grid_msg.info.height = this->grid[0].size();
    grid_msg.info.origin.position.x = -(float)this->grid.size() / 2.0;
    grid_msg.info.origin.position.y = -(float)this->grid[0].size() / 2.0;
    grid_msg.info.origin.position.z = 0;
    grid_msg.info.origin.orientation.x = 0;
    grid_msg.info.origin.orientation.y = 0;
    grid_msg.info.origin.orientation.z = 0;
    grid_msg.info.origin.orientation.w = 1;
    // grid_msg.header.frame_id = "free";

    vector<int8_t> data;

    for (int x = 0; x < this->grid.size(); x++)
        for (int y = 0; y < this->grid[0].size(); y++)
            data.push_back(int8_t(this->grid[x][y].getMass(BeliefMassFunction::State::FREE) * 100));

    grid_msg.data = data;

    return grid_msg;
}

void EOGM::setOrigin(double x, double y)
{
    this->origin_x = x;
    this->origin_y = y;
}

int8_t EOGM::fromFloatToByte(float value)
{
    return static_cast<int8_t>(std::round((value * 0xFF) + INT8_MIN));
}

float EOGM::fromByteToFloat(int8_t value)
{
    return (static_cast<float>(value) - INT8_MIN) / 0xFF;
}

void EOGM::fuse(const EOGM &other)
{
    if (other.resolution != this->resolution)
    {
        ROS_ERROR("Grids have different resolutions");
        return;
    }

    ROS_INFO("Fusing grids : %f, %f | %f, %f", other.origin_x, other.origin_y, this->origin_x, this->origin_y);

    int32_t x_offset = (other.origin_x - this->origin_x) / this->resolution;
    int32_t y_offset = (other.origin_y - this->origin_y) / this->resolution;

    ROS_INFO("Fusing grids with offsets: %d, %d", x_offset, y_offset);

#pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < other.grid.size(); i++)
    {
        for (size_t j = 0; j < other.grid[0].size(); j++)
        {
            int32_t k = i + x_offset;
            int32_t l = j + y_offset;

            // Ignore points outside the grid
            if (k < 0 || k >= this->grid.size() || l < 0 || l >= this->grid[0].size())
                continue;

#pragma omp critical
            this->grid[k][l] += other.grid[i][j];
        }
    }
}