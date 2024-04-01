#include "common/EOGM.h"
#include <cmath>
#include <ros/ros.h>

EOGM::EOGM(unsigned int width, unsigned int height, float resolution) : resolution(resolution)
{
    this->grid = vector<std::vector<BeliefMassFunction>>(width / resolution, vector<BeliefMassFunction>(height / resolution, BeliefMassFunction()));
}

EOGM::EOGM(vector<std::vector<unsigned int>> occupied, vector<std::vector<unsigned int>> free, float resolution, float certainty) : resolution(resolution)
{
    this->grid = vector<std::vector<BeliefMassFunction>>(occupied.size(), vector<BeliefMassFunction>(occupied[0].size(), BeliefMassFunction()));

    for (int x = 0; x < occupied.size(); x++)
    {
        for (int y = 0; y < occupied[0].size(); y++)
        
        {
            if (occupied[x][y] > 0)
                this->grid[x][y] = BeliefMassFunction(BeliefMassFunction::State::OCCUPIED, certainty);
            else if (occupied[x][y] < free[x][y])
                this->grid[x][y] = BeliefMassFunction(BeliefMassFunction::State::FREE, certainty);
        }
    }
}

EOGM::EOGM(nav_msgs::OccupancyGrid grid_msg, float certainty) : resolution(grid_msg.info.resolution)
{
    this->grid = vector<std::vector<BeliefMassFunction>>(grid_msg.info.width, vector<BeliefMassFunction>(grid_msg.info.height, BeliefMassFunction()));

    for (int x = 0; x < grid_msg.info.width; x++)
    {
        for (int y = 0; y < grid_msg.info.height; y++)
        {
            int index = x + y * grid_msg.info.width;
            if (grid_msg.data[index] == 100)
            {
                this->grid[x][y] = BeliefMassFunction(BeliefMassFunction::State::OCCUPIED, certainty);
            }
            else if (grid_msg.data[index] == 0)
            {
                this->grid[x][y] = BeliefMassFunction(BeliefMassFunction::State::FREE, certainty);
            }
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

    vector<int8_t> data;

    for (int x = 0; x < this->grid.size(); x++)
        for (int y = 0; y < this->grid[0].size(); y++)
            data.push_back(int8_t(this->grid[x][y].getMass(BeliefMassFunction::State::OCCUPIED) * 100));
        

    grid_msg.data = data;

    return grid_msg;
}
