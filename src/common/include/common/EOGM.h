#pragma once

#include <vector>
#include <map>
#include "Point2D.h"
#include "BeliefMassFunction.h"
#include <nav_msgs/OccupancyGrid.h>

using namespace std;

class EOGM
{
public:
    EOGM() = default;
    EOGM(unsigned int width, unsigned int height, float resolution);
    EOGM(vector<std::vector<float>> occupied, vector<std::vector<float>> free, float resolution);
    ~EOGM() = default;

    /// Get the grid as a map of points to cell states (occupied, free, unknown)
    /// @param rotation_matrix The rotation matrix of the robot according to the origin
    /// @param translation_vector The translation vector of the robot according to the origin
    map<Point2D, BeliefMassFunction> getGrid(float rotation_matrix[2][2], float translation_vector[2]);
    nav_msgs::OccupancyGrid getOccupancyGrid();
    nav_msgs::OccupancyGrid getFreeGrid();



private:
    vector<std::vector<BeliefMassFunction>> grid;

    float resolution;
};
