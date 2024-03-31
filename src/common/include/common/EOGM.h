#pragma once

#include <vector>
#include <map>
#include "Point2D.h"
#include <nav_msgs/OccupancyGrid.h>

using namespace std;

enum class CellState
{
    OCCUPIED,
    FREE,
    UNKNOWN
};

class EOGM
{
public:
    EOGM(vector<std::vector<unsigned int>> occupied, vector<std::vector<unsigned int>> free, float resolution);
    ~EOGM() = default;

    /// Get the grid as a map of points to cell states (occupied, free, unknown)
    /// @param rotation_matrix The rotation matrix of the robot according to the origin
    /// @param translation_vector The translation vector of the robot according to the origin
    map<Point2D, CellState> getGrid(float rotation_matrix[2][2], float translation_vector[2]);
    nav_msgs::OccupancyGrid getOccupancyGrid();

private:
    vector<std::vector<CellState>> grid;
    float resolution;
};
