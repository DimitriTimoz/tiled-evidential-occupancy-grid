#pragma once

#include <vector>
#include <map>
#include "Point2D.h"
#include "BeliefMassFunction.h"
#include <nav_msgs/OccupancyGrid.h>
#include <tuple>
#include <octomap/OcTree.h>

class EOGM
{
public:
    EOGM() = default;
    EOGM(unsigned int width, unsigned int height, float resolution);
    EOGM(std::vector<std::vector<float>> occupied, std::vector<std::vector<float>> free, float resolution);
    EOGM(nav_msgs::OccupancyGrid occupancy_grid, nav_msgs::OccupancyGrid free_grid, float resolution);

    ~EOGM();

    /// Get the grid as a map of points to cell states (occupied, free, unknown)
    /// @param rotation_matrix The rotation matrix of the robot according to the origin
    /// @param translation_vector The translation vector of the robot according to the origin
    std::map<Point2D, BeliefMassFunction> getGrid(float rotation_matrix[2][2], float translation_vector[2]);
    nav_msgs::OccupancyGrid getOccupancyGrid();
    nav_msgs::OccupancyGrid getFreeGrid();
    octomap::OcTree& getOctomap();

    void setOrigin(double x, double y);

    void fuse(const EOGM &other);

    static int8_t fromFloatToByte(float);
    static float fromByteToFloat(int8_t);

    std::tuple<size_t, size_t> fromCoordinateToIndex(double x, double y);

private:
    std::vector<std::vector<BeliefMassFunction>> grid;

    double x, y;
    float resolution;
    octomap::OcTree* tree;
};
