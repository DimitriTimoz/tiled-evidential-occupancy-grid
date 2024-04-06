#pragma once

#include <vector>
#include <map>
#include "Point2D.h"
#include "BeliefMassFunction.h"
#include <nav_msgs/OccupancyGrid.h>
#include <tuple>
#include <octomap/ColorOcTree.h>

class EOGM
{
public:
    // - Methods
    // - - Constructors
    EOGM() = default;
    EOGM(unsigned int width, unsigned int height, float resolution);
    EOGM(std::vector<std::vector<float>> occupied, std::vector<std::vector<float>> free, float resolution);
    EOGM(nav_msgs::OccupancyGrid occupancy_grid, nav_msgs::OccupancyGrid free_grid, float resolution);
    // - - Destructor
    ~EOGM();

    // - - Getters
    /// Get the grid as a map of points to cell states (occupied, free, unknown)
    /// @param rotation_matrix The rotation matrix of the robot according to the origin
    /// @param translation_vector The translation vector of the robot according to the origin
    std::map<Point2D, BeliefMassFunction> getGrid(float rotation_matrix[2][2], float translation_vector[2]);
    nav_msgs::OccupancyGrid getOccupancyGrid();
    nav_msgs::OccupancyGrid getFreeGrid();
    octomap::ColorOcTree &getOctomap();

    // - - Setters
    void setOrigin(double x, double y);

    // - - Operations

    ///@brief Fuse this EOGM with another using Dempster-Shafer theory. This also update the associated octomap.
    ///
    ///@param other  The other EOGM to fuse with
    void fuse(const EOGM &other);
    void updateOctomap(int x, int y, const BeliefMassFunction *);
    void updateOctomap(octomap::point3d[], const BeliefMassFunction *[], size_t size);

    // - - Conversion
    inline static int8_t fromFloatToByte(float value)
    {
        return static_cast<int8_t>(std::round((value * 0xFF) + INT8_MIN));
    }

    inline static float fromByteToFloat(int8_t value)
    {
        return (static_cast<float>(value) - INT8_MIN) / 0xFF;
    }

    // - - Operators
    ///@brief Add (fuse) two EOGMs together
    ///
    ///@param other
    EOGM &operator+=(const EOGM &other);

private:
    // - Attributes
    // - - Constants
    const octomap::ColorOcTreeNode::Color occupied_color = octomap::ColorOcTreeNode::Color(255, 0, 0);
    const octomap::ColorOcTreeNode::Color free_color = octomap::ColorOcTreeNode::Color(0, 255, 0);
    const octomap::ColorOcTreeNode::Color conflict_color = octomap::ColorOcTreeNode::Color(0, 0, 255);

    // - - Variables
    std::vector<std::vector<BeliefMassFunction>> grid;

    double x, y;
    float resolution;
    octomap::ColorOcTree *tree;
};
