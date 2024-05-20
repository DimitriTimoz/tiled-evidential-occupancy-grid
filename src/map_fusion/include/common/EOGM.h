#pragma once

#include <vector>
#include <map>
#include "BeliefMassFunction.h"
#include <nav_msgs/OccupancyGrid.h>
#include <tuple>
#include <octomap/ColorOcTree.h>

/// @brief The Evidential Occupancy Grid Map (EOGM) class
class EOGM
{
public:
    // - Methods
    // - - Constructors
    EOGM() = delete; // Remove default constructor

    ///@brief Construct a new EOGM object in a unknown state
    ///
    ///@param resolution The resolution of the grid
    EOGM(float resolution);

    /// @brief Construct a new EOGM object with a given width, height and resolution in an unknown state
    /// @param width The width of the grid
    /// @param height The height of the grid
    /// @param resolution The resolution of the grid
    EOGM(unsigned int width, unsigned int height, float resolution);

    /// @brief Construct a new EOGM object with a given width, height and resolution in an unknown state
    /// @param occupied The grid of occupied cells
    /// @param free The grid of free cells
    /// @param resolution The resolution of the grid
    EOGM(std::vector<std::vector<float>> &occupied, std::vector<std::vector<float>> &free, float resolution);

    /// @brief Construct a new EOGM object with a given width, height and resolution in an unknown state
    /// @param occupancy_grid  The occupancy grid
    /// @param free_grid  The free grid
    /// @param resolution  The resolution of the grid
    EOGM(nav_msgs::OccupancyGrid occupancy_grid, nav_msgs::OccupancyGrid free_grid, float resolution);

    // - - Destructor
    ~EOGM();

    // - - Getters
    /// @brief Get the occupancy grid
    /// @return nav_msgs::OccupancyGrid
    nav_msgs::OccupancyGrid getOccupancyGrid();

    /// @brief Get the free grid
    /// @return nav_msgs::OccupancyGrid
    nav_msgs::OccupancyGrid getFreeGrid();

    /// @brief Get the octomap
    /// @return octomap::ColorOcTree
    octomap::ColorOcTree &getOctomap();

    /// @brief Get the width of the grid in cells
    /// @return unsigned int The width of the grid
    unsigned int getWidth() const;

    /// @brief Get the height of the grid in cells
    /// @return unsigned int The height of the grid
    unsigned int getHeight() const;

    /// @brief Get the real width of the grid in meters (width * resolution)
    /// @return float The real width of the grid
    float getRealWidth() const;

    /// @brief Get the real height of the grid in meters (height * resolution)
    /// @return float The real height of the grid
    float getRealHeight() const;

    // - - Setters

    /// @brief Set the origin of the grid
    /// @param x The x coordinate of the origin
    /// @param y The y coordinate of the origin
    void setOrigin(double x, double y);

    // - - Operations

    /// @brief Set the cell at a given position
    /// @param x The x coordinate of the cell
    /// @param y The y coordinate of the cell
    /// @param value The value of the cell
    void setCell(int x, int y, BeliefMassFunction value);

    /// @brief Resize the grid to a new width and height
    /// @details This will not update the origin of the grid
    /// @param width The new width of the grid
    /// @param height The new height of the grid
    void resize(unsigned int width, unsigned int height);

    ///@brief Fuse this EOGM with another using Dempster-Shafer theory. This also update the associated octomap.
    ///
    ///@param other  The other EOGM to fuse with
    void fuse(const EOGM &other);

    // - - Conversion
    /// @brief Convert a float to a byte
    inline static int8_t fromFloatToByte(float value)
    {
        return static_cast<int8_t>(std::round((value * 0xFF) + INT8_MIN));
    }

    /// @brief Convert a byte to a float
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
    // - Methods

    /// @brief Update the octomap with a given mass function
    /// @param x The x coordinate of the cell
    /// @param y The y coordinate of the cell
    /// @param mass The mass function
    void updateOctomap(int x, int y, const BeliefMassFunction *);

    /// @brief Update the octomap with a given mass function
    /// @param points The points to update
    /// @param masses The mass functions to update
    /// @param size The size of the arrays
    void updateOctomap(octomap::point3d[], const BeliefMassFunction *[], size_t size);

    // - Attributes
    // - - Constants
    const octomap::ColorOcTreeNode::Color occupied_color = octomap::ColorOcTreeNode::Color(255, 0, 0);
    const octomap::ColorOcTreeNode::Color free_color = octomap::ColorOcTreeNode::Color(0, 255, 0);
    const octomap::ColorOcTreeNode::Color conflict_color = octomap::ColorOcTreeNode::Color(0, 0, 255);

    // - - Variables
    /// @brief The grid of mass functions
    std::vector<std::vector<BeliefMassFunction>> grid;

    /// @brief The origin of the grid
    double x, y;

    /// @brief The resolution of the grid
    float resolution;

    /// @brief The octomap
    octomap::ColorOcTree *tree;
};
