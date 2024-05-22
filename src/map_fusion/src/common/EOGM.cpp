#include "common/EOGM.h"
#include <cmath>
#include <ros/ros.h>

EOGM::EOGM(float resolution) : resolution(resolution), x(0), y(0) {}

EOGM::EOGM(unsigned int width, unsigned int height, float resolution) : EOGM(resolution)
{
    this->grid = std::vector<std::vector<BeliefMassFunction>>(width / resolution, std::vector<BeliefMassFunction>(height / resolution, BeliefMassFunction()));
    this->tree = new octomap::ColorOcTree(1);
}

EOGM::~EOGM()
{
    delete tree;
}

EOGM::EOGM(std::vector<std::vector<float>> &occupied, std::vector<std::vector<float>> &free, float resolution) : EOGM(occupied.size(), occupied[0].size(), resolution)
{
    if (occupied.size() != free.size() || occupied[0].size() != free[0].size())
        return;

    for (int x = 0; x < occupied.size(); x++)
    {
        for (int y = 0; y < occupied[0].size(); y++)
        {
            if (occupied[x][y] == free[x][y])
                this->grid[x][y] = BeliefMassFunction();
            else if (occupied[x][y] > free[x][y])
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
            int8_t occupancy = occupancy_grid.data[x + (y * width)];
            int8_t free = free_grid.data[x + (y * width)];

            if (occupancy == free)
                this->grid[x][y] = BeliefMassFunction();
            else if (occupancy > free)
                this->grid[x][y] = BeliefMassFunction(BeliefMassFunction::State::OCCUPIED, EOGM::fromByteToFloat(occupancy));
            else
                this->grid[x][y] = BeliefMassFunction(BeliefMassFunction::State::FREE, EOGM::fromByteToFloat(free));
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

    std::vector<int8_t> data;
    data.reserve(this->grid.size() * this->grid[0].size());

    for (int x = 0; x < this->grid.size(); x++)
        for (int y = 0; y < this->grid[0].size(); y++)
            data.push_back(int(this->grid[x][y].getOccupancyProbability() * 100));

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

    std::vector<int8_t> data;

    for (int x = 0; x < this->grid.size(); x++)
        for (int y = 0; y < this->grid[0].size(); y++)
            data.push_back(int8_t(this->grid[x][y].getFreeProbability()));

    grid_msg.data = data;

    return grid_msg;
}

void EOGM::setOrigin(double x, double y)
{
    this->x = x;
    this->y = y;
}

void EOGM::fuse(const EOGM &other)
{
    if (other.resolution != this->resolution)
    {
        ROS_ERROR("Grids have different resolutions");
        return;
    }

    int x_offset = (other.x - this->x) / this->resolution;
    int y_offset = (other.y - this->y) / this->resolution;

    BeliefMassFunction placeholders[8];

    unsigned int outside_rows = 0, outside_columns = 0;

#pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < other.getWidth(); i++)
    {
        // - Accumulate the conjunctions in batches of 8
        BeliefMassFunction *a[8];
        const BeliefMassFunction *b[8];
        octomap::point3d points[8];
        int m = 0;

        int k = i + x_offset;

        // Ignore points outside the grid
        if (k < 0 || k >= this->getWidth())
        {
            outside_rows++;
            continue;
        }

        for (size_t j = 0; j < other.getHeight(); j++)
        {
            int l = j + y_offset;

            // Ignore points with a mass of 1
            if (other.grid[i][j].getMass(BeliefMassFunction::State::UNKNOWN) == 1)
                continue;

            // Ignore points outside the grid
            if (l < 0 || l >= this->getHeight())
            {
                outside_columns++;
                continue;
            }

            a[m] = &this->grid[k][l];
            b[m] = &other.grid[i][j];

            points[m] = octomap::point3d(k, l, 0);
            m++;

            // Compute the conjunctions in batches of 8
            if (m >= 8)
            {
                BeliefMassFunction::computeConjunctionLevels(a, b);

#pragma omp critical
                this->updateOctomap(points, (const BeliefMassFunction **)a, 8);

                m = 0;
                memset(a, 0, 8 * sizeof(BeliefMassFunction *));
                memset(b, 0, 8 * sizeof(BeliefMassFunction *));
            }
        }

        // Flush the remaining conjunctions
        if (m > 0)
        {
            // Fill the rest of the array with placeholders
            for (int n = m; n < 8; n++)
            {
                a[n] = &placeholders[n];
                b[n] = &placeholders[n];
            }

            BeliefMassFunction::computeConjunctionLevels(a, b);

#pragma omp critical
            this->updateOctomap(points, (const BeliefMassFunction **)a, m);

            memset(a, 0, 8 * sizeof(BeliefMassFunction *));
            memset(b, 0, 8 * sizeof(BeliefMassFunction *));
        }
    }

    this->tree->prune();

    if (outside_rows > 0 || outside_columns > 0)
    {
        ROS_WARN("Ignored %d rows and %d columns. Consider resizing the grid.", outside_rows, outside_columns);
    }
}

octomap::ColorOcTree &EOGM::getOctomap()
{
    return *this->tree;
}

void EOGM::updateOctomap(int x, int y, const BeliefMassFunction *mass)
{
    float free = mass->getMass(BeliefMassFunction::State::FREE);
    float occupancy = mass->getMass(BeliefMassFunction::State::OCCUPIED);
    float conflict = mass->getMass(BeliefMassFunction::State::CONFLICT);

    if (conflict > 0)
    {
        conflict = conflict * 10;

        for (int i = 0; i < conflict; i++)
            this->tree->updateNode(octomap::point3d(x, y, i), true, false)->setColor(this->conflict_color);
        for (int i = conflict; i < 10; i++)
            this->tree->updateNode(octomap::point3d(x, y, i), false, false);
    }
    else if (occupancy > free)
    {
        occupancy = mass->getOccupancyProbability() * 10;

        for (int i = 0; i < occupancy; i++)
            this->tree->updateNode(octomap::point3d(x, y, i), true, false)->setColor(this->occupied_color);
        for (int i = occupancy; i < 10; i++)
            this->tree->updateNode(octomap::point3d(x, y, i), false, false);
    }
    else
    {
        free = mass->getFreeProbability() * 10;

        for (int i = 0; i < free; i++)
            this->tree->updateNode(octomap::point3d(x, y, i), true, false)->setColor(this->free_color);
        for (int i = free; i < 10; i++)
            this->tree->updateNode(octomap::point3d(x, y, i), false, false);
    }
}

void EOGM::updateOctomap(octomap::point3d points[], const BeliefMassFunction *masses[], size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        this->updateOctomap(points[i].x(), points[i].y(), masses[i]);
    }
}

void EOGM::setCell(int x, int y, BeliefMassFunction value)
{
    this->grid[x][y] = value;
}

void EOGM::resize(unsigned int width, unsigned int height)
{
    this->grid.resize(width);
    for (int x = 0; x < width; x++)
    {
        this->grid[x].resize(height);
        this->grid[x].assign(height, BeliefMassFunction());
    }
}

unsigned int EOGM::getWidth() const
{
    return this->grid.size();
}

unsigned int EOGM::getHeight() const
{
    return this->grid[0].size();
}

float EOGM::getRealWidth() const
{
    return this->grid.size() * this->resolution;
}

float EOGM::getRealHeight() const
{
    return this->grid[0].size() * this->resolution;
}
