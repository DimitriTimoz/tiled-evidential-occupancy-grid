#include "common/EOGM.h"
#include <cmath>

EOGM::EOGM(vector<std::vector<unsigned int>> occupied, vector<std::vector<unsigned int>> free, int width, int height, float resolution)
{
    this->grid = vector<std::vector<CellState>>(width, vector<CellState>(height, CellState::UNKNOWN));

    for (int x = 0; x < width; x++)
    {
        for (int y = 0; y < height; y++)
        {
            if (occupied[x][y] > free[x][y])
            {
                this->grid[x][y] = CellState::OCCUPIED;
            }
            else if (occupied[x][y] < free[x][y])
            {
                this->grid[x][y] = CellState::FREE;
            }
        }
    }
}

map<Point2D, CellState> EOGM::getGrid(float rotation_matrix[2][2], float translation_vector[2])
{
    map<Point2D, CellState> grid_map;
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
