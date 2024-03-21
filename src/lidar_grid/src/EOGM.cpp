#include "EOGM.h"

EOGM::EOGM(vector<std::vector<int>> occupied, vector<std::vector<int>> free, int width, int height, float resolution)
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
