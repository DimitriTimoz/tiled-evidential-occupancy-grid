#pragma once
#include <vector>

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
    EOGM(vector<std::vector<int>> occupied, vector<std::vector<int>> free, int width, int height, float resolution);
    ~EOGM() = default;

    

private:
    vector<std::vector<CellState>> grid;
};

