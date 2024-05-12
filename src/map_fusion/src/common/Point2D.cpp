#include "Point2D.h"

bool Point2D::operator==(const Point2D &other) const
{
    return this->x == other.x && this->y == other.y;
}

bool Point2D::operator<(const Point2D &other) const
{
    return this->x < other.x && this->y < other.y;
}

Point2D Point2D::operator+(const Point2D &other) const 
{
    return Point2D(this->x + other.x, this->y + other.y);
}

Point2D Point2D::operator-(const Point2D &other) const 
{
    return Point2D(this->x - other.x, this->y - other.y);
}

Point2D Point2D::operator*(const int &other) const 
{
    return Point2D(this->x * other, this->y * other);
}

Point2D Point2D::rotate(float rotation_matrix[2][2])
{
    int x = this->x * rotation_matrix[0][0] + this->y * rotation_matrix[0][1];
    int y = this->x * rotation_matrix[1][0] + this->y * rotation_matrix[1][1];
    return Point2D(x, y);
}
