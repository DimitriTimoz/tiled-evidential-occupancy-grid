#pragma once

class Point2D 
{
public:
    int x, y;

    Point2D(int x, int y) : x(x), y(y) {}

    bool operator==(const Point2D &other) const;
    bool operator<(const Point2D& other) const;
    Point2D operator+(const Point2D &other) const;
    Point2D operator-(const Point2D &other) const;
    Point2D operator*(const int &other) const;
    Point2D rotate(float rotation_matrix[2][2]);
};
