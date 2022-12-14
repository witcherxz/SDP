#include <cmath>
#include "map.h"

Point::Point() : x(0), y(0) {}
Point::Point(double x, double y) { setCoordinate(x, y); }
void Point::setCoordinate(double x, double y)
{
    this->x = x;
    this->y = y;
}

double Point::length(Point other)
{
    return sqrt(pow(other.x - this->x, 2) + pow(other.y - this->y, 2));
}

Line::Line(Point sp, Point ep)
{
    this->sp = sp;
    this->ep = ep;
    length = sp.length(ep);
}
Line::Line(Point sp, double length, double angle)
{
    double x = length * cos(angle);
    double y = length * sin(angle);
    ep = Point(x, y);
}

Map::Map()
{
    currentPosition = Point(0, 0);
}

void Map::changeCurrentPosition(Point p, int x, int y)
{
    p.setCoordinate(x, y);
}

GridMap::GridMap(double height, double width, double cellSize) : Map()
{
    this->width = width;
    this->height = height;
    this->cellSize = cellSize;
    int rows = int(height / cellSize);
    int cols = int(width / cellSize);
    map = cv::Mat(rows, cols , CV_8U);
}
