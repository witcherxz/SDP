#include <cmath>
#include "map.h"
#include <iostream>
Point::Point() : x(0), y(0) {}
Point::Point(double x, double y) { setCoordinate(x, y); }
void Point::setCoordinate(double x, double y)
{
    this->x = x;
    this->y = y;
}

double Point::length(const Point& other) const
{
    return sqrt(pow(other.x - this->x, 2) + pow(other.y - this->y, 2));
}

std::tuple<double, double> Point::getCoordinate() const{
    return std::make_tuple(x, y);
}

bool Point::operator==(const Point& other) const{
    return this->x == other.x && this->y == other.y;
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

std::tuple<Point, Point> Line::getPoints(){
    return std::make_tuple(sp, ep);
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
    map = cv::Mat(rows, cols , CV_8U, 255);
}

double GridMap::getCellSize(){
    return cellSize;
}

std::tuple<double, double> GridMap::getDimensions(){
    return std::make_tuple(width, height);
}

void GridMap::checkBounders(int r, int c){
    assert(r >= 0 && r < width && c >= 0 && c < height);
}

void GridMap::occupyCell(int r, int c){
    checkBounders(r, c);
    map.at<char>(r, c) = OCCUPIED;
}

void GridMap::freeCell(int r, int c){
    checkBounders(r, c);
    map.at<char>(r, c) = FREE;
}


cv::Mat GridMap::getMapCopy(){
    return map.clone();
}

void GridMap::print(){
    std::cout << "Map = \n" << map << ";" << std::endl << std::endl;
}

bool GridMap::isFree(int r, int c){
    checkBounders(r, c);
    return map.at<uchar>(r, c) == FREE;
}