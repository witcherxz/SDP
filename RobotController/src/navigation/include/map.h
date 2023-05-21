#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <tuple>

#ifndef SPD_ROBOTVISION_MAP_H
#define SPD_ROBOTVISION_MAP_H

#define OCCUPIED 0
#define FREE 255

class Point
{
private:
    double x;
    double y;

public:
    Point();

    Point(double x, double y);

    bool operator==(const Point &other) const;

    std::tuple<double, double> getCoordinate() const;

    void setCoordinate(double x, double y);

    double length(const Point &other) const;
};

class Line
{
private:
    double length;
    
    Point sp;
    Point ep;

public:
    Line(Point sp, Point ep);

    Line(Point sp, double length, double angle);

    std::tuple<Point, Point> getPoints();
};

class Map
{
public:
    Map();

    void changeCurrentPosition(Point p, int x, int y);

    void loadMap(std::string filepath);

protected:
    Point currentPosition;
};

class GridMap : public Map
{
private:
    cv::Mat map;
    double width;
    double height;
    double cellSize;
    void checkBounders(int r, int c);

public:
    GridMap(double height, double width, double cellSize);

    double getCellSize();

    std::tuple<double, double> getDimensions();

    void occupyCell(int r, int c);

    void freeCell(int r, int c);

    bool isFree(int r, int c);

    cv::Mat getMapCopy();

    void print();
};

#endif // SPD_ROBOTVISION_MAP_H