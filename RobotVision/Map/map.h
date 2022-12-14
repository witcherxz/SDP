#include <opencv2/core/mat.hpp>

#ifndef SPD_ROBOTVISION_MAP_H
#define SPD_ROBOTVISION_MAP_H

#define OCCUPIED 0
#define FREE 255

class Point {
    private:
        double x;
        double y;
    public:
        Point();
        Point(double x, double y);
        void setCoordinate(double x, double y);
        double length(Point other);

};

class Line {
private:
    double length;
    Point sp;
    Point ep;
public:
    Line(Point sp, Point ep);
    Line(Point sp, double length, double angle);
};

class Map {
    public:    
        Map();
        void changeCurrentPosition(Point p ,int x, int y);

    protected:
        Point currentPosition;
};

class GridMap: public Map {
    private:
        cv::Mat map;
        double width;
        double height;
        double cellSize;
    public:
        GridMap(double height, double width, double cellSize);
};
#endif //SPD_ROBOTVISION_MAP_H