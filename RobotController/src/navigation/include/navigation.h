// #include <ros/ros.h>
#include <iostream>
#include <stack>
#include <list>
#include "map.h"
#include "map_creator.h"

class Navigation
{
protected:
    GridMap map;
    Point start;
    Point goal;
    std::vector<Point> Path;

public:
    Navigation(GridMap input_map, Point input_start, Point input_goal);
    void set_Start_point(Point point);
    void set_Goal_point(Point point);
    void set_Map(GridMap input_map);
    void set_Path(std::vector<Point> Path);

    void Print_Start_point();
    void Print_Goal_point();
    void Print_Path();

    Point get_Start_point();
    Point get_Goal_point();
    GridMap get_Map();
    std::vector<Point> get_Shortest_Path();
    std::vector<Point> get_Path();
};