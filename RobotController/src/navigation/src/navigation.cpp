#include "../include/navigation.h"
#include "a_star.cpp"

// #define CELL 0.4

Navigation::Navigation(GridMap input_map, Point input_start, Point input_goal): map(map)
{
    map = input_map;
    cellSize = map.getCellSize();
    start = input_start;
    goal = input_goal;
}

void Navigation::set_Start_point(Point point) {
    start = point;
}
void Navigation::set_Goal_point(Point point) {
    double x, y;
    std::tie(x, y) = point.getCoordinate();
    point.setCoordinate(x / cellSize, y / cellSize);
    goal = point;
}
void Navigation::set_Map(GridMap input_map) {
    map = input_map;
}
void Navigation::set_Path(std::vector<Point> Path) {
Path = Path;
}

void Navigation::Print_Start_point()
{
    std::cout<<"Start Point (x, y): ("<< std::get<0>(start.getCoordinate()) * cellSize <<", "<<std::get<1>(start.getCoordinate()) * cellSize <<")"<<std::endl;
}
void Navigation::Print_Goal_point()
{
    std::cout<<"Goal Point (x, y): ("<< std::get<0>(goal.getCoordinate()) * cellSize <<", "<<std::get<1>(goal.getCoordinate()) * cellSize <<")"<<std::endl;
}

void Navigation::Print_Path()
{
      std::cout <<"---- Path (x, y) ----"<<std::endl;
      for (size_t i = 0; i < Path.size(); ++i) {
        std::cout << i <<": ("<< std::get<0>(Path[i].getCoordinate()) * cellSize << ", " << std::get<1>(Path[i].getCoordinate()) * cellSize <<")"<<std::endl;
    }
      std::cout <<"---------------------"<<std::endl;
}

void Navigation::info(){
  std::cout <<"---- Navigation system ----"<<std::endl;
  Navigation::Print_Start_point();
  Navigation::Print_Goal_point();
  std::cout <<"---------------------------"<<std::endl;
}


Point Navigation::get_Start_point()
{
  return start;
}

Point Navigation::get_Goal_point()
{
  return goal;
}

GridMap Navigation::get_Map()
{
  return map;
}
std::vector<Point> Navigation::get_Shortest_Path(){
  int x, y;
  std::tie(x, y) = goal.getCoordinate();
  Path = findShortestPath(start, goal, map);
  Path.erase(Path.begin());
  return Path;
}
std::vector<Point> Navigation::get_Path()
{
  return Path;
}