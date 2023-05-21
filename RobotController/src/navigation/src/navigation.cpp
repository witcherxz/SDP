#include "../include/navigation.h"

Navigation::Navigation(GridMap input_map, Point input_start, Point input_goal): map(map)
{
    map = input_map;
    start = input_start;
    goal = input_goal;
}

void Navigation::set_Start_point(Point point) {
    start = point;
}
void Navigation::set_Goal_point(Point point) {
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
    std::cout<<"Start Point (x, y): ("<< std::get<0>(start.getCoordinate())*0.4<<", "<<std::get<1>(start.getCoordinate())*0.4<<")"<<std::endl;
}
void Navigation::Print_Goal_point()
{
    std::cout<<"Goal Point (x, y): ("<< std::get<0>(goal.getCoordinate())*0.4<<", "<<std::get<1>(goal.getCoordinate())*0.4<<")"<<std::endl;
}

void Navigation::Print_Path()
{
      for (size_t i = 0; i < myVector.size(); ++i) {
        std::cout << i <<": ("<< std::get<0>(Path[i].getCoordinate())*0.4 << ", " << std::get<1>(Path[i].getCoordinate())0.4<<")"<<std::endl;
    }
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
std::vector<Point> get_Shortest_Path(){
  Path = findShortestPath(start, goal, lab_Map);
  return Path;
}
std::stack<std::pair<double, double>> Navigation::get_Path()
{
  return Path;
}