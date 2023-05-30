#include "../include/navigation.h"
#include "../include/Robot_controller.h"

GridMap lab_Map = GridMap(15, 15, 0.4);

Line l1 = Line(Point(0, 1), Point(3,1));
Line l2 = Line(Point(5,0), Point(5,4));
Line l3 = Line(Point(2,4), Point(5,4));
Line l4 = Line(Point(2,4), Point(2,6));


int main(int argc, char **argv)
{
  MapCreator a(lab_Map);
  a.addLine(l1);
  a.addLine(l2);
  a.addLine(l3);
  a.addLine(l4);
  
  std::cout << "--------- From the main --------------"<<std::endl;\
  Navigation nav(lab_Map, Point(0, 0), Point(5,5));
  nav.Print_Goal_point();
  nav.Print_Start_point();
  robot_controller robot;
  robot.go_to_goal(nav.get_Shortest_Path());
}