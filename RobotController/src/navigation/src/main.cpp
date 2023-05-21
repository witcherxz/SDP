#include "../include/navigation.h"
#include "../include/Robot_controller.h"


GridMap lab_Map = GridMap(15, 15, 0.4);

Line l1 = Line(Point(0, 1), Point(3,1));
Line l2 = Line(Point(5,0), Point(5,4));
Line l3 = Line(Point(2,4), Point(5,4));
Line l4 = Line(Point(2,4), Point(2,6));

void show_menu(robot_controller &pioneer_p3dx, Navigation &navigation){
  navigation.info();
  pioneer_p3dx.info();
  std::cout<<"1- To Set The goal point"<<std::endl;
  std::cout<<"2- To Set The Max speed (Move)"<<std::endl;
  std::cout<<"3- To Set The Max speed (Rotate)"<<std::endl;
  std::cout<<"4- To Set The Kp (Move)"<<std::endl;
  std::cout<<"5- To Set The Kp (Rotate)"<<std::endl;
  std::cout<<"6- To Set The Tolerance (Move)"<<std::endl;
  std::cout<<"7- To Set The Tolerance (Rotate)"<<std::endl;
  std::cout<<"8- To Start the journey"<<std::endl;
  
  int choice;
  std::cout << "Enter your choice: ";
  std::cin >> choice;
    // Perform actions based on user choice
  switch (choice) {

      // Handle goal point setting  
    case 1:{
      double x;
      double y;
      std::cout << "Enter X: ";
      std::cin >> x;
      std::cout << "Enter Y: ";
      std::cin >> y;
      navigation.set_Goal_point(Point(x,y));
      std::cout << "Goal point had been set";
      break;
  }

      // Handle max speed (move) setting
    case 2:{
       std::cout << "Enter Max Speed (Move): ";
       double speed_move;
       std::cin >> speed_move;
       pioneer_p3dx.set_max_speed_move(speed_move);
       std::cout << "Max speed (Move) had been set";
      break;
    }

      // Handle max speed (rotate) setting
    case 3:{
       std::cout << "Enter Max Speed (Rotate): ";
       double speed_rotate;
       std::cin >> speed_rotate;
       pioneer_p3dx.set_max_speed_rotate(speed_rotate);
       std::cout << "Max speed (Rotate) had been set";
      break;
      }
    

      // Handle Kp (move) setting
    case 4:{
       std::cout << "Enter Kp (Move): ";
       double kp_move;
       std::cin >> kp_move;
       pioneer_p3dx.set_kp_move(kp_move);
       std::cout << "Kp (Move) had been set";
      break;
      }

    case 5:{
      // Handle Kp (rotate) setting
      std::cout << "Enter Kp (Rotate): ";
       double kp_rotate;
       std::cin >> kp_rotate;
       pioneer_p3dx.set_kp_rotate(kp_rotate);
       std::cout << "Kp (Rotate) had been set";
      break;
      }
    case 6:{
      // Handle tolerance (move) setting
       std::cout << "Enter tolerance (Move): ";
       double tolerance_move;
       std::cin >> tolerance_move;
       pioneer_p3dx.set_tolerance_move(tolerance_move);
       std::cout << "Tolerance (Move) had been set";
      break;
    }

    case 7:{
      // Handle tolerance (rotate) setting
       std::cout << "Enter tolerance (Rotate): ";
       double tolerance_rotate;
       std::cin >> tolerance_rotate;
       pioneer_p3dx.set_tolerance_rotate(tolerance_rotate);
       std::cout << "Tolerance (Rotate) had been set";
      break;
    }

    case 8:{
      std::cout << "The journey will start"<<std::endl;
      std::tuple<double,double,double> start = pioneer_p3dx.get_current_location();
      navigation.set_Start_point(Point(std::get<0>(start), std::get<1>(start)));
      pioneer_p3dx.go_to_goal(navigation.get_Shortest_Path());
      break;
  }
    default:
      std::cout << "Invalid choice!" << std::endl;
      break;
}

}

int main(int argc, char **argv)
{
  MapCreator a(lab_Map);
  a.addLine(l1);
  a.addLine(l2);
  a.addLine(l3);
  a.addLine(l4);
  robot_controller pioneer_p3dx;
  Navigation navigation(lab_Map, Point(0,0), Point(0,0));
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  std::tuple<double,double,double> start = pioneer_p3dx.get_current_location();
  navigation.set_Start_point(Point(std::get<0>(start), std::get<1>(start)));
  while(ros::ok()){
  show_menu(pioneer_p3dx, navigation);
  std::cout << "\033[2J\033[1;1H";
  ros::spinOnce();
  }
  return 0;
}