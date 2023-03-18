#include <ros/ros.h>
#include "std_msgs/String.h" 
#include "geometry_msgs/Pose2D.h"

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <math.h>

int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "UserInput");

  // Create a ROS node handle
  ros::NodeHandle n;
  ros::Publisher userInputs =  n.advertise<geometry_msgs::Pose2D>("finalPose", 10);
  ros::Rate loop_rate(10);
  geometry_msgs::Pose2D msg;

  while(ros::ok()){
  std::cout<< "Enter final x postion" << std::endl;
	std::cin >> msg.x;
  std::cout<< "Done, Final x postion: " << msg.x << std::endl;
  
  std::cout<< "Enter final y postion" << std::endl;
  std::cin >> msg.y;
  std::cout<< "Done, Final y postion: " << msg.y << std::endl;
  
  std::cout<< "Enter final theta postion" << std::endl;
	std::cin >> msg.theta;
  std::cout<< "Done, Final theta postion: " << msg.theta << std::endl;

	userInputs.publish(msg);
  std::cout<< "Msg of userinputs had been published" << std::endl;
  ros::spinOnce();
	loop_rate.sleep();
  }

}