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
  ros::init(argc, argv, "localization");

  // Create a ROS node handle
  ros::NodeHandle n;
  ros::Publisher userInputs =  n.advertise<geometry_msgs::Pose2D>("initialPose", 10);
  ros::Rate loop_rate(10);
  geometry_msgs::Pose2D msg;

  while(ros::ok()){
  msg.x = 0;
  msg.y = 0;
  msg.theta = 0;
  std::cout<< "intial x postion: " << msg.x << std::endl;
  std::cout<< "intial y postion: " << msg.y << std::endl;
  std::cout<< "intial orientation: " << msg.theta << std::endl;
  userInputs.publish(msg);
  std::cout<< "Msg of localization had been published" << std::endl;
  ros::spinOnce();
  loop_rate.sleep();
  }

}