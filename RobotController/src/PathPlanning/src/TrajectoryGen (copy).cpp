#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "rosgraph_msgs/Clock.h"
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <sstream>
#include <conio.h>

#define KEY_UP 65
#define KEY_DOWN 66
#define KEY_LEFT 68
#define KEY_RIGHT 67

int main(int argc, char **argv){
	std::cout <<"Out the loop" << std::endl;
	// Initialize nodes
	ros::init(argc, argv, "TrajectoryGen");
	ros::NodeHandle node;
	geometry_msgs::Twist msg;
	//Publisher for movement data
	ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	double t;
    	double rad_deg = M_PI/180;
	ros::Rate rate(10);
    	int c = 0;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	
	while(ros::ok()){
		switch((c=getch())){
	case KEY_UP:
            	std::cout <<"Up" << std::endl;
		msg.linear.x = 1;
		msg.angular.z = 0;
		pub.publish(msg);
		ros::spinOnce();
		rate.sleep();
		break;
        case KEY_DOWN:
            	std::cout << "Down" << std::endl;
		msg.linear.x = -1;
		msg.angular.z = 0;
		pub.publish(msg);
		ros::spinOnce();
		rate.sleep();
            	break;
        case KEY_LEFT:
            	std::cout << "Left" << std::endl;  // key left
		msg.linear.x = 0;
		msg.angular.z =1;
		pub.publish(msg);
		ros::spinOnce();
		rate.sleep();
            	break;
            break;
        case KEY_RIGHT:
            	std::cout <<"Right" << std::endl;  // key right
            	msg.linear.x = 0;
		msg.angular.z =-1;
		pub.publish(msg);
		ros::spinOnce();
		rate.sleep(); 
		break;       
		}
	std::cout<<c;
	}
	return 0;
}
