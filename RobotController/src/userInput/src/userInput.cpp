#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf2_msgs/TFMessage.h"
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

const double PI = 3.141592653589793238463;

geometry_msgs::Twist msg;

void rotate(double angular_speed, double relative_angle, bool isClockWise)
{
	ros::NodeHandle n;
	ros::Publisher path = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);
	ros::Rate rate(50);
	if (isClockWise)
		msg.angular.z = abs(angular_speed);
	else
		msg.angular.z = -abs(angular_speed);
	double current_angle = 0;
	double t0 = ros::Time::now().toSec();
	do
	{
		path.publish(msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1 - t0);
		ros::spinOnce();
		rate.sleep();
	} while (current_angle < relative_angle);
	msg.angular.z = 0;
	path.publish(msg);
	ros::spinOnce();
	msg.angular.z = 0;
	msg.linear.x = 0;
}

int main(int argc, char **argv)
{
	// Initialize nodes
	ros::init(argc, argv, "TrajectoryGen");
	ros::NodeHandle node;
	// Publisher for movement data
	ros::Publisher pub = node.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);
	double t;

	ros::Rate rate(10);
	int c = 0;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;

	while (ros::ok())
	{
		switch ((c = getch()))
		{
		case KEY_UP:
			std::cout << "Up" << std::endl;
			msg.linear.x = 0.5;
			msg.angular.z = 0;
			pub.publish(msg);
			rate.sleep();
			msg.linear.x = 0;
			msg.angular.z = 0;
			break;
		case KEY_DOWN:
			std::cout << "Down" << std::endl;
			msg.linear.x = -0.5;
			msg.angular.z = 0;
			pub.publish(msg);
			rate.sleep();
			msg.linear.x = 0;
			msg.angular.z = 0;
			break;
		case KEY_LEFT:
			std::cout << "Left" << std::endl; // key left
			rotate(1, (90 * PI / 180), true);
			break;

		case KEY_RIGHT:
			std::cout << "Right" << std::endl; // key right
			rotate(1, (90 * PI / 180), false);
			break;

		default:
			continue;
		}
	}
	return 0;
}
