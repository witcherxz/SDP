#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"
// #include "map.h"
// #include "map_creator.h"

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <math.h>
#include <chrono>
#include <thread>

// #include <opencv2/core.hpp>
// #include <opencv2/core/utility.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/calib3d.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/videoio.hpp>
// #include <opencv2/highgui.hpp>

static float pi = 3.1415926535897;
static float initial_x = 0;
static float initial_y = 0;
static float initial_theta = 0;

static float final_x = 0;
static float final_y = 0;
static float final_theta = 0;
static float current_angle = 0;
geometry_msgs::Twist msgofpath;

static float moved_distance_x = 0;
static float moved_distance_y = 0;
static float moved_distance_theta = 0;
static int choice;
void updateCurrentDistance(const tf2_msgs::TFMessage::ConstPtr &msg)
{
  moved_distance_x = msg->transforms[0].transform.translation.x;
  moved_distance_y = msg->transforms[0].transform.translation.y;
  moved_distance_theta = msg->transforms[0].transform.rotation.z;
}

double getDistance(double x, double y)
{
  return sqrt(pow(x - moved_distance_x, 2) + pow(y - moved_distance_y, 2));
}

double getOreintation(double x, double y)
{
  double rotate = 0;
  std::cout << ":---------------------------------- For Oreintation ---------------------------------- " << std::endl;
  std::cout << " x: " << x << std::endl;
  std::cout << " y: " << y << std::endl;
  std::cout << " Current angle: " << current_angle * 180 / pi << std::endl;
  std::cout << " Oreintation: " << (atan2((y - moved_distance_y), (x - moved_distance_x))) * 180 / pi << std::endl;
  rotate = current_angle + atan2((y - moved_distance_y), (x - moved_distance_x));
  current_angle = -atan2((y - moved_distance_y), (x - moved_distance_x));
  std::cout << " Current angle after rotation: " << current_angle * 180 / pi << std::endl;
  std::cout << ":------------------------------------------------------------------------------------- " << std::endl;
  return rotate;
}

void move(double distance, bool isForward)
{
  ros::NodeHandle n;
  ros::Publisher path = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);
  ros::Rate rate(50);

  double final_distance;
  double inital_x = moved_distance_x;
  double inital_y = moved_distance_y;
  double moved_distance = getDistance(inital_x, inital_y);

  if (distance <= 1 && isForward)
  {
    msgofpath.linear.x = 0.3;
    final_distance = moved_distance + distance;
  }
  else if (distance <= 1 && !isForward)
  {
    msgofpath.linear.x = -0.3;
    final_distance = moved_distance + distance;
  }
  else
  {
    if (isForward)
      msgofpath.linear.x = 1;
    else
      msgofpath.linear.x = -1;

    final_distance = moved_distance + distance - 1;
  }

  do
  {
    if (moved_distance > final_distance)
      break;
    path.publish(msgofpath);
    ros::spinOnce();
    rate.sleep();
    moved_distance = getDistance(inital_x, inital_y);
  } while (moved_distance < final_distance);

  msgofpath.linear.x = 0;
  path.publish(msgofpath);
  ros::spinOnce();
}

void rotate(double angular_speed, double relative_angle, bool isClockWise)
{
  ros::NodeHandle n;
  ros::Publisher path = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);
  ros::Rate rate(50);

  if (isClockWise)
    msgofpath.angular.z = abs(angular_speed);
  else
    msgofpath.angular.z = -abs(angular_speed);
  double current_angle = 0;
  double t0 = ros::Time::now().toSec();
  do
  {
    path.publish(msgofpath);
    double t1 = ros::Time::now().toSec();
    current_angle = angular_speed * (t1 - t0);
    ros::spinOnce();
    rate.sleep();
  } while (current_angle < relative_angle);
  msgofpath.angular.z = 0;
  path.publish(msgofpath);
  ros::spinOnce();
}

// void goToGoal(geometry_msgs::Pose goal_pose, double ditsance_tolrance)
// {
//   ros::NodeHandle n;
//   ros::Publisher path = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);
//   ros::Rate rate(50);
//   double E = 0;

//   do
//   {
//     double kp = 1.0;
//     // double ki = 0.02;
//     double e = getDistance(goal_pose.position.x, goal_pose.position.y);
//     std::cout << "Distance:" << e << std::endl;
//     // double E = E + e;
//     msgofpath.linear.x = kp * e;

//     rotate(1, , true);
//     path.publish(msgofpath);
//     ros::spinOnce();
//     rate.sleep();
//   } while (getDistance(goal_pose.position.x, goal_pose.position.y) > ditsance_tolrance);
//   std::cout << "End Move to go" << std::endl;
//   msgofpath.linear.x = 0;
//   msgofpath.angular.z = 0;
//   path.publish(msgofpath);
// }

void Perfectmove(geometry_msgs::Pose goal_pose)
{
  ros::NodeHandle n;
  ros::Publisher path = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);
  ros::Rate rate(50);

  double distance = getDistance(goal_pose.position.x, goal_pose.position.y);
  std::cout << ":---------------------------------- For Distance ---------------------------------- " << std::endl;
  std::cout << " x: " << goal_pose.position.x << std::endl;
  std::cout << " y: " << goal_pose.position.y << std::endl;
  std::cout << " Distance: " << distance << std::endl;
  std::cout << ":------------------------------------------------------------------------------------- " << std::endl;
  double oreintation = getOreintation(goal_pose.position.x, goal_pose.position.y);
  if (oreintation < 0)
    rotate(1, -oreintation, false);
  else
    rotate(1, oreintation, true);

  move(distance, true);
}

void finalPose(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  std::cout << "User inputs recived" << std::endl;
  geometry_msgs::Pose goal_pose;
  ROS_INFO_STREAM("Received message: " << msg->x);
  goal_pose.position.x = msg->x;
  ROS_INFO_STREAM("Received message: " << msg->y);
  goal_pose.position.y = msg->y;
  ROS_INFO_STREAM("Received message: " << msg->theta);
  goal_pose.orientation.z = msg->theta;
  std::cout << "Choose mode: 1- move forward | 2- Rotate | 3- Go-to-Goal" << std::endl;
  std::cin >> choice;
  if (choice == 1)
    move(goal_pose.position.x, true);
  else if (choice == 2)
    rotate(1, goal_pose.orientation.z, true);
  else if (choice == 3)
    Perfectmove(goal_pose);
  else
    std::cout << "Wrong choice" << std::endl;
}
void initialPose(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  std::cout << "Localization recived" << std::endl;
  ROS_INFO_STREAM("Received message: " << msg->x);
  initial_x = msg->x;
  ROS_INFO_STREAM("Received message: " << msg->y);
  initial_y = msg->y;
  ROS_INFO_STREAM("Received message: " << msg->theta);
  initial_theta = msg->theta;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigator");
  ros::NodeHandle n;
  ros::Publisher path = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);
  ros::Rate rate(10);

  ros::Subscriber sub1 = n.subscribe("finalPose", 100, finalPose);
  ros::Subscriber sub2 = n.subscribe("initialPose", 100, initialPose);
  ros::Subscriber sub3 = n.subscribe("/tf", 100, updateCurrentDistance);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}