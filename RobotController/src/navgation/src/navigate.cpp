#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "tf/transform_datatypes.h"

// #include
// #include "map.h"
// #include "map_creator.h"
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <math.h>
#include <cmath>
#include <chrono>
#include <thread>

// #include <opencv2/core.hpp>
// #include <opencv2/core/utility.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/calib3d.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/videoio.hpp>
// #include <opencv2/highgui.hpp>

// UserInput Variables
static float goal_x = 0;
static float goal_y = 0;
static float goal_theta = 0;

// Localization Variables
static float Localization_x = 0;
static float Localization_y = 0;
static float Localization_theta = 0;
static float current_angle = 0;

// Odometry Variables
static float odom_translation_x = 0;
static float odom_translation_y = 0;
static float odom_translation_z = 0;
static float odom_rotation_x = 0;
static float odom_rotation_y = 0;
static float odom_rotation_z = 0;
static float odom_rotation_w = 0;

// Nodes pub/sub Handling Methods

// Odmetery Updadting
void Odometer_Updateing(const tf2_msgs::TFMessage::ConstPtr &msg)
{
  // Translation
  odom_translation_x = msg->transforms[0].transform.translation.x;
  odom_translation_y = msg->transforms[0].transform.translation.y;
  odom_translation_z = msg->transforms[0].transform.translation.z;
  // Rotation
  odom_rotation_x = msg->transforms[0].transform.rotation.x;
  odom_rotation_y = msg->transforms[0].transform.rotation.y;
  odom_rotation_z = msg->transforms[0].transform.rotation.z;
  odom_rotation_w = msg->transforms[0].transform.rotation.w;
}
// Localiaztion Updating
void Camera_Updating(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  std::cout << "Localization recived" << std::endl;
  ROS_INFO_STREAM("Received message: " << msg->x);
  Localization_x = msg->x;
  ROS_INFO_STREAM("Received message: " << msg->y);
  Localization_y = msg->y;
  ROS_INFO_STREAM("Received message: " << msg->theta);
  Localization_theta = msg->theta;
}
// UserInput Updating
void UserInput_Updating(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  std::cout << "User inputs recived" << std::endl;
  ROS_INFO_STREAM("Received message: " << msg->x);
  goal_x = msg->x;
  ROS_INFO_STREAM("Received message: " << msg->y);
  goal_y = msg->y;
  ROS_INFO_STREAM("Received message: " << msg->theta);
  goal_theta = msg->theta;
}

// Helper Methodes Section

/**
Calculates the Euclidean distance between the current position of a robot
and a target position in a 2D space.
@param x - The x-coordinate of the target position.
@param y - The y-coordinate of the target position.
@return The Euclidean distance between the current position and the target position.
@remarks This function uses the Pythagorean theorem to calculate the distance
between two points, given their respective x and y coordinates. The function
first calculates the squared difference between the x and y coordinates of the
current position and the target position, and then takes the square root of the
sum of the squares to obtain the distance. The x and y coordinates of the current
position are obtained from the global variables odom_translation_x and odom_translation_y.
@see https://en.wikipedia.org/wiki/Pythagorean_theorem
*/
double getDistance(double x, double y)
{
  return sqrt(pow(x - odom_translation_x, 2) + pow(y - odom_translation_y, 2));
}

/**
Calculates the angle that a robot needs to rotate to face a target position
in a 2D space.
@param x - The x-coordinate of the target position.
@param y - The y-coordinate of the target position.
@return The angle, in radians, that the robot needs to rotate to face the target position.
@remarks This function calculates the angle between the current position and
the target position, given their respective x and y coordinates. The function
uses the arctangent function to determine the angle, and then subtracts the
current angle of the robot from the calculated angle to obtain the angle of
rotation. The current angle of the robot is updated to be equal to the angle
between the current position and the target position. The returned angle is
expressed in radians.
@see https://en.wikipedia.org/wiki/Atan2
*/
double getOreintation(double x, double y)
{
  double rotate = 0;
  rotate = atan2((y - odom_translation_y), (x - odom_translation_x)) - current_angle;
  current_angle = atan2((y - odom_translation_y), (x - odom_translation_x));
  if (rotate < 0)
  {
    if (rotate > -M_PI)
      return rotate;
    else
      return rotate + (2 * M_PI);
  }
  else
  {
    if (rotate < M_PI)
      return rotate;
    else
      return rotate - (2 * M_PI);
  }
  return rotate;
}

void move(ros::Publisher &robot_controller, ros::Rate &rate, geometry_msgs::Twist &cmd_msg, geometry_msgs::Pose &goal_pose, bool isLast)
{
  double desired_Destination;
  double inital_x = odom_translation_x;
  double inital_y = odom_translation_y;
  double current_distance_traveled = getDistance(inital_x, inital_y);
  double distance_to_travel = getDistance(goal_pose.position.x, goal_pose.position.y);

  if (distance_to_travel <= 1)
  {
    cmd_msg.linear.x = 0.3;
    desired_Destination = current_distance_traveled + distance_to_travel;
  }
  else if (distance_to_travel >= 1)
  {
    cmd_msg.linear.x = 1;
    if (!isLast)
      desired_Destination = current_distance_traveled + distance_to_travel - 1;
    else
      desired_Destination = current_distance_traveled + distance_to_travel;
  }

  while (current_distance_traveled < desired_Destination)
  {
    robot_controller.publish(cmd_msg);
    ros::spinOnce();
    rate.sleep();
    current_distance_traveled = getDistance(inital_x, inital_y);
  }

  cmd_msg.linear.x = 0;
  robot_controller.publish(cmd_msg);
  ros::spinOnce();
}

void move1(ros::Publisher &robot_controller, ros::Rate &rate, geometry_msgs::Twist &cmd_msg, geometry_msgs::Pose &goal_pose, bool isLast)
{
  // double desired_Destination;
  double inital_x = odom_translation_x;
  double inital_y = odom_translation_y;
  double current_distance_traveled = getDistance(inital_x, inital_y);
  double distance_to_travel = getDistance(goal_pose.position.x, goal_pose.position.y);

  while (distance_to_travel > 0.1)
  {
    double k = 0.5;
    cmd_msg.linear.x = k * distance_to_travel;
    if (cmd_msg.linear.x > 1.2)
      cmd_msg.linear.x = 1.2;
    std::cout << "distance_to_travel: " << distance_to_travel << std::endl;
    std::cout << "cmd_msg.linear.xs: " << cmd_msg.linear.x << std::endl;
    robot_controller.publish(cmd_msg);
    ros::spinOnce();
    rate.sleep();
    distance_to_travel = getDistance(goal_pose.position.x, goal_pose.position.y);
  }

  cmd_msg.linear.x = 0;
  robot_controller.publish(cmd_msg);
  ros::spinOnce();
}

void rotate(ros::Publisher &robot_controller, ros::Rate &rate, geometry_msgs::Twist &cmd_msg, geometry_msgs::Pose &goal_pose)
{
  double oreintation = getOreintation(goal_pose.position.x, goal_pose.position.y);
  if (oreintation > 0)
    cmd_msg.angular.z = 1;
  else
    cmd_msg.angular.z = -1;

  double current_angle = 0;
  double t0 = ros::Time::now().toSec();

  while (current_angle < abs(oreintation))
  {
    robot_controller.publish(cmd_msg);
    ros::spinOnce();
    rate.sleep();
    double t1 = ros::Time::now().toSec();
    current_angle = 1 * (t1 - t0);
  }
  cmd_msg.angular.z = 0;
  robot_controller.publish(cmd_msg);
  ros::spinOnce();
  rate.sleep();
}

void go_to_goal(ros::Publisher &robot_controller, ros::Rate &rate, geometry_msgs::Twist &cmd_msg, geometry_msgs::Pose &goal_pose, bool isLast)
{

  rotate(robot_controller, rate, cmd_msg, goal_pose);
  move1(robot_controller, rate, cmd_msg, goal_pose, isLast);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigator");
  ros::NodeHandle n;

  // Robot Parameters Handling
  ros::Publisher robot_controller = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 10);
  ros::Rate rate(10);
  geometry_msgs::Twist cmd_msg;
  geometry_msgs::Pose goal_pose;

  // Navigation Node subscribe Handling
  ros::Subscriber sub1 = n.subscribe("/UserInput", 100, UserInput_Updating);
  ros::Subscriber sub2 = n.subscribe("/Location", 100, Camera_Updating);
  ros::Subscriber sub3 = n.subscribe("/tf", 1, Odometer_Updateing);

  while (ros::ok())
  {
    // geometry_msgs::Pose goal_pose;
    int ti = 3;

    ros::Duration(2).sleep(); // Sleep for one second
    ros::spinOnce();

    goal_pose.position.y = 0;
    goal_pose.position.x = 2.45;
    go_to_goal(robot_controller, rate, cmd_msg, goal_pose, false);
    // go_to_goal(robot_controller, rate, cmd_msg, goal_pose, false);

    // // ros::Duration(ti).sleep(); // Sleep for one second
    ros::spinOnce();
    ros::spinOnce();

    goal_pose.position.y = 2.85;
    goal_pose.position.x = 2.45;
    go_to_goal(robot_controller, rate, cmd_msg, goal_pose, false);
    // go_to_goal(robot_controller, rate, cmd_msg, goal_pose, false);
    // // sleep(ti);
    // // ros::Duration(ti).sleep(); // Sleep for one second
    ros::spinOnce();
    ros::spinOnce();

    goal_pose.position.y = 2.85;
    goal_pose.position.x = 4.8;
    go_to_goal(robot_controller, rate, cmd_msg, goal_pose, false);
    // go_to_goal(robot_controller, rate, cmd_msg, goal_pose, false);
    // // sleep(sleep);
    // // ros::Duration(ti).sleep(); // Sleep for one second
    ros::spinOnce();
    ros::spinOnce();
    goal_pose.position.y = 0;
    goal_pose.position.x = 4.8;
    go_to_goal(robot_controller, rate, cmd_msg, goal_pose, false);
    // go_to_goal(robot_controller, rate, cmd_msg, goal_pose, false);
    // // sleep(sleep);
    // // ros::Duration(ti).sleep(); // Sleep for one second
    ros::spinOnce();
    ros::spinOnce();
    // break;

    goal_pose.position.y = 0;
    goal_pose.position.x = 0;
    go_to_goal(robot_controller, rate, cmd_msg, goal_pose, false);

    ros::spinOnce();
    rate.sleep();
    break;
  }
  return 0;
}