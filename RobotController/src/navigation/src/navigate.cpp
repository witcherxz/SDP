#include "../include/navigation.h"
#include "../include/Robot_controller.h"


GridMap lab_Map = GridMap(15, 15, 0.4);

Line l1 = Line(Point(0, 1), Point(3,1));
Line l2 = Line(Point(5,0), Point(5,4));
Line l3 = Line(Point(2,4), Point(5,4));
Line l4 = Line(Point(2,4), Point(2,6));

#define TOLERANCE 0.05
#define Kp 0.3

// UserInput Variables
static float goal_x = 0;
static float goal_y = 0;
static float goal_theta = 0;

// Localization Variables
static float Localization_x = 0;
static float Localization_y = 0;
static float Localization_theta = 0;

// Odometry Variables
static float odom_translation_x = 0;
static float odom_translation_y = 0;
static float odom_translation_z = 0;
static double roll;
static double pitch;
static double yaw;

// Nodes pub/sub Handling Methods

// Odmetery Updadting
void objectFound(const std_msgs::Bool::ConstPtr &msg)
{
  if (msg->data)
  {
    ROS_INFO("Received true");
  }
  else
  {
    ROS_INFO("Received false");
  }
}
// Localiaztion Updating
void localization(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  // std::cout << "Localization recived" << std::endl;
  Localization_x = msg->x;
  Localization_y = msg->y;
  Localization_theta = msg->theta;

  // std::cout << "X: " << Localization_x << std::endl;
  // std::cout << "Y: " << Localization_y << std::endl;
  // std::cout << "Theta: " << Localization_theta * (180 / M_PI) << std::endl;
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
  ros::spinOnce();
  return sqrt(pow(x - Localization_x, 2) + pow(y - Localization_y, 2));
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
  ros::spinOnce();
  double rotate = 0;
  rotate = atan2((y - Localization_y), (x - Localization_x)) - Localization_theta;

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

void move(ros::Publisher &robot_controller, ros::Rate &rate, geometry_msgs::Twist &cmd_msg, geometry_msgs::Pose2D &goal_pose, bool isLast)
{
  double desired_Destination;
  double inital_x = Localization_x;
  double inital_y = Localization_y;
  double current_distance_traveled = getDistance(inital_x, inital_y);
  double distance_to_travel = getDistance(goal_pose.x, goal_pose.y);

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

void move1(ros::Publisher &robot_controller, ros::Rate &rate, geometry_msgs::Twist &cmd_msg, geometry_msgs::Pose2D &goal_pose)
{
  // double desired_Destination;
  double distance_to_travel = 0;
  double inital_x = Localization_x;
  double inital_y = Localization_y;
  double current_distance_traveled = getDistance(inital_x, inital_y);
  double prev_distance_to_travel = distance_to_travel;
  distance_to_travel = getDistance(goal_pose.x, goal_pose.y);
  bool distance_is_decreasing = (distance_to_travel - prev_distance_to_travel) > -0.01;
  while (distance_to_travel > TOLERANCE || distance_is_decreasing)
  {
    cmd_msg.linear.x = Kp * distance_to_travel;
    if (cmd_msg.linear.x > 0.1)
      cmd_msg.linear.x = 0.1;
    robot_controller.publish(cmd_msg);
    ros::spinOnce();
    rate.sleep();
    prev_distance_to_travel = distance_to_travel;
    distance_to_travel = getDistance(goal_pose.x, goal_pose.y);
    distance_is_decreasing = (distance_to_travel - prev_distance_to_travel) < -0.01;
    // std::cout << "0DISTANCE2 LEFT: " << distance_to_travel << " - " << Localization_x << " - " << Localization_y << std::endl;
  }
  // std::cout << "DISTANCE2 LEFT: " << distance_to_travel << " - " << Localization_x << " - " << Localization_y << std::endl;
  cmd_msg.linear.x = 0;
  robot_controller.publish(cmd_msg);
  ros::spinOnce();
}

void rotate(ros::Publisher &robot_controller, ros::Rate &rate, geometry_msgs::Twist &cmd_msg, geometry_msgs::Pose2D &goal_pose)
{
  double oreintation = getOreintation(goal_pose.x, goal_pose.y);
  double desired_Angle = Localization_theta + oreintation;
  std::cout << "Rotation: " << oreintation * (180 / M_PI) << std::endl;
  std::cout << "Goal(x, y): " << goal_pose.x << ", " << goal_pose.y << std::endl;
  ros::Rate rate1(48);
  if (oreintation > 0)
  {
    cmd_msg.angular.z = 1.2;
  }
  else
  {
    cmd_msg.angular.z = -1.2;
  }
  double current_angle = 0;
  double t0 = ros::Time::now().toSec();
  while (current_angle < abs(oreintation))
  {
    robot_controller.publish(cmd_msg);
    double t1 = ros::Time::now().toSec();
    current_angle = 1.2 * (t1 - t0);
    ros::spinOnce();
    rate1.sleep();
  }
  cmd_msg.angular.z = 0;
  robot_controller.publish(cmd_msg);
  ros::spinOnce();
  rate.sleep();
}

void rotate1(ros::Publisher &robot_controller, ros::Rate &rate, geometry_msgs::Twist &cmd_msg, geometry_msgs::Pose2D &goal_pose)
{
  double orientation = getOreintation(goal_pose.x, goal_pose.y);

  // Define proportional controller gains
  double kp = 0.2;

  // Set initial angular velocity
  double angular_vel = 1.2;
  while (std::abs(orientation) > 0.05)
  {
    // Adjust angular velocity using proportional control
    // std::cout << "ANGLE LEFT: " << orientation * (180 / M_PI) << " - " << Localization_x << " - " << Localization_y << std::endl;
    angular_vel = kp * orientation;

    // Limit maximum angular velocity to avoid overshooting
    angular_vel = std::min(angular_vel, 0.3);
    angular_vel = std::max(angular_vel, -0.3);

    // Update Twist message with new angular velocity
    cmd_msg.angular.z = angular_vel;
    std::cout << "/* Anguler Speed */ : " << angular_vel << std::endl;
    // Publish Twist message and update ROS
    robot_controller.publish(cmd_msg);
    ros::spinOnce();
    rate.sleep();
    ros::spinOnce();
    orientation = getOreintation(goal_pose.x, goal_pose.y);
  }

  // Stop the robot after rotation is complete
  cmd_msg.angular.z = 0;
  robot_controller.publish(cmd_msg);
  ros::spinOnce();
  rate.sleep();
}

void go_to_goal(ros::Publisher &robot_controller, ros::Rate &rate, geometry_msgs::Twist &cmd_msg, geometry_msgs::Pose2D &goal_pose)
{
  // std::chrono::milliseconds duration(5000);
  // std::this_thread::sleep_for(duration);
  std::cout << "----------- GOING TO GOAL -----------" << std::endl;
  std::cout << "Goal(x, y): " << goal_pose.x << ", " << goal_pose.y << std::endl;
  std::cout << "Rotation: " << getOreintation(goal_pose.x, goal_pose.y) * (180 / M_PI) << std::endl;
  std::cout << "Distance: " << getDistance(goal_pose.x, goal_pose.y) << std::endl;
  ros::spinOnce();
  rate.sleep();
  std::cout << "----------- Start Rotation -----------" << std::endl;
  rotate1(robot_controller, rate, cmd_msg, goal_pose);
  std::cout << "----------- Rotation is Done -----------" << std::endl;
  ros::spinOnce();
  rate.sleep();
  // std::this_thread::sleep_for(duration);
  std::cout << "----------- Start Moving -----------" << std::endl;
  move1(robot_controller, rate, cmd_msg, goal_pose);
  std::cout << "----------- Moving is Done -----------" << std::endl;
  ros::spinOnce();
  rate.sleep();
  std::cout << "----------- GOAL DONE -----------" << std::endl;
}

std::stack<std::pair<double, double>> Robotpath;

void pathQueue_init(std::vector<Point> path)
{

  double path_angle = 0;
  std::tuple<double, double> points;

  // points = path.back().getCoordinate();
  // Robotpath.push(std::make_pair(std::get<0>(points) * 0.4, std::get<1>(points) * 0.4));
  // path.pop_back();

  while (!path.empty())
  {
    points = path.back().getCoordinate();
    // if ((std::abs(std::get<0>(points) - Robotpath.top().first) == 0 || std::abs(std::get<1>(points) - Robotpath.top().second) == 0))
    // {
    //   path.pop_back();
    // }
    // else
    // {
    //   Robotpath.push(std::make_pair(std::get<0>(points) * 0.4, std::get<1>(points) * 0.4));
    //   path.pop_back();
    // }
    Robotpath.push(std::make_pair(std::get<0>(points) * 0.4, std::get<1>(points) * 0.4));
    path.pop_back();
  }
  Robotpath.pop();
}
int main(int argc, char **argv)
{
  std::cout << "----------- Start Navigation -----------" << std::endl;
  ros::init(argc, argv, "navigator");
  ros::NodeHandle n;

  // Robot Parameters Handling
  ros::Publisher robot_controller = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 10);
  ros::Rate rate(100);
  geometry_msgs::Twist cmd_msg;
  geometry_msgs::Pose2D goal_pose;

  // Navigation Node subscribe Handling
  ros::Subscriber sub1 = n.subscribe("/objectFound", 1, objectFound);
  ros::Subscriber sub2 = n.subscribe("/Location", 1, localization);
  int loop = 0;
  // pathQueue_init();
  std::pair<double, double> popped_pair;

  MapCreator a(lab_Map);
  a.addLine(l1);
  a.addLine(l2);
  a.addLine(l3);
  a.addLine(l4);

  while (Localization_x == 0)
  {
    ros::spinOnce();
    rate.sleep();
  };
  Point start = Point(Localization_x / 0.4, Localization_y / 0.4);
  Point goal = Point(5, 5);
  pathQueue_init(findShortestPath(start, goal, lab_Map));
  ros::Duration(3).sleep();
  while (ros::ok())
  {
    popped_pair = Robotpath.top();
    Robotpath.pop();
    goal_pose.x = popped_pair.first;
    goal_pose.y = popped_pair.second;
    // std::chrono::milliseconds duration(5000);
    // std::this_thread::sleep_for(duration);
    go_to_goal(robot_controller, rate, cmd_msg, goal_pose);
    ros::spinOnce();

    if (Robotpath.empty())
    {
      ros::spin();
      pathQueue_init(findShortestPath(start, goal, lab_Map));
    }
  }
  return 0;
}