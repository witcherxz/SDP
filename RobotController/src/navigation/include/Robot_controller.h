#include <iostream>
#include <ros/ros.h>
#include <vector>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "std_msgs/Bool.h"
#include "map.h"
#include "map_creator.h"

class robot_controller{
protected:

    //Instances
    ros::Publisher RobotController;
    ros::Subscriber Object_Detection;
    ros::Subscriber Localiaztion;
    geometry_msgs::Twist cmd_msg;
    geometry_msgs::Pose2D goal_pose;
    std_msgs::Bool Object_Found;
    
    int rate;
    double cellSize;
    double max_speed_move;
    double max_speed_rotate;
    double tolerance_move;
    double tolerance_rotate;
    double kp_move;
    double kp_rotate;


public:
    static double Localization_x;
    static double Localization_y;
    static double Localization_theta;
    robot_controller(double cellSize);
    
    double getDistance(double x, double y);
    double getOreintation(double x, double y);

    void objectFound(const std_msgs::Bool::ConstPtr &msg);
    void localization(const geometry_msgs::Pose2D::ConstPtr &msg);

    void set_ros_rate(int rate);
    
    void set_max_speed_move(double speed);
    void set_max_speed_rotate(double speed);

    void set_tolerance_move(double tolerance);
    void set_tolerance_rotate(double tolerance);
    void set_kp_move(double kp);
    void set_kp_rotate(double kp);


    double get_max_speed_move();
    double get_max_speed_rotate();
    double get_tolerance_move();
    double get_tolerance_rotate();
    double get_kp_move();
    double get_kp_rotate();
    std::tuple<double, double, double> get_current_location();
    void Object_check();
    void move(double x, double y);
    void rotate(double x, double y);
    void going_to_goal(double x, double y);
    void go_to_goal(std::vector<Point> Path);

    void info();

};
