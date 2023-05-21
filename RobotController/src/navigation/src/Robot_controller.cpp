#include "../include/Robot_controller.h"
    double robot_controller::Localization_x = 0;
    double robot_controller::Localization_y = 0;
    double robot_controller::Localization_theta = 0;
    robot_controller::robot_controller(){
        //Instances 
        char **argv;
        int argc = 0;
        ros::init(argc, argv, "navigator");
        ros::NodeHandle node;

        RobotController =  node.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 10);
        this->Object_Detection = node.subscribe("objectFound", 1, &robot_controller::objectFound, this);
        this->Localiaztion = node.subscribe("Location", 1, &robot_controller::localization, this);
        rate = 100;
        max_speed_move = 0.1;
        max_speed_rotate = 0.3;
        tolerance_move = 0.05;
        tolerance_rotate = 0.05;
        kp_move = 0.3;
        kp_rotate = 0.2;
    }
    
    double robot_controller::getDistance(double x, double y){
          return sqrt(pow(x - robot_controller::Localization_x, 2) + pow(y - robot_controller::Localization_y, 2));
    }
    double robot_controller::getOreintation(double x, double y){
          double rotate = 0;
          rotate = atan2((y - robot_controller::Localization_y), (x - robot_controller::Localization_x)) - Localization_theta;       
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

    void robot_controller::objectFound(const std_msgs::Bool::ConstPtr &msg){
          if (msg->data)
            {
            ROS_INFO("Received true");
            }
            else
            {
            ROS_INFO("Received false");
            }
    }
    void robot_controller::localization(const geometry_msgs::Pose2D::ConstPtr &msg){
          robot_controller::Localization_x = msg->x;
          robot_controller::Localization_y = msg->y;
          robot_controller::Localization_theta = msg->theta;
    }

    void robot_controller::set_ros_rate(int Rate){
        rate = Rate;
    }
    
    void robot_controller::set_max_speed_move(double speed){
        max_speed_move = speed;
    }
    void robot_controller::set_max_speed_rotate(double speed){
        max_speed_rotate = speed;
    }

    void robot_controller::set_tolerance_move(double tolerance){
        tolerance_move = tolerance;
    }

    void robot_controller::set_tolerance_rotate(double tolerance){
        tolerance_rotate = tolerance;
    }
    
    void robot_controller::set_kp_move(double kp){
        kp_move = kp;
    }
    void robot_controller::set_kp_rotate(double kp){
        kp_rotate = kp;
    }

    double robot_controller::get_max_speed_move(){
        return max_speed_move;
    }
    double robot_controller::get_max_speed_rotate(){
        return max_speed_rotate;
    }

    double robot_controller::get_tolerance_move(){
        return tolerance_move;
    }
    double robot_controller::get_tolerance_rotate(){
        return tolerance_rotate;
    }
    double robot_controller::get_kp_move(){
        return kp_move;
    }
    double robot_controller::get_kp_rotate(){
        return kp_rotate;
    }

    void robot_controller::move(double x, double y){
        ros::Rate move_rate(rate);
        double distance_to_travel = 0;
        double prev_distance_to_travel = distance_to_travel;
        distance_to_travel = getDistance(x, y);
        bool distance_is_decreasing = (distance_to_travel - prev_distance_to_travel) > -0.01;
        while (distance_to_travel > tolerance_move || distance_is_decreasing)
        {
        cmd_msg.linear.x = kp_move * distance_to_travel;
        cmd_msg.linear.x = std::min(cmd_msg.linear.x, max_speed_move);
        cmd_msg.linear.x = std::max(cmd_msg.linear.x, -max_speed_move);
        RobotController.publish(cmd_msg);
        ros::spinOnce();
        move_rate.sleep();
        prev_distance_to_travel = distance_to_travel;
        distance_to_travel = getDistance(x, y);
        distance_is_decreasing = (distance_to_travel - prev_distance_to_travel) < -0.01;
        // std::cout << "0DISTANCE2 LEFT: " << distance_to_travel << " - " << robot_controller::Localization_x << " - " << robot_controller::Localization_y << std::endl;
        }
        // std::cout << "DISTANCE2 LEFT: " << distance_to_travel << " - " << Localization_x << " - " << Localization_y << std::endl;
        cmd_msg.linear.x = 0;
        RobotController.publish(cmd_msg);
        ros::spinOnce();
    }
    void robot_controller::rotate(double x, double y){
        ros::Rate rotation_rate(rate);

        while (std::abs(getOreintation(x, y)) > 0.05)
        {
        // Adjust angular velocity using proportional control
        // std::cout << "ANGLE LEFT: " << getOreintation(x, y) * (180 / M_PI) << " - " << Localization_x << " - " << Localization_y << std::endl;
        cmd_msg.angular.z = kp_rotate * getOreintation(x, y);
        

        // Limit maximum angular velocity to avoid overshooting
        cmd_msg.angular.z = std::min(cmd_msg.angular.z, max_speed_rotate);
        cmd_msg.angular.z = std::max(cmd_msg.angular.z, -max_speed_rotate);

        // std::cout << "/* Anguler Speed */ : " << cmd_msg.angular.z << std::endl;
        // Publish Twist message and update ROS
        RobotController.publish(cmd_msg);
        ros::spinOnce();
        rotation_rate.sleep();
        ros::spinOnce();
        }

        // Stop the robot after rotation is complete
        cmd_msg.angular.z = 0;
        RobotController.publish(cmd_msg);
        ros::spinOnce();
        rotation_rate.sleep();
    }
    void robot_controller::go_to_goal(std::vector<Point> Path){
        ros::Rate rate(rate);
        // std::chrono::milliseconds duration(5000);
        // std::this_thread::sleep_for(duration);
        std::cout << "PATH SIZE: "<< Path.size()<<"\n";
        while(!Path.empty()){
        std::cout << "----------- GOING TO GOAL -----------" << std::endl;
        std::cout << "Goal(x, y): " << std::get<0>(Path.front().getCoordinate()) * 0.4 << ", " << std::get<1>(Path.front().getCoordinate()) * 0.4 << std::endl;
        std::cout << "Rotation: " << getOreintation(std::get<0>(Path.front().getCoordinate())* 0.4, std::get<1>(Path.front().getCoordinate())* 0.4) * (180 / M_PI) << std::endl;
        std::cout << "Distance: " << getDistance(std::get<0>(Path.front().getCoordinate())* 0.4, std::get<1>(Path.front().getCoordinate())* 0.4) << std::endl;
        ros::spinOnce();
        rate.sleep();
        std::cout << "----------- Start Rotation -----------" << std::endl;
        rotate(std::get<0>(Path.front().getCoordinate())*0.4, std::get<1>(Path.front().getCoordinate())*0.4);
        std::cout << "----------- Rotation is Done -----------" << std::endl;
        ros::spinOnce();
        rate.sleep();
        // std::this_thread::sleep_for(duration);
        std::cout << "----------- Start Moving -----------" << std::endl;
        move(std::get<0>(Path.front().getCoordinate())*0.4, std::get<1>(Path.front().getCoordinate())*0.4);
        std::cout << "----------- Moving is Done -----------" << std::endl;
        ros::spinOnce();
        rate.sleep();
        std::cout << "----------- GOAL DONE -----------" << std::endl;
        Path.erase(Path.begin());
        }
    }
