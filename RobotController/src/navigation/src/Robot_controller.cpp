#include "../include/Robot_controller.h"
    double robot_controller::Localization_x = 0;
    double robot_controller::Localization_y = 0;
    double robot_controller::Localization_theta = 0;
    robot_controller::robot_controller(double cellSize){
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
        tolerance_move = 0.1;
        tolerance_rotate = 0.05;
        kp_move = 0.3;
        kp_rotate = 0.2;
        this->cellSize = cellSize;
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
        Object_Found.data = msg->data;
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

    std::tuple<double, double, double> robot_controller::get_current_location(){
        return std::make_tuple(robot_controller::Localization_x, robot_controller::Localization_y, robot_controller::Localization_theta);
    }

void robot_controller::Object_check(){
    ros::Rate object_rate(rate);
    while(Object_Found.data){
    cmd_msg.linear.x = 0;
    cmd_msg.angular.z = 0;
    RobotController.publish(cmd_msg);
    ros::spinOnce();
    object_rate.sleep();
    }
}


    void robot_controller::move(double x, double y){
        ros::Rate move_rate(rate);
        // while (distance_to_travel > tolerance_move || distance_is_decreasing)
        while (getDistance(x, y) > tolerance_move){
        cmd_msg.linear.x = kp_move * getDistance(x, y);
        cmd_msg.linear.x = std::min(cmd_msg.linear.x, max_speed_move);
        cmd_msg.linear.x = std::max(cmd_msg.linear.x, -max_speed_move);
        Object_check();
        RobotController.publish(cmd_msg);
        ros::spinOnce();
        move_rate.sleep();
        std::cout << "Distance Left: " << getDistance(x, y) << std::endl;
        }
        cmd_msg.linear.x = 0;
        RobotController.publish(cmd_msg);
        ros::spinOnce();
    }
    void robot_controller::rotate(double x, double y){
        ros::Rate rotation_rate(rate);

        while (std::abs(getOreintation(x, y)) > 0.05)
        {
        // Adjust angular velocity using proportional control
        cmd_msg.angular.z = kp_rotate * getOreintation(x, y);
        // Limit maximum angular velocity to avoid overshooting
        cmd_msg.angular.z = std::min(cmd_msg.angular.z, max_speed_rotate);
        cmd_msg.angular.z = std::max(cmd_msg.angular.z, -max_speed_rotate);
        Object_check();
        RobotController.publish(cmd_msg);
        ros::spinOnce();
        rotation_rate.sleep();
        // std::cout << "Rotation Left: " << getOreintation(x, y) * (180/M_PI) << std::endl;
        }

        // Stop the robot after rotation is complete
        cmd_msg.angular.z = 0;
        RobotController.publish(cmd_msg);
        ros::spinOnce();
        rotation_rate.sleep();
    }

    void robot_controller::going_to_goal(double x, double y){
        ros::Rate move_rate(rate);

        while (getDistance(x,y) > tolerance_move)
        {
        cmd_msg.linear.x = kp_move * getDistance(x, y);
        cmd_msg.linear.x = std::min(cmd_msg.linear.x, max_speed_move);
        cmd_msg.linear.x = std::max(cmd_msg.linear.x, -max_speed_move);

        // Adjust angular velocity using proportional control
        cmd_msg.angular.z = kp_rotate * getOreintation(x, y);
        cmd_msg.angular.z = std::min(cmd_msg.angular.z, max_speed_rotate);
        cmd_msg.angular.z = std::max(cmd_msg.angular.z, -max_speed_rotate);
        // Limit maximum angular velocity to avoid overshooting
        
        Object_check();
        RobotController.publish(cmd_msg);
        ros::spinOnce();
        move_rate.sleep();
        // std::cout << "Rotation Left: " << getOreintation(x, y) * (180/M_PI) << std::endl;
        }
        // Stop the robot after rotation is complete
        cmd_msg.linear.x = 0;
        cmd_msg.angular.z = 0;
        RobotController.publish(cmd_msg);
        ros::spinOnce();
        move_rate.sleep();
    }



    void robot_controller::go_to_goal(std::vector<Point> Path){
        ros::Rate rate(rate);
        // std::chrono::milliseconds duration(5000);
        // std::this_thread::sleep_for(duration);
        std::cout << "PATH SIZE: "<< Path.size()<<"\n";
        while(!Path.empty()){
        std::cout << "----------- GOING TO GOAL -----------" << std::endl;
        std::cout << "Goal(x, y): " << std::get<0>(Path.front().getCoordinate()) * cellSize << ", " << std::get<1>(Path.front().getCoordinate()) * cellSize << std::endl;
        std::cout << "Rotation: " << getOreintation(std::get<0>(Path.front().getCoordinate())* cellSize, std::get<1>(Path.front().getCoordinate())* cellSize) * (180 / M_PI) << std::endl;
        std::cout << "Distance: " << getDistance(std::get<0>(Path.front().getCoordinate())* cellSize, std::get<1>(Path.front().getCoordinate())* cellSize) << std::endl;
        std::cout << "----------- Going to Goal -----------" << std::endl;
        rotate(std::get<0>(Path.front().getCoordinate())*cellSize, std::get<1>(Path.front().getCoordinate())*cellSize);
        going_to_goal(std::get<0>(Path.front().getCoordinate())*cellSize, std::get<1>(Path.front().getCoordinate())*cellSize);
        std::cout << "Distance left: " << getDistance(std::get<0>(Path.front().getCoordinate())* cellSize, std::get<1>(Path.front().getCoordinate())* cellSize) << std::endl;
        Path.erase(Path.begin());
        ros::spinOnce();
        rate.sleep();
        }
    
    }
    
    void robot_controller::info(){
        std::cout << "----------- Robot Controller -----------" << std::endl;
        std::tuple<double, double, double> temp = robot_controller::get_current_location();
        std::cout << "X:     "<< std::get<0>(temp) << std::endl;
        std::cout << "Y:     "<< std::get<1>(temp) << std::endl;
        std::cout << "Theta: "<< std::get<2>(temp) * (180/M_PI) << u8"\u00B0" << std::endl;
        std::cout << "********** Rotate information **********" << std::endl;
        std::cout << "Maximmum speed: "<<  robot_controller::get_max_speed_rotate() * (180/M_PI) <<u8"\u00B0"<<"/s"<< std::endl;
        std::cout << "Constant Kp:    "<<  robot_controller::get_kp_rotate() << std::endl;
        std::cout << "Tolerance:      "<<  robot_controller::get_tolerance_rotate() * (180/M_PI) << u8"\u00B0" << std::endl;
        std::cout << "*********** Move information ***********" << std::endl;
        std::cout << "Maximmum speed: "<<  robot_controller::get_max_speed_move() <<"m/s"<< std::endl;
        std::cout << "Constant Kp:    "<<  robot_controller::get_kp_move() << std::endl;
        std::cout << "Tolerance:      "<<  robot_controller::get_tolerance_move() <<"m"<< std::endl;        
        std::cout << "----------------------------------------" << std::endl;
    }
