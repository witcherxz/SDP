#include "../RobotVision/aruco_scanner.h"
#include <opencv2/core/types.hpp>
#include "geometry_msgs/Pose2D.h"
#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include <tf2_msgs/TFMessage.h>
#include "tf/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/TransformStamped.h"
#include <math.h>

class Localization
{
    ArucoScanner arucoScanner = ArucoScanner();
    cv::Point_<double> pose;
    std::unordered_map<int, cv::Mat> arucosCoef;
    void loadArucosCoef();
    void publish(cv::Mat pose);
    void odometerUpdateing(const tf2_msgs::TFMessage::ConstPtr &msg);
    cv::Mat getPostionFromMarker();
    ros::Publisher publisher;
    ros::Subscriber odometerSubscriber;
    
    const float angleOffset = M_PI / 2;
    bool firstTimeNewArucoDetected = false;
    bool firstOdomRead = true;
    float x = 0;
    float y = 0;
    float theta = 0;
    float start_theta = 0;

    float first_x = 0;
    float first_y = 0;
    float first_theta = 0;

    float last_x = 0;
    float last_y = 0;
    float last_theta = 0;

    float marker_x = 0;
    float marker_y = 0;
    float marker_theta = 0;

    float odom_x = 0;
    float odom_y = 0;
    float old_odom_x = 0;
    float old_odom_y = 0;
    float old_odom_theta = 0;

    float disp_odom_x = 0;
    float disp_odom_y = 0;
    float delta_disp_odom = 0;
    float disp_odom_theta = 0;

    float odom_theta = 0;


public:
    cv::Mat
    getPostion();
    Localization();
    void start(bool showCamera);
};