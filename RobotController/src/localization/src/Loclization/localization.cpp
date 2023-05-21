#include "localization.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core/types.hpp>
#include "../RobotVision/opencv_constants.h"
#include <iostream>
#include <fstream>

#include <chrono>
#include <thread>


void Localization::odometerUpdateing(const tf2_msgs::TFMessage::ConstPtr &msg)
{
    double roll;
    double pitch;
    double yaw;

    old_odom_x = (firstOdomRead) ? msg->transforms[0].transform.translation.x : odom_x;
    old_odom_y = (firstOdomRead) ? msg->transforms[0].transform.translation.y : odom_y;
    odom_x = msg->transforms[0].transform.translation.x;
    odom_y = msg->transforms[0].transform.translation.y;
    tf2::Quaternion myQuaternion;
    myQuaternion.setX(msg->transforms[0].transform.rotation.x);
    myQuaternion.setY(msg->transforms[0].transform.rotation.y);
    myQuaternion.setZ(msg->transforms[0].transform.rotation.z);
    myQuaternion.setW(msg->transforms[0].transform.rotation.w);
    tf2::Matrix3x3 mat(myQuaternion);
    mat.getRPY(roll, pitch, yaw);
    old_odom_theta = (firstOdomRead) ? yaw : odom_theta;
    odom_theta = yaw;
    firstOdomRead = false;
    float delta_odom_x = abs(odom_x - old_odom_x);
    float delta_odom_y = abs(odom_y - old_odom_y);
    delta_disp_odom = sqrt(delta_odom_x*delta_odom_x + delta_odom_y*delta_odom_y);

    if(odom_theta < 0 ) odom_theta += 2*M_PI;
    if(old_odom_theta < 0 ) old_odom_theta += 2*M_PI;
    
    float delta_odom_theta = odom_theta - old_odom_theta;
    if(delta_odom_theta > M_PI){
        delta_odom_theta -= 2*M_PI;
    }else if(delta_odom_theta < -M_PI){
        delta_odom_theta += 2*M_PI;
    }

    disp_odom_theta += delta_odom_theta;
}

Localization::Localization()
{
    char **argv;
    int argc = 0;
    ros::init(argc, argv, "localization");
    ros::NodeHandle n;
    this->publisher = n.advertise<geometry_msgs::Pose2D>("Location", 10);
    this->odometerSubscriber = n.subscribe("/tf", 1, &Localization::odometerUpdateing, this);
    loadArucosCoef();
}
void Localization::publish(cv::Mat pose)
{
    geometry_msgs::Pose2D loc_msg;
    // geometry_msgs::Pose2D *loc_msg_ptr = &loc_msg;
    loc_msg.x = pose.at<float>(0, 0);
    loc_msg.y = pose.at<float>(0, 1);
    loc_msg.theta = pose.at<float>(0, 2);
    publisher.publish(loc_msg);
}
void Localization::start(bool showCamera)
{
    
    std::function<void(cv::Mat &)> locProccess = [=](cv::Mat &frame)
    {
        arucoScanner.estimateMarkersPose(frame);
        if (showCamera)
            arucoScanner.drawArucoMarker(frame);
        cv::Mat p = getPostion();
        publish(p);
        ros::spinOnce();
        std::chrono::milliseconds duration(100);
        std::this_thread::sleep_for(duration);
        // this->rate.sleep();
    };
    arucoScanner.openCamera(locProccess, showCamera);
}

void Localization::loadArucosCoef()
{
    cv::FileStorage fs("/home/ceies/t05/Calibration_Folder/arucos_coef.json", cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open file" << std::endl;
        exit(1);
    }

    cv::FileNode rootNode = fs.root();
    for (cv::FileNodeIterator it = rootNode.begin(); it != rootNode.end(); ++it)
    {
        cv::Mat matrix;
        (*it) >> matrix;
        int id = std::stoi((*it).name());
        arucosCoef[id] = matrix;
    }
    fs.release();
}
cv::Mat Localization::getPostionFromMarker(){
    std::vector<int> ids = arucoScanner.getDetectedMarkers();
    cv::Mat xy = (cv::Mat_<float>(1, 2) << 0, 0);
    float angle = 0;
    std::vector<int> validMarkers = {};
    for(int id : ids){
        if (arucosCoef.count(id) > 0)
            validMarkers.push_back(id);
    }
    float waight = 1.0 / validMarkers.size();
    for (int id : validMarkers)
    {
        cv::Point_<double> p = arucoScanner.getPostion(id);
        angle = arucoScanner.getOrientation(id) * (M_PI / 180);
        cv::Mat mat = (cv::Mat_<float>(3, 1) << 1, p.x, p.y);
        cv::Mat fromGroundTruth = mat.t() * arucosCoef[id].t();
        xy = xy + (fromGroundTruth * waight);
    }
    cv::Mat pose = (cv::Mat_<float>(1, 3) << xy.at<float>(0, 0), xy.at<float>(0, 1), angle);
    return pose;
}
cv::Mat Localization::getPostion()
{
    float center = 0.1 / 2;
    float alpha = 0.5;
    float angle_alpha = 1;
    float threshold = 5;
    cv::Mat xytheta = (cv::Mat_<float>(1, 3) << 0, 0, 0);
    cv::Mat poseFromMarker = getPostionFromMarker();
    
    disp_odom_x += (delta_disp_odom * cos(theta));
    disp_odom_y += (delta_disp_odom * sin(theta));
    float filt_odom_x =  disp_odom_x + first_x;
    float filt_odom_y =  disp_odom_y + first_y; 
    float filt_odom_theta =  fmod(disp_odom_theta + first_theta, 2*M_PI); 
    if(filt_odom_theta < 0 ) filt_odom_theta += 2*M_PI;
    float tfot = fmod(filt_odom_theta + angleOffset, 2 * M_PI);
    std::cout << "  filt_odom_theta : " << filt_odom_theta * (180/M_PI)<< std::endl;
    std::cout << "      disp_odom_theta : " << disp_odom_theta * (180 / M_PI) << std::endl;
    std::cout << "      first_theta : " << first_theta * (180 / M_PI) << std::endl;


    if(poseFromMarker.at<float>(0, 0) != 0 && poseFromMarker.at<float>(0, 1) != 0){
        marker_x = poseFromMarker.at<float>(0, 0);
        marker_y = poseFromMarker.at<float>(0, 1);
        marker_theta = 2*M_PI - poseFromMarker.at<float>(0, 2);// Odom [<-(+) (-)->] Aruco Odom [<-(-) (+)->] change aruco to odom rule
        marker_theta -= M_PI / 2;
        if(marker_theta < 0) marker_theta += 2*M_PI;
        x = (marker_x + center * cos(marker_theta)) * alpha + filt_odom_x * (1 - alpha);
        y = (marker_y + center * sin(marker_theta)) * alpha + filt_odom_y * (1 - alpha);
        // if(theta < threshold || (theta + threshold) > 360) angle_alpha = 0;
        // math.atan2((math.sin(a) + math.sin(b)), (math.cos(a) + math.cos(b)))
        // theta = atan2(sin(marker_theta) + sin(filt_odom_theta), cos(marker_theta) + cos(filt_odom_theta));
        theta = marker_theta * angle_alpha + filt_odom_theta * (1 - angle_alpha);
        x += center * cos(theta);
        y += center * sin(theta);
        std::cout << "  marker_theta : " << marker_theta * (180/M_PI)<< std::endl;
        if(firstTimeNewArucoDetected++ == 1){
            // firstTimeNewArucoDetected = true;
            first_x = marker_x;
            first_y = marker_y;
            first_theta = marker_theta;
            disp_odom_theta = 0;
            disp_odom_x = 0;
            disp_odom_y = 0;
        }
    }else{
        x = filt_odom_x;
        y = filt_odom_y;
        theta = filt_odom_theta;
        firstTimeNewArucoDetected = 0;
    }
    cv::Mat pose = (cv::Mat_<float>(1, 3) << x, y, theta);
    std::cout << x << " " << y << " " << theta * (180 / M_PI) << "\n" ;
    std::cout << "======================================\n";
    return pose;
}
