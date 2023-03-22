#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include "Map/map.h"
#include "CameraCalibration/camera_calibration.h"
#include "opencv_constants.h"
#include <functional>
#include "./aruco_scanner.h"
#include <unordered_map>
#include <algorithm>
#include <cmath>

ArucoScanner::ArucoScanner(){
    loadCameraCalibration(constants::cameraCalibrationPath, cameraMatrix, distortionCoefficients);
    double cx, cy;
    std::tie(cx, cy) = CameraCenterCalibration::loadCameraCenter();
    center = (cv::Mat_<double>(3, 1) << cx, cy, 0);
}

void ArucoScanner::showTransformationInfo(const cv::Mat &frame) {
    if (translationVectors.size().width > 0) {
        std::ostringstream distance;
        std::ostringstream rotation;
        cv::Point2d pp(cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2));
        cv::Scalar color(0, 0, 255);
        cv::circle(frame, pp, 5, color, 3);
        int id = getIdOfClosestMarker();
        distance << "ID: " << id << ", pose : " << "{ x : "<< xytheta[id][0] << ", y : " << xytheta[id][1] << ", theta : " << xytheta[id][2] << "}";
        putText(frame, distance.str(), cv::Point2i(20, frame.size().height - 20), cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0, 255, 0), 3);
    }
}

std::string gstreamer_pipeline () {    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(constants::capture_width) + ", height=(int)" +
           std::to_string(constants::capture_height) + ", framerate=(fraction)" + std::to_string(constants::framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(constants::flip_method) + " ! video/x-raw, width=(int)" + std::to_string(constants::display_width) + ", height=(int)" +
           std::to_string(constants::display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

void ArucoScanner::openCamera(std::function<void(cv::Mat&)> func, bool showCamera){
    cv::VideoCapture vid = constants::use_Gstream ? 
    cv::VideoCapture(gstreamer_pipeline(), cv::CAP_GSTREAMER) : cv::VideoCapture(constants::cam_id);

    cv::namedWindow("Cam");
    cv::Mat frame;
    if (!vid.isOpened()) showCamErrorMassage();
    while (vid.read(frame)) {
        func(frame);
        if(showCamera) imshow("Cam", frame);
        cv::waitKey(1);
    } 
}

void ArucoScanner::drawArucoMarker(cv::Mat& frame){
    cv::aruco::drawDetectedMarkers(frame, markerCorners);
    for (int i = 0; i < markerIds.size(); i++) {
        for (int j = 0; j < markerCorners[i].size(); j++)
        {
            cv::circle(frame, markerCorners[i][j], 10.5, cv::Scalar(0));
        }
        
        cv::Vec3d vec({rotationVectors(i)[0], rotationVectors(i)[1], rotationVectors(i)[2]});
        drawFrameAxes(frame, cameraMatrix, distortionCoefficients, vec, translationVectors(i),
        constants::arucoSquareDimension);
        showTransformationInfo(frame);
    }
}

cv::Mat rotationMatrixToEulerAngles(cv::Mat R){
    double sy = sqrt(pow(R.at<double>(0, 0), 2) + pow(R.at<double>(1, 0), 2));
    bool singular = sy < 1e-6;
    double x, y ,z;
    if(!singular){
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }else{
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }

    cv::Mat eulerVec = cv::Mat(3, 1, CV_64F);

    eulerVec.at<double>(0,0) = x * (180 / PI);
    eulerVec.at<double>(1,0) = y * (180 / PI);
    eulerVec.at<double>(2,0) = z * (180 / PI);
    return  eulerVec;


}

void ArucoScanner::addPose(double x, double y, double angle, int markerId){
    double alpha = 0.90;
    double threshold = 5;
    // if(angle < threshold || (angle + threshold) > 360) alpha = 0; // TODO: To fix average of 360-0 wrap around angle
    if(xytheta.count(markerId) > 0){
        xytheta[markerId][0] = xytheta[markerId][0]*alpha + x * (1-alpha);
        xytheta[markerId][1] = xytheta[markerId][1]*alpha + y * (1-alpha);
        xytheta[markerId][2] = xytheta[markerId][2]*alpha + angle * (1-alpha);
    }else{
        std::vector<double> pose = {x, y, angle};
        xytheta[markerId] = pose;
    }
}

void ArucoScanner::poseCorrection(){
    cv::Mat_<double> orientation;
    cv::Mat R_ct;
    for (int i = 0; i < markerIds.size(); i++){
        cv::Rodrigues(rotationVectors(i), R_ct);
        cv::Mat R =  R_ct.t();
        orientation = rotationMatrixToEulerAngles(R);
        double yaw = orientation(2, 0) * (PI/180);
        cv::Mat_<double> zRotation = (cv::Mat_<double>(3,3) << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0,0,1);
        cv::Mat_<double> fixedPose = -zRotation.t() * (translationVectors(i));
        addPose(fixedPose(0, 0), fixedPose(1, 0), orientation(2, 0), markerIds[i]);

    }
}

void ArucoScanner::estimateMarkersPose(const cv::Mat frame){
    cv::Mat ud_frame;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(constants::dictionaryName);
    cv::undistort(frame, ud_frame, cameraMatrix, distortionCoefficients);
    cv::aruco::detectMarkers(frame,dictionary ,markerCorners, markerIds);
    if(markerIds.size() > 0){
        vecs3d rvecs(cv::Size(3, markerIds.size())), tvecs(cv::Size(3, markerIds.size()));
        cv::aruco::estimatePoseSingleMarkers(markerCorners, constants::arucoSquareDimension, cameraMatrix, distortionCoefficients, 
        rvecs, tvecs);
        translationVectors = tvecs;
        rotationVectors = rvecs;
        poseCorrection();
    }
}

void ArucoScanner::monitorArucoMarkers(bool showCamera){
    std::function<void(cv::Mat &)> arucoMarkerDrawProccess = [=](cv::Mat& frame) {
        estimateMarkersPose(frame);
        drawArucoMarker(frame);
    };
    openCamera(arucoMarkerDrawProccess, showCamera);
}


cv::Point_<double> ArucoScanner::getPostion(int markerId){
    assert(xytheta.count(markerId) > 0);
    return cv::Point_<double>(xytheta[markerId][0], xytheta[markerId][1]);
}

std::tuple<double, double, double> ArucoScanner::getOriginalPosition(int markerId){
    assert(markerIds.size() > 0);
    for (int i = 0; i < markerIds.size(); i++){
        if(markerIds[i] == markerId){
            return std::make_tuple(translationVectors(i)[0], translationVectors(i)[1], translationVectors(i)[2]);
        }
    }
}
std::tuple<double, double, double> ArucoScanner::getOriginalOrientation(int markerId){
    assert(markerIds.size() > 0);
    for (int i = 0; i < markerIds.size(); i++){
        if(markerIds[i] == markerId){
            return std::make_tuple(rotationVectors(i)[0], rotationVectors(i)[1], rotationVectors(i)[3]);
        }
    }
}
double ArucoScanner::getOrientation(int markerId){
    assert(xytheta.size() > 0);
    return xytheta[markerId][2];
}

bool ArucoScanner::isArucoFound(){
    return markerIds.size() > 0;
}

std::vector<int> ArucoScanner::getDetectedMarkers(){
    return markerIds;
}

double euclideanDistance(double x, double y){
    return sqrt(x*x + y*y);
}

int ArucoScanner::getIdOfClosestMarker(){
    assert(markerIds.size() > 0);
    int idIndex = 0;
    std::vector<double> pose = xytheta[markerIds[idIndex]];
    double shortestD = euclideanDistance(pose[0], pose[1]);
    for (int i = 1; i < markerIds.size(); i++){
        pose = xytheta[markerIds[i]];
        double d = euclideanDistance(pose[0], pose[1]);
        if(d < shortestD){
            shortestD = d;
            idIndex = i;
        }
    }
    return markerIds[idIndex];
}

int ArucoScanner::getNumberOfAruco(){
    return markerIds.size();
}

void ArucoScanner::showCamErrorMassage() {
    std::cout << "Error : Could not open camera";
    exit(1);
}