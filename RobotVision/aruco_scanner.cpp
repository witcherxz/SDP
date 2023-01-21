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
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    // detectorParams->adaptiveThreshConstant = 4;
    detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(constants::dictionaryName);
    arucoDetector = cv::aruco::ArucoDetector(markerDictionary, detectorParams);
    double cx, cy;
    std::tie(cx, cy) = CameraCenterCalibration::loadCameraCenter();
    center = (cv::Mat_<double>(3, 1) << cx, cy, 0);
}


void ArucoScanner::showTransformationInfo(const cv::Mat &frame) {
    if (translationVectors.size().width > 0) {
        std::ostringstream distance;
        std::ostringstream rotation;
        cv::Point2d pp(cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2));
        cv::Scalar color(0, 0, 255); // red color
        cv::circle(frame, pp, 5, color, 3);
        int id = getIdOfClosestMarker();
        distance << "ID: " << id << ", pose : " << "{ x : "<< xytheta[id][0] << ", y : " << xytheta[id][1] << ", theta : " << xytheta[id][2] << "}";
        putText(frame, distance.str(), cv::Point2i(20, frame.size().height - 20), cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0, 255, 0), 3);
    }
}

void ArucoScanner::openCamera(std::function<void(cv::Mat&)> func, bool showCamera){
    cv::VideoCapture vid(1);
    cv::namedWindow("Cam");
    cv::Mat frame;
    cv::Mat resized;
    if (!vid.isOpened()) showCamErrorMassage();
    vid.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    vid.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    while (vid.read(frame)) {
        func(frame);
        cv::resize(frame, resized, cv::Size(), 0.5, 0.5);
        if(showCamera) imshow("Cam", resized);
        cv::waitKey(1);
    } 
}

void ArucoScanner::drawArucoMarker(cv::Mat& frame){
    // cv::aruco::drawDetectedMarkers(frame, markerCorners);
    for (int i = 0; i < markerIds.size(); i++) {
        cv::Vec3d vec({rotationVectors(i)[0], rotationVectors(i)[1], rotationVectors(i)[2]});
        drawFrameAxes(frame, cameraMatrix, distortionCoefficients, vec, translationVectors(i),
        constants::arucoSquareDimension);
        showTransformationInfo(frame);
    }
}

// bool isRotationMatrix(cv::Mat mat){
//     cv::Mat tMat;
//     tMat = mat.t();
//     cv::Mat identity
// }
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
    double alpha = 0.9;
    double threshold = 5;
    if(angle < threshold || (angle + threshold) > 360) alpha = 0; // To fix average of 360-0 wrap around angle
    if(xytheta.count(markerId) > 0){
        xytheta[markerId][0] = xytheta[markerId][0]*alpha + x * (1-alpha);
        xytheta[markerId][1] = xytheta[markerId][1]*alpha + y * (1-alpha);
        xytheta[markerId][2] = xytheta[markerId][2]*alpha + angle * (1-alpha);
    }else{
        std::vector<double> pose = {x, y, angle};
        xytheta[markerId] = pose;
    }
    // printf("id: %d, x : %.2f, y : %.2f, theta : %.2f\n",markerId,xytheta[markerId][0],xytheta[markerId][1],xytheta[markerId][2]);
}

void ArucoScanner::poseCorrection(){
    cv::Mat_<double> orientation;
    cv::Mat R_ct;
    for (int i = 0; i < markerIds.size(); i++){
        cv::Rodrigues(rotationVectors(i), R_ct);
        cv::Mat R =  R_ct.t();
        orientation = rotationMatrixToEulerAngles(R);
        if(orientation(2, 0) < 0) {
            orientation(2, 0) = orientation(2, 0) + 360;
        }
        double yaw = -orientation(2, 0) * (PI / 180);
        double pitch = orientation(0, 0);
        double roll = orientation(1, 0);
        printf("pitch: %.2f, roll : %.2f, yaw : %.2f\r", pitch, roll, yaw * (180/PI));
        cv::Mat_<double> zRotation = (cv::Mat_<double>(3,3) << cos(yaw), -sin(yaw ), 0, sin(yaw), cos(yaw  ), 0, 0,0,1);
        cv::Mat_<double> fixedPose = zRotation.t() * (translationVectors(i) - center);
        fixedPose = fixedPose + center;
        if(orientation(2, 0) < 0) {
            orientation(2, 0) = orientation(2, 0) + 360;
        }
        addPose(fixedPose(0, 0), fixedPose(1, 0), orientation(2, 0), markerIds[i]); 
    }
}
void ArucoScanner::estimateMarkersPose(const cv::Mat frame){
    
    // Set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    float markerLength = constants::arucoSquareDimension;
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);
    arucoDetector.detectMarkers(frame, markerCorners, markerIds);
    
    if(markerIds.size() > 0){
        
        vecs3d rvecs(cv::Size(3, markerIds.size())), tvecs(cv::Size(3, markerIds.size()));
        cv::aruco::estimatePoseSingleMarkers(markerCorners, constants::arucoSquareDimension, cameraMatrix, distortionCoefficients, 
        rvecs, tvecs);
        // for (size_t i = 0; i < markerIds.size(); i++)
        // {
        //     solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distortionCoefficients, rvecs.at<cv::Vec3d>(i), tvecs.at<cv::Vec3d>(i));
        //     // printf("pose : {x : %0.2f, y : %0.2f, z : %0.2f, thetaX : %0.2f, thetaY : %0.2f, thetaZ : %0.2f,}\n", tvecs.at<cv::Vec3d>(i)[0], tvecs.at<cv::Vec3d>(i)[1], tvecs.at<cv::Vec3d>(i)[2], rvecs.at<cv::Vec3d>(i)[0], rvecs.at<cv::Vec3d>(i)[1], rvecs.at<cv::Vec3d>(i)[2]);
        // }
        
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