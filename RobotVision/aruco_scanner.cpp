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

const double PI  =3.141592653589793238463;

void ArucoScanner::showCamErrorMassage() {
    std::cout << "Error : Could not open camera";
    exit(1);
}

template<typename T>
std::vector<T> findDiff(std::vector<T> x, std::vector<T> y) {        // no-ref, no-const
    std::vector<T> diff;
    std::sort(x.begin(), x.end());
    std::sort(y.begin(), y.end());
    std::set_difference(x.begin(), x.end(), y.begin(), y.end(), std::back_inserter(diff));
    return diff;
}

void ArucoScanner::showTransformationInfo(const cv::Mat &frame) {
    if (translationVectors.size().width > 0) {
        std::ostringstream distance;
        std::ostringstream rotation;
        distance << "ID: " << markerIds[0] << ", pose : " << "{ x : "<< xytheta[0] << ", y : " << xytheta[1] << ", theta : " << xytheta[2];
        putText(frame, distance.str(), cv::Point2i(10, frame.cols - 520), cv::FONT_HERSHEY_COMPLEX, .7,
                cv::Scalar(0, 255, 0));
        putText(frame, rotation.str(), cv::Point2i(10, frame.cols - 450), cv::FONT_HERSHEY_COMPLEX, .7,
                cv::Scalar(0, 255, 0));
    }
}

void ArucoScanner::openCamera(std::function<void(cv::Mat&)> func, bool showCamera=true){
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
        // if (cv::waitKey(1) > 0) break;
    } 
}


void ArucoScanner::drawArucoMarker(cv::Mat& frame){
    for (int i = 0; i < markerIds.size(); i++) {
        drawFrameAxes(frame, cameraMatrix, distortionCoefficients, rotationVectors(i), translationVectors(i),
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


void ArucoScanner::addPose(double x, double y, double angle){
    double alpha = 0.80;
    double threshold = 5;
    if(angle < threshold || (angle + threshold) > 360) alpha = 0; // To fix average of 360-0 wrap around angle
    xytheta[0] = xytheta[0]*alpha + x * (1-alpha);
    xytheta[1] = xytheta[1]*alpha + y * (1-alpha);
    xytheta[2] = xytheta[2]*alpha + angle * (1-alpha);
    printf("x : %.2f, y : %.2f, theta : %.2f\n",xytheta[0],xytheta[1],xytheta[2]);
}

void ArucoScanner::poseCorrection(vecs3d tvecs, vecs3d rvecs){
    cv::Mat_<double> orientation;
    cv::Mat R_ct;
    cv::Rodrigues(rvecs(0), R_ct);
    orientation = rotationMatrixToEulerAngles(flipPitch * R_ct.t());
    double theta = orientation(2, 0) * (PI / 180);
    cv::Mat_<double> zRotation = (cv::Mat_<double>(3,3) << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0,0,1);
    cv::Mat_<double> fixedPose = zRotation * (tvecs(0) - center);
    fixedPose = fixedPose + center;
    if(orientation(2, 0) < 0) {
        orientation(2, 0) = orientation(2, 0) + 360;
    }
    addPose(fixedPose(0, 0), fixedPose(1, 0), orientation(2, 0));
            
}
void ArucoScanner::estimateMarkersPose(const cv::Mat frame){
    arucoDetector.detectMarkers(frame, markerCorners, markerIds);
    if(markerIds.size() > 0){
            vecs3d rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, constants::arucoSquareDimension, cameraMatrix, distortionCoefficients, 
            rvecs, tvecs);
            translationVectors = tvecs;
            rotationVectors = rvecs;
            poseCorrection(tvecs, rvecs);
    }
}

void ArucoScanner::monitorArucoMarkers(){
    std::function<void(cv::Mat &)> arucoMarkerDrawProccess = [=](cv::Mat& frame) {
        estimateMarkersPose(frame);
        drawArucoMarker(frame);
    };
    openCamera(arucoMarkerDrawProccess);
}


std::tuple<double, double> ArucoScanner::getPostion(){
    assert(xytheta.size() > 0);
    return std::make_tuple(xytheta[0], xytheta[1]);
}

std::tuple<double, double> ArucoScanner::getOriginalPosition(){
    assert(translationVectors.size().width > 0);
    return std::make_tuple(translationVectors(0)[0], translationVectors(0)[1]);
}

double ArucoScanner::getOrientation(){
    assert(xytheta.size() > 0);
    return xytheta[2];
}

bool ArucoScanner::isArucoFound(){
    return xytheta.size() > 0;
}

ArucoScanner::ArucoScanner(){
    loadCameraCalibration(constants::cameraCalibrationPath, cameraMatrix, distortionCoefficients);
    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(constants::dictionaryName);
    arucoDetector = cv::aruco::ArucoDetector(markerDictionary);
    double cx, cy;
    std::tie(cx, cy) = CameraCenterCalibration::loadCameraCenter();
    center = (cv::Mat_<double>(3, 1) << cx, cy, 0);
}