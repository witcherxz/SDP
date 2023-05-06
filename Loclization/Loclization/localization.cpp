#include "localization.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core/types.hpp>
#include "../RobotVision/opencv_constants.h"
#include <iostream>
#include <fstream>

void saveCameraCalibration(const std::string &filename, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs) {
    std::cout << filename << std::endl;
    cv::FileStorage fs(filename, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    if(!fs.isOpened()){
        std::cout << "Failed to open path : " << filename << std::endl;
        exit(1);
    }
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
}

Localization::Localization(){
    loadArucosCoef();
}

void Localization::start(bool showCamera){
    std::function<void(cv::Mat &)> locProccess = [=](cv::Mat& frame) {
        arucoScanner.estimateMarkersPose(frame);
        if(showCamera) arucoScanner.drawArucoMarker(frame);
        cv::Mat p = getPostion();
        std::cout << p.at<float>(0,0) << " " << p.at<float>(0, 1) << "\n";
    };
    arucoScanner.openCamera(locProccess, showCamera);
}

void Localization::loadArucosCoef(){
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

cv::Mat Localization::getPostion(){
    std::vector<int> ids = arucoScanner.getDetectedMarkers();
    float waight = 1.0 / ids.size();
    cv::Mat pose = (cv::Mat_<float>(1,2) << 0, 0);
    for(int id : ids)
    {
        cv::Point_<double> p = arucoScanner.getPostion(id);
        cv::Mat mat = (cv::Mat_<float>(3,1) << 1 , p.x, p.y);
        if(arucosCoef.count(id) > 0){
            cv::Mat fromGroundTruth = mat.t() * arucosCoef[id].t();
            pose = pose + (fromGroundTruth * waight);
        }else{
            pose = mat;
        }
    }
    return pose;
}

