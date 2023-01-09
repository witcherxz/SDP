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
        distance << "ID: " << markerIds[0] << ", distance : " << rotationVectors.at<cv::Vec3d>(0);
        rotation << "rotation : " << rotationVectors.at<cv::Vec3d>(0
        );
        putText(frame, distance.str(), cv::Point2i(10, frame.cols - 220), cv::FONT_HERSHEY_COMPLEX, .7,
                cv::Scalar(0, 255, 0));
        putText(frame, rotation.str(), cv::Point2i(10, frame.cols - 180), cv::FONT_HERSHEY_COMPLEX, .7,
                cv::Scalar(0, 255, 0));
    }
}
void ArucoScanner::openCamera(std::function<void(cv::Mat&)> func){
    cv::VideoCapture vid(0);
    cv::namedWindow("Cam");
    cv::Mat frame;
     // Read the image file
    // cv::imshow("image", image);
    // cv::Mat image = cv::imread("C:\\Users\\Legend\\Desktop\\mat.jpg");
    std::cout << "camera created" << std::endl;
    if (!vid.isOpened()) showCamErrorMassage();
    vid.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    vid.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    while (vid.read(frame)) {
        func(frame);
        // func(image);
        
    imshow("Cam", frame);
        if (cv::waitKey(10) > 0) break;
    } 
}
void ArucoScanner::drawArucoMarker(cv::Mat& frame){
    for (int i = 0; i < markerIds.size(); i++) {
        drawFrameAxes(frame, cameraMatrix, distortionCoefficients, rotationVectors.at<cv::Vec3d>(i), translationVectors.at<cv::Vec3d>(i),
        constants::arucoSquareDimension);
        showTransformationInfo(frame);
    }

}


void ArucoScanner::estimateMarkersPose(const cv::Mat frame){
    // Set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);

    const float markerLength = constants::arucoSquareDimension;
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);
    arucoDetector.detectMarkers(frame, markerCorners, markerIds);
    // std::vector<int> disappearMarkers= findDiff<int>(markerIds, oldDetectedMarker);
    // for(auto marker : disappearMarkers){
    //     rotationVectors.erase(marker);
    //     translationVectors.erase(marker);
    // }
    int numberOfDetectedMarker = markerIds.size();
    if(numberOfDetectedMarker > 0){
        // // std::vector<std::vector<cv::Point2f>> corners;
            cv::Mat rvecs, tvecs;
            rvecs.create(numberOfDetectedMarker, 1, CV_64FC3);
            tvecs.create(numberOfDetectedMarker, 1, CV_64FC3);
            // cv::aruco::estimatePoseSingleMarkers(markerCorners, constants::arucoSquareDimension, cameraMatrix, distortionCoefficients, rvecs, tvecs);
            rotationVectors = rvecs;
            translationVectors = tvecs;
            std::cout << tvecs << std::endl;
        
        for (int i = 0; i < markerIds.size(); i++) {
            int id = markerIds[i];
            
            // bool markerRecordExist = translationVectors.count(id) > 0;
            cv::solvePnP(objPoints, markerCorners[i], cameraMatrix, distortionCoefficients, rvecs.at<cv::Vec3d>(i), tvecs.at<cv::Vec3d>(i), false);
            
            // for (int i = 0; i < rvecs.size().width; i++){
            //     std::cout << rvecs.at<cv::Vec3d>(i)[0] << std::endl;
            // }
            
            // cv::OutputArray
            // std::cout << "ADD rot called with " << rvecs.clone() << " as an input" << std::endl;
            // rotVecAvg.add(rvecs.clone());
            // std::cout << "ADD trans called with " << tvecs.clone() << " as an input" << std::endl;
            // tranVecAvg.add(tvecs.clone());
            // rotationVectors = rotVecAvg.getAverage();
            rotationVectors = rvecs;
            translationVectors = tvecs;
            // std::cout << rotationVectors << std::endl;

            // if(markerRecordExist){
            //     // cv::solvePnPRefineLM(objPoints, markerCorners[i], cameraMatrix, distortionCoefficients, rotationVectors[i], translationVectors[i]);
            //     cv::solvePnPRefineVVS(objPoints, markerCorners[i], cameraMatrix, distortionCoefficients, rotationVectors[i], translationVectors[i]);
            //     // cv::solvePnPRansac(objPoints, markerCorners[i], cameraMatrix, distortionCoefficients, rotationVectors[i], translationVectors[i], true, 100);
            //     // cv::solvePnP(objPoints, markerCorners[i], cameraMatrix, distortionCoefficients, rotationVectors[i], translationVectors[i], true);
            // }else{
            //     cv::solvePnP(objPoints, markerCorners[i], cameraMatrix, distortionCoefficients, rotationVectors[i], translationVectors[i]);
            // }
        }
        idsVectorSize = markerIds.size();
    }
}

void ArucoScanner::monitorArucoMarkers(){
    openCamera(arucoMarkerDrawProccess);
}

ArucoScanner::vecs3d ArucoScanner::getTranslationVectors(){
    return translationVectors;
}
ArucoScanner::vecs3d ArucoScanner::getRotationVectors(){
    return rotationVectors;
}

ArucoScanner::ArucoScanner(){
    loadCameraCalibration(constants::cameraCalibrationPath, cameraMatrix, distortionCoefficients);
    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(constants::dictionaryName);
    // const cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters();
    arucoDetector = cv::aruco::ArucoDetector(markerDictionary);

}

AverageFilter::AverageFilter(){
    this->bufferSize = 20;
    // this->buffer.resize(bufferSize);
    // this->valBuffer.resize(bufferSize);
}

void AverageFilter::add(cv::Mat pose){
    buffer.insert(buffer.begin(), pose);
    valBuffer.insert(valBuffer.begin(), pose.at<cv::Vec3d>(0)[0]);
    std::cout << "---------------------------------------------" << std::endl;
    for (int i = 0; i < buffer.size(); i++)
    {
        std::cout << buffer[i] << std::endl;
    }
    std::cout << "---------------------------------------------" << std::endl;
    
    if (valBuffer.size() > bufferSize){
        buffer.pop_back();
        valBuffer.pop_back();
    }
    std::cout << "SIZE :" << buffer.size() << std::endl;
    average = cv::Mat::zeros(pose.size(), CV_64FC3);
    int max = mostFrequentElement(valBuffer);
    if(buffer[max].at<cv::Vec3d>(0)[0] == 0){
        std::cout << "max : " << max << std::endl;
        std::cout << "valBuf(max) : " << valBuffer[max] << std::endl;
        // std::cout << "buf(19) : " << buffer[19] << std::endl;
        std::cout << "buf(max) : " << buffer[max] << std::endl;
            std::cout << "---------------------------------------------" << std::endl;
            // for (int i = 0; i < buffer.size(); i++)
            // {
            //     std::cout << buffer[i] << std::endl;
            // }
    }
    average = buffer[max];
}
cv::Mat AverageFilter::getAverage(){
    return average;
}

int AverageFilter::mostFrequentElement(const std::vector<double> vec){
    int max = 0;
    int maxCount = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        double value = vec[i];
        int counter = 0;
        for (int j = 0; j < vec.size(); j++)
        {
            if(value == vec[j]){
                counter++;
            }
        }
        if(counter > maxCount){
            max = i;
        }
    }
    return max;
    
}