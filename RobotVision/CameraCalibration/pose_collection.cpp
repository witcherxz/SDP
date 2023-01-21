#include "pose_collection.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
void PoseCollection::startPoseCollection(){
    std::function<void(cv::Mat&)> collectionProcess = [=](cv::Mat& frame) {
        as.estimateMarkersPose(frame);
        char input = cv::waitKey(1);
        switch (input)
        {
        case ' ':
            pushCameraRecords();
            pushRealRecord();
            break;
        case 's':
            saveSystemCalibration();
            break;
        }
    };
        as.openCamera(collectionProcess);
}

void PoseCollection::saveSystemCalibration(){
    cv::FileStorage fs(filename, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    if(!fs.isOpened()){
        std::cout << "Could not open file :" << fs.NAME_EXPECTED << std::endl;
        exit(1);
    }
    fs.startWriteStruct("real", cv::FileNode::SEQ);
    // fs << real;
    for(int i = 0; i < realRecord.size(); i++)
    {
        double x, y, theta;
        std::tie(x, y, theta) = realRecord[i];
        fs.startWriteStruct("", cv::FileNode::MAP);
        fs << "ids" << detectedMarkersRecord[i];
        fs << "x" << x;
        fs << "y" << y;
        fs << "theta" << theta;
        fs.endWriteStruct();
    }
    fs.endWriteStruct();
    fs.startWriteStruct("camera", cv::FileNode::MAP);
    // fs << camera;
    for(auto pair : cameraRecord)
    {

        int id = pair.first;
        fs.startWriteStruct("_"+std::to_string(id), cv::FileNode::SEQ);
        for(auto pose : pair.second)
        {
            fs.startWriteStruct("", cv::FileNode::MAP);
            fs << "x" << pose[0];
            fs << "y" << pose[1];
            fs << "theta" << pose[2];
            fs << "o_x" << pose[3];
            fs << "o_y" << pose[4];
            fs << "o_z" << pose[5];
            fs << "o_theta_x" << pose[6];
            fs << "o_theta_y" << pose[7];
            fs << "o_theta_z" << pose[8];
            fs.endWriteStruct();
        }
        fs.endWriteStruct();
    }
    fs.endWriteStruct();
    std::cout << "Points saved at : " << constants::systemCalibrationPath << std::endl;
}

void PoseCollection::pushCameraRecords(){
    std::vector<int> detectedMarkers = as.getDetectedMarkers();
        for (int i = 0; i < detectedMarkers.size(); i++)
        {
            int markerId = detectedMarkers[i];
            cv::Point_<double> point = as.getPostion(markerId);
            std::tuple<double, double, double> oPoint = as.getOriginalPosition(markerId);
            std::tuple<double, double, double> oOrientation = as.getOriginalPosition(markerId);
            double angle = as.getOrientation(markerId);
            c_pose_t record = {point.x, point.y, angle, std::get<0>(oPoint),std::get<1>(oPoint), std::get<2>(oPoint), std::get<0>(oOrientation),std::get<1>(oOrientation), std::get<2>(oOrientation) };//std::make_tuple(point.x, point.y, angle);
            cameraRecord[markerId].push_back(record);
        }
        detectedMarkersRecord.push_back(detectedMarkers);
}

void PoseCollection::pushRealRecord(){
    std::string buff;
    double x, y, theta;
    // std::cout << "Enter x :" << std::endl;
    // std::cin >> buff;
    x = 0;//std::stod(buff);
    // std::cout << "Enter y :" << std::endl;
    // std::cin >> buff;
    y = 0;//std::stod(buff);
    std::cout << "Enter theta:" << std::endl;
    std::cin >> buff;
    theta = std::stod(buff);
    pose_t record = std::make_tuple(x, y, theta);
    realRecord.push_back(record);
}
