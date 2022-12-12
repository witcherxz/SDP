#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include "map.cpp"
#include "opencv_constants.h"

void showCamErrorMassage()
{
    std::cout << "Error : Could not open camera";
    exit(1);
}

void monitorCamera(cv::Mat frame, cv::VideoCapture vid, const std::function<void()> &func)
{
    while (vid.read(frame))
    {
        func();
    }
}

void showTranslationInfo(const cv::Mat &frame, const std::vector<cv::Vec3d> &translationVectors)
{
    if (translationVectors.size() > 0)
    {
        std::ostringstream distance;
        distance << "distance : " << translationVectors[0];
        putText(frame, distance.str(), cv::Point2i(10, frame.cols - 200), cv::FONT_HERSHEY_COMPLEX, .7,
                cv::Scalar(0, 255, 0));
    }
}

void updatePositionOnMap(int arucoXPos, int arucoYPos, const std::vector<cv::Vec3d> &translationVectors, BasicMap &map)
{
    if (!translationVectors.empty())
    {
        map.changePointPosition(arucoXPos - translationVectors[0][0], arucoYPos + translationVectors[0][1]);
    }
}

void drawMarkersOnFrame(const cv::Mat &frame, const cv::Mat &distanceCoefficients, const cv::Mat &cameraMatrix,
                         const std::vector<int> &markerIds, const std::vector<cv::Vec3d> &rotationVectors,
                         const std::vector<cv::Vec3d> &translationVectors)
{
    for (int i = 0; i < markerIds.size(); i++)
    {
        // ostringstream x;
        drawFrameAxes(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i],
                      constants::arucoSquareDimension);
    }
}

void monitorArucoMarkers(cv::Mat frame, const cv::Mat &distortionCoefficients, const cv::Mat &cameraMatrix)
{
    int arucoXPos = 4;
    int arucoYPos = 6;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::vector<cv::Vec3d> rotationVectors, translationVectors;
    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(constants::dictionaryName);
    cv::VideoCapture vid(0);
    cv::namedWindow("Cam");
    if (!vid.isOpened())
        showCamErrorMassage();
    BasicMap map(30, 10, 0.4);

    map.drawMap();
    while (vid.read(frame))
    {
        cv::aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
        cv::aruco::estimatePoseSingleMarkers(markerCorners, constants::arucoSquareDimension, cameraMatrix, distortionCoefficients, rotationVectors, translationVectors);
        drawMarkersOnFrame(frame, distortionCoefficients, cameraMatrix, markerIds, rotationVectors, translationVectors);
        updatePositionOnMap(arucoXPos, arucoYPos, translationVectors, map);
        showTranslationInfo(frame, translationVectors);
        imshow("Cam", frame);
        if (cv::waitKey(10) > 0)
            break;
    }
}

void startCamMonitoring(const cv::Mat &cameraMatrix, const cv::Mat &distanceCoefficients)
{
    cv::Mat frame;
    monitorArucoMarkers(frame, distanceCoefficients, cameraMatrix);
}
