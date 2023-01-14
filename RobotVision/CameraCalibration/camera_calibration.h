#include <opencv2/core/mat.hpp>
#include <iostream>
#include "../aruco_scanner.h"
#include <opencv2/highgui.hpp>

#ifndef ROBOTVISION_CAMERACALIBRATION_H
#define ROBOTVISION_CAMERACALIBRATION_H
//x, y, z, theta x, theta y, theta z
typedef std::tuple<double, double, double, double, double, double> c_record_t;
typedef std::tuple<double, double, double> r_record_t;

void startPosCollection();
bool loadCameraCalibration(std::string path, cv::Mat &camMatrix, cv::Mat &distCoeffs);
void saveSystemCalibration(const std::string &filename, const std::vector<r_record_t> &real, const std::vector<c_record_t> &camera);

void startCameraCalibration();

class CameraCenterCalibration {
    double storedAngle;
    double angle;
    bool isOpposite = false;
    double cx;
    double cy;
    std::vector<std::tuple<double, double>> points;
    std::vector<std::tuple<double, double>> oppositePoints;
    ArucoScanner arucoScanner = ArucoScanner();
    void showAngleInfo(cv::Mat frame);
    void intrinsicCalibration();
    void centerCalibrationProccess(cv::Mat& frame);
    void calculateCenter();
    void saveCalibration();
    void addPoint();
    public:
        void centerCalibration();
        static std::tuple<double, double> loadCameraCenter();
};
#endif // ROBOTVISION_CAMERACALIBRATION_H
