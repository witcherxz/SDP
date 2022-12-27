#include <opencv2/core/mat.hpp>
#include <iostream>

#ifndef ROBOTVISION_CAMERACALIBRATION_H
#define ROBOTVISION_CAMERACALIBRATION_H
typedef std::tuple<double, double, double, double> record_t;

void startPosCollection();
bool loadCameraCalibration(std::string path, cv::Mat &camMatrix, cv::Mat &distCoeffs);
void saveSystemCalibration(const std::string &filename, const std::vector<record_t> &real, const std::vector<record_t> &camera);

void startCameraCalibration();

#endif // ROBOTVISION_CAMERACALIBRATION_H
