#include <opencv2/core/mat.hpp>
#include <iostream>

#ifndef ROBOTVISION_CAMERACALIBRATION_H
#define ROBOTVISION_CAMERACALIBRATION_H
//x, y, z, theta x, theta y, theta z
typedef std::tuple<double, double, double, double, double, double> c_record_t;
typedef std::tuple<double, double, double> r_record_t;

void startPosCollection();
bool loadCameraCalibration(std::string path, cv::Mat &camMatrix, cv::Mat &distCoeffs);
void saveSystemCalibration(const std::string &filename, const std::vector<r_record_t> &real, const std::vector<c_record_t> &camera);

void startCameraCalibration();

#endif // ROBOTVISION_CAMERACALIBRATION_H
