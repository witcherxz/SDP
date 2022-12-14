#include <opencv2/core/mat.hpp>
#include <iostream>

#ifndef ROBOTVISION_CAMERACALIBRATION_H
#define ROBOTVISION_CAMERACALIBRATION_H

bool loadCameraCalibration(std::string path, cv::Mat &camMatrix, cv::Mat &distCoeffs);

void startCameraCalibration();

#endif // ROBOTVISION_CAMERACALIBRATION_H
