#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>

#include <stdlib.h>

#include <filesystem>

namespace constants {
    std::string homedir = std::getenv("HOME");
    extern const float calibrationSquareDimension = 0.025f; // in meter
    extern const float arucoSquareDimension = 0.14f; // in meter
    extern const cv::Size chessboardDimensions = cv::Size(6, 9);
    extern const int dictionaryName = cv::aruco::DICT_4X4_50;
    extern const std::string cameraCalibrationPath = homedir + "\\Documents\\cameraCalibration";
    extern const std::string systemCalibrationPath = homedir + "\\Documents\\systemCalibration.json";
    
} // namespace constants
