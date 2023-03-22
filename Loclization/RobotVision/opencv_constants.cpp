#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>

#include <stdlib.h>

// #include <filesystem>

namespace constants {
    std::string homedir = std::getenv("HOME");
    extern const bool use_Gstream = true;
    extern const int cam_id = 0;
    extern const float calibrationSquareDimension = 0.025f; // in meter
    extern const float arucoSquareDimension = 20; // in cm
    extern const cv::Size chessboardDimensions = cv::Size(6, 9);
    extern const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryName = cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50;
    extern const std::string cameraCalibrationPath = homedir + "/t05/Calibration_Folder/cameraCalibration";
    extern const std::string systemCalibrationPath = homedir + "/t05/Calibration_Folder/systemCalibration.json";
    extern const std::string cameraCenterCalibrationPath = homedir + "/t05/Calibration_Folder/cameraCenterCalibration";
    extern const std::string calibrationImagesFolder = homedir + "/t05/Calibration_Folder/calibrationImagesFolder";

    // GStream parametars
    extern const int capture_width = 1280 ;
    extern const int capture_height = 720 ;
    extern const int display_width = 1280 ;
    extern const int display_height = 720 ;
    extern const int framerate = 30 ;
    extern const int flip_method = 0 ;
} // namespace constants
