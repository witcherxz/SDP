#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>

namespace constants
{
    extern const float calibrationSquareDimension = 0.025f; // in meter
    extern const float arucoSquareDimension = 0.14f; // in meter
    extern const cv::Size chessboardDimensions = cv::Size(6, 9);
    extern const int dictionaryName = cv::aruco::DICT_6X6_1000;
} // namespace constants
