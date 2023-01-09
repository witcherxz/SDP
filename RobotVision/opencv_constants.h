#ifndef SPD_ROBOTVISION_OPENCVCONSTANTS_H
#define SPD_ROBOTVISION_OPENCVCONSTANTS_H
namespace constants {
    extern const float calibrationSquareDimension; // in meter
    extern const float arucoSquareDimension; // in meter
    extern const cv::Size chessboardDimensions;
    extern const int dictionaryName;
    extern const std::string cameraCalibrationPath;
    extern const std::string systemCalibrationPath;
    extern const std::string calibrationImagesFolder;
} // namespace constants

#endif