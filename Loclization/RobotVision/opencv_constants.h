#ifndef SPD_ROBOTVISION_OPENCVCONSTANTS_H
#define SPD_ROBOTVISION_OPENCVCONSTANTS_H
namespace constants {
    extern const float calibrationSquareDimension; // in meter
    extern const float arucoSquareDimension; // in meter
    extern const cv::Size chessboardDimensions;
    extern const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryName;
    extern const std::string cameraCalibrationPath;
    extern const std::string systemCalibrationPath;
    extern const std::string calibrationImagesFolder;
    extern const std::string cameraCenterCalibrationPath;
    extern const bool use_Gstream;
    extern const int cam_id;
    extern const int capture_width;
    extern const int capture_height;
    extern const int display_width;
    extern const int display_height;
    extern const int framerate;
    extern const int flip_method;
} // namespace constants

#endif