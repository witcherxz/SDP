#include "../RobotVision/aruco_scanner.h"
#include <opencv2/core/types.hpp>
class Localization{    
    ArucoScanner arucoScanner = ArucoScanner();
    cv::Point_<double> pose;
    double orientation;
    std::unordered_map<int, cv::Mat> arucosCoef;
    cv::Mat getPostion();
    void loadArucosCoef();
    public:
        Localization();
        void start(bool showCamera);
};