#include "../RobotVision/aruco_scanner.h"
#include <opencv2/core/types.hpp>
class Localization{    
    ArucoScanner arucoScanner = ArucoScanner();
    cv::Point_<double> pose;
    double orientation;
    std::unordered_map<int, cv::Mat> arucosCoef;
    void loadArucosCoef();
    public:
        cv::Mat getPostion();
        Localization();
        void start(bool showCamera);
};