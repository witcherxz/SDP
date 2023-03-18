#include "../RobotVision/aruco_scanner.h"
#include <opencv2/core/types.hpp>
class Localization{    
    ArucoScanner arucoScanner = ArucoScanner();
    cv::Point_<double> rootOffset;
    cv::Point_<double> track;
    cv::Point_<double> groundTruth;
    double orientation;
    bool isInit = false;
    int trackedMarker;
    cv::Point_<double> getPostion();
    void setInitialPose();
    void tracker();
    void updateTrack();
    public:
        Localization();
};