#include "../RobotVision/aruco_scanner.h"
#include <opencv2/core/types.hpp>
class Localization{    
    ArucoScanner arucoScanner = ArucoScanner();
    cv::Point_<double> rootOffset;
    cv::Point_<double> track;
    cv::Point_<double> groundTruth;
    void setInitialPose();
    void tracker();
    void updateTrack();
    bool isInit = false;
    int trackedMarker;
    public:
        Localization();
};