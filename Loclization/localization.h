#include "../RobotVision/aruco_scanner.h"

class Localization{    
    ArucoScanner arucoScanner = ArucoScanner();
    std::tuple<double, double> track;
    std::tuple<double, double> groundTruth;
    void tracker();
};