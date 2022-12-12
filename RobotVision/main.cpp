
#include <opencv2/core/mat.hpp>
#include "cameraCalibration/camera_calibration.h"
#include "aruco_scanner.h"

int main(int argc, char **argv) {
    cv::Mat cameraMatrix, distortionCoefficients;
//    startCameraCalibration();
     loadCameraCalibration("C:\\Users\\Legend\\Desktop\\SDP\\SDP\\RobotVision\\cameraCalibration", cameraMatrix, distortionCoefficients);
     startCamMonitoring(cameraMatrix, distortionCoefficients);
    return 0;
}
