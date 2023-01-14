
#include "iostream"
#include <opencv2/core/mat.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "RobotVision/cameraCalibration/camera_calibration.h"
#include "RobotVision/aruco_scanner.h"
#include "RobotVision/Map/map.h"
#include "RobotVision/Map/map_creator.h"
#include "RobotVision/Map/map_viewer.h"
#include "RobotVision/Map/a_star.cpp"
#include "RobotVision/aruco_gnerator.h"
#include "RobotVision/opencv_constants.h"

int main(int argc, char **argv) {
  // startPosCollection();
  //  startCameraCalibration();
  // ArucoScanner as = ArucoScanner();
  // as.monitorArucoMarkers();
  CameraCenterCalibration cc = CameraCenterCalibration();
  cc.centerCalibration();
  return 0;
}