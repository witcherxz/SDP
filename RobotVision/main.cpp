
#include <opencv2/core/mat.hpp>
#include "cameraCalibration/camera_calibration.h"
#include "aruco_scanner.h"
#include "Map/map.h"
#include "Map/map_creator.h"
#include "Map/map_viewer.h"
#include "Map/a_star.cpp"
#include "iostream"
#include "aruco_gnerator.h"
#include "opencv_constants.h"

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <tuple>
void displayMap(cv::Mat& mat){
  cv::Mat image_scaled;
  cv::resize(mat, image_scaled, cv::Size(), 2, 2, cv::INTER_LINEAR);
    cv::imshow("Map", image_scaled);
    cv::waitKey(0);
}
int main(int argc, char **argv) {
    // startPosCollection();
  //  startCameraCalibration();
     startCamMonitoring(constants::cameraCalibrationPath);
//     int scale = 10;
//     GridMap map(10 * scale, 10 * scale, 1);
//     std::cout << "Map created" << std::endl;
//     MapCreator mapCreator(map);
//     std::cout << "Map Creator has created" << std::endl;
//     Line line0(Point(1* scale, 1* scale), Point(5 * scale , 1* scale));
//     Line line1(Point(3* scale, 1* scale), Point(3* scale, 8* scale));
//     Line line2(Point(5* scale, 1* scale), Point(5* scale, 3* scale));
//     std::cout << "Line created" << std::endl;
//     mapCreator.addLine(line0);
//     mapCreator.addLine(line1);
//     mapCreator.addLine(line2);
//     std::cout << "Lines Added" << std::endl;
//     // displayMap(map.getMapCopy());
//     Point start(0, 0);
//     Point goal(9* scale, 9* scale);
//     std::vector<Point> path = findShortestPath(start, goal ,map);
//     std::cout << "Shortest path is found" << std::endl;
// cv::Mat mapCopy = map.getMapCopy();

// // Convert the path points to pixel coordinates
// std::vector<cv::Point> pathPixels;
// for (Point p : path) {
//     double x, y;
//   std::tie(x, y) = p.getCoordinate();
//     std::cout << "x : "<< x << ", y :" << y << std::endl;
//   int r = int(y);
//   int c = int(x);
//   pathPixels.emplace_back(r, c);
// }

// // Draw the path segments
// for (size_t i = 1; i < pathPixels.size(); i++) {
//   cv::line(mapCopy, pathPixels[i - 1], pathPixels[i], cv::Scalar(0, 255, 0), 1);
// }
//     displayMap(mapCopy);
    return 0;
  // createArucoMarkers(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50, "./img", 50);
}