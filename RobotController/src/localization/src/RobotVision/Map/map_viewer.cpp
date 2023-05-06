#include "map_viewer.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
// #include <opencv2\imgproc.hpp>

MapViewer::MapViewer(GridMap map): map(map){
}

void MapViewer::drawMap() {
    // cv::resize(mat,dst, cv::Size(), 10, 10);
    
    // cv::namedWindow("Display window");
    // cv::imshow("Display window", dst);
    // cv::waitKey(0);
}