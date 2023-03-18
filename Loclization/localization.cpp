#include "localization.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>
Localization::Localization(){
    std::cout << "press i in the ground truth" << std::endl;
    std::function<void(cv::Mat &)> trackProccess = [=](cv::Mat& frame) {
        arucoScanner.estimateMarkersPose(frame);
        if (!arucoScanner.isArucoFound()){
            return;
        }
        arucoScanner.drawArucoMarker(frame);
        if(!isInit){
            setInitialPose();
        }else{
            tracker();
        }
    };
    arucoScanner.openCamera(trackProccess);
}
void Localization::tracker(){
    updateTrack();
    printf("Pose { x: %.0f, y: %.0f }\n", track.x, track.y);
    
}
cv::Point_<double> Localization::getPostion(){
    cv::Point_<double> initPose = arucoScanner.getPostion(trackedMarker);
    double r = (orientation - arucoScanner.getOrientation(trackedMarker)) * (PI/180);
    double x = initPose.x;
    double y = initPose.y;
    double nx = x;//cos(r)*x - sin(r)*y;
    double ny = y;//sin(r)*x + cos(r)*y;
    return cv::Point_<double>(nx, ny);
}
void Localization::updateTrack(){
    int closestMarker = arucoScanner.getIdOfClosestMarker();
    if(trackedMarker != closestMarker){
        trackedMarker = closestMarker;
        rootOffset = track - getPostion();
    }
    track = getPostion() + rootOffset; 
}

void Localization::setInitialPose(){
    char input = cv::waitKey(1);
    if(input == 'i'){
        trackedMarker = arucoScanner.getIdOfClosestMarker();
        rootOffset = -arucoScanner.getPostion(trackedMarker);
        orientation = arucoScanner.getOrientation(trackedMarker);
        track = cv::Point_<double>(0, 0);
        isInit = true;
    }
}
