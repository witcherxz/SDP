#include "localization.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>
Localization::Localization(){
    std::cout << "press i in the ground truth" << std::endl;
    std::function<void(cv::Mat &)> trackProccess = [=](cv::Mat& frame) {
        arucoScanner.estimateMarkersPose(frame);
        if (!arucoScanner.isArucoFound())
        {
            return;
        }
        
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
    
    std::cout << "Pose : " << track << std::endl;
    
}
void Localization::updateTrack(){
    int closestMarker = arucoScanner.getIdOfClosestMarker();
    if(trackedMarker != closestMarker){
        trackedMarker = closestMarker;
        rootOffset = track - arucoScanner.getPostion(trackedMarker);
    }
    track = arucoScanner.getPostion(trackedMarker) + rootOffset; 
}
void Localization::setInitialPose(){
    char input = cv::waitKey(1);
    if(input == 'i'){
        trackedMarker = arucoScanner.getIdOfClosestMarker();
        rootOffset = -arucoScanner.getPostion(trackedMarker);
        track = cv::Point_<double>(0, 0);
        isInit = true;
    }
}
