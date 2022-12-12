#include <opencv2/aruco.hpp>

#if !defined(SDP_ROBOTVISION_ARUCOMARKERS_H)
#define SDP_ROBOTVISION_ARUCOMARKERS_H


void createArucoMarkers(cv::aruco::PREDEFINED_DICTIONARY_NAME name, std::string path, int numberOfGeneration=50);

#endif // SDP_ROBOTVISION_ARUCOMARKERS_H
