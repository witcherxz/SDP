#include <opencv2/core/mat.hpp>

#if !defined(SDP_ROBOTVISION_ARUCOPOSEESTEMATION_H)
#define SDP_ROBOTVISION_ARUCOPOSEESTEMATION_H
void startCamMonitoring(const cv::Mat &cameraMatrix, const cv::Mat &distanceCoefficients);
#endif // SDP_ROBOTVISION_ARUCOPOSEESTEMATION_H
