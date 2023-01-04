#include <opencv2/core/mat.hpp>

#if !defined(SDP_ROBOTVISION_ARUCOPOSEESTEMATION_H)
#define SDP_ROBOTVISION_ARUCOPOSEESTEMATION_H

void estimateMarkersPose(const cv::Mat frame, const cv::Mat distortionCoefficients, const cv::Mat cameraMatrix, std::vector<int>& markerIds, std::vector<cv::Vec3d> &rotationVectors, std::vector<cv::Vec3d> &translationVectors);
void drawMarkersOnFrame(const cv::Mat &frame, const cv::Mat &distanceCoefficients, const cv::Mat &cameraMatrix,
                        const std::vector<int> &markerIds, const std::vector<cv::Vec3d> &rotationVectors,
                        const std::vector<cv::Vec3d> &translationVectors);
void startCamMonitoring(std::string cameraCalibrationPath);

#endif // SDP_ROBOTVISION_ARUCOPOSEESTEMATION_H
