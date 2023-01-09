
#if !defined(SDP_ROBOTVISION_ARUCOPOSEESTEMATION_H)
#define SDP_ROBOTVISION_ARUCOPOSEESTEMATION_H
#include <opencv2/core/mat.hpp>
#include <functional>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <unordered_map>


// class AverageFilter{
// public:
//     AverageFilter();
//     // AverageFilter(int bufferSize);
//     void add(cv::Mat pose);
//     cv::Mat getAverage();

// private:
//     int bufferSize;
//     std::vector<cv::Mat> buffer;
//     cv::Mat average;
// };


class AverageFilter{
public:
    AverageFilter();
    void add(cv::Mat vec);
    cv::Mat getAverage();

private:
    int mostFrequentElement(std::vector<double>);
    int bufferSize;
    std::vector<cv::Mat> buffer;
    std::vector<double> valBuffer;
    cv::Mat average;
};

class ArucoScanner{
    public:
        typedef cv::Mat vecs3d;
        // typedef std::vector<cv::Vec3d> vecs3d;
        // typedef std::unordered_map<int, cv::Vec3d> vecs3d;
        ArucoScanner();
        void estimateMarkersPose(cv::Mat frame);
        void monitorArucoMarkers();
        vecs3d getTranslationVectors();
        vecs3d getRotationVectors();

    private:
        vecs3d translationVectors;
        vecs3d rotationVectors;
        cv::Mat cameraMatrix, distortionCoefficients;
        cv::aruco::Dictionary markerDictionary;
        cv::aruco::DetectorParameters detectorParams;
        cv::aruco::ArucoDetector arucoDetector;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        std::vector<int> markerIds;
        std::vector<int> oldDetectedMarker;
        AverageFilter rotVecAvg;
        AverageFilter tranVecAvg;
        int idsVectorSize = 0;

        void showTransformationInfo(const cv::Mat &frame);
        void openCamera(std::function<void(cv::Mat&)> func);
        void drawArucoMarker(cv::Mat& frame);
        void showCamErrorMassage();
        std::function<void(cv::Mat&)> arucoMarkerDrawProccess = [=](cv::Mat& frame) {
            estimateMarkersPose(frame);
            drawArucoMarker(frame);
        };
        
};

#endif // SDP_ROBOTVISION_ARUCOPOSEESTEMATION_H
