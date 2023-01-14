
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

class ArucoScanner{
    public:
        typedef cv::Mat_<cv::Vec3d> vecs3d;
        ArucoScanner();
        void estimateMarkersPose(cv::Mat frame);
        void monitorArucoMarkers();
        std::vector<int> getDetectedMarkers();
        std::tuple<double, double> getPostion(int markerId);
        std::tuple<double, double> getOriginalPosition(int markerId);
        double getOrientation(int markerId);
        void proccessFrameOnAruco(std::function<void(cv::Mat&)> func);
        void openCamera(std::function<void(cv::Mat&)> func, bool showCamera=true);
        bool isArucoFound();
        int getIdOfClosestMarker();

    private:
    
        cv::Mat_<double> center;
        std::vector<int> markerIds;
        vecs3d translationVectors;
        vecs3d rotationVectors;
        std::unordered_map<int,std::vector<double>> xytheta;
        cv::Mat cameraMatrix, distortionCoefficients;
        cv::aruco::Dictionary markerDictionary;
        cv::aruco::DetectorParameters detectorParams;
        cv::aruco::ArucoDetector arucoDetector;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        std::vector<int> oldDetectedMarker;
        int idsVectorSize = 0;
        cv::Mat flipPitch = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, -1, 0, 0, 0, -1);
        
        void drawArucoMarker(cv::Mat& frame);
        void showCamErrorMassage();
        void showTransformationInfo(const cv::Mat &frame);
        void poseCorrection();
        void addPose(double x, double y, double angle, int i);
};

#endif // SDP_ROBOTVISION_ARUCOPOSEESTEMATION_H
