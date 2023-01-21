
#ifndef ROBOTVISION_POSECOLLECTION_H
#define ROBOTVISION_POSECOLLECTION_H

#include "../aruco_scanner.h"
#include "../opencv_constants.h"

class PoseCollection {
    public:
        void startPoseCollection();
    private:
        typedef std::tuple<double, double, double> pose_t;
        typedef std::vector<double> c_pose_t;
        std::unordered_map<int, std::vector<c_pose_t>> cameraRecord;
        std::vector<pose_t> realRecord;
        std::vector<std::vector<int>> detectedMarkersRecord;
        std::string filename = constants::systemCalibrationPath;
        ArucoScanner as = ArucoScanner();
        void saveSystemCalibration();
        void pushCameraRecords();
        void pushRealRecord();
        void popRecord();
};
#endif // ROBOTVISION_POSECOLLECTION_H