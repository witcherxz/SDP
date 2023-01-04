#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include "../aruco_scanner.h"
#include "../opencv_constants.h"
#include "./camera_calibration.h"

using namespace cv;
using namespace std;

void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f> &corners) {
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            corners.push_back(Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
        }
    }
}

void
getChessboardCornersFromImages(vector<Mat> images, vector<vector<Point2f>> &allFoundCorners, bool showCorners = false) {
    for (vector<Mat>::iterator itr = images.begin(); itr != images.end(); itr++) {
        vector<Point2f> pointBuf;
        bool chessBoardFound = findChessboardCorners(*itr, constants::chessboardDimensions, pointBuf,
                                                     CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        if (chessBoardFound) {
            allFoundCorners.push_back(pointBuf);
        }
        if (showCorners) {
            drawChessboardCorners(*itr, constants::chessboardDimensions, pointBuf, chessBoardFound);
            imshow("cornerViewer", *itr);
            waitKey(0);
        }
    }
}

void cameraCalibration(vector<Mat> images, Size boardSize, float squareEdgeLength, Mat &cameraMatrix,
                       Mat &distortionCoefficients) {
    vector<vector<Point2f>> chessboardImageSpacePoints;
    getChessboardCornersFromImages(images, chessboardImageSpacePoints, false);
    vector<vector<Point3f>> realWorldSpacePoints(1);
    createKnownBoardPosition(boardSize, squareEdgeLength, realWorldSpacePoints[0]);
    realWorldSpacePoints.resize(chessboardImageSpacePoints.size(), realWorldSpacePoints[0]);
    vector<Mat> rVector, tVector;
    distortionCoefficients = Mat::zeros(8, 1, CV_16F);
    calibrateCamera(realWorldSpacePoints, chessboardImageSpacePoints, boardSize, cameraMatrix, distortionCoefficients,
                    rVector, tVector);
}

static void saveCameraCalibration(const string &filename, const Mat &cameraMatrix, const Mat &distCoeffs) {
    std::cout << filename << std::endl;
    FileStorage fs(filename, FileStorage::WRITE);
    if(!fs.isOpened()){
        std::cout << "Failed to open path : " << filename << std::endl;
        exit(1);
    }
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
}

void saveSystemCalibration(const string &filename, const vector<r_record_t> &real,const std::vector<std::vector<int>> listOfMarkerIds ,const std::map<int ,std::vector<c_record_t>> &camera){
    cv::FileStorage fs(filename, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    if(!fs.isOpened()){
        exit(1);
    }
    fs.startWriteStruct("real", cv::FileNode::SEQ);
    for(int i = 0; i < real.size(); i++)
    {
        double x, y, theta;
        std::tie(x, y, theta) = real[i];
        fs.startWriteStruct("", cv::FileNode::MAP);
        fs << "ids" << listOfMarkerIds[i];
        fs << "x" << x;
        fs << "y" << y;
        fs << "theta" << theta;
        fs.endWriteStruct();
    }
    fs.endWriteStruct();
    fs.startWriteStruct("camera", cv::FileNode::MAP);
    for(auto pair : camera)
    {
        int id = pair.first;
        fs.startWriteStruct("_"+std::to_string(id), cv::FileNode::SEQ);
        for(auto pos : pair.second)
        {
            double x, y, z, thetaX, thetaY, thetaZ;
            std::tie(x, y, z, thetaX, thetaY, thetaZ) = pos;
            fs.startWriteStruct("", cv::FileNode::MAP);
            fs << "x" << x;
            fs << "y" << y;
            fs << "z" << z;
            fs << "thetaX" << thetaX;
            fs << "thetaY" << thetaY;
            fs << "thetaZ" << thetaZ;
            fs.endWriteStruct();
        }
        fs.endWriteStruct();
    }
    fs.endWriteStruct();
}

bool loadCameraCalibration(string path, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(path, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

void saveFrame(const Mat &frame, vector<Mat> &saveImages) {
    Mat temp;
    frame.copyTo(temp);
    saveImages.push_back(temp);
}

void calibrateCameraFromSavedImages(Mat &cameraMatrix, Mat &distortionCoefficients, vector<Mat> &saveImages) {
    cout << "Processing..." << endl;
    cameraCalibration(saveImages, constants::chessboardDimensions, constants::calibrationSquareDimension, cameraMatrix,
                      distortionCoefficients);
    cout << "saving" << endl;
    std::cout << constants::cameraCalibrationPath << std::endl;
    saveCameraCalibration(constants::cameraCalibrationPath, cameraMatrix, distortionCoefficients);
    cout << "Done!" << endl;
}

void executeKeyCommand(const Mat &frame, Mat &cameraMatrix, Mat &distortionCoefficients, vector<Mat> &saveImages,
                       int &imagesCounter, bool isChessboardFound) {

    switch (waitKey(1)) {
        case ' ':
            if (isChessboardFound) {
                saveFrame(frame, saveImages);
                imagesCounter++;
            }
            break;
        case 13:
            if (saveImages.size() > 15) {
                calibrateCameraFromSavedImages(cameraMatrix, distortionCoefficients, saveImages);
            }
            break;
        default:
            break;
    }
}

void showNumberOfImagesTaken(const Mat frame, int imagesCounter) {
    ostringstream counterLabel;
    counterLabel << "Number of images taken : " << imagesCounter;
    putText(frame, counterLabel.str(), Point2i(10, frame.cols - 200), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 0));
}
void pushCameraRecords(std::map<int ,std::vector<c_record_t>> &cameraRecords,std::vector<int> ids,std::vector<cv::Vec3d> rVecs, std::vector<cv::Vec3d> tVecs){
    for (int i = 0; i < tVecs.size(); i++)
    {
        c_record_t record = std::make_tuple(tVecs[i][0], tVecs[i][1], tVecs[i][2], rVecs[i][0], rVecs[i][1], rVecs[i][2]);
        cameraRecords[ids[i]].push_back(record);
    }
}
void pushRealRecord(std::vector<r_record_t>& real){
    // std::vector<std::string> names{"x", "y" ,"theta"};
    std::string buff;
    double x, y, theta;
    std::cout << "Enter x :" << std::endl;
    std::cin >> buff;
    x = std::stod(buff);
    std::cout << "Enter y :" << std::endl;
    std::cin >> buff;
    y = std::stod(buff);
    std::cout << "Enter theta:" << std::endl;
    std::cin >> buff;
    theta = std::stod(buff);
    r_record_t realRecord = std::make_tuple(x, y, theta);
    real.push_back(realRecord);
}
void startPosCollection(){
    Mat frame;
    VideoCapture vid(0);
    cv::Mat cameraMatrix, distortionCoefficients;
    std::vector<int> markerIds;
    std::vector<std::vector<int>> capturedMarkers;
    std::vector<cv::Vec3d> rotationVectors, translationVectors;
    std::vector<r_record_t> real;
    std::map<int, std::vector<c_record_t>> camera;
    
    loadCameraCalibration(constants::cameraCalibrationPath, cameraMatrix, distortionCoefficients);
    while (vid.read(frame)) {
        estimateMarkersPose(frame, distortionCoefficients, cameraMatrix, markerIds, rotationVectors, translationVectors);
        drawMarkersOnFrame(frame, distortionCoefficients, cameraMatrix, markerIds, rotationVectors, translationVectors);
        char c = cv::waitKey(1);
        if(c == ' '){
            pushRealRecord(real);
            pushCameraRecords(camera, markerIds, rotationVectors, translationVectors);
            capturedMarkers.push_back(markerIds);

        }
        if(c == 's'){
            saveSystemCalibration(constants::systemCalibrationPath, real, capturedMarkers ,camera);
            std::cout << "File Saved in : " << constants::systemCalibrationPath << std::endl;
            exit(0);
        }
        imshow("Cam", frame);
    }
}
void startCameraCalibration() {
    Mat frame;
    Mat drawToFrame;
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distortionCoefficients;
    vector<Mat> saveImages;
    vector<vector<Point2f>> markerCorners, rejectedCandidates;
    VideoCapture vid(0);
    int imagesCounter = 0;
    namedWindow("Cam", WINDOW_AUTOSIZE);
    while (vid.read(frame)) {
        vector<Vec2f> foundPoints;
        bool isFound = findChessboardCorners(frame, constants::chessboardDimensions, foundPoints,
                                             CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        frame.copyTo(drawToFrame);
        showNumberOfImagesTaken(drawToFrame, imagesCounter);
        drawChessboardCorners(drawToFrame, constants::chessboardDimensions, foundPoints, isFound);
        imshow("Cam", drawToFrame);
        executeKeyCommand(frame, cameraMatrix, distortionCoefficients, saveImages, imagesCounter, isFound);
    }
}

