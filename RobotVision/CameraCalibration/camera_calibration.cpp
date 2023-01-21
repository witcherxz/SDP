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

void testReadFile(const string filename){
    cv::FileStorage fs(filename, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    std::cout << fs.getFirstTopLevelNode().name() << std::endl;
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

void saveImagesInLocalStorage(std::vector<cv::Mat> images){
    for(int i = 0; i < images.size(); i++)
    {   std::string path = constants::calibrationImagesFolder + "\\" + std::to_string(i) + ".jpeg";
        cv::imwrite(path, images[i]);
    }
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
                saveImagesInLocalStorage(saveImages);
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
    putText(frame, counterLabel.str(), Point2i(100, frame.cols - 900), FONT_HERSHEY_PLAIN, 5, Scalar(0, 255, 0));
}




void startCameraCalibration() {
    Mat frame;
    Mat resized;
    Mat drawToFrame;
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distortionCoefficients;
    vector<Mat> saveImages;
    vector<vector<Point2f>> markerCorners, rejectedCandidates;
    VideoCapture vid(1);
    vid.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    vid.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    int imagesCounter = 0;
    namedWindow("Cam", WINDOW_AUTOSIZE);
    while (vid.read(frame)) {
        vector<Vec2f> foundPoints;
        bool isFound = findChessboardCorners(frame, constants::chessboardDimensions, foundPoints,
                                             CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        frame.copyTo(drawToFrame);
        showNumberOfImagesTaken(drawToFrame, imagesCounter);
        drawChessboardCorners(drawToFrame, constants::chessboardDimensions, foundPoints, isFound);
        cv::resize(drawToFrame, resized, cv::Size(), 0.5, 0.5);
        imshow("Cam", resized);
        executeKeyCommand(frame, cameraMatrix, distortionCoefficients, saveImages, imagesCounter, isFound);
    }
}

void CameraCenterCalibration::showAngleInfo(cv::Mat frame){
    std::string info = "ID :" + std::to_string(arucoScanner.getIdOfClosestMarker()) +", Angle : " + std::to_string(angle);
    cv::putText(frame, info, cv::Point(100, 150), cv::FONT_HERSHEY_SIMPLEX, 2,cv::Scalar(0, 255, 0), 3);
}
void CameraCenterCalibration::saveCalibration(){
    std::string filePath = constants::cameraCenterCalibrationPath;
    FileStorage fs(filePath, FileStorage::WRITE);
    if(!fs.isOpened()){
        std::cout << "Failed to open path : " << filePath << std::endl;
        exit(1);
    }
    fs << "cx" << cx;
    fs << "cy" << cy;
}

void CameraCenterCalibration::calculateCenter(){
    if(points.size() != oppositePoints.size()){
        std::cout << "The number of taken points does not equal the number of opposite points" << std::endl;
        return;
    }
    double avgcx = 0;
    double avgcy = 0;
    for (int i = 0; i < points.size(); i++){   
        double px, py;
        double opx, opy;
        std::tie(px, py) = points[i];
        std::tie(opx, opy) = oppositePoints[i];
        avgcx += (px + opx) / 2;
        avgcy += (py + opy) / 2;
    }
    int n = points.size();
    cx = avgcx / n;
    cy = avgcy / n;
}

void CameraCenterCalibration::addPoint(){
    double x, y, z;
    if(arucoScanner.isArucoFound()){
        std::tie(x, y, z) = arucoScanner.getOriginalPosition(arucoScanner.getIdOfClosestMarker());
        if(!isOpposite){
            std::cout << "base point is taken " << std::endl;
            points.push_back(std::make_tuple(x, y));
        }else{
            std::cout << "opposite point is taken " << std::endl;
            oppositePoints.push_back(std::make_tuple(x, y));
        }
        std::cout << "x : " << x << ", y : " << y << std::endl;
        isOpposite = !isOpposite;
    }else{
        std::cout << "Marker is not found" << std::endl;
    }

}

void CameraCenterCalibration::popPoint(){
    if(isOpposite){
        std::cout << "base point is removed " << std::endl;
        points.pop_back();
    }else{
        std::cout << "opposite point is removed " << std::endl;
        oppositePoints.pop_back();
    }
    isOpposite = !isOpposite;
}

void CameraCenterCalibration::centerCalibrationProccess(cv::Mat& frame){
    arucoScanner.estimateMarkersPose(frame);
    if(arucoScanner.isArucoFound()){
        angle = arucoScanner.getOrientation(arucoScanner.getIdOfClosestMarker());
        showAngleInfo(frame);
    }
    char input;
    input = cv::waitKey(1);
    if(input == 'c'){
        addPoint();
    }else if(input == 's'){
        calculateCenter();
        saveCalibration();
    }else if (input == 'p'){
        calculateCenter();
        std::cout << "cx : " << cx << ", cy : " << cy << std::endl;
    }else if (input == 'r'){
        popPoint();
    }
}

void CameraCenterCalibration::centerCalibration(){
    std::cout << "Take multiple pair data points that have the same postion but opposite angles" << std::endl;
    std::cout << "Press c to take a point" << std::endl;
    std::cout << "Press s to calculate average center and save it" << std::endl;
    std::cout << "Press p to print the calculated center" << std::endl;
    std::function<void(cv::Mat&)> Process = [=](cv::Mat& frame) {
            centerCalibrationProccess(frame);
    };
    arucoScanner.openCamera(Process);
}

std::tuple<double, double> CameraCenterCalibration::loadCameraCenter(){
    FileStorage fs(constants::cameraCenterCalibrationPath, FileStorage::READ);
    if (!fs.isOpened()) return std::make_tuple(0, 0);
    double cx, cy;
    fs["cx"] >> cx;
    fs["cy"] >> cy;
    return std::make_tuple(cx, cy);
}