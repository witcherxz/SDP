#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>
#include "map.cpp"
//#include <opencv2/calib3d.hpp>


using namespace cv;
using namespace std;
// TODO: Add the measurement for the calibration board cell and the aruco measurement
const float calibrationSquareDimension = 0.025f; // in meter
const float arucoSquareDimension = 0.14f; // in meter
const Size chessboardDimensions = Size(6, 9);
const int dictionaryName = aruco::DICT_6X6_1000;

void createArucoMarkers(cv::aruco::PREDEFINED_DICTIONARY_NAME name, string path) {
    Mat outputMarker;
    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(name);
    for (int i = 0; i < 50; i++) {

        aruco::drawMarker(markerDictionary, i, 500, outputMarker);
        ostringstream fileName;
        fileName << path << "aruco-" << i << ".png";
        imwrite(fileName.str(), outputMarker);
    }
}

void createKnownBoardPostion(Size boardSize, float squareEdgeLength, vector<Point3f> &corners) {
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            corners.push_back(Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
        }
    }
}

void
getChessboardCornersFromImages(vector<Mat> images, vector<vector<Point2f>> &allFoundCorners, bool showResults = false) {
    for (vector<Mat>::iterator itr = images.begin(); itr != images.end(); itr++) {
        vector<Point2f> pointBuf;
        // TODO: Do some experement on the flags
        bool found = findChessboardCorners(*itr, chessboardDimensions, pointBuf,
                                           CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            allFoundCorners.push_back(pointBuf);
        }
        if (showResults) {
            drawChessboardCorners(*itr, chessboardDimensions, pointBuf, found);
            imshow("cornerViewer", *itr);
            waitKey(0);
        }
    }
}

void openCam() {
    VideoCapture vid(0);

    if (!vid.isOpened()) return;

    Mat frame;
    while (vid.read(frame)) {
        imshow("Cam", frame);
        if (waitKey(1000 / 20) > 0) break;
    }
}

void cameraCalibration(vector<Mat> images, Size boardSize, float squareEdgeLength, Mat &cameraMatrix,
                       Mat &distanceCoefficients) {
    vector<vector<Point2f>> chessboardImageSpacePoints;
    getChessboardCornersFromImages(images, chessboardImageSpacePoints, false);

    vector<vector<Point3f>> realWorldSpacePoints(1);
    createKnownBoardPostion(boardSize, squareEdgeLength, realWorldSpacePoints[0]);
    realWorldSpacePoints.resize(chessboardImageSpacePoints.size(), realWorldSpacePoints[0]);

    vector<Mat> rVector, tVector;
    distanceCoefficients = Mat::zeros(8, 1, CV_16F);

    calibrateCamera(realWorldSpacePoints, chessboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients,
                    rVector, tVector);
}

void showCamErrorMassage() {
    std::cout << "Error : Could not open camera";
    exit(1);
}

void monitorCamera(Mat frame, VideoCapture vid, const std::function<void()> &func) {
    while (vid.read(frame)) {
        func();
    }
}
void showTranslationInfo(const Mat &frame, const vector<cv::Vec3d> &translationVectors) {
    if (translationVectors.size() > 0) {
        ostringstream distance;
        distance << "distance : " << translationVectors[0];
        putText(frame, distance.str(), Point2i(10, frame.cols - 200), FONT_HERSHEY_COMPLEX, .7,
                Scalar(0, 255, 0));
    }
}

void updatePositionOnMap(int arucoXPos, int arucoYPos, const vector<cv::Vec3d> &translationVectors, BasicMap &map) {
    if (!translationVectors.empty()) {
        map.updatePostion(arucoXPos - translationVectors[0][0], arucoYPos + translationVectors[0][1]);
    }
}

void drawMarkersOnCamera(const Mat &frame, const Mat &distanceCoefficients, const Mat &cameraMatrix,
                         const vector<int> &markerIds, const vector<cv::Vec3d> &rotationVectors,
                         const vector<cv::Vec3d> &translationVectors) {
    for (int i = 0; i < markerIds.size(); i++) {
        // ostringstream x;
        drawFrameAxes(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i],
                      arucoSquareDimension);
    }
}


void monitorArucoMarkers(Mat frame, const Mat &distanceCoefficients, const Mat &cameraMatrix) {
    int arucoXPos = 4;
    int arucoYPos = 6;
    vector<int> markerIds;
    vector<vector<Point2f>> markerCorners, rejectedCandidates;
    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(dictionaryName);
    std::vector<cv::Vec3d> rotationVectors, translationVectors;
    VideoCapture vid(0);
    namedWindow("Cam");

    if (!vid.isOpened()) showCamErrorMassage();
    BasicMap map(30, 10, 0.4);
    map.drawMap(2);
    while (vid.read(frame)) {
        aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
        aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients,
                                         rotationVectors, translationVectors);
        drawMarkersOnCamera(frame, distanceCoefficients, cameraMatrix, markerIds, rotationVectors, translationVectors);
        updatePositionOnMap(arucoXPos, arucoYPos, translationVectors, map);
        showTranslationInfo(frame, translationVectors);
        imshow("Cam", frame);
        if (waitKey(10) > 0) break;
    }
}

void startCamMonitoring(const Mat &cameraMatrix, const Mat &distanceCoefficients) {
    Mat frame;
    monitorArucoMarkers(frame, distanceCoefficients, cameraMatrix);
}

static void saveCameraCalibration(const string &filename, const Mat &cameraMatrix, const Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::WRITE);

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
}

bool loadCameraCalibration(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
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

void caleprateCameraFromSavedImages(Mat &cameraMatrix, Mat &distanceCoefficients, vector<Mat> &saveImages) {
    cout << "Processing..." << endl;
    cameraCalibration(saveImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix,
                      distanceCoefficients);
    cout << "saving" << endl;
    saveCameraCalibration("cameraCalibration", cameraMatrix, distanceCoefficients);
    cout << "Done!" << endl;
}

int excecuteKeyCommand(const Mat &frame, Mat &cameraMatrix, Mat &distanceCoefficients, vector<Mat> &saveImages,
                       int imagesCounter, bool isChessboardFound, char c) {
    switch (c) {
        case ' ':
            if (isChessboardFound) {
                saveFrame(frame, saveImages);
                imagesCounter++;
            }
            break;
        case 13:
            if (saveImages.size() > 15) {
                caleprateCameraFromSavedImages(cameraMatrix, distanceCoefficients, saveImages);
            }
            break;
        default:
            break;
    }
    return imagesCounter;
}

void showNumberOfImagesTaken(const Mat frame, int imagesCounter) {
    ostringstream counterLabel;
    counterLabel << "Number of images taken : " << imagesCounter;
    putText(frame, counterLabel.str(), Point2i(10, frame.cols - 200), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 0));
}

void startCameraCalibration() {
    Mat frame;
    Mat drawToFrame;
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distanceCoefficients;
    vector<Mat> saveImages;
    vector<vector<Point2f>> markerCorners, rejectedCandidates;
    VideoCapture vid(0);

    if (!vid.isOpened()) showCamErrorMassage();
    int framesPerSecond = 60;
    namedWindow("Cam", WINDOW_AUTOSIZE);
    int imagesCounter = 0;
    while (vid.read(frame)) {
        vector<Vec2f> foundPoints;
        bool isFound = false;
        isFound = findChessboardCorners(frame, chessboardDimensions, foundPoints,
                                        CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        showNumberOfImagesTaken(frame, imagesCounter);
        drawChessboardCorners(frame, chessboardDimensions, foundPoints, isFound);
        imshow("Cam", frame);
        char c = waitKey(1000 / framesPerSecond);
        imagesCounter = excecuteKeyCommand(frame, cameraMatrix, distanceCoefficients, saveImages, imagesCounter,
                                           isFound, c);

    }
}

int main(int argc, char **argv) {
    Mat cameraMatrix, distanceCoefficients;
    startCameraCalibration();
    // loadCameraCalibration("C:\\Users\\Legend\\Desktop\\SDP\\SDP\\RopotVision\\cameraCalibration", cameraMatrix, distanceCoefficients);
    // startCamMonitoring(cameraMatrix, distanceCoefficients);
    return 0;
}
