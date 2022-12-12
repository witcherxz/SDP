#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include "../opencv_constants.h"

using namespace cv;
using namespace std;

void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f> &corners) {
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            corners.push_back(Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
        }
    }
}

void getChessboardCornersFromImages(vector<Mat> images, vector<vector<Point2f>> &allFoundCorners, bool showCorners = false) {
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

void calibrateCameraFromSavedImages(Mat &cameraMatrix, Mat &distortionCoefficients, vector<Mat> &saveImages) {
    cout << "Processing..." << endl;
    cameraCalibration(saveImages, constants::chessboardDimensions, constants::calibrationSquareDimension, cameraMatrix,
                      distortionCoefficients);
    cout << "saving" << endl;
    saveCameraCalibration("cameraCalibration", cameraMatrix, distortionCoefficients);
    cout << "Done!" << endl;
}

void executeKeyCommand(const Mat &frame, Mat &cameraMatrix, Mat &distortionCoefficients, vector<Mat> &saveImages,
                      int &imagesCounter, bool isChessboardFound) {
    
    switch (waitKey(0)) {
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
        showNumberOfImagesTaken(frame, imagesCounter);
        drawChessboardCorners(frame, constants::chessboardDimensions, foundPoints, isFound);
        imshow("Cam", frame);
        executeKeyCommand(frame, cameraMatrix, distortionCoefficients, saveImages, imagesCounter,isFound);
    }
}

