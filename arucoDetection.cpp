#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace std;
// TODO: Add the measurement for the calibration board cell and the aruco measurement
const float calibrationSquareDimension = 0.0239f; // in meter
const float arucoDimension = 0.133f; // in meter
const Size chessboardDimensions = Size(6, 9);

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

void createKnownBoardPostion(Size boardSize, float squareEdgeLength, vector<Point3f>& corners) {
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            corners.push_back(Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
        }
    }
}

void getChessboardCornersFromImages(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults = false) {
    for (vector<Mat>::iterator itr = images.begin(); itr != images.end(); itr++) {
        vector<Point2f> pointBuf;
        // TODO: Do some experement on the flags
        bool found = findChessboardCorners(*itr, chessboardDimensions, pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
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

void cameraCalivration(vector<Mat> images, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients) {
    vector<vector<Point2f>> chessboardImageSpacePoints;
    getChessboardCornersFromImages(images, chessboardImageSpacePoints, false);

    vector <vector<Point3f>> realWorldSpacePoints(1);
    createKnownBoardPostion(boardSize, squareEdgeLength, realWorldSpacePoints[0]);
    realWorldSpacePoints.resize(chessboardImageSpacePoints.size(), realWorldSpacePoints[0]);

    vector<Mat> rVector, tVector;
    distanceCoefficients = Mat::zeros(8, 1, CV_16F);

    calibrateCamera(realWorldSpacePoints, chessboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVector, tVector);


}

bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients) {
    ofstream outStream(name);
    if (outStream) {
        uint16_t rows = cameraMatrix.rows;
        uint16_t cols = cameraMatrix.cols;
        outStream << rows << endl;
        outStream << cols << endl;
        for (int r = 0; r < rows; r++){
            for (int c = 0; c < cols; c++) {
                double value = cameraMatrix.at<double>(r, c);
                outStream << value << endl;
            }
        }
        rows = distanceCoefficients.rows;
        cols = distanceCoefficients.cols;
        outStream << rows << endl;
        outStream << cols << endl;
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                double value = distanceCoefficients.at<double>(r, c);
                outStream << value << endl;
            }
        }
        return true;
    }
    return false;
}

bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distanceCoefficients) {
    ifstream inStream(name);
    if (inStream) {
        uint16_t rows;
        uint16_t cols;
        inStream >> rows;
        inStream >> cols;
        cout << "Load Camera Calibration\n";
        Mat cameraMatrix = Mat(Size(cols, rows), CV_64F);
        for (int r = 0; r < rows; r++){
            for (int c = 0; c < cols; c++) {
                double read = 0.0f;
                inStream >> read;
                cameraMatrix.at<double>(r, c) = read;
                cout << cameraMatrix.at<double>(r, c) << "\n";
            }
        }

        distanceCoefficients = Mat::zeros(rows, cols, CV_64F);
        cout << "distanceCoefficients : \n";
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                double read = 0.0f;
                inStream >> read;
                distanceCoefficients.at<double>(r, c) = read;
                cout << cameraMatrix.at<double>(r, c) << "\n";
            }
        }
        inStream.close();
        return 1;

    }
    return false;
}


int startCamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients,float arucoSquareDimension) {
    Mat frame;

    vector<int> markerIds;
    vector<vector<Point2f>> markerCorners, rejectedCandidates;
    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    VideoCapture vid(0);

    if (!vid.isOpened()) return -1;
    namedWindow("Cam");
    //std::vector<cv::Vec3d> rotationVectors, translationVectors;
    Mat rotationVectors, translationVectors;
    cout << "start monitring \n";
    while (vid.read(frame)) {
        aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
        aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);
        cout << markerIds.size();
        for (int i = 0; i < markerIds.size(); i++) {
            cout << "Marker ID : " << markerIds.at(i);
        }
        for (int i = 0; i < markerIds.size(); i++){
            drawFrameAxes(frame, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors, arucoSquareDimension, 0.1);
        }
        imshow("Cam", frame);
        if (waitKey(30) > 0) break;
    }
    return 1;   
}
void startCameraCalibration() {
    Mat frame;
    Mat drawToFrame;

    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);

    Mat distanceCoefficients;

    vector<Mat> saveImages;

    vector<vector<Point2f>> markerCorners, rejectedCandidates;

    VideoCapture vid(0);

    if (!vid.isOpened()) return;

    int framesPerSecond = 20;
    namedWindow("Instruction", WINDOW_AUTOSIZE);
    namedWindow("Cam", WINDOW_AUTOSIZE);
    Mat instImage = imread("calibrationInstruction.png");
    int imagesCounter = 0;

    imshow("Instruction", instImage);
    while (vid.read(frame)) {
        vector<Vec2f> foundPoints;
        bool isFound = false;
        isFound = findChessboardCorners(frame, chessboardDimensions, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        ostringstream counterLabel;
        counterLabel << "Number of images taken : " << imagesCounter;
        putText(frame, counterLabel.str(), Point2i(10, frame.cols - 200), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 0));
        frame.copyTo(drawToFrame);
        drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, isFound);

        if (isFound) {
            imshow("Cam", drawToFrame);
        }
        else {
            imshow("Cam", frame);
        }
        char c = waitKey(1000 / framesPerSecond);
        switch (c)
        {
        case ' ':
            if (isFound) {
                Mat temp;
                frame.copyTo(temp);
                saveImages.push_back(temp);
                imagesCounter++;
            }
            break;
        case 13:
            if (saveImages.size() > 15) {
                cout << "calculating" << endl;
                cameraCalivration(saveImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
                cout << "saving" << endl;
                saveCameraCalibration("cameraCalibration", cameraMatrix, distanceCoefficients);
                cout << "end saving" << endl;
            }
            break;
        case 27:
            break;
        default:
            //return 0;
            break;
        }

    }
}


int main(int argc, char** argv)
{   
    Mat cameraMatrix, distanceCoefficients;
    loadCameraCalibration("cameraCalibration", cameraMatrix, distanceCoefficients);
    startCamMonitoring(cameraMatrix, distanceCoefficients, arucoDimension);
    //Mat img = imread("marker.png");
    //imshow("test", img);
    return 0;
}
