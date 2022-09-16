#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>
//#include <opencv2/calib3d.hpp>


using namespace cv;
using namespace std;
// TODO: Add the measurement for the calibration board cell and the aruco measurement
const float calibrationSquareDimension = 0.025f; // in meter
const float arucoDimension = 0.186f; // in meter
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

int startCamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients,float arucoSquareDimension, aruco::PREDEFINED_DICTIONARY_NAME dictionaryName) {
    Mat frame;

    vector<int> markerIds;
    vector<vector<Point2f>> markerCorners, rejectedCandidates;
    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(dictionaryName);

    VideoCapture vid(0);

    if (!vid.isOpened()) return -1;
    namedWindow("Cam");
    std::vector<cv::Vec3d> rotationVectors, translationVectors;
    while (vid.read(frame)) {
        aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
        aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);
        for (int i = 0; i < markerIds.size(); i++){
            // ostringstream x;
            drawFrameAxes(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], arucoSquareDimension);
            
        }
            if(translationVectors.size() > 0){
                ostringstream distance;
                distance << "distance : " <<  translationVectors[0];
                putText(frame, distance.str(), Point2i(10, frame.cols - 200), FONT_HERSHEY_COMPLEX, .7, Scalar(0, 255, 0));
            }
            //cout << "Dist : " << (translationVectors.at(2) * 100) << " cm";
            //cout << "Rot : " << (rotationVectors.at(2) / 3.14159265358979323846f * 180) << "deg";
        imshow("Cam", frame);
        if (waitKey(30) > 0) break;
    }
    FileStorage fs("aruco rotation and translation.json", FileStorage::Mode::WRITE);
    fs << "rotation";
    fs << rotationVectors;
    fs << "translation";
    fs << translationVectors;
    return 1;   

}
static void saveCameraCalibration(const string& filename,const Mat& cameraMatrix, const Mat& distCoeffs)
{
    FileStorage fs(filename, FileStorage::WRITE);

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
}

bool loadCameraCalibration(string filename, Mat& camMatrix, Mat& distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
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
    // namedWindow("Instruction", WINDOW_AUTOSIZE);
    namedWindow("Cam", WINDOW_AUTOSIZE);
    // Mat instImage = imread("calibrationInstruction.png");
    int imagesCounter = 0;

    // imshow("Instruction", instImage);
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
    // startCameraCalibration();
    loadCameraCalibration("cameraCalibration", cameraMatrix, distanceCoefficients);
    startCamMonitoring(cameraMatrix, distanceCoefficients, arucoDimension, aruco::DICT_6X6_1000);
    return 0;
}
