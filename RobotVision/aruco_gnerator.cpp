#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
// #include <fstream>
void saveMarker(cv::Mat outputMarker, std::string path, int id){
    std::ostringstream fileName;
    fileName << path << "aruco-" << id << ".png";
    cv::imwrite(fileName.str(), outputMarker);
}

void createArucoMarkers(cv::aruco::PREDEFINED_DICTIONARY_NAME name, std::string path, int numberOfGeneration=50) {
    cv::Mat outputMarker;
    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(name);
    for (int i = 0; i < numberOfGeneration; i++) {
        cv::aruco::drawMarker(markerDictionary, i, 500, outputMarker);
        saveMarker(outputMarker, path, i);
    }
}