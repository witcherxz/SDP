cmake_minimum_required(VERSION 3.0.0)
project(${CMAKE_PROJECT_NAME} VERSION 0.1.0)

add_library(cameraCalibration camera_calibration.cpp)
add_library(poseCollection pose_collection.cpp pose_collection.h)
