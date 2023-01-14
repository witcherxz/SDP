cmake_minimum_required(VERSION 3.0.0)
project(${CMAKE_PROJECT_NAME} VERSION 0.1.0)

add_library(arucoGnerator aruco_gnerator.cpp aruco_gnerator.h)
add_library(arucoScanner aruco_scanner.cpp aruco_scanner.h)

add_subdirectory(CameraCalibration)
add_subdirectory(Map)