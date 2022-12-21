cmake_minimum_required(VERSION 3.0.0)
project(RobotVision VERSION 0.1.0)

add_executable(RobotVision main.cpp opencv_constants.cpp)

add_library(arucoGnerator aruco_gnerator.cpp aruco_gnerator.h)
# add_library(arucoScanner aruco_scanner.cpp aruco_scanner.h)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(RobotVision ${OpenCV_LIBS} cameraCalibration map mapCreator mapViewer aStar)

add_subdirectory(CameraCalibration)
add_subdirectory(Map)