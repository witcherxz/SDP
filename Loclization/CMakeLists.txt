cmake_minimum_required(VERSION 3.0.0)
project(SDP VERSION 0.1.0)

add_executable(${CMAKE_PROJECT_NAME} main.cpp ./RobotVision/opencv_constants.cpp)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(${CMAKE_PROJECT_NAME} arucoScanner cameraCalibration map mapCreator mapViewer aStar arucoGnerator localization poseCollection ${OpenCV_LIBS})

add_subdirectory(RobotVision)
add_subdirectory(Loclization)