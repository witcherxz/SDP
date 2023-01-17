cmake_minimum_required(VERSION 3.0.0)
project(AhmadIbnMajid VERSION 0.1.0)

add_executable(${CMAKE_PROJECT_NAME} main.cpp ./RobotVision/opencv_constants.cpp)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(${CMAKE_PROJECT_NAME} ${OpenCV_LIBS} cameraCalibration map mapCreator mapViewer aStar arucoGnerator arucoScanner localization)

add_subdirectory(RobotVision)
add_subdirectory(Loclization)