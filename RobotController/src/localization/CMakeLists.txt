cmake_minimum_required(VERSION 3.0.2)
project(localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(roscpp REQUIRED)
find_package(OpenCV REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
 CATKIN_DEPENDS roscpp
 DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  LIBRARIES p2os_driver
  CATKIN_DEPENDS p2os_msgs nav_msgs roscpp geometry_msgs diagnostic_updater tf std_msgs kdl_parser
   ${Boost_INCLUDE_DIRS} 
  )

## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/RobotVision/opencv_constants.cpp src/Loclization/localization.cpp)
target_link_libraries(${PROJECT_NAME}_node
${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node arucoScanner cameraCalibration map mapCreator mapViewer aStar poseCollection ${OpenCV_LIBS})
add_subdirectory(src/RobotVision)
add_subdirectory(src/Loclization)



