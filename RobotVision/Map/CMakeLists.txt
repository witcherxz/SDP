cmake_minimum_required(VERSION 3.0.0)
project(RobotVision VERSION 0.1.0)

add_library(map map.cpp map.h)
add_library(mapScanner map_scanner.cpp map_scanner.h)
