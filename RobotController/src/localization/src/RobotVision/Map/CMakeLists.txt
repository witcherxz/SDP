cmake_minimum_required(VERSION 3.0.0)
project(${CMAKE_PROJECT_NAME} VERSION 0.1.0)

add_library(map map.cpp map.h)
add_library(mapViewer map_viewer.cpp map_viewer.h)
add_library(mapCreator map_creator.cpp map_creator.h)
add_library(aStar a_star.cpp)
