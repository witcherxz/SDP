add_executable(RropotVision main.cpp)

find_package(OpenCv REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(RropotVision ${OpenCV_LIBS})
