cmake_minimum_required(VERSION 3.10)
project(camera_calibration)

set(CMAKE_CXX_STANDARD 11
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(raspicam REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${raspicam_INCLUDE_DIRS})

add_executable(camera_calibration camera_calibration.cpp)

target_link_libraries(camera_calibration ${OpenCV_LIBS} raspicam raspicam_cv)
