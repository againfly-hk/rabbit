cmake_minimum_required(VERSION 3.10)
project(camera_calibration)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

find_path(RASPICAM_INCLUDE_DIR raspicam/raspicam_cv.h)
find_library(RASPICAM_CV_LIB raspicam_cv)
find_library(RASPICAM_LIB raspicam)

include_directories(${RASPICAM_INCLUDE_DIR})

add_executable(camera_calibration camera_calibration.cpp)

target_link_libraries(camera_calibration ${OpenCV_LIBS} ${RASPICAM_LIB} ${RASPICAM_CV_LIB})
