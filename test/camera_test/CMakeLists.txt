cmake_minimum_required(VERSION 3.10)
project(green_detector)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=armv7-a -mfpu=neon -mtune=cortex-a7")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_library(RASPICAM_CV_LIB raspicam_cv)
find_library(RASPICAM_LIB raspicam)

add_executable(green_detector main.cpp)
target_link_libraries(green_detector ${OpenCV_LIBS} ${RASPICAM_CV_LIB} ${RASPICAM_LIB})
