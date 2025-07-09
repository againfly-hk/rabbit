cmake_minimum_required(VERSION 3.10)
project(MyRaspberryPiProject)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/inc
    ${PROJECT_SOURCE_DIR}/inc/bmi088
)

# Find pigpio
find_library(PIGPIO_LIB pigpio REQUIRED)
find_library(PTHREAD_LIB pthread REQUIRED)

# Find raspicam
find_path(RASPICAM_INCLUDE_DIR raspicam/raspicam_cv.h)
find_library(RASPICAM_CV_LIB raspicam_cv)
find_library(RASPICAM_LIB raspicam)

include_directories(${RASPICAM_INCLUDE_DIR})

# Collect all source files
file(GLOB SRC
    src/*.cpp
    src/bmi088/*.cpp
)

add_executable(main ${SRC})

target_link_libraries(main
    ${OpenCV_LIBS}
    ${PIGPIO_LIB}
    ${PTHREAD_LIB}
    ${RASPICAM_CV_LIB}
    ${RASPICAM_LIB}
)