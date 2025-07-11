cmake_minimum_required(VERSION 3.5)
project(raspicam_fps_example)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_library(RASPICAM_LIB raspicam_cv)

add_executable(raspicam_fps main.cpp)

target_include_directories(raspicam_fps PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(raspicam_fps PRIVATE ${OpenCV_LIBS} ${RASPICAM_LIB})
