cmake_minimum_required(VERSION 3.10)
project(imu_cali)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${PROJECT_SOURCE_DIR}/inc
    ${PROJECT_SOURCE_DIR}/inc/bmi088
)

find_library(PIGPIO_LIB pigpio REQUIRED)

file(GLOB SRC
    src/*.cpp
    src/bmi088/*.cpp
)

add_executable(imu_cali ${SRC})

target_link_libraries(imu_cali
    ${PIGPIO_LIB}
)