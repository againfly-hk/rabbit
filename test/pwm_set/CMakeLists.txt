cmake_minimum_required(VERSION 3.10)
project(pwm_set)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(pwm_set main.cpp)

target_link_libraries(pwm_set
    pigpio
)
