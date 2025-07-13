cmake_minimum_required(VERSION 3.10)
project(pwm_control)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加可执行文件
add_executable(pwm_control main.cpp)

# 链接 pigpio 所需的库
target_link_libraries(pwm_control
    pigpio
    rt
    pthread
)
