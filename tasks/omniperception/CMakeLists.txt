cmake_minimum_required(VERSION 3.16)

# OpenVINO 的位置由顶层 CMakeLists 统一识别（不要再在这里写死路径覆盖 OpenVINO_DIR）
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(omniperception OBJECT 
    decider.cpp
    perceptron.cpp
)
target_link_libraries(omniperception openvino::runtime)