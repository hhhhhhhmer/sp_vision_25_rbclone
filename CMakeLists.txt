cmake_minimum_required(VERSION 3.16.3)

set(CMAKE_BUILD_TYPE Release) #-O3优化
project(sp_vision)

enable_testing()

set(CMAKE_CXX_STANDARD 17)


# 设置用户本地库路径(jetson 专属)
set(USER_LOCAL_DIR $ENV{HOME}/.local)
list(APPEND CMAKE_PREFIX_PATH ${USER_LOCAL_DIR})
list(APPEND CMAKE_MODULE_PATH ${USER_LOCAL_DIR}/lib/cmake)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")


# Backend selection. Keep OpenVINO as the default, while allowing the TensorRT
# engine backend to be enabled from the command line without editing this file.
option(OPENVINO_MAKE "Build the OpenVINO backend" OFF)
option(TENSOR_RT_MAKE "Build the TensorRT backend" ON)

if(OPENVINO_MAKE AND TENSOR_RT_MAKE)
    message(FATAL_ERROR "OPENVINO_MAKE and TENSOR_RT_MAKE cannot both be enabled")
endif()

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

if(OPENVINO_MAKE)
    set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
    find_package(OpenVINO REQUIRED)
endif()

if(TENSOR_RT_MAKE)
    set(CMAKE_CUDA_ARCHITECTURES 87 CACHE STRING "CUDA architectures for TensorRT kernels")
    find_path(TENSORRT_INCLUDE_DIR NvInfer.h)
    find_library(TENSORRT_INFER_LIBRARY nvinfer)
    find_library(TENSORRT_PLUGIN_LIBRARY nvinfer_plugin)

    if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_INFER_LIBRARY OR NOT TENSORRT_PLUGIN_LIBRARY)
        message(FATAL_ERROR "TensorRT headers or libraries were not found")
    endif()
endif()

find_package(Ceres REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

include_directories(${PROJECT_SOURCE_DIR}/io/serial/include)


add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
if(OPENVINO_MAKE OR TENSOR_RT_MAKE)
    add_subdirectory(tasks/auto_buff)
endif()
if(OPENVINO_MAKE)
    add_subdirectory(tasks/omniperception)
endif()



####################src###################
if(OPENVINO_MAKE)
add_executable(rb_auto_aim_debug  src/rb_auto_aim_debug.cpp)
add_executable(rb_binocular_aim_debug  src/rb_binocular_aim_debug.cpp)
add_executable(sb_omn_aim_mpc src/sb_omn_aim_mpc.cpp)
add_executable(omni_perception src/omni_perception.cpp)
add_executable(hero  src/hero.cpp)
add_executable(rb_auto_aim_debug_async src/rb_auto_aim_debug_async.cpp)
add_executable(rb_binocular_aim_debug_async src/rb_binocular_aim_debug_async.cpp)



target_link_libraries(rb_auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(sb_omn_aim_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io omniperception)
target_link_libraries(omni_perception ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io omniperception)
target_link_libraries(rb_binocular_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(hero ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(rb_auto_aim_debug_async ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
target_link_libraries(rb_binocular_aim_debug_async ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

endif()

if(OPENVINO_MAKE OR TENSOR_RT_MAKE)
add_executable(rb_auto_standard src/rb_auto_standard.cpp)
add_executable(rb_auto_standard_debug src/rb_auto_standard_debug.cpp)
add_executable(rb_auto_standard_async src/rb_auto_standard_async.cpp)
add_executable(rb_auto_standard_debug_async src/rb_auto_standard_debug_async.cpp)
add_executable(rb_auto_aim_debug_fft  src/rb_auto_aim_debug_fft.cpp)

target_link_libraries(rb_auto_standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(rb_auto_standard_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(rb_auto_standard_async ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(rb_auto_standard_debug_async ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(rb_auto_aim_debug_fft ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
endif()
# rv
add_executable(rb_rv_aim_debug src/rb_rv_aim_debug.cpp)
target_link_libraries(rb_rv_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim  tools io)



##################calibration################
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_input calibration/calibrate_input.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)
add_executable(test_simple calibration/test_simple.cpp)
add_executable(calibrate calibration/calibrate.cpp)

target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(calibrate_input ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)
target_link_libraries(test_simple ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(calibrate ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)

##################tests##################
add_executable(fft_fit_test tests/fft_fit_test.cpp tools/fft.cpp)
add_test(NAME fft_fit_test COMMAND fft_fit_test)

# add_executable(
#     kf_posterior_residual_test
#     tests/kf_posterior_residual_test.cpp
#     tasks/auto_aim/armor.cpp
#     tasks/auto_aim/target.cpp
#     tasks/auto_aim/kf_example/ca_from_tju.cpp
#     tasks/auto_aim/kf_example/rv_from_fyt.cpp)
# target_link_libraries(
#     kf_posterior_residual_test
#     ${OpenCV_LIBS}
#     fmt::fmt
#     spdlog::spdlog
#     tools)
# add_test(NAME kf_posterior_residual_test COMMAND kf_posterior_residual_test)

if(OPENVINO_MAKE)
    add_executable(auto_aim_test tests/auto_aim_test.cpp)
    target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

    add_executable(auto_buff_test tests/auto_buff_test.cpp)
    target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)

    add_executable(camera_detect_test tests/camera_detect_test.cpp)
    target_link_libraries(camera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim io)

    add_executable(omniperception_test tests/omniperception_test.cpp)
    target_link_libraries(omniperception_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim omniperception)

    add_executable(binocular_camera_test tests/binocular_camera_test.cpp)
    target_link_libraries(binocular_camera_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

    add_executable(4cmaera_test tests/4cmaera_test.cpp)
    target_link_libraries(4cmaera_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

    add_executable(cameras_test tests/cameras_test.cpp)
target_link_libraries(cameras_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

add_executable(handeye_test tests/sp_example/handeye_test.cpp)
target_link_libraries(handeye_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

add_executable(gimbal_response_test tests/sp_example/gimbal_response_test.cpp)
target_link_libraries(gimbal_response_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)

add_executable(dm_test tests/sp_example/dm_test.cpp)
target_link_libraries(dm_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)

add_executable(minimum_vision_system tests/sp_example/minimum_vision_system.cpp)
target_link_libraries(minimum_vision_system ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

add_executable(camera_thread_test tests/sp_example/camera_thread_test.cpp)
target_link_libraries(camera_thread_test ${OpenCV_LIBS} fmt::fmt auto_aim tools io)

add_executable(planner_test_offline tests/sp_example/planner_test_offline.cpp)
target_link_libraries(planner_test_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io omniperception)

    add_executable(auto_aim_test_async tests/auto_aim_test_async.cpp)
    target_link_libraries(auto_aim_test_async ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
endif()

if(TENSOR_RT_MAKE)

    add_executable(auto_buff_test tests/auto_buff_test.cpp)
    target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)

    add_executable(auto_aim_test tests/auto_aim_test.cpp)
    target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

    add_executable(auto_aim_test_async tests/auto_aim_test_async.cpp)
    target_link_libraries(auto_aim_test_async ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

    

    add_executable(rbnx_auto_aim_debug src/rbnx_auto_aim_debug.cpp)
    target_link_libraries(rbnx_auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim  tools io)

    add_executable(rbnx_auto_aim_debug_async src/rbnx_auto_aim_debug_async.cpp)
    target_link_libraries(rbnx_auto_aim_debug_async ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)


endif()


add_executable(camera_test tests/camera_test.cpp)
target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)

add_executable(rv_camera_detect_test tests/rv_camera_detect_test.cpp)
target_link_libraries(rv_camera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim  tools io)

add_executable(gimbal_test tests/gimbal_test.cpp)
target_link_libraries(gimbal_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(serial_test tests/serial_test.cpp)
target_link_libraries(serial_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)


# 检查 ROS 环境
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(tf2_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)

if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
    # add_executable(publish_test tests/publish_test.cpp)
    # add_executable(subscribe_test tests/subscribe_test.cpp)
    # add_executable(topic_loop_test tests/topic_loop_test.cpp)
    
    # add_executable(sentry_binocular_aim_mpc src/sentry_binocular_aim_mpc.cpp)
    # add_executable(sentry_omn_aim_mpc src/sentry_omn_aim_mpc.cpp)

    # target_link_libraries(sentry_omn_aim_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    # target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim)
    # target_link_libraries(subscribe_test ${OpenCV_LIBS} fmt::fmt tools io)
    # target_link_libraries(topic_loop_test ${OpenCV_LIBS} fmt::fmt tools io)
    # target_link_libraries(sentry_binocular_aim_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
else()
    message(STATUS "ROS2 environment not found, skipping ROS2-related targets")
endif()
