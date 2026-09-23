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


# ---------------------------------------------------------------------------
# 推理后端：按 CPU 架构自动识别，不需要再手动改 option
#   aarch64 / arm64  (Jetson 系列)   -> TensorRT
#   x86_64  / amd64  (x86 小电脑)    -> OpenVINO
# 需要强制指定时使用 -DSP_VISION_BACKEND=OPENVINO / TENSORRT / NONE
# （NONE = 不编译检测后端，只编译算法库和标定/测试工具）
# ---------------------------------------------------------------------------
set(SP_VISION_BACKEND "AUTO" CACHE STRING
    "推理后端: AUTO(按 CPU 架构识别) / OPENVINO / TENSORRT / NONE")
set_property(CACHE SP_VISION_BACKEND PROPERTY STRINGS AUTO OPENVINO TENSORRT NONE)

if(SP_VISION_BACKEND STREQUAL "AUTO")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
        set(SP_VISION_BACKEND "TENSORRT")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
        set(SP_VISION_BACKEND "OPENVINO")
    else()
        message(FATAL_ERROR
            "无法识别的 CPU 架构: ${CMAKE_SYSTEM_PROCESSOR}。"
            "请用 -DSP_VISION_BACKEND=OPENVINO 或 -DSP_VISION_BACKEND=TENSORRT 手动指定后端")
    endif()
    message(STATUS "自动识别 CPU 架构: ${CMAKE_SYSTEM_PROCESSOR} -> ${SP_VISION_BACKEND} 后端")
endif()

# OPENVINO_MAKE / TENSOR_RT_MAKE 由 SP_VISION_BACKEND 推导，保留这两个名字是因为
# tasks/auto_aim、tasks/auto_buff 等子目录仍按它们分支。
if(SP_VISION_BACKEND STREQUAL "OPENVINO")
    set(OPENVINO_MAKE ON)
    set(TENSOR_RT_MAKE OFF)
elseif(SP_VISION_BACKEND STREQUAL "TENSORRT")
    set(OPENVINO_MAKE OFF)
    set(TENSOR_RT_MAKE ON)
elseif(SP_VISION_BACKEND STREQUAL "NONE")
    set(OPENVINO_MAKE OFF)
    set(TENSOR_RT_MAKE OFF)
else()
    message(FATAL_ERROR
        "SP_VISION_BACKEND 只能是 AUTO / OPENVINO / TENSORRT / NONE，当前为: ${SP_VISION_BACKEND}")
endif()

# 老 build 目录里残留的 OPENVINO_MAKE/TENSOR_RT_MAKE 缓存不再生效（上面的普通变量
# 已经覆盖它们），这里顺手清掉，避免以后误以为旧开关还管用。
if(DEFINED CACHE{OPENVINO_MAKE} OR DEFINED CACHE{TENSOR_RT_MAKE})
    message(STATUS
        "清理历史缓存的 OPENVINO_MAKE/TENSOR_RT_MAKE；后端以 SP_VISION_BACKEND 为准")
    unset(OPENVINO_MAKE CACHE)
    unset(TENSOR_RT_MAKE CACHE)
endif()

option(ENABLE_GPU_TESTS "Enable TensorRT tests that require a CUDA GPU" OFF)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

if(OPENVINO_MAKE)
    # OpenVINO 位置：命令行 -DOpenVINO_DIR=... / 环境变量优先，否则试常见安装路径
    # （apt 装的 openvino-2024.6.0 在 /usr/lib/cmake，由 CMake 默认搜索路径找到）
    if(NOT OpenVINO_DIR AND NOT DEFINED ENV{OpenVINO_DIR})
        foreach(_ov_dir
                /opt/intel/openvino2024.6.0/runtime/cmake
                /opt/intel/openvino_2024.6.0/runtime/cmake
                /opt/intel/openvino/runtime/cmake)
            if(EXISTS "${_ov_dir}/OpenVINOConfig.cmake")
                set(OpenVINO_DIR "${_ov_dir}")
                break()
            endif()
        endforeach()
    endif()

    find_package(OpenVINO REQUIRED COMPONENTS Runtime)
    message(STATUS "使用 OpenVINO: ${OpenVINO_DIR}")
endif()

if(TENSOR_RT_MAKE)
    set(TENSORRT_ROOT "" CACHE PATH "TensorRT SDK root (optional when installed system-wide)")

    find_package(CUDAToolkit QUIET)
    if(NOT CUDAToolkit_FOUND)
        message(FATAL_ERROR
            "本次识别的后端是 TensorRT（架构 ${CMAKE_SYSTEM_PROCESSOR}），但没有找到 CUDA Toolkit；"
            "Jetson 请先刷好 JetPack，x86 请安装 CUDA Toolkit")
    endif()

    # CUDA 目标算力（nvcc 的 -gencode 目标 GPU）：
    #   1) 命令行 -DCMAKE_CUDA_ARCHITECTURES=89 优先
    #   2) 否则查当前 GPU：nvidia-smi 的 compute_cap 8.9 -> 89（多卡取第一张）
    #   3) 查不到再按架构兜底：aarch64(Jetson Orin) = 87；x86_64 交给 CMake 默认值
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
        set(_cuda_arch "")
        find_program(NVIDIA_SMI_EXECUTABLE nvidia-smi)
        if(NVIDIA_SMI_EXECUTABLE)
            execute_process(
                COMMAND "${NVIDIA_SMI_EXECUTABLE}" --query-gpu=compute_cap --format=csv,noheader
                OUTPUT_VARIABLE _cuda_compute_cap
                ERROR_QUIET
                RESULT_VARIABLE _cuda_smi_result
                OUTPUT_STRIP_TRAILING_WHITESPACE)
            if(_cuda_smi_result EQUAL 0 AND _cuda_compute_cap MATCHES "^[0-9]+\\.[0-9]+")
                string(REGEX REPLACE "\n.*" "" _cuda_compute_cap "${_cuda_compute_cap}") # 多卡只取第一张
                string(REGEX REPLACE "^([0-9]+)\\.([0-9]+).*" "\\1\\2" _cuda_arch "${_cuda_compute_cap}")
            endif()
        endif()

        # 卡可能比本机 nvcc 新（例：8.9 的卡 + nvcc 11.5 只支持到 8.7），直接用会
        # "nvcc fatal: Unsupported gpu architecture"。本机还可能装了多个 nvcc
        # （/usr/bin/nvcc 与 /usr/local/cuda/bin/nvcc 版本不同，CMake 用哪个不一定），
        # 所以取所有候选都支持的最大算力；同时会生成 PTX，向下兼容能正常运行。
        if(_cuda_arch)
            set(_nvcc_candidates
                "${CMAKE_CUDA_COMPILER}"
                "${CUDAToolkit_NVCC_EXECUTABLE}"
                "$ENV{CUDACXX}"
                "$ENV{CUDA_HOME}/bin/nvcc"
                "/usr/bin/nvcc")
            file(GLOB _cuda_opt_bins "/usr/local/cuda*/bin/nvcc")
            list(APPEND _nvcc_candidates ${_cuda_opt_bins})

            set(_cuda_arch_max 9999) # 所有候选 nvcc 都支持的最大算力
            set(_cuda_arch_min 0)    # 候选里最宽松的最低支持算力
            set(_nvcc_checked "")
            set(_nvcc_seen "")
            foreach(_nvcc ${_nvcc_candidates})
                if(NOT _nvcc OR NOT EXISTS "${_nvcc}")
                    continue()
                endif()
                get_filename_component(_nvcc_real "${_nvcc}" REALPATH) # 同一 nvcc 可能经多个路径出现
                if(_nvcc_real IN_LIST _nvcc_seen)
                    continue()
                endif()
                list(APPEND _nvcc_seen "${_nvcc_real}")

                execute_process(COMMAND "${_nvcc_real}" --list-gpu-arch
                    OUTPUT_VARIABLE _nvcc_arch_list
                    ERROR_QUIET
                    RESULT_VARIABLE _nvcc_list_result)
                if(NOT _nvcc_list_result EQUAL 0)
                    continue() # 老 nvcc 没有 --list-gpu-arch，跳过检查
                endif()

                set(_this_min 9999)
                set(_this_max 0)
                string(REGEX MATCHALL "compute_([0-9]+)" _nvcc_archs "${_nvcc_arch_list}")
                foreach(_nvcc_arch ${_nvcc_archs})
                    string(REPLACE "compute_" "" _nvcc_arch_num "${_nvcc_arch}")
                    if(_nvcc_arch_num GREATER _this_max)
                        set(_this_max "${_nvcc_arch_num}")
                    endif()
                    if(_nvcc_arch_num LESS _this_min)
                        set(_this_min "${_nvcc_arch_num}")
                    endif()
                endforeach()

                if(_this_max GREATER 0)
                    list(APPEND _nvcc_checked "${_nvcc}(compute_${_this_min}~${_this_max})")
                    if(_this_max LESS _cuda_arch_max)
                        set(_cuda_arch_max "${_this_max}")
                    endif()
                    if(_this_min GREATER _cuda_arch_min)
                        set(_cuda_arch_min "${_this_min}")
                    endif()
                endif()
            endforeach()

            if(_nvcc_checked)
                string(REPLACE ";" "  " _nvcc_checked_msg "${_nvcc_checked}")
                if(_cuda_arch LESS _cuda_arch_min)
                    message(FATAL_ERROR
                        "GPU 算力 ${_cuda_arch} 已经不被本机任何 nvcc 支持（${_nvcc_checked_msg}），"
                        "请安装对应版本的 CUDA Toolkit，或用 -DCMAKE_CUDA_ARCHITECTURES=... 手动指定")
                elseif(_cuda_arch GREATER _cuda_arch_max)
                    message(WARNING
                        "GPU 算力 ${_cuda_arch} 超出本机 nvcc 支持范围（${_nvcc_checked_msg}），"
                        "自动改用 ${_cuda_arch_max}（会一起生成 PTX，向下兼容，能正常运行）；"
                        "想用 ${_cuda_arch} 请安装更新的 CUDA Toolkit")
                    set(_cuda_arch "${_cuda_arch_max}")
                endif()
            endif()
        endif()

        if(NOT _cuda_arch AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
            set(_cuda_arch 87) # Jetson Orin
        endif()

        if(_cuda_arch)
            set(CMAKE_CUDA_ARCHITECTURES "${_cuda_arch}" CACHE STRING "CUDA architectures for TensorRT kernels")
            message(STATUS "自动识别 CUDA 目标算力: ${CMAKE_CUDA_ARCHITECTURES}")
        else()
            message(STATUS "没查到 GPU 算力，CUDA 目标算力交给 CMake 默认值（可用 -DCMAKE_CUDA_ARCHITECTURES=89 指定）")
        endif()
    else()
        message(STATUS "使用指定的 CUDA 目标算力: ${CMAKE_CUDA_ARCHITECTURES}")
    endif()

    # TensorRT 位置：-DTENSORRT_ROOT=... > 环境变量 > 系统安装 > /opt/TensorRT-*
    set(_TENSORRT_HINTS
        ${TENSORRT_ROOT}
        $ENV{TENSORRT_ROOT}
        $ENV{TensorRT_ROOT}
        /usr/src/tensorrt)
    file(GLOB _TENSORRT_OPT_DIRS "/opt/TensorRT-*")
    list(APPEND _TENSORRT_HINTS ${_TENSORRT_OPT_DIRS})

    find_path(TENSORRT_INCLUDE_DIR NvInfer.h
        HINTS ${_TENSORRT_HINTS}
        PATH_SUFFIXES include
                      include/aarch64-linux-gnu
                      include/x86_64-linux-gnu
                      targets/aarch64-linux-gnu/include
                      targets/x86_64-linux-gnu/include)
    find_library(TENSORRT_INFER_LIBRARY nvinfer
        HINTS ${_TENSORRT_HINTS}
        PATH_SUFFIXES lib lib64
                      lib/aarch64-linux-gnu lib/x86_64-linux-gnu
                      targets/x86_64-linux-gnu/lib targets/aarch64-linux-gnu/lib)
    find_library(TENSORRT_PLUGIN_LIBRARY nvinfer_plugin
        HINTS ${_TENSORRT_HINTS}
        PATH_SUFFIXES lib lib64
                      lib/aarch64-linux-gnu lib/x86_64-linux-gnu
                      targets/x86_64-linux-gnu/lib targets/aarch64-linux-gnu/lib)

    if(NOT TENSORRT_INCLUDE_DIR)
        message(FATAL_ERROR
            "本次识别的后端是 TensorRT（架构 ${CMAKE_SYSTEM_PROCESSOR}），但没有找到 NvInfer.h；"
            "可用 -DTENSORRT_ROOT=<TensorRT 目录> 指定，或 -DSP_VISION_BACKEND=OPENVINO 换后端")
    elseif(NOT TENSORRT_INFER_LIBRARY OR NOT TENSORRT_PLUGIN_LIBRARY)
        message(FATAL_ERROR
            "本次识别的后端是 TensorRT（架构 ${CMAKE_SYSTEM_PROCESSOR}），但没有找到 "
            "libnvinfer / libnvinfer_plugin；可用 -DTENSORRT_ROOT=<TensorRT 目录> 指定")
    endif()
    message(STATUS "使用 TensorRT: ${TENSORRT_INFER_LIBRARY}")
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

add_executable(rune_config_test tests/rune_config_test.cpp)
target_link_libraries(
    rune_config_test
    ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)
target_include_directories(rune_config_test PRIVATE
    ${PROJECT_SOURCE_DIR}/tasks/auto_buff/rp_core/config)
target_compile_definitions(rune_config_test PRIVATE
    AUTO_BUFF_TEST_DEFAULT_CONFIG="${PROJECT_SOURCE_DIR}/tasks/auto_buff/config/power_rune_defaults.yaml")
add_test(NAME rune_config_test COMMAND rune_config_test
    ${PROJECT_SOURCE_DIR}/configs/sb_long.yaml
    ${PROJECT_SOURCE_DIR}/configs/xiaohei.yaml)

add_executable(rune_motion_fit_test tests/rune_motion_fit_test.cpp)
target_link_libraries(
    rune_motion_fit_test
    ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)
target_include_directories(rune_motion_fit_test PRIVATE
    ${PROJECT_SOURCE_DIR}/tasks/auto_buff/rp_core/include
    ${PROJECT_SOURCE_DIR}/tasks/auto_buff/rp_core/config)
target_compile_definitions(rune_motion_fit_test PRIVATE
    AUTO_BUFF_TEST_DEFAULT_CONFIG="${PROJECT_SOURCE_DIR}/tasks/auto_buff/config/power_rune_defaults.yaml")
add_test(NAME rune_motion_fit_test COMMAND rune_motion_fit_test
    ${PROJECT_SOURCE_DIR}/configs/sb_long.yaml)

add_executable(rune_pose_adapter_test tests/rune_pose_adapter_test.cpp)
target_link_libraries(
    rune_pose_adapter_test
    ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)
target_include_directories(rune_pose_adapter_test PRIVATE
    ${PROJECT_SOURCE_DIR}/tasks/auto_buff/adapter
    ${PROJECT_SOURCE_DIR}/tasks/auto_buff/rp_core/include)
add_test(NAME rune_pose_adapter_test COMMAND rune_pose_adapter_test)

add_executable(rune_detector_decode_test tests/rune_detector_decode_test.cpp)
target_include_directories(rune_detector_decode_test PRIVATE
    ${PROJECT_SOURCE_DIR}/tasks/auto_buff/detector)
target_link_libraries(
    rune_detector_decode_test
    ${OpenCV_LIBS} fmt::fmt spdlog::spdlog auto_buff tools)
add_test(NAME rune_detector_decode_test COMMAND rune_detector_decode_test)
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

add_executable(
    center_acceleration_estimator_test
    tests/center_acceleration_estimator_test.cpp
    tasks/auto_aim/tracking/center_acceleration_estimator.cpp)
add_test(NAME center_acceleration_estimator_test COMMAND center_acceleration_estimator_test)

add_executable(thread_safe_queue_test tests/thread_safe_queue_test.cpp)
add_test(NAME thread_safe_queue_test COMMAND thread_safe_queue_test)

add_executable(thread_pool_test tests/thread_pool_test.cpp)
add_test(NAME thread_pool_test COMMAND thread_pool_test)

# 绝对时刻周期调度器：节拍网格对齐、超时跳拍不连发、均值不受工作耗时影响
add_executable(periodic_timer_test tests/periodic_timer_test.cpp)
target_link_libraries(periodic_timer_test ${OpenCV_LIBS} fmt::fmt tools)
add_test(NAME periodic_timer_test COMMAND periodic_timer_test)

add_executable(plotter_replay tools/plotter_replay.cpp tools/plotter.cpp)

add_executable(gimbal_calib tools/gimbal_calib.cpp)
target_link_libraries(gimbal_calib ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)

add_executable(rbplan_smoke_test tests/rbplan_smoke_test.cpp)
target_link_libraries(rbplan_smoke_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
# 方案 C 冒烟测试：校验 rbplan 在"真实云台状态 x0 + 提前量"下全程有限值且开火门限可达。
# 此前只有 add_executable、没有 add_test，ctest 永远不会执行到它。
add_test(
    NAME rbplan_smoke_test
    COMMAND rbplan_smoke_test ${PROJECT_SOURCE_DIR}/configs/sb_short.yaml)

add_executable(plotter_test tests/plotter_test.cpp tools/plotter.cpp)
add_test(NAME plotter_test COMMAND plotter_test)

add_executable(enemy_color_policy_test tests/enemy_color_policy_test.cpp)
target_link_libraries(enemy_color_policy_test ${OpenCV_LIBS})
add_test(NAME enemy_color_policy_test COMMAND enemy_color_policy_test)

add_executable(roi_test tests/roi_test.cpp)
target_link_libraries(roi_test ${OpenCV_LIBS})
add_test(NAME roi_test COMMAND roi_test)

add_executable(
    camera_image_processing_test
    tests/camera_image_processing_test.cpp
    io/camera/image_processing.cpp)
target_link_libraries(camera_image_processing_test ${OpenCV_LIBS})
add_test(NAME camera_image_processing_test COMMAND camera_image_processing_test)

add_executable(planner_raii_test tests/planner_raii_test.cpp)
target_link_libraries(
    planner_raii_test
    ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
add_test(
    NAME planner_raii_test
    COMMAND planner_raii_test ${PROJECT_SOURCE_DIR}/configs/sb_long.yaml)

add_executable(
    kf_posterior_residual_test
    tests/kf_posterior_residual_test.cpp
    tasks/auto_aim/model/armor.cpp
    tasks/auto_aim/tracking/target.cpp
    tasks/auto_aim/tracking/ca_from_tju.cpp
    tasks/auto_aim/tracking/rv_from_fyt.cpp
    tasks/auto_aim/tracking/uv_model.cpp)
target_link_libraries(
    kf_posterior_residual_test
    ${OpenCV_LIBS}
    fmt::fmt
    spdlog::spdlog
    tools)
add_test(NAME kf_posterior_residual_test COMMAND kf_posterior_residual_test)

add_executable(
    uv_jacobian_test
    tests/uv_jacobian_test.cpp
    tasks/auto_aim/tracking/uv_model.cpp)
target_link_libraries(uv_jacobian_test ${OpenCV_LIBS} fmt::fmt tools)
add_test(NAME uv_jacobian_test COMMAND uv_jacobian_test)

add_executable(
    uv_filter_test
    tests/uv_filter_test.cpp
    tasks/auto_aim/tracking/uv_model.cpp
    tasks/auto_aim/tracking/rv_from_fyt.cpp
    tasks/auto_aim/model/armor.cpp)
target_link_libraries(uv_filter_test ${OpenCV_LIBS} fmt::fmt tools)
add_test(NAME uv_filter_test COMMAND uv_filter_test)

add_executable(
    uv_regression_test
    tests/uv_regression_test.cpp
    tasks/auto_aim/tracking/uv_model.cpp
    tasks/auto_aim/tracking/rv_from_fyt.cpp
    tasks/auto_aim/tracking/target.cpp
    tasks/auto_aim/model/armor.cpp)
target_link_libraries(uv_regression_test ${OpenCV_LIBS} fmt::fmt tools)
add_test(NAME uv_regression_test COMMAND uv_regression_test)

if(OPENVINO_MAKE)
    add_executable(auto_aim_test tests/auto_aim_test.cpp)
    target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

    # 离线回放：真实检测 -> Tracker -> Planner，导出上位机命令流（用于甩头归因）
    add_executable(planner_offline_replay tests/planner_offline_replay.cpp)
    target_link_libraries(planner_offline_replay ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

    add_executable(tracker_offline_bench tests/tracker_offline_bench.cpp)
    target_link_libraries(tracker_offline_bench ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

    add_executable(auto_buff_test tests/auto_buff_test.cpp)
    target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)

    add_executable(rune_system_safety_test tests/rune_system_safety_test.cpp)
    target_link_libraries(
        rune_system_safety_test
        ${OpenCV_LIBS} fmt::fmt spdlog::spdlog auto_buff tools)
    add_test(
        NAME rune_system_safety_test
        COMMAND rune_system_safety_test ${PROJECT_SOURCE_DIR}/configs/sb_long.yaml)

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

    add_executable(perceptron_stop_test tests/perceptron_stop_test.cpp)
    target_link_libraries(
        perceptron_stop_test
        ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim omniperception)
    add_test(
        NAME perceptron_stop_test
        COMMAND perceptron_stop_test ${PROJECT_SOURCE_DIR}/configs/test/sentry.yaml)

    add_executable(safety_boundaries_test tests/safety_boundaries_test.cpp)
    target_link_libraries(
        safety_boundaries_test
        ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim omniperception)
    add_test(
        NAME safety_boundaries_test
        COMMAND safety_boundaries_test ${PROJECT_SOURCE_DIR}/configs/test/sentry.yaml)
endif()

if(TENSOR_RT_MAKE)

    add_executable(auto_buff_test tests/auto_buff_test.cpp)
    target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)

    add_executable(auto_aim_test tests/auto_aim_test.cpp)
    target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

    add_executable(tracker_offline_bench tests/tracker_offline_bench.cpp)
    target_link_libraries(tracker_offline_bench ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

    add_executable(auto_aim_test_async tests/auto_aim_test_async.cpp)
    target_link_libraries(auto_aim_test_async ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

    if(ENABLE_GPU_TESTS)
        add_executable(trt_pipeline_test tests/trt_pipeline_test.cpp)
        target_link_libraries(
            trt_pipeline_test
            ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
        add_test(
            NAME trt_pipeline_0526_test
            COMMAND trt_pipeline_test ${PROJECT_SOURCE_DIR}/configs/mouse.yaml)
        add_test(
            NAME trt_pipeline_0708_test
            COMMAND trt_pipeline_test ${PROJECT_SOURCE_DIR}/tests/config/trt_0708_test.yaml)
        set_tests_properties(
            trt_pipeline_0526_test trt_pipeline_0708_test
            PROPERTIES LABELS gpu TIMEOUT 30)
    endif()

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
