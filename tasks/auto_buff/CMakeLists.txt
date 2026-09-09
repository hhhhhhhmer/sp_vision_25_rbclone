cmake_minimum_required(VERSION 3.16)
find_package(Ceres REQUIRED)

set(AUTO_BUFF_SOURCES
    buff_type.cpp
    buff_track_bank.cpp
    buff_detector.cpp
    buff_solver.cpp
    buff_aimer.cpp
    buff_target.cpp
    yolo11_buff.cpp
)

if(TENSOR_RT_MAKE)
    enable_language(CUDA)
    list(APPEND AUTO_BUFF_SOURCES trt_yolo11_buff_kernel.cu)
endif()

add_library(auto_buff OBJECT ${AUTO_BUFF_SOURCES})

target_link_libraries(auto_buff PUBLIC auto_aim ${CERES_LIBRARIES})

if(OPENVINO_MAKE)
    target_compile_definitions(auto_buff PRIVATE SP_AUTO_BUFF_OPENVINO)
    target_link_libraries(auto_buff PUBLIC openvino::runtime)
elseif(TENSOR_RT_MAKE)
    find_package(CUDAToolkit REQUIRED)
    target_compile_definitions(auto_buff PRIVATE SP_AUTO_BUFF_TENSORRT)
    target_include_directories(auto_buff PRIVATE ${TENSORRT_INCLUDE_DIR})
    target_link_libraries(auto_buff PUBLIC
        ${TENSORRT_INFER_LIBRARY}
        ${TENSORRT_PLUGIN_LIBRARY}
        CUDA::cudart)
endif()
