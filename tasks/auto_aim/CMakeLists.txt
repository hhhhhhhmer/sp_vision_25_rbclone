cmake_minimum_required(VERSION 3.16)

if(OPENVINO_MAKE)
    set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
    find_package(OpenVINO REQUIRED COMPONENTS Runtime)
    add_definitions(-DOPENVINO_MAKE)
    add_library(auto_aim OBJECT 
        armor.cpp
        classifier.cpp 
        detector.cpp
        solver.cpp
        aimer.cpp
        target.cpp
        kf_example/ca_from_tju.cpp
        kf_example/rv_from_fyt.cpp
        tracker.cpp
        voter.cpp
        shooter.cpp
        rv_detector.cpp
        rv_number.cpp
        yolo.cpp
        ov_yolos/yolov5.cpp
        ov_yolos/yolov8.cpp
        ov_yolos/yolo11.cpp
        multithread/commandgener.cpp
        multithread/mt_detector.cpp
        planner/planner.cpp
    )
    add_subdirectory(planner/tinympc)
    target_link_libraries(auto_aim io openvino::runtime tinympcstatic)

    message(STATUS "--------------------OpenVINO_MAKE is ON--------------------")
endif()


if(TENSOR_RT_MAKE)
    add_definitions(-DTENSOR_RT_MAKE)
    
    find_package(CUDAToolkit REQUIRED)

    # OpenCV
    find_package(OpenCV 4.5.4 REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})

    enable_language(CUDA)

    add_library(auto_aim OBJECT 
        armor.cpp
        solver.cpp
        aimer.cpp
        target.cpp
        kf_example/ca_from_tju.cpp
        kf_example/rv_from_fyt.cpp
        tracker.cpp
        voter.cpp
        shooter.cpp
        rv_detector.cpp
        rv_number.cpp
        yolo.cpp
        trt_yolos/trt_yolo_0708.cpp
        trt_yolos/trt_0708_kernel.cu
        trt_yolos/trt_yolo_0526.cpp
        trt_yolos/trt_0526_kernel.cu
        planner/planner.cpp
    )
    add_subdirectory(planner/tinympc)
    target_link_libraries(auto_aim  PUBLIC 
                                    io 
                                    tinympcstatic     
                                    ${OpenCV_LIBS}
                                    ${TENSORRT_INFER_LIBRARY}
                                    CUDA::cudart  
                                    pthread)
    target_include_directories(auto_aim PRIVATE ${TENSORRT_INCLUDE_DIR})

    message(STATUS "--------------------TENSOR_RT_MAKE is ON--------------------")
endif()

if(NOT OPENVINO_MAKE AND NOT TENSOR_RT_MAKE)
    add_library(auto_aim OBJECT 
        armor.cpp
        solver.cpp
        aimer.cpp
        target.cpp
        kf_example/ca_from_tju.cpp
        kf_example/rv_from_fyt.cpp
        tracker.cpp
        voter.cpp
        shooter.cpp
        rv_detector.cpp
        rv_number.cpp
        planner/planner.cpp
    )
    add_subdirectory(planner/tinympc)
    target_link_libraries(auto_aim io tinympcstatic)
    message(STATUS "------------------OpenVINO_MAKE is OFF  TENSOR_RT_MAKE is OFF----------------------")
endif()
