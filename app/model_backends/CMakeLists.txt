# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# dynalgo_model_backends — C++ inference backends (TensorRT, ONNX Runtime, RKNN)

cmake_minimum_required(VERSION 3.16)

project(dynalgo_model_backends LANGUAGES CXX)

# Common preprocessing utilities
add_subdirectory(common)

# TensorRT backend
option(ENABLE_TENSORRT "Enable TensorRT backend" OFF)
if(ENABLE_TENSORRT)
    find_package(CUDA REQUIRED)
    find_package(TensorRT REQUIRED)
    add_subdirectory(tensorrt)
endif()

# ONNX Runtime backend
option(ENABLE_ONNXRUNTIME "Enable ONNX Runtime backend" OFF)
if(ENABLE_ONNXRUNTIME)
    find_package(ONNXRuntime REQUIRED)
    add_subdirectory(onnxruntime)
endif()

# RKNN backend
option(ENABLE_RKNN "Enable RKNN backend" OFF)
if(ENABLE_RKNN)
    add_subdirectory(rknn)
endif()

# Meta-library that links all enabled backends
# Only create if at least one backend is enabled
if(ENABLE_TENSORRT OR ENABLE_ONNXRUNTIME OR ENABLE_RKNN)
    add_library(dynalgo_model_backends STATIC
        model_backends_dummy.cpp
    )

    target_include_directories(dynalgo_model_backends PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../core
        ${CMAKE_CURRENT_SOURCE_DIR}/common
    )

    target_link_libraries(dynalgo_model_backends PUBLIC
        dynalgo::core
        dynalgo::model_backends_common
    )

    if(ENABLE_TENSORRT)
        target_link_libraries(dynalgo_model_backends PUBLIC dynalgo::model_backend_trt)
    endif()

    if(ENABLE_ONNXRUNTIME)
        target_link_libraries(dynalgo_model_backends PUBLIC dynalgo::model_backend_ort)
    endif()

    if(ENABLE_RKNN)
        target_link_libraries(dynalgo_model_backends PUBLIC dynalgo::model_backend_rknn)
    endif()

    # C++17 required
    target_compile_features(dynalgo_model_backends PUBLIC cxx_std_17)

    add_library(dynalgo::model_backends ALIAS dynalgo_model_backends)
else()
    # Create interface library when no backends enabled
    add_library(dynalgo_model_backends INTERFACE)
    target_include_directories(dynalgo_model_backends INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../core
        ${CMAKE_CURRENT_SOURCE_DIR}/common
    )
    target_link_libraries(dynalgo_model_backends INTERFACE
        dynalgo::core
        dynalgo::model_backends_common
    )
    add_library(dynalgo::model_backends ALIAS dynalgo_model_backends)
endif()