# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# dynalgo_model_backend_trt — TensorRT C++ inference backend

cmake_minimum_required(VERSION 3.16)

project(dynalgo_model_backend_trt LANGUAGES CXX)

if(NOT ENABLE_TENSORRT)
    message(WARNING "ENABLE_TENSORRT=OFF, skipping TensorRT backend")
    return()
endif()

find_package(CUDA REQUIRED)
find_package(TensorRT REQUIRED)

add_library(dynalgo_model_backend_trt STATIC
    trt_backend.cpp
)

target_include_directories(dynalgo_model_backend_trt PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${CMAKE_CURRENT_SOURCE_DIR}/../../core
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIRS}
)

target_link_libraries(dynalgo_model_backend_trt PUBLIC
    dynalgo::core
    dynalgo::model_backends_common
    ${CUDA_LIBRARIES}
    ${TENSORRT_LIBRARIES}
)

target_compile_definitions(dynalgo_model_backend_trt PUBLIC ENABLE_TENSORRT)
target_compile_features(dynalgo_model_backend_trt PUBLIC cxx_std_17)

# CUDA compilation for any .cu files (if added later)
set(CUDA_SEPARABLE_COMPILATION ON)

add_library(dynalgo::model_backend_trt ALIAS dynalgo_model_backend_trt)

install(FILES
    trt_backend.hpp
    DESTINATION include/dynalgo
)