# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# dynalgo_model_backend_ort — ONNX Runtime C++ inference backend

cmake_minimum_required(VERSION 3.16)

project(dynalgo_model_backend_ort LANGUAGES CXX)

if(NOT ENABLE_ONNXRUNTIME)
    message(WARNING "ENABLE_ONNXRUNTIME=OFF, skipping ONNX Runtime backend")
    return()
endif()

find_package(ONNXRuntime REQUIRED)

add_library(dynalgo_model_backend_ort STATIC
    ort_backend.cpp
)

target_include_directories(dynalgo_model_backend_ort PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${CMAKE_CURRENT_SOURCE_DIR}/../../core
    ${ONNXRUNTIME_INCLUDE_DIRS}
)

target_link_libraries(dynalgo_model_backend_ort PUBLIC
    dynalgo::core
    dynalgo::model_backends_common
    ${ONNXRUNTIME_LIBRARIES}
)

target_compile_definitions(dynalgo_model_backend_ort PUBLIC ENABLE_ONNXRUNTIME)
target_compile_features(dynalgo_model_backend_ort PUBLIC cxx_std_17)

add_library(dynalgo::model_backend_ort ALIAS dynalgo_model_backend_ort)

install(FILES
    ort_backend.hpp
    DESTINATION include/dynalgo
)