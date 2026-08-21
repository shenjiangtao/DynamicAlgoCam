# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# dynalgo_model_backends_common — Shared preprocessing, NMS, postprocessing

cmake_minimum_required(VERSION 3.16)

project(dynalgo_model_backends_common LANGUAGES CXX)

add_library(dynalgo_model_backends_common STATIC
    preprocessing.cpp
)

target_include_directories(dynalgo_model_backends_common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../core
)

target_link_libraries(dynalgo_model_backends_common PUBLIC
    dynalgo::core
)

# Optional CUDA acceleration for preprocessing/NMS
option(ENABLE_CUDA "Enable CUDA acceleration for preprocessing" OFF)
if(ENABLE_CUDA)
    find_package(CUDA REQUIRED)
    target_compile_definitions(dynalgo_model_backends_common PUBLIC ENABLE_CUDA)
    target_link_libraries(dynalgo_model_backends_common PUBLIC ${CUDA_LIBRARIES})
endif()

target_compile_features(dynalgo_model_backends_common PUBLIC cxx_std_17)

add_library(dynalgo::model_backends_common ALIAS dynalgo_model_backends_common)

install(FILES
    preprocessing.hpp
    DESTINATION include/dynalgo
)