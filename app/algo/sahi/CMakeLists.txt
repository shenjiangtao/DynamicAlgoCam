# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# SAHI (Slicing Aided Hyper Inference) C++ implementation

cmake_minimum_required(VERSION 3.16)

project(dynalgo_sahi LANGUAGES CXX)

add_library(dynalgo_sahi STATIC
    sahi.cpp
)

target_include_directories(dynalgo_sahi PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../core
)

target_link_libraries(dynalgo_sahi PUBLIC
    dynalgo::core
)

target_compile_features(dynalgo_sahi PUBLIC cxx_std_17)

add_library(dynalgo::sahi ALIAS dynalgo_sahi)

install(FILES
    sahi.hpp
    DESTINATION include/dynalgo
)