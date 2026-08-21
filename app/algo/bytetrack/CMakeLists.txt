# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# ByteTrack C++ multi-object tracker

cmake_minimum_required(VERSION 3.16)

project(dynalgo_bytetrack LANGUAGES CXX)

add_library(dynalgo_bytetrack STATIC
    bytetrack.cpp
)

target_include_directories(dynalgo_bytetrack PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../core
)

target_link_libraries(dynalgo_bytetrack PUBLIC
    dynalgo::core
)

target_compile_features(dynalgo_bytetrack PUBLIC cxx_std_17)

add_library(dynalgo::bytetrack ALIAS dynalgo_bytetrack)

install(FILES
    bytetrack.hpp
    DESTINATION include/dynalgo
)