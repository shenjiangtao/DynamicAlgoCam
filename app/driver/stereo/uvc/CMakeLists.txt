# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# dynalgo_driver_stereo_uvc — Generic UVC stereo camera driver

cmake_minimum_required(VERSION 3.16)

project(dynalgo_driver_stereo_uvc LANGUAGES CXX)

add_library(dynalgo_driver_stereo_uvc STATIC
    ../stereo_uvc.cpp
)

target_include_directories(dynalgo_driver_stereo_uvc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../core
)

target_link_libraries(dynalgo_driver_stereo_uvc PUBLIC
    dynalgo::core
)

target_compile_features(dynalgo_driver_stereo_uvc PUBLIC cxx_std_17)

add_library(dynalgo::driver_stereo_uvc ALIAS dynalgo_driver_stereo_uvc)