# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# dynalgo_opencv_plugin — Optional OpenCV-based color conversion plugin.
# Only built when OpenCV is found. Linked into dynalgo_capture if available.
# Depends on: dynalgo_core, dynalgo_capture, OpenCV.

cmake_minimum_required(VERSION 3.10)
project(dynalgo_opencv_plugin)

if(OpenCV_FOUND)
    add_library(dynalgo_opencv_plugin STATIC
        ${CMAKE_CURRENT_LIST_DIR}/utils_opencv.cpp
        ${CMAKE_CURRENT_LIST_DIR}/utils_opencv.hpp
        ${CMAKE_CURRENT_LIST_DIR}/dynalgo_color_convert_cv.hpp
        ${CMAKE_CURRENT_LIST_DIR}/dynalgo_color_convert_cv.cpp
    )

    target_link_libraries(dynalgo_opencv_plugin PUBLIC
        dynalgo::core
        dynalgo::capture
        ${OpenCV_LIBS}
    )

    target_include_directories(dynalgo_opencv_plugin PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
    )

    add_library(dynalgo::opencv_plugin ALIAS dynalgo_opencv_plugin)
    set_target_properties(dynalgo_opencv_plugin PROPERTIES FOLDER "app/plugins")
    message(STATUS "OpenCV plugin enabled for dynalgo_capture")
else()
    message(STATUS "OpenCV not found — opencv plugin disabled")
endif()
