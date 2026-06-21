# Copyright (c) NIO Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# nio_opencv_plugin — Optional OpenCV-based color conversion plugin.
# Only built when OpenCV is found. Linked into nio_capture if available.
# Depends on: nio_core, nio_capture, OpenCV.

cmake_minimum_required(VERSION 3.10)
project(nio_opencv_plugin)

if(OpenCV_FOUND)
    add_library(nio_opencv_plugin STATIC
        ${CMAKE_CURRENT_LIST_DIR}/utils_opencv.cpp
        ${CMAKE_CURRENT_LIST_DIR}/utils_opencv.hpp
        ${CMAKE_CURRENT_LIST_DIR}/nio_color_convert_cv.hpp
        ${CMAKE_CURRENT_LIST_DIR}/nio_color_convert_cv.cpp
    )

    target_link_libraries(nio_opencv_plugin PUBLIC
        nio::core
        nio::capture
        ${OpenCV_LIBS}
    )

    target_include_directories(nio_opencv_plugin PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
    )

    add_library(nio::opencv_plugin ALIAS nio_opencv_plugin)
    set_target_properties(nio_opencv_plugin PROPERTIES FOLDER "app/plugins")
    message(STATUS "OpenCV plugin enabled for nio_capture")
else()
    message(STATUS "OpenCV not found — opencv plugin disabled")
endif()
