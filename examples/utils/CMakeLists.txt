# Copyright (c) Orbbec Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# ob_examples_utils — Static library of shared utilities for NIO examples.
# Provides: signal handling, timestamps, H.264 encoding, stream I/O,
# color conversion (with optional OpenCV support), and logging.
# FFmpeg and OpenCV deps are linked PUBLIC so consumers inherit them.

cmake_minimum_required(VERSION 3.10)
project(ob_examples_utils)

find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(SWSCALE REQUIRED libswscale)
pkg_check_modules(SWRESAMPLE REQUIRED libswresample)

add_library(ob_examples_utils STATIC
${CMAKE_CURRENT_LIST_DIR}/utils_c.c
${CMAKE_CURRENT_LIST_DIR}/utils_c.h
${CMAKE_CURRENT_LIST_DIR}/utils.cpp
${CMAKE_CURRENT_LIST_DIR}/utils.hpp
${CMAKE_CURRENT_LIST_DIR}/nio_log.hpp
${CMAKE_CURRENT_LIST_DIR}/nio_common.hpp
${CMAKE_CURRENT_LIST_DIR}/nio_common.cpp
${CMAKE_CURRENT_LIST_DIR}/nio_h264_encoder.hpp
${CMAKE_CURRENT_LIST_DIR}/nio_h264_encoder.cpp
${CMAKE_CURRENT_LIST_DIR}/nio_stream_io.hpp
${CMAKE_CURRENT_LIST_DIR}/nio_stream_io.cpp
${CMAKE_CURRENT_LIST_DIR}/nio_color_convert.hpp
${CMAKE_CURRENT_LIST_DIR}/nio_color_convert.cpp
)

target_link_libraries(ob_examples_utils PUBLIC
ob::${OB_SDK_LIB_NAME}
${AVCODEC_LIBRARIES}
${AVFORMAT_LIBRARIES}
${AVUTIL_LIBRARIES}
${SWSCALE_LIBRARIES}
${SWRESAMPLE_LIBRARIES}
)

target_include_directories(ob_examples_utils PUBLIC
${AVCODEC_INCLUDE_DIRS}
${AVFORMAT_INCLUDE_DIRS}
${AVUTIL_INCLUDE_DIRS}
${SWSCALE_INCLUDE_DIRS}
${SWRESAMPLE_INCLUDE_DIRS}
)

target_link_directories(ob_examples_utils PUBLIC
${AVCODEC_LIBRARY_DIRS}
${AVFORMAT_LIBRARY_DIRS}
${AVUTIL_LIBRARY_DIRS}
${SWSCALE_LIBRARY_DIRS}
${SWRESAMPLE_LIBRARY_DIRS}
)

if(${OpenCV_FOUND})
target_sources(ob_examples_utils PRIVATE
${CMAKE_CURRENT_LIST_DIR}/utils_opencv.cpp
${CMAKE_CURRENT_LIST_DIR}/utils_opencv.hpp
${CMAKE_CURRENT_LIST_DIR}/nio_color_convert_cv.hpp
${CMAKE_CURRENT_LIST_DIR}/nio_color_convert_cv.cpp
)
target_link_libraries(ob_examples_utils PUBLIC ${OpenCV_LIBS})
target_include_directories(ob_examples_utils PUBLIC ${OpenCV_INCLUDE_DIRS})
endif()

find_package(Threads REQUIRED)
target_link_libraries(ob_examples_utils PUBLIC Threads::Threads)

target_include_directories(ob_examples_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(ob::examples::utils ALIAS ob_examples_utils)
set_target_properties(ob_examples_utils PROPERTIES FOLDER "examples")
