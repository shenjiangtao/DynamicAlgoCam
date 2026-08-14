# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# dynalgo_capture — Capture/recording layer: encoding, file I/O, stream tasks,
# frame queues, session management, color conversion, SDL preview.
# Depends on: dynalgo_core, FFmpeg, SDL2.

cmake_minimum_required(VERSION 3.10)
project(dynalgo_capture)

find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(SWSCALE REQUIRED libswscale)
pkg_check_modules(SWRESAMPLE REQUIRED libswresample)
pkg_check_modules(SDL2 REQUIRED sdl2)

add_library(dynalgo_capture STATIC
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_h264_encoder.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_h264_encoder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_stream_io.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_stream_io.cpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_color_convert.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_color_convert.cpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_sdl_viewer.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_sdl_viewer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_stream_tasks.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_stream_tasks.cpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_capture_config.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_capture_session.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_capture_session.cpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_frame_queue.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_frame_queue.cpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_frame_consumer.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_frame_consumer.cpp
)

target_compile_options(dynalgo_capture PRIVATE
    -Wno-error=pedantic -Wno-pedantic
    -Wno-error=unused-parameter -Wno-unused-parameter
)

target_link_libraries(dynalgo_capture PUBLIC
    dynalgo::core
    ${AVCODEC_LIBRARIES}
    ${AVFORMAT_LIBRARIES}
    ${AVUTIL_LIBRARIES}
    ${SWSCALE_LIBRARIES}
    ${SWRESAMPLE_LIBRARIES}
    ${SDL2_LIBRARIES}
)

target_include_directories(dynalgo_capture PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${AVCODEC_INCLUDE_DIRS}
    ${AVFORMAT_INCLUDE_DIRS}
    ${AVUTIL_INCLUDE_DIRS}
    ${SWSCALE_INCLUDE_DIRS}
    ${SWRESAMPLE_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
)

target_link_directories(dynalgo_capture PUBLIC
    ${AVCODEC_LIBRARY_DIRS}
    ${AVFORMAT_LIBRARY_DIRS}
    ${AVUTIL_LIBRARY_DIRS}
    ${SWSCALE_LIBRARY_DIRS}
    ${SWRESAMPLE_LIBRARY_DIRS}
)

add_library(dynalgo::capture ALIAS dynalgo_capture)
set_target_properties(dynalgo_capture PROPERTIES FOLDER "app/capture")
