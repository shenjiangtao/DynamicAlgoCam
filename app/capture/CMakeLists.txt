# Copyright (c) NIO Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# nio_capture — Capture/recording layer: encoding, file I/O, stream tasks,
# frame queues, session management, color conversion, SDL preview.
# Depends on: nio_core, FFmpeg, SDL2.

cmake_minimum_required(VERSION 3.10)
project(nio_capture)

find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(SWSCALE REQUIRED libswscale)
pkg_check_modules(SWRESAMPLE REQUIRED libswresample)
pkg_check_modules(SDL2 REQUIRED sdl2)

add_library(nio_capture STATIC
    ${CMAKE_CURRENT_LIST_DIR}/nio_h264_encoder.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_h264_encoder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_stream_io.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_stream_io.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_color_convert.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_color_convert.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_sdl_viewer.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_sdl_viewer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_stream_tasks.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_stream_tasks.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_capture_config.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_capture_session.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_capture_session.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_frame_queue.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_frame_queue.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_frame_consumer.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_frame_consumer.cpp
)

target_compile_options(nio_capture PRIVATE
    -Wno-error=pedantic -Wno-pedantic
    -Wno-error=unused-parameter -Wno-unused-parameter
)

target_link_libraries(nio_capture PUBLIC
    nio::core
    ${AVCODEC_LIBRARIES}
    ${AVFORMAT_LIBRARIES}
    ${AVUTIL_LIBRARIES}
    ${SWSCALE_LIBRARIES}
    ${SWRESAMPLE_LIBRARIES}
    ${SDL2_LIBRARIES}
)

target_include_directories(nio_capture PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${AVCODEC_INCLUDE_DIRS}
    ${AVFORMAT_INCLUDE_DIRS}
    ${AVUTIL_INCLUDE_DIRS}
    ${SWSCALE_INCLUDE_DIRS}
    ${SWRESAMPLE_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
)

target_link_directories(nio_capture PUBLIC
    ${AVCODEC_LIBRARY_DIRS}
    ${AVFORMAT_LIBRARY_DIRS}
    ${AVUTIL_LIBRARY_DIRS}
    ${SWSCALE_LIBRARY_DIRS}
    ${SWRESAMPLE_LIBRARY_DIRS}
)

add_library(nio::capture ALIAS nio_capture)
set_target_properties(nio_capture PROPERTIES FOLDER "app/capture")
