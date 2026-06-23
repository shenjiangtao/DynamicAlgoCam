# Copyright (c) NIO Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# nio_core — SDK-neutral core types and abstract interfaces.
# No external SDK dependencies (no OrbbecSDK, no rs_driver).
# Provides: NioFormat, NioFrameType, NioFrame, NioDevice/Pipeline/Context,
#           logging, timestamps, thread utilities, signal handling.

cmake_minimum_required(VERSION 3.10)
project(nio_core)

add_library(nio_core STATIC
    ${CMAKE_CURRENT_LIST_DIR}/utils_c.c
    ${CMAKE_CURRENT_LIST_DIR}/utils_c.h
    ${CMAKE_CURRENT_LIST_DIR}/utils_types.h
    ${CMAKE_CURRENT_LIST_DIR}/utils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/utils.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_log.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_common.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_common.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_types.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_frame.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_frame.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_device.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_thread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_thread.cpp
)

find_package(Threads REQUIRED)
target_link_libraries(nio_core PUBLIC Threads::Threads)

# Forward optional driver compile definitions to nio_core so that
# #ifdef ENABLE_ORBBEC / ENABLE_RS_AC1 guards in core headers/sources work.
if(ENABLE_ORBBEC)
    target_compile_definitions(nio_core PUBLIC ENABLE_ORBBEC)
endif()
if(ENABLE_RS_AC1)
    target_compile_definitions(nio_core PUBLIC ENABLE_RS_AC1)
endif()

target_include_directories(nio_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(nio::core ALIAS nio_core)
set_target_properties(nio_core PROPERTIES FOLDER "app/core")
