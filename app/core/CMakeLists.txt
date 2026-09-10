# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# dynalgo_core — SDK-neutral core types and abstract interfaces.
# No external SDK dependencies (no OrbbecSDK, no rs_driver).
# Provides: DynalgoFormat, DynalgoFrameType, DynalgoFrame, DynalgoDevice/Pipeline/Context,
#           logging, timestamps, thread utilities, signal handling.
#           HAL-based capture session, HAL factory, config manager.

cmake_minimum_required(VERSION 3.10)
project(dynalgo_core)

add_library(dynalgo_core STATIC
    ${CMAKE_CURRENT_LIST_DIR}/utils_c.c
    ${CMAKE_CURRENT_LIST_DIR}/utils_c.h
    ${CMAKE_CURRENT_LIST_DIR}/utils_types.h
    ${CMAKE_CURRENT_LIST_DIR}/utils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/utils.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_log.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_common.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_common.cpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_types.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_frame.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_frame.cpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_device.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_thread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_thread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_model.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_model_factory.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_model_factory.cpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_kalman_tracker.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_kalman_tracker.cpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_actuator.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_actuator_factory.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_actuator_factory.cpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_hal_factory.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_hal_factory.cpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_config_manager.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_config_manager.cpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_capture_session.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dynalgo_capture_session.cpp
    ${CMAKE_CURRENT_LIST_DIR}/heterogeneous_compute_hal.cpp
    ${CMAKE_CURRENT_LIST_DIR}/multimodal_fusion_hal.cpp
    ${CMAKE_CURRENT_LIST_DIR}/plugin_manager_hal.cpp
)

find_package(Threads REQUIRED)
find_package(yaml-cpp QUIET)

target_link_libraries(dynalgo_core PUBLIC
    Threads::Threads
    dynalgo_hal_common
    ${CMAKE_DL_LIBS}
)

if(yaml-cpp_FOUND)
    target_link_libraries(dynalgo_core PUBLIC yaml-cpp)
    target_compile_definitions(dynalgo_core PRIVATE DYNALGO_HAVE_YAML=1)
endif()

target_include_directories(dynalgo_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

add_library(dynalgo::core ALIAS dynalgo_core)
set_target_properties(dynalgo_core PROPERTIES FOLDER "app/core")
