# Copyright (c) NIO Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# NIO multi-device capture project — root entry point.
#
# Directory layout:
#   vendors/OrbbecSDK/   — Orbbec depth camera SDK (vendored copy)
#   vendors/RoboSense/   — RoboSense LiDAR driver (vendored copy)
#   app/                  — NIO application layer (core / driver / capture / ...)
#   docs/                 — NIO-specific documentation
#   scripts/              — NIO-specific scripts

cmake_minimum_required(VERSION 3.10)

project(NioCapture VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(NIO_PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

option(ENABLE_ORBBEC "Enable Orbbec device support" ON)
option(ENABLE_RS_AC1 "Enable RoboSense RS-AC1 support" ON)

# Default to Release if user didn't specify — capture tool should ship
# optimized. Developers wanting debug symbols pass -DCMAKE_BUILD_TYPE=Debug.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    message(STATUS "NIO: CMAKE_BUILD_TYPE not set — defaulting to Release")
endif()

# Release/RelWithDebInfo: replace CMake's default -O2 with -O3, fold in
# -march=native + -DNDEBUG (the latter is already added by CMake for
# Release, but explicit is better when overriding -O level). Bounded to
# GCC/Clang — MSVC uses /O2 by default and is left alone.
set(NIO_RELEASE_FLAGS "")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(NIO_RELEASE_FLAGS "-O3;-march=native;-DNDEBUG")
endif()
foreach(_cfg RELEASE RELWITHDEBINFO)
    foreach(_lang C CXX)
        foreach(_flag IN LISTS NIO_RELEASE_FLAGS)
            add_compile_options("$<$<AND:$<CONFIG:${_cfg}>,$<COMPILE_LANGUAGE:${_lang}>>:${_flag}>")
        endforeach()
    endforeach()
endforeach()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_DEFAULT_SOURCE")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_DEFAULT_SOURCE")

if(ENABLE_ORBBEC)
    set(OB_BUILD_MAIN_PROJECT ON CACHE BOOL "OrbbecSDK is the main SDK project" FORCE)
    set(OB_SDK_LIB_NAME "OrbbecSDK" CACHE STRING "OrbbecSDK library name" FORCE)
    add_subdirectory(vendors/OrbbecSDK)
else()
    message(STATUS "Orbbec support disabled — skipping OrbbecSDK")
endif()

if(ENABLE_RS_AC1)
    set(RS_DRIVER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vendors/RoboSense")
    set(DISABLE_PCAP_PARSE ON CACHE BOOL "Disable PCAP file parse" FORCE)
    set(COMPILE_DEMOS OFF CACHE BOOL "Build rs_driver demos" FORCE)
    set(COMPILE_TOOLS OFF CACHE BOOL "Build rs_driver tools" FORCE)
    set(ENABLE_USB ON CACHE BOOL "Enable USB interface in rs_driver" FORCE)

    set(_SAVED_CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    set(_SAVED_CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REPLACE "-pedantic" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    string(REPLACE "-pedantic" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    add_subdirectory(vendors/RoboSense ${CMAKE_BINARY_DIR}/rs_driver)
    set(CMAKE_C_FLAGS "${_SAVED_CMAKE_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${_SAVED_CMAKE_CXX_FLAGS}")
else()
    message(STATUS "RS-AC1 support disabled — skipping RoboSense")
endif()

if(NOT ENABLE_ORBBEC AND NOT ENABLE_RS_AC1)
    message(FATAL_ERROR "Both ENABLE_ORBBEC and ENABLE_RS_AC1 are OFF — no vendor SDK selected, aborting")
endif()

add_subdirectory(app)

