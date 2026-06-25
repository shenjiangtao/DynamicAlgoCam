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

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_DEFAULT_SOURCE")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_DEFAULT_SOURCE")

set(OB_BUILD_MAIN_PROJECT ON CACHE BOOL "OrbbecSDK is the main SDK project" FORCE)
set(OB_SDK_LIB_NAME "OrbbecSDK" CACHE STRING "OrbbecSDK library name" FORCE)

add_subdirectory(vendors/OrbbecSDK)

add_subdirectory(app)

