# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# dynalgo_driver_stereo — Hardware-sync stereo camera drivers

cmake_minimum_required(VERSION 3.16)

project(dynalgo_driver_stereo LANGUAGES CXX)

option(ENABLE_STEREO_UVC "Enable generic UVC stereo camera" ON)
option(ENABLE_ZED "Enable Stereolabs ZED SDK" OFF)
option(ENABLE_MYNT_EYE "Enable MYNT EYE SDK" OFF)
option(ENABLE_DEPTHAI "Enable Luxonis DepthAI/OAK" OFF)

if(ENABLE_STEREO_UVC)
    add_subdirectory(uvc)
endif()

if(ENABLE_ZED)
    find_package(ZED REQUIRED)
    add_subdirectory(zed)
endif()

if(ENABLE_MYNT_EYE)
    find_package(MYNT_EYE REQUIRED)
    add_subdirectory(mynt_eye)
endif()

if(ENABLE_DEPTHAI)
    find_package(DepthAI REQUIRED)
    add_subdirectory(depthai)
endif()

# StereoRectifier requires OpenCV
find_package(OpenCV QUIET COMPONENTS core imgproc calib3d)

# Meta-library
if(ENABLE_STEREO_UVC OR ENABLE_ZED OR ENABLE_MYNT_EYE OR ENABLE_DEPTHAI)
    if(OpenCV_FOUND)
        add_library(dynalgo_driver_stereo STATIC
            driver_stereo_dummy.cpp
            stereo_rectifier.cpp
        )

        target_include_directories(dynalgo_driver_stereo PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/../../core
            ${OpenCV_INCLUDE_DIRS}
        )

        target_compile_definitions(dynalgo_driver_stereo PUBLIC DYNALOGO_HAVE_OPENCV)
        target_link_libraries(dynalgo_driver_stereo PUBLIC
            dynalgo::core
            ${OpenCV_LIBS}
        )
    else()
        # OpenCV not found - build without stereo_rectifier
        add_library(dynalgo_driver_stereo STATIC
            driver_stereo_dummy.cpp
        )

        target_include_directories(dynalgo_driver_stereo PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/../../core
        )

        target_link_libraries(dynalgo_driver_stereo PUBLIC
            dynalgo::core
        )
    endif()

    if(ENABLE_STEREO_UVC)
        target_link_libraries(dynalgo_driver_stereo PUBLIC dynalgo::driver_stereo_uvc)
    endif()

    target_compile_features(dynalgo_driver_stereo PUBLIC cxx_std_17)

    add_library(dynalgo::driver_stereo ALIAS dynalgo_driver_stereo)
else()
    # Create interface library when no backends enabled
    add_library(dynalgo_driver_stereo INTERFACE)
    target_include_directories(dynalgo_driver_stereo INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../../core
    )
    target_link_libraries(dynalgo_driver_stereo INTERFACE
        dynalgo::core
    )
    add_library(dynalgo::driver_stereo ALIAS dynalgo_driver_stereo)
endif()

install(FILES
    stereo_adapter.hpp
    DESTINATION include/dynalgo
)