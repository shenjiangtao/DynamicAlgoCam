# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# dynalgo_model_backend_rknn — RKNN Runtime C++ inference backend

cmake_minimum_required(VERSION 3.16)

project(dynalgo_model_backend_rknn LANGUAGES CXX)

if(NOT ENABLE_RKNN)
    message(WARNING "ENABLE_RKNN=OFF, skipping RKNN backend")
    return()
endif()

# RKNN library typically installed at /usr/lib or via Rockchip SDK
find_library(RKNN_LIBRARY rknnrt PATHS /usr/lib /usr/local/lib /opt/rockchip/rknn-toolkit2)
if(NOT RKNN_LIBRARY)
    message(WARNING "RKNN library not found, RKNN backend will be stub only")
endif()

add_library(dynalgo_model_backend_rknn STATIC
    rknn_backend.cpp
)

target_include_directories(dynalgo_model_backend_rknn PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${CMAKE_CURRENT_SOURCE_DIR}/../../core
)

target_link_libraries(dynalgo_model_backend_rnt PUBLIC
    dynalgo::core
    dynalgo::model_backends_common
    ${RKNN_LIBRARY}
    dl
)

target_compile_definitions(dynalgo_model_backend_rknn PUBLIC ENABLE_RKNN)
target_compile_features(dynalgo_model_backend_rknn PUBLIC cxx_std_17)

add_library(dynalgo::model_backend_rknn ALIAS dynalgo_model_backend_rknn)

install(FILES
    rknn_backend.hpp
    DESTINATION include/dynalgo
)