# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# dynalgo_algo static library — algorithm orchestration (EngagementLoop, TargetSelector,
# TrackBundle, DummyModelBackend). Links dynalgo_core + dynalgo_actuators.

cmake_minimum_required(VERSION 3.16)

project(dynalgo_algo LANGUAGES CXX)

add_library(dynalgo_algo STATIC
    dynalgo_engagement_loop.cpp
    dynalgo_engagement_consumer.cpp
    dynalgo_target_selector.cpp
    dynalgo_track_bundle.cpp
    dummy_model_backend.cpp
)

target_include_directories(dynalgo_algo PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../core
)

target_link_libraries(dynalgo_algo PUBLIC
    dynalgo::core
    dynalgo::actuators
)

# C++17 required
target_compile_features(dynalgo_algo PUBLIC cxx_std_17)

# Alias for downstream consumers
add_library(dynalgo::algo ALIAS dynalgo_algo)

# Install headers
install(FILES
    dynalgo_engagement_loop.hpp
    dynalgo_engagement_consumer.hpp
    dynalgo_target_selector.hpp
    dynalgo_track_bundle.hpp
    dummy_model_backend.hpp
    DESTINATION include/dynalgo
)