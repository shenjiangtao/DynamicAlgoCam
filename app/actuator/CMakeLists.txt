# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# dynalgo_actuators — external actuator adapters (laser, gimbal, dummy).
# Concrete backends inherit DynalgoActuator (defined in dynalgo_core) and self-register
# via registerActuator() at static-init time. This static lib must be linked
# by any binary that wants the actuator types compiled in.
#
# No vendor SDK dependencies here — real-actuator backends bring their own.

cmake_minimum_required(VERSION 3.10)
project(dynalgo_actuators)

add_library(dynalgo_actuators STATIC
    ${CMAKE_CURRENT_LIST_DIR}/dummy_actuator.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dummy_actuator.cpp
)

target_link_libraries(dynalgo_actuators PUBLIC dynalgo::core)

target_include_directories(dynalgo_actuators PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(dynalgo::actuators ALIAS dynalgo_actuators)
set_target_properties(dynalgo_actuators PROPERTIES FOLDER "app/actuator")
