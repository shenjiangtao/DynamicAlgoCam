# Copyright (c) NIO Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# nio_actuators — external actuator adapters (laser, gimbal, dummy).
# Concrete backends inherit NioActuator (defined in nio_core) and self-register
# via registerActuator() at static-init time. This static lib must be linked
# by any binary that wants the actuator types compiled in.
#
# No vendor SDK dependencies here — real-actuator backends bring their own.

cmake_minimum_required(VERSION 3.10)
project(nio_actuators)

add_library(nio_actuators STATIC
    ${CMAKE_CURRENT_LIST_DIR}/dummy_actuator.hpp
    ${CMAKE_CURRENT_LIST_DIR}/dummy_actuator.cpp
)

target_link_libraries(nio_actuators PUBLIC nio::core)

target_include_directories(nio_actuators PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(nio::actuators ALIAS nio_actuators)
set_target_properties(nio_actuators PROPERTIES FOLDER "app/actuator")
