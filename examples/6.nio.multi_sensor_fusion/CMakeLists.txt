# Copyright (c) NIO Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# nio_multi_sensor_fusion — Multi-sensor fusion with IMU integration.
# FFmpeg deps are inherited via PUBLIC link from ob_examples_utils.

cmake_minimum_required(VERSION 3.10)
project(nio_multi_sensor_fusion)

add_executable(${PROJECT_NAME} nio_multi_sensor_fusion.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

target_link_libraries(${PROJECT_NAME}
ob::${OB_SDK_LIB_NAME}
ob::examples::utils
)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "examples")
if(MSVC)
set_target_properties(${PROJECT_NAME} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
