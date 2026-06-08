# Copyright (c) NIO Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# nio_multi_capture — Multi-device capture example.
# FFmpeg deps are inherited via PUBLIC link from ob_examples_utils.

cmake_minimum_required(VERSION 3.10)
project(nio_multi_capture)

add_executable(${PROJECT_NAME} nio_multi_capture.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE GIT_COMMIT_RESULT
)
if(GIT_COMMIT_RESULT EQUAL 0 AND GIT_COMMIT_SHORT)
    target_compile_definitions(${PROJECT_NAME} PRIVATE GIT_COMMIT_HASH="${GIT_COMMIT_SHORT}")
else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE GIT_COMMIT_HASH="unknown")
endif()

target_link_libraries(${PROJECT_NAME}
ob::${OB_SDK_LIB_NAME}
ob::examples::utils
)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "examples")
if(MSVC)
set_target_properties(${PROJECT_NAME} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
