# Copyright (c) NIO Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# nio_multi_capture — Multi-device capture application.
# Standalone application entry point (main).
# Links against the layered libraries: nio_core, nio_drivers, nio_capture.

cmake_minimum_required(VERSION 3.10)
project(nio_multi_capture)

add_executable(${PROJECT_NAME} nio_multi_capture.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
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

# rs_driver vendored headers trigger pedantic warnings that become errors.
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wno-error=pedantic -Wno-pedantic
    -Wno-error=unused-parameter -Wno-unused-parameter
)

target_link_libraries(${PROJECT_NAME}
    nio::core
    nio::drivers
    nio::capture
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    FOLDER "app"
    INSTALL_RPATH "$ORIGIN/../lib"
    INSTALL_RPATH_USE_LINK_PATH FALSE
    BUILD_WITH_INSTALL_RPATH TRUE
)
if(MSVC)
    set_target_properties(${PROJECT_NAME} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
