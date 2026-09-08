cmake_minimum_required(VERSION 3.16)

project(dynalgo_hal_logger_spdlog LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(dynalgo_hal_logger_spdlog SHARED
    spdlog_logger_hal.cpp
)

target_include_directories(dynalgo_hal_logger_spdlog PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../../../../include
)

find_path(SPDLOG_INCLUDE_DIR spdlog/spdlog.h
    PATHS /usr/include /usr/local/include /opt/homebrew/include)
find_library(SPDLOG_LIB spdlog
    PATHS /usr/lib /usr/local/lib /opt/homebrew/lib)

target_link_libraries(dynalgo_hal_logger_spdlog PRIVATE
    dynalgo_hal_common
)

if(SPDLOG_LIB)
    target_link_libraries(dynalgo_hal_logger_spdlog PRIVATE ${SPDLOG_LIB})
else()
    target_compile_definitions(dynalgo_hal_logger_spdlog PRIVATE DYNALGO_NO_SPDLOG=1)
endif()

target_compile_definitions(dynalgo_hal_logger_spdlog PRIVATE
    DYNALGO_HAL_VERSION_MAJOR=1
    DYNALGO_HAL_VERSION_MINOR=0
)

set_target_properties(dynalgo_hal_logger_spdlog PROPERTIES
    VERSION ${DYNALGO_VERSION}
    SOVERSION 1
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

install(TARGETS dynalgo_hal_logger_spdlog
        LIBRARY DESTINATION lib/dynalgo/hal
        RUNTIME DESTINATION bin)
