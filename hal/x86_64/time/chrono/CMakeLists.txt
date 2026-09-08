cmake_minimum_required(VERSION 3.16)

project(dynalgo_hal_time_chrono LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(dynalgo_hal_time_chrono SHARED
    chrono_time_hal.cpp
)

target_include_directories(dynalgo_hal_time_chrono PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../../../../include
)

target_link_libraries(dynalgo_hal_time_chrono PRIVATE
    dynalgo_hal_common
)

target_compile_definitions(dynalgo_hal_time_chrono PRIVATE
    DYNALGO_HAL_VERSION_MAJOR=1
    DYNALGO_HAL_VERSION_MINOR=0
)

set_target_properties(dynalgo_hal_time_chrono PROPERTIES
    VERSION ${DYNALGO_VERSION}
    SOVERSION 1
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

install(TARGETS dynalgo_hal_time_chrono
        LIBRARY DESTINATION lib/dynalgo/hal
        RUNTIME DESTINATION bin)
