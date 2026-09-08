cmake_minimum_required(VERSION 3.16)

# ${platform} ${type} ${vendor} HAL
project(dynalgo_hal_display_sdl2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(dynalgo_hal_display_sdl2 SHARED
    sdl2_display_hal.cpp
)

target_include_directories(dynalgo_hal_display_sdl2 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../../../../include
)

target_link_libraries(dynalgo_hal_display_sdl2 PRIVATE
    dynalgo_hal_common
)

target_compile_definitions(dynalgo_hal_display_sdl2 PRIVATE
    DYNALGO_HAL_VERSION_MAJOR=1
    DYNALGO_HAL_VERSION_MINOR=0
)

set_target_properties(dynalgo_hal_display_sdl2 PROPERTIES
    VERSION ${DYNALGO_VERSION}
    SOVERSION 1
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

install(TARGETS dynalgo_hal_display_sdl2
        LIBRARY DESTINATION lib/dynalgo/hal
        RUNTIME DESTINATION bin)
