cmake_minimum_required(VERSION 3.16)

# ${platform} ${type} ${vendor} HAL
project(dynalgo_hal_sensor_robosense_lidar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find RoboSense SDK
set(ROBOSENSE_SDK_SEARCH_PATHS
    ${CMAKE_SOURCE_DIR}/../../../../vendors/RoboSense
    /usr/local/lib
    /opt/RoboSense/lib
    /usr/lib/x86_64-linux-gnu
)

find_library(ROBOSENSE_SDK_LIB rs_driver
    PATHS ${ROBOSENSE_SDK_SEARCH_PATHS}
    NO_DEFAULT_PATH
)

if(NOT ROBOSENSE_SDK_LIB)
    message(WARNING "RoboSense SDK not found, HAL will use dynamic loading only")
    add_library(RoboSense::rs_driver INTERFACE IMPORTED)
    set_target_properties(RoboSense::rs_driver PROPERTIES
        INTERFACE_LINK_LIBRARIES "${CMAKE_DL_LIBS}"
    )
else()
    add_library(RoboSense::rs_driver UNKNOWN IMPORTED)
    set_target_properties(RoboSense::rs_driver PROPERTIES
        IMPORTED_LOCATION ${ROBOSENSE_SDK_LIB}
    )
endif()

add_library(dynalgo_hal_sensor_robosense_lidar SHARED
    robosense_lidar_hal.cpp
)

target_include_directories(dynalgo_hal_sensor_robosense_lidar PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../../../../include
)

target_link_libraries(dynalgo_hal_sensor_robosense_lidar PRIVATE
    dynalgo_hal_common
    ${CMAKE_DL_LIBS}
    RoboSense::rs_driver
)

target_compile_definitions(dynalgo_hal_sensor_robosense_lidar PRIVATE
    DYNALGO_HAL_VERSION_MAJOR=1
    DYNALGO_HAL_VERSION_MINOR=0
)

set_target_properties(dynalgo_hal_sensor_robosense_lidar PROPERTIES
    VERSION ${DYNALGO_VERSION}
    SOVERSION 1
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

install(TARGETS dynalgo_hal_sensor_robosense_lidar
        LIBRARY DESTINATION lib/dynalgo/hal
        RUNTIME DESTINATION bin)