cmake_minimum_required(VERSION 3.16)

# ${platform} ${type} ${vendor} HAL
project(dynalgo_hal_camera_orbbec LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Orbbec SDK - check build directory first, then system paths
set(ORBBEC_SDK_SEARCH_PATHS
    ${CMAKE_BINARY_DIR}/linux_x86_64/lib
    ${CMAKE_SOURCE_DIR}/../../../../vendors/OrbbecSDK/lib
    /usr/local/lib
    /opt/OrbbecSDK/lib
    /usr/lib/x86_64-linux-gnu
)

find_library(ORBBEC_SDK_LIB OrbbecSDK
    PATHS ${ORBBEC_SDK_SEARCH_PATHS}
    NO_DEFAULT_PATH
)

if(NOT ORBBEC_SDK_LIB)
    message(WARNING "Orbbec SDK not found, HAL will use dynamic loading only")
    # Create a dummy target so linking doesn't fail
    add_library(OrbbecSDK::ObSensor INTERFACE IMPORTED)
    set_target_properties(OrbbecSDK::ObSensor PROPERTIES
        INTERFACE_LINK_LIBRARIES "${CMAKE_DL_LIBS}"
    )
else()
    add_library(OrbbecSDK::ObSensor UNKNOWN IMPORTED)
    set_target_properties(OrbbecSDK::ObSensor PROPERTIES
        IMPORTED_LOCATION ${ORBBEC_SDK_LIB}
    )
endif()

add_library(dynalgo_hal_camera_orbbec SHARED
    orbbec_camera_hal.cpp
)

target_include_directories(dynalgo_hal_camera_orbbec PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../../../../include
)

target_link_libraries(dynalgo_hal_camera_orbbec PRIVATE
    dynalgo_hal_common
    ${CMAKE_DL_LIBS}
    OrbbecSDK::ObSensor
)

target_compile_definitions(dynalgo_hal_camera_orbbec PRIVATE
    DYNALGO_HAL_VERSION_MAJOR=1
    DYNALGO_HAL_VERSION_MINOR=0
)

set_target_properties(dynalgo_hal_camera_orbbec PROPERTIES
    VERSION ${DYNALGO_VERSION}
    SOVERSION 1
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

install(TARGETS dynalgo_hal_camera_orbbec
        LIBRARY DESTINATION lib/dynalgo/hal
        RUNTIME DESTINATION bin)
