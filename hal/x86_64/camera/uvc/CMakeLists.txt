cmake_minimum_required(VERSION 3.16)

# x86_64 UVC Camera HAL
project(dynalgo_hal_camera_uvc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(dynalgo_hal_camera_uvc SHARED
    uvc_camera_hal.cpp
)

target_include_directories(dynalgo_hal_camera_uvc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../../../../include
)

target_link_libraries(dynalgo_hal_camera_uvc PRIVATE
    dynalgo_hal_common
)

if(V4L2_LIB)
    target_link_libraries(dynalgo_hal_camera_uvc PRIVATE ${V4L2_LIB})
else()
    target_compile_definitions(dynalgo_hal_camera_uvc PRIVATE DYNALGO_NO_V4L2=1)
endif()

if(V4L2CONVERT_LIB)
    target_link_libraries(dynalgo_hal_camera_uvc PRIVATE ${V4L2CONVERT_LIB})
endif()

target_link_libraries(dynalgo_hal_camera_uvc PRIVATE
    ${CMAKE_THREAD_LIBS_INIT}
    ${DL_LIB}
    ${RT_LIB}
)

target_compile_definitions(dynalgo_hal_camera_uvc PRIVATE
    DYNALGO_HAL_VERSION_MAJOR=1
    DYNALGO_HAL_VERSION_MINOR=0
)

set_target_properties(dynalgo_hal_camera_uvc PROPERTIES
    VERSION ${DYNALGO_VERSION}
    SOVERSION 1
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

install(TARGETS dynalgo_hal_camera_uvc
        LIBRARY DESTINATION lib/dynalgo/hal
        RUNTIME DESTINATION bin)