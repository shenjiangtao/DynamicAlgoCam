cmake_minimum_required(VERSION 3.16)

# aarch64_nvidia NvSIPL Camera HAL
project(dynalgo_hal_camera_nvsipl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(dynalgo_hal_camera_nvsipl SHARED
    nvsipl_camera_hal.cpp
    nvsipl_platform_cfg.cpp
)

target_include_directories(dynalgo_hal_camera_nvsipl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../../../../include
    ${NVSIPL_INCLUDE_DIRS}
    ${NVMEDIA_INCLUDE_DIRS}
    ${NVSCIBUF_INCLUDE_DIRS}
    ${NVSCISYNC_INCLUDE_DIRS}
    ${NVSCISTREAM_INCLUDE_DIRS}
)

target_link_libraries(dynalgo_hal_camera_nvsipl PRIVATE
    dynalgo_hal_common
    ${NVSIPL_LIB}
    ${NVMEDIA_LIB}
    ${NVSCIBUF_LIB}
    ${NVSCISYNC_LIB}
    ${NVSCISTREAM_LIB}
    ${NVSCIEVENT_LIB}
    ${NVMEDIA_2D_LIB}
    ${TEGRA_WFD_LIB}
    ${CUDLA_LIB}
)

target_compile_definitions(dynalgo_hal_camera_nvsipl PRIVATE
    DYNALGO_HAL_VERSION_MAJOR=1
    DYNALGO_HAL_VERSION_MINOR=0
    DYNALGO_HAVE_NVSIPL=1
    DYNALGO_HAVE_NVMEDIA=1
    DYNALGO_HAVE_NVSCIBUF=1
    DYNALGO_HAVE_NVSCISYNC=1
    DYNALGO_HAVE_NVSCISTREAM=1
)

set_target_properties(dynalgo_hal_camera_nvsipl PROPERTIES
    VERSION ${DYNALGO_VERSION}
    SOVERSION 1
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

install(TARGETS dynalgo_hal_camera_nvsipl
        LIBRARY DESTINATION lib/dynalgo/hal
        RUNTIME DESTINATION bin)
