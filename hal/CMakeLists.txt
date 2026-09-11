cmake_minimum_required(VERSION 3.16)

project(DynalgoHAL VERSION ${DYNALGO_VERSION} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(HAL_VERSION_MAJOR 1)
set(HAL_VERSION_MINOR 0)
set(HAL_VERSION_PATCH 0)

# Platform detection (inherited from parent)
if(NOT DYNALGO_PLATFORM)
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64")
        set(DYNALGO_PLATFORM "x86_64")
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
        if(EXISTS "/etc/nv_tegra_release" OR EXISTS "/proc/device-tree/compatible")
            set(DYNALGO_PLATFORM "aarch64_nvidia")
        else()
            set(DYNALGO_PLATFORM "aarch64_generic")
        endif()
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "mips")
        set(DYNALGO_PLATFORM "mips64_loongson")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "riscv")
        set(DYNALGO_PLATFORM "riscv64_starfive")
    else()
        message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()
endif()

message(STATUS "HAL: Building for platform ${DYNALGO_PLATFORM}")

# Default vendors per platform
if(DYNALGO_PLATFORM STREQUAL "x86_64")
    set(DYNALGO_DEFAULT_CAMERA_VENDOR "uvc" CACHE STRING "Camera vendor")
    set(DYNALGO_DEFAULT_ENCODER_VENDOR "ffmpeg" CACHE STRING "Encoder vendor")
    set(DYNALGO_DEFAULT_INFERENCE_VENDOR "tensorrt" CACHE STRING "Inference vendor")
    set(DYNALGO_DEFAULT_DISPLAY_VENDOR "sdl2" CACHE STRING "Display vendor")
    set(DYNALGO_DEFAULT_ACTUATOR_VENDOR "generic_serial" CACHE STRING "Actuator vendor")
    set(DYNALGO_DEFAULT_SENSOR_VENDOR "generic" CACHE STRING "Sensor vendor")
    set(DYNALGO_DEFAULT_LOGGER_VENDOR "spdlog" CACHE STRING "Logger vendor")
    set(DYNALGO_DEFAULT_TIME_VENDOR "chrono" CACHE STRING "Time vendor")
    
    # Multi-camera HAL options for x86_64
    option(ENABLE_ORBBEC_HAL "Build Orbbec Camera HAL (305/305g/335L/336L)" ON)
    option(ENABLE_ROBOSENSE_HAL "Build RoboSense AC1 Camera HAL" ON)
    option(ENABLE_STEREO_HAL "Build Generic UVC Stereo Camera HAL" ON)
    option(ENABLE_ROBOSENSE_LIDAR_HAL "Build RoboSense AC1 LiDAR Sensor HAL" ON)
    
elseif(DYNALGO_PLATFORM STREQUAL "aarch64_nvidia")
    set(DYNALGO_DEFAULT_CAMERA_VENDOR "nvsipl" CACHE STRING "Camera vendor")
    set(DYNALGO_DEFAULT_ENCODER_VENDOR "nvmedia" CACHE STRING "Encoder vendor")
    set(DYNALGO_DEFAULT_INFERENCE_VENDOR "tensorrt" CACHE STRING "Inference vendor")
    set(DYNALGO_DEFAULT_DISPLAY_VENDOR "nvdisplay" CACHE STRING "Display vendor")
    set(DYNALGO_DEFAULT_ACTUATOR_VENDOR "gpio" CACHE STRING "Actuator vendor")
    set(DYNALGO_DEFAULT_SENSOR_VENDOR "iio" CACHE STRING "Sensor vendor")
    set(DYNALGO_DEFAULT_LOGGER_VENDOR "spdlog" CACHE STRING "Logger vendor")
    set(DYNALGO_DEFAULT_TIME_VENDOR "ptp" CACHE STRING "Time vendor")
elseif(DYNALGO_PLATFORM STREQUAL "aarch64_generic")
    set(DYNALGO_DEFAULT_CAMERA_VENDOR "v4l2" CACHE STRING "Camera vendor")
    set(DYNALGO_DEFAULT_ENCODER_VENDOR "ffmpeg" CACHE STRING "Encoder vendor")
    set(DYNALGO_DEFAULT_INFERENCE_VENDOR "onnxruntime" CACHE STRING "Inference vendor")
    set(DYNALGO_DEFAULT_DISPLAY_VENDOR "wayland" CACHE STRING "Display vendor")
    set(DYNALGO_DEFAULT_ACTUATOR_VENDOR "gpio" CACHE STRING "Actuator vendor")
    set(DYNALGO_DEFAULT_SENSOR_VENDOR "iio" CACHE STRING "Sensor vendor")
    set(DYNALGO_DEFAULT_LOGGER_VENDOR "spdlog" CACHE STRING "Logger vendor")
    set(DYNALGO_DEFAULT_TIME_VENDOR "chrono" CACHE STRING "Time vendor")
else()
    set(DYNALGO_DEFAULT_CAMERA_VENDOR "v4l2" CACHE STRING "Camera vendor")
    set(DYNALGO_DEFAULT_ENCODER_VENDOR "ffmpeg" CACHE STRING "Encoder vendor")
    set(DYNALGO_DEFAULT_INFERENCE_VENDOR "onnxruntime" CACHE STRING "Inference vendor")
    set(DYNALGO_DEFAULT_DISPLAY_VENDOR "fbdev" CACHE STRING "Display vendor")
    set(DYNALGO_DEFAULT_ACTUATOR_VENDOR "gpio" CACHE STRING "Actuator vendor")
    set(DYNALGO_DEFAULT_SENSOR_VENDOR "iio" CACHE STRING "Sensor vendor")
    set(DYNALGO_DEFAULT_LOGGER_VENDOR "spdlog" CACHE STRING "Logger vendor")
    set(DYNALGO_DEFAULT_TIME_VENDOR "chrono" CACHE STRING "Time vendor")
endif()

# Allow vendor override
set(DYNALGO_CAMERA_VENDOR "${DYNALGO_DEFAULT_CAMERA_VENDOR}" CACHE STRING "Camera vendor")
set(DYNALGO_ENCODER_VENDOR "${DYNALGO_DEFAULT_ENCODER_VENDOR}" CACHE STRING "Encoder vendor")
set(DYNALGO_INFERENCE_VENDOR "${DYNALGO_DEFAULT_INFERENCE_VENDOR}" CACHE STRING "Inference vendor")
set(DYNALGO_DISPLAY_VENDOR "${DYNALGO_DEFAULT_DISPLAY_VENDOR}" CACHE STRING "Display vendor")
set(DYNALGO_ACTUATOR_VENDOR "${DYNALGO_DEFAULT_ACTUATOR_VENDOR}" CACHE STRING "Actuator vendor")
set(DYNALGO_SENSOR_VENDOR "${DYNALGO_DEFAULT_SENSOR_VENDOR}" CACHE STRING "Sensor vendor")
set(DYNALGO_LOGGER_VENDOR "${DYNALGO_DEFAULT_LOGGER_VENDOR}" CACHE STRING "Logger vendor")
set(DYNALGO_TIME_VENDOR "${DYNALGO_DEFAULT_TIME_VENDOR}" CACHE STRING "Time vendor")

# Include platform-specific config
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/platforms/${DYNALGO_PLATFORM}.cmake")
    include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/platforms/${DYNALGO_PLATFORM}.cmake")
else()
    message(WARNING "No platform config for ${DYNALGO_PLATFORM}")
endif()

# Common HAL library
add_library(dynalgo_hal_common STATIC
    common/buffer_manager.cpp
    common/sync_manager.cpp
    common/plugin_loader.cpp
    common/hal_base.cpp
)

target_include_directories(dynalgo_hal_common PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(dynalgo_hal_common PRIVATE
    DYNALGO_HAL_VERSION_MAJOR=${HAL_VERSION_MAJOR}
    DYNALGO_HAL_VERSION_MINOR=${HAL_VERSION_MINOR}
    DYNALGO_HAL_VERSION_PATCH=${HAL_VERSION_PATCH}
)

# Add subdirectories for selected vendors
message(STATUS "HAL: Camera vendor = ${DYNALGO_CAMERA_VENDOR}")
message(STATUS "HAL: Encoder vendor = ${DYNALGO_ENCODER_VENDOR}")
message(STATUS "HAL: Inference vendor = ${DYNALGO_INFERENCE_VENDOR}")
message(STATUS "HAL: Display vendor = ${DYNALGO_DISPLAY_VENDOR}")
message(STATUS "HAL: Actuator vendor = ${DYNALGO_ACTUATOR_VENDOR}")

# Camera HAL - Build default vendor
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${DYNALGO_PLATFORM}/camera/${DYNALGO_CAMERA_VENDOR}/CMakeLists.txt")
    add_subdirectory(${DYNALGO_PLATFORM}/camera/${DYNALGO_CAMERA_VENDOR})
else()
    message(WARNING "Camera HAL not found: ${DYNALGO_PLATFORM}/camera/${DYNALGO_CAMERA_VENDOR}")
endif()

# Additional Camera HALs for x86_64
if(DYNALGO_PLATFORM STREQUAL "x86_64")
    if(ENABLE_ORBBEC_HAL AND NOT "${DYNALGO_CAMERA_VENDOR}" STREQUAL "orbbec")
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/x86_64/camera/orbbec/CMakeLists.txt")
            add_subdirectory(x86_64/camera/orbbec)
        else()
            message(WARNING "Orbbec Camera HAL not found")
        endif()
    endif()
    
    if(ENABLE_ROBOSENSE_HAL AND NOT "${DYNALGO_CAMERA_VENDOR}" STREQUAL "robosense")
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/x86_64/camera/robosense/CMakeLists.txt")
            add_subdirectory(x86_64/camera/robosense)
        else()
            message(WARNING "RoboSense Camera HAL not found")
        endif()
    endif()
    
    if(ENABLE_STEREO_HAL AND NOT "${DYNALGO_CAMERA_VENDOR}" STREQUAL "stereo")
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/x86_64/camera/stereo/CMakeLists.txt")
            add_subdirectory(x86_64/camera/stereo)
        else()
            message(WARNING "Stereo Camera HAL not found")
        endif()
    endif()
endif()

# Encoder HAL
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${DYNALGO_PLATFORM}/encoder/${DYNALGO_ENCODER_VENDOR}/CMakeLists.txt")
    add_subdirectory(${DYNALGO_PLATFORM}/encoder/${DYNALGO_ENCODER_VENDOR})
else()
    message(WARNING "Encoder HAL not found: ${DYNALGO_PLATFORM}/encoder/${DYNALGO_ENCODER_VENDOR}")
endif()

# Inference HAL
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${DYNALGO_PLATFORM}/inference/${DYNALGO_INFERENCE_VENDOR}/CMakeLists.txt")
    add_subdirectory(${DYNALGO_PLATFORM}/inference/${DYNALGO_INFERENCE_VENDOR})
else()
    message(WARNING "Inference HAL not found: ${DYNALGO_PLATFORM}/inference/${DYNALGO_INFERENCE_VENDOR}")
endif()

# Display HAL
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${DYNALGO_PLATFORM}/display/${DYNALGO_DISPLAY_VENDOR}/CMakeLists.txt")
    add_subdirectory(${DYNALGO_PLATFORM}/display/${DYNALGO_DISPLAY_VENDOR})
else()
    message(WARNING "Display HAL not found: ${DYNALGO_PLATFORM}/display/${DYNALGO_DISPLAY_VENDOR}")
endif()

# Actuator HAL
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${DYNALGO_PLATFORM}/actuator/${DYNALGO_ACTUATOR_VENDOR}/CMakeLists.txt")
    add_subdirectory(${DYNALGO_PLATFORM}/actuator/${DYNALGO_ACTUATOR_VENDOR})
else()
    message(WARNING "Actuator HAL not found: ${DYNALGO_PLATFORM}/actuator/${DYNALGO_ACTUATOR_VENDOR}")
endif()

# Sensor HAL
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${DYNALGO_PLATFORM}/sensor/${DYNALGO_SENSOR_VENDOR}/CMakeLists.txt")
    add_subdirectory(${DYNALGO_PLATFORM}/sensor/${DYNALGO_SENSOR_VENDOR})
else()
    message(WARNING "Sensor HAL not found: ${DYNALGO_PLATFORM}/sensor/${DYNALGO_SENSOR_VENDOR}")
endif()

# Additional Sensor HALs for x86_64
if(DYNALGO_PLATFORM STREQUAL "x86_64")
    if(ENABLE_ROBOSENSE_LIDAR_HAL AND NOT "${DYNALGO_SENSOR_VENDOR}" STREQUAL "robosense_lidar")
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/x86_64/sensor/robosense/CMakeLists.txt")
            add_subdirectory(x86_64/sensor/robosense)
        else()
            message(WARNING "RoboSense LiDAR Sensor HAL not found")
        endif()
    endif()
endif()

# Logger HAL
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${DYNALGO_PLATFORM}/logger/${DYNALGO_LOGGER_VENDOR}/CMakeLists.txt")
    add_subdirectory(${DYNALGO_PLATFORM}/logger/${DYNALGO_LOGGER_VENDOR})
else()
    message(WARNING "Logger HAL not found: ${DYNALGO_PLATFORM}/logger/${DYNALGO_LOGGER_VENDOR}")
endif()

# Time HAL
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${DYNALGO_PLATFORM}/time/${DYNALGO_TIME_VENDOR}/CMakeLists.txt")
    add_subdirectory(${DYNALGO_PLATFORM}/time/${DYNALGO_TIME_VENDOR})
else()
    message(WARNING "Time HAL not found: ${DYNALGO_PLATFORM}/time/${DYNALGO_TIME_VENDOR}")
endif()

# Install headers
install(DIRECTORY ../include/dynalgo/hal/
        DESTINATION include/dynalgo/hal
        FILES_MATCHING PATTERN "*.hpp")

# Install common library
install(TARGETS dynalgo_hal_common
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Install HAL libraries
install(TARGETS ${HAL_CAMERA_LIB} ${HAL_ENCODER_LIB} ${HAL_INFERENCE_LIB}
                ${HAL_DISPLAY_LIB} ${HAL_ACTUATOR_LIB} ${HAL_SENSOR_LIB}
                ${HAL_LOGGER_LIB} ${HAL_TIME_LIB}
        LIBRARY DESTINATION lib/dynalgo/hal
        RUNTIME DESTINATION bin)