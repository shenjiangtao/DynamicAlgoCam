# riscv64_starfive Platform Configuration
# StarFive (JH7110) / T-Head RISC-V platform

message(STATUS "Platform: riscv64_starfive (StarFive/T-Head)")

# Compiler flags for RISC-V (RV64GC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=rv64gc -mabi=lp64d")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=rv64gc -mabi=lp64d")

# Enable RISC-V features
add_compile_definitions(
    DYNALGO_PLATFORM_RISCV64_STARFIVE=1
    DYNALGO_HAVE_RVV=1
)

# Find system libraries
find_package(Threads REQUIRED)
find_library(DL_LIB dl)
find_library(RT_LIB rt)
find_package(PkgConfig QUIET)

# V4L2 for camera
find_library(V4L2_LIB v4l2)
find_library(V4L2CONVERT_LIB v4lconvert)

# FFmpeg for encoding
if(PkgConfig_FOUND)
    pkg_check_modules(FFMPEG libavcodec libavformat libavutil libswscale libswresample)
endif()

# StarFive specific libraries
find_library(STARFIVE_VENC_LIB starfive_venc)
find_library(STARFIVE_ISP_LIB starfive_isp)
find_library(THREAD_NPU_LIB thead_npu)

# DRM/KMS for display
if(PkgConfig_FOUND)
    pkg_check_modules(DRM libdrm)
    pkg_check_modules(GBM gbm)
endif()

# IIO for sensors
find_library(IIO_LIB iio)

# GPIO
find_library(GPIOD_LIB gpiod)

# spdlog
find_package(spdlog QUIET)

# Platform libraries
set(PLATFORM_LIBS
    ${CMAKE_THREAD_LIBS_INIT}
    ${DL_LIB}
    ${RT_LIB}
    ${V4L2_LIB}
    ${V4L2CONVERT_LIB}
    ${IIO_LIB}
    ${GPIOD_LIB}
    ${STARFIVE_VENC_LIB}
    ${STARFIVE_ISP_LIB}
    ${THREAD_NPU_LIB}
)

if(FFMPEG_FOUND)
    list(APPEND PLATFORM_LIBS ${FFMPEG_LIBRARIES})
    add_compile_definitions(DYNALGO_HAVE_FFMPEG=1)
endif()

if(DRM_FOUND)
    list(APPEND PLATFORM_LIBS ${DRM_LIBRARIES})
    add_compile_definitions(DYNALGO_HAVE_DRM=1)
endif()

if(GBM_FOUND)
    list(APPEND PLATFORM_LIBS ${GBM_LIBRARIES})
    add_compile_definitions(DYNALGO_HAVE_GBM=1)
endif()

if(SPDLOG_INCLUDE_DIR)
    list(APPEND PLATFORM_LIBS ${SPDLOG_LIB})
    add_compile_definitions(DYNALGO_HAVE_SPDLOG=1)
endif()

# Vendor library paths
set(VENDOR_LIB_DIRS
    ${CMAKE_INSTALL_PREFIX}/lib/dynalgo/hal
    /usr/local/lib/dynalgo/hal
    /usr/lib/dynalgo/hal
    /opt/dynalgo/lib/hal
)

# Default vendor library names
set(HAL_CAMERA_LIB "dynalgo_hal_camera_${DYNALGO_CAMERA_VENDOR}")
set(HAL_ENCODER_LIB "dynalgo_hal_encoder_${DYNALGO_ENCODER_VENDOR}")
set(HAL_INFERENCE_LIB "dynalgo_hal_inference_${DYNALGO_INFERENCE_VENDOR}")
set(HAL_DISPLAY_LIB "dynalgo_hal_display_${DYNALGO_DISPLAY_VENDOR}")
set(HAL_ACTUATOR_LIB "dynalgo_hal_actuator_${DYNALGO_ACTUATOR_VENDOR}")
set(HAL_SENSOR_LIB "dynalgo_hal_sensor_${DYNALGO_SENSOR_VENDOR}")
set(HAL_LOGGER_LIB "dynalgo_hal_logger_${DYNALGO_LOGGER_VENDOR}")
set(HAL_TIME_LIB "dynalgo_hal_time_${DYNALGO_TIME_VENDOR}")

# Platform-specific definitions
add_compile_definitions(
    DYNALGO_PLATFORM_NAME="riscv64_starfive"
    DYNALGO_VENDOR_DEFAULT="starfive"
)