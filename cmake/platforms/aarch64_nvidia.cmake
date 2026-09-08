# aarch64_nvidia Platform Configuration
# NVIDIA Jetson Orin / Drive Orin platform

message(STATUS "Platform: aarch64_nvidia (NVIDIA Jetson/Drive)")

# Compiler flags for Orin (Ampere + ARMv8.2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a -mtune=cortex-a78ae")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8.2-a -mtune=cortex-a78ae")

# Enable NEON and ARMv8.2 features
add_compile_definitions(
    DYNALGO_PLATFORM_AARCH64_NVIDIA=1
    DYNALGO_HAVE_NEON=1
    DYNALGO_HAVE_ARM_V8_2=1
    DYNALGO_HAVE_ATOMICS=1
)

# CUDA (required on Jetson)
find_package(CUDA 12.2 REQUIRED)
set(CUDA_ARCHITECTURES "87" CACHE STRING "Orin compute capability (Ampere)")

# TensorRT
find_package(TensorRT 8.6 REQUIRED)

# NVIDIA SDK libraries (DriveOS / JetPack)
set(NVIDIA_SDK_PATHS
    /usr/lib/aarch64-linux-gnu/tegra
    /usr/lib/aarch64-linux-gnu
    /usr/local/lib
    /opt/nvidia/driveos
)

find_library(NVSIPL_LIB nvsipl PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_LIB nvmedia PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_IEP_LIB nvmedia_iep_sci PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_2D_LIB nvmedia2d PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSCIBUF_LIB nvscibuf PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSCISYNC_LIB nvscisync PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSCISTREAM_LIB nvscistream PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSCIEVENT_LIB nvscievent PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSICIPC_LIB nvsciipc PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSCICOMMON_LIB nvsciCommon PATHS ${NVIDIA_SDK_PATHS})
find_library(TEGRA_WFD_LIB tegrawfd PATHS ${NVIDIA_SDK_PATHS})
find_library(CUDLA_LIB cudla PATHS ${NVIDIA_SDK_PATHS})
find_library(NVML_LIB nvidia-ml PATHS ${NVIDIA_SDK_PATHS})

# V4L2 fallback
find_library(V4L2_LIB v4l2)
find_library(V4L2CONVERT_LIB v4lconvert)

# IIO for sensors
find_library(IIO_LIB iio)

# spdlog
find_package(spdlog QUIET)
if(NOT spdlog_FOUND)
    find_path(SPDLOG_INCLUDE_DIR spdlog/spdlog.h)
    find_library(SPDLOG_LIB spdlog)
endif()

# Platform libraries
set(PLATFORM_LIBS
    ${CUDA_LIBRARIES}
    ${TENSORRT_LIBRARIES}
    ${NVSIPL_LIB}
    ${NVMEDIA_LIB}
    ${NVMEDIA_IEP_LIB}
    ${NVMEDIA_2D_LIB}
    ${NVSCIBUF_LIB}
    ${NVSCISYNC_LIB}
    ${NVSCISTREAM_LIB}
    ${NVSCIEVENT_LIB}
    ${NVSICIPC_LIB}
    ${NVSCICOMMON_LIB}
    ${TEGRA_WFD_LIB}
    ${CUDLA_LIB}
    ${NVML_LIB}
    ${V4L2_LIB}
    ${V4L2CONVERT_LIB}
    ${IIO_LIB}
)

if(SPDLOG_INCLUDE_DIR)
    list(APPEND PLATFORM_LIBS ${SPDLOG_LIB})
endif()

# Platform compile definitions
add_compile_definitions(
    DYNALGO_PLATFORM_AARCH64_NVIDIA=1
    DYNALGO_HAVE_NVSIPL=1
    DYNALGO_HAVE_NVMEDIA=1
    DYNALGO_HAVE_NVSCIBUF=1
    DYNALGO_HAVE_NVSCISYNC=1
    DYNALGO_HAVE_NVSCISTREAM=1
    DYNALGO_HAVE_CUDLA=1
    DYNALGO_HAVE_TENSORRT=1
    DYNALGO_HAVE_CUDA=1
    DYNALGO_HAVE_IIO=1
)

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
    DYNALGO_PLATFORM_NAME="aarch64_nvidia"
    DYNALGO_VENDOR_DEFAULT="nvidia"
)