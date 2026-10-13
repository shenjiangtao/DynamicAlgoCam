# aarch64_nvidia Platform Configuration
# NVIDIA Jetson Orin / Drive Orin platform

message(STATUS "Platform: aarch64_nvidia (NVIDIA Jetson/Drive)")

# Use aarch64-linux-gnu cross-compiler
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Compiler flags for Orin (Ampere + ARMv8.2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a -mtune=cortex-a76")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8.2-a -mtune=cortex-a76")

# Enable NEON and ARMv8.2 features
add_compile_definitions(
    DYNALGO_PLATFORM_AARCH64_NVIDIA=1
    DYNALGO_HAVE_NEON=1
    DYNALGO_HAVE_ARM_V8_2=1
    DYNALGO_HAVE_ATOMICS=1
)

# CUDA (manual setup - container has CUDA 11.4 for x86_64 only)
# We use cross-compilation without host CUDA; target CUDA is on the device
# If needed, CUDA_PATH can be set to the target device's CUDA installation
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-11.4" CACHE PATH "CUDA toolkit root")
set(CUDA_ARCHITECTURES "87" CACHE STRING "Orin compute capability (Ampere)")

# TensorRT (optional - may not be in container)
find_package(TensorRT 8.6 QUIET)
if(NOT TensorRT_FOUND)
    message(WARNING "TensorRT not found in container, continuing without it")
endif()

# NVIDIA SDK libraries (DriveOS / JetPack)
set(NVIDIA_SDK_PATHS
    /drive/drive-linux/lib-target
    /drive/drive-linux/lib
    /usr/lib/aarch64-linux-gnu/tegra
    /usr/lib/aarch64-linux-gnu
    /usr/local/lib
    /opt/nvidia/driveos
)

# Include directories
include_directories(/drive/drive-linux/include)

find_library(NVSIPL_LIB nvsipl PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSCIBUF_LIB nvscibuf PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSCISYNC_LIB nvscisync PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSCISTREAM_LIB nvscistream PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSCIEVENT_LIB nvscievent PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSICIPC_LIB nvsciipc PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSCICOMMON_LIB nvsciCommon PATHS ${NVIDIA_SDK_PATHS})
find_library(TEGRA_WFD_LIB tegrawfd PATHS ${NVIDIA_SDK_PATHS})
find_library(CUDLA_LIB nvcudla PATHS ${NVIDIA_SDK_PATHS})
find_library(NVML_LIB nvidia-ml PATHS ${NVIDIA_SDK_PATHS})

# NvMedia component libraries (no single libnvmedia.so in container)
find_library(NVMEDIA_2D_LIB nvmedia2d PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_IEP_LIB nvmedia_iep_sci PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_DLA_LIB nvmedia_dla PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_EGLSTREAM_LIB nvmedia_eglstream PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_IDE_PARSER_LIB nvmedia_ide_parser PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_IDE_SCI_LIB nvmedia_ide_sci PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_IJPD_SCI_LIB nvmedia_ijpd_sci PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_IJPE_SCI_LIB nvmedia_ijpe_sci PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_IOFA_SCI_LIB nvmedia_iofa_sci PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_TENSOR_LIB nvmedia_tensor PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_LDC_LIB nvmedialdc PATHS ${NVIDIA_SDK_PATHS})

# Combine all NvMedia component libraries
set(NVMEDIA_LIBS
    ${NVMEDIA_2D_LIB}
    ${NVMEDIA_IEP_LIB}
    ${NVMEDIA_DLA_LIB}
    ${NVMEDIA_EGLSTREAM_LIB}
    ${NVMEDIA_IDE_PARSER_LIB}
    ${NVMEDIA_IDE_SCI_LIB}
    ${NVMEDIA_IJPD_SCI_LIB}
    ${NVMEDIA_IJPE_SCI_LIB}
    ${NVMEDIA_IOFA_SCI_LIB}
    ${NVMEDIA_TENSOR_LIB}
    ${NVMEDIA_LDC_LIB}
)

# For backward compatibility, set NVMEDIA_LIB to NVMEDIA_LIBS
set(NVMEDIA_LIB ${NVMEDIA_LIBS})

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