# x86_64 Platform Configuration
# Generic PC / Industrial x86 platform

message(STATUS "Platform: x86_64 (Generic PC)")

# Compiler flags for x86_64
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=x86-64 -mtune=generic")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=x86-64 -mtune=generic")

# Enable SIMD
add_compile_definitions(
    DYNALGO_PLATFORM_X86_64=1
    DYNALGO_HAVE_AVX2=1
    DYNALGO_HAVE_SSE4_2=1
)

# Find system libraries
find_package(Threads REQUIRED)
find_library(DL_LIB dl)
find_library(RT_LIB rt)

# V4L2 for camera
find_library(V4L2_LIB
    NAMES v4l2
    PATHS /lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/lib
)
find_library(V4L2CONVERT_LIB
    NAMES v4lconvert
    PATHS /lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/lib
)

if(NOT V4L2_LIB)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(V4L2 libv4l2)
        pkg_check_modules(V4LCONVERT libv4lconvert)
        if(V4L2_FOUND)
            set(V4L2_LIB ${V4L2_LIBRARIES})
        endif()
        if(V4LCONVERT_FOUND)
            set(V4L2CONVERT_LIB ${V4LCONVERT_LIBRARIES})
        endif()
    endif()
endif()

if(NOT V4L2_LIB)
    message(WARNING "V4L2 library not found, camera HAL may not work")
endif()
if(NOT V4L2CONVERT_LIB)
    message(WARNING "V4L2 convert library not found")
endif()

# FFmpeg for encoding
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(FFMPEG libavcodec libavformat libavutil libswscale libswresample)
endif()

# SDL2 for display
if(PkgConfig_FOUND)
    pkg_check_modules(SDL2 sdl2)
endif()

# USB for actuators
find_library(USB_LIB usb-1.0)

# spdlog for logging
find_package(spdlog QUIET)
if(NOT spdlog_FOUND)
    find_path(SPDLOG_INCLUDE_DIR spdlog/spdlog.h
        PATHS /usr/include /usr/local/include /opt/homebrew/include)
    find_library(SPDLOG_LIB spdlog
        PATHS /usr/lib /usr/local/lib /opt/homebrew/lib)
endif()

# OpenSSL for secure comms
find_package(OpenSSL QUIET)

# Platform libraries
set(PLATFORM_LIBS
    ${CMAKE_THREAD_LIBS_INIT}
    ${DL_LIB}
    ${RT_LIB}
    ${V4L2_LIB}
    ${V4L2CONVERT_LIB}
    ${USB_LIB}
)

if(FFMPEG_FOUND)
    list(APPEND PLATFORM_LIBS ${FFMPEG_LIBRARIES})
    add_compile_definitions(DYNALGO_HAVE_FFMPEG=1)
endif()

if(SDL2_FOUND)
    list(APPEND PLATFORM_LIBS ${SDL2_LIBRARIES})
    add_compile_definitions(DYNALGO_HAVE_SDL2=1)
endif()

if(SPDLOG_INCLUDE_DIR)
    list(APPEND PLATFORM_LIBS ${SPDLOG_LIB})
    add_compile_definitions(DYNALGO_HAVE_SPDLOG=1)
endif()

if(OPENSSL_FOUND)
    list(APPEND PLATFORM_LIBS OpenSSL::SSL OpenSSL::Crypto)
    add_compile_definitions(DYNALGO_HAVE_OPENSSL=1)
endif()

# Vendor library paths
set(VENDOR_LIB_DIRS
    ${CMAKE_INSTALL_PREFIX}/lib/dynalgo/hal
    /usr/local/lib/dynalgo/hal
    /usr/lib/dynalgo/hal
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
    DYNALGO_PLATFORM_NAME="x86_64"
    DYNALGO_VENDOR_DEFAULT="generic"
)