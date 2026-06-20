# Copyright (c) Orbbec Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# ob_examples_utils — Static library of shared utilities for NIO examples.
# Provides: signal handling, timestamps, H.264 encoding, stream I/O,
# color conversion (with optional OpenCV support), and logging.
# FFmpeg and OpenCV deps are linked PUBLIC so consumers inherit them.

cmake_minimum_required(VERSION 3.10)
project(ob_examples_utils)

find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(SWSCALE REQUIRED libswscale)
pkg_check_modules(SWRESAMPLE REQUIRED libswresample)
pkg_check_modules(SDL2 REQUIRED sdl2)

add_library(ob_examples_utils STATIC
    ${CMAKE_CURRENT_LIST_DIR}/utils_c.c
    ${CMAKE_CURRENT_LIST_DIR}/utils_c.h
    ${CMAKE_CURRENT_LIST_DIR}/utils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/utils.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_log.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_common.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_common.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_h264_encoder.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_h264_encoder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_stream_io.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_stream_io.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_color_convert.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_color_convert.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_sdl_viewer.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_sdl_viewer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_thread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_thread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_stream_tasks.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_stream_tasks.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_capture_config.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_capture_session.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_capture_session.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_frame_queue.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_frame_queue.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_frame_consumer.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_frame_consumer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_types.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_ob_adapter.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_frame.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_frame.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_ob_frame_adapter.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_device.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_ob_device.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_ob_device.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_rs_adapter.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_rs_frame_adapter.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_rs_device.hpp
    ${CMAKE_CURRENT_LIST_DIR}/nio_rs_device.cpp
)

target_link_libraries(ob_examples_utils PUBLIC
    ob::${OB_SDK_LIB_NAME}
    ${AVCODEC_LIBRARIES}
    ${AVFORMAT_LIBRARIES}
    ${AVUTIL_LIBRARIES}
    ${SWSCALE_LIBRARIES}
    ${SWRESAMPLE_LIBRARIES}
    ${SDL2_LIBRARIES}
)

target_include_directories(ob_examples_utils PUBLIC
    ${AVCODEC_INCLUDE_DIRS}
    ${AVFORMAT_INCLUDE_DIRS}
    ${AVUTIL_INCLUDE_DIRS}
    ${SWSCALE_INCLUDE_DIRS}
    ${SWRESAMPLE_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
)

target_link_directories(ob_examples_utils PUBLIC
    ${AVCODEC_LIBRARY_DIRS}
    ${AVFORMAT_LIBRARY_DIRS}
    ${AVUTIL_LIBRARY_DIRS}
    ${SWSCALE_LIBRARY_DIRS}
    ${SWRESAMPLE_LIBRARY_DIRS}
)

if(${OpenCV_FOUND})
    target_sources(ob_examples_utils PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/utils_opencv.cpp
        ${CMAKE_CURRENT_LIST_DIR}/utils_opencv.hpp
        ${CMAKE_CURRENT_LIST_DIR}/nio_color_convert_cv.hpp
        ${CMAKE_CURRENT_LIST_DIR}/nio_color_convert_cv.cpp
    )
    target_link_libraries(ob_examples_utils PUBLIC ${OpenCV_LIBS})
    target_include_directories(ob_examples_utils PUBLIC ${OpenCV_INCLUDE_DIRS})
endif()

find_package(Threads REQUIRED)
target_link_libraries(ob_examples_utils PUBLIC Threads::Threads)

target_include_directories(ob_examples_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(ob::examples::utils ALIAS ob_examples_utils)
set_target_properties(ob_examples_utils PROPERTIES FOLDER "examples")

# RS-AC1 support (optional)
option(ENABLE_RS_AC1 "Enable RoboSense RS-AC1 support" OFF)

if(ENABLE_RS_AC1)
    set(RS_DRIVER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../RoboSense/rs_driver-dev_opt_AC1")

    # Configure rs_driver options before add_subdirectory.
    set(DISABLE_PCAP_PARSE ON CACHE BOOL "Disable PCAP file parse" FORCE)
    set(COMPILE_DEMOS OFF CACHE BOOL "Build rs_driver demos" FORCE)
    set(COMPILE_TOOLS OFF CACHE BOOL "Build rs_driver tools" FORCE)
    set(ENABLE_USB ON CACHE BOOL "Enable USB interface in rs_driver" FORCE)

    target_compile_definitions(ob_examples_utils PRIVATE
        ENABLE_RS_AC1
        DISABLE_PCAP_PARSE
        ENABLE_IMU_PARSE
        ENABLE_IMAGE_PARSE
    )

    # rs_driver headers trigger GCC warnings (unused-parameter, etc.)
    # that become errors under OB SDK's -Werror. Downgrade to warnings.
    target_compile_options(ob_examples_utils PRIVATE -Wno-error)

    target_include_directories(ob_examples_utils PUBLIC
        ${RS_DRIVER_ROOT}/src
    )

    # Build rs_driver as subproject (provides rsac_usb target).
    # OB SDK sets -pedantic and -Werror globally via add_compile_options().
    # rs_driver's vendored libusb/libuvc cannot build with these flags.
    # Workaround: override CMAKE_C/CXX_FLAGS for the subdirectory,
    # and remove -Werror from COMPILE_OPTIONS for rs_driver targets.
    set(_SAVED_CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    set(_SAVED_CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REPLACE "-pedantic" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    string(REPLACE "-pedantic" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    add_subdirectory(${RS_DRIVER_ROOT} ${CMAKE_BINARY_DIR}/rs_driver)
    set(CMAKE_C_FLAGS "${_SAVED_CMAKE_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${_SAVED_CMAKE_CXX_FLAGS}")

    # Remove -Werror from rs_driver transitives (add_compile_options is
    # directory-scoped and leaks into subprojects). We override per-target.
    function(_rs_remove_werror TARGET)
        if(TARGET ${TARGET})
            get_target_property(_opts ${TARGET} COMPILE_OPTIONS)
            if(_opts)
                list(FILTER _opts EXCLUDE REGEX "-Werror")
                set_target_properties(${TARGET} PROPERTIES COMPILE_OPTIONS "${_opts}")
            endif()
            # Also strip inherited directory-scope -Werror by adding -Wno-error
            target_compile_options(${TARGET} PRIVATE -Wno-error)
        endif()
    endfunction()

    _rs_remove_werror(usb-ac-static)
    _rs_remove_werror(uvc-ac-static)
    _rs_remove_werror(rsac_usb)

    # Link the rs_driver USB library and its transitive deps.
    target_link_libraries(ob_examples_utils PUBLIC rsac_usb usb-ac-static uvc-ac-static pthread)

    message(STATUS "RS-AC1 support enabled (rs_driver root: ${RS_DRIVER_ROOT})")
endif()
