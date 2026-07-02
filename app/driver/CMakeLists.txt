# Copyright (c) NIO Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# nio_drivers — Device driver layer: SDK-specific adapters.
# Orbbec driver (optional) + RoboSense RS-AC1 driver (optional).
# Depends on: nio_core, (optionally) OrbbecSDK, (optionally) rs_driver.

cmake_minimum_required(VERSION 3.10)
project(nio_drivers)

add_library(nio_drivers STATIC
    ${CMAKE_CURRENT_LIST_DIR}/dummy.cpp
)

target_link_libraries(nio_drivers PUBLIC
    nio::core
)

if(ENABLE_ORBBEC)
    target_include_directories(nio_drivers PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/orbbec
    )

    target_sources(nio_drivers PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/orbbec/nio_ob_adapter.hpp
        ${CMAKE_CURRENT_LIST_DIR}/orbbec/nio_ob_frame_adapter.hpp
        ${CMAKE_CURRENT_LIST_DIR}/orbbec/nio_ob_device.hpp
        ${CMAKE_CURRENT_LIST_DIR}/orbbec/nio_ob_device.cpp
    )

    target_compile_definitions(nio_drivers PUBLIC
        ENABLE_ORBBEC
    )

    target_link_libraries(nio_drivers PUBLIC
        ob::${OB_SDK_LIB_NAME}
    )

    message(STATUS "Orbbec support enabled (OrbbecSDK)")
endif()

if(ENABLE_RS_AC1)
    target_include_directories(nio_drivers PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/robosense
    )

    target_sources(nio_drivers PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/robosense/nio_rs_adapter.hpp
        ${CMAKE_CURRENT_LIST_DIR}/robosense/nio_rs_frame_adapter.hpp
        ${CMAKE_CURRENT_LIST_DIR}/robosense/nio_rs_device.hpp
        ${CMAKE_CURRENT_LIST_DIR}/robosense/nio_rs_device.cpp
    )

    target_compile_definitions(nio_drivers PUBLIC
        ENABLE_RS_AC1
        DISABLE_PCAP_PARSE
        ENABLE_USB
        ENABLE_IMU_PARSE
        ENABLE_IMAGE_PARSE
    )

    target_compile_options(nio_drivers PRIVATE -Wno-error)

    target_include_directories(nio_drivers PUBLIC
        ${RS_DRIVER_ROOT}/src
        ${RS_DRIVER_ROOT}/thirdparty/libusb/libusb
        ${RS_DRIVER_ROOT}/thirdparty/libuvc/include
        ${CMAKE_BINARY_DIR}/rs_driver/thirdparty/libuvc
    )

    function(_rs_remove_werror TARGET)
        if(TARGET ${TARGET})
            get_target_property(_opts ${TARGET} COMPILE_OPTIONS)
            if(_opts)
                list(FILTER _opts EXCLUDE REGEX "-Werror")
                set_target_properties(${TARGET} PROPERTIES COMPILE_OPTIONS "${_opts}")
            endif()
            target_compile_options(${TARGET} PRIVATE -Wno-error)
        endif()
    endfunction()

    _rs_remove_werror(usb-ac-static)
    _rs_remove_werror(uvc-ac-static)

    target_sources(nio_drivers PRIVATE
        ${RS_DRIVER_ROOT}/src/rs_driver/driver/input/input_usb.cpp
    )
    target_link_libraries(nio_drivers PUBLIC usb-ac-static uvc-ac-static pthread)

    message(STATUS "RS-AC1 support enabled (rs_driver root: ${RS_DRIVER_ROOT})")
endif()

if(ENABLE_ORBBEC OR ENABLE_RS_AC1)
    target_include_directories(nio_drivers PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    target_sources(nio_drivers PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/nio_driver_factory.hpp
        ${CMAKE_CURRENT_LIST_DIR}/nio_driver_factory.cpp
    )
endif()

if(NOT ENABLE_ORBBEC AND NOT ENABLE_RS_AC1)
    message(WARNING "Both ENABLE_ORBBEC and ENABLE_RS_AC1 are OFF — no device drivers will be built")
endif()

add_library(nio::drivers ALIAS nio_drivers)
set_target_properties(nio_drivers PROPERTIES FOLDER "app/driver")
