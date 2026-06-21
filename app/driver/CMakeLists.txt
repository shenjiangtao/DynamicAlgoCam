# Copyright (c) NIO Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# nio_drivers — Device driver layer: SDK-specific adapters.
# Orbbec driver (always on) + RoboSense RS-AC1 driver (optional).
# Depends on: nio_core, OrbbecSDK, (optionally) rs_driver.

cmake_minimum_required(VERSION 3.10)
project(nio_drivers)

add_library(nio_drivers STATIC
    ${CMAKE_CURRENT_LIST_DIR}/orbbec/nio_ob_adapter.hpp
    ${CMAKE_CURRENT_LIST_DIR}/orbbec/nio_ob_frame_adapter.hpp
    ${CMAKE_CURRENT_LIST_DIR}/orbbec/nio_ob_device.hpp
    ${CMAKE_CURRENT_LIST_DIR}/orbbec/nio_ob_device.cpp
    ${CMAKE_CURRENT_LIST_DIR}/robosense/nio_rs_adapter.hpp
    ${CMAKE_CURRENT_LIST_DIR}/robosense/nio_rs_frame_adapter.hpp
    ${CMAKE_CURRENT_LIST_DIR}/robosense/nio_rs_device.hpp
    ${CMAKE_CURRENT_LIST_DIR}/robosense/nio_rs_device.cpp
)

target_link_libraries(nio_drivers PUBLIC
    nio::core
    ob::${OB_SDK_LIB_NAME}
)

target_include_directories(nio_drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/orbbec
    ${CMAKE_CURRENT_SOURCE_DIR}/robosense
)

# RS-AC1 support (optional)
option(ENABLE_RS_AC1 "Enable RoboSense RS-AC1 support" ON)

if(ENABLE_RS_AC1)
    set(RS_DRIVER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../RoboSense/rs_driver-dev_opt_AC1")

    set(DISABLE_PCAP_PARSE ON CACHE BOOL "Disable PCAP file parse" FORCE)
    set(COMPILE_DEMOS OFF CACHE BOOL "Build rs_driver demos" FORCE)
    set(COMPILE_TOOLS OFF CACHE BOOL "Build rs_driver tools" FORCE)
    set(ENABLE_USB ON CACHE BOOL "Enable USB interface in rs_driver" FORCE)

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
    )

    set(_SAVED_CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    set(_SAVED_CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REPLACE "-pedantic" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    string(REPLACE "-pedantic" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    add_subdirectory(${RS_DRIVER_ROOT} ${CMAKE_BINARY_DIR}/rs_driver)
    set(CMAKE_C_FLAGS "${_SAVED_CMAKE_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${_SAVED_CMAKE_CXX_FLAGS}")

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

    # Include InputUsb implementation directly in nio_drivers (instead of
    # linking rsac_usb shared lib) because rsac_usb.so hides all symbols
    # with -fvisibility=hidden and our inline InputFactory usage needs
    # the InputUsb vtable/typeinfo at link time.
    target_sources(nio_drivers PRIVATE
        ${RS_DRIVER_ROOT}/src/rs_driver/driver/input/input_usb.cpp
    )
    target_include_directories(nio_drivers PUBLIC
        ${RS_DRIVER_ROOT}/thirdparty/libusb/libusb
        ${RS_DRIVER_ROOT}/thirdparty/libuvc/include
        ${CMAKE_BINARY_DIR}/rs_driver/thirdparty/libuvc
    )
    target_link_libraries(nio_drivers PUBLIC usb-ac-static uvc-ac-static pthread)

    message(STATUS "RS-AC1 support enabled (rs_driver root: ${RS_DRIVER_ROOT})")
endif()

add_library(nio::drivers ALIAS nio_drivers)
set_target_properties(nio_drivers PROPERTIES FOLDER "app/driver")
