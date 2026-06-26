# Copyright (c) NIO Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# app/ — NIO multi-device capture application.
#
# Layered architecture (dependencies flow downward only):
#   nio_multi_capture  →  nio_capture  →  nio_drivers  →  nio_core
#                                         ↗               ↓
#                                    rs_driver*        OrbbecSDK*
#
#   * rs_driver and OrbbecSDK are only linked when ENABLE_RS_AC1 / ENABLE_ORBBEC are ON
#     (options declared in root CMakeLists.txt)
#
# Optional plugin:
#   nio_opencv_plugin  →  nio_capture, nio_core, OpenCV

add_subdirectory(core)
add_subdirectory(driver)
add_subdirectory(capture)

find_package(OpenCV QUIET)
add_subdirectory(plugins/opencv)
add_subdirectory(nio_multi_capture)
