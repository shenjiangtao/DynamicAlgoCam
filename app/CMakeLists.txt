# Copyright (c) NIO Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# app/ — NIO multi-device capture application.
#
# Layered architecture (dependencies flow downward only):
#   dynamic_algo_cam  →  nio_capture  →  nio_drivers  →  nio_core
#                                         ↗               ↓
#                                    rs_driver*        OrbbecSDK*
#   Optional:
#     nio_actuators    →  nio_core            (--engage-actuator)
#     nio_opencv_plugin →  nio_capture, nio_core, OpenCV

add_subdirectory(core)
add_subdirectory(driver)
add_subdirectory(capture)
add_subdirectory(actuator)

find_package(OpenCV QUIET)
add_subdirectory(plugins/opencv)
add_subdirectory(dynamic_algo_cam)
