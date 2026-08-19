# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# app/ — DynamicAlgoCam multi-device capture application.
#
# Layered architecture (dependencies flow downward only):
#   dynamic_algo_cam  →  dynalgo_capture  →  dynalgo_drivers  →  dynalgo_core
#                                         ↗               ↓
#                                    rs_driver*        OrbbecSDK*
#   Optional:
#     dynalgo_actuators    →  dynalgo_core            (--engage-actuator)
#     dynalgo_algo         →  dynalgo_core, dynalgo_actuators  (--engage-model --engage-actuator)
#     dynalgo_opencv_plugin →  dynalgo_capture, dynalgo_core, OpenCV

add_subdirectory(core)
add_subdirectory(driver)
add_subdirectory(capture)
add_subdirectory(actuator)
add_subdirectory(algo)

find_package(OpenCV QUIET)
add_subdirectory(plugins/opencv)
add_subdirectory(dynamic_algo_cam)
