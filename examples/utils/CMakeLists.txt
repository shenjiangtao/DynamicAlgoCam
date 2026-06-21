# Copyright (c) Orbbec Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# ob_examples_utils — MIGRATED to app/ layered architecture.
# This directory is retained for backward compatibility only.
# All source files have been moved; see README_MIGRATION.md.
#
# The alias ob::examples::utils still resolves but now points
# to the combined layered libraries via nio::capture.

if(NOT TARGET ob::examples::utils)
    add_library(ob_examples_utils INTERFACE)
    target_link_libraries(ob_examples_utils INTERFACE nio::capture)
    if(TARGET nio_opencv_plugin)
        target_link_libraries(ob_examples_utils INTERFACE nio::opencv_plugin)
    endif()
    add_library(ob::examples::utils ALIAS ob_examples_utils)
endif()
