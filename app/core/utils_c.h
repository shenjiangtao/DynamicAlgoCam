// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#pragma once

#include <stdint.h>
#include <stdlib.h>

#ifdef ENABLE_ORBBEC
#include <libobsensor/ObSensor.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

uint64_t ob_smpl_get_current_timestamp_ms(void);

char ob_smpl_wait_for_key_press(uint32_t timeout_ms);

#ifdef ENABLE_ORBBEC
bool ob_smpl_is_lidar_device(ob_device* device);
#endif

int ob_smpl_support_ansi_escape(void);

bool ob_smpl_is_gemini305_device(int vid, int pid);
bool ob_smpl_is_gemini305g_device(int vid, int pid, const char* connectionType);
bool ob_smpl_is_astra_mini_device(int vid, int pid);

#ifdef ENABLE_ORBBEC
#define CHECK_OB_ERROR_EXIT(error)                                               \
    if (*error) {                                                                \
        const char* error_message = ob_error_get_message(*error);                \
        ob_status status = ob_error_get_status(*error);                          \
        fprintf(stderr, "Error: %s (status: %d)\n", error_message, (int)status); \
        ob_delete_error(*error);                                                 \
        *error = NULL;                                                           \
        exit(-1);                                                                \
    }                                                                            \
    *error = NULL;
#endif

#ifdef __cplusplus
}
#endif
