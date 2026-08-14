// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#pragma once

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

uint64_t dynalgo_get_current_timestamp_ms(void);

char dynalgo_wait_for_key_press(uint32_t timeout_ms);

int dynalgo_support_ansi_escape(void);

#ifdef __cplusplus
}
#endif
