// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#pragma once

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

uint64_t nio_get_current_timestamp_ms(void);

char nio_wait_for_key_press(uint32_t timeout_ms);

int nio_support_ansi_escape(void);

#ifdef __cplusplus
}
#endif
