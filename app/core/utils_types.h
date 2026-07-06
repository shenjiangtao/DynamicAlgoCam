// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#ifdef __cplusplus
extern "C" {
#endif

/* ESC_KEY: scan-code of the Escape key.  enum constant (instead of a
 * #define) so it has a symbol entry, won't pollute the macro namespace,
 * and obeys C/C++ scoping rules.  Works in both C and C++ translation units. */
enum { ESC_KEY = 27 };

#ifdef __cplusplus
}
#endif
