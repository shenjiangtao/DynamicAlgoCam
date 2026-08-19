// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

// [文件说明 / File Description]
// 中文：C语言类型定义，包含ESC键扫描码等基础常量
// English: C-language type definitions containing basic constants like ESC key scan code

#ifdef __cplusplus
extern "C" {
#endif

// [常量说明 / Constant Description]
// 中文：ESC键扫描码，使用枚举常量而非宏定义，避免命名空间污染
// English: ESC key scan code, uses enum constant instead of #define to avoid macro namespace pollution
enum { ESC_KEY = 27 };

#ifdef __cplusplus
}
#endif
