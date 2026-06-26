# nio_multi_capture 技术参考文档

> 基于源码 `app/` 目录实现分析。所有行为描述以实现代码为依据。

---

## 1. 概述

`nio_multi_capture` 是基于 NioDevice/NioPipeline 抽象层的多厂商多设备并发采集工具。它自动发现所有已连接的 Orbbec 深度摄像头和 RoboSense RS-AC1 LiDAR，为每台设备建立独立的采集管道，将彩色、深度、红外流编码为 H.264 文件，深度原始数据写入 `.raw` 二进制文件，IMU 数据写入 CSV 文本文件。支持 D2C（深度对齐到彩色）融合：深度帧 jet colormap 着色后与彩色帧 alpha 混合，输出融合 H.264 文件。

**适用受众**：
- **集成开发者**：需要将多设备采集嵌入自有数据管线
- **运维人员**：需要部署、配置和排查录制任务
- **数据分析师**：需要理解输出文件格式以进行后处理

---

## 2. 依赖与版本要求

### 2.1 外部依赖

| 依赖 | 最低版本 | 所属层 | 用途 |
|------|----------|--------|------|
| **OrbbecSDK v2** (`libobsensor`) | SDK v2.x | nio_drivers | 设备发现、Pipeline 管理、帧回调、D2C 对齐 |
| **FFmpeg libavcodec** | 4.x+ | nio_capture | H.264 编码 (libx264)、MJPEG 解码 |
| **FFmpeg libswscale** | 4.x+ | nio_capture | 像素格式转换 (src→YUV420P) |
| **FFmpeg libavutil** | 4.x+ | nio_capture | AVFrame/AVPacket 分配 |
| **FFmpeg libavformat** | 4.x+ | nio_capture | 格式支持（链接要求） |
| **FFmpeg libswresample** | 4.x+ | nio_capture | 链接要求 |
| **SDL2** | 2.x | nio_capture | 实时预览窗口 |
| **pthreads** | 标准 | nio_core | 线程支持 |
| **OpenCV** (可选) | 3.x+ | nio_opencv_plugin | 额外可视化功能 |

### 2.2 系统与编译要求

| 项目 | 要求 | 来源 |
|------|------|------|
| CMake | >= 3.10 | 根 `CMakeLists.txt` |
| C++ 标准 | C++14 | 根 `CMakeLists.txt:17` |
| C 标准 | C99 | 根 `CMakeLists.txt:19` |
| 编译器 | GCC / Clang | — |
| libx264 | 运行时可用 | `ffmpeg -encoders \| grep libx264` |

### 2.3 CMake 构建选项

选项在**根** `CMakeLists.txt` 中声明（非 `app/` 目录）：

| 选项 | 默认值 | 效果 |
|------|--------|------|
| `ENABLE_ORBBEC` | ON | 编译 `app/driver/orbbec/*`，定义 `-DENABLE_ORBBEC`，链接 `ob::OrbbecSDK` |
| `ENABLE_RS_AC1` | ON | 编译 `app/driver/robosense/*`，定义 `-DENABLE_RS_AC1 -DDISABLE_PCAP_PARSE -DENABLE_USB -DENABLE_IMU_PARSE -DENABLE_IMAGE_PARSE`，链接 `usb-ac-static` `uvc-ac-static` |

约束：至少一个必须为 ON，否则 CMake `FATAL_ERROR`。

---

## 3. 架构设计

### 3.1 分层架构

```
┌─────────────────────────────────────────────────┐
│  nio_multi_capture (executable)                  │  main, CLI (parseArgs),
│  → nio::core + nio::drivers + nio::capture       │  signalHandler, 采集主循环
├─────────────────────────────────────────────────┤
│  nio_capture (static lib, nio::capture)          │  CaptureSession,
│                                                   │  H264Encoder, StreamTask,
│  依赖: nio::core + FFmpeg + SDL2                 │  FrameConsumer, SDLViewer
├─────────────────────────────────────────────────┤
│  nio_drivers (static lib, nio::drivers)           │  discoverDevices() 工厂
│  ├─ orbbec/  (ENABLE_ORBBEC)                    │  ObDevice, ObPipeline, ObContext
│  ├─ robosense/ (ENABLE_RS_AC1)                  │  RsDevice, RsPipeline, RsContext
│  └─ nio_driver_factory (either enabled)          │
│  依赖: nio::core + ob::OrbbecSDK | rs_driver    │
├─────────────────────────────────────────────────┤
│  nio_core (static lib, nio::core)                │  NioDevice (abstract),
│                                                   │  NioPipeline (abstract),
│  依赖: Threads::Threads                          │  NioContext (abstract),
│                                                   │  NioFrame, NioFrameSet, NioFormat,
│                                                   │  Logger, StreamTask, FrameQueue
└─────────────────────────────────────────────────┘
  + nio_opencv_plugin (conditional, if OpenCV found)
```

**硬性规则**：`ENABLE_ORBBEC` / `ENABLE_RS_AC1` 宏和厂商 SDK 头文件仅允许出现在 `app/driver/`。`app/core/` 和 `app/capture/` 必须零厂商 SDK 依赖。

### 3.2 类继承体系

```
NioDevice (abstract, app/core/nio_device.hpp)
├── ObDevice    (app/driver/orbbec/nio_ob_device.hpp)     [ENABLE_ORBBEC]
└── RsDevice    (app/driver/robosense/nio_rs_device.hpp)  [ENABLE_RS_AC1]

NioPipeline (abstract)
├── ObPipeline  [ENABLE_ORBBEC]     — wraps ob::Pipeline
└── RsPipeline  [ENABLE_RS_AC1]     — wraps rs_driver

NioContext (abstract)
├── ObContext    [ENABLE_ORBBEC]    — OrbbecSDK device enumeration
└── RsContext    [ENABLE_RS_AC1]    — rs_driver USB device enumeration

NioD2CAlign (abstract)
└── ObD2CAlign  [ENABLE_ORBBEC]    — wraps ob::Align

StreamTask (base, app/core/nio_thread.hpp)
├── EncodeStreamTask     — H.264 encode + file write
├── DepthRawTask         — raw depth + PCD write
├── FusionStreamTask     — alpha-blended depth-over-color + encode
└── ImuStreamTask        — IMU CSV write

FrameConsumer (base, app/capture/nio_frame_consumer.hpp)
├── ColorFrameConsumer
├── DepthFrameConsumer
├── IRFrameConsumer
├── IRLeftFrameConsumer
└── IRRightFrameConsumer
```

### 3.3 完整数据流

```
SDK 回调线程 (ob::Pipeline / rs_driver)
│
├─ adapter 层: 像素数据拷贝出 SDK 缓冲区
│  ├─ obFrameSetToNio()   [Orbbec: 逐帧 memcpy, 断开 SDK 缓冲依赖]
│  └─ rsDepthToNioFrame() [RoboSense: 逐帧 memcpy]
│
├─ VideoFrameQueue::push()  [drop-oldest 策略, SDK 回调永不阻塞]
│
├─ Video 消费者线程: pop() → FrameConsumer::consume()
│  ├─ ColorFrameConsumer → EncodeStreamTask → .h264
│  ├─ DepthFrameConsumer → EncodeStreamTask → .h264
│  │                     └→ DepthRawTask → .raw (Y16, 自定义头部)
│  ├─ IR*FrameConsumer → EncodeStreamTask → .h264
│  └─ FusionStreamTask (if canFuse):
│       ├─ jet colormap(depth → RGB)
│       ├─ alpha blend: fused = (1-α)×color + α×depth_colored
│       └─ H264Encoder → _d2c_fused_*.h264
│
└─ IMU 消费者线程: pop() → ImuStreamTask → .txt (CSV)
```

### 3.4 H264Encoder 编码管线

```
输入像素 → [MJPEG 解码?] → sws_scale(YUV420P) → avcodec_send_frame → avcodec_receive_packet → .h264
```

- **MJPEG**：FFmpeg MJPEG 解码器 → `sws_scale` → YUV420P → libx264
- **YUYV/UYVY/RGB/BGRA/Y16/Y8/I420/NV12/NV21**：直接 `sws_scale` → YUV420P → 编码
- **原生 H264/H265/HEVC**：跳过编码器，关键帧门控后直接写入
- **编码参数**：`ultrafast` / `zerolatency` preset，4 Mbps 码率，GOP=fps，BT.709 full range

---

## 4. CMake 目标详细映射

| 目标 | 类型 | 别名 | 源文件 | 链接依赖 | 公开定义 |
|------|------|------|--------|----------|----------|
| `nio_core` | STATIC | `nio::core` | utils.c, utils.cpp, nio_common.cpp, nio_frame.cpp, nio_thread.cpp | `Threads::Threads` | — |
| `nio_drivers` | STATIC | `nio::drivers` | dummy.cpp + 条件编译: orbbec/*.cpp, robosense/*.cpp, nio_driver_factory.cpp | `nio::core` + `ob::OrbbecSDK` / `usb-ac-static`+`uvc-ac-static` | `ENABLE_ORBBEC`, `ENABLE_RS_AC1`, `DISABLE_PCAP_PARSE`, `ENABLE_USB`, `ENABLE_IMU_PARSE`, `ENABLE_IMAGE_PARSE` |
| `nio_capture` | STATIC | `nio::capture` | nio_h264_encoder.cpp, nio_stream_io.cpp, nio_color_convert.cpp, nio_sdl_viewer.cpp, nio_stream_tasks.cpp, nio_capture_session.cpp, nio_frame_queue.cpp, nio_frame_consumer.cpp | `nio::core`, FFmpeg (5 libs), SDL2 | — |
| `nio_multi_capture` | EXEC | — | nio_multi_capture.cpp | `nio::core`, `nio::drivers`, `nio::capture` | `GIT_COMMIT_HASH` (PRIVATE) |
| `nio_opencv_plugin` | STATIC (条件) | `nio::opencv_plugin` | utils_opencv.cpp, nio_color_convert_cv.cpp | `nio::core`, `nio::capture`, OpenCV | — |

### 4.1 根 CMakeLists.txt 厂商配置

| 条件 | 缓存变量 | 值 | 说明 |
|------|----------|---|------|
| `ENABLE_ORBBEC` | `OB_BUILD_MAIN_PROJECT` | ON (FORCE) | 告知 OrbbecSDK 为子项目 |
| `ENABLE_ORBBEC` | `OB_SDK_LIB_NAME` | "OrbbecSDK" (FORCE) | 库名 |
| `ENABLE_RS_AC1` | `RS_DRIVER_ROOT` | `${CMAKE_CURRENT_SOURCE_DIR}/vendors/RoboSense` | rs_driver 路径 |
| `ENABLE_RS_AC1` | `DISABLE_PCAP_PARSE` | ON (FORCE) | 禁用 PCAP 解析 |
| `ENABLE_RS_AC1` | `COMPILE_DEMOS` | OFF (FORCE) | 抑制 rs_driver demo 目标 |
| `ENABLE_RS_AC1` | `COMPILE_TOOLS` | OFF (FORCE) | 抑制 rs_driver tool 目标 |
| `ENABLE_RS_AC1` | `ENABLE_USB` | ON (FORCE) | 启用 USB 接口 |

---

## 5. 运行参考

### 5.1 CLI 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `-c <name...>` | 无(全部设备) | 按设备名子串过滤 |
| `-s <dir>` | `capture_output/` | 保存目录 |
| `--alpha VAL` | 0.5 | 透明度 [0.0, 1.0]，自动钳位 |
| `--depth-min M` | 0.3 | jet colormap 最小深度 (米) |
| `--depth-max M` | 5.0 | jet colormap 最大深度 (米) |
| `--no-fusion` | false | 禁用 D2C 融合 |
| `--no-show` | false | 禁用 SDL 预览 |
| `--help` | — | 帮助 |

### 5.2 停止录制

- `SIGINT` / `SIGTERM` → `g_running = 0`
- 主循环退出 → stop pipelines → close encoders → close files
- 设备间启动间隔 500ms，防止 USB 带宽争抢

---

## 6. 输出文件格式

### 6.1 目录结构

```
<saveDir>/<sessionTimestamp>/<deviceName>/
├── <name>_color_<ts>.h264
├── <name>_depth_<ts>.h264
├── <name>_depth_raw_<ts>.raw
├── <name>_ir_left_<ts>.h264       (Orbbec 有 IR 的设备)
├── <name>_ir_right_<ts>.h264      (Orbbec 有 IR 的设备)
├── <name>_imu_<ts>.txt            (有 IMU 的设备: 335L, 336L)
├── <name>_d2c_fused_<ts>.h264     (启用融合时)
└── <name>_point_raw_<ts>.raw      (RS-AC1 专有)
```

文件创建门控条件：

| 文件 | 门控条件 |
|------|----------|
| `*_color_*.h264` | `hasColor && colorFormat != UNKNOWN` |
| `*_depth_*.h264` + `*_depth_raw_*.raw` | `hasDepth && depthFormat != UNKNOWN` |
| `*_ir_left_*.h264` | `hasIRLeft` |
| `*_ir_right_*.h264` | `hasIRRight` |
| `*_imu_*.txt` | `hasAccel && hasGyro` |
| `*_point_raw_*.raw` | `pipeline->isPointCloudDepth()` |
| `*_d2c_fused_*.h264` | `canFuse` (= hasColor && hasDepth) |

### 6.2 Depth Raw 文件格式

头部 44 字节：

| 偏移 | 大小 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| 0 | 16 | Magic | char[16] | `"ORBBEC_DEPTH_RAW"` |
| 16 | 4 | Width | uint32_t | 图像宽度 |
| 20 | 4 | Height | uint32_t | 图像高度 |
| 24 | 4 | BPP | uint32_t | 每像素字节数 (=2) |
| 28 | 4 | Scale | float32 | 深度换算因子 |
| 32 | 4 | FrameSize | uint32_t | 每帧字节数 (w×h×2) |
| 36 | 8 | StartTS | uint64_t | 起始时间戳 (ms) |

帧数据：uint16_t 小端序，行优先。**深度(米) = raw_uint16 × scale**。

### 6.3 Point Cloud Raw 文件格式 (RS-AC1)

`NIO_POINT_CLOUD_RAW` 容器：

```
[File Header - 68 bytes, frame 0 only]
  [20B] magic = "NIO_POINT_CLOUD_RAW\0"
  [4B]  version = 1
  [4B]  fieldCount = 6
  [40B] fieldNames = "x\0y\0z\0intensity\0ring\0timestamp" (padded)

[Frame Header - 32 bytes, every frame]
  [8B] frameIndex (uint64)
  [8B] timestampUs (uint64)
  [4B] pointCount (uint32)
  [4B] pointDataBytes (uint32) = pointCount × 26
  [8B] reserved (uint64, zero)

[Frame Data - pointCount × 26 bytes each]
  float x(4) + float y(4) + float z(4) + float intensity(4) + uint16 ring(2) + double timestamp(8)
```

解析工具：`python3 app/tools/parse_point_raw.py <file.raw> [--frame N|--all|--info]`

### 6.4 IMU CSV 格式

```
# host_ts_ms,type,device_ts_us,x,y,z,temperature
1748000000000,ACCEL,123456789,0.123,-0.456,9.807,36.5
1748000000000,GYRO,123456789,0.001,0.002,-0.003,36.5
```

采样率约 200 Hz (Gemini 335L/336L)。RS-AC1 无 IMU 输出。

### 6.5 H.264 文件规格

- Profile: Constrained Baseline, YUV420P, BT.709 full range
- 码率: 4 Mbps
- GOP: = fps (30fps 时每秒一个 IDR)
- 原生 H264/H265 设备: NAL 直写，首关键帧门控

---

## 7. 核心算法

### 7.1 流配置选择 (`selectBestProfile`)

遍历传感器所有流配置，打分选最高：

| 条件 | 分数 |
|------|------|
| 格式匹配 preferredFormat | +1000 |
| 宽度 640 | +100 |
| 宽度 848 | +90 |
| 宽度 1280 | +80 |
| 帧率 30fps | +50 |
| 帧率 25fps | +45 |
| 帧率 15fps | +30 |

首选格式回退：
| 传感器 | preferredFormat | 回退策略 |
|--------|----------------|----------|
| Color | `OB_FORMAT_MJPG` | 首个非 UNKNOWN |
| Depth | `OB_FORMAT_Y16` | 首个非 UNKNOWN |
| IR / IR_LEFT / IR_RIGHT | `OB_FORMAT_Y8` | 强制 Y8 |

### 7.2 深度精度映射 (Orbbec)

| `OB_PROP_DEPTH_PRECISION_LEVEL_INT` | Scale | 说明 |
|--------------------------------------|-------|------|
| 0 | 0.001 | 1mm |
| 1 | 0.0005 | 0.5mm |
| 2 | 0.00025 | 0.25mm |
| 3 | 0.0001 | 0.1mm |
| 其他 | 0.001 | 默认 1mm |

RS-AC1 固定 scale=5.0 (5mm)。

### 7.3 Jet Colormap

```cpp
x = v / 255.0
r = uint8(255 * clamp(1.5 - |4x - 3|, 0, 1))
g = uint8(255 * clamp(1.5 - |4x - 2|, 0, 1))
b = uint8(255 * clamp(1.5 - |4x - 1|, 0, 1))
```

深度归一化：`norm = clamp((distM - depthMinM) / (depthMaxM - depthMinM), 0, 1)`

### 7.4 Alpha 混合

```
if rawVal == 0 (无效深度):
    fused = 原始彩色
else:
    distM = rawVal × scale / 1000.0
    norm = clamp((distM - depthMinM) / (depthMaxM - depthMinM), 0, 1)
    jetColormap(norm × 255 → cr, cg, cb)
    fused = (1 - alpha) × color + alpha × jet
```

---

## 8. 线程安全设计

| 保护对象 | 互斥锁 | 粒度 |
|----------|--------|------|
| 每流 H.264 文件写入 | `StreamEncoder::mtx` | 每个流独立 |
| 深度 raw 文件写入 | `SensorFiles::depthRawMtx` | 每设备独立 |
| IMU 文件写入 | `SensorFiles::imuMtx` | 每设备独立 |
| 帧计数器 | `SensorFiles::countMtx` | 每设备独立 |

**Drop-oldest 队列** (`FrameQueue<T>`)：有界线程安全队列，满时 `push()` 驱逐队首。SDK 回调线程永不阻塞，消费者始终获取最新数据。

**生命周期安全**：`DiscoveredDevice` 以 `shared_ptr` 存储，lambda 回调值捕获 `shared_ptr`，确保回调期间对象存活。

---

## 9. 设备兼容性

### 9.1 已测试设备

| 设备 | VID:PID | Color | Depth | IR | IMU | 特殊处理 |
|------|---------|-------|-------|----|-----|----------|
| Gemini 305 | `2bc5:0840` | 640×480 MJPG | 640×576 Y16 | IR_L+IR_R | — | 首个 profile fmt=0 需回退；305g 禁用 IR_LEFT |
| Gemini 336L | `2bc5:0807` | 1280×720 MJPG | 640×576 Y16 | IR_L+IR_R | ✓ | 346 个颜色 profile |
| Gemini 335L | `2bc5:0804` | 1280×720 MJPG | 640×480 Y16 | IR_L+IR_R | ✓ | — |
| RoboX AC1 | `3840:1010` | 1920×1080 NV12 | 96×288 Y16 | — | — | 需 unbind uvcvideo; 点云 27648 pts/frame |

### 9.2 Gemini 305g 特殊处理

GMSL2 连接的 305g 报告 IR_LEFT 传感器但实际不可用，启用后 Pipeline 启动失败。程序自动检测并禁用。

### 9.3 RS-AC1 特殊处理

- USB 3.0 必须
- 需 unbind 内核 `uvcvideo` 驱动
- D2C 始终为硬件（工厂校准）
- 深度分辨率 96×288，上采样到 1920×1080
- 固定 depth scale = 5.0 (5mm)

---

## 10. 后处理工具

| 工具 | 路径 | 依赖 | 用途 |
|------|------|------|------|
| `parse_depth_raw.py` | `app/tools/` | numpy, matplotlib | 深度 raw 解析/可视化 |
| `parse_point_raw.py` | `app/tools/` | numpy, matplotlib | 点云 raw 解析/可视化 |

---

## 11. 典型输出大小 (15 秒录制)

| 文件 | Gemini 335L | RoboX AC1 |
|------|-------------|-----------|
| `color.h264` | ~8 MB | ~4 MB |
| `depth.h264` | ~8 MB | ~100–150 KB |
| `depth_raw.raw` | ~230 MB | ~6 MB |
| `ir_left/right.h264` | ~8 MB each | — |
| `imu.txt` | ~300–500 KB | — |
| `d2c_fused.h264` | ~2–3 MB | ~4–5 MB |
| `point_raw.raw` | — | ~75–100 MB |
| **总计** | **~280 MB** | **~90 MB** |
