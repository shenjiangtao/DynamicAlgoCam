# nio_multi_capture 技术参考文档

> 基于源码 `app/nio_multi_capture/nio_multi_capture.cpp` 及其工具库实现分析。
> 所有行为描述均以实现代码为依据，未经验证的功能不做推测。

---

## 1. 概述

`nio_multi_capture` 是基于 OrbbecSDK v2 的多设备并发采集工具。它自动发现所有已连接的 Orbbec 摄像头，为每台设备建立独立的视频/IMU 采集管道，将彩色、深度、红外流编码为 H.264 文件，深度原始数据写入 `.raw` 二进制文件，IMU 数据写入 CSV 文本文件。当设备同时具备彩色和深度传感器时，支持 D2C（深度对齐到彩色）融合：将深度帧 jet colormap 着色后与彩色帧 alpha 混合，输出融合 H.264 文件。

**适用受众**：
- **集成开发者**：需要将多设备采集嵌入自有数据管线
- **运维人员**：需要部署、配置和排查录制任务
- **数据分析师**：需要理解输出文件格式以进行后处理

---

## 2. 依赖与版本要求

### 2.1 核心 SDK

| 依赖 | 最低版本 | 引用位置 | 用途 |
|------|----------|----------|------|
| **OrbbecSDK v2** (`libobsensor`) | SDK v2.x | `nio_multi_capture.cpp:22`, `CMakeLists.txt:30` | 设备发现、Pipeline 管理、帧回调、D2C 对齐 |
| **FFmpeg libavcodec** | 4.x+ | `nio_h264_encoder.hpp:28` | H.264 编码 (libx264)、MJPEG 解码 |
| **FFmpeg libswscale** | 4.x+ | `nio_h264_encoder.hpp:31` | 像素格式转换 (src→YUV420P) |
| **FFmpeg libavutil** | 4.x+ | `nio_h264_encoder.hpp:29` | AVFrame/AVPacket 分配、选项设置 |
| **FFmpeg libavformat** | 4.x+ | `examples/utils/CMakeLists.txt:14` | 格式支持（链接要求） |
| **FFmpeg libswresample** | 4.x+ | `examples/utils/CMakeLists.txt:17` | 音频重采样（链接要求，本例未直接使用） |

### 2.2 工具库 (`ob_examples_utils`)

程序链接 `ob::examples::utils` 静态库（`CMakeLists.txt:31`），该库通过 `PUBLIC` 链接传递 FFmpeg 依赖，消费者无需重复声明。

| 模块 | 头文件 | 提供功能 |
|------|--------|----------|
| `nio_common` | `nio_common.hpp` | 信号处理 (`g_running`/`signalHandler`)、时间戳 (`getTimestampMs`)、目录创建 (`mkdirp`)、SEI NAL 写入、设备名匹配 (`deviceMatches`)、流配置选择 (`selectBestProfile`) |
| `nio_h264_encoder` | `nio_h264_encoder.hpp` | `H264Encoder` 类：封装 FFmpeg x264 编码器 + 可选 MJPEG 解码器，支持 RGB/BGR/Y16/YUYV/MJPEG 等输入格式 |
| `nio_stream_io` | `nio_stream_io.hpp` | `StreamEncoder`/`SensorFiles` 结构体、H.264 NAL 写入（关键帧门控）、深度 raw 文件写入、4MB 缓冲文件 I/O |
| `nio_color_convert` | `nio_color_convert.hpp` | `MjpgDecoderRes` (RAII MJPEG 解码器)、`decodeColorToRGB()` (任意格式→RGB24)、jet colormap、5×7 位图文字渲染、四象限合成 |
| `nio_log` | `nio_log.hpp` | `Logger` 单例：多级别日志（TRACE→FATAL）、线程安全、文件+控制台双输出 |
| `utils` | `utils.hpp` | `ob_smpl` 命名空间：键盘轮询 (`waitForKeyPressed`)、设备型号判断 (`isGemini305gDevice`)、时间工具 |

### 2.3 系统与编译要求

| 项目 | 要求 | 来源 |
|------|------|------|
| CMake | >= 3.10 | `CMakeLists.txt:7` |
| C++ 标准 | C++11 | `CMakeLists.txt:12` |
| 编译器 | GCC/Clang/MSVC | `CMakeLists.txt:12` |
| libx264 | 运行时可用 | FFmpeg 编码器依赖，验证：`ffmpeg -encoders \| grep libx264` |
| pthread | 标准 | `examples/utils/CMakeLists.txt:72` |

### 2.4 可选依赖

| 依赖 | 条件 | 用途 |
|------|------|------|
| OpenCV | `find_package(OpenCV)` 成功 | `utils_opencv.cpp`、`nio_color_convert_cv.cpp` 中的额外可视化功能（本示例未使用） |
| matplotlib | Python 后处理 | `parse_depth_raw.py` 深度可视化（非运行时依赖） |
| numpy | Python 后处理 | `parse_depth_raw.py` 数据解析 |
| opencv-python | Python 后处理 | `colorize_from_video.py` 伪彩色渲染 |

---

## 3. 架构设计

### 3.1 双管道架构

每台设备使用两个独立的 `ob::Pipeline`（`nio_multi_capture.cpp:56-57`）：

```
┌─────────────────────────────────────────────────┐
│ DeviceCapture                                    │
│                                                  │
│  videoPipeline ──── color / depth / IR / IR_L / IR_R  │
│  imuPipeline   ──── accel / gyro                    │
│                                                  │
│  两者共享同一 ob::Device 对象                         │
└─────────────────────────────────────────────────┘
```

**原因**：OrbbecSDK 要求加速度计和陀螺仪必须在独立的 Pipeline 实例上启动（`nio_multi_capture.cpp:860-861` 注释），且使用 `OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE` 模式确保 ACCEL+GYRO 同时到达才触发回调。

### 3.2 完整数据流

```
Device → videoPipeline → FrameSet Callback
│
├── [D2C Fusion Path] (if canFuse=true)
│   ├── HW D2C: frameSet already aligned by device
│   ├── SW D2C: ob::Align(OB_STREAM_COLOR).process(frameSet) → alignedFS
│   ├── colorFrame → decodeColorToRGB() → colorRGBBuf (RGB24)
│   ├── depthFrame → Y16→meters→normalize→jetColormap → RGB24
│   ├── alpha blend: fused = (1-α)*color + α*depth_colored
│   │   ├── rawVal==0 → 保留原始彩色
│   │   └── rawVal>0 → 混合着色深度
│   └── fusedEncoder->encodeRGB() → _d2c_fused_<ts>.h264
│
├── [Individual Stream Recording]
│   ├── colorFrame → writeStreamFrame() → _color_<ts>.h264
│   ├── depthFrame → writeStreamFrame() → _depth_<ts>.h264
│   │              └→ writeDepthRawWithHeader() → _depth_raw_<ts>.raw
│   ├── irFrame → writeStreamFrame() → _ir_<ts>.h264
│   ├── irLeftFrame → writeStreamFrame() → _ir_left_<ts>.h264
│   └── irRightFrame → writeStreamFrame() → _ir_right_<ts>.h264
│
└── IMU Pipeline → FrameSet Callback
    ├── accelFrame → as<AccelFrame>()->getValue() → _imu_<ts>.txt
    └── gyroFrame → as<GyroFrame>()->getValue() → _imu_<ts>.txt
```

### 3.3 H264Encoder 编码管线

```
输入像素 → [MJPEG解码?] → sws_scale(YUV420P) → avcodec_send_frame → avcodec_receive_packet → 写入.h264
```

- **MJPEG 格式**：FFmpeg MJPEG 解码器 → YUVJ422P/YUV420P → `sws_scale` → YUV420P → libx264 编码
- **YUYV/UYVY/RGB/BGRA/Y16/Y8/I420/NV12/NV21**：直接 `sws_scale` → YUV420P → 编码
- **原生 H264/H265/HEVC**：跳过编码器，关键帧门控后直接写入 NAL 单元
- **编码参数**：`ultrafast` / `zerolatency` preset，4Mbps 码率，GOP=fps，BT.709 full range

> 代码参考：`nio_h264_encoder.hpp:9-16`（色彩空间设计决策）、`examples/utils/CMakeLists.txt:19`（编码器比特率 4000000）

### 3.4 关键数据结构

#### `DeviceCapture` (`nio_multi_capture.cpp:52-81`)

| 字段 | 类型 | 说明 |
|------|------|------|
| `videoPipeline` | `shared_ptr<ob::Pipeline>` | 视频流管道 |
| `imuPipeline` | `shared_ptr<ob::Pipeline>` | IMU 管道（独立实例） |
| `deviceName` | `string` | 设备名（空格→`_`）+ 序列号 |
| `sensorFiles` | `shared_ptr<SensorFiles>` | 每流编码器与输出文件 |
| `hasIMU` | `bool` | 是否同时有 accel+gyro |
| `depthScale` | `float` | Y16→米换算因子 |
| `alignFilter` | `shared_ptr<ob::Align>` | SW D2C 对齐滤镜 |
| `fusedEncoder` | `shared_ptr<H264Encoder>` | 融合流 RGB 编码器 |
| `colorRGBBuf` / `fusedRGBBuf` | `shared_ptr<vector<uint8_t>>` | 解码后 RGB 缓冲 / 融合输出缓冲 |
| `alpha` | `float` | 深度叠加透明度 (0~1) |
| `depthMinM` / `depthMaxM` | `float` | jet colormap 归一化范围（米） |
| `hwD2CMode` | `bool` | 是否使用硬件 D2C |

#### `CaptureConfig` (`nio_multi_capture.cpp:84-93`)

| 字段 | 类型 | 默认值 | CLI 参数 |
|------|------|--------|----------|
| `cameraFilter` | `vector<string>` | 空(全部) | `-c <name...>` |
| `saveDir` | `string` | `capture_output` | `-s <dir>` |
| `alpha` | `float` | 0.5 | `--alpha` |
| `depthMinM` | `float` | 0.3 | `--depth-min` |
| `depthMaxM` | `float` | 5.0 | `--depth-max` |
| `enableFusion` | `bool` | true | `--no-fusion` 禁用 |

#### `StreamEncoder` (`nio_stream_io.hpp:40-53`)

| 字段 | 类型 | 说明 |
|------|------|------|
| `encoder` | `shared_ptr<H264Encoder>` | 软件编码器（原生 H264 时为 null） |
| `file` | `shared_ptr<ofstream>` | 4MB 缓冲输出文件 |
| `mtx` | `mutex` | 文件写入互斥锁 |
| `h264KeyFrameWritten` | `bool` | 原生 H264 关键帧门控 |
| `isNativeH264` | `bool` | 设备直出 H264 标志 |

---

## 4. 构建与部署

### 4.1 构建步骤

```bash
# 1. 确认依赖
pkg-config --modversion libavcodec libswscale libavutil libavformat libswresample
ffmpeg -encoders | grep libx264   # 确认 libx264 可用

# 2. 构建
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target nio_multi_capture -j$(nproc)

# 3. 验证
./nio_multi_capture --help
```

### 4.2 运行前检查清单

| 检查项 | 命令 | 预期 |
|--------|------|------|
| USB 设备可见 | `lsusb \| grep -i orbbec` | 列出 VID=0x2bc5 的设备 |
| udev 规则已安装 | `ls /etc/udev/rules.d/\|grep orbbec` | 存在 `99-orbbec-usb.rules` |
| USB 缓冲区大小 | `cat /sys/module/usbcore/parameters/usbfs_memory_mb` | >= 128（多设备建议 256） |
| 用户权限 | `groups $(whoami)` | 包含 `plugdev` 或有 USB 设备访问权限 |
| 磁盘空间 | `df -h <saveDir>` | 每设备每分钟约 1.5GB（含 raw） |

### 4.3 USB 缓冲区配置（多设备必须）

```bash
# 临时生效
echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb

# 永久生效
echo "options usbcore usbfs_memory_mb=256" | sudo tee /etc/modprobe.d/usbcore.conf
sudo modprobe -r usbcore && sudo modprobe usbcore
# 或重启
```

> 代码参考：`nio_multi_capture.cpp:228-242`（自动检测并警告）

---

## 5. 运行参考

### 5.1 命令行参数

| 参数 | 短选项 | 默认值 | 说明 |
|------|--------|--------|------|
| `-c <name...>` | — | 无(全部设备) | 按设备名子串过滤，可多次指定 |
| `-s <dir>` | — | `capture_output/` | 录制文件保存根目录 |
| `--alpha VAL` | — | 0.5 | 深度叠加透明度 [0.0, 1.0]，自动钳位 |
| `--depth-min M` | — | 0.3 | jet colormap 最小深度（米） |
| `--depth-max M` | — | 5.0 | jet colormap 最大深度（米） |
| `--no-fusion` | — | false | 禁用 D2C 融合，仅保存独立流 |
| `--help` | `-h` | — | 显示帮助并退出 |

> 代码参考：`nio_multi_capture.cpp:113-168`（parseArgs 实现）；alpha 值自动钳位到 [0.0, 1.0]（`:141`）

### 5.2 运行示例

```bash
# 录制所有设备
./nio_multi_capture

# 仅录制 305 和 336L
./nio_multi_capture -c "305" "336L"

# 自定义保存目录
./nio_multi_capture -s /HDD/nio_capture

# 自定义融合参数
./nio_multi_capture -c "336L" --alpha 0.7 --depth-min 0.2 --depth-max 3.0

# 仅录制原始流（无融合）
./nio_multi_capture --no-fusion

# 组合
./nio_multi_capture -c "305" -s /HDD/nio_capture --alpha 0.6
```

### 5.3 停止录制

- **Ctrl+C** / **SIGTERM**：设置 `g_running=false`，主循环退出后依次 stop pipeline → close encoder → close file
- **键盘 'q' / ESC**：在主循环轮询中检测到后触发优雅退出
- 停止顺序（`nio_multi_capture.cpp:1022-1063`）：pipeline.stop() → encoder.close() → file.close()

---

## 6. 输出文件格式

### 6.1 目录结构

```
<saveDir>/<sessionTimestamp>/<deviceName>/
├── <deviceName>_color_<ts>.h264
├── <deviceName>_depth_<ts>.h264
├── <deviceName>_depth_raw_<ts>.raw
├── <deviceName>_ir_<ts>.h264           (单 IR 设备)
├── <deviceName>_ir_left_<ts>.h264       (双 IR 设备)
├── <deviceName>_ir_right_<ts>.h264      (双 IR 设备)
├── <deviceName>_imu_<ts>.txt            (有 IMU 的设备)
└── <deviceName>_d2c_fused_<ts>.h264     (启用融合时)
```

- `<sessionTimestamp>`：毫秒级 Linux 时间戳（`getTimestampMs()`）
- `<deviceName>`：设备名空格替换为 `_`，拼接序列号（如 `Orbbec_Gemini_336L_CPC64630008B`）
- `<ts>`：设备启动时的毫秒时间戳

> 代码参考：`nio_multi_capture.cpp:274-277`（命名规则）、`:279-280`（目录创建）

### 6.2 H.264 文件

- 编码格式：Constrained Baseline profile, YUV420P, BT.709 full range
- 码率：4 Mbps（`createStreamEncoder` 中 `bitRate=4000000`，`nio_stream_io.cpp:149`）
- 帧率：由设备流配置决定（通常 30fps）
- 关键帧间隔：GOP = fps（即 30fps 时每秒一个 IDR 帧）
- **原生 H264/H265 设备**：NAL 单元直接写入，首个关键帧门控确保文件以 SPS/PPS/IDR 开头

> 代码参考：`nio_stream_io.cpp:28-42`（isH264KeyFrame 扫描 NAL 类型 5/7/8）、`:52-69`（writeH264Frame 关键帧门控）

### 6.3 Depth Raw 文件格式

**头部（44 字节）** (`nio_stream_io.cpp:72-108`)：

| 偏移 | 大小 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| 0 | 16 | Magic | char[16] | `"ORBBEC_DEPTH_RAW"` |
| 16 | 4 | Width | uint32_t | 图像宽度 |
| 20 | 4 | Height | uint32_t | 图像高度 |
| 24 | 4 | BPP | uint32_t | 每像素字节数（=2，Y16） |
| 28 | 4 | Scale | float32 | 深度单位换算因子（如 0.001 = 1mm 精度） |
| 32 | 4 | FrameSize | uint32_t | 每帧字节数（width × height × 2） |
| 36 | 8 | StartTS | uint64_t | 录制起始时间戳（设备微秒时间戳，若为0则用系统时间） |

**帧数据**：首帧后追加原始 Y16 像素数据，uint16_t 小端序，行优先。每帧 `frameSize` 字节。

**深度值换算**：`深度(米) = raw_uint16 × scale`

> 首帧写头部，后续帧仅写数据。每帧后 flush（`:107`）。

### 6.4 IMU 文本文件格式

首行注释头，后续每行一条数据（`nio_multi_capture.cpp:882-905`）：

```
# host_ts_ms,type,device_ts_us,x,y,z,temperature
1748000000000,ACCEL,123456789,0.123,-0.456,9.807,36.5
1748000000000,GYRO,123456789,0.001,0.002,-0.003,36.5
```

| 字段 | 说明 |
|------|------|
| `host_ts_ms` | 主机时间戳（毫秒），用于与视频帧时间对齐 |
| `type` | `ACCEL`（加速度，m/s²）或 `GYRO`（角速度，rad/s） |
| `device_ts_us` | 设备时间戳（微秒） |
| `x/y/z` | 三轴数据 |
| `temperature` | 传感器温度（℃） |

> IMU 采样率约 200Hz（实测 Gemini 336L/335L），不受多设备影响。

---

## 7. 核心算法

### 7.1 流配置选择 (`selectBestProfile`)

> 代码参考：`nio_common.hpp:56-57`（声明）、`nio_common.cpp`（实现）

遍历传感器的所有流配置，按以下规则打分：

| 条件 | 分数 |
|------|------|
| 格式匹配 preferredFormat | +1000 |
| 宽度 640 | +100 |
| 宽度 848 | +90 |
| 宽度 1280 | +80 |
| 帧率 30fps | +50 |
| 帧率 25fps | +45 |
| 帧率 15fps | +30 |

选择总分最高的配置。若首选格式返回 `OB_FORMAT_UNKNOWN`，则回退搜索第一个非 UNKNOWN 格式的配置（`nio_multi_capture.cpp:350-361`、`:387-399`）。

**各传感器首选格式**：
| 传感器 | preferredFormat | 回退格式 |
|--------|----------------|----------|
| Color | `OB_FORMAT_MJPG` | 第一个非 UNKNOWN |
| Depth | `OB_FORMAT_Y16` | 第一个非 UNKNOWN |
| IR / IR_LEFT / IR_RIGHT | `OB_FORMAT_Y8` | 强制 `OB_FORMAT_Y8`（即使返回 UNKNOWN） |

### 7.2 深度精度映射

> 代码参考：`nio_multi_capture.cpp:420-445`

| `OB_PROP_DEPTH_PRECISION_LEVEL_INT` | Scale | 说明 |
|--------------------------------------|-------|------|
| 0 | 0.001 | 1mm 精度 |
| 1 | 0.0005 | 0.5mm 精度 |
| 2 | 0.00025 | 0.25mm 精度 |
| 3 | 0.0001 | 0.1mm 精度 |
| 其他/异常 | 0.001 | 默认 1mm |

深度值换算：`scale` 优先使用每帧 `DepthFrame::getValueScale()`（`nio_multi_capture.cpp:700-706`），若获取失败则回退到设备初始化时的 `depthScale`。

### 7.3 HW D2C 检测算法

> 代码参考：`nio_multi_capture.cpp:170-188`

```cpp
checkIfSupportHWD2CAlign(pipeline, colorProfile, depthProfile):
  hwD2CDepthProfiles = pipeline->getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE)
  if (hwD2CDepthProfiles->getCount() == 0) → 不支持
  for each profile in hwD2CDepthProfiles:
    if (width == depthW && height == depthH && format == depthFmt && fps == depthFps) → 支持
  → 不支持
```

支持时设置 `config->setAlignMode(ALIGN_D2C_HW_MODE)`（`:536`），不支持时创建 `ob::Align(OB_STREAM_COLOR)` 软件滤镜（`:599`）。

### 7.4 Jet Colormap 算法

> 代码参考：`nio_multi_capture.cpp:724-729`

```cpp
jetColormap(v, r, g, b):
  x = v / 255.0
  r = uint8(255 * clamp(1.5 - |4x - 3|, 0, 1))
  g = uint8(255 * clamp(1.5 - |4x - 2|, 0, 1))
  b = uint8(255 * clamp(1.5 - |4x - 1|, 0, 1))
```

深度归一化：`norm = clamp((distM - depthMinM) / (depthMaxM - depthMinM), 0, 1)`，然后 `v = uint8(norm * 255)`。

### 7.5 Alpha 混合规则

> 代码参考：`nio_multi_capture.cpp:732-760`

```
if rawVal == 0 (无效深度):
    fused = 原始彩色
else:
    distM = rawVal × scale / 1000.0   // Y16 → 米
    norm = clamp((distM - depthMinM) / (depthMaxM - depthMinM), 0, 1)
    jetColormap(norm × 255 → cr, cg, cb)
    fused = (1 - alpha) × color + alpha × jet
```

> **注意**：`depthScale` 是 Y16 → 毫米的换算，代码中再除以 1000 得到米（`:744`）。depth 帧对齐后的分辨率可能与 color 不同，混合时取 `min(w, depthW) × min(h, depthH)`（`:733-734`），未覆盖区域保留原始彩色。

---

## 8. 线程安全设计

> 代码参考：`nio_stream_io.hpp:58-71`（SensorFiles 结构）

| 保护对象 | 互斥锁 | 位置 |
|----------|--------|------|
| 每流 H.264 文件写入 | `StreamEncoder::mtx` | 每个流独立锁 |
| 深度 raw 文件写入 | `SensorFiles::depthRawMtx` | 每设备独立锁 |
| IMU 文件写入 | `SensorFiles::imuMtx` | 每设备独立锁 |
| 帧计数器 | `SensorFiles::countMtx` | 每设备独立锁 |
| 融合文件写入 | `DeviceCapture::fusedMtx` | 每设备独立锁 |

**生命周期安全**：`DeviceCapture` 以 `shared_ptr` 存储在 `vector` 中，lambda 回调以值捕获 `shared_ptr`（引用计数+1），确保回调执行期间对象不被释放。

> 历史缺陷修复：早期版本 `DeviceCapture cap` 在栈上创建并以引用捕获到 lambda，`vector::push_back(move(cap))` 导致重分配后原地址失效，回调访问已释放内存 → Segfault。修复为 `shared_ptr` 值捕获。

---

## 9. 设备兼容性

### 9.1 已测试设备

| 设备 | PID | 连接 | Color | Depth | IR | IR Left | IR Right | IMU | 特殊处理 |
|------|-----|------|-------|-------|----|---------|----------|-----|----------|
| Gemini 305 | 0x0840 | USB3.2 | 640×480@30 MJPG | 640×576@30 Y16 | — | ✓ | ✓ | ✗ | 首个 profile fmt=0 需回退；IR_LEFT 需禁用 |
| Gemini 336L | 0x0807 | USB3.2 | 640×480@30 MJPG | 640×576@30 Y16 | — | ✓ | ✓ | ✓ | 346 个颜色 profile |
| Gemini 335L | 0x0804 | USB3.2 | 640×480@30 | 640×480@30 Y16 | — | ✓ | ✓ | ✓ | — |

### 9.2 Gemini 305g 特殊处理

> 代码参考：`nio_multi_capture.cpp:521-526`

```cpp
if (ob_smpl::isGemini305gDevice(vid, pid, connectionType)) {
    config->disableStream(OB_SENSOR_IR_LEFT);
    hasIRLeft = false;
}
```

Gemini 305g 报告了 IR_LEFT 传感器但实际不可用，启用后会导致 Pipeline 启动失败。程序自动检测并禁用。

### 9.3 流配置选择注意事项

- **Gemini 305**：第一个颜色 profile 返回 `OB_FORMAT_UNKNOWN` (format=0)，程序回退搜索非 UNKNOWN 格式的 profile
- **MJPEG 颜色流**：需要额外的 MJPEG 解码步骤，编码管线为 MJPG→解码→sws_scale→libx264
- **原生 H264 设备**（如 Gemini 2）：跳过软件编码，NAL 直接写入

---

## 10. 后处理工具

### 10.1 `parse_depth_raw.py` — 深度原始数据解析

> 文件：`scripts/nio_multi_capture/parse_depth_raw.py`

```bash
# 查看头部信息与统计
python3 parse_depth_raw.py <file.raw> --stats

# 生成彩色深度图 + 直方图 + 截面图
python3 parse_depth_raw.py <file.raw> --output depth_vis

# 生成所有帧的可视化
python3 parse_depth_raw.py <file.raw> --all --output depth_vis

# 终端 ASCII 深度图
python3 parse_depth_raw.py <file.raw> --ascii

# 指定范围和色图
python3 parse_depth_raw.py <file.raw> --min-depth 300 --max-depth 3000 --colormap turbo
```

**依赖**：`numpy`（必需），`matplotlib`（图片输出必需，否则仅终端输出）

**输出**：三子图 — 深度彩色图、深度分布直方图、中心行截面图

### 10.2 `colorize_from_video.py` — 视频伪彩色渲染

> 文件：`scripts/nio_multi_capture/colorize_from_video.py`

```bash
python3 colorize_from_video.py input.mp4 output.mp4 --colormap viridis --min 0 --max 4.0 --fps 30
```

**依赖**：`cv2`（opencv-python）、`numpy`、`ffmpeg`（运行时子进程调用）

**工作流**：逐帧读取视频 → 提取 Y 通道 → 归一化 → OpenCV colormap → FFmpeg pipe → H.264 编码输出

### 10.3 `colorize_from_video.sh` — 一键深度着色脚本

> 文件：`scripts/nio_multi_capture/colorize_from_video.sh`

```bash
./colorize_from_video.sh input.h264 output.mp4 --colormap viridis --min 0 --max 4.0 --fps 30
```

**流程**：H.264 裸流 → FFmpeg 封装为临时 MP4 → Python 逐帧着色 → 输出 MP4

### 10.4 `batch_wrap_color_depth.sh` — 批量 H264 封装

> 文件：`scripts/nio_multi_capture/batch_wrap_color_depth.sh`

```bash
# 批量封装目录中 .h264 → .mp4
./batch_wrap_color_depth.sh ./captures 4

# 仅处理配对的 color+depth
./batch_wrap_color_depth.sh ./captures 4 --paired-only
```

**功能**：自动识别 color/depth 文件名配对，优先无损封装 (`-c copy`)，失败则转码封装。

---

## 11. 故障排查

### 11.1 设备发现

| 现象 | 排查 | 修复 |
|------|------|------|
| "No Orbbec device found!" | `lsusb \| grep 2bc5` | 安装 udev 规则、拔插设备 |
| 设备列表为空但 lsusb 可见 | `groups $(whoami)` | 确认用户在 `plugdev` 组；更新 SDK 版本 |

### 11.2 USB 带宽

| 现象 | 原因 | 修复 |
|------|------|------|
| `UVC_ERROR_NO_MEM` / 第二台设备启动失败 | `usbfs_memory_mb` 不足 | 设为 256（见 §4.3） |
| 某设备 FPS=0 | USB 控制器带宽竞争 | 分布到不同 USB 控制器、`--no-fusion`、降帧率 |

### 11.3 编码与文件

| 现象 | 原因 | 修复 |
|------|------|------|
| H.264 文件无法播放 | 文件以非关键帧开头 / 编码器初始化失败 | 使用 `ffplay -f h264 <file>`；检查 FFmpeg/libx264 安装 |
| 彩色 H.264 为空 (0 字节) | MJPEG 解码失败 | 检查 FFmpeg MJPEG 解码器；查看 stderr 错误信息 |
| 文件开头花屏 | 首帧非 IDR | 正常现象（原生 H264 关键帧门控可能在首个 GOP 后才写入）；跳过前几秒 |

### 11.4 深度数据

| 现象 | 原因 | 修复 |
|------|------|------|
| 深度值全为 0 | 超出量程 / 反射率过低 | 调整 `--depth-min`；融合时 0 值像素自动保留彩色 |
| 深度值 65535 | 传感器饱和（Gemini 305 常见） | 后处理过滤 `raw_uint16 == 65535`；`--depth-max 5.0` 限制融合范围 |
| `.raw` 无法解析 | 头部损坏 / 尺寸不匹配 | `parse_depth_raw.py --stats` 检查头部字段 |

### 11.5 IMU 数据

| 现象 | 排查 |
|------|------|
| IMU 文件为空 | 确认设备有 ACCEL+GYRO（Gemini 305 无）；检查 "Starting IMU pipeline" 日志 |
| ACCEL Y ≈ -9.81 | 正常（静止重力方向） |
| GYRO ≈ 0 | 正常（静止状态） |

### 11.6 关键日志行

| 日志关键字 | 含义 |
|-----------|------|
| `Git commit:` | 构建版本（`GIT_COMMIT_HASH` 编译定义） |
| `HW D2C supported` | 设备支持硬件深度对齐 |
| `using SW alignment` | 设备不支持 HW D2C，回退软件对齐 |
| `usbfs_memory_mb=... too low` | USB 缓冲区配置不足，需增大 |
| `Pipeline start failed` | 管道启动异常，设备可能被占用 |
| `Failed to init fused H264 encoder` | 融合编码器初始化失败，将跳过融合 |
| `Gemini 305g detected, disabled IR_LEFT` | 自动禁用 305g 不可用的 IR_LEFT |

---

## 12. 已知限制与注意事项

1. **CPU 开销**：每设备每帧需 MJPEG 解码 + RGB 转换 + 逐像素深度着色 + alpha 混合 + H264 编码。3 设备同时融合时 CPU 负载高，建议 `--no-fusion` 或减少设备数量。

2. **3 设备融合稳定性**：偶尔某设备 FPS=0（USB 带宽竞争），建议将设备分布到不同 USB 控制器。

3. **Gemini 305 深度饱和**：大量 65535mm 值（超出量程），`--depth-max 5.0` 可过滤。深度有效像素比例约 60%。

4. **文件缓冲区**：所有 H.264 写入使用 4MB 用户空间缓冲（`NIO_FILE_BUF_SIZE`，`nio_stream_io.hpp:35`），减少系统调用频率。编码器 close 时 flush。

5. **设备启动间隔**：每台设备 Pipeline 启动后等待 500ms（`nio_multi_capture.cpp:855`），防止 USB 带宽争抢。

6. **IMU 管道独立**：IMU 必须在独立 Pipeline 实例上启动，无法与视频流共用同一 Pipeline（OrbbecSDK 限制）。

7. **融合帧率**：D2C 融合输出帧率 = `min(colorFps, depthFps)`（`nio_multi_capture.cpp:604`）。

8. **编码色彩空间**：使用 BT.709 full range (`AVCOL_RANGE_JPEG`)，避免 limited range (16-235) 导致的颜色发白。

---

## 13. OrbbecSDK v2 API 参考

以下为本程序直接调用的 OrbbecSDK v2 C++ API：

| API | 引用位置 | 用途 |
|-----|----------|------|
| `ob::Context` | `:211` | SDK 上下文，设备发现入口 |
| `Context::queryDeviceList()` | `:213` | 枚举所有已连接设备 |
| `ob::Device` | `:258` | 设备操作 |
| `Device::getDeviceInfo()` | `:259` | 获取设备名、SN、PID、连接类型 |
| `Device::timerSyncWithHost()` | `:296` | 设备时钟与主机同步 |
| `Device::isGlobalTimestampSupported()` | `:302` | 检查全局时间戳支持 |
| `Device::enableGlobalTimestamp()` | `:304` | 启用全局时间戳 |
| `Device::getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT)` | `:421` | 读取深度精度等级 |
| `Device::getSensorList()` | `:317` | 枚举传感器列表 |
| `ob::Pipeline` | `:312` | 流管道管理 |
| `Pipeline::enableFrameSync()` | `:649` | 启用帧同步 |
| `Pipeline::start(config, callback)` | `:654` | 启动流采集 + 设置帧回调 |
| `Pipeline::stop()` | `:1023` | 停止流采集 |
| `Pipeline::getDevice()` | `:866` | 获取 Pipeline 关联的 Device |
| `Pipeline::getD2CDepthProfileList()` | `:173` | 获取 HW D2C 支持的深度配置列表 |
| `ob::Config` | `:313` | 流配置容器 |
| `Config::enableStream()` | `:364` 等 | 启用指定流 |
| `Config::disableStream()` | `:522` | 禁用指定流（Gemini 305g IR_LEFT） |
| `Config::setAlignMode(ALIGN_D2C_HW_MODE)` | `:536` | 设置硬件 D2C 对齐模式 |
| `Config::setFrameAggregateOutputMode()` | `:315`, `:871` | 设置帧聚合模式 |
| `Config::enableAccelStream()` / `enableGyroStream()` | `:869-870` | 启用 IMU 流 |
| `ob::Align` | `:599` | 软件 D2C 对齐滤镜 |
| `Align::process(frameSet)` | `:667` | 执行对齐 |
| `ob::FrameSet` | `:655` | 帧集合 |
| `FrameSet::getFrame(OB_FRAME_COLOR)` 等 | `:675-676` | 按类型获取帧 |
| `ob::VideoFrame` | `:711` | 视频帧接口 |
| `ob::DepthFrame` | `:701` | 深度帧接口 |
| `DepthFrame::getValueScale()` | `:703` | 获取每帧深度换算因子 |
| `ob::AccelFrame` / `ob::GyroFrame` | `:888`, `:900` | IMU 帧接口 |
| `AccelFrame::getValue()` / `GyroFrame::getValue()` | `:889`, `:901` | 获取 IMU 三轴数据 |
| `ob::TypeHelper::convertOBFrameTypeToString()` | `:998` | 帧类型转字符串 |
| `ob::StreamProfileList` | `:342` | 流配置列表 |
| `ob::VideoStreamProfile` | `:347` 等 | 视频流配置（分辨率/格式/帧率） |

---

## 14. FFmpeg API 参考

| API | 头文件 | 用途 |
|-----|--------|------|
| `avcodec_find_encoder_by_name("libx264")` | `libavcodec` | 查找 H.264 编码器 |
| `avcodec_alloc_context3()` / `avcodec_open2()` | `libavcodec` | 编码器上下文创建与打开 |
| `avcodec_send_frame()` / `avcodec_receive_packet()` | `libavcodec` | 编码帧发送与包接收 |
| `avcodec_find_decoder(AV_CODEC_ID_MJPEG)` | `libavcodec` | MJPEG 解码器查找 |
| `sws_getContext()` / `sws_scale()` | `libswscale` | 像素格式转换 |
| `av_frame_alloc()` / `av_packet_alloc()` | `libavutil` | 帧/包内存分配 |
| `av_image_fill_arrays()` | `libavutil` | 图像数据填充 |
| `av_opt_set()` | `libavutil` | 编码器选项设置 |

> 所有 FFmpeg API 通过 `extern "C"` 链接（`nio_h264_encoder.hpp:27-32`），避免 C++ name mangling。

---

## 15. 像素格式支持矩阵

| OBFormat | FFmpeg 源格式 | 编码路径 | sws 目标 |
|----------|--------------|----------|----------|
| `OB_FORMAT_MJPG` (5) | AV_CODEC_ID_MJPEG → YUVJ422P | 解码→sws→编码 | YUV420P |
| `OB_FORMAT_YUYV` | `AV_PIX_FMT_YUYV422` | sws→编码 | YUV420P |
| `OB_FORMAT_UYVY` | `AV_PIX_FMT_UYVY422` | sws→编码 | YUV420P |
| `OB_FORMAT_RGB` | `AV_PIX_FMT_RGB24` | sws→编码 | YUV420P |
| `OB_FORMAT_BGR` | `AV_PIX_FMT_BGR24` | sws→编码 | YUV420P |
| `OB_FORMAT_RGBA` | `AV_PIX_FMT_RGBA` | sws→编码 | YUV420P |
| `OB_FORMAT_BGRA` | `AV_PIX_FMT_BGRA` | sws→编码 | YUV420P |
| `OB_FORMAT_Y16` (8) | `AV_PIX_FMT_GRAY16LE` | sws→编码 | YUV420P |
| `OB_FORMAT_Y8` (9) | `AV_PIX_FMT_GRAY8` | sws→编码 | YUV420P |
| `OB_FORMAT_NV12` | `AV_PIX_FMT_NV12` | sws→编码 | YUV420P |
| `OB_FORMAT_NV21` | `AV_PIX_FMT_NV21` | sws→编码 | YUV420P |
| `OB_FORMAT_I420` | `AV_PIX_FMT_YUV420P` | sws→编码 | YUV420P |
| `OB_FORMAT_H264` / `H265` / `HEVC` | — | **跳过编码**，NAL 直写 | — |

> MJPEG 解码器可能输出 YUVJ422P (4:2:2) 而非 YUV420P (4:2:0)，因此 sws 上下文在首帧解码后延迟创建（`nio_h264_encoder.hpp:13-16`、`nio_color_convert.hpp:9`）。

---

## 16. 关键历史 Bug 修复摘要

| Bug | 现象 | 根因 | 修复 | 代码参考 |
|-----|------|------|------|----------|
| Lambda 悬空引用 | 多设备启动后 Segfault | `DeviceCapture` 栈上创建，引用捕获到 lambda，vector 重分配后地址失效 | 改用 `shared_ptr<DeviceCapture>` 值捕获 | `:52`, `:922` |
| `getVideoStreamProfile(i)` 误用 | 流配置不匹配 | `getVideoStreamProfile(width, height, format, fps)` 中 `i` 被解释为 width | 改用 `getProfile(i)->as<ob::VideoStreamProfile>()` | `:353` |
| MJPEG 编码输出 0 字节 | Gemini 305 颜色 .h264 空 | MJPEG srcFmt 映射为 YUV420P，与 dstFmt 相同，跳过 swsCtx 创建；但解码器 stride/padding 不一致 | 对 MJPG 始终创建 swsCtx | `nio_h264_encoder.hpp:13-16` |
| `OB_FORMAT_UNKNOWN` 导致流无数据 | Gemini 305 第一个 profile format=0 | 某些设备首个 profile 为占位/默认值 | 回退搜索非 UNKNOWN 格式 profile | `:350-361` |
| IR 使用 Y16 格式 | IR 画面过暗 | Y16 格式不适合红外可视化 | IR/IR_LEFT/IR_RIGHT 首选格式改为 `OB_FORMAT_Y8` | `:449`, `:468`, `:489` |
| 60fps USB 带宽不足 | 多流时 USB 缓冲区溢出 | 60fps 带宽需求过高 | profile 评分中 30fps 得 50 分（优先于 60fps） | `selectBestProfile` 评分规则 |
| `UVC_ERROR_NO_MEM` | 第二台设备启动失败 | `usbfs_memory_mb=16` 不足 | 增大到 256，程序自动检测警告 | `:228-242` |

---

## 17. 性能参考数据

> 来源：`test_report/TEST_REPORT.md`，三设备并发录制 ~13s

| 场景 | 设备 | Color FPS | Depth FPS | IR FPS | IMU FPS |
|------|------|-----------|-----------|--------|---------|
| 单设备 | 335L | 30.0 | 30.0 | 30.0 | ~200 |
| 单设备 | 305 | 30.0 | 30.0 | 30.0 | N/A |
| 多设备 | 305+336L+335L | 30.0 | 30.0 | 30.0 | ~197-200 |

**H.264 编码质量**（多设备模式）：
- 码率：3.5~4.5 Mbps
- Color QP：19~24（MJPEG→H264 转码细节损失）
- Depth/IR QP：10~16（编码效率高）

**存储需求**：每设备每分钟约 1.5GB（含 depth .raw 约 14MB/s，H.264 约 0.5MB/s × 4~5 流）

---

## 18. 未解决信息与后续建议

1. **OrbbecSDK 精确版本号**：本仓库未包含 SDK 版本信息，建议在 `CMakeLists.txt` 或 `AGENTS.md` 中记录构建所用的 SDK 版本。

2. **HW D2C 设备列表**：当前仅 Gemini 2 / Femto Mega 等型号支持 HW D2C，完整支持列表需从 Orbbec 官方文档确认。

3. **多设备 USB 拓扑建议**：文档未说明具体哪条 USB 总线对应哪个物理接口，建议运维部署时运行 `lsusb -t` 确认设备分布。

4. **I420 格式设备**：代码中支持 I420 格式但未在测试设备上验证。

5. **OpenCV 可选模块**：`ob_examples_utils` 在 `find_package(OpenCV)` 成功时编译 `nio_color_convert_cv.cpp`，但本示例未使用 OpenCV 功能，文档中不涉及。

6. **libavformat / libswresample 使用**：这两个库作为链接依赖存在，但本程序代码未直接调用其 API。它们是 FFmpeg 生态的间接依赖。

7. **编码器 SEI UUID**：默认值为 `"nio@orbbec-fusio"`（`nio_h264_encoder.hpp:92`），`createStreamEncoder` 传入相同默认值但 `writeSEI=false`（`nio_multi_capture.cpp:556`），即各独立流不写 SEI。融合流使用 `initRGB()` 也传入默认 UUID。
