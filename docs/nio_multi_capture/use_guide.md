### nio.multi_capture 简介

**nio_multi_capture** 是一个多品牌深度摄像头同时采集与录制的工具。支持 **Orbbec** 和 **RoboSense RS-AC1** 两种设备，会自动发现连接的设备、选择合适的流配置、启动每台设备的视频与 IMU 管道，并将采集到的流写入磁盘（原生 H.264 或通过 FFmpeg 转码为 H.264、原始深度数据、IMU 文本等）。同时支持 **D2C 深度对齐到彩色** 后进行 jet colormap 着色与 alpha 混合，将融合结果编码为 H.264 文件。

---

### 功能说明

- **多设备自动发现与过滤**：支持 Orbbec 和 RoboSense RS-AC1 两种设备，通过 `-c` 参数按设备名子串过滤要录制的设备（如 `-c "305" "336L"`）。
- **自定义保存目录**：支持通过 `-s` 参数指定录制文件的保存目录（如 `-s /HDD/nio_capture`）。
- **D2C 深度-彩色融合**：使用 `ob::Align(OB_STREAM_COLOR)` 将深度帧对齐到彩色坐标系，jet colormap 着色后 alpha 混合，输出融合 H.264 文件（Orbbec 设备）。
- **智能流配置选择**：为每个传感器选择"最佳"的流配置（分辨率、帧率、像素格式）。
- **多种像素格式转码**：使用 FFmpeg（libavcodec/libswscale）将多种输入像素格式统一转为 YUV420P 并编码为 H.264。
- **原生 H.264 直写**：若设备输出原生 H.264/H.265/HEVC，程序可直接写入并处理起始码与关键帧门控。
- **深度原始文件**：支持写入带自定义头部的 `.raw` 深度文件，头部包含魔数、宽高、bpp、scale、frameSize、时间戳。
- **无头模式**：支持 `--no-show` 参数禁用 SDL 窗口，适用于无显示器环境。
- **并发安全**：每流写入使用独立互斥锁保护，深度原始与 IMU 文件也有专用锁。
- **运行时提示**：检测 USB 内存配置并在多设备场景下给出优化建议。

---

### 构建与依赖

**必备依赖**
- CMake（建议 >= 3.10）与支持 C++11 的编译器（GCC/Clang/MSVC）
- FFmpeg 开发库：`libavcodec`、`libavutil`、`libswscale`、`libavformat`（开发头文件与库）
- SDL2 开发库（用于实时预览，无头模式可选）

**可选依赖（通过 CMake option 控制编译）**
- Orbbec SDK（`libobsensor`）— 编译选项 `ENABLE_ORBBEC=ON`（默认开启）
- RoboSense rs_driver — 编译选项 `ENABLE_RS_AC1=ON`（默认开启）

**CMake 构建选项**

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `ENABLE_ORBBEC` | ON | 启用 Orbbec 设备支持 (需要 OrbbecSDK) |
| `ENABLE_RS_AC1` | ON | 启用 RoboSense RS-AC1 支持 (需要 rs_driver) |

> 两个选项可独立开关。若都关闭，构建将只生成空驱动，无法发现任何设备。
> RS-AC1 的 rs_driver 及其依赖 (libusb-ac, libuvc-ac) 均为静态链接，无需额外 .so。

**CMakeLists.txt**
```cmake
# FFmpeg deps are inherited via PUBLIC link from ob_examples_utils.
# No need to find_package FFmpeg in this file.

cmake_minimum_required(VERSION 3.10)
project(nio_multi_capture)

add_executable(${PROJECT_NAME} nio_multi_capture.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

target_link_libraries(${PROJECT_NAME}
    ob::${OB_SDK_LIB_NAME}
    ob::examples::utils
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    FOLDER "app"
    INSTALL_RPATH "$ORIGIN/../lib"
    INSTALL_RPATH_USE_LINK_PATH FALSE
    BUILD_WITH_INSTALL_RPATH TRUE
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
```

**构建步骤（示例）**
1. 在项目根目录创建 `build`：
   ```bash
   mkdir -p build && cd build
   ```
2. 运行 CMake 并编译：
   ```bash
   cmake ..
   make -j$(nproc)
   ```
3. 可执行文件位于 `build` 目录下，或按 CMake 配置的输出路径。

---

### 运行与使用

**命令行参数**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `-c <name...>` | 无(全部设备) | 按设备名子串过滤，可指定多个（如 `-c "305" "336L"`） |
| `-s <dir>` | `capture_output/` | 录制文件的保存目录 |
| `--alpha VALUE` | 0.5 | 深度叠加透明度 (0.0=纯彩色, 1.0=纯深度着色) |
| `--depth-min M` | 0.3 | 着色最小深度 (米)，低于此值的像素显示原始彩色 |
| `--depth-max M` | 5.0 | 着色最大深度 (米)，高于此值的像素显示原始彩色 |
| `--no-fusion` | - | 禁用 D2C 融合输出（仅保存各流独立文件） |
| `--no-show` | - | 禁用 SDL 实时预览窗口（无头模式） |
| `--help` | - | 显示帮助信息 |

**基本运行**
- 录制所有设备（默认保存到 `capture_output/`）：
```bash
./nio_multi_capture
```
- 使用 `-c` 按摄像头类型过滤：
```bash
./nio_multi_capture -c "305" "336L"
```
- 使用 `-s` 指定保存目录：
```bash
./nio_multi_capture -s /HDD/nio_capture
```
- 组合使用 `-c` 和 `-s`：
```bash
./nio_multi_capture -c "305" -s /HDD/nio_capture
```
- 自定义融合参数：
```bash
./nio_multi_capture -c "336L" --alpha 0.7 --depth-min 0.2 --depth-max 3.0
```
- 仅录制原始流，不做融合：
```bash
./nio_multi_capture --no-fusion
```

**输出目录**
- 程序会在指定目录下创建：**`<saveDir>/<sessionTimestamp>/<DEVICE_NAME>/`**
- 默认为 `capture_output/<sessionTimestamp>/<DEVICE_NAME>/`
- 使用 `-s /HDD/nio_capture` 时为 `/HDD/nio_capture/<sessionTimestamp>/<DEVICE_NAME>/`
- 其中 `<sessionTimestamp>` 为毫秒级时间戳，设备目录内包含各传感器对应的文件，例如：
  - `DEVICE_color_<ts>.h264`
  - `DEVICE_depth_<ts>.h264`
  - `DEVICE_depth_raw_<ts>.raw`（写入原始深度摄像头的数据）
  - `DEVICE_ir_left_<ts>.h264`
  - `DEVICE_ir_right_<ts>.h264`
  - `DEVICE_imu_<ts>.txt`
  - `DEVICE_d2c_fused_<ts>.h264`（D2C 融合后的 H.264 文件）

**停止录制**
- 使用 **Ctrl+C** 或发送 SIGTERM，程序会尝试优雅停止并关闭文件与编码器。

---

### 文件格式与注意事项

- **H.264 写入**：对原生 H.264 流，程序会检测 NAL 单元并等待首个关键帧后开始写入，避免文件以不完整 GOP 开头。
- **深度原始 `.raw`**：首帧写入自定义头部，包含魔数 `ORBBEC_DEPTH_RAW`、宽高、bpp、scale、frameSize、时间戳，后续追加原始 16 位深度帧。读取时请使用头部的 `scale` 将原始值转换为米等物理单位。
- **像素格式支持**：程序支持 YUYV、UYVY、RGB/BGR、RGBA/BGRA、NV12/NV21、Y16、Y8、I420、MJPG 等输入格式并转为 YUV420P 编码。若需新增格式，请在 `H264Encoder::init` 中扩展映射并确保 `sws_getContext` 支持转换。
- **性能与 USB 内存**：多设备场景下请检查 `/sys/module/usbcore/parameters/usbfs_memory_mb`，建议设置为 `>= 128` 或更高以避免 USB 缓冲不足。Orbbec (VID=2bc5) 和 RoboSense RS-AC1 (VID=3244) 均为 USB3 设备，均需 usbfs 缓冲支持。

---

### 常见问题与排查建议

- **程序提示 "No device found!"**
- 检查设备是否正确连接（Orbbec VID=2bc5, RS-AC1 VID=3244）、驱动是否安装、当前用户是否有访问设备的权限。确认 `ENABLE_ORBBEC`/`ENABLE_RS_AC1` 编译选项是否启用。
- **录制文件无法播放或损坏**
- 若为原生 H.264，确认文件中已包含关键帧；若使用转码，检查 FFmpeg 编码器初始化是否成功（程序会在 stderr 输出错误信息）。
- **多设备掉帧或不稳定**
- 增加 `usbfs_memory_mb`，减少分辨率或帧率，或逐台排查带宽瓶颈。
- **深度值单位不明确**
- 程序会尝试读取设备深度精度并设置 `depthScale`，`.raw` 头部包含该 `scale`，读取时需乘以该值得到米为单位的深度。

---

### 架构设计

#### 双管道架构

每台设备使用两个独立的 `ob::Pipeline`：

1. **videoPipeline** — 负责 color、depth、IR、IR_LEFT、IR_RIGHT 流的采集与编码
2. **imuPipeline** — 负责 ACCEL 和 GYRO 流的采集

两者共享同一 `ob::Device` 对象。IMU 管道使用 `OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE` 模式，确保加速度计和陀螺仪数据同时到达才触发回调。

#### 数据流

```
Device → Pipeline → FrameSet Callback
├─ [ob::Align(OB_STREAM_COLOR)] → aligned FrameSet
│  ├─ Color Frame → decodeColorToRGB() → RGB24
│  ├─ Aligned Depth Frame (Y16) → jet colormap → RGB24
│  └─ alpha blend: fused = (1-α)*color + α*depth_colored
│     → H264Encoder (RGB24 → sws_scale → libx264 encode) → _d2c_fused_<ts>.h264
├─ Color Frame → H264Encoder (MJPEG decode → sws_scale → libx264 encode) → .h264
├─ Depth Frame ┬→ H264Encoder (Y16 → sws_scale → libx264 encode) → .h264
│              └→ writeDepthRawWithHeader() → .raw
├─ IR Frame → H264Encoder (Y8 → sws_scale → libx264 encode) → .h264
├─ IR Left → H264Encoder → .h264
└─ IR Right → H264Encoder → .h264

IMU Pipeline → FrameSet Callback
├─ AccelFrame → as<AccelFrame>() → getValue() → .txt
└─ GyroFrame → as<GyroFrame>() → getValue() → .txt
```
Device → Pipeline → FrameSet Callback
  ├─ Color Frame  → H264Encoder (MJPEG decode → sws_scale → libx264 encode) → .h264
  ├─ Depth Frame  ┬→ H264Encoder (Y16 → sws_scale → libx264 encode) → .h264
  │               └→ writeDepthRawWithHeader() → .raw
  ├─ IR Frame     → H264Encoder (Y8 → sws_scale → libx264 encode) → .h264
  ├─ IR Left      → H264Encoder → .h264
  └─ IR Right     → H264Encoder → .h264

IMU Pipeline → FrameSet Callback
  ├─ AccelFrame → as<AccelFrame>() → getValue() → .txt
  └─ GyroFrame  → as<GyroFrame>()  → getValue() → .txt
```

#### H264Encoder 编码管线

```
输入像素 → [MJPG解码?] → sws_scale(YUV420P) → avcodec_send_frame → avcodec_receive_packet → 写入.h264
```

- 对于 MJPG 格式：先经 FFmpeg MJPEG 解码器解码为 YUV420P，再经 sws_scale 统一格式后编码
- 对于 YUYV/UYVY/RGB/BGRA/Y16/Y8/I420/NV12/NV21：直接 sws_scale 转 YUV420P 后编码
- 对于原生 H264/H265/HEVC：跳过编码器，直接写入（关键帧门控）
- 编码参数：`ultrafast` / `zerolatency` preset，4Mbps 码率，GOP = fps

#### 流配置选择算法 (`selectBestProfile`)

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

选择总分最高的配置。若首选格式返回 `OB_FORMAT_UNKNOWN`，则回退搜索第一个非 UNKNOWN 格式的配置。

#### Depth Raw 文件格式

**头部（44 字节）：**

| 偏移 | 大小 | 字段 | 说明 |
|------|------|------|------|
| 0 | 16 | Magic | "ORBBEC_DEPTH_RAW" |
| 16 | 4 | Width | uint32_t，图像宽度 |
| 20 | 4 | Height | uint32_t，图像高度 |
| 24 | 4 | BPP | uint32_t，每像素字节数（=2，Y16） |
| 28 | 4 | Scale | float32，深度单位（如 0.001 = 1mm 精度） |
| 32 | 4 | FrameSize | uint32_t，每帧字节数（w×h×2） |
| 36 | 8 | StartTS | uint64_t，录制起始时间戳（毫秒） |

**帧数据：** 每帧 `frameSize` 字节，uint16_t 小端序，行优先。深度值（米）= `raw_uint16 × scale`。

#### 深度精度映射（`OB_PROP_DEPTH_PRECISION_LEVEL_INT`）

| Precision Level | Scale | 说明 |
|----------------|-------|------|
| 0 | 0.001 | 1mm 精度 |
| 1 | 0.0005 | 0.5mm 精度 |
| 2 | 0.00025 | 0.25mm 精度 |
| 3 | 0.0001 | 0.1mm 精度 |

#### 线程安全

- 每个 `StreamEncoder` 有独立的 `std::mutex` 保护文件写入
- `depthRawMtx` 和 `imuMtx` 分别保护深度原始文件和 IMU 文件
- `countMtx` 保护帧计数器
- `DeviceCapture` 使用 `shared_ptr` 存储在 vector 中，避免 lambda 回调中的悬空引用

#### 信号处理与优雅退出

- `SIGINT`/`SIGTERM` → `g_running = false`
- 主循环退出后依次：stop pipeline → close encoder → close file
- 设备间启动间隔 500ms，防止 USB 带宽争抢

---

### 深度原始数据解析工具 (`parse_depth_raw.py`)

```bash
# 查看 .raw 文件头部信息与统计
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

依赖：`numpy`，可选 `matplotlib`（用于图片输出）。

---

### 调试记录与关键 Bug 修复

#### 1. Lambda 捕获悬空引用 → Segfault

**现象**：多设备启动后立即段错误。

**原因**：`DeviceCapture cap` 在栈上创建，lambda 以引用捕获 `&cap`。随后 `captures.push_back(std::move(cap))` 导致 vector 重分配，原 `cap` 地址失效，回调中访问已释放内存。

**修复**：改用 `vector<shared_ptr<DeviceCapture>>`，lambda 以值捕获 `shared_ptr`（引用计数+1），确保生命周期安全。

#### 2. `getVideoStreamProfile(i)` API 误用

**现象**：流配置选择返回不匹配的分辨率/格式。

**原因**：`getVideoStreamProfile(width, height, format, fps)` 不是索引接口，传入 `i` 被解释为 `width` 参数。

**修复**：改用 `getProfile(i)->as<ob::VideoStreamProfile>()` 遍历全部配置。

#### 3. MJPEG 颜色编码输出 0 字节

**现象**：Gemini 305 颜色流 `.h264` 文件为空。

**原因**：MJPEG 源 `srcFmt` 被映射为 `AV_PIX_FMT_YUV420P`，与目标 `dstFmt` 相同，跳过了 `swsCtx_` 创建。但 MJPEG 解码器输出的帧 stride/padding 与编码器输入不一致，需要 `sws_scale` 对齐。

**修复**：对 MJPG/MJPEG 格式始终创建 `swsCtx_`，解码后统一经 `sws_scale` 转换。

#### 4. `OB_FORMAT_UNKNOWN` (format=0) 导致流启动失败

**现象**：Gemini 305 的第一个颜色流配置返回 format=0（UNKNOWN），`enableStream` 后无数据。

**原因**：某些设备的第一个 profile 格式为 UNKNOWN（占位或默认值）。

**修复**：在 `selectBestProfile` 返回 UNKNOWN 后，回退搜索第一个非 UNKNOWN 格式的 profile。若仍无可用格式则跳过该传感器。

#### 5. IR 流使用 Y16 格式（应为 Y8）

**现象**：IR 画面过暗，Y16 格式不适合红外可视化。

**修复**：将 IR/IR_LEFT/IR_RIGHT 的 `preferredFormat` 改为 `OB_FORMAT_Y8`。

#### 6. 60fps 导致 USB 带宽不足

**现象**：多流时 USB 缓冲区溢出。

**修复**：profile 评分中 30fps 得 50 分（优先于 60fps），降低 USB 带宽需求。

#### 7. 多设备 `UVC_ERROR_NO_MEM`

**现象**：第二台设备 pipeline 启动失败，UVC 报 NO_MEM 错误。

**原因**：Linux 默认 `usbfs_memory_mb=16`，不足以支撑多台 USB3 摄像头同时工作。

**修复**：
```bash
echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
# 或永久生效：
# 在 /etc/modprobe.d/ 创建：options usbcore usbfs_memory_mb=256
```

程序在启动时自动检测 Orbbec (VID=2bc5) 和 RoboSense RS-AC1 (VID=3244) 设备数量，并按每设备 256MB 计算所需 usbfs 内存，不足时输出警告。

---

### 已测试设备

| 设备 | PID | 连接 | Color | Depth | IR | IR Left | IR Right | IMU | 备注 |
|------|-----|------|-------|-------|----|---------|----------|-----|------|
| Gemini 305 | 0x0840 | USB3 | 640×480@30 MJPG | 640×576@30 Y16 | ✓ | ✓ (仅非 GMSL2) | ✓ | ✓ | 首个 profile fmt=0，需回退搜索 |
| Gemini 336L | - | USB3 | 640×480@30 MJPG | 640×576@30 Y16 | ✓ | - | - | ✓ | 346 个颜色 profile |
| Gemini 335L | 0x0804 | USB3.2 | 640×480@30 | 640×480@30 Y16 | ✓ | - | - | ✓ | |

> 单设备 30fps 全流录制已验证通过，H264 可用 `ffplay -f h264 <file>.h264` 播放，深度 raw 可用 `parse_depth_raw.py` 解析。

---

### 参考示例

开发过程中参考了以下 OrbbecSDK_v2_examples：

| 示例 | 用途 |
|------|------|
| `save_to_disk` | 文件保存模式、深度原始写入参考 |
| `record` | Pipeline 录制 API 参考 |
| `multi_devices` | 多设备枚举与管理 |
| `color` / `depth` / `ir` | 单传感器流配置与回调 |
| `imu` | IMU 数据采集（AccelFrame/GyroFrame） |
| `multi_streams` | 多流同步采集与 FrameSet 处理 |
| `callback` | 回调模式 vs 轮询模式 |
| `device_record_nogui` | 无 GUI 录制模式参考 |

---

### IMU 数据格式（`.txt`）

首行为注释头，后续每行一条数据：

```
# host_ts_ms,type,device_ts_us,x,y,z,temperature
1748000000000,ACCEL,123456789,0.123,-0.456,9.807,36.5
1748000000000,GYRO,123456789,0.001,0.002,-0.003,36.5
```

- `host_ts_ms`：主机时间戳（毫秒），用于与视频帧时间对齐
- `type`：`ACCEL`（加速度，m/s²）或 `GYRO`（角速度，rad/s）
- `device_ts_us`：设备时间戳（微秒）
- `x/y/z`：三轴数据
- `temperature`：传感器温度（℃）

---

## nio.d2c_fusion — Depth-to-Color Fusion Tool

### 概述

**nio_d2c_fusion** 是一个实时深度-彩色融合工具。它使用 Orbbec SDK 的 `ob::Align(OB_STREAM_COLOR)` 滤镜将深度帧对齐到彩色帧坐标系，然后将深度数据着色（jet colormap）并与彩色帧 alpha 混合，最终编码为 H.264 裸流输出。

源码位于 `examples/7.nio.d2c_fusion/nio_d2c_fusion.cpp`。

### 工作流程

```
设备 → Pipeline(color+depth) → FrameSet
  → ob::Align(OB_STREAM_COLOR) → aligned FrameSet
    → color frame (decode MJPG/RGB/etc → RGB24)
    → aligned depth frame (Y16, same resolution as color after D2C)
  → depth Y16 → normalize by depth range → jet colormap → RGB24
  → alpha blend: fused = (1-α)*color + α*depth_colored
  → H264Encoder (FFmpeg libx264 ultrafast) → .h264 file
```

### 用法

```bash
# 所有设备，默认参数 (alpha=0.5, 0.3m-5.0m)
./nio_d2c_fusion

# 按设备名过滤
./nio_d2c_fusion 336L

# 自定义 alpha 和深度范围
./nio_d2c_fusion --alpha 0.7 --depth-min 0.2 --depth-max 3.0

# 组合过滤 + 参数
./nio_d2c_fusion 336L 335L --alpha 0.6 --depth-max 8.0

# 查看帮助
./nio_d2c_fusion --help
```

### 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `[device_filter...]` | 无(全部设备) | 按设备名子串过滤 |
| `--alpha VALUE` | 0.5 | 深度叠加透明度 (0.0=纯彩色, 1.0=纯深度着色) |
| `--depth-min M` | 0.3 | 着色最小深度 (米)，低于此值的像素显示原始彩色 |
| `--depth-max M` | 5.0 | 着色最大深度 (米)，高于此值的像素显示原始彩色 |
| `--help` | - | 显示帮助信息 |

### 输出目录结构

```
fusion_output/<session_timestamp>/
  <device_name>/
    <device_name>_d2c_fused_<timestamp>.h264
```

### 构建与运行

```bash
cd build && cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target nio_d2c_fusion -j$(nproc)

# 运行
./linux_x86_64/bin/nio_d2c_fusion
```

### 播放输出

```bash
# 直接播放 H264 裸流
ffplay -f h264 -framerate 30 <fused_file>.h264

# 封装为 MP4 后播放
ffmpeg -y -fflags +genpts -r 30 -i <fused_file>.h264 -c copy output.mp4
ffplay output.mp4
```

### 已知限制

- **3 设备同时融合**：偶尔可能出现某设备 FPS=0（USB 带宽竞争），建议将设备分布到不同 USB 控制器
- **CPU 开销**：每帧需执行 MJPG 解码 + RGB 转换 + 逐像素深度着色 + alpha 混合 + H264 编码，多设备时 CPU 负载较高
- **深度范围**：Gemini 305 有大量 65535mm 饱和值，`--depth-max` 设为 5.0 可过滤这些噪声
