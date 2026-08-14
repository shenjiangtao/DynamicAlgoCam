### nio.multi_capture 简介

**dynamic_algo_cam** 是一个多品牌深度摄像头同时采集与录制的工具。支持 **Orbbec**（Gemini 305/335L/336L）和 **RoboSense RS-AC1** 两种设备，自动发现连接的设备、选择合适的流配置、启动每台设备的视频与 IMU 管道，将采集到的流写入磁盘。同时支持 D2C 深度对齐到彩色后的 jet colormap 着色与 alpha 混合，输出融合 H.264 文件。

---

### 功能说明

- **多设备自动发现与过滤**：支持 Orbbec 和 RoboSense RS-AC1 设备，通过 `-c` 参数按设备名子串过滤
- **自定义保存目录**：`-s` 参数指定保存目录
- **D2C 深度-彩色融合**：深度帧对齐到彩色坐标系，jet colormap 着色后 alpha 混合，输出融合 H.264
- **智能流配置选择**：自动为每个传感器选择最佳分辨率/帧率/格式
- **多种像素格式转码**：FFmpeg (libavcodec/libswscale) 将输入格式统一转为 YUV420P 再 H.264 编码
- **原生 H.264 直写**：设备直出 H.264/H.265 时跳过转码，关键帧门控后直接写入
- **深度原始文件**：`.raw` 格式，含自定义头部（魔数、宽高、BPP、scale、时间戳）
- **点云原始文件**：RS-AC1 专有，`*.point_raw_*.raw`，`NIO_POINT_CLOUD_RAW` 容器格式
- **无头模式**：`--no-show` 禁用 SDL 窗口
- **Drop-oldest 队列**：SDK 回调永不阻塞，过时帧自动丢弃，保证实时性
- **Git hash 版本追踪**：构建时嵌入 git commit hash，日志中打印

---

### 构建与依赖

**必备依赖**
- CMake >= 3.10
- C++14 编译器（GCC / Clang）
- FFmpeg 开发库：`libavcodec`、`libavutil`、`libswscale`、`libavformat`、`libswresample`
- SDL2 开发库
- pthreads

**可选依赖**
- OpenCV — 启用 `nio_opencv_plugin`（自动检测，未找到时跳过）
- Python 3 + numpy + matplotlib — 后处理工具可选

**CMake 构建选项**（在根目录 `CMakeLists.txt` 中声明）

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `ENABLE_ORBBEC` | ON | 启用 Orbbec 设备支持，链接 OrbbecSDK |
| `ENABLE_RS_AC1` | ON | 启用 RoboSense RS-AC1 支持，链接 rs_driver |

> 至少一个选项必须为 ON，否则 CMake 报 `FATAL_ERROR` 终止。
> RS-AC1 的 rs_driver 及其依赖 (libusb-ac, libuvc-ac) 均为静态链接，无需额外 `.so`。
> OrbbecSDK 为动态链接 (`libOrbbecSDK.so`)，运行时需设置 `LD_LIBRARY_PATH`。

**构建步骤**
```bash
mkdir -p build && cd build

# 两个厂商全部启用（默认）
cmake ..

# 仅 Orbbec
cmake .. -DENABLE_RS_AC1=OFF

# 仅 RS-AC1
cmake .. -DENABLE_ORBBEC=OFF

cmake --build . -j$(nproc)
```

---

### 运行与使用

**命令行参数**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `-c <name...>` | 无(全部设备) | 按设备名子串过滤，可指定多个 |
| `-s <dir>` | `capture_output/` | 录制文件保存目录 |
| `--alpha VALUE` | 0.5 | 深度叠加透明度 (0.0=纯彩色, 1.0=纯深度着色) |
| `--depth-min M` | 0.3 | 着色最小深度 (米) |
| `--depth-max M` | 5.0 | 着色最大深度 (米) |
| `--no-fusion` | off | 禁用 D2C 融合输出 |
| `--no-show` | off | 禁用 SDL 实时预览窗口 |
| `--help` | — | 显示帮助信息 |

**运行示例**
```bash
./dynamic_algo_cam                                      # 录制全部设备
./dynamic_algo_cam -c "305" "336L"                      # 按设备名过滤
./dynamic_algo_cam -s /HDD/nio_capture                  # 自定义保存目录
./dynamic_algo_cam --alpha 0.7 --depth-min 0.2 --depth-max 3.0  # 自定义融合参数
./dynamic_algo_cam --no-fusion                          # 仅录制原始流
./dynamic_algo_cam --no-show                            # 无头模式
```

**输出目录**
```
<saveDir>/<sessionTimestamp>/<deviceName>/
├── <name>_color_<ts>.h264              # 彩色流 H.264
├── <name>_depth_<ts>.h264              # 深度流 H.264
├── <name>_depth_raw_<ts>.raw           # 深度原始数据 (Y16, NIO_DEPTH_RAW 格式)
├── <name>_ir_left_<ts>.h264            # IR 左 (Orbbec 有 IR 的设备)
├── <name>_ir_right_<ts>.h264           # IR 右 (Orbbec 有 IR 的设备)
├── <name>_imu_<ts>.txt                 # IMU CSV (有 IMU 的设备)
├── <name>_d2c_fused_<ts>.h264          # D2C 融合 H.264 (启用融合时)
└── <name>_point_raw_<ts>.raw           # 点云原始 (RS-AC1 专有)
```

**停止录制**
- `Ctrl+C` 或 `SIGTERM` → `g_running = 0` → 优雅停止：stop pipeline → close encoder → close file

---

### 运行前检查清单

| 检查项 | 命令 | 预期 |
|--------|------|------|
| USB 设备可见 | `lsusb \| grep -E '2bc5\|3840'` | 列出对应 VID 设备 |
| udev 规则 (Orbbec) | `ls /etc/udev/rules.d/ \| grep orbbec` | `99-obsensor-usb.rules` 存在 |
| RS-AC1 udev 规则 | `ls /etc/udev/rules.d/ \| grep robosense` | `99-robosense-ac1.rules` 存在 |
| RS-AC1 uvcvideo 已 unbind | `lsusb -t \| grep uvcvideo` | RS-AC1 设备不应绑定 uvcvideo |
| USB 缓冲区 | `cat /sys/module/usbcore/parameters/usbfs_memory_mb` | >= 128（多设备建议 256） |
| 用户权限 | `groups $(whoami)` | 包含 `plugdev` 或有 USB 设备访问权限 |
| OrbbecSDK 运行时库 | `ldd dynamic_algo_cam \| grep OrbbecSDK` | 能找到 `libOrbbecSDK.so` |
| 磁盘空间 | `df -h <saveDir>` | 每设备每分钟约 100–280 MB |

---

### 文件格式说明

#### H.264 文件

- Constrained Baseline profile, YUV420P, BT.709 full range
- 码率 4 Mbps, GOP = fps (30fps 时每秒一个 IDR)
- 原生 H.264/H.265 设备：NAL 直接写入，首关键帧门控

#### 深度 Raw 文件

头部 44 字节：

| 偏移 | 大小 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| 0 | 16 | Magic | char[16] | `"NIO_DEPTH_RAW"` |
| 16 | 4 | Width | uint32_t | 图像宽度 |
| 20 | 4 | Height | uint32_t | 图像高度 |
| 24 | 4 | BPP | uint32_t | 每像素字节数 (=2) |
| 28 | 4 | Scale | float32 | 深度单位换算因子 |
| 32 | 4 | FrameSize | uint32_t | 每帧字节数 (w×h×2) |
| 36 | 8 | StartTS | uint64_t | 录制起始时间戳 (ms) |

帧数据：uint16_t 小端序，行优先。**深度(米) = raw_uint16 × scale**。

#### 点云 Raw 文件 (RS-AC1)

`NIO_POINT_CLOUD_RAW` 容器格式：68B 文件头 + 每帧 32B 帧头 + pointCount × 26B 数据。
单点结构：float x + float y + float z + float intensity + uint16 ring + double timestamp。

#### IMU CSV 文件

```
# host_ts_ms,type,device_ts_us,x,y,z,temperature
1748000000000,ACCEL,123456789,0.123,-0.456,9.807,36.5
1748000000000,GYRO,123456789,0.001,0.002,-0.003,36.5
```

- `host_ts_ms`：主机时间戳 (ms)
- `type`：`ACCEL` (m/s²) 或 `GYRO` (rad/s)
- `device_ts_us`：设备时间戳 (μs)
- 采样率约 200 Hz (Gemini 335L/336L)

---

### 后处理工具

```bash
# 深度 raw 解析
python3 app/tools/parse_depth_raw.py <file.raw> --stats
python3 app/tools/parse_depth_raw.py <file.raw> --output vis --all
python3 app/tools/parse_depth_raw.py <file.raw> --ascii

# 点云 raw 解析 (RS-AC1)
python3 app/tools/parse_point_raw.py <file.raw> --info
python3 app/tools/parse_point_raw.py <file.raw> --frame 0

# H.264 播放
ffplay -f h264 <file>.h264

# H.264 转 MP4
ffmpeg -y -fflags +genpts -r 30 -i <file>.h264 -c copy output.mp4
```

依赖：`numpy` (必需), `matplotlib` (图片输出必需)。

---

### 已测试设备

| 设备 | VID:PID | Color | Depth | IR | IMU | 备注 |
|------|---------|-------|-------|----|-----|------|
| Gemini 305 | `2bc5:0840` | 640×480 MJPG | 640×576 Y16 | IR_L + IR_R | — | 首个 profile fmt=0 需回退 |
| Gemini 335L | `2bc5:0804` | 1280×720 MJPG | 640×480 Y16 | IR_L + IR_R | ✓ | |
| Gemini 336L | `2bc5:0807` | 1280×720 MJPG | 640×576 Y16 | IR_L + IR_R | ✓ | |
| RoboX AC1 | `3840:1010` | 1920×1080 NV12 | 96×288 Y16 | — | — | 需 unbind uvcvideo |

---

### 架构设计

#### 分层架构

```
┌──────────────────────────────────────┐
│  dynamic_algo_cam (executable)       │  main(), CLI 解析, 信号处理
├──────────────────────────────────────┤
│  nio_capture (static lib)             │  CaptureSession, H264Encoder,
│                                       │  StreamTask, FrameConsumer, SDLViewer
├──────────────────────────────────────┤
│  nio_drivers (static lib)             │  discoverDevices() 工厂
│  ├─ orbbec/  (ENABLE_ORBBEC)         │  ObDevice, ObPipeline, ObContext
│  └─ robosense/ (ENABLE_RS_AC1)       │  RsDevice, RsPipeline, RsContext
├──────────────────────────────────────┤
│  nio_core (static lib)               │  NioDevice, NioPipeline, NioFrame,
│                                       │  NioFormat, NioFrameType, Logger
└──────────────────────────────────────┘
  + nio_opencv_plugin (optional)        → nio_core, nio_capture, OpenCV
```

#### 数据流

```
SDK 回调 (OrbbecSDK / rs_driver 线程)
  → 像素数据拷贝出 SDK 缓冲区 (adapter 层)
  → NioFrameSet (shared_ptr, 堆分配)
  → VideoFrameQueue::push() [drop-oldest]
  → 消费者线程: pop()
  → FrameConsumer::consume()
    → StreamTask::enqueue(FrameBlob) [二次拷贝]
    → StreamTask worker: processFrame()
      ├─ EncodeStreamTask:  H264Encoder → .h264
      ├─ DepthRawTask:     writeDepthRaw + writePcd → .raw
      ├─ FusionStreamTask:  jetColormap + alpha blend → H264Encoder → .h264
      └─ ImuStreamTask:    append CSV → .txt
```

关键设计：像素数据拷贝两次（adapter 层拷出 SDK 缓冲区，task 层拷入 FrameBlob），确保下游处理完全 SDK-agnostic，SDK 缓冲区可立即归还。

#### 线程模型

| 线程 | 产生者 | 生命周期 | 功能 |
|------|--------|----------|------|
| SDK 回调线程 | OrbbecSDK / rs_driver | start→stop | push NioFrameSet 到 VideoFrameQueue |
| Video 消费者 | CaptureSession | start→stop | pop 队列，分发到各 FrameConsumer |
| IMU 消费者 | CaptureSession | start→stop | pop IMU 队列，分发到 ImuStreamTask |
| StreamTask worker (x5) | 各 Consumer | start→stop | 编码/写入磁盘 |
| SDL decode + render | SDLViewer | init→shutdown | 解码渲染预览 |

#### 流配置选择算法

遍历传感器所有流配置，按规则打分选最高分：

| 条件 | 分数 |
|------|------|
| 格式匹配 preferredFormat | +1000 |
| 宽度 640 | +100 |
| 宽度 848 | +90 |
| 宽度 1280 | +80 |
| 帧率 30fps | +50 |
| 帧率 25fps | +45 |
| 帧率 15fps | +30 |

首选格式回退：Color→MJPG, Depth→Y16, IR→Y8。首个 profile 格式为 UNKNOWN 时回退搜索。

---

### 关键 Bug 修复记录

1. **Lambda 捕获悬空引用 → Segfault** — `DeviceCapture` 栈对象以引用捕获到 lambda，vector 重分配后地址失效。修复：改为 `shared_ptr` 值捕获。
2. **`getVideoStreamProfile(i)` API 误用** — 传入索引 `i` 被解释为 `width` 参数。修复：改用 `getProfile(i)->as<>()` 遍历。
3. **MJPEG 编码输出 0 字节** — MJPEG 解码后帧 stride 与编码器输入不一致，需 `sws_scale` 对齐。修复：MJPG 格式始终创建 swsCtx。
4. **`OB_FORMAT_UNKNOWN` (format=0)** — 首个 profile 格式为 UNKNOWN。修复：回退搜索非 UNKNOWN 格式。
5. **IR 流 Y16 格式应为 Y8** — IR 过暗。修复：preferredFormat 改为 `OB_FORMAT_Y8`。
6. **60fps USB 带宽不足** — 评分中 30fps 优先 (50 分 > 60fps)。
7. **多设备 `UVC_ERROR_NO_MEM`** — `usbfs_memory_mb` 默认 16MB 不够。修复：设为 256MB。

---

### RS-AC1 使用注意事项

- **USB 3.0 必须**：RS-AC1 仅支持 USB 3.0
- **uvcvideo 必须解绑**：RS-AC1 使用自定义 libuvc 驱动，需先 unbind 内核 uvcvideo
- **点云数据**：RS-AC1 是唯一输出点云的设备（96×288 = 27,648 点/帧, 10fps）
- **D2C 融合**：RS-AC1 D2C 始终为硬件（工厂校准），深度从 96×288 上采样到 1920×1080，近距离有明显块效应
- **IMU**：RS-AC1 通过 rs_driver HID 输出 ~100Hz accel+gyro 数据，温度字段为 0（驱动未暴露）
