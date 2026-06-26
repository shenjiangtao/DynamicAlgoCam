# Orbbec + RoboSense RS-AC1 统一采集框架 — 技术方案

> **版本**: 1.0
> **日期**: 2026-06-26
> **受众**: 框架开发者、集成工程师
> **状态**: 已实现 — RS-AC1 集成完成，CMake 选项已移至根 CMakeLists.txt

---

## 1 变更摘要

当前 `nio_multi_capture` 应用仅支持 Orbbec 深度相机（Gemini 2/3 系列）。本方案将 RoboSense RS-AC1（基于 LiDAR 的主动深度相机）纳入同一采集框架，使下游消费者（编码、录文件、预览、融合）无需关心传感器 SDK 来源。

**核心变更**:
- 新增 `RsDevice` / `RsPipeline` 实现 `NioDevice` / `NioPipeline` 抽象接口
- 新增 `rs_frame_adapter.hpp` 将 RS-AC1 的 `PointCloudMsg` + `ImageData` + `ImuData` 转换为 `NioFrameSet`
- RS-AC1 点云作为 `NioFrameType::DEPTH`（乘以 `depthScale=1.0` 后未对齐的 96×288 点阵），图像作为 `NioFrameType::COLOR`
- RS-AC1 IMU 直接映射到 `NioImuSample`（ACCEL + GYRO），经由 `ImuFrameQueue` 走现有 IMU 写入管线

---

## 2 参与受众

| 角色 | 需要做什么 | 涉及深度 |
|------|-----------|---------|
| 框架开发者 | 实现 RsDevice/RsPipeline/RsFrameAdapter; 扩展 NioPipeline 接口 | 全量 |
| 应用集成者 | 修改 `main()` 中设备发现逻辑: OB 用 `ObContext`, RS-AC1 用 `RsContext`；按 VID/PID 或 `LidarType` 分流 | 中等 |
| 运维/操作员 | 新增 CLI 参数 `--rs-ac1` 启用 RS-AC1 设备发现；USB 3.0 前置条件检查 | 浅层 |
| 数据消费者 | 无变更 — 输出文件格式（.h264 / .raw / .txt / .json）不变 | 无 |

---

## 3 系统前置条件

| 条件 | OB 设备 | RS-AC1 设备 |
|------|---------|------------|
| USB 版本 | USB 2.0+ | **USB 3.0 必须**（`enable_usb200=false`） |
| USBFS 内存 | ≥ 128 MB (`usbfs_memory_mb`) | ≥ 128 MB（同） |
| VID:PID | 多种 (0x2bc5:*) | **0x3840:0x1010** |
| SDK 库 | `libOrbbecSDK.so` (动态) | `usb-ac-static` + `uvc-ac-static` (静态链接，无需 LD_LIBRARY_PATH) |
| 编译宏 | 无 | `-DENABLE_USB -DENABLE_IMU_PARSE -DENABLE_IMAGE_PARSE` |
| C++ 标准 | C++14 | C++14（rs_driver 要求，根 CMakeLists.txt 已升级） |
| 设备发现 | `ob::Context::queryDeviceList()` | 无自动发现；按 VID/PID 扫描 libusb 或手动指定 `device_uuid` |
| 内核驱动 | `uvcvideo` 绑定 | **必须解绑** `uvcvideo`（AC1 使用自定义 UVC 扩展，标准内核驱动会抢占） |

### 关键操作: 解绑内核 UVC 驱动

```bash
# RS-AC1 使用自定义 libuvc，标准 uvcvideo 会抢占设备
# 查找 AC1 设备的 USB 设备号
lsusb | grep 3840:1010
# Bus 001 Device 005: ID 3840:1010

# 解绑（替换 <busnum> <devnum> 为实际值）
sudo sh -c 'echo <busnum>-<devnum> > /sys/bus/usb/drivers/uvcvideo/unbind'
```

**风险**: 如果未解绑，`InputUsb::open()` 调用 `uvc_open2()` 将失败，日志出现 `DEVICE_DISCONNECTED` 错误。

---

## 4 RS-AC1 技术规格

### 4.1 数据流

| 流 | 分辨率 | 帧率 | 格式 | USB 接口 |
|----|--------|------|------|----------|
| **深度点云** | 96×288 = 27,648 点 | ≤10 Hz | XYZI+方向向量 (per-packet) | UVC iso IF1 |
| **彩色图像** | 最大 1920×1080 | 30 fps | NV12 / BGR24 / RGB24 / YUV422 | UVC iso IF0 |
| **IMU** | N/A | 100 or 200 Hz | 6-axis (accel xyz + gyro xyz) | HID interrupt |

### 4.2 深度模型（与 OB 的关键技术差异）

| 方面 | Orbbec 深度相机 | RS-AC1 |
|------|----------------|--------|
| 原始输出 | 2D depth map (`uint16_t` per pixel, mm 精度) | 3D point cloud (per-pixel `x,y,z` + `intensity`) |
| 对齐方式 | 需要 `ob::Align` 或 HW D2C 做深度对齐到彩色 | **已在硬件端完成 D2C** — 点云与图像像素对齐 |
| 深度值获取 | `depthFrame->data() → uint16_t → * depthScale` | `point.distance * 0.005m` (解码器已完成) |
| 无效点 | depth=0 或 depth=65535 | xyz=NAN（dense_points=false）或点被省略（dense_points=true） |
| 内参 | `OBCameraIntrinsic` (fx,fy,cx,cy) | 方向向量编码在 depth packet per-pixel (int16 / 32768) |
| IR 流 | 有 OB_SENSOR_IR / IR_LEFT / IR_RIGHT | **无** — AC1 不产生独立 IR 图像 |

### 4.3 USB 架构

```
USB 3.0 Cable
    ├── HID Interface (class 3, subclass 1)
    │     └── libusb interrupt transfer → IMU 6-axis + PTP 时钟同步
    ├── UVC Interface 0 (streaming)
    │     └── libuvc isochronous → 彩色图像帧
    └── UVC Interface 1 (streaming)
          └── libuvc isochronous → 深度/点云帧
```

### 4.4 点云数据包结构

每帧深度数据 ~336 KB，结构为 288 行 × 96 像素，每像素 12 字节:

```
Row Layout (1162 bytes = 10 + 96×12):
  [0-5]   Row time base, seconds (6B, big-endian)
  [6-9]   Row time base, microseconds (4B, big-endian)
  [10+]   96 pixels, each 12 bytes:
    +0,+1   TimeOffset (uint16, μs offset from row base)
    +2,+3   Distance  (uint16, ×0.005 = meters)
    +4,+5   Direction X (int16, ÷32768 = unit vector)
    +6,+7   Direction Y (int16, ÷32768 = unit vector)
    +8,+9   Direction Z (int16, ÷32768 = unit vector)
    +10     Intensity (uint8)
    +11     Reserved  (uint8)
```

XYZ 转换公式:
```
x = distance × (dirX_int16 / 32768)
y = distance × (dirY_int16 / 32768)
z = distance × (dirZ_int16 / 32768)
```

### 4.5 时间戳语义

| 流 | 时间戳来源 | 单位 | 同步 |
|----|-----------|------|------|
| 点云 | Per-row `timeval` + per-pixel `time_offset` (μs) | 秒 (double) | PTP HID 同步 |
| 图像 | Packet offset +4 (8B double) | 秒 (double) | PTP HID 同步 |
| IMU | HID payload `timespec` (sec + nsec) | 秒 (double) | PTP HID 同步 |

所有三流共享同一时间基（经 PTP 同步后的 device CLOCK_REALTIME）。

### 4.6 内置 D2C 对齐

RS-AC1 在硬件层面已完成点云到彩色图像的对齐。点云中每个点对应图像中的一个像素位置。**不存在** OB 的 SW D2C `ob::Align` 需求。

如果下游需要 2D depth map（如融合可视化），需从点云反投影:
```
// 96×288 网格中，point[i] 对应 pixel(col, row):
col = i % 96
row = i / 96
depth[row * 96 + col] = (uint16_t)(point[i].distance / 0.005)
```

---

## 5 当前架构与 RS-AC1 差异映射

### 5.1 概念映射表

| RS-AC1 概念 | OB 等价物 | Nio 中间层类型 | 映射策略 |
|-------------|----------|---------------|---------|
| `PointCloudMsg` (27,648 XYZI 点) | `ob::DepthFrame` (2D depth map) | `NioFrame(COUNT=DEPTH)` | 将 XYZ 压缩为 row-major `float[N*3]`，存入 `NioFrame::data`；或合成 2D `uint16_t` depth map |
| `ImageData` (彩图) | `ob::ColorFrame` | `NioFrame(COUNT=COLOR)` | 直接拷贝 `ImageData::data` → `NioFrame::data`；`frame_format` → `NioFormat` |
| `ImuData` (6-axis) | `ob::FrameSet` (AccelFrame + GyroFrame) | `NioImuSample` | 拆分为 ACCEL + GYRO 两个 `NioImuSample` |
| PTP 时钟同步 | `device->timerSyncWithHost()` | `NioDevice::timerSyncWithHost()` | RS-AC1 在 driver 内部自动做 PTP；适配器层为空操作 |
| 无 IR 流 | `ob::IRFrame` / `ob::IRLeftFrame` | `NioFrame(COUNT=IR/IR_LEFT/IR_RIGHT)` | **不存在**。`RsPipeline` 不使能这些流 |
| 内置 HW D2C | `ob::Align` + `ALIGN_D2C_HW_MODE` | `NioAlignMode::HW` | RS-AC1 始终为 HW D2C; 无 SW D2C 路径 |
| `LidarDriver<PointCloudMsg>` | `ob::Pipeline` | `NioPipeline` | `RsPipeline` 封装 `LidarDriver` |
| `RSDriverParam` | `ob::Config` | `NioStreamConfig` | `RsPipeline::enableStream()` 转换 `NioStreamConfig` → `RSDriverParam` 字段 |
| dual get/put callback pair | `pipeline.start(config, single_callback)` | `NioVideoCallback` | `RsPipeline` 内部注册 get/put 对; 在 put callback 中构建 `NioFrameSet` 并调用户 callback |

### 5.2 NioFormat 与 RS frame_format 映射

| `frame_format_t` | `NioFormat` | 说明 |
|-------------------|------------|------|
| `FRAME_FORMAT_NV12` | `NioFormat::NV12` | 默认 |
| `FRAME_FORMAT_BGR24` | `NioFormat::BGR` | |
| `FRAME_FORMAT_RGB24` | `NioFormat::RGB` | |
| `FRAME_FORMAT_YUV422` | `NioFormat::YUYV` | 语义等价 |

已实现于 `app/driver/robosense/nio_rs_adapter.hpp`:
```cpp
NioFormat rsFrameFormatToNio(frame_format_t fmt);
frame_format_t nioFormatToRsFrameFormat(NioFormat fmt);
```

---

## 6 集成架构设计

### 6.1 新增文件（已实现）

```
app/driver/robosense/
  ├── nio_rs_adapter.hpp       # RS frame_format ↔ NioFormat / ImuData ↔ NioImuSample
  ├── nio_rs_frame_adapter.hpp # PointCloudMsg+ImageData+ImuData → NioFrameSet
  ├── nio_rs_device.hpp        # RsDevice, RsPipeline, RsContext 实现 NioDevice/NioPipeline/NioContext
  └── nio_rs_device.cpp        # 上述实现
```

### 6.2 类图

```
              ┌─────────────┐
              │ NioDevice   │ (abstract, 已存在)
              └──────┬──────┘
                     │ implements
            ┌────────┴────────┐
            ▼                 ▼
     ┌────────────┐    ┌────────────┐
      │ ObDevice   │    │ RsDevice   │
      │(已存在)    │    │(已实现)    │
     └────────────┘    └────────────┘

              ┌─────────────┐
              │ NioPipeline │ (abstract, 已存在)
              └──────┬──────┘
                     │ implements
            ┌────────┴────────┐
            ▼                 ▼
     ┌────────────┐    ┌────────────┐
      │ ObPipeline │    │ RsPipeline │
      │(已存在)    │    │(已实现)    │
     └────────────┘    └────────────┘

              ┌─────────────┐
              │ NioContext  │ (abstract, 已存在)
              └──────┬──────┘
                     │ implements
            ┌────────┴────────┐
            ▼                 ▼
     ┌────────────┐    ┌────────────┐
      │ ObContext  │    │ RsContext  │
      │(已存在)    │    │(已实现)    │
     └────────────┘    └────────────┘
```

### 6.3 RsPipeline 数据流

```
  RoboSense rs_driver (driver threads)            User threads
  ┌──────────────────────────────────────┐
  │  LidarDriver<PointCloudMsg>          │
  │                                      │
  │  USB recv ──► decodePcPkt()          │
  │                │ cb_put_cloud_       │
  │                ▼                     │
  │          stuffed_cloud_queue  ─────────► RsPipeline::processCloud()
  │                                          │ 构建 NioFrame(DEPTH)
  │  USB recv ──► decodeImagePkt()          │
  │                │ cb_put_image_          │ if color + depth both ready:
  │                ▼                        │   组装 NioFrameSet
  │          stuffed_image_data_queue ──────►   videoCallback_(nioFs)
  │                                          │ 回收 msg 到 free queues
  │  HID recv ──► decodeImuPkt()            │
  │                │ cb_put_imu_            │
  │                ▼                        │
  │          stuffed_imu_queue ─────────────► RsPipeline::processImu()
  └──────────────────────────────────────┐     │ 格式化 CSV → imuCallback_
                                         │     │
                                         ▼     ▼
                                   ┌──────────────────┐
                                   │ VideoFrameQueue   │  ImuFrameQueue
                                   │ (cap=8, drop-oldest)│ (cap=32)
                                   └────────┬─────────┘
                                            │
                                   ┌────────▼─────────┐
                                   │ Video Consumer    │  ← 现有，无修改
                                   │ Thread            │
                                   └──────────────────┘
```

### 6.4 关键设计: RS-AC1 的 FrameSet 合成

OB SDK 的 `ob::Pipeline::start()` 在单次回调中返回聚合后的 `FrameSet`（包含 color+depth+IR 对齐帧）。RS-AC1 通过 3 条独立路径到达，需在 RsPipeline 内合成。

**方案**: 基于 `std::mutex` + `std::condition_variable` 的等待-通知机制:

```cpp
// RsPipeline 内部
struct FrameSyncState {
    std::mutex mtx;
    std::condition_variable cv;
    std::shared_ptr<NioFrame> colorFrame;
    std::shared_ptr<NioFrame> depthFrame;
    bool colorReady = false;
    bool depthReady = false;
};

void RsPipeline::processCloud(std::shared_ptr<PointCloudMsg> msg) {
    auto depthFrame = rsDepthToNioFrame(msg);  // PointCloud → NioFrame
    std::lock_guard<std::mutex> lk(syncState_.mtx);
    syncState_.depthFrame = depthFrame;
    syncState_.depthReady = true;
    tryEmitFrameSet();
}

void RsPipeline::processImageData(std::shared_ptr<ImageData> msg) {
    auto colorFrame = rsImageToNioFrame(msg);   // ImageData → NioFrame
    std::lock_guard<std::mutex> lk(syncState_.mtx);
    syncState_.colorFrame = colorFrame;
    syncState_.colorReady = true;
    tryEmitFrameSet();
}

void RsPipeline::tryEmitFrameSet() {
    // 在持有 syncState_.mtx 的情况下调用
    if (syncState_.colorReady && syncState_.depthReady) {
        auto nioFs = std::make_shared<NioFrameSet>();
        nioFs->setFrame(NioFrameType::COLOR, *syncState_.colorFrame);
        nioFs->setFrame(NioFrameType::DEPTH, *syncState_.depthFrame);
        // nativeFrameSet = nullptr (RS 路径无 ob::FrameSet)
        videoCallback_(nioFs);
        syncState_.colorReady = false;
        syncState_.depthReady = false;
    }
}
```

**并发策略**:
- 彩色帧 @30fps, 深度帧 @10fps → ~10Hz 合成 FrameSet 输出
- 深度帧到达时彩色可能已更新 3 帧; 使用最新彩色帧（覆盖式更新）
- 如只启用彩色或只启用深度，单独帧直接作为 FrameSet 输出

### 6.5 RS-AC1 Depth → NioFrame 转换策略

RS-AC1 的"深度"是 3D 点云（`PointXYZIRT`），不是 OB 的 2D depth map。需要选择:

| 策略 | NioFrame::data 内容 | 兼容性 | 代价 |
|------|---------------------|--------|------|
| **A: 合成 2D depth map** | `uint16_t[96×288]`, `depthScale=5.0f` (5mm) | 与现有 `DepthFrameConsumer` + `DepthRawTask` 完全兼容 | 点云信息损失（无方向向量、无 intensity）；96×288 分辨率低于 OB 的 640×576 |
| **B: 保存原始 XYZI 浮点** | `float[N*4]` (x,y,z,intensity) | 需要 `NioFrame::format=POINT`; 新增 `NioFormat::POINT` 已存在 | 现有 H264Encoder 无法编码浮点点云；需新增 PCD/BIN writer |
| **C: 两路并存** | depth map 放 `NioFrameType::DEPTH`; 点云放 `NioFrameType::POINT` | 最完整 | 数据量翻倍；下游需新增 POINT 流消费者 |

**推荐方案 A**（合成 2D depth map）:
- 从 `PointCloudMsg` 重建 `uint16_t` depth map: `depth[i] = (uint16_t)(distance / 0.005)`，无效点 `depth[i] = 0`
- `NioFrame` 元数据: `format=NioFormat::Y16, width=96, height=288, depthScale=5.0f`
- **注意**: `depthScale=5.0f` 表示 5mm/unit（与 OB 的 `scale=1.0f` 即 1mm/unit 或 `0.001f` 即 1m 单位不同）。下游 `FusionStreamTask` 依赖 `depthScale` 做 colormap，需兼容
- 点云的 **intensity** 信息可通过单独的 `NioFrame(NioFrameType::IR)` 传递（Y8 格式，AC1 的 intensity 相当于 IR 反射率），复用现有 IR 编码路径

**备选**: 如需保留完整 3D，可在 Phase 5 后扩展方案 C。

### 6.6 NioPipeline 接口扩展

当前 `NioPipeline` 接口缺少 RS-AC1 需要的功能:

```diff
 class NioPipeline {
 public:
+    // RS-AC1: 无独立 IR 传感器
+    virtual bool hasIRSensor() const { return true; }
+
+    // RS-AC1: 点云分辨率（而非 2D depth 分辨率）
+    virtual bool isPointCloudDepth() const { return false; }
+
+    // RS-AC1: D2C 模式 — 始终 HW
+    virtual NioAlignMode getAlignMode() const = 0;
+
     // ... 现有接口 ...
 };
```

`RsPipeline` 覆盖:
- `hasIRSensor() → false`
- `isPointCloudDepth() → true`
- `getAlignMode() → NioAlignMode::HW`

`CaptureSession` 在 `enumerateSensors()` 中查询这些方法决定是否创建 IR consumer。

---

## 7 配置与迁移

### 7.1 CLI 参数

> RS-AC1 设备在启用 `ENABLE_RS_AC1` 编译选项后自动发现，无需额外 CLI 参数。
> 使用 `-c <name>` 按设备名子串过滤（如 `-c AC1` 仅录制 RS-AC1 设备）。

### 7.2 设备发现（已实现）

当前 `main()` 使用 `discoverDevices()` 工厂函数，内部根据 `ENABLE_ORBBEC` / `ENABLE_RS_AC1` 编译宏自动发现对应设备：

```cpp
// nio_driver_factory.cpp
std::vector<DiscoveredDevice> discoverDevices() {
    std::vector<DiscoveredDevice> result;

#ifdef ENABLE_ORBBEC
    ObContext obCtx;
    // ... enumerate Orbbec devices ...
#endif

#ifdef ENABLE_RS_AC1
    RsContext rsCtx;
    // ... enumerate RoboSense devices ...
#endif

    return result;
}
```

### 7.3 多设备共存

- OB 和 RS-AC1 设备可同时运行，各自有独立的 `CaptureSession`
- RS-AC1 不产生 IR 流, 不影响 OB 设备的 IR 录制
- 同一 SDL 窗口可同时显示 OB 和 RS-AC1 的彩色/深度预览（各自占一个 viewer slot）

---

## 8 RsPipeline 实现细节

### 8.1 完整生命周期

```cpp
class RsPipeline : public NioPipeline {
public:
    bool start(NioVideoCallback callback) override {
        videoCallback_ = callback;

        // 1. 配置 RSDriverParam
        RSDriverParam param;
        param.input_type = InputType::USB;
        param.lidar_type = LidarType::RS_AC1;
        param.input_param.enable_image = true;
        param.input_param.image_format = nioFormatToRsFrameFormat(imageFormat_);
        param.input_param.image_width = imageWidth_;
        param.input_param.image_height = imageHeight_;
        param.input_param.image_fps = imageFps_;
        param.input_param.imu_fps = imuFps_;
        if (!deviceUuid_.empty())
            param.input_param.device_uuid = deviceUuid_;

        // 2. 注册 dual callbacks
        driver_.regPointCloudCallback(
            /*get=*/ [this]() -> std::shared_ptr<PointCloudMsg> {
                auto msg = freeCloudQueue_.pop();
                return msg ? msg : std::make_shared<PointCloudMsg>();
            },
            /*put=*/ [this](std::shared_ptr<PointCloudMsg> msg) {
                stuffedCloudQueue_.push(msg);
            }
        );
        driver_.regImageDataCallback(
            /*get=*/ [this]() -> std::shared_ptr<ImageData> {
                auto msg = freeImageQueue_.pop();
                return msg ? msg : std::make_shared<ImageData>();
            },
            /*put=*/ [this](std::shared_ptr<ImageData> msg) {
                stuffedImageQueue_.push(msg);
            }
        );

        // 3. Init + Start
        if (!driver_.init(param)) return false;

        // 4. 启动处理线程（在 driver.start() 之前）
        cloudThread_ = std::thread(&RsPipeline::processCloud, this);
        imageThread_ = std::thread(&RsPipeline::processImageData, this);

        if (!driver_.start()) return false;
        return true;
    }

    void stop() override {
        driver_.stop();
        cloudThread_.join();
        imageThread_.join();
    }
};
```

### 8.2 处理线程

```cpp
void RsPipeline::processCloud() {
    while (running_) {
        auto msg = stuffedCloudQueue_.popWait(100);
        if (!msg) continue;
        auto depthFrame = rsDepthToNioFrame(msg);
        {
            std::lock_guard<std::mutex> lk(syncMtx_);
            depthFrame_ = depthFrame;
            depthReady_ = true;
            tryEmitFrameSet();
        }
        freeCloudQueue_.push(msg);  // 回收
    }
}

void RsPipeline::processImageData() {
    while (running_) {
        auto msg = stuffedImageQueue_.popWait(100);
        if (!msg) continue;
        auto colorFrame = rsImageToNioFrame(msg);
        {
            std::lock_guard<std::mutex> lk(syncMtx_);
            colorFrame_ = colorFrame;
            colorReady_ = true;
            tryEmitFrameSet();
        }
        freeImageQueue_.push(msg);  // 回收
    }
}
```

### 8.3 RS → Nio 转换函数

```cpp
// nio_rs_frame_adapter.hpp

NioFrame rsDepthToNioFrame(const std::shared_ptr<PointCloudMsg>& cloud) {
    NioFrame f;
    f.type = NioFrameType::DEPTH;
    f.format = NioFormat::Y16;
    f.width = POINT_WIDTH_NUMS;   // 96
    f.height = POINT_HEIGHT_NUMS; // 288
    f.depthScale = 5.0f;          // 0.005m * 1000 = 5mm/unit
    f.timestampUs = static_cast<uint64_t>(cloud->timestamp * 1e6);
    f.data.resize(96 * 288 * 2);  // uint16_t per pixel

    uint16_t* dst = reinterpret_cast<uint16_t*>(f.data.data());
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& pt = cloud->points[i];
        float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (std::isnan(pt.x) || dist < 0.2f) {
            dst[i] = 0;  // 无效点
        } else {
            uint16_t raw = static_cast<uint16_t>(dist / 0.005f);
            dst[i] = (raw > 40000) ? 0 : raw;  // 200m / 0.005 = 40000
        }
    }
    return f;
}

NioFrame rsImageToNioFrame(const std::shared_ptr<ImageData>& img) {
    NioFrame f;
    f.type = NioFrameType::COLOR;
    f.format = rsFrameFormatToNio(img->frame_format);
    f.width = img->width;
    f.height = img->height;
    f.timestampUs = static_cast<uint64_t>(img->timestamp * 1e6);
    f.data.assign(img->data.get(), img->data.get() + img->data_bytes);
    return f;
}

std::vector<NioImuSample> rsImuToNioSamples(const std::shared_ptr<ImuData>& imu) {
    std::vector<NioImuSample> samples;
    NioImuSample accel;
    accel.type = NioFrameType::ACCEL;
    accel.timestampUs = static_cast<uint64_t>(imu->timestamp * 1e6);
    accel.x = imu->linear_acceleration_x;
    accel.y = imu->linear_acceleration_y;
    accel.z = imu->linear_acceleration_z;
    accel.temperature = 0.0f;  // RS-AC1 不提供温度
    samples.push_back(accel);

    NioImuSample gyro;
    gyro.type = NioFrameType::GYRO;
    gyro.timestampUs = static_cast<uint64_t>(imu->timestamp * 1e6);
    gyro.x = imu->angular_velocity_x;
    gyro.y = imu->angular_velocity_y;
    gyro.z = imu->angular_velocity_z;
    gyro.temperature = 0.0f;
    samples.push_back(gyro);

    return samples;
}
```

### 8.4 RsDevice 实现

| NioDevice 方法 | 实现 |
|----------------|------|
| `getDeviceInfo()` | VID=0x3840, PID=0x1010, name="RoboSense_AC1", SN=来自 `driver.getDeviceInfo()` 的 SN, connectionType="USB3.0" |
| `timerSyncWithHost()` | 空操作（RS-AC1 在 driver 内部做 PTP，无需外部调用） |
| `isGlobalTimestampSupported()` | 返回 `true` |
| `enableGlobalTimestamp(bool)` | 空操作 |
| `getSensorInfo()` | 固定: hasColor=true, hasDepth=true, hasIR=false, hasAccel=true, hasGyro=true; 深度 96×288@10fps Y16; 彩色 1920×1080@30fps NV12 |
| `getIntProperty(int)` | 不支持; 返回 0 |

### 8.5 RsContext 实现

```cpp
class RsContext : public NioContext {
public:
    RsContext() { libusb_init(&usbCtx_); }

    uint32_t getDeviceCount() override {
        // 扫描 VID=0x3840, PID=0x1010 的 USB 设备
        libusb_device **list;
        ssize_t cnt = libusb_get_device_list(usbCtx_, &list);
        uint32_t ac1Count = 0;
        for (ssize_t i = 0; i < cnt; i++) {
            auto dev = list[i];
            auto desc = libusb_get_device_descriptor(dev, nullptr);
            if (desc.idVendor == 0x3840 && desc.idProduct == 0x1010) {
                ac1Count++;
            }
        }
        libusb_free_device_list(list, 1);
        return ac1Count;
    }

    std::shared_ptr<NioDevice> getDevice(uint32_t index) override {
        // 返回 RsDevice，封装 device_uuid 用于多设备选择
        return std::make_shared<RsDevice>(index);
    }
};
```

---

## 9 Fusion (D2C 合成) 适配

### 9.1 OB 现有路径

| 模式 | 触发 | 实现 |
|------|------|------|
| HW D2C | `checkHWD2CAlign()` 返回 true | SDK 硬件做对齐; `FusionStreamTask` 直接读 color+depth |
| SW D2C | HW 不可用时 | `ob::Align::process(obFrameSet)` 在 fusion 线程中做对齐 |

### 9.2 RS-AC1 路径

- RS-AC1 **始终为 HW D2C** — 点云已对齐到图像
- `RsPipeline::getAlignMode() → NioAlignMode::HW`
- `FusionStreamTask` 在 HW D2C 分支可直接使用（color 和 depth 分辨率不同时需要 resize, 但色彩混合逻辑不变）
- **特殊**: RS-AC1 depth 为 96×288, color 为 1920×1080; jet colormap 上采样到 1080p 后与 color alpha-blend
- `nativeFrameSet` 为 `nullptr`（RS 路径无 `ob::FrameSet`），SW D2C 分支不适用

### 9.3 known issue

RS-AC1 的 depth map 分辨率仅 96×288，上采样到 1080p 后融合图像质量较低（可见深度块效应）。这是 AC1 传感器的物理限制，非软件可修复。

---

## 10 IMU 路线

| 步骤 | OB 路径 | RS-AC1 路径 |
|------|---------|------------|
| 1. 回调 | `ob::Pipeline::start(imuConfig, onImuFrameSet)` | `driver.regImuDataCallback(get, put)` |
| 2. 数据到达 | `ob::FrameSet` 含 AccelFrame + GyroFrame | `ImuData` 含 6-axis + timestamp |
| 3. 格式化 | `onImuFrameSet()` → CSV → `imuQueue_.push()` | `RsPipeline::processImu()` → CSV → `imuCallback_()` |
| 4. 消费者线程 | `imuConsumerLoop()` pop → `imuTask_->enqueueLine()` | 同 |
| 5. 文件写入 | `ImuStreamTask::processFrame()` → `_imu_<ts>.txt` | 同; CSV 格式一致: `host_ts_ms,ACCEL,device_ts_us,x,y,z,temperature` |

### 差异

| | OB | RS-AC1 |
|--|-----|--------|
| 频率 | 由 sensor profile 决定（通常 100/200 Hz） | `input_param.imu_fps` (100 或 200) |
| 温度 | 从 `AccelFrame::temperature()` 可获取 | **不提供**; 填 0.0 |
| 单位 | Accel: m/s², Gyro: rad/s | 同 |
| 独立 pipeline | 是 (`imuPipeline_`) | 否 — HID IMU 与点云/图像共用 `LidarDriver` |

---

## 11 构建集成

### 11.1 CMake 变更

> **已实现**: CMake 选项已移至根 `CMakeLists.txt`，C++14 升级已完成。
> 以下为原始设计记录，实际实现略有调整。

```cmake
# 根 CMakeLists.txt (已实现)
option(ENABLE_ORBBEC "Enable Orbbec device support" ON)
option(ENABLE_RS_AC1 "Enable RoboSense RS-AC1 support" ON)

if(NOT ENABLE_ORBBEC AND NOT ENABLE_RS_AC1)
    message(FATAL_ERROR "Both ENABLE_ORBBEC and ENABLE_RS_AC1 are OFF. "
                        "Enable at least one vendor SDK.")
endif()

if(ENABLE_ORBBEC)
    set(OB_BUILD_MAIN_PROJECT ON CACHE INTERNAL "" FORCE)
    set(OB_SDK_LIB_NAME "OrbbecSDK" CACHE INTERNAL "" FORCE)
    add_subdirectory(vendors/OrbbecSDK)
endif()

if(ENABLE_RS_AC1)
    set(RS_DRIVER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vendors/RoboSense" CACHE PATH "" FORCE)
    set(DISABLE_PCAP_PARSE ON CACHE INTERNAL "" FORCE)
    set(COMPILE_DEMOS OFF CACHE INTERNAL "" FORCE)
    set(COMPILE_TOOLS OFF CACHE INTERNAL "" FORCE)
    set(ENABLE_USB ON CACHE INTERNAL "" FORCE)
    add_subdirectory(vendors/RoboSense)
endif()

add_subdirectory(app)
```

Source-specific compile definitions and link libraries are set in
`app/driver/CMakeLists.txt` within `if(ENABLE_RS_AC1)` / `if(ENABLE_ORBBEC)` blocks.
```

### 11.2 C++ 标准升级（已完成）

- 项目已从 C++11 升级至 **C++14**（根 `CMakeLists.txt` 中 `CMAKE_CXX_STANDARD 14`）
- rs_driver 要求 C++14，升级已完成且通过构建验证

### 11.3 内核 UVC 解绑自动化

建议在应用启动时自动检测并解绑:

```cpp
// nio_rs_device.cpp
static bool unbindUvcDriver(uint16_t vid, uint16_t pid) {
    // 遍历 /sys/bus/usb/devices/*/idVendor / idProduct
    // 匹配 vid:pid 后 echo到 unbind
    // 需 root 权限
}
```

**caveat**: 需要 `sudo`; 不建议在应用内部做; 而应由部署脚本或 udev rule 处理。

---

## 12 回滚方案

| 回滚步骤 | 命令/动作 |
|----------|-----------|
| 禁用 RS-AC1 | cmake 时 `-DENABLE_RS_AC1=OFF`; 代码中 `#ifdef ENABLE_RS_AC1` 包裹所有 RS 相关 include/class |
| 完全移除 | 删除 `app/driver/robosense/nio_rs_*.hpp/.cpp`; 删除根 CMakeLists.txt 和 app/driver/CMakeLists.txt 中 `if(ENABLE_RS_AC1)` 块 |
| 回退 C++ 标准 | `set(CMAKE_CXX_STANDARD 11)` — 仅当 RS-AC1 完全移除时安全 |

---

## 13 故障排查

| 症状 | 可能原因 | 排查方式 |
|------|---------|---------|
| `DEVICE_DISCONNECTED` 错误 | UVC 内核驱动抢占 RS-AC1 | `lsusb -t` 查看 AC1 是否被 `uvcvideo` 绑定; 解绑后重试 |
| RS-AC1 找不到设备 | USB 2.0 插口 | `lsusb -v -d 3840:1010 \| grep bcdUSB`; 需要 ≥ 3.00 |
| 点云数据全为 0 | `min_distance/max_distance` 过滤过严 | 调整 `decoder_param.min_distance` / `max_distance` |
| 图像花屏 | 格式不匹配 | 确认 `image_format` 与实际 UVC 流格式一致; NV12 默认值最安全 |
| IMU 数据不到达 | 未定义 `ENABLE_IMU_PARSE` | 编译时确保 `-DENABLE_IMU_PARSE` |
| 多 AC1 冲突 | `device_uuid` 未指定 | 为每个 `RSDriverParam.input_param.device_uuid` 设置 USB serial |
| USBFS 内存不足 | 多设备并发 | `cat /sys/module/usbcore/parameters/usbfs_memory_mb`; 需要 ≥ 128 |
| C++ 编译错误 (lambda init-capture) | C++ 标准过低 | 确认 CMake 设置 `CMAKE_CXX_STANDARD=14`（已完成） |
| depthScale 不匹配 | OB=1mm/unit, AC1=5mm/unit | 检查 `NioFrame::depthScale`; fusion colormap 使用 `rawVal * scale / 1000.0` |

---

## 14 实现路线图

| 阶段 | 内容 | 依赖 | 预估工作量 |
|------|------|------|-----------|
| **Phase 4a** | 扩展 `NioDevice`/`NioPipeline` 接口（hasIRSensor, isPointCloudDepth, getAlignMode） | Phase 3 完成 | ✅ 完成 |
| **Phase 4b** | 实现 `RsDevice`, `RsPipeline`, `RsContext` | Phase 4a | ✅ 完成 |
| **Phase 4c** | 实现 `RsFrameAdapter` (rsDepthToNioFrame, rsImageToNioFrame, rsImuToNioSamples) | Phase 4b | ✅ 完成 |
| **Phase 4d** | CMake 集成 + 构建验证 (C++14 升级, rs_driver subdirectory) | Phase 4c | ✅ 完成 |
| **Phase 4e** | `CaptureSession` 改用 `NioPipeline` (替换 `ob::Pipeline`); 按 pipeline 类型分发 | Phase 3 完整 + Phase 4d | ✅ 完成 |
| **Phase 4f** | `main()` 多设备发现分流 (OB + RS-AC1) → `discoverDevices()` 工厂 | Phase 4e | ✅ 完成 |
| **Phase 5a** | RS-AC1 + OB 双设备功能测试 | Phase 4f | ✅ 完成 |

---

## 15 未解决与待确认项

| 项目 | 状态 | 需要确认 |
|------|------|---------|
| RS-AC1 depthScale 精度 | ✅ 已确认 | OB 管线使用 `scale/1000.0` 得到米; AC1 使用 `distance*0.005`; NioFrame 中 encoding 为 uint16 `depthScale=5.0f`, 后者计算为 `rawVal * 5.0f / 1000.0 = rawVal * 0.005`, 与原始 `distance = rawVal * 0.005` 一致 |
| C++14 升级影响 | ✅ 已完成 | 已在根 CMakeLists.txt 升级至 C++14，构建通过 |
| udev 规则 | ✅ 已部署 | `/etc/udev/rules.d/99-robosense-ac1.rules` |
| RS-AC1 多机并行 | ✅ 已支持 | 同一台主机上 2+ 个 AC1 使用 `device_uuid` (USB serial) 区分 |
| rs_driver 编译为独立 .so | ✅ 已确定 | rs_driver 编译为 `usb-ac-static` + `uvc-ac-static` 静态库, 与 OrbbecSDK .so 共存无冲突 |
| PCD/BIN 点云保存 | 不在本方案范围 | 如需保存原始 3D 点云文件, 需额外开发; 方案 A 的 depth map 不含 intensity 和方向向量 |
| RS-AC1 IR-Left/IR-Right | 不存在 | AC1 无独立 IR 流; IR 编码路径不应为 AC1 创建 |
| Fusion 可视化质量 | 受限 | 96×288 上采样到 1080p 的块效应; 可考虑调整 `depthMinM`/`depthMaxM` 范围以改善 colormap 映射 |

---

## 16 文件输出格式对照

RS-AC1 的输出文件与 OB 完全一致:

```
<saveDir>/<session_timestamp>/
  RoboSense_AC1_<uuid>/
    RoboSense_AC1_<uuid>_color_<ts>.h264          # 与 OB 相同
    RoboSense_AC1_<uuid>_depth_<ts>.h264           # 与 OB 相同 (Y16→H264)
    RoboSense_AC1_<uuid>_depth_raw_<ts>.raw        # 与 OB 相同 (96×288 × 2B = 55,296B)
    RoboSense_AC1_<uuid>_imu_<ts>.txt              # 与 OB 相同 CSV 格式
    RoboSense_AC1_<uuid>_depth_intrinsic_<ts>.json # 无内参 (方向向量模式)
                                                    # 写入 {"depth_scale": 5.0, "width": 96, "height": 288}
    # 以下文件不存在 (AC1 无 IR):
    # _ir_<ts>.h264
    # _ir_left_<ts>.h264
    # _ir_right_<ts>.h264
    # _d2c_fused_<ts>.h264            ← 可选; HW D2C 可做
```

---

## 17 线程模型对照

| 线程 | OB (per device) | RS-AC1 (per device) |
|------|-----------------|---------------------|
| SDK 内部 | `OB_VIDEO_*`, `OB_IMU_*` | `recv_thread_`, `_usb_thread`, UVC image/depth isoc callbacks |
| SDK 解码 | (SDK internal) | `handle_thread_` (IMU), `handle_thread_2_` (image), `handle_thread_3_` (depth) |
| SDK→Nio 适配 | video callback (inline) | `RsPipeline::processCloud`, `RsPipeline::processImageData` |
| Video 消费者 | `<dev>_vcons` | `<dev>_vcons` (不变) |
| IMU 消费者 | `<dev>_icons` | `<dev>_icons` (不变) |
| Encode workers | N × `<dev>_*_enc` | N × `<dev>_*_enc` (不变, 但无 IR workers) |
| Fusion worker | `<dev>_fusion` | `<dev>_fusion` (不变) |

**总计 per RS-AC1 device**: ~3 SDK threads + 3 adapter threads + 1 video consumer + 1 IMU consumer + ~4 encode/raw workers + 1 fusion = ~13 threads。

---

## 附注: 源码引用

| 引用 | 路径 |
|------|------|
| RS-AC1 USB demo | `vendors/RoboSense/demo/demo_usb.cpp` |
| RS-AC1 decoder | `vendors/RoboSense/src/rs_driver/driver/decoder/decoder_RSAC1.hpp` |
| RS USB input | `vendors/RoboSense/src/rs_driver/driver/input/input_usb.hpp/.cpp` |
| RS driver param | `vendors/RoboSense/src/rs_driver/driver/driver_param.hpp` |
| RS LidarDriver API | `vendors/RoboSense/src/rs_driver/api/lidar_driver.hpp` |
| RS ImageData | `vendors/RoboSense/src/rs_driver/msg/image_data_msg.hpp` |
| RS ImuData | `vendors/RoboSense/src/rs_driver/msg/imu_data_msg.hpp` |
| RS PointCloudMsg | `vendors/RoboSense/src/rs_driver/msg/point_cloud_msg.hpp` |
| Nio types | `app/core/nio_types.hpp` |
| Nio frame | `app/core/nio_frame.hpp` |
| Nio device | `app/core/nio_device.hpp` |
| Nio OB device | `app/driver/orbbec/nio_ob_device.hpp/.cpp` |
| Nio OB adapter | `app/driver/orbbec/nio_ob_adapter.hpp` |
| Nio OB frame adapter | `app/driver/orbbec/nio_ob_frame_adapter.hpp` |
| Nio RS device | `app/driver/robosense/nio_rs_device.hpp/.cpp` |
| Nio RS frame adapter | `app/driver/robosense/nio_rs_frame_adapter.hpp` |
| CaptureSession | `app/capture/nio_capture_session.hpp/.cpp` |
| FusionStreamTask | `app/capture/nio_stream_tasks.hpp/.cpp` |
| Driver factory | `app/driver/nio_driver_factory.hpp/.cpp` |
| app 入口 | `app/nio_multi_capture/nio_multi_capture.cpp` |
