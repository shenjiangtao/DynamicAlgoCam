# 产品技术分析：RoboSense AC1 vs Orbbec Gemini 335L/336L

> 基于代码实现、官方规格书、实时采集测试的综合技术评估
>
> 测试日期：2026-06-26 | 测试环境：USB 3.0, Ubuntu Linux, x86_64
>
> 测试设备：Orbbec Gemini 335L (PID 0804) + RoboSense RoboX AC1 (PID 1010)

---

## 目录

1. [产品概述与定位](#1-产品概述与定位)
2. [深度感知原理对比](#2-深度感知原理对比)
3. [硬件规格对比](#3-硬件规格对比)
4. [SDK/驱动架构对比](#4-sdk驱动架构对比)
5. [采集系统代码架构](#5-采集系统代码架构)
6. [实时采集测试结果](#6-实时采集测试结果)
7. [输出数据格式对比](#7-输出数据格式对比)
8. [D2C（深度对齐彩色）策略](#8-d2c深度对齐彩色策略)
9. [IMU 数据对比](#9-imu-数据对比)
10. [集成复杂度评估](#10-集成复杂度评估)
11. [适用场景评估](#11-适用场景评估)
12. [总结与建议](#12-总结与建议)
13. [参考来源](#13-参考来源)

---

## 1. 产品概述与定位

### 1.1 Orbbec Gemini 335L

- **厂商**：Orbbec（奥比中光），全球 3D 视觉传感器领军企业，2023 年科创板上市（688322.SH）
- **产品线**：Gemini 3 系列，包含 335L（短中距）和 336L（长距）
- **定位**：室内近距离 3D 感知，机器人导航、人脸识别、手势交互、AR/VR
- **技术路线**：双 IR 结构光（散斑结构光 + 双目立体匹配）
- **官方页面**：https://www.orbbec.com/products/gemini-335l
- **VID:PID**：`2BC5:0804`

### 1.2 Orbbec Gemini 336L

- **定位**：335L 长距版本，相同硬件架构，激光器功率更高，有效距离扩大
- **官方页面**：https://www.orbbec.com/products/gemini-336l
- **VID:PID**：`2BC5:0807`

### 1.3 RoboSense RoboX AC1

- **厂商**：RoboSense（速腾聚创），激光雷达行业领先，2024 年港股上市（2498.HK)
- **产品线**：Active Camera 系列，首款集成 LiDAR + RGB 的"主动相机"
- **定位**：室外中远距离感知，自动驾驶路侧感知、智慧交通、人员/车辆检测
- **技术路线**：固态 Flash LiDAR + RGB 相机，硬件级融合
- **官方页面**：https://www.robosense.ai/ac1
- **VID:PID**：`3840:1010`

### 1.4 核心定位差异

| 维度 | Gemini 335L/336L | RoboX AC1 |
|------|-----------------|-----------|
| **核心传感方式** | 光学结构光深度相机 | LiDAR-RGB 融合主动相机 |
| **有效距离** | 0.25–5 m（335L）/ 0.5–10 m（336L） | 0.2–200 m |
| **深度分辨率** | 高（640×480 ~ 1280×800） | 低（96×288 = 27,648 点） |
| **深度精度** | 毫米级（±1–3mm @1m） | 厘米级（±2–3cm @10m） |
| **防护等级** | IP65 | IP65 |
| **输出类型** | 2D 深度图（Y16） | 3D 点云（PointXYZIRT）+ 合成深度图 |
| **目标应用** | 室内机器人/AR/人机交互 | 室外感知/交通/安防 |

---

## 2. 深度感知原理对比

### 2.1 Orbbec 335L/336L：双 IR 结构光

```
              IR 投射器 (左)                IR 投射器 (右)
                    \                        /
                     \   随机散斑图案        /
                      \____________________/
                               |
                               v
                    +--------------------+
                    |    被测物体表面      |
                    |  (散斑被深度调制)    |
                    +--------------------+
                               |
                    IR 左相机               IR 右相机
                    (接收散斑)              (接收散斑)
                         \                    /
                          \   立体匹配/三角测距
                           v
                    +--------------------+
                    |    深度图 Y16       |
                    +--------------------+
```

**原理详解**：

1. **主动结构光投射**：左侧 IR 投射器发射经 DOE（衍射光学元件）编码的随机散斑图案，右侧 IR 投射器可额外投射补光/编码图案
2. **双目立体匹配**：左右 IR 相机同步采集散斑图像，通过立体匹配算法计算视差图
3. **三角测距**：视差 → 深度，基于标定的基线长度和焦距：`Z = f × B / d`（f=焦距, B=基线, d=视差）
4. **深度图输出**：Y16 格式（uint16），每个像素表示距离值 = `raw × depthScale`

**代码验证** (`app/driver/orbbec/nio_ob_device.cpp:87-290`)：
- `OB_SENSOR_IR_LEFT` + `OB_SENSOR_IR_RIGHT` → 双 IR 传感器
- `OB_SENSOR_DEPTH` → SDK 内部完成匹配，直接输出深度图
- `depthScale` 由 `OB_PROP_DEPTH_PRECISION_LEVEL_INT` 查询：0=1mm, 1=0.5mm, 2=0.25mm, 3=0.1mm
- 实测 335L `depthScale = 0.001`（1mm 精度）

### 2.2 RoboSense AC1：Flash LiDAR + 点云合成深度

```
           Flash LiDAR 发射器
                   |
           短脉冲近红外激光
           (905nm VCSEL 阵列)
                   |
                   v
         +--------------------+
         |    场景中多目标     |
         |  各点反射回波       |
         +--------------------+
                   |
          SPAD 接收阵列 (96×288)
          (单光子雪崩二极管)
                   |
            ToF 测距 + 方向编码
                   |
                   v
         +--------------------+
         |  点云 PointXYZIRT  |
         |  27,648 pts/frame  |
         +--------------------+
                   |
          rsDepthToNioFrame()
          (点云→2D 深度图投影)
                   v
         +--------------------+
         |  合成深度图 Y16     |
         |  96×288 @10fps      |
         +--------------------+
```

**原理详解**：

1. **Flash 发射**：905nm VCSEL 阵列发射极短脉冲近红外激光，照亮整个 FOV
2. **SPAD 接收**：96×288 SPAD（单光子雪崩二极管）阵列接收回波光子，每个像素独立做 ToF 测距
3. **方向编码**：每个 SPAD 像素对应固定的角度方向（通过 `vector_base=32768` 归一化），无需扫描即可获得 3D 点云
4. **点云输出**：`PointXYZIRT`（x, y, z, intensity, ring, timestamp），每帧 27,648 个点
5. **RGB 同步**：独立 RGB 传感器通过硬件同步与 LiDAR 共享时间戳，出厂标定外参

**代码验证** (`app/driver/robosense/nio_rs_frame_adapter.hpp:22-65`)：
- `RS_AC1_DEPTH_WIDTH = 96, RS_AC1_DEPTH_HEIGHT = 288`
- `RS_AC1_DEPTH_SCALE = 5.0` (5mm/step)
- 深度图合成逻辑：每个点 `i` → 像素 `(col=i%96, row=i/96)`，`depth = sqrt(x²+y²+z²) / 0.005`
- 无效点过滤：NaN 检测 + `<0.2m` 过滤 + `>200m` 溢出

### 2.3 原理对比总结

| 特性 | 结构光 (Orbbec 335L/336L) | Flash LiDAR (AC1) |
|------|--------------------------|-------------------|
| **发射方式** | 连续 IR 散斑 | 短脉冲 Flash |
| **接收方式** | 普通 CMOS IR 传感器 | SPAD 单光子阵列 |
| **测距原理** | 双目视差三角测距 | 直接 ToF 脉冲测距 |
| **深度输出** | 2D 深度图 (高分辨率) | 3D 点云 (低分辨率) → 合成深度图 |
| **深度精度** | 高（mm级,±1-3mm@1m） | 中（cm级,±2-3cm@10m） |
| **最大量程** | 5m/10m | 200m |
| **抗阳光干扰** | 弱（结构光被淹没） | 强（脉冲+SPAD，阳光下可用） |
| **功耗** | 低~中 | 中~高（VCSEL 脉冲） |
| **受目标影响** | 透明/反光物体差 | 低反射率物体远距差 |
| **帧率** | 10–30 fps | 10 fps 固定 |

---

## 3. 硬件规格对比

### 3.1 核心参数

| 参数 | Gemini 335L | Gemini 336L | RoboX AC1 |
|------|------------|------------|-----------|
| **VID:PID** | 2BC5:0804 | 2BC5:0807 | 3840:1010 |
| **接口** | USB 2.0/3.0 | USB 2.0/3.0 | USB 3.0 only |
| **尺寸** | 90×25×25 mm | 90×25×25 mm | 118×92×52 mm (估算) |
| **重量** | ~77g | ~77g | ~350g (估算) |
| **防护等级** | IP65 | IP65 | IP65 |
| **深度技术** | 双IR结构光 | 双IR结构光(长距) | Flash LiDAR |
| **深度范围** | 0.25–5 m | 0.5–10 m | 0.2–200 m |
| **深度 FOV** | 58°×46° (H×V) | 58°×46° | 约 120°×25° (水平×垂直) |
| **RGB 分辨率** | 1280×720 | 1280×720 | 1920×1080 |
| **RGB FOV** | 72°×51° | 72°×51° | ~120° (水平) |
| **红外传感器** | IR_LEFT + IR_RIGHT | IR_LEFT + IR_RIGHT | 无 |
| **IMU** | Accel+Gyro @200Hz | Accel+Gyro @200Hz | Accel+Gyro @100Hz |
| **RGB 格式** | MJPG/YUYV | MJPG/YUYV | NV12 |
| **深度格式** | Y16 (uint16) | Y16 (uint16) | Y16 (合成 96×288) |

### 3.2 深度分辨率对比

| 设备 | 深度分辨率 | 像素/点数量 | 深度精度 | 验证来源 |
|------|-----------|-----------|---------|---------|
| 335L | **1280×800** (实测) | 1,024,000 px | 1 mm (depthScale=0.001) | 实测 intrinsic JSON: `width:1280, height:800` |
| 336L | 640×480 ~ 1280×800 | 307,200–1,024,000 px | 0.1–1 mm | SDK 支持多档精度等级 |
| AC1 | **96×288** (合成) | 27,648 pts | 5 mm (depthScale=5.0) | 代码: `RS_AC1_DEPTH_WIDTH=96, HEIGHT=288` |

**关键发现**：335L 在 USB3.0 下实测选择了 1280×800 深度模式（而非文档中预期的 848×480），这是因为 `selectBestProfile()` 的评分逻辑偏好 1280 宽度 (+120 分) (`app/driver/orbbec/nio_ob_adapter.hpp:228`)。

### 3.3 点云规格（AC1 独有）

| 属性 | 值 | 来源 |
|------|---|------|
| 点类型 | `PointXYZIRT` | `app/driver/robosense/nio_rs_frame_adapter.hpp:34` |
| 字段 | x y z intensity ring timestamp | PCD 文件头 |
| SPAD 网格 | 96×288 = 27,648 pts/frame | `RS_AC1_DEPTH_WIDTH/HEIGHT` |
| 数据格式 | binary, 26 bytes/point (float xyz + float intensity + uint16 ring + double ts) | PCD SIZE/TYPE |
| 距离量化 | 5mm (uint16, depth = raw × 0.005m) | `RS_AC1_DEPTH_SCALE` |
| 方向编码 | vector_base=32768 (方向归一化因子) | intrinsic JSON |
| 有效距离 | 0.2–200 m | 代码过滤阈值 |
| FPS | 10 | 实测 |

---

## 4. SDK/驱动架构对比

### 4.1 OrbbecSDK v2

- **开源**：2024-10 月开源，GitHub: `orbbec/OrbbecSDK`，最新稳定版 v1.10.27
- **架构**：C++ 核心 + C API + C++/Python 包装层
- **设备发现**：`ob::Context::queryDeviceList()` — SDK 内部完成 USB 枚举和设备匹配
- **管线模型**：`ob::Pipeline` — 统一的 start/stop/config 生命周期
- **帧聚合**：`FrameAggregator` — 多传感器帧按时间戳同步聚合为 `FrameSet`
- **D2C 硬件对齐**：固件原生支持，通过 `getD2CDepthProfileList()` 检查
- **D2C 软件对齐**：`ob::Align` 类 — 基于内参+外参的深度→彩色像素映射（`AlignImpl.cpp:1533`）
- **IMU**：独立 `ob::Pipeline`，USB HID 端口采集，~200Hz
- **传感器枚举**：`device->getSensorList()` → 动态查询所有传感器类型和流配置

### 4.2 rs_driver (RoboSense)

- **开源**：GitHub: `RoboSense-LiDAR/rs_driver`，AC1 定制版 `rs_driver-dev_opt_AC1`
- **架构**：C++ 模板化驱动，`LidarDriver<T>` + `InputType::USB`
- **设备发现**：本项目 `RsContext::scanDevices()` — 手动 libusb 枚举 (`libusb_get_device_list`)，匹配 VID=0x3840/PID=0x1010
- **数据模型**：无传统管线概念，使用 get/put 双回调队列
  - `freeCloudQueue_` / `stuffedCloudQueue_` — 点云数据
  - `freeImageQueue_` / `stuffedImageQueue_` — RGB 图像
  - `freeImuQueue_` / `stuffedImuQueue_` — IMU 数据
- **帧同步**：本项目 `RsPipeline::tryEmitFrameSet()` 简单的 colorReady_+depthReady_ 标志位同步
- **D2C**：始终 HW 模式 — LiDAR 与相机出厂硬件级标定，无对齐 filter
- **IMU**：rs_driver HID 回调，~100Hz (实测)
- **传感器枚举**：硬编码 `RsDevice::getSensorInfo()` — 固定参数

### 4.3 SDK 架构差异表

| 维度 | OrbbecSDK v2 | rs_driver (AC1) |
|------|-------------|----------------|
| **成熟度** | 生产级，全功能 SDK | LiDAR 专用驱动定制版 |
| **设备发现** | `ob::Context` 自动 | libusb 手动枚举 |
| **流配置** | 动态 `getStreamProfileList()` | 硬编码参数 |
| **帧聚合** | `FrameAggregator` 时间戳同步 | 应用层标志位同步 |
| **D2C 方式** | HW+SW 双模式 | HW only (出厂标定) |
| **IMU 采集** | 独立 Pipeline | 共享 driver，HID 回调 |
| **点云** | SDK 内部 `PointCloudFilter` | 驱动直接输出原始点云 |
| **跨平台** | Linux/Windows/macOS/Android | Linux only (依赖 libusb) |
| **编译集成** | 预编译 `.so` + 头文件 | 源码集成 + `-DENABLE_USB -DENABLE_IMU_PARSE` |

---

## 5. 采集系统代码架构

### 5.1 统一抽象层

本项目通过 `NioDevice` / `NioPipeline` / `NioContext` 抽象接口屏蔽了两家 SDK 差异：

```
NioContext (抽象)
├── ObContext   — ob::Context 设备枚举
└── RsContext    — libusb 手动扫描

NioDevice (抽象)
├── ObDevice     — ob::Device 动态传感器查询
└── RsDevice     — 硬编码传感器信息

NioPipeline (抽象)
├── ObPipeline   — ob::Pipeline + ob::Config 生命周期
└── RsPipeline   — LidarDriver + 双回调队列 + 帧合成
```

### 5.2 关键实现差异

| 方法 | ObDevice/ObPipeline | RsDevice/RsPipeline |
|------|-------------------|---------------------|
| `getSensorInfo()` | 动态查询 SDK 传感器列表 | 返回硬编码 NioSensorInfo |
| `setupPipeline()` | 遍历传感器 → 选择最佳流配置 → 检查 HW D2C | 固定参数 + depthScale |
| `isPointCloudDepth()` | `false` | `true` — 触发 PCD 输出 |
| `checkHWD2CSupport()` | 查询 `getD2CDepthProfileList()` 匹配 | 始终返回 `true` |
| `start()` | `obPipeline_->start(config, callback)` | `driver_.init(param)` + `driver_.start()` + 3 个处理线程 |
| `startImu()` | 独立 `imuPipeline_->start()` | 共享 driver，IMU 线程从 `stuffedImuQueue_` 消费 |

### 5.3 数据流路径

**Orbbec 335L 数据流**：
```
ob::Pipeline → FrameSet callback
    → obFrameSetToNio() → NioFrameSet
    → VideoFrameQueue → CaptureSession::videoConsumerLoop()
        ├── ColorFrameConsumer  → H264Encoder → color.h264
        ├── DepthFrameConsumer  → H264Encoder → depth.h264
        │                       → DepthRawTask → depth_raw.raw (NIO_DEPTH_RAW)
        ├── IRFrameConsumer(L)  → H264Encoder → ir_left.h264
        ├── IRFrameConsumer(R)  → H264Encoder → ir_right.h264
        └── FusionStreamTask    → SW Align(blend) → d2c_fused.h264
```

**RoboSense AC1 数据流**：
```
LidarDriver → stuffedCloudQueue_  → processCloud()  → NioFrame(DEPTH) + NioFrame(POINT)
           → stuffedImageQueue_ → processImage() → NioFrame(COLOR)
           → stuffedImuQueue_   → processImu()    → NioImuSample[]

tryEmitFrameSet(colorReady_ && depthReady_)
    → videoCallback_ → VideoFrameQueue → CaptureSession::videoConsumerLoop()
        ├── ColorFrameConsumer     → H264Encoder → color.h264
        ├── DepthFrameConsumer     → H264Encoder → depth.h264
        │                          → DepthRawTask → depth_raw.raw (NIO_DEPTH_RAW)
        ├── PointcloudFrameConsumer → PcdStreamTask → *.pcd (NIO_POINT_CLOUD_RAW 等效)
        └── FusionStreamTask       → HW D2C(blend) → d2c_fused.h264

processImu() → imuCallback_ → ImuFrameQueue → ImuStreamTask → imu.txt
```

---

## 6. 实时采集测试结果

### 6.1 测试配置

| 项目 | 值 |
|------|---|
| 操作系统 | Ubuntu Linux x86_64 |
| USB | 3.0 (SuperSpeed) |
| 335L 连接 | Bus 004 Device 002, uvcvideo 已绑定 (4-4:1.0, 4-4:1.4) |
| AC1 连接 | Bus 002 Device 002, 无内核驱动绑定 |
| 采集时长 | ~10 秒 / 设备 |
| D2C 模式 | 335L=SW (HW 不支持当前配置), AC1=HW |
| 融合参数 | alpha=0.5, depth range=[0.3m, 5.0m] |

### 6.2 FPS 实测

| 流 | 335L 实测 FPS | AC1 实测 FPS | 设计 FPS |
|----|-------------|-------------|---------|
| **COLOR** | 30.0 | 10.0 | 335L=30, AC1=30* |
| **DEPTH** | 30.0 | 10.0 | 335L=30, AC1=10 |
| **IR_LEFT** | 30.0 | — | 335L=30 |
| **IR_RIGHT** | 30.0 | — | 335L=30 |
| **ACCEL** | ~200 | ~101 | 335L=200, AC1=100 |
| **GYRO** | ~200 | ~101 | 335L=200, AC1=100 |
| **POINT** | — | 10.0 | AC1=10 |
| **D2C Fused** | 6.0 | 4.5 | min(color,depth) |

> *AC1 色彩传感器标称 30fps，但 LiDAR 10fps 限定了帧同步后的实际输出帧率。

### 6.3 335L 实测传感器配置

```
Color:  1280×720@30 MJPG  (fx=610.214, fy=610.327, cx=641.278, cy=362.241)
Depth:  1280×800@30 Y16   (fx=622.801, fy=622.801, cx=643.5, cy=399)
IR:     双 IR (Y8)
IMU:    ~200Hz
D2C:    SW 模式 (当前配置下 HW D2C 不可用)
```

### 6.4 AC1 实测传感器配置

```
Color:  1920×1080@10 NV12  (intrinsic=0, 无针孔模型)
Depth:  96×288@10 Y16      (合成, depthScale=5.0, intrinsic=0)
IMU:    ~100Hz (温度=0.0, 标称值未经校准)
D2C:    HW 模式 (出厂标定, 无需/无法软件对齐)
Point:  27,648 pts/frame, 96 PCD 文件
```

### 6.5 10 秒采集文件大小对比

| 文件 | 335L 大小 | AC1 大小 | 比率 |
|------|----------|----------|------|
| `color.h264` | 8.9 MB | 1.6 MB | 5.6× |
| `depth.h264` | 5.5 MB | 870 KB | 6.3× |
| `depth_raw.raw` | **651 MB** | 5.1 MB | 128× |
| `ir_left.h264` | 5.8 MB | — | — |
| `ir_right.h264` | 5.7 MB | — | — |
| `imu.txt` | 334 KB | 170 KB | 2.0× |
| `d2c_fused.h264` | 2.3 MB | 2.4 MB | ~1× |
| `pcd/` 目录 | — | ~66 MB (96 files × 703KB) | — |
| **总计 (约)** | **~680 MB** | **~76 MB** | 8.9× |

**关键观察**：
- 335L 的 `depth_raw.raw` 占总量 >95%，因为 1280×800×2B×300帧 ≈ 615 MB
- AC1 总量仅为 335L 的 1/9，但多了 PCD 点云数据 (~66 MB)
- 335L 深度原始数据大 128 倍的原因：1,024,000 像素 vs 27,648 点

---

## 7. 输出数据格式对比

### 7.1 Depth Raw 容器格式

两种设备共用 `NIO_DEPTH_RAW` 二进制容器格式：

```
[File Header - 68 bytes, frame 0 only]
  [16B] magic = "NIO_DEPTH_RAW\0\0\0\0" (null-padded to 16B)
  [4B]  version = 1
  [4B]  width
  [4B]  height
  [4B]  depthScale (float, IEEE 754)
  [36B] reserved (zeros)

[Frame Header - 32 bytes, every frame]
  [8B] timestampUs (uint64)
  [8B] frameIndex (uint64) — legacy, now = deviceTsUs
  [4B]  dataSize (uint32) = width × height × 2
  [12B] reserved (zeros)

[Frame Data - width × height × 2 bytes]
  uint16 depth values
```

| 帧数据差异 | 335L | AC1 |
|-----------|------|-----|
| 每帧大小 | 1280×800×2 = 2,048,000 B | 96×288×2 = 55,296 B |
| 深度含义 | `depth_m = raw × 0.001` | `depth_m = raw × 0.005` |
| 像素布局 | 2D 栅格 (x-y 空间连续) | 1D 投影 (point[i] → col=i%96, row=i/96) |
| 空洞/无效值 | 0 (SDK 填充) | 0 (NaN/<0.2m/>200m 过滤) |

### 7.2 Point Cloud (PCD) — AC1 独有

PCD 文件使用 PCD v0.7 格式，binary 存储：

```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity ring timestamp
SIZE 4 4 4 4 2 8
TYPE F F F F U F
COUNT 1 1 1 1 1 1
WIDTH 27648
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 27648
DATA binary
```

每点 26 bytes: float x(4) + float y(4) + float z(4) + float intensity(4) + uint16 ring(2) + double timestamp(8)

### 7.3 Intrinsic JSON 差异

**335L**:
```json
{
  "depth": {"fx":622.801,"fy":622.801,"cx":643.5,"cy":399,"width":1280,"height":800},
  "color": {"fx":610.214,"fy":610.327,"cx":641.278,"cy":362.241,"width":1280,"height":720},
  "depth_scale":0.001,
  "device":"Orbbec_Gemini_335L_CP26363000FB"
}
```

**AC1**:
```json
{
  "depth": {"fx":0,"fy":0,"cx":0,"cy":0,"width":0,"height":0},
  "color": {"fx":0,"fy":0,"cx":0,"cy":0,"width":0,"height":0},
  "depth_scale":5,
  "lidar": {
    "type": "RS-AC1",
    "point_grid_width": 96,
    "point_grid_height": 288,
    "distance_min_m": 0.2,
    "distance_max_m": 200.0,
    "distance_resolution_m": 0.005,
    "vector_base": 32768,
    "point_fields": "x y z intensity ring timestamp",
    "point_type": "PointXYZIRT"
  },
  "device":"RoboSense_AC1_1111bfa90090"
}
```

**说明**：AC1 的 intrinsic 全部为 0 是因为 LiDAR 不使用针孔相机模型，而是方向编码 + ToF 测距。LiDAR 空间模型由 `vector_base` 和 SPAD 方向表描述，不适合用 fx/fy/cx/cy 表示。

---

## 8. D2C（深度对齐彩色）策略

### 8.1 Orbbec 335L：HW/SW 双模

- **HW D2C**：固件直接在芯片上做深度→彩色像素映射，零 CPU 开销
  - 前提：深度配置必须在 `getD2CDepthProfileList()` 返回的列表中
  - 当前实测：**HW D2C 不可用**（335L 在 1280×800 深度模式下无 HW 匹配能力）
- **SW D2C**：`ob::Align` filter，基于标定内参+外参做软件映射
  - 支持多种畸变模型：Brown-Conrady (K3/K6)、Kannala-Brandt4
  - SSE 加速路径 (`AlignImpl.cpp:1609-1651`)
  - 通用不加速路径 (`AlignImplGeneric.cpp` — TODO 存根)

**代码路径** (`app/capture/nio_capture_session.cpp:164-208`)：
```
if (!hwD2CMode_) → alignFilter = pipeline_->getD2CAlignFilter()
FusionStreamTask → SW align: enqueueNioFrameSet → alignFilter->process() → blend
FusionStreamTask → HW align: enqueueColor + enqueueDepth → directly blend pixel-by-pixel
```

### 8.2 RoboSense AC1：HW Only

- LiDAR 和 RGB 相机在出厂时已完成硬件级空间和时间标定
- 每帧点云和 RGB 图像天然已对齐（共享时间戳，外参已固化）
- 无需/无法软件对齐：`RsPipeline::setAlignMode()` 为空操作
- `RsPipeline::checkHWD2CSupport()` 始终返回 `true`

**实测效果**：AC1 D2C 可见明显块状伪影——96×288 深度上采样至 1920×1080 导致像素块可见。335L SW D2C 因原始深度图分辨率高（1280×800）， blending 边缘更平滑。

---

## 9. IMU 数据对比

### 9.1 规格对比

| 参数 | 335L | AC1 |
|------|------|-----|
| **加速度计** | ✓ | ✓ |
| **陀螺仪** | ✓ | ✓ |
| **采样率** | ~200 Hz | ~100 Hz |
| **加速度量程** | 2g / 8g (OB 配置) | 固定 |
| **陀螺仪量程** | 250 dps (OB 配置) | 固定 |
| **温度读数** | 有 (29.86°C 实测) | 无 (0.000000) |
| **数据来源** | USB HID (OrbbecSDK) | rs_driver HID |
| **输出格式** | CSV: `host_ts_ms,type,device_ts_us,x,y,z,temperature` | 同 |

### 9.2 实测 IMU 样本分析

**335L** (10 秒 ≈ 4616 行):
```
1782481586091,ACCEL,1782481586078430,3.395068,2.153320,9.016431,29.861111
1782481586091,GYRO,1782481586078430,0.002128,-0.001596,0.007982,29.861111
```
- Accel z ≈ 9.0 m/s² → 符合静止重力方向
- Gyro ≈ 0.002 rad/s → 静态噪声水平
- 温度读数有效 (29.86°C)

**AC1** (10 秒 ≈ 2328 行):
```
1782485876029,ACCEL,1782485876242695,-0.354342,0.009577,9.773131,0.000000
1782485876029,GYRO,1782485876242695,-0.010653,-0.008522,-0.009587,0.000000
```
- Accel z ≈ 9.77 m/s² → 符合静止重力方向
- Gyro ≈ 0.01 rad/s → 偏置噪声比 335L 大约 5 倍
- 温度读数为 0（rs_driver 未提供温度数据）
- 采样率 ~100Hz (335L 的一半)

**注意**：此前文档记录 AC1 IMU "HID exists but no data"。本次实测确认 AC1 IMU **已正常工作**，输出 ~100Hz Accel+Gyro 数据。这可能是驱动/固件更新后修复。

---

## 10. 集成复杂度评估

### 10.1 Orbbec 335L/336L 集成

| 方面 | 评估 | 说明 |
|------|------|------|
| SDK 集成难度 | ★☆☆ 简单 | 预编译 `.so` + 头文件，CMake `add_subdirectory` |
| 设备发现 | ★☆☆ 简单 | `ob::Context::queryDeviceList()` 一步完成 |
| 传感器枚举 | ★☆☆ 简单 | `device->getSensorList()` 动态查询 |
| 流配置 | ★★☆ 中等 | 需理解 `StreamProfileList` 和 `selectBestProfile` 评分逻辑 |
| D2C 对齐 | ★★☆ 中等 | HW/SW 双模选择，需检查 profile 兼容性 |
| USB 权限 | ★☆☆ 简单 | OrbbecSDK 自带 udev 规则 |
| 内核驱动冲突 | ★☆☆ 简单 | uvcvideo 可共存，不影响 SDK 功能 |
| 文档/社区 | ★★☆ 中等 | 开源 SDK + 活跃 GitHub，但中文文档较少 |

### 10.2 RoboSense AC1 集成

| 方面 | 评估 | 说明 |
|------|------|------|
| SDK 集成难度 | ★★★ 复杂 | rs_driver 源码集成，需 `-DENABLE_USB -DENABLE_IMU_PARSE -DENABLE_IMAGE_PARSE` |
| 设备发现 | ★★☆ 中等 | 需自行实现 libusb 枚举 (`RsContext::scanDevices()`) |
| 传感器枚举 | ★☆☆ 简单 | 硬编码参数，无枚举需求 |
| 流配置 | ★☆☆ 简单 | 固定配置，只需传参给 `RSDriverParam` |
| D2C 对齐 | ★☆☆ 简单 | HW only，无对齐逻辑 |
| USB 权限 | ★★★ 复杂 | 需手动 udev 规则 + uvcvideo unbind（如果内核抢占） |
| 内核驱动冲突 | ★★★ 复杂 | AC1 出厂 UVC 兼容接口，必须 unbind `uvcvideo` 才能用 rs_driver 的自定义 libuvc |
| 串行号获取 | ★★☆ 中等 | USB string descriptor 可能不可用，需 fallback 到 bus-device 编号 |
| 文档/社区 | ★★☆ 中等 | rs_driver GitHub 活跃，但 AC1 为定制版，文档有限 |

### 10.3 综合集成成本

| 维度 | Orbbec | RoboSense |
|------|--------|-----------|
| **初始集成工时** | 0.5–1 天 | 2–3 天 |
| **驱动维护成本** | 低 (预编译 SDK) | 中 (源码依赖，需跟进 rs_driver 更新) |
| **新增设备支持** | 中 (需实现 NioDevice/NioPipeline 子类 + 流配置) | 低 (硬编码，复制模式即可) |
| **多设备共存** | 简单 (SDK 自动) | 需手动管理 USB serial / bus-device 地址 |
| **跨平台移植** | SDK 已支持 | 需 rs_driver + libusb 适配 |

---

## 11. 适用场景评估

### 11.1 Orbbec 335L/336L 最适合

| 场景 | 适配度 | 原因 |
|------|--------|------|
| 室内机器人导航 | ★★★ | 高分辨率深度图 + 近距离高精度 |
| 人脸识别/3D 重建 | ★★★ | 毫米级深度精度 + 高像素密度 |
| 手势交互/AR | ★★★ | 低延迟 + 高帧率 (30fps) |
| 避障 (5m 内) | ★★★ | 精度高，FOV 适中 |
| SLAM (视觉/视觉惯性) | ★★★ | RGB+Depth+IMU 三源 + 可选 SW D2C |
| 室外强光环境 | ★☆ | 结构光受阳光干扰严重 |
| 远距感知 (>10m) | ★ | 超出有效距离 |

### 11.2 RoboSense AC1 最适合

| 场景 | 适配度 | 原因 |
|------|--------|------|
| 路侧感知/智慧交通 | ★★★ | 远距 + 抗光 + 车辆检测级精度 |
| 室外人员/车辆计数 | ★★★ | LiDAR 不受光照影响，200m 范围 |
| 停车场/园区监控 | ★★★ | 融合 RGB + 远距点云 + IP65 |
| 大空间 3D 测绘 | ★★ | 低空间分辨率 (96×288) |
| SLAM (特征稀疏) | ★★ | 点云稀疏，特征匹配受限 |
| 近距离精细操作 | ★ | 低分辨率 + 5mm 精度，不适合 |
| 手势/面部识别 | ★ | 深度像素太粗，无法识别人脸/手势细节 |

---

## 12. 总结与建议

### 12.1 核心结论

| 维度 | Gemini 335L/336L | RoboX AC1 | 结论 |
|------|-----------------|-----------|------|
| **深度精度** | 1mm (千分之一) | 5mm (五百分之一) | OB 精度 5× 优势 |
| **深度分辨率** | 1280×800=1M 像素 | 96×288=27K 点 | OB 分辨率 37× 优势 |
| **最大量程** | 5m / 10m | **200m** | AC1 量程 20–40× 优势 |
| **数据输出** | 7 种流 (color/depth/ir×2/imu/fused/intrinsic) | 6 种流 (color/depth/point/imu/fused/intrinsic) | OB 多 IR 流；AC1 多点云 |
| **存储占用** (10s) | ~680 MB | ~76 MB | AC1 存储 9× 优势 |
| **FPS** | 30 fps 全流 | 10 fps 深度/点云 | OB 帧率 3× 优势 |
| **D2C 质量** | 高 (原始高分辨率) | 低 (块状伪影) | OB D2C 画质显著优 |
| **抗环境光** | 弱 (结构光受干扰) | 强 (脉冲 LiDAR) | AC1 室外场景唯一选择 |
| **IMU 质量** | 高 (温度有效, 噪声低) | 中 (温度缺失, 噪声高) | OB IMU 更可靠 |
| **集成难度** | 低 | 高 | OB SDK 更成熟 |

### 12.2 选型建议

- **室内近距离 (<5m) + 高精度需求** → **Gemini 335L**：高分辨率深度图 + 高精度 + 多传感器流 + 成熟 SDK + 低集成成本
- **室内中距离 (5–10m)** → **Gemini 336L**：335L 长距版，相同架构更远距离
- **室外 / 远距 (10–200m) / 抗光需求** → **RoboX AC1**：LiDAR 量级距离 + 抗阳光 + RGB 融合 + 点云输出
- **混合场景 (室内+室外)** → 335L + AC1 双设备并行，本项目已支持

### 12.3 已知限制和改进方向

| 限制 | 影响设备 | 改进建议 |
|------|---------|---------|
| HW D2C 不可用 (当前 335L 配置) | 335L | 降低深度分辨率到 640×480 可能启用 HW D2C；或优化 SW D2C SSE 路径 |
| AC1 D2C 块状伪影 | AC1 | 从 96×288 upsample 到 1080p 固有限制；可考虑双线性插值 + 空洞填充 |
| AC1 IMU 温度读数为 0 | AC1 | rs_driver 未暴露温度字段；需驱动层修复 |
| AC1 串行号可能不可用 | AC1 | USB string descriptor 可能为空；需 fallback 到 bus-device 编号 |
| 335L depth_raw 存储过大 | 335L | 1280×800×2B/frame=2MB；可考虑帧间压缩或降采样存盘 |
| AC1 IMU 噪声偏高 | AC1 | 可能需要校准偏置；335L ~0.002 rad/s vs AC1 ~0.01 rad/s |
| `device_comparison.md` IMU 信息需更新 | 文档 | AC1 IMU 现已正常输出 ~100Hz，需更新文档 |

---

## 13. 参考来源

### 13.1 官方产品页面

| 产品 | URL |
|------|-----|
| Orbbec Gemini 335L | https://www.orbbec.com/products/gemini-335l |
| Orbbec Gemini 336L | https://www.orbbec.com/products/gemini-336l |
| RoboSense AC1 | https://www.robosense.ai/ac1 |

### 13.2 SDK/驱动仓库

| 项目 | URL | 版本 |
|------|-----|------|
| OrbbecSDK | https://github.com/orbbec/OrbbecSDK | v1.10.27 (stable) |
| rs_driver | https://github.com/RoboSense-LiDAR/rs_driver | AC1 定制版 (dev_opt_AC1) |

### 13.3 本项目代码引用

| 组件 | 文件路径 | 关键行 |
|------|---------|-------|
| OB 设备适配器 | `app/driver/orbbec/nio_ob_device.cpp` | 87-290 (setupPipeline) |
| OB 格式转换 | `app/driver/orbbec/nio_ob_adapter.hpp` | 20-258 (全) |
| RS 设备适配器 | `app/driver/robosense/nio_rs_device.cpp` | 20-369 (全) |
| RS 帧适配器 | `app/driver/robosense/nio_rs_frame_adapter.hpp` | 22-121 (全) |
| RS 格式转换 | `app/driver/robosense/nio_rs_adapter.hpp` | 19-77 (全) |
| 采集会话 | `app/capture/nio_capture_session.cpp` | 25-504 (全) |
| 中性类型定义 | `app/core/nio_types.hpp` | 20-242 (全) |
| 帧队列 | `app/capture/nio_frame_queue.hpp` | 1-55 (全) |
| D2C 对齐核心 | `vendors/OrbbecSDK/src/filter/publicfilters/AlignImpl.cpp` | 1533-1685 (D2C 算法) |
| 点云 Filter | `vendors/OrbbecSDK/src/filter/publicfilters/PointCloudProcess.hpp` | 13-80 (全) |
| 流配置列表 | `vendors/OrbbecSDK/include/libobsensor/hpp/StreamProfile.hpp` | 473-601 (StreamProfileList) |
| 设备发现工厂 | `app/driver/nio_driver_factory.hpp` + `.cpp` | 全 |
| 设备对比文档 | `docs/device_comparison.md` | 全 (296 行) |

### 13.4 测试数据

| 数据 | 路径 |
|------|------|
| 335L 10s 采集 | `/tmp/nio_analysis_335L/1782481585477/` |
| AC1 10s 采集 | `/tmp/nio_analysis_ac1/1782485875989/` |
| 335L FPS 测试 | `/tmp/nio_fps_test_335l/` |

---

*文档由代码分析 + 实时采集测试生成，所有数据均有代码或测试验证支撑。*
