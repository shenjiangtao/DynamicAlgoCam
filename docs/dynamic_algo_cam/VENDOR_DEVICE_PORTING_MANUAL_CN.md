# 硬件设备移植指南

> 基于源码 `app/core/`、`app/driver/`、`app/capture/` 实现分析。所有接口描述以实际代码签名为依据。

**适用受众**：需要将新厂商深度摄像头接入 dynamic_algo_cam 采集框架的集成开发者。

---

## 1. 架构概览

```
app/
├── core/           # SDK 无关层：NioDevice, NioPipeline, NioFrame, NioTypes
├── driver/         # SDK 适配层（唯一允许引用厂商 SDK 头文件的层）
│   ├── orbbec/     # Orbbec SDK 适配器
│   ├── robosense/  # RoboSense rs_driver 适配器
│   └── nio_driver_factory.hpp/cpp  # 设备发现工厂
├── capture/         # CaptureSession — 仅使用 NioDevice/NioPipeline 接口
├── plugins/opencv/  # OpenCV 工具 — 仅使用 NioFrame/NioFormat
└── dynamic_algo_cam/ # 主程序 — 仅调用 discoverDevices()
```

**硬性规则**：`ENABLE_*` 宏、厂商 SDK 头文件（`libobsensor/`、`rs_driver/` 等）、厂商特定常量（如 `OB_DEVICE_VID = 0x2bc5`）仅允许出现在 `app/driver/`。其余 `app/` 代码必须 SDK-agnostic。`app/core/` 零厂商 SDK 依赖编译。

---

## 2. 抽象接口 (app/core/nio_device.hpp)

每个厂商适配器必须实现以下三个类：

### 2.1 NioDevice

| 方法 | 返回值 | 用途 |
|------|--------|------|
| `getDeviceInfo()` | `NioDeviceInfo` | name, serialNumber, vid, pid, connectionType |
| `timerSyncWithHost()` | void | 设备时钟同步到主机 |
| `isGlobalTimestampSupported()` | bool | 查询全局时间戳支持能力 |
| `enableGlobalTimestamp(bool)` | void | 启用/禁用全局时间戳 |
| `getSensorInfo()` | `NioSensorInfo` | 缓存的传感器存在性 + 流配置摘要 |
| `getIntProperty(int)` | int32_t | 读取整数属性（如深度精度） |
| `hasIRSensor()` | bool | 设备是否具有任何 IR 传感器 |
| **`setupPipeline(NioPipeline&)`** | `NioSensorInfo` | **核心方法**：枚举传感器、选择配置、启用流、应用设备 quirks、检测 D2C 模式、返回解析后的传感器信息 |

**框架调用流程**（`app/capture/nio_capture_session.cpp:25-53`）：

```cpp
// CaptureSession::setup() 执行顺序：
device_->timerSyncWithHost();                      // 时钟同步（异常仅警告）
device_->enableGlobalTimestamp(true);               // 支持时启用
sensorInfo_ = device_->setupPipeline(*pipeline_);   // 核心：填充所有传感器信息
// sensorInfo_ 驱动后续所有编码器/文件创建决策
```

### 2.2 NioPipeline

| 方法 | 返回值 | 用途 |
|------|--------|------|
| `enableStream(NioStreamConfig)` | void | 配置流（类型、宽高、帧率、格式） |
| `disableStream(NioFrameType)` | void | 禁用某流类型 |
| `setAggregateAllTypeFrameRequire(bool)` | void | 帧聚合模式 |
| `setAlignMode(NioAlignMode)` | void | 设置 D2C 对齐模式（HW/SW/NONE） |
| `checkHWD2CSupport(...)` | bool | 检查给定配置下 HW D2C 可行性 |
| `enableFrameSync()` | void | 启用硬件帧同步 |
| **`start(NioVideoCallback)`** | bool | **核心方法**：启动管道；回调接收 `shared_ptr<NioFrameSet>` |
| `startImu(NioImuCallback)` | bool | 启动 IMU 流；回调接收 `vector<NioImuSample>` |
| `stop()` | void | 停止视频管道 |
| `stopImu()` | void | 停止 IMU 管道 |
| `getDevice()` | `shared_ptr<NioDevice>` | 返回底层设备 |
| `isPointCloudDepth()` | bool | 深度为 3D 点云时覆写为 `true`（默认 false） |
| `getAlignMode()` | `NioAlignMode` | 当前对齐模式 |
| `getD2CAlignFilter()` | `shared_ptr<NioD2CAlign>` | 返回 SW 对齐滤镜（默认 nullptr） |

### 2.3 NioContext

| 方法 | 返回值 | 用途 |
|------|--------|------|
| `getDeviceCount()` | uint32_t | 已连接设备数 |
| `getDevice(uint32_t)` | `shared_ptr<NioDevice>` | 按索引获取设备 |

### 2.4 NioD2CAlign（SW D2C 场景）

| 方法 | 返回值 | 用途 |
|------|--------|------|
| `process(shared_ptr<void>, NioAlignedFrame&)` | bool | 接收类型擦除的原生 FrameSet，执行对齐，填充输出结构 |

---

## 3. 值类型 (app/core/nio_types.hpp, nio_frame.hpp)

### 3.1 NioFormat

```
UNKNOWN, Y8, Y16, YUYV, UYVY, YUY2, MJPG, MJPEG, NV12, NV21, I420,
RGB, BGR, RGBA, BGRA, H264, H265, HEVC, POINT, RGB888
```

辅助函数：
- `nioFormatBpp(NioFormat)` → 每像素字节数（Y8=1, Y16=2, RGB=3, RGBA=4, 其余=0）
- `nioFormatRawSize(NioFormat, w, h)` → 原始缓冲区大小（字节）

### 3.2 NioFrameType

```
COLOR, DEPTH, IR, IR_LEFT, IR_RIGHT, ACCEL, GYRO,
COLOR_LEFT, COLOR_RIGHT, CONFIDENCE, POINT, COUNT
```

### 3.3 NioFrame

| 字段 | 类型 | 说明 |
|------|------|------|
| `type` | NioFrameType | 传感器来源 |
| `format` | NioFormat | 像素格式 |
| `width`, `height` | int | 帧尺寸（IMU/点云为 0） |
| `timestampUs` | uint64_t | 微秒时间戳 |
| `depthScale` | float | 米/原始单位（默认 1.0） |
| `data` | `vector<uint8_t>` | 拥有像素数据（深拷贝） |
| `rawData()` | `const uint8_t*` | 指向 data 的裸指针 |
| `dataSize()` | uint32_t | 字节数 |

### 3.4 NioFrameSet

| 方法 | 说明 |
|------|------|
| `getFrame(NioFrameType)` | 获取帧指针（不存在返回 nullptr） |
| `setFrame(NioFrameType, NioFrame)` | 插入帧（转移所有权） |
| `allFrames()` | 遍历所有帧 |
| `nativeFrameSet` | `shared_ptr<void>` — SW D2C 对齐用的不透明原生 FrameSet |

### 3.5 NioSensorInfo

| 字段 | 说明 |
|------|------|
| `hasColor`, `hasDepth`, `hasIR`, `hasIRLeft`, `hasIRRight`, `hasAccel`, `hasGyro` | 传感器存在性 |
| `colorFormat`, `depthFormat`, `irFormat`, `irLeftFormat`, `irRightFormat` | 解析后的 NioFormat |
| `colorW/H/Fps`, `depthW/H/Fps`, `irW/H/Fps`, `irLW/H/Fps`, `irRW/H/Fps` | 解析后的流配置 |
| `depthIntrinsic`, `colorIntrinsic` | `NioIntrinsic`（fx, fy, cx, cy, width, height） |
| `depthScale` | 米/原始深度单位 |

### 3.6 NioAlignedFrame

| 字段 | 说明 |
|------|------|
| `colorData`, `colorSize`, `colorTs` | 对齐后的彩色原始数据 |
| `depthData`, `depthSize`, `depthTs` | 对齐后的深度原始数据 |
| `depthScale` | 该帧的深度 scale |

---

## 4. 已有适配器参考

### 4.1 Orbbec 适配器 (app/driver/orbbec/)

| 文件 | 内容 | 约行数 |
|------|------|--------|
| `nio_ob_adapter.hpp` | `obFormatToNio()`, `nioFormatToOb()`, `obFrameTypeToNio()`, `nioFrameTypeToOb()`, `nioFrameTypeToObSensor()`, `obIntrinsicToNio()`, `nioIntrinsicToOb()`, `selectBestProfile()`, `isLiDARDevice()`, `OB_DEVICE_VID` (0x2bc5), `isGemini305Device()`, `isGemini305gDevice()`, `isAstraMiniDevice()` | ~210 |
| `nio_ob_device.hpp` | `ObDevice`, `ObPipeline`, `ObContext` 声明 | ~106 |
| `nio_ob_device.cpp` | 完整实现：设备枚举、传感器配置、HW/SW D2C、管道启停、IMU 管道 | ~447 |
| `nio_ob_frame_adapter.hpp` | `obFrameSetToNio()`（深拷贝所有视频帧）、`obImuToNioSamples()` | ~118 |
| `nio_ob_d2c_align.hpp` | `ObD2CAlign : NioD2CAlign` — 封装 `ob::Align` | ~54 |

**关键模式**：

- `ObDevice::setupPipeline()` 将 `NioPipeline&` 下转为 `ObPipeline&`，遍历 SDK 传感器列表，对每个传感器调用 `selectBestProfile()`，在 `ob::Config` 上启用流，应用设备 quirks（如 Gemini 305g 禁用 IR_LEFT），检查 HW D2C，返回完整填充的 `NioSensorInfo`。
- `ObPipeline::start()` 封装 SDK 回调：调用 `obFrameSetToNio(obFs)` 深拷贝，附加 `nativeFrameSet = static_pointer_cast<void>(obFs)`，然后调用用户回调。
- `ObPipeline::getD2CAlignFilter()` — SW 模式返回 `make_shared<ObD2CAlign>(alignFilter_)`，否则 nullptr。
- `ObD2CAlign::process()` 从 `nativeFrameSet` 还原 `ob::FrameSet`，调用 `ob::Align::process()`，提取 color/depth 指针到 `NioAlignedFrame`。

### 4.2 RoboSense 适配器 (app/driver/robosense/)

| 文件 | 内容 | 约行数 |
|------|------|--------|
| `nio_rs_adapter.hpp` | `rsFrameFormatToNio()`, `nioFormatToRsFrameFormat()`, `rsImuToNioSamples()` | ~75 |
| `nio_rs_device.hpp` | `RsDevice`, `RsPipeline`, `RsContext` 声明 | ~163 |
| `nio_rs_device.cpp` | 完整实现：USB 发现、双 get/put 回调队列、FrameSet 合成、点云转深度图 | ~373 |
| `nio_rs_frame_adapter.hpp` | `rsDepthToNioFrame()`, `rsPointToNioFrame()`, `rsImageToNioFrame()` | ~121 |

**关键模式**：

- `RsDevice` 传感器信息固定（无 IR，depth=96×288@10 Y16，color=1920×1080@30 NV12）。
- `RsPipeline` 使用 `LidarDriver` + 双回调队列（`freeCloudQueue` ↔ `stuffedCloudQueue` 等）。三个处理线程排空 stuffed 队列，color 和 depth 同时到达时合成 `NioFrameSet`。
- `rsDepthToNioFrame()` 从 3D 点云合成 96×288 Y16 深度图（distance / 0.005 → uint16，无效/NaN 为 0）。
- `RsPipeline::isPointCloudDepth()` 返回 `true`；`getAlignMode()` 始终返回 `HW`。
- `RsContext::scanDevices()` 使用 libusb 查找 VID=0x3840/PID=0x1010 设备，从 USB 描述符读取序列号，回退到 busnum-devnum UUID。
- `RsPipeline::start()` **必须在** `driver_.start()` **之前**启动处理线程，确保首个数据到达时线程就绪。

---

## 5. 逐步移植清单

### 阶段 1：创建适配器文件

为新厂商 "XYZ" 在 `app/driver/xyz/` 下创建：

```
app/driver/xyz/
├── nio_xyz_adapter.hpp       # 类型映射：XYZ↔Nio
├── nio_xyz_frame_adapter.hpp # 帧转换：XYZ frame → NioFrame
├── nio_xyz_device.hpp        # XyzDevice, XyzPipeline, XyzContext 声明
└── nio_xyz_device.cpp        # 完整实现
```

如设备支持 SW D2C 对齐，还需：
```
├── nio_xyz_d2c_align.hpp     # XyzD2CAlign : NioD2CAlign
```

### 阶段 2：实现类型映射 (nio_xyz_adapter.hpp)

必须实现的转换：

| 从 | 到 | 说明 |
|----|----|------|
| 厂商格式枚举 | `NioFormat` | 映射所有厂商像素格式 |
| `NioFormat` | 厂商格式枚举 | 反向映射（可能有损） |
| 厂商帧类型 | `NioFrameType` | 映射传感器类型 |
| `NioFrameType` | 厂商传感器类型 | 用于流启停 |
| 厂商内参 | `NioIntrinsic` | 复制 fx/fy/cx/cy/width/height |
| `NioIntrinsic` | 厂商内参 | 反向映射 |

可能需要的辅助函数：
- `selectBestProfile()` — 若厂商 SDK 支持配置枚举，参考 `nio_ob_adapter.hpp`；若固定配置则硬编码。
- `isLiDARDevice()` — 若厂商支持 LiDAR 传感器，添加到适配器；否则可省略。

### 阶段 3：实现帧转换 (nio_xyz_frame_adapter.hpp)

至少提供：

```cpp
// 厂商视频帧 → NioFrameSet（深拷贝像素数据）
NioFrameSet xyzFrameSetToNio(VendorFrameSetType vendorFs);

// 厂商 IMU 数据 → vector<NioImuSample>
std::vector<NioImuSample> xyzImuToNioSamples(VendorImuType vendorImu);
```

`xyzFrameSetToNio` 的关键规则：
1. **深拷贝像素数据**：`nf.data.assign(data, data + size)` — 绝不可持有 SDK 缓冲区指针。
2. **填充所有 NioFrame 字段**：type, format, width, height, timestampUs, depthScale（深度帧），data。
3. **附加原生 FrameSet**（仅 SW D2C 需要时）：`nioFs.nativeFrameSet = static_pointer_cast<void>(vendorFs)`。否则留空。
4. **优雅处理缺失帧**：某个流不在厂商 FrameSet 中时跳过 — `NioFrameSet` 默认为空。

### 阶段 4：实现设备类 (nio_xyz_device.hpp/cpp)

#### XyzDevice : NioDevice

```cpp
class XyzDevice : public NioDevice {
public:
    explicit XyzDevice(VendorDeviceHandle dev);

    NioDeviceInfo getDeviceInfo() const override;
    void timerSyncWithHost() override;
    bool isGlobalTimestampSupported() const override;
    void enableGlobalTimestamp(bool enable) override;
    NioSensorInfo getSensorInfo() const override;
    int32_t getIntProperty(int propertyId) override;
    bool hasIRSensor() const override;
    NioSensorInfo setupPipeline(NioPipeline& pipeline) override;

    // 暴露厂商句柄供 Pipeline 构造
    VendorDeviceHandle xyzDevice() const { return xyzDevice_; }
private:
    VendorDeviceHandle xyzDevice_;
};
```

**`setupPipeline()` 是最复杂的方法。** 它必须：

1. 将 `NioPipeline&` 下转为 `XyzPipeline&`。
2. 枚举厂商传感器列表。
3. 对每个传感器类型选择最佳流配置（分辨率、格式、帧率）。
4. 在厂商管道配置上启用选定配置。
5. 用解析后的值填充 `NioSensorInfo`。
6. 应用设备特殊处理（禁用故障流等）。
7. 检查 D2C 对齐能力 → 调用 `pipeline.setAlignMode()`。

   **D2C-HW 感知的深度 profile 选择（Orbbec 参考）：** 选深度 profile 前，先调用等价于
   `getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE)` 的厂商接口拿到 HW-D2C 支持集合，
   作为 `selectBestProfile()` 的 `hwD2CSupportedProfiles` 参数传入——出现在该集合中的 profile
   加 +800 分，最大化命中硬件对齐、减少软件对齐降级。详见技术参考 §7.1.1。

8. 返回完整填充的 `NioSensorInfo`。

#### XyzPipeline : NioPipeline

```cpp
class XyzPipeline : public NioPipeline {
public:
    explicit XyzPipeline(std::shared_ptr<XyzDevice> device);

    void enableStream(const NioStreamConfig& cfg) override;
    void disableStream(NioFrameType type) override;
    // ... 所有 NioPipeline 虚方法 ...

    bool start(NioVideoCallback callback) override;
    void stop() override;
    std::shared_ptr<NioD2CAlign> getD2CAlignFilter() const override;
private:
    std::shared_ptr<XyzDevice> xyzDevice_;
    VendorPipelineHandle xyzPipeline_;
    NioVideoCallback videoCallback_;
    // ...
};
```

**`start()` 关键模式：**

```cpp
bool XyzPipeline::start(NioVideoCallback callback) {
    videoCallback_ = callback;
    try {
        vendorPipeline_->start(vendorConfig_, [this](VendorFrameSetType vfs) {
            if (vfs) {
                auto nioFs = std::make_shared<NioFrameSet>(xyzFrameSetToNio(vfs));
                // 仅 SW D2C 对齐需要时附加 nativeFrameSet：
                nioFs->nativeFrameSet = std::static_pointer_cast<void>(vfs);
                videoCallback_(nioFs);
            }
        });
        return true;
    } catch (VendorError& e) {
        NIO_LOG_ERROR_S("Pipeline start failed: " << e.what());
        return false;
    }
}
```

**异步源**（如 RoboSense 点云和图像在不同线程到达）：
- 使用处理线程排空厂商队列。
- 用 mutex 维护 `colorFrame_`、`depthFrame_`、`colorReady_`、`depthReady_`。
- 当两者就绪时，调用 `tryEmitFrameSet()` 组装并发射 `NioFrameSet`。
- **注意**：处理线程必须在 `driver_.start()` 之前启动（见 RsPipeline 模式）。

#### XyzContext : NioContext

- 实现设备发现（USB 扫描、SDK 上下文查询等）。
- `getDeviceCount()` → 已连接设备数。
- `getDevice(index)` → `make_shared<XyzDevice>(vendorDev)`。

### 阶段 5：注册到驱动工厂 (nio_driver_factory.cpp)

```cpp
#ifdef ENABLE_XYZ
#include "xyz/nio_xyz_device.hpp"
#endif

std::vector<DiscoveredDevice> discoverDevices() {
    std::vector<DiscoveredDevice> result;

    // ... 已有 ENABLE_ORBBEC 块 ...
    // ... 已有 ENABLE_RS_AC1 块 ...

#ifdef ENABLE_XYZ
    XyzContext xyzCtx;
    uint32_t xyzCount = xyzCtx.getDeviceCount();
    for (uint32_t i = 0; i < xyzCount; i++) {
        auto nioDev = xyzCtx.getDevice(i);
        auto xyzDev = std::dynamic_pointer_cast<XyzDevice>(nioDev);
        if (!xyzDev)
            continue;
        DiscoveredDevice dd;
        dd.device = nioDev;
        dd.pipeline = std::make_shared<XyzPipeline>(xyzDev);
        result.push_back(std::move(dd));
    }
#endif

    return result;
}
```

`DiscoveredDevice` 结构定义在 `app/driver/nio_driver_factory.hpp:20-24`：
```cpp
struct DiscoveredDevice {
    std::shared_ptr<NioDevice> device;
    std::shared_ptr<NioPipeline> pipeline;
};
```

### 阶段 6：更新 CMake

#### 根 CMakeLists.txt

`option()` 声明在**根** `CMakeLists.txt` 中（非 `app/driver/CMakeLists.txt`）。`add_subdirectory(vendors/XYZ)` 及其缓存变量也在根 CMakeLists.txt 中，以 `if(ENABLE_XYZ)` 门控：

```cmake
# 根 CMakeLists.txt — 选项声明 + 厂商子目录
option(ENABLE_XYZ "Enable XYZ device support" OFF)

if(ENABLE_XYZ)
    set(XYZ_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vendors/XYZ" CACHE PATH "" FORCE)
    # ... 厂商特定缓存变量 ...
    add_subdirectory(vendors/XYZ)
endif()
```

同时更新 FATAL_ERROR 守卫条件（当前在第 57 行）：

```cmake
# 当前：
if(NOT ENABLE_ORBBEC AND NOT ENABLE_RS_AC1)
    message(FATAL_ERROR "Both ENABLE_ORBBEC and ENABLE_RS_AC1 are OFF ...")
endif()

# 添加新厂商后更新为：
if(NOT ENABLE_ORBBEC AND NOT ENABLE_RS_AC1 AND NOT ENABLE_XYZ)
    message(FATAL_ERROR "No vendor SDK selected, aborting")
endif()
```

#### app/driver/CMakeLists.txt

按已有模式新增 `if(ENABLE_XYZ)` 块，包括源文件、包含路径、编译定义和链接库：

```cmake
if(ENABLE_XYZ)
    target_include_directories(nio_drivers PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/xyz
    )

    target_sources(nio_drivers PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/xyz/nio_xyz_adapter.hpp
        ${CMAKE_CURRENT_LIST_DIR}/xyz/nio_xyz_frame_adapter.hpp
        ${CMAKE_CURRENT_LIST_DIR}/xyz/nio_xyz_device.hpp
        ${CMAKE_CURRENT_LIST_DIR}/xyz/nio_xyz_device.cpp
    )

    target_compile_definitions(nio_drivers PUBLIC
        ENABLE_XYZ
    )

    target_link_libraries(nio_drivers PUBLIC
        XYZ::SDK
    )

    message(STATUS "XYZ support enabled")
endif()
```

司机工厂源文件条件也需扩展（当前在第 94 行）：

```cmake
# 当前：
if(ENABLE_ORBBEC OR ENABLE_RS_AC1)

# 更新为：
if(ENABLE_ORBBEC OR ENABLE_RS_AC1 OR ENABLE_XYZ)
```

---

## 6. D2C 对齐策略

| 场景 | 模式 | 实现 |
|------|------|------|
| **仅 HW D2C**（如 RS-AC1） | `getAlignMode()` 返回 `HW`，`getD2CAlignFilter()` 返回 nullptr | CaptureSession 直接从 `NioFrameSet` 提取 color/depth 数据；无 SW 对齐步骤 |
| **仅 SW D2C** | `getAlignMode()` 返回 `SW`，`getD2CAlignFilter()` 返回有效 `NioD2CAlign*` | CaptureSession 调用 `alignFilter->process(nioFs->nativeFrameSet, out)`；nativeFrameSet 必须填充 |
| **HW 优先，SW 回退**（如 Orbbec） | `setupPipeline()` 查询 HW 能力；不可用时回退到 SW | `checkHWD2CSupport()` → `setAlignMode(HW or SW)` |
| **无 D2C** | `getAlignMode()` 返回 `NONE`，`getD2CAlignFilter()` 返回 nullptr | 不执行对齐 |

**注意**：`nativeFrameSet` 是过渡性设计，存在是因为 `NioD2CAlign::process()` 需要原始 SDK 帧执行厂商特定对齐。若你的厂商对齐可仅基于原始像素数据工作，可在 `process()` 实现中不依赖 `nativeFrameSet`（传 `nullptr`，直接操作 `NioFrame::data`）。

融合决策流程（`app/capture/nio_capture_session.cpp:164-208`）：

```cpp
hwD2CMode_ = (pipeline_->getAlignMode() == NioAlignMode::HW);
if (!hwD2CMode_) {
    alignFilter = pipeline_->getD2CAlignFilter();  // SW 模式获取对齐滤镜
}
```

---

## 7. 数据流图

```
厂商 SDK 回调
  │
  ▼  xyzFrameSetToNio() — 深拷贝像素数据
  │  nioFs->nativeFrameSet = static_pointer_cast<void>(vendorFs)  [仅 SW D2C]
  │
NioFrameSet（拥有像素数据 + 可选不透明原生句柄）
  │
  ▼  videoCallback_(nioFs)  —  SDK 回调中零阻塞
  │
VideoFrameQueue（有界 SPSC，容量 8）
  │
  ▼  Video 消费者线程 pop
  │
  ├──► FusionTask (HW: 从 NioFrame 取 color/depth)
  │                 (SW: enqueueNioFrameSet → alignFilter->process())
  ├──► ColorFrameConsumer → H264 编码器 + viewer
  ├──► DepthFrameConsumer → jet colormap H264 + .raw 文件 + viewer
  ├──► IRFrameConsumer → H264 + viewer
  └──► PointCloudConsumer → PCD 文件
```

文件创建门控条件（`app/capture/nio_capture_session.cpp:59-158`）：

| 文件 | 门控条件 |
|------|----------|
| `*_color_*.h264` | `hasColor && colorFormat != UNKNOWN` |
| `*_depth_*.h264` + `*_depth_raw_*.raw` | `hasDepth && depthFormat != UNKNOWN` |
| `*_ir_left_*.h264` | `hasIRLeft` |
| `*_ir_right_*.h264` | `hasIRRight` |
| `*_imu_*.txt` | `hasAccel && hasGyro` |
| `*_point_raw_*.raw` | `pipeline->isPointCloudDepth()` |
| `*_d2c_fused_*.h264` | `canFuse`（= hasColor && hasDepth） |

---

## 8. 故障排查

| 现象 | 可能原因 | 修复方法 |
|------|----------|----------|
| 编译错误：`NIO_DEVICE_VID` / `isGemini305*` 出现在 core | 厂商特定 VID/PID 检查泄漏到 core | 移至 `app/driver/VENDOR/nio_xyz_adapter.hpp`，使用 `VENDOR_DEVICE_VID` 常量 + inline 函数 |
| 编译错误：`ob::` / `OB_` 出现在非 driver 文件 | SDK 类型在 driver 层外被使用 | 替换为 `Nio*` 等价物；将 SDK 特定逻辑移至适配器 |
| 消费者线程 Segfault | `NioFrame::data` 为空或 `rawData()` 返回 nullptr | 检查 `xyzFrameSetToNio()` 是否通过 `nf.data.assign()` 拷贝像素数据 |
| SW D2C 输出乱码 | `nativeFrameSet` 为 null 或类型错误 | 确保 `start()` 中设置 `nioFs->nativeFrameSet = static_pointer_cast<void>(vendorFs)`，`XyzD2CAlign::process()` 中正确还原类型 |
| 重复帧 / 时间戳混乱 | 厂商回调在不正确线程触发 / 未同步 | 使用 mutex + ready 标志处理异步源（见 RsPipeline 模式） |
| Pipeline start 静默失败 | 异常在 `start()` 中被吞 | 检查 `NIO_LOG_ERROR_S` 输出；在调用厂商 start 前确认配置完整 |
| `setupPipeline()` 返回全零 `NioSensorInfo` | 下转为具体 Pipeline 类型失败 | 确保工厂创建正确的 `XyzPipeline` 类型与 `XyzDevice` 配对 |
| 多设备 USB 内存错误 | `usbfs_memory_mb` 太低 | `echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb` |
| 驱动工厂链接错误 | `discoverDevices()` 未编译 | 确认 `app/driver/CMakeLists.txt:94` 条件包含新厂商宏 |

---

## 9. 待解决 / 开放项

1. **`NioFrameSet::nativeFrameSet`** 是过渡性设计。目标：当所有 D2C 对齐可基于 `NioFrame::data` 工作时移除。
2. **`NioPipeline::enableStream(NioStreamConfig)`** 对 Orbbec 当前为 no-op（流在 `setupPipeline()` 内启用）。未来重构应使流启停完全通过此 API 配置。
3. **设备属性 ID**（`getIntProperty(int)`）传递厂商原始 ID。若更多厂商需要，应抽象为 `NioPropertyID` 枚举。
4. **点云深度**（`isPointCloudDepth() == true`）当前仅 RS-AC1 使用。96×288 合成深度图是厂商特定的。不同点云布局的厂商可能需要新的深度抽象。
5. **FATAL_ERROR 守卫**：根 `CMakeLists.txt` 要求至少一个 `ENABLE_*` 选项为 ON。添加新厂商时需更新守卫条件。
