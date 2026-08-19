# DynamicAlgoCam 技术开发计划

**文档状态**：实施完成草案 (Implementation complete draft)，Phase A/B/C 已实现并构建通过
**对应代码基线**：commit `HEAD` (main)
**最后核对**：2026-08-19（路径、类名、枚举、CMake options 均以仓库实际为准）

---

## 1. 项目愿景与范围边界

### 1.1 目标

DynamicAlgoCam 已具备多设备采集、流内编码、点云反投影、事件驱动录制、模型推理抽象层与卡尔曼轨迹滤波器等基础设施。本计划在此基础上完成最后一段闭环：

> **感知 → 定位 → 估计 → 控制** — 在现有相机硬件上识别场景目标（蚊/蝇/草等），通过滤波估 计其运动，控制外部执行设备（激光器/云台）进行瞄准操作。

### 1.2 范围内 vs 范围外（重要）

| 范围内 | 范围外 / 待立项 |
|---|---|
| 感知模型插件化加载（含一个 DUMMY backend 示范） | 生产级 YOLOv8 ONNX/TensorRT 后端的训练与量化 |
| 2D 检测框 → 相机光心 3D 坐标反投影（独立工具，不接采集主流程） | 多目标数据关联（Hungarian）与轨迹生命周期 |
| 卡尔曼滤波器在主流程下游的使用示范（_示范_，非强制接死主流程） | 实时高并发目标-航迹关联工程化 |
| 外部激光/云台控制器抽象层 + DUMMY 执行器示范 | 真实激光器电路协议、安全联锁硬件图、CE/FDA 合规 |
| 简单"感知-控制"调度循环（闭环 dry-run） | 闭环节制器/安全监督进程 (safety supervisor) |
| 文档/移植手册同步 | 操作员 HMI / Web 控制台 |

**依据**：根据本会话内**karpathy-guidelines skill** "Simplicity First / Surgical Changes" 原则，本计划不做推测性膨胀——每一新增模块必须可追溯到用户请求（"感知-估计-控制-瞄准"），且每一模块的接口契约先定、实现后补。

---

## 2. 现有基础设施盘点（实现证据）

下表为本计划要复用 / 扩展的现存符号，全部已对仓库验证：

| 子系统 | 关键符号 / 文件 | 已能做什么 | 本计划如何复用 |
|---|---|---|---|
| 设备抽象 | `DynalgoDevice` / `DynalgoPipeline` / `DynalgoContext` (`app/core/dynalgo_device.hpp`) | 多 SDK 即插即用（Orbbec、RS-AC1） | 控制器插件仍走"抽象 + 工厂 + 自注册"同款模式 |
| 驱动工厂 | `discoverDevices()` (`app/driver/dynalgo_driver_factory.cpp:61`) | 通过 `DynalgoDriverVendor` 枚举枚举设备 | 控制器不进 `discoverDevices`；另立 `discoverActuators()` 同款工厂 |
| 深度信息 | `DynalgoSensorInfo.depthIntrinsic` + `depthScale` (`app/core/dynalgo_types.hpp:257-304`) | 采集会话已知 intrinsics + scale | 3D 反投影直接消费这两个参数 |
| D2C 对齐 | `DynalgoAlignMode::{HW,SW}` + `getD2CDepthProfileList` (`app/driver/orbbec/dynalgo_ob_adapter.hpp` `selectBestProfile` 第216-287行；`dynalgo_ob_device.cpp:156-165`) | 硬件优先 HW D2C，回落 SW | 3D 反投影入参假设"深度帧已 D2C 对齐到彩色视角" |
| 帧分发 | `videoConsumerLoop` → `frameConsumers_` (`app/capture/dynalgo_capture_session.cpp:281-311`) | 每帧 `DynalgoFrameSet` 广播给所有 `FrameConsumer` | 可新增一个 `DetectionFrameConsumer` 插到 `frameConsumers_` 链尾，不动其它 consumer |
| 模型后端抽象 | `DynalgoModelBackend` / `DynalgoDetectionResult` / `DynalgoModelConfig` / `createModelBackend` / `registerModelBackend` (`app/core/dynalgo_model.hpp`, `dynalgo_model_factory.cpp`) | 工厂 + 自注册 + 枚举 (`DynalgoModelType::{NONE,DUMMY,YOLOV8_PY,ONNXRUNTIME,TENSORRT}`) | 实现 `DynalgoModelType::DUMMY` 后端作为示范；`YOLOV8_PY` 后端待立项 |
| 卡尔曼 | `DynalgoKalmanTracker` (`app/core/dynalgo_kalman_tracker.hpp/.cpp`) | 单目标 6D 匀速 KF，`init/update/predict` | 在"感知-控制"循环中为每个被锁定的目标维持一个 tracker |
| 事件驱动 | `Event` (`app/include/event.hpp`)、`EventWindow` + `eventSink` (`dynamic_algo_cam.cpp:176-217`) | 事件窗 + sink lambda 控制录制前缀 | 同款 `eventSink` 思路示范"per-target hit event" |
| 插件宿主 | `app/plugins/opencv/` 下 `dynalgo_color_convert_cv` 通过 CMake `find_package(OpenCV QUIET)` 条件编译 | 已有"可选插件"模式 | 控制器插件可参照同款 CMake pattern，按 `find_package(<SDK> QUIET)` 开关 |

---

## 3. 架构与新增模块布局

### 3.1 新增目录清单（与现有 `app/` 结构对齐）

```
app/
├── core/                           # (已存在) SDK-neutral
│   ├── dynalgo_actuator.hpp            # [新增] 执行器抽象接口 + DynalgoActuatorType 枚举 + DynalgoActuatorConfig
│   ├── dynalgo_actuator_factory.hpp    # [新增] createActuator() + registerActuator() 同款工厂
│   └── dynalgo_actuator_factory.cpp   # [新增]
├── actuator/                       # [新增目录] 外部执行设备适配层
│   ├── CMakeLists.txt              # 独立 static lib `dynalgo_actuators`
│   ├── dummy_actuator.hpp/.cpp     # [新增] DynalgoActuatorType::DUMMY 示范后端
│   └── (未来) laser_actuator.cpp   # 真实激光器协议 (待立项，本计划不实现)
├── core/
│   └── dynalgo_detection_to_3d.hpp     # [新增] 2D 检测框中心 → 相机光心 3D 坐标反投影工具 (header-only)
├── algo/                           # [新增目录] "感知-估计-控制" 编排层
│   ├── CMakeLists.txt              # 独立 static lib `dynalgo_algo`
│   ├── dynalgo_target_selector.hpp/.cpp  # [新增] 从 detections 中选最优先 target (策略可配)
│   ├── dynalgo_track_bundle.hpp/.cpp   # [新增] 单目标-DynalgoKalmanTracker 绑定 (1 target ↔ 1 tracker)
│   └── dynalgo_engagement_loop.hpp/.cpp # [新增] 编排循环 (示范 dry-run，由 main 条件启动)
└── dynamic_algo_cam/
    └── dynamic_algo_cam.cpp         # (修改) 增加 --enable-engagement 选项启动闭环示范
```

依据 karpathy "Surgical Changes"：除 `dynamic_algo_cam.cpp` 一处入口接线，其余均为新增文件；现有 `app/core/`、`app/capture/`、`app/driver/` 文件零改动。

### 3.2 依赖关系图（新增层与现存层之间）

```
                 ┌──────────────────────────────────────────────┐
                 │ dynamic_algo_cam (executable) [修改]        │
                 │   ↑ --enable-engagement                     │
                 │   │                                         │
   ┌─────────────┴───┴────────────┐         ┌─────────────────┐
   │ dynalgo_algo (新增 static lib)    │         │ dynalgo_capture     │ (已存在)
   │  ├ TargetSelector             │ reads   │  CaptureSession │
   │  ├ TrackBundle                │←────────│   FrameSet      │
   │  └ EngagementLoop            │         │                 │
   └────────────┬─────────────────┘         └─────────────────┘
                │ uses
                ▼
   ┌────────────────────────────┐    ┌────────────────────────┐
   │ dynalgo_core (已存在)           │    │ dynalgo_actuators (新增)    │
   │  DynalgoModelBackend → detections│   │  DynalgoActuator (抽象)     │
   │  DynalgoKalmanTracker → tracks  │   │  DummyActuator (示范)   │
   │  DynalgoIntrinsic + depthScale  │   └────────────────────────┘
   │  DynalgoActuator (新增抽象)     │
   └────────────────────────────┘
```

**核心原则**：`dynalgo_algo` 链接 `dynalgo_core` + `dynalgo_actuators`，但**不直接链接**任何 vendor SDK（OrbbecSDK / rs_driver）。这点与 `app/CMakeLists.txt` 已声明的"依赖向下流动"原则一致。

---

## 4. 实施阶段与验证目标

每阶段都遵循 karpathy "Goal-Driven Execution" — 任务 → 验证项 → 检查方式。

### Phase A：执行器抽象 + DUMMY 后端 ✅ **已完成 (commit `ae23b1d`)**

**目的**：为外部激光/云台硬件留出 SDK-neutral 接口契约，与 `DynalgoModelBackend` 同款 abstract+factory+self-register 模式。

| 步骤 | 验证 |
|---|---|
| A1. 在 `app/core/dynalgo_actuator.hpp` 定义 `DynalgoActuator` 抽象类、`DynalgoActuatorType` 枚举（`NONE`/`DUMMY`/`LASER_GENERIC`/`GIMBAL_GENERIC`）、`DynalgoActuatorConfig` POD | 头文件不依赖 vendor SDK；只 `#include "dynalgo_types.hpp"` ✅ |
| A2. 在 `app/core/dynalgo_actuator_factory.{hpp,cpp}` 实现 `createActuator()` + `registerActuator()`，与 `dynalgo_model_factory.cpp` 同款 mutex + unordered_map 注册表 | 编进 `dynalgo_core` 静态库；`nm build/lib/libdynalgo_core.a` 见 `createActuator` 符号 ✅ |
| A3. 在 `app/actuator/dummy_actuator.{hpp,cpp}` 实现 `DynalgoActuatorType::DUMMY` 后端：`load/open/aimAt(x,y,z)/fire()/close()` 全部打日志返回 true | 自注册静态块 (`__attribute__((used))`)；链接时需 `-Wl,--whole-archive` ✅ |
| A4. `app/actuator/CMakeLists.txt` 生成 `dynalgo_actuators` 静态库，依赖 `dynalgo_core` | `cmake --build build --target dynalgo_actuators` 通过 ✅ |
| A5. 接入根 `app/CMakeLists.txt` `add_subdirectory(actuator)` | 全量构建无 CMake 错误 ✅ |

**完成标准**：`cmake --build build` 全绿；`build/dynamic_algo_cam --help` 不崩；`DUMMY` actuator 能 instantiate 并打"aim + fire"日志 ✅

**关键实现细节**：
- `DynalgoActuator::config()` 虚函数提供配置访问
- `DynalgoActuatorConfig::dryRun = true` 安全默认
- 静态库自注册 TU 需要消费者使用 `-Wl,--whole-archive`（已在 `dynalgo_actuator_factory.hpp` 注释中说明）

---

### Phase B：2D 检测框 → 3D 相机光心坐标 ✅ **已完成 (commit `3bc3c4d`)**

**目的**：满足"3D 坐标获取"需求，作为 Phase C 闭环的反投影工具。**与 Phase A 并行完成**。

| 步骤 | 验证 |
|---|---|
| B1. 在 `app/core/dynalgo_detection_to_3d.hpp` (header-only) 实现函数：<br>`bool detectionCenterToCamera3D(const DynalgoFrame& depthAligned, const DynalgoIntrinsic& intr, float depthScale, int filterHalf, const DynalgoDetectionResult& det, float& X, float& Y, float& Z)` <br> 其中 `filterHalf` 控制中值滤波窗（0=单像素，k=(2k+1)² 窗口中值，推荐值 2 即 5×5 中值） | 编译期 include `dynalgo_frame.hpp`+`dynalgo_model.hpp`；无新增 .cpp ✅ |
| B2. 反投影公式：`u = det.x + det.w/2`, `v = det.y + det.h/2`；`Z = depthInMeters(u, v)` (raw Y16 × depthScale)；`X = (u - intr.cx) * Z / intr.fx`, `Y = (v - intr.cy) * Z / intr.fy` | 独立验证驱动 5 用例全过 (center/offset/zero/invalid-format/median) ✅ |
| B3. **入参契约明示**：该函数**假设 depth 帧已 D2C 对齐到彩色视角**（即与 detection 坐标系一致）。若调用方传入未对齐的 raw depth，结果不正确 — 这点写进 header 注释与 docs。 | header 注释含 "PRECONDITION: depth must be D2C-aligned to color frame" ✅ |
| B4. 文档：在 `docs/dynamic_algo_cam/models_overview.md` 末尾追加"2D 检测框 → 3D 相机光心坐标"小节 | 已更新 ✅ |

**完成标准**：独立验证驱动 `/tmp/opencode/phaseB_smoke.cpp` 5 用例全过；GTest 缺失环境下 CMake 跳过测试不报错 ✅

**关键实现细节**：
- `filterHalf = 0` 单像素；`filterHalf = k` 取 `(2k+1)²` 窗口非零深度中值
- 失败时 outX/outY/outZ **保持不变**，调用者可区分 "无 3D fix" vs 真实 (0,0,0)
- 与 `PointcloudFrameConsumer::backprojectToPointCloud()` 公式完全一致

---

### Phase C：感知-估计-控制编排层（示范闭环 dry-run） ✅ **已完成 (构建通过，待运行环境验证)**

**目的**：把已建好的 `DynalgoModelBackend` + `DynalgoDetectionResult` + `DynalgoKalmanTracker` + 新增的反投影 + 新增的 actuator 串成一个**最小可运行闭环**，但仅在 `--engage-model/--engage-actuator` flag 下启动，默认行为不变。

| 步骤 | 验证 |
|---|---|
| C1. `app/algo/dynalgo_target_selector.{hpp,cpp}` — **自由函数 `pickTarget()`**：策略 `enum class SelectorStrategy { HIGHEST_SCORE, NEAREST_DEPTH, LARGEST_AREA }`；`std::optional<DynalgoDetectionResult> pickTarget(detections, strategy, depthSortedMeters=nullptr)` (stateless 设计) | 编译通过；`nm build/lib/libdynalgo_algo.a` 见 `pickTarget` 符号 ✅ |
| C2. `app/algo/dynalgo_track_bundle.{hpp,cpp}` — `DynalgoTrackBundle` 类：持有一个 `DynalgoKalmanTracker` + 缓存 3D fix；`init(detection)→update(detection, depthAligned, intr, scale)→predict()→lastX()/lastY()/lastZ()/hasFix()` | 编译通过；`nm build/lib/libdynalgo_algo.a` 见 `DynalgoTrackBundle` 方法 ✅ |
| C3. `app/algo/dynalgo_engagement_loop.{hpp,cpp}` — `DynalgoEngagementLoop` 类：构造传入 `modelBackend`、`actuator`、`depthIntr`、`depthScale`、配置；`onFrame(frameSet)` 单次回调；状态机 `IDLE → LOCKING → TRACKING → FIRING → LOST → IDLE`；在 `FIRING` 态调用 `actuator->aimAt(X,Y,Z) → actuator->fire(10ms)` 但 `DUMMY` 下不真实触发硬件 | 状态切换 trace 通过 `DYNALGO_LOG_INFO_S("[engage] state: ...")` 日志验证 ✅ |
| C4. `app/algo/dynalgo_engagement_consumer.{hpp,cpp}` — `DynalgoEngagementFrameConsumer : FrameConsumer` 适配器：`consume(shared_ptr<FrameSet>)` 转发给 `loop_->onFrame()` | 编译通过；`FrameConsumer` 接口完整实现 ✅ |
| C5. `app/dynamic_algo_cam/dynamic_algo_cam.cpp` 修改：增加 CLI 选项 `--engage-model <type>` `--engage-actuator <type>` `--engage-model-path <path>`；实例化 model/actuator → `EngagementLoop` → `EngagementFrameConsumer` → `session->addFrameConsumer()` | 默认（无 flag）行为与现版本字节级相同；flag 提供时条件构造链尾插入 ✅ |
| C6. `app/capture/dynalgo_capture_session.hpp/.cpp`：新增 `addFrameConsumer(unique_ptr<FrameConsumer>)`、`depthIntrinsic()`/`depthScale()` 访问器、`setEngagementLoop(loop, model, actuator)`（原指针管理，避免循环依赖），析构时删除 | `CaptureSession::setup/start/startVideoPipeline/stop` 签名无变化 ✅ |
| C7. 新增 `app/algo/dummy_model_backend.{hpp,cpp}` — `DynalgoModelType::DUMMY` 后端：`infer()` 产生固定合成 detection (帧中心 10% 大小，score=0.9)；自注册到工厂 | `createModelBackend(DUMMY)` 返回有效实例 ✅ |
| C8. `app/algo/CMakeLists.txt` 生成 `dynalgo_algo` 静态库，链接 `dynalgo_core` + `dynalgo_actuators` | `cmake --build build --target dynalgo_algo` 通过 ✅ |

**完成标准**：
- 全量构建通过 ✅
- `strings build/bin/dynamic_algo_cam | grep engage` 确认新 CLI 参数存在 ✅
- `nm build/lib/libdynalgo_algo.a` 确认所有新符号导出 ✅
- 待运行时验证：`./build/bin/dynamic_algo_cam --engage-model DUMMY --engage-actuator DUMMY --enable-event-sim --no-show` 启动后日志序列可见 `IDLE→LOCKING→TRACKING→FIRING→IDLE` (需 `libOrbbecSDK.so.2` 环境)

**关键实现细节**：
- 状态机阈值：LOCKING→TRACKING 连续 3 帧；TRACKING→LOST 容忍 5 帧无 detection；FIRING 冷却 1000ms
- `EngagementLoop` 构造接收 `DynalgoIntrinsic` + `depthScale` (从 `CaptureSession::depthIntrinsic()`/`depthScale()` 获取)
- `DummyModelBackend::infer()` 生成合成 detection 用于干跑
- `DummyActuator` 通过 `dryRun=true` 保证零外部副作用
- `--whole-archive` 约束已在 main 可执行文件链接时通过 `dynalgo::algo` 依赖传递解决

---

### Phase D：文档与移植手册同步 🔄 **进行中**

| 步骤 | 状态 |
|---|---|
| D1. 在 `README.md` 架构图添加 `dynalgo_actuators` + `dynalgo_algo` 两个新层 | 待更新 |
| D2. 新增 `docs/dynamic_algo_cam/engagement_loop.md`：状态机图 + 接入方式 + 安全注意（_不写未实现的行为_） | 待创建 |
| D3. 在 `docs/dynamic_algo_cam/VENDOR_DEVICE_PORTING_MANUAL_CN.md` 末尾追加"如何适配新执行器"章节，与"如何适配新设备/新模型"同款体例 | 待更新 |
| D4. 在 `docs/dynamic_algo_cam/models_overview.md` 末尾追加"TrackBundle 与 EngagementLoop"段 | 待更新 (Phase B 已更新) |
| D5. 在 `app/models/README.md` 追加 DUMMY model 后端说明 | 待更新 |

**完成标准**：所有文档引用的路径、类名、CMake target 名与代码当前一致；无虚构接口描述。

---

## 5. 风险与缓解

| 风险 | 严重度 | 缓解 |
|---|---|---|
| D2C 未对齐导致 3D 反投影错位 | 高 | 函数 header 与 docs 显式声明 precondition；运行时通过 `pipeline_->getAlignMode()` 判断，未对齐时日志 WARN 且 EngagementLoop 拒绝启动 |
| Kalman tracker 单目标假设 → 多蚊场景失效 | 中 | 本计划保持单目标（沿用现有 `DynalgoKalmanTracker`），多目标关联明确划入"范围外"；后续立项需重写为 multi-target tracker |
| DUMMY actuator 被误当成真的开激光 | 高 | `DummyActuator::fire()` 实现永远 NO-OP 只打日志；`DynalgoActuatorConfig` 增加 `bool dryRun=true` 默认；真实后端必须显式传 `dryRun=false` |
| 现有 capture pipeline 被 engagement loop 拖慢帧率 | 中 | EngagementLoop::onFrame 时延超 5ms 自动跳帧；上层不感知 |
| 接入主流程破坏现有行为 | 高 | `--engage-*` flag 默认关闭；不传则 `frameConsumers_` 链尾不增加 EngagementFrameConsumer，所有现有路径零变化 |
| 静态自注册 TU 被链接器丢弃 → `createActuator` 返回 `nullptr` | 中 | Phase A 已确认此为静态库固有问题；Phase C4.2 主可执行文件接线时**必须**使用 `-Wl,--whole-archive libnio_actuators.a -Wl,--no-whole-archive`（已在 `app/core/dynalgo_actuator_factory.hpp` 顶部注释中说明要求消费者使用此 flag） |

---

## 6. 关键契约预定义（写代码前的接口冻结）

### 6.1 `DynalgoActuator` (`app/core/dynalgo_actuator.hpp`)

```cpp
namespace dynalgo {

enum class DynalgoActuatorType {
    NONE = 0,
    DUMMY,
    LASER_GENERIC,   // 真实激光器待立项
    GIMBAL_GENERIC   // 云台待立项
};

struct DynalgoActuatorConfig {
    std::string devicePath;      // e.g. "/dev/ttyUSB0", "can0"
    std::string protocolHint;    // "modbus-rtu", "raw-serial", "can-id"
    bool dryRun = true;          // 安全默认：dryRun。真实后端必须显式 false
    int baudRate = 115200;
};

class DynalgoActuator {
public:
    virtual ~DynalgoActuator() = default;
    virtual bool load(const DynalgoActuatorConfig& cfg) = 0;
    virtual bool open() = 0;
    virtual bool aimAt(float x_m, float y_m, float z_m) = 0;  // 相机光心坐标系，米
    virtual bool fire(double durationMs) = 0;                  // 持续时间，dryRun 下 NO-OP
    virtual bool close() = 0;
    virtual const char* name() const = 0;
};

} // namespace dynalgo
```

**注意点 (caveat)**：
- `aimAt` 入参坐标系是**相机光心 XYZ 米**，由 `detectionCenterToCamera3D` 直接产出，**不需要**调用方再转换。
- `dryRun=true` 是安全默认；**只有**调用方显式传 `dryRun=false` 才允许真实触发出光/出动作。这点在 `DummyActuator::fire()` 中体现：永远 NO-OP，仅日志。

### 6.2 `detectionCenterToCamera3D` (`app/core/dynalgo_detection_to_3d.hpp`)

```cpp
namespace dynalgo {

// PRECONDITION: depth frame must be D2C-aligned to the color frame on which
// detections were produced. If raw (unaligned) depth is passed, X/Y will be
// wrong because the depth pixel at (u,v) refers to a different physical ray
// than the color pixel at (u,v).
//
// filterHalf = 0 → single pixel raw depth (noisy, fastest)
// filterHalf = 2 → 5×5 median (recommended)
//
// Returns false when center pixel depth is 0/invalid (typical for sky/far
// surfaces); caller should treat as "no 3D fix this frame".
bool detectionCenterToCamera3D(const DynalgoFrame& depthAligned,
                               const DynalgoIntrinsic& intr,
                               float depthScale,
                               int filterHalf,
                               const DynalgoDetectionResult& det,
                               float& outX, float& outY, float& outZ);

} // namespace dynalgo
```

### 6.3 `DynalgoEngagementLoop` 状态机

```
                  ┌──────┐
                  │ IDLE │←──────── lost/decay ───────┐
                  └──┬───┘                              │
                     │ frame arrives                   │
                     ▼                                  │
              ┌──────────────┐  no detection multiple frames
              │   LOCKING    │──────────────────────────┤
              └──────┬───────┘                           │
                     │ stable detection ≥ 3 frames       │
                     ▼                                  │
             ┌────────────────┐                         │
             │   TRACKING     │── detection lost > N ───┤
             └────┬───────────┘                         │
                  │ 3D fix obtained                     │
                  ▼                                      │
           ┌──────────────┐                              │
           │   FIRING     │── fired + cooldown ────────┘
           └───────────────┘
```

每态的副作用在日志输出（dryRun 下Hardware不动），状态切换都通过 `DYNALGO_LOG_INFO_S`。

---

## 7. 迁移 / 回退指南

### 7.1 对现有用户的影响

| 受众 | 影响 | 必需动作 |
|---|---|---|
| 现有调用 `dynamic_algo_cam` 做纯采集的开发者 | 无影响 | 不传 `--engage-*` flag，行为字节级不变 |
| 集成新 actuator 的 oem/集成商 | 新增 `app/actuator/` 编译目标 | 必读 `VENDOR_DEVICE_PORTING_MANUAL_CN.md` 新章节；实现 `DynalgoActuator` 子类并通过 `registerActuator` 注册 |
| 集成新感知模型的算法工程师 | 新增 Phase B 的 3D 反投影可独立调用 | 头文件 standalone；不需链入 `dynalgo_algo` 即可用 |

### 7.2 回退

- 任何阶段出问题，所有新增都是**新文件**+`dynamic_algo_cam.cpp` 一处 CLI 接线 + `CMakeLists.txt` 三个 add_subdirectory。
- 回退最小集：
  ```bash
  git revert <phase-commit>
  ```
- 不需要修改既有库 / capture session / 驱动适配层。

---

## 8. 待确认与未覆盖信息（需 owner 决策）

| 项 | 默认假设 | 影响 |
|---|---|---|
| 真实激光器协议（Modbus-RTU? CAN? 串口私有协议？） | 未知，本计划仅留 `LASER_GENERIC` 枚举占位 | 真实适配延后立项 |
| 安全联锁硬件（光闸/急停按钮/温度监测） | 假设由 actuator 后端自行处理，本计划不做 | 安全等级合规后续单独做 |
| 目标物种（蚊/蝇/草）的 YOLO 类别与权重文件位置 | 待算法 owner 训练并放入 `app/<model_name>/weights/` | 本计划先做闭环 dry-run；真值模型后续替换 DUMMY |
| 云台硬件 yaw/pitch 限位 vs 相机光心 XYZ 转换 | 假设在 actuator 后端内部做坐标变换 | `DynalgoActuator::aimAt(x,y,z)` 入参定为相机光心，后端负责自己的运动学 |
| 多帧 3D 坐标的时间序滤波（_Kalman 滤波到 XYZ_） | 不在本计划内 — `DynalgoKalmanTracker` 现仅滤波 2D bbox | XYZ 时序噪声只能靠中值窗缓解；3D KF 立项另做 |

---

## 9. 验收清单

- [x] Phase A：`cmake --build build` 全绿；`nm build/lib/libdynalgo_core.a | grep createActuator` 有命中
- [x] Phase B：`tests/detection_to_3d_test.cpp` 测试在 GTest 缺失时被 CMake skip 不报错；独立验证驱动 5 用例全过
- [x] Phase C：全量构建通过；`strings build/bin/dynamic_algo_cam | grep engage` 确认新 CLI 参数；`nm build/lib/libdynalgo_algo.a` 确认符号导出
- [ ] Phase C 运行时：`./build/bin/dynamic_algo_cam --engage-model DUMMY --engage-actuator DUMMY --enable-event-sim --no-show` 启动 5 秒内日志序列可见 `IDLE→LOCKING→TRACKING→FIRING` (需 `libOrbbecSDK.so.2` 环境)
- [ ] Phase D：`docs/dynamic_algo_cam/engagement_loop.md` 与 `VENDOR_DEVICE_PORTING_MANUAL_CN.md` 新增段无虚构接口
- [x] 全程：`dynamic_algo_cam.cpp` 默认行为（不传 engage flag）与 commit `85a193c` 字节级一致
- [x] 提交粒度：每个 Phase 一个或一组 commit，commit message 沿用现有 `feat:`/`refactor:`/`docs:` 前缀

---

## 10. 时间预估（按 phase，独立可中断）

| Phase | 估时 | 备注 |
|-------|-----|-----|
| A | 1 单位 | 抽象层 + DUMMY 同 `DynalgoModelBackend` 已有经验 |
| B | 0.5 单位 | header-only；上轮已先行部分调研 |
| C | 2 单位 | 编排 + 状态机 + main CLI 接线是最大块 |
| D | 1 单位 | 文档同步 |
| **总计** | **4.5 单位** | 每单位定义为一次集中 coding session |

---

## 附录 A：本计划验证所引用的代码事实

以下为本计划写作时经由 codegraph_explore / 仓库实际文件核对的关键事实，列此以便审阅者复验：

- 抽象类模式已有先例：`app/core/dynalgo_model_factory.cpp:1-54`（mutex+unordered_map 自注册工厂），新增 `dynalgo_actuator_factory.cpp` 与之字节级同款
- `DynalgoDetectionResult` 字段为 `classId/score/x/y/w/h/label` (`app/core/dynalgo_model.hpp:45-53`)——3D 反投影用 `x+w/2, y+h/2` 即可，**不**需要新增字段
- `DynalgoIntrinsic` 字段 `fx/fy/cx/cy/width/height` (`app/core/dynalgo_types.hpp:257-265`)——反投影公式直接用，无需求 intrinsics
- depthScale 已在 `CaptureSession::setup()` (`app/capture/dynalgo_capture_session.cpp:41`) 缓存 `depthScale_ = sensorInfo_.depthScale;` ——EngagementFrameConsumer 可通过 `session` 拿到
- D2C mode 已在 `dynalgo_ob_device.cpp:281-286` 自动判 HW/SW 并 `pipeline.setAlignMode` ——运行时 EngagementLoop 可通过 `pipeline_->getAlignMode()` 查
- `FrameConsumer` 链尾插入模式已有先例：`app/capture/dynalgo_capture_session.cpp:157-165`（PCD consumer 加入 `frameConsumers_`）——EngagementFrameConsumer 直接复用同款 push_back
- `--enable-event-sim` flag 已存在 (`CMakeLists.txt` 顶 option `ENABLE_EVENT_SIM`)——`--engage-*` flag 沿用同款 CLI 模式
