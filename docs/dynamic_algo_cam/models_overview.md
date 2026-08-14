# Models 子项目目录规划

工程在 `app/models/` 下纳入 **Python 算法 / 推理模型包**，与本工程主要的 C++ 采集应用
(`app/dynamic_algo_cam/`) 解耦。该目录永远不被 CMake 处理，保证 C++ 构建与 Python
runtime 分层清晰。

## 与现有目录的关系

| 目录 | 内容 | 构建方式 |
|------|------|----------|
| `vendors/` | 第三方 **C++ SDK**（OrbbecSDK、RoboSense） | 主 CMakeLists.txt 调用 `add_subdirectory()` |
| `app/dynamic_algo_cam/` | 工程主 C++ 可执行入口 | 主 CMake build |
| `app/models/` | 第三方 **Python 算法/推理包** | 独立 Python：`pip install -e ./app/models/<name>` 或直接 `python -m <package>` |

## 现已纳入

| 子目录 | 上游 | 版本 | License |
|--------|------|------|---------|
| `app/models/yolov8/` | [ultralytics/ultralytics](https://github.com/ultralytics/ultralytics) | 8.0.29 (`yolov8/ultralytics/__init__.py` 内 `__version__`) | **GPL-3.0** |

## YOLOv8 子目录结构（与上游一致）

```
app/models/yolov8/
├── ultralytics/             # 可 import 的 Python 包
├── tests/                   # 上游测试
├── docs/                    # 上游 mkdocs 文档源
├── examples/                # 官方用法示例
├── requirements.txt         # Python 依赖
├── setup.py / setup.cfg      # 上游打包文件
├── LICENSE                  # GPL-3.0
├── CITATION.cff / README.md # 上游元数据
└── docker/                  # 上游 docker 文件
```

本工程对 YOLOv8 不做源码裁剪——保持 GPL-3 要求的"源码可获取"。所有上游
`README.md`、`tests/`、`examples/`、`docs/` 一并保留，避免被 GPL-3 "choice of
medium" 之类条款触发风险。

## 使用方式

YOLOv8 包是 vendored 副本，不会编入 `dynamic_algo_cam`；可以独立运行：

```bash
# 一次性安装 Python 依赖
pip install -r app/models/yolov8/requirements.txt

# 以 editable 模式安装 vendored 包（推荐，便于 import）
pip install -e app/models/yolov8/

# 运行上游示例
python app/models/yolov8/examples/<script>.py
```

未来如需与 C++ 采集应用联动（例如摄像头帧喂入推理 → 检测框回灌），应通过周边
脚本/IPC 桥接，不要把 Python 调用混入 `app/dynamic_algo_cam.cpp` 主流程。

## 许可披露（强制要求）

YOLOv8 是 **GPL-3.0**，与本仓库主体 **MIT** 不兼容。一旦 YOLOv8 源码作为
combined work 的一部分对外发布，整体受 GPL-3.0 copyleft 约束。三种合规选择：

1. **继续 vendoring + GPL-3 披露 + 遵守 copyleft**——发布产物时随附完整 YOLOv8 源码，
   并明确告知衍生作品整体适用 GPL-3。
2. **以独立子进程/RPC 调用使用 YOLOv8，不将源码纳入工程产物**——可保留 MIT 主体；
   `app/models/yolov8/` 仅作开发期参考实现。
3. **从工程移除 YOLOv8**——彻底回避冲突。

详见根 `README.md` 的 License 段落与 `app/models/yolov8/LICENSE`。

## C++ 模型推理抽象层

工程主体（C++ 采集应用）需要一个**SDK-neutral 的模型推理接入点**，让 YOLOv8 或
后续其它模型可在不被采集主流程耦合的前提下接入。该抽象层落在 `app/core/`，与
现有 `DynalgoDevice` / `DynalgoPipeline` 同级，**不引入任何推理 SDK 依赖**：

| 文件 | 作用 |
|------|------|
| `app/core/dynalgo_model.hpp` | `DynalgoModelType` 枚举、`DynalgoDetectionResult` 结构、`DynalgoModelBackend` 抽象类、`DynalgoModelConfig` |
| `app/core/dynalgo_model_factory.hpp` | `createModelBackend(type)` 工厂入口 + `registerModelBackend` 注册钩子 |
| `app/core/dynalgo_model_factory.cpp` | 工厂实现 + 进程级注册表（mutex 保护），已编入 `libnio_core.a` |

### 接口契约

```cpp
enum class DynalgoModelType { NONE, DUMMY, YOLOV8_PY, ONNXRUNTIME, TENSORRT };

struct DynalgoDetectionResult {
    int   classId; float score;
    float x, y, w, h;     // 源帧坐标系下的 bounding box (像素)
    std::string label;
};

struct DynalgoModelConfig {
    std::string modelPath;    // weights/model file
    std::string deviceHint;   // "cpu"/"gpu"/"cuda:0"/...
    float confThreshold = 0.25f;
    float iouThreshold  = 0.45f;
};

class DynalgoModelBackend {
    virtual bool   load(const DynalgoModelConfig& cfg) = 0;
    virtual bool   infer(const DynalgoFrame& frame,
                         std::vector<DynalgoDetectionResult>& out) = 0;
    virtual const char* name() const = 0;
};

std::unique_ptr<DynalgoModelBackend> createModelBackend(DynalgoModelType type);
```

### 使用模式

```cpp
auto backend = dynalgo::createModelBackend(dynalgo::DynalgoModelType::YOLOV8_PY);
if (!backend) { /* 该 backend 未编入 */ }
DynalgoModelConfig cfg; cfg.modelPath = "yolov8n.pt";
if (backend->load(cfg)) {
    std::vector<DynalgoDetectionResult> results;
    if (backend->infer(currentFrame, results)) { /* 消费 results */ }
}
```

### 当前状态与边界

- **本阶段只加抽象层，不接 YOLOv8**。`createModelBackend(YOLOV8_PY)` 在尚未注册任何
  backend 时返回 `nullptr`，调用方应态然处理。
- 后续具体 backend（YOLOv8 Python 桥接、ONNX Runtime、TensorRT、Dummy 测试实现）应
  放 `app/driver/<name>/` 或独立子目录，在各自 `.cpp` 内用 `static-init + 宏守卫` 调用
  `registerModelBackend` 自注册（与 `DynalgoDriverFactory` 风格一致）。
- **采集主流程不直接依赖此抽象类**——这是专留给"算法线程"或"上层业务模块"的接口；
  PcdTask / 录文件 / SDL 预览均与此层无关。

### 已验证

- `cmake --build build --target dynamic_algo_cam` 通过；
- `nm build/lib/libnio_core.a | grep dynalgo::` 显示 `createModelBackend` 与 `registerModelBackend`
  符号已编入静态库。

## C++ 卡尔曼轨迹滤波器

工程以"先有 abstract infrastructure，不耦合采集主流程"为原则，新增了一个**单目标 2D
bbox 卡尔曼滤波器**，用于后续模型推理输出框的轨迹平滑与下一帧位置预测。

| 文件 | 作用 |
|------|------|
| `app/core/dynalgo_kalman_tracker.hpp` | `DynalgoKalmanTracker` 类：状态 `[cx,cy,w,h,vx,vy]`、匀速运动模型、`init/update/predict` 接口 |
| `app/core/dynalgo_kalman_tracker.cpp` | 完整 KF predict/update 实现，固定 6 维（用 `std::array<double,...>` 手写线性代数，**不引入 Eigen**），已编入 `libnio_core.a` |

### 接口契约

```cpp
class DynalgoKalmanTracker {
    void   init(const DynalgoDetectionResult& det);                              // 显式初始化（可选；首帧 update 会自动初始化）
    DynalgoDetectionResult update(const DynalgoDetectionResult& det);               // 吃测量并返回后验平滑 bbox
    DynalgoDetectionResult predict();                                           // 时间推进，给出下一帧先验 bbox
    bool   initialised() const;                                             // T 表示已 init 或已收到首帧测量
    // 可调参：dt_, processNoise_, measNoise_（成员变量，构造时给默认值）
};
```

### 使用模式

```cpp
dynalgo::DynalgoKalmanTracker trk;                // 每个被跟踪目标一个实例
trk.init(firstDetection);                 // 显式初始化（可选）
for (auto& det : frameDetections) {        // frameDetections 来自 DynalgoModelBackend::infer
    dynalgo::DynalgoDetectionResult smoothed = trk.update(det);  // 平滑当前帧
    dynalgo::DynalgoDetectionResult next     = trk.predict();    // 预测下一帧位置
}
```

### 范围与边界（与"集成卡尔曼滤波"请求严格匹配）

- **单目标**：本阶段实现的是单目标 KF，不包含多目标数据关联（Hungarian / Greedy 匹配、轨迹生命周期
  管理）。多目标跟踪需要上层业务模块在 `DynalgoModelBackend::infer` 输出与多个 `DynalgoKalmanTracker` 实例
  之间做匹配，属后续工作。
- **不接入采集主流程**：`DynalgoKalmanTracker` 当前不被 `app/dynamic_algo_cam.cpp` 或
  `PointcloudFrameConsumer` 等任何采集主路径代码调用；它是预留给"算法线程/上层业务模块"的基础设施。
- **无外部依赖**：dynalgo_core 保持 SDK-neutral，KF 用 `std::array` 手写线性代数，未拉 Eigen / OpenCV。

### 已验证

- `cmake --build build --target dynamic_algo_cam` 通过；
- `nm build/lib/libnio_core.a | grep DynalgoKalmanTracker` 显示
  `init`、`update`、`predict`、构造函数符号已编入静态库。

  > 注：上句"libnio_core.a"为重命名前的旧名；自工程命名重构（commit `25be197`）后实际路径为
  > `build/lib/libdynalgo_core.a`，命令相应演化为
  > `nm build/lib/libdynalgo_core.a | grep DynalgoKalmanTracker`。

---

## 2D 检测框 → 3D 相机光心坐标

实现：[`app/core/dynalgo_detection_to_3d.hpp`](../../app/core/dynalgo_detection_to_3d.hpp)
（header-only，挂在 `dynalgo::` namespace 内；不新增 `.cpp`）。

### 接口

```cpp
namespace dynalgo {

// PRECONDITION: depth frame MUST be D2C-aligned to the color frame on which
//               detections were produced. If a raw (unaligned) depth frame is
//               passed, X / Y will be wrong (the (u,v) pixel index refers to a
//               different physical ray than the color pixel at (u,v)).
//
// filterHalf = 0 → single-pixel raw depth (noisy but fastest)
// filterHalf = k → median over the (2k+1)×(2k+1) window around (u,v),
//                   ignoring zero / invalid depth pixels. Recommended: 2
//                   (5×5 window) for near-to-mid-range small targets.
//
// Returns false on:
//   - depthAligned.format != DynalgoFormat::Y16
//   - depthScale <= 0, intr.fx == 0 or intr.fy == 0 (degenerate intrinsic)
//   - depthAligned too small for the width×height×2-byte Y16 layout
//   - detection centre outside the frame
//   - valid (non-zero) pixel count in the window is zero (no 3D fix this frame)
//
// On false, outX / outY / outZ are left UNCHANGED so the caller can
// distinguish "no 3D fix this frame" from a real fix at (0,0,0).
bool detectionCenterToCamera3D(const DynalgoFrame& depthAligned,
                                const DynalgoIntrinsic& intr,
                                float depthScale,
                                int filterHalf,
                                const DynalgoDetectionResult& det,
                                float& outX, float& outY, float& outZ);

} // namespace dynalgo
```

### 数学公式

与 [`PointcloudFrameConsumer::backprojectToPointCloud()`]
(../../app/capture/dynalgo_frame_consumer.cpp) 现有反投影路径同一针孔模型：

```
u = det.x + det.w / 2          // detection 几何中心，整数像素
v = det.y + det.h / 2
Z = median_or_single_pixel_raw * depthScale    // Y16 单位 mm × scale → 米
X = (u - intr.cx) * Z / intr.fx
Y = (v - intr.cy) * Z / intr.fy
```

### 坐标系

原点位于相机光心；+X 向右、+Y 向下、+Z 向前（与 `PointcloudFrameConsumer`
输出云同系）。适合直接喂给后续 `DynalgoActuator::aimAt(X, Y, Z)` 接口。

### 推荐参数

| 场景 | filterHalf | 说明 |
|---|---|---|
| 小目标（蚊/蝇/草），近-中距离 | **2**（5×5 中值） | 抑制单像素噪声；足够小，对运动目标不糊位 |
| 远距小目标，散热面深噪声大 | 3（7×7 中值） | 平台倾向：更大窗抗压；对运动更糊 |
| 调试/快速验证 | 0（单像素） | 最快；对边缘/无效像素甚敏感 |

### 范围与边界（对应 `DEVELOPMENT_PLAN.md` §1.2 / `IMPLEMENTATION_TASKS.md` B 段）

- **不接入采集主流程**：本 header 里函数仅靠 `DynalgoFrame` / `DynalgoIntrinsic` /
  `DynalgoDetectionResult` 的值类型，不被
  `app/dynamic_algo_cam.cpp` 或任何 `FrameConsumer` 调用；它是为 Phase C "感知-估计-控制"
  闭环编  `DynalgoTrackBundle` 准备的反投影工具。
- **不做多帧 3D 时序滤波**：本函数只做单帧瞬时反投影；XYZ 时序滤波（现 `DynalgoKalmanTracker`
  只滤波 2D bbox）列为 `IMPLEMENTATION_TASKS.md` 的范围外事项 O6。
- **不做"最近点"/"mask 重心"采样**：现 `DynalgoDetectionResult` 只是 AABB；函数按用户上轮
  确认"框几何中心 + 小邻域中值"方案实现。

### 已验证（B2/B3 本 phase 内）

- **孤立头文件编译**：`g++ -std=c++14 -Iapp/core -fsyntax-only dynalgo_detection_to_3d.hpp` 通过
  （除"#pragma once in main file"提示无害）；
- **独立验证驱动**（替代 GTest，因本环境未装 GTest）
  `/tmp/opencode/phaseB_smoke.cpp` 5 用例全过：
  - (a) 中心 detection + 全 1000 mm 深度 → `(0, 0, 1.0)` m；
  - (b) 偏移 detection + 全 2000 mm 深度 → 与闭式预测
    `X=240/700, Y=0, Z=2.0` 欧氏误差 `0.00e+00 < 1e-4`；
  - (c) 全零深度 → `false`，out-params 原样保留；
  - (d) 错格式 (RGB 非 Y16) → `false`；
  - (e) `filterHalf=1` 中值窗 + 500 mm 深度 → `(0, 0, 0.5) m`。
  最终输出 `B-SMOKE-OK`，退出 0。
- **GTest 测试源**：`tests/detection_to_3d_test.cpp` 已加入 `tests/CMakeLists.txt`；在装了 GTest
  的环境通过 `cmake -DBUILD_TESTS=ON && cmake --build build && ctest -R detection_to_3d` 可跑
  5 个 `TEST()`（对应上述 a–e 用例）。本环境 GTest 缺失，根 `CMakeLists.txt`
  第 107-116 行的 `BUILD_TESTS=OFF + find_package(GTest QUIET)` 控制按预定方式跳过测试子目录。
- **默认路径无破坏**：`cmake --build build --target dynamic_algo_cam` 通过（headr 引入不破坏任何现存 target）。
