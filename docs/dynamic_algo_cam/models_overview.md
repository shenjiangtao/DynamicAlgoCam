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
现有 `NioDevice` / `NioPipeline` 同级，**不引入任何推理 SDK 依赖**：

| 文件 | 作用 |
|------|------|
| `app/core/nio_model.hpp` | `NioModelType` 枚举、`NioDetectionResult` 结构、`NioModelBackend` 抽象类、`NioModelConfig` |
| `app/core/nio_model_factory.hpp` | `createModelBackend(type)` 工厂入口 + `registerModelBackend` 注册钩子 |
| `app/core/nio_model_factory.cpp` | 工厂实现 + 进程级注册表（mutex 保护），已编入 `libnio_core.a` |

### 接口契约

```cpp
enum class NioModelType { NONE, DUMMY, YOLOV8_PY, ONNXRUNTIME, TENSORRT };

struct NioDetectionResult {
    int   classId; float score;
    float x, y, w, h;     // 源帧坐标系下的 bounding box (像素)
    std::string label;
};

struct NioModelConfig {
    std::string modelPath;    // weights/model file
    std::string deviceHint;   // "cpu"/"gpu"/"cuda:0"/...
    float confThreshold = 0.25f;
    float iouThreshold  = 0.45f;
};

class NioModelBackend {
    virtual bool   load(const NioModelConfig& cfg) = 0;
    virtual bool   infer(const NioFrame& frame,
                         std::vector<NioDetectionResult>& out) = 0;
    virtual const char* name() const = 0;
};

std::unique_ptr<NioModelBackend> createModelBackend(NioModelType type);
```

### 使用模式

```cpp
auto backend = nio::createModelBackend(nio::NioModelType::YOLOV8_PY);
if (!backend) { /* 该 backend 未编入 */ }
NioModelConfig cfg; cfg.modelPath = "yolov8n.pt";
if (backend->load(cfg)) {
    std::vector<NioDetectionResult> results;
    if (backend->infer(currentFrame, results)) { /* 消费 results */ }
}
```

### 当前状态与边界

- **本阶段只加抽象层，不接 YOLOv8**。`createModelBackend(YOLOV8_PY)` 在尚未注册任何
  backend 时返回 `nullptr`，调用方应态然处理。
- 后续具体 backend（YOLOv8 Python 桥接、ONNX Runtime、TensorRT、Dummy 测试实现）应
  放 `app/driver/<name>/` 或独立子目录，在各自 `.cpp` 内用 `static-init + 宏守卫` 调用
  `registerModelBackend` 自注册（与 `NioDriverFactory` 风格一致）。
- **采集主流程不直接依赖此抽象类**——这是专留给"算法线程"或"上层业务模块"的接口；
  PcdTask / 录文件 / SDL 预览均与此层无关。

### 已验证

- `cmake --build build --target dynamic_algo_cam` 通过；
- `nm build/lib/libnio_core.a | grep nio::` 显示 `createModelBackend` 与 `registerModelBackend`
  符号已编入静态库。
