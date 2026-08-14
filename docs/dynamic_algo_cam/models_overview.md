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
