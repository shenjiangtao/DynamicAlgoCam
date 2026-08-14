# DynamicAlgoCam 实施任务清单

**配套设计文档**：[`DEVELOPMENT_PLAN.md`](DEVELOPMENT_PLAN.md)
**实施追踪基线**：commit `85a193c` (main)
**勾选约定**：`- [ ]` 未开始 / `- [~]` 进行中 / `- [x]` 已完成并验证 / `- [-]` 已放弃或并入他项
**最后更新**：2026-08-14

---

## 全局原则

- 每一步对应一次或一组可独立提交的 commit；commit message 沿用现有 `feat:` / `refactor:` / `docs:` 前缀
- 每一步必须先写**验证准则**最终通过后才能勾选 `- [x]`，依 karpathy "Goal-Driven Execution"
- 不允许推测性膨胀：每条新增文件 / 改动行必须可追溯到本清单中某一项
- 现有 `app/core/`、`app/capture/`、`app/driver/` 现存文件**零改动**，除非本清单显式列出
- `dynamic_algo_cam.cpp` 默认行为在 `--engage-*` flag 未传时与 commit `85a193c` 字节级一致

---

## Phase A — 执行器抽象层 + DUMMY 后端

**目标**：为外部激光/云台硬件留出 SDK-neutral 接口契约，沿用 `DynalgoModelBackend` 同款 abstract + factory + self-register 模式。
**预计代码量**：4 个新文件 + 3 处 CMakeLists 修改。
**对应设计文档**：DEVELOPMENT_PLAN.md §3.1, §4 Phase A, §6.1。

### A0. 接口契约冻结（写代码前）
- [x] A0.1 复核 `DEVELOPMENT_PLAN.md` §6.1 `DynalgoActuator` 抽象类签名，由 owner 确认无异议
  - 验证：以书面/issue 形式记录 owner 签字或回复确认
- [x] A0.2 确认 `DynalgoActuatorType` 枚举集合：`NONE / DUMMY / LASER_GENERIC / GIMBAL_GENERIC`（真实设备后两类占位，本 phase 不实现）
  - 验证：枚举列表与契约一致
- [x] A0.3 确认 `DynalgoActuatorConfig` POD 字段：`devicePath / protocolHint / dryRun(=true) / baudRate`
  - 验证：默认 `dryRun=true` 安全默认被明确接受

### A1. 抽象头文件
- [x] A1.1 新增 `app/core/dynalgo_actuator.hpp`：定义 `DynalgoActuatorType` enum、`DynalgoActuatorConfig` struct、`DynalgoActuator` abstract class
  - 验证：头文件仅 `#include "dynalgo_types.hpp"` + STL，不依赖任何 vendor SDK
  - 验证：`grep -n 'ob::\\|rs::\\|libobsensor' app/core/dynalgo_actuator.hpp` 无命中
  - 对应代码：`app/core/dynalgo_actuator.hpp`（新增）

### A2. 工厂 + 自注册表
- [x] A2.1 新增 `app/core/dynalgo_actuator_factory.hpp`：声明 `createActuator()` 与 `registerActuator()` 同款接口
  - 验证：与 `app/core/dynalgo_model_factory.hpp` 同款签名风格
  - 验证：含 `--whole-archive` 静态库链接注意事项注释（自注册模式固有问题）
  - 对应代码：`app/core/dynalgo_actuator_factory.hpp`（新增）
- [x] A2.2 新增 `app/core/dynalgo_actuator_factory.cpp`：mutex + `unordered_map<DynalgoActuatorType, ActuatorCreator>` 实现，与 `dynalgo_model_factory.cpp:1-54` 字节级同款模式
  - 验证：`DynalgoActuatorType::NONE` 输入返回 `nullptr`；重复注册 `DYNALGO_LOG_WARN_S`
  - 对应代码：`app/core/dynalgo_actuator_factory.cpp`（新增）
- [x] A2.3 修改 `app/core/CMakeLists.txt`：将 `dynalgo_actuator_factory.cpp` 加入 `dynalgo_core` 静态库源列表
  - 验证：`cmake --build build --target dynalgo_core` 通过
  - 验证：`nm build/lib/libnio_core.a | grep createActuator` 有符号命中（已确认：`_ZN3nio14createActuatorENS_15NioActuatorTypeE` 与 `registerActuator` 均存在）

### A3. DUMMY 后端示范
- [x] A3.1 新增 `app/actuator/dummy_actuator.hpp` + `dummy_actuator.cpp`：实现 `DummyActuator : DynalgoActuator`
  - 验证：`load()` 接受 cfg 返回 true；`open()` 打日志返回 true；`aimAt(x,y,z)` 打日志返回 true；`fire(durationMs)` 永远 NO-OP，仅 `DYNALGO_LOG_INFO_S`；`close()` 打日志返回 true；`name()` 返回 `"DUMMY"`
  - 验证：源文件中 `fire()` 实现体不含任何 `::write` / `::open` / 真实 syscall
  - 对应代码：`app/actuator/dummy_actuator.{hpp,cpp}`（新增）
- [x] A3.2 在 `dummy_actuator.cpp` 末尾放置匿名命名空间自注册静态块（`__attribute__((used))`），调用 `registerActuator(DynalgoActuatorType::DUMMY, ...)`；自注册体本身被编译入 `libnio_actuators.a`，但消费者必须 `--whole-archive` 拉入
  - 验证：链接入 `dynalgo_actuators` 后 `nm libnio_actuators.a` 可见 `DummyActuator::fire/open/aimAt/close/name` 与 `registerActuator` 引用符号
- [x] A3.3 新增 `app/actuator/CMakeLists.txt`：生成静态库 `dynalgo_actuators`，PUBLIC 链接 `dynalgo::core`，DAlias `dynalgo::actuators`
  - 验证：`cmake --build build --target dynalgo_actuators` 通过
- [x] A3.4 修改根 `app/CMakeLists.txt`：在 `add_subdirectory(capture)` 之后追加 `add_subdirectory(actuator)`，同步更新顶部架构图注释
  - 验证：`cmake --build build` 全量构建无 CMake 错误（增量 + 重建均通过）

### A4. 冒烟测试
- [x] A4.1 临时 ad-hoc 程序实例化 `DummyActuator` 并调用 `aimAt(0.5,-0.2,2)→fire(15)`，编译命令使用 `-Wl,--whole-archive ./libnio_actuators.a -Wl,--no-whole-archive`
  - 验证：日志五行可见 `load → open → aimAt → fire → close`，`dryRun=on` 标志正确出现在每行；`name()="DUMMY"`；最终 `SMOKE-OK`
  - 备注：此驱动不入主分支 commit；临时于 `/tmp/opencode/phaseA_smoke.cpp`
- [x] A4.2 全量 `cmake --build build` 验证
  - 验证：增量构建只有新目标与执行文件 relink，旧 target 字节级无重新编译
- [x] A4.3 `./build/bin/dynamic_algo_cam --help` 验证 (相对于 commit `85a193c`) 不引入新运行时问题
  - 验证：失败原因为预先存在的 Orbbec 运行时库路径问题 (`libOrbbecSDK.so.2` not found)，与 Phase A 无关——已用 `git stash` 在基线版本上重现相同错误
  - 验证：动态符号 `nm build/bin/dynamic_algo_cam | grep actuator` 命中 0（证明 Phase A 未泄漏至 default 链接路径，行为字节级与现有版本一致）

### A5. **静态自注册链注意事项**（写入 Phase C 链路必备）
- [x] A5.1 `dynalgo_actuator_factory.hpp` 顶部已加说明注释：消费端链接 `libnio_actuators.a` 时必须使用 `-Wl,--whole-archive ... -Wl,--no-whole-archive`，否则 registrar TU 被链接器丢弃 → `createActuator(DUMMY)` 返回 `nullptr`
  - 验证：注释中 `target_link_libraries(<consumer> PRIVATE "-Wl,--whole-archive" dynalgo::actuators "-Wl,--no-whole-archive")` 模板可见
  - **此约束将直接影响 Phase C4.2 主可执行文件接线**

### A6. 待提交清单
- [x] A6.1 `git add -A` 一次包含以下新增 / 修改文件：
  - `app/core/dynalgo_actuator.hpp`（新）
  - `app/core/dynalgo_actuator_factory.hpp`（新）
  - `app/core/dynalgo_actuator_factory.cpp`（新）
  - `app/core/CMakeLists.txt`（修改，3 行追加）
  - `app/actuator/dummy_actuator.hpp`（新）
  - `app/actuator/dummy_actuator.cpp`（新）
  - `app/actuator/CMakeLists.txt`（新）
  - `app/CMakeLists.txt`（修改，1 行 `add_subdirectory(actuator)` + 架构图注释更新）
  - `docs/dynamic_algo_cam/IMPLEMENTATION_TASKS.md`（本清单本 phase 与 A5/A6 勾选）
  - `docs/dynamic_algo_cam/DEVELOPMENT_PLAN.md`（§5 风险表追加 `--whole-archive` 静态库链接约束）
- [x] A6.2 `git commit` message 沿用 `feat: add SDK-neutral actuator abstraction layer + DUMMY backend in dynalgo_core/app/actuator`
- [x] A6.3 `git push origin main`
- [ ] A6.4 commit hash `ae23b1d` 已回填至 §"实施记录区" 与 §"验收里程碑 M1"

---

## Phase B — 2D 检测框 → 3D 相机光心反投影

**目标**：header-only 工具函数，输入 D2C 对齐深度帧 + intrinsics + depthScale + detection，输出光心坐标系 3D 米坐标。不接主流程。
**预计代码量**：1 个 header + 1 个单元测试 + 文档同步。
**对应设计文档**：DEVELOPMENT_PLAN.md §4 Phase B, §6.2。
**前置依赖**：可与 Phase A 并行；不依赖 A 的任何产物。

### B0. 公式再次确认
- [ ] B0.1 复核反投影公式：`u = det.x + det.w/2`，`v = det.y + det.h/2`，`Z = rawDepthAt(u,v) * depthScale`，`X = (u - intr.cx) * Z / intr.fx`，`Y = (v - intr.cy) * Z / intr.fy`
  - 验证：与 `app/capture/dynalgo_frame_consumer.cpp:139-176` 现成 `PointcloudFrameConsumer` 反投影路径一致（同公式）

### B1. header-only 工具
- [ ] B1.1 新增 `app/core/dynalgo_detection_to_3d.hpp`：声明 `detectionCenterToCamera3D(...)` 函数，含 PRECONDITION 注释块
  - 验证：注释明示 "depth must be D2C-aligned to color frame"
  - 验证：注释明示 `filterHalf=0`/`=2` 含义
  - 验证：函数对深度为 0 / 越界 / NaN 返回 false，输出 X/Y/Z 不变更
  - 对应代码：`app/core/dynalgo_detection_to_3d.hpp`（新增）
- [ ] B1.2 修改 `app/core/CMakeLists.txt`：仅将 header 加入 install 列表，不加入源（header-only）
  - 验证：`grep -c 'dynalgo_detection_to_3d.cpp' app/core/CMakeLists.txt` = 0
- [ ] B1.3 实现 `detectionCenterToCamera3D` inline 主体：取 `filterHalf > 0` 时对 `(2*filterHalf+1)^2` 窗内非零深度取中值；`filterHalf == 0` 时取单像素
  - 验证：单元测试 (B2.1) 的两个用例通过

### B2. 单元测试
- [ ] B2.1 新增 `tests/detection_to_3d_test.cpp`：测试用例 (a) 中心 detection + 全 1000mm 深度 → 反投影 `(0,0,1.0)`；(b) 偏离中心 detection + 全 2000mm 深度 → 反投影值距欧式距离与公式预测误差 `< 1e-4`
  - 验证：GTest 可用时 `ctest --test-dir build -R detection_to_3d` 通过；GTest 不可用时该项被 CMake `skip`（沿用现有 `tests/CMakeLists.txt` 处理 `event_window_test.cpp` 的模式）
  - 对应代码：`tests/detection_to_3d_test.cpp`（新增）
- [ ] B2.2 修改 `tests/CMakeLists.txt` 把新测试源加入；保留现有 GTest 可用性检测的"缺则跳过"逻辑
  - 验证：无 GTest 环境 `cmake ..` 无 FATAL_ERROR

### B3. 文档同步
- [ ] B3.1 在 `docs/dynamic_algo_cam/models_overview.md` 末尾追加 "2D 检测框 → 3D 相机光心坐标" 小节，含函数签名 + PRECONDITION + 推荐参数
  - 验证：md 内容与 header 注释一致；引用路径与仓库实际一致

---

## Phase C — 感知-估计-控制编排闭环（示范 dry-run）

**目标**：把 `DynalgoModelBackend` + `DynalgoDetectionResult` + `DynalgoKalmanTracker` + Phase B 反投影 + Phase A actuator 串成最小可运行闭环。仅当 `--engage-*` flag 传入时启动。
**预计代码量**：4 个新文件 + 1 处 main CLI 接线 + 文档。
**对应设计文档**：DEVELOPMENT_PLAN.md §4 Phase C, §6.3 状态机图。
**前置依赖**：依赖 A2 (`DynalgoActuator`) 与 B1 (`detectionCenterToCamera3D`)。

### C0. 状态机契约再次冻结
- [ ] C0.1 复核状态机 `IDLE → LOCKING → TRACKING → FIRING → LOST（折回 IDLE）` 各态触发条件、冷却时长、丢目标回退阈值，写入设计文档 §6.3 表格
  - 验证：owner 签字确认或在 issue 中记录默认值（建议 LOCKING→TRACKING 需连续 3 帧稳定 detection；TRACKING→LOST 容忍 N=5 帧无 detection）
- [ ] C0.2 确认 `EngagementFrameConsumer` 通过 `FrameConsumer` 接口插入 `session` 的 `frameConsumers_` 链尾 — 复用 `app/capture/dynalgo_capture_session.cpp:157-165` 模式
  - 验证：`CaptureSession::setup/start/startVideoPipeline/stop` 签名零修改

### C1. TargetSelector
- [ ] C1.1 新增 `app/algo/dynalgo_target_selector.hpp`：`SelectorStrategy` enum（`HIGHEST_SCORE / NEAREST_DEPTH / LARGEST_AREA`）；`DynalgoTargetSelector::pick(detections, depthAligned=nullptr)` 返回 `std::optional<DynalgoDetectionResult>`
  - 验证：header 仅 `#include "dynalgo_model.hpp" + <optional> + <vector>`
  - 对应代码：`app/algo/dynalgo_target_selector.hpp`（新增）
- [ ] C1.2 实现 `app/algo/dynalgo_target_selector.cpp`
  - 验证：单元测试 (C5.1) 通过
- [ ] C1.3 新增 `tests/target_selector_test.cpp`：4 个 detections 各按策略选对
  - 验证：通过（或 GTest 缺则 skip）

### C2. TrackBundle
- [ ] C2.1 新增 `app/algo/dynalgo_track_bundle.hpp`：`DynalgoTrackBundle` 持有 `DynalgoKalmanTracker` 一个，加 `init(detection)` / `update(detection, depthAligned, intr, scale)` / `predict()` / `getLast3D()` 接口
  - 验证：`update()` 内部先调 `detectionCenterToCamera3D` 缓存 3D；再调 `DynalgoKalmanTracker::update` 做 2D 平滑
  - 对应代码：`app/algo/dynalgo_track_bundle.{hpp,cpp}`（新增）
- [ ] C2.2 实现 `app/algo/dynalgo_track_bundle.cpp`；构造时 `DynalgoKalmanTracker` 默认参数；不允许 `init()` 重复调用（WARN 并重置）
  - 验证：编入 `dynalgo_algo` 静态库后 `nm | grep DynalgoTrackBundle::update` 有符号

### C3. EngagementLoop 状态机
- [ ] C3.1 新增 `app/algo/dynalgo_engagement_loop.hpp`：`DynalgoEngagementLoop` 类，构造入参 `modelBackend / actuator / selector / trackBundle`；公开 `onFrame(frameSet)` 回调与 `stop()`
  - 验证：依赖 `DynalgoModelBackend` + `DynalgoActuator` + `DynalgoTargetSelector` 抽象指针，不依赖任何 vendor SDK
  - 对应代码：`app/algo/dynalgo_engagement_loop.{hpp,cpp}`（新增）
- [ ] C3.2 实现 `app/algo/dynalgo_engagement_loop.cpp`：状态机 + 每态 `DYNALGO_LOG_INFO_S` trace；`FIRING` 态调用 `actuator->aimAt(X,Y,Z) → fire(durationMs)`；DUMMY actuator 与默认 `dryRun=true` 保证零外部副作用
  - 验证：状态切换均通过 `DYNALGO_LOG_INFO_S` 输出可被日志检索
  - 验证：`aimAt` 入参坐标系为光心 XYZ 米（直接用 TrackBundle `getLast3D()`）
- [ ] C3.3 新增 `app/algo/CMakeLists.txt`：生成静态库 `dynalgo_algo`，链接 `dynalgo_core` + `dynalgo_actuators`（不直接链接 OrbbecSDK / rs_driver）
  - 验证：`cmake --build build --target dynalgo_algo` 通过
- [ ] C3.4 修改根 `app/CMakeLists.txt`：在 `add_subdirectory(actuator)` 之后加 `add_subdirectory(algo)`
  - 验证：全量构建无错

### C4. EngagementFrameConsumer 接线
- [ ] C4.1 新增 `app/algo/dynalgo_engagement_consumer.hpp/.cpp`：`DynalgoEngagementFrameConsumer : FrameConsumer`，`consume(frameSet)` 调 `loop_->onFrame(frameSet)`
  - 验证：基类 `FrameConsumer` 接口完整实现（参考 `app/capture/dynalgo_frame_consumer.hpp:126-195` 中 PointcloudFrameConsumer 同款派生）
- [ ] C4.2 修改 `app/dynamic_algo_cam/dynamic_algo_cam.cpp`：增加 CLI option `--engage-model <type>` 与 `--engage-actuator <type>`；若两者均提供则通过 `createModelBackend` + `createActuator` 实例化，构造 EngagementLoop + EngagementFrameConsumer，**注入到对应 session 的 frameConsumers_ 链尾**
  - 验证：未传 flag 时 `frameConsumers_` 链尾该 consumer 不存在；现有路径字节级不变
  - 验证：传 flag 但无具体后端时，打日志 WARN 且继续采集不崩
  - 对应代码：`app/dynamic_algo_cam/dynamic_algo_cam.cpp`（修改，仅 CLI 解析 + 条件构造链尾插入）

### C5. 端到端 dry-run 冒烟
- [ ] C5.1 启用 DUMMY model 注入若干假 detection 喂入 loop：本地 ad-hoc 验证 5 秒内日志序列可见 `IDLE → LOCKING → TRACKING → FIRING → IDLE`
  - 验证：`./build/dynamic_algo_cam --engage-model DUMMY --engage-actuator DUMMY --enable-event-sim --no-show` 启动后 30s 内日志含上述序列
- [ ] C5.2 DUMMY actuator 日志可见 `aimAt(1,2,3) → fire(10ms)` 调用形式
  - 验证：grep 日志含 `aimAt` 与 `fire` 行
- [ ] C5.3 Ctrl+C 优雅退出，无 hang、无 leak（valgrind --leak-check=full 可选）
  - 验证：进程响应 SIGINT 退出码 0

---

## Phase D — 文档与移植手册同步

**目标**：所有新增模块文档化，无虚构接口描述。
**对应设计文档**：DEVELOPMENT_PLAN.md §4 Phase D。
**前置依赖**：Phase A/B/C 全部完成。

### D1. README
- [ ] D1.1 修改 `README.md` 架构图 ASCII chart：在 `dynalgo_capture` 下方新增 `dynalgo_actuators` + `dynalgo_algo` 两层注明 "optional, --engage-* flag"
  - 验证：图表与实际目录结构一致
- [ ] D1.2 修改 `README.md` Build section：增 `--engage-*` CLI 例子
  - 验证：命令实际可执行

### D2. 新增 engagement 文档
- [ ] D2.1 新增 `docs/dynamic_algo_cam/engagement_loop.md`：状态机图（mermaid 或 ASCII）+ 接入方式 + 安全注意事项 + dryRun 默认 + 反投影 precondition
  - 验证：内容与 C3 实现完全一致；不写未实现 actuator 的行为
  - 对应代码：`docs/dynamic_algo_cam/engagement_loop.md`（新增）

### D3. 移植手册
- [ ] D3.1 修改 `docs/dynamic_algo_cam/VENDOR_DEVICE_PORTING_MANUAL_CN.md` 末尾追加"如何适配新执行器"章节：与"如何适配新设备" / "如何适配新模型"章节同款体例
  - 验证：包含 DynalgoActuator 子类实现 checklist + registerActuator 自注册 walkthrough + dryRun 安全默认强调
- [ ] D3.2 修改 `docs/dynamic_algo_cam/VENDOR_DEVICE_PORTING_MANUAL.md`（英文版）保持与中文同款同步
  - 验证：英文章节号与中文对应

### D4. 模型子项目文档
- [ ] D4.1 修改 `docs/dynamic_algo_cam/models_overview.md`：在 Phase B 添加的小节后追加"TrackBundle 与 EngagementLoop"段
  - 验证：内容与 C2/C3 实现一致
- [ ] D4.2 修改 `app/models/README.md`：在 "Currently vendored" 表下方追加提示 — DUMMY model 后端用于工程 dry-run 测试，与 `app/actuator/dummy_actuator.cpp` 配对
  - 验证：表述不暗示生产可用

### D5. 最终全量验证
- [ ] D5.1 全量 `cmake --build build`（含 all targets）+ `ctest --test-dir build` 若可用
  - 验证：零警告错误（除现有警告）
- [ ] D5.2 `./build/dynamic_algo_cam --help` 默认行为与 commit `85a193c` 字节级一致（仅新增 `--engage-*` options）
  - 验证：`diff <(--help 旧) <(--help 新)` 仅含新行
- [ ] D5.3 `git log --oneline 85a193c..HEAD` 每条 commit 均可对应到本清单中某一项 task id
  - 验证：无脱节 commit

---

## 范围外事项追踪（不在本清单执行，仅登记备查）

| 编号 | 事项 | 立项条件 | 登记日 |
|---|---|---|---|
| O1 | 真实激光器协议适配 (`LASER_GENERIC`) | 决定激光器型号 + 通信协议后 | 2026-08-14 |
| O2 | 真实云台协议适配 (`GIMBAL_GENERIC`) | 决定云台型号 + 运动学后 | 2026-08-14 |
| O3 | 安全联锁 / 急停硬件 + 软件安全监督进程 | 进入真实硬件测试前 | 2026-08-14 |
| O4 | 生产级 YOLOv8 ONNX/TensorRT C++ backend | 训练出真实场景权重 | 2026-08-14 |
| O5 | 多目标 tracker（Hungarian + 轨迹生命周期） | 单目标闭环完成且性能验证通过后 | 2026-08-14 |
| O6 | 3D 坐标 KF 时序滤波（XYZ 而非 bbox） | 现有 `DynalgoKalmanTracker` 单目标验证有效后 | 2026-08-14 |
| O7 | 操作员 HMI / Web 控制台 | 现场试用阶段 | 2026-08-14 |
| O8 | `DynalgoModelType::YOLOV8_PY` 真后端（Python subprocess/IPC 桥） | 决定 Python ↔ C++ 进程边界后 | 2026-08-14 |

---

## 验收里程碑

- [x] **M1** Phase A 完成 — commit hash: `ae23b1d` — 验证 `nm build/lib/libnio_core.a | grep createActuator` 命中（**注意**：重命名工程后此命令演变为 `nm build/lib/libdynalgo_core.a | grep createActuator`；详见 §M6 重命名记录）
- [ ] **M2** Phase B 完成 — commit hash: `________` — 验证 GTest skip 或通过，header install 列表更新
- [ ] **M3** Phase C 完成 — commit hash: `________` — 验证 `--engage-model DUMMY --engage-actuator DUMMY` 启动后日志可见状态机序列
- [ ] **M4** Phase D 完成 — commit hash: `________` — 验证全量构建 + `--help` diff
- [ ] **M5** 全工程验证：默认行为与 `85a193c` 一致 — 验证 commit 链清晰对应清单项
- [x] **M6** 工程命名重构 `nio::` → `dynalgo::` 完成 — commit hash: `25be197` — 验证见下方"T1 工程命名重构"段

---

## 实施记录区（提交后回填，不预先填写）

| Task ID | Commit | 完成日 | 备注 |
|---|---|---|---|
| A0-A5 | `ae23b1d` | 2026-08-14 | Phase A 实现 + 验证完成；commit `ae23b1d` 已 push 至 origin/main（`85a193c..ae23b1d`） |
| T1 工程命名重构 | `25be197` | 2026-08-14 | 见下方"T1 工程命名重构"段；commit `25be197` 已 push 至 origin/main（`532f7ad..25be197`） |

---

## T1 工程命名重构（`nio::` → `dynalgo::`）

**背景**：用户指示按"工程所在的仓库 + 工程的目标"重命名 C++ 命名空间，不再使用 `nio`。新名经用户确认：namespace = `dynalgo`，类前缀 = `Dynalgo*`（PascalCase），宏前缀 = `DYNALGO_*`，文件名 = `dynalgo_*.hpp/.cpp`，版权头 = `Copyright (c) shenjiangtao. All Rights Reserved.`。

### 重命名映射表

| 类别 | 旧 | 新 |
|---|---|---|
| C++ namespace | `nio` | `dynalgo` |
| 类前缀（PascalCase） | `NioDevice`、`NioPipeline`、`NioFrame`、`NioKalmanTracker`、`NioActuator`、`NioModel*`、`NioDetectionResult` 等 32 个 | `Dynalgo*` |
| 宏前缀 | `NIO_LOG_INFO_S` 等 16 个 | `DYNALGO_LOG_*` |
| 宏符号 | `NIO_FILE_BUF_SIZE`、`NIO_PROJECT_ROOT_DIR`、`NIO_RELEASE_FLAGS`、`NIO_DEVICE_VID` | `DYNALGO_*` |
| 文件名 | `nio_*.hpp/.cpp`（54 个文件） | `dynalgo_*` |
| CMake target | `nio_core`/`nio_drivers`/`nio_capture`/`nio_actuators`/`nio_opencv_plugin` | `dynalgo_*` |
| CMake ALIAS | `nio::core`/`nio::drivers`/`nio::capture`/`nio::actuators`/`nio::opencv_plugin` | `dynalgo::*` |
| C 函数（`utils_c.h/.c`） | `nio_wait_for_key_press`、`nio_support_ansi_escape`、`nio_get_current_timestamp_ms` | `dynalgo_*` |
| 版权头 | `Copyright (c) NIO Inc. All Rights Reserved.` | `Copyright (c) shenjiangtao. All Rights Reserved.` |

### **众保护：以下文件格式的 magic strings 保留不变**

下列字符串是写入磁盘文件头部的字节序列，由配套解析脚本（`app/tools/parse_*.py`）读取，**若改动则旧文件无法解析**，故此 T1 重构刻意保留其字面值：

| Magic | 文件类型 | 引用位置 |
|---|---|---|
| `NIO_DEPTH_RAW` | 深度原始 `.raw` 文件头 magic | `app/capture/dynalgo_stream_io.cpp:91` |
| `NIO_PCD_STREAM` | 点云流 `.pcs` 文件头 magic | `app/capture/dynalgo_stream_io.cpp:266` |
| `NIO_POINT_CLOUD_RAW` | RS-AC1 `*.point_raw_*.raw` 文件头 magic | 仅文档里描述（`docs/dynamic_algo_cam/dynamic_algo_cam_technical_reference.md`、`use_guide.md`），代码尚未实现；T1 不动以保证文档与未来实现一致 |

历史磁盘路径中的 `nio` 子串（如 `docs/AC1_vs_335L_336L_TECH.md` 中的 `/tmp/nio_analysis_*`）也保留 — 是历史指代、非代码符号。

### T1 验证清单

- [x] T1.0 已用 `grep -rn` 验证：`\\bnio::` / `Nio[A-Z]*` / `namespace nio` / `\\bNIO_LOG_*` 在 `app/`、`tests/`、`docs/dynamic_algo_cam/`、根 `CMakeLists.txt`、`README.md` 内**零命中**（排除上述 protected magic 与 `/tmp/nio_*` 历史路径）
- [x] T1.1 **文件重命名**：`git mv` 54 个 `nio_*.{hpp,cpp,h,c}` → `dynalgo_*.{hpp,cpp,h,c}`；`git status` 全部为 `R` (rename) 状态，保留 git 历史追踪
- [x] T1.2 **批量字面量替换**：72 文件内容替换（namespace / 类名 / 宏 / `#include` / CMake 源列表）
- [x] T1.3 **版权头批量替换**：68 个源文件顶部 `NIO Inc.` → `shenjiangtao`
- [x] T1.4 **文档同步**：`docs/dynamic_algo_cam/` 全部 8 篇文档、`README.md`、根 `CMakeLists.txt` 中的代码符号引用同步更新；porting 教程中的 VENDOR 模板示例文件名 `nio_xyz_*` → `dynalgo_xyz_*` 同步更新
- [x] T1.5 **CMake 重新配置 + 全量构建**：`rm -rf build && cmake -S . -B build && cmake --build build -j` 通过；4 个静态库均按新名输出：`build/lib/libdynalgo_core.a` / `libdynalgo_drivers.a` / `libdynalgo_capture.a` / `libdynalgo_actuators.a`；可执行文件 `build/bin/dynamic_algo_cam` 链接成功
- [x] T1.6 **符号验证**：`nm build/lib/libdynalgo_core.a | grep -c "dynalgo::\\|Dynalgo"` 命中 20，`grep -c "nio::\\|Nio[A-Z]"` 命中 0 — 命名重构在 binary 符号层完全无残留
- [x] T1.7 **端到端冒烟**：重写的 `/tmp/opencode/phaseA_smoke.cpp`（连入 `dynalgo_actuator.hpp`、`dynalgo_actuator_factory.hpp`、`dynalgo_log.hpp`）通过 `-Wl,--whole-archive libdynalgo_actuators.a -Wl,--no-whole-archive` 链接 `libdynalgo_core.a`，运行后 `load → open → aimAt → fire → close` 5 行日志可见，`name()=DUMMY`，`SMOKE-OK`，退出 0
- [x] T1.8 **protected magic 完整性**：`dynalgo_stream_io.{hpp,cpp}` 内 `NIO_DEPTH_RAW` / `NIO_PCD_STREAM` 仍为原始字面字符串（保留旧文件兼容性）
- [ ] T1.9 **commit & push**：commit `25be197` 已 push；待 hash 回填
- [x] T1.10 提交后回填 commit hash 到 §"实施记录区" 与本段 §M6（hash: `25be197`）

---

## 引用仓库事实（用于审阅者复验）

本清单中提及的现存代码事实，均可在以下路径直接核对：

- 抽象类 + 自注册工厂模式先例：`app/core/dynalgo_model_factory.cpp:1-54`
- `DynalgoDetectionResult` 字段定义：`app/core/dynalgo_model.hpp:45-53`
- `DynalgoIntrinsic` 字段定义：`app/core/dynalgo_types.hpp:257-265`
- `depthScale` 缓存路径：`app/capture/dynalgo_capture_session.cpp:41`
- D2C 自动判定路径：`app/driver/orbbec/dynalgo_ob_device.cpp:281-286`
- `FrameConsumer` 链尾插入先例：`app/capture/dynalgo_capture_session.cpp:157-165`（PCD consumer）
- 反投影公式已有先例：`app/capture/dynalgo_frame_consumer.cpp:139-176`
- `--enable-event-sim` CLI flag 先例：根 `CMakeLists.txt`
- `FrameConsumer` 基类派生先例（pointcloud）：`app/capture/dynalgo_frame_consumer.hpp:126-195`
