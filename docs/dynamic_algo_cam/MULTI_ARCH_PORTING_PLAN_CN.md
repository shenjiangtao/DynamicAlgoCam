# DynamicAlgoCam 工业级多平台架构设计 / Industrial-Grade Multi-Platform Architecture

---

## 1. 架构愿景 / Architectural Vision

**目标**: 构建 **生产级、可安全认证** 的计算机视觉平台，遵循 **工业软件交付标准**（类 AUTOSAR 分层、Android HAL 模式、Linux V4L2/IIO 子系统）。

### 核心原则 / Core Principles

| 原则 | 说明 |
|---|---|
| **应用-硬件解耦** | App 层仅依赖抽象 HAL 接口；零硬件头文件依赖 |
| **HAL 即契约** | HAL = **稳定 C/C++ ABI**；厂商实现，应用消费 |
| **平台 → 厂商 → 板卡** | 3 级层级：平台 → 厂商 → 板卡 |
| **配置胜过代码** | 硬件带板通过 **配置文件** 而非代码修改 |
| **设计即安全** | 确定性内存、有界延迟、故障隔离 |
| **供应商独立** | 每平台多厂商；构建/部署时热插拔 |

---

## 2. 分层架构 / Layered Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           APPLICATION LAYER (App)                           │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │  Capture    │ │  Perception │ │  Planning   │ │  Control    │           │
│  │  Session    │ │  (Detection,│ │  (Tracking, │ │  (Actuator, │           │
│  │  Manager    │ │   Tracking) │ │   Fusion)   │ │   Safety)   │           │
│  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘ └──────┬──────┘           │
└─────────┼───────────────┼───────────────┼───────────────┼──────────────────┘
          │               │               │               │
          ▼               ▼               ▼               ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         HAL INTERFACE LAYER (Stable ABI)                    │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │ ICameraHAL  │ │ IEncoderHAL │ │ IInferHAL   │ │ IDisplayHAL │           │
│  │ IActuatorHAL│ │ ISensorHAL  │ │ ILoggerHAL  │ │ ITimeHAL    │           │
│  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘ └──────┬──────┘           │
└─────────┼───────────────┼───────────────┼───────────────┼──────────────────┘
          │               │               │               │
          ▼               ▼               ▼               ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      PLATFORM ADAPTATION LAYER (CS/HAL Impl)                │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    PLATFORM: x86_64 (Generic PC)                     │   │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐   │   │
│  │  │ Vendor:     │ │ Vendor:     │ │ Vendor:     │ │ Vendor:     │   │   │
│  │  │ uvc         │ │ orbbec      │ │ robosense   │ │ intel_realsense│  │   │
│  │  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘   │   │
│  ├─────────────────────────────────────────────────────────────────────┤   │
│  │                    PLATFORM: aarch64 (NVIDIA Jetson/Drive)           │   │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐   │   │
│  │  │ Vendor:     │ │ Vendor:     │ │ Vendor:     │ │ Vendor:     │   │   │
│  │  │ nvsipl      │ │ nvmedia     │ │ tensorrt    │ │ cudla       │   │   │
│  │  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘   │   │
│  ├─────────────────────────────────────────────────────────────────────┤   │
│  │                    PLATFORM: mips64 (Loongson)                       │   │
│  │  ┌─────────────┐ ┌─────────────┐                                    │   │
│  │  │ Vendor:     │ │ Vendor:     │                                    │   │
│  │  │ loongson_v4l2│ │ loongson_gpu│                                    │   │
│  │  └─────────────┘ └─────────────┘                                    │   │
│  ├─────────────────────────────────────────────────────────────────────┤   │
│  │                    PLATFORM: riscv64 (StarFive/T-Head)               │   │
│  │  ┌─────────────┐ ┌─────────────┐                                    │   │
│  │  │ Vendor:     │ │ Vendor:     │                                    │   │
│  │  │ starfive_v4l2│ │thead_npu    │                                    │   │
│  │  └─────────────┘ └─────────────┘                                    │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
          │               │               │               │
          ▼               ▼               ▼               ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         BOARD SUPPORT PACKAGE (BSP)                         │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │ Board Config│ │ Pinmux/     │ │ Power/      │ │ Calibration │           │
│  │ (YAML)      │ │ GPIO Maps   │ │ Thermal     │ │ Data        │           │
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘           │
└─────────────────────────────────────────────────────────────────────────────┘
          │               │               │               │
          ▼               ▼               ▼               ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            SILICON / FIRMWARE                               │
│         Sensors (MIPI CSI-2)  •  ISP/VPU  •  GPU/DLA/NPU  •  Actuators      │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. 目录结构 / Directory Structure

```
DynamicAlgoCam/
├── CMakeLists.txt                    # 顶层：平台检测、厂商选择
├── cmake/
│   ├── platforms/                    # 平台检测与默认值
│   │   ├── x86_64.cmake
│   │   ├── aarch64_nvidia.cmake
│   │   ├── mips64_loongson.cmake
│   │   └── riscv64_starfive.cmake
│   ├── vendors/                      # 厂商专用 find 模块
│   │   ├── FindNvSIPL.cmake
│   │   ├── FindNvMedia.cmake
│   │   ├── FindOrbbecSDK.cmake
│   │   ├── FindRoboSense.cmake
│   │   └── FindLoongsonGPU.cmake
│   └── toolchains/                   # 交叉编译工具链
│       ├── jetson-aarch64.cmake
│       ├── loongson-mips64.cmake
│       └── starfive-riscv64.cmake
│
├── include/                          # PUBLIC HAL HEADERS (稳定 ABI)
│   └── dynalgo/
│       ├── hal/
│       │   ├── camera_hal.hpp        # ICameraHAL, CameraConfig, FrameSet
│       │   ├── encoder_hal.hpp       # IEncoderHAL, EncodeConfig, Bitstream
│       │   ├── inference_hal.hpp     # IInferenceHAL, ModelConfig, Tensor
│       │   ├── display_hal.hpp       # IDisplayHAL, DisplayConfig
│       │   ├── actuator_hal.hpp      # IActuatorHAL, ActuatorConfig
│       │   ├── sensor_hal.hpp        # ISensorHAL (IMU, GPS, 等)
│       │   ├── logger_hal.hpp        # ILoggerHAL
│       │   ├── time_hal.hpp          # ITimeHAL (单调时钟, PTP, GPS)
│       │   └── hal_types.hpp         # 通用 HAL 类型、错误码
│       ├── core/
│       │   ├── dynalgo_frame.hpp     # DynalgoFrame (HAL 无关)
│       │   ├── dynalgo_types.hpp     # 核心类型 (检测、bbox、3D)
│       │   └── dynalgo_errors.hpp    # 错误码 (HAL + App)
│       └── config/
│           ├── platform_config.hpp   # PlatformConfig (YAML schema)
│           └── vendor_config.hpp     # VendorConfig (厂商参数)
│
├── hal/                              # HAL 实现 (按平台/厂商)
│   ├── CMakeLists.txt                # 按 DYNALGO_PLATFORM 添加子目录
│   ├── common/                       # 共享 HAL 工具、基类
│   │   ├── hal_base.hpp
│   │   ├── buffer_manager.hpp        # NvSciBuf/V4L2/dma-buf 统一封装
│   │   ├── sync_manager.hpp          # NvSciSync/fence/pthread 统一封装
│   │   └── plugin_loader.hpp         # 动态库加载
│   │
│   ├── x86_64/                       # x86 平台
│   │   ├── CMakeLists.txt
│   │   ├── camera/
│   │   │   ├── uvc/                  # 厂商：通用 UVC
│   │   │   ├── orbbec/               # 厂商：Orbbec
│   │   │   ├── robosense/            # 厂商：RoboSense
│   │   │   ├── realsense/            # 厂商：Intel RealSense
│   │   │   └── stereo/               # 厂商：通用双目 UVC
│   │   ├── encoder/
│   │   │   ├── ffmpeg/               # 厂商：FFmpeg 软编
│   │   │   ├── vaapi/                # 厂商：Intel VAAPI 硬编
│   │   │   └── nvenc/                # 厂商：NVIDIA NVENC (x86 dGPU)
│   │   ├── inference/
│   │   │   ├── tensorrt/             # 厂商：TensorRT x86
│   │   │   ├── onnxruntime/          # 厂商：ONNX Runtime
│   │   │   └── openvino/             # 厂商：Intel OpenVINO
│   │   ├── display/
│   │   │   ├── sdl2/                 # 厂商：SDL2
│   │   │   ├── wayland/              # 厂商：Wayland/DRM
│   │   │   └── x11/                  # 厂商：X11
│   │   └── actuator/
│   │       ├── generic_serial/       # 厂商：串口/USB 执行器
│   │       └── can/                  # 厂商：CAN 总线执行器
│   │
│   ├── aarch64_nvidia/               # ARM 平台：NVIDIA Jetson/Drive
│   │   ├── CMakeLists.txt
│   │   ├── camera/
│   │   │   ├── nvsipl/               # 厂商：NVIDIA SIPL (主力)
│   │   │   │   ├── nvsipl_camera_hal.cpp
│   │   │   │   ├── nvsipl_camera_hal.hpp
│   │   │   │   ├── nvsipl_producer.cpp
│   │   │   │   ├── nvsipl_consumer_cuda.cpp
│   │   │   │   ├── nvsipl_consumer_enc.cpp
│   │   │   │   ├── nvsipl_consumer_display.cpp
│   │   │   │   ├── nvsipl_platform_cfg.cpp
│   │   │   │   └── CMakeLists.txt
│   │   │   ├── v4l2/                 # 厂商：通用 V4L2 回退
│   │   │   └── argus/                # 厂商：libargus (遗留)
│   │   ├── encoder/
│   │   │   ├── nvmedia/              # 厂商：NvMedia IEP (硬编 H.264/265)
│   │   │   └── nvenc/                # 厂商：NVENC (Jetson Orin)
│   │   ├── inference/
│   │   │   ├── tensorrt/             # 厂商：TensorRT + DLA
│   │   │   ├── cudla/                # 厂商：CUDLA 直连
│   │   │   └── triton/               # 厂商：Triton Inference Server
│   │   ├── display/
│   │   │   ├── nvdisplay/            # 厂商：NvDisplay/DRM
│   │   │   └── wfd/                  # 厂商：OpenWFD
│   │   └── actuator/
│   │       ├── gpio/                 # 厂商：GPIO PWM
│   │       └── can/                  # 厂商：CAN (via mttcan)
│   │
│   ├── mips64_loongson/              # MIPS 平台：龙芯
│   │   ├── CMakeLists.txt
│   │   ├── camera/
│   │   │   ├── v4l2/                 # 厂商：V4L2
│   │   │   └── loongson_isp/         # 厂商：龙芯 ISP
│   │   ├── encoder/
│   │   │   └── loongson_venc/        # 厂商：龙芯视频编码器
│   │   ├── inference/
│   │   │   └── loongson_npu/         # 厂商：龙芯 NPU
│   │   └── ...
│   │
│   └── riscv64_starfive/             # RISC-V 平台：StarFive/T-Head
│       ├── CMakeLists.txt
│       ├── camera/
│       │   ├── v4l2/
│       │   └── starfive_isp/
│       ├── encoder/
│       │   └── starfive_venc/
│       ├── inference/
│       │   └── thead_npu/
│       └── ...
│
├── app/                              # 应用层 (仅依赖 HAL)
│   ├── CMakeLists.txt
│   ├── core/
│   │   ├── dynalgo_capture_session.cpp    # 仅使用 ICameraHAL
│   │   ├── dynalgo_pipeline_manager.cpp   # 管线编排
│   │   ├── dynalgo_frame_manager.cpp      # 帧生命周期 (HAL 缓冲)
│   │   └── dynalgo_config_manager.cpp     # 平台/厂商/板卡配置
│   ├── algo/
│   │   ├── engagement/
│   │   │   ├── dynalgo_engagement_loop.cpp
│   │   │   ├── dynalgo_target_selector.cpp
│   │   │   └── dynalgo_track_bundle.cpp
│   │   ├── perception/
│   │   │   ├── detection_postprocess.cpp
│   │   │   ├── nms.cpp                   # 纯算法，无硬件依赖
│   │   │   └── tracker.cpp
│   │   └── fusion/
│   │       ├── sensor_fusion.cpp
│   │       └── calibration.cpp
│   ├── dynamic_algo_cam/
│   │   ├── dynamic_algo_cam.cpp          # 主入口，HAL 工厂初始化
│   │   └── cli_parser.cpp
│   └── tools/
│       ├── calibration_tool.cpp
│       └── benchmark_tool.cpp
│
├── bsp/                                # 板级支持包
│   ├── CMakeLists.txt
│   ├── configs/
│   │   ├── platforms/
│   │   │   ├── x86_64_generic.yaml
│   │   │   ├── aarch64_jetson_agx_orin.yaml
│   │   │   ├── aarch64_jetson_orin_nx.yaml
│   │   │   ├── aarch64_drive_orin.yaml
│   │   │   ├── mips64_loongson_3a5000.yaml
│   │   │   └── riscv64_starfive_jh7110.yaml
│   │   ├── vendors/
│   │   │   ├── nvsipl/
│   │   │   │   ├── imx728.yaml
│   │   │   │   ├── imx623.yaml
│   │   │   │   ├── ar0820.yaml
│   │   │   │   └── max96712.yaml
│   │   │   ├── orbbec/
│   │   │   │   ├── femto_bolt.yaml
│   │   │   │   └── gemini_335.yaml
│   │   │   └── robosense/
│   │   │       ├── rs_helios.yaml
│   │   │       └── rs_pearl.yaml
│   │   └── boards/
│   │       ├── nvidia_jetson_agx_orin_devkit.yaml
│   │       ├── nvidia_jetson_orin_nx_custom.yaml
│   │       ├── loongson_3a5000_devboard.yaml
│   │       └── starfive_visionfive2.yaml
│   ├── pinmux/
│   │   ├── jetson_orin_cam0_cam1.dtsi
│   │   └── jetson_orin_gpio_actuator.dtsi
│   ├── power/
│   │   ├── jetson_orin_power_profile.yaml
│   │   └── drive_orin_power_policy.yaml
│   └── calibration/
│       ├── imx728_front_center.yml
│       ├── imx623_rear_left.yml
│       └── stereo_calibration.yml
│
├── scripts/
│   ├── build.sh                        # 本地构建
│   ├── build_cross.sh                  # 交叉编译 (平台参数)
│   ├── flash_jetson.sh
│   ├── flash_loongson.sh
│   ├── run_tests.sh
│   └── package_release.sh
│
├── tests/
│   ├── unit/
│   │   ├── hal/
│   │   │   ├── test_camera_hal.cpp
│   │   │   ├── test_encoder_hal.cpp
│   │   │   └── test_inference_hal.cpp
│   │   ├── algo/
│   │   └── core/
│   ├── integration/
│   │   ├── test_capture_pipeline.cpp
│   │   ├── test_encode_pipeline.cpp
│   │   └── test_inference_pipeline.cpp
│   ├── performance/
│   │   ├── benchmark_latency.cpp
│   │   ├── benchmark_throughput.cpp
│   │   └── benchmark_memory.cpp
│   └── hardware/
│       ├── test_jetson_cameras.py
│       └── test_x86_cameras.py
│
├── docs/
│   ├── architecture/
│   │   ├── HAL_SPEC.md                 # HAL 接口规范
│   │   ├── PLATFORM_PORTING_GUIDE.md   # 新平台移植指南
│   │   ├── VENDOR_PORTING_GUIDE.md     # 新厂商移植指南
│   │   ├── BOARD_BRINGUP_GUIDE.md      # 板卡配置指南
│   │   └── SAFETY_CERTIFICATION_GUIDE.md
│   ├── dynamic_algo_cam/
│   │   ├── MULTI_ARCH_PORTING_PLAN.md  # 本文档
│   │   ├── MULTI_ARCH_PORTING_PLAN_CN.md
│   │   └── ...
│   └── api/
│       └── hal_api_reference.md
│
└── vendors/                            # 第三方 SDK (git submodules)
    ├── OrbbecSDK/
    ├── RoboSense/
    └── (NvSIPL/NvMedia 来自系统 SDK)
```

---

## 10. UPEP: 统一感知执行平台扩展 / UPEP: Unified Perception & Execution Platform Extensions

UPEP 扩展将 DynamicAlgoCam 从以相机为中心的平台转变为 **统一的多模态感知执行平台**，支持双目相机、激光雷达、传感器融合、动态算法加载和异构计算调度。

### 10.1 UPEP 架构概览 / UPEP Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        应用层                                               │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │  捕获会话   │ │  感知       │ │  规划       │ │  控制       │           │
│  │  Manager    │ │  (检测,     │ │  (跟踪,     │ │  (执行器,   │           │
│  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘ └──────┬──────┘           │
└─────────┼───────────────┼───────────────┼───────────────┼──────────────────┘
          │               │               │               │
          ▼               ▼               ▼               ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                    UPEP 扩展层 (新增)                                        │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │ 多模态融合  │ │ 异构计算    │ │ 插件管理    │ │ 任务调度    │           │
│  │  Fusion HAL │ │ Compute HAL │ │ Manager HAL │ │ Scheduler   │           │
│  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘ └──────┬──────┘           │
└─────────┼───────────────┼───────────────┼───────────────┼──────────────────┘
          │               │               │               │
          ▼               ▼               ▼               ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         HAL 接口层 (稳定 ABI)                               │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │ ICameraHAL  │ │ IEncoderHAL │ │ IInferHAL   │ │ IDisplayHAL │           │
│  │ IActuatorHAL│ │ ISensorHAL  │ │ ILoggerHAL  │ │ ITimeHAL    │           │
│  │ IFusionHAL  │ │ ISyncHAL    │ │ IComputeHAL │ │ IPluginHAL  │           │
│  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘ └──────┬──────┘           │
└─────────┼───────────────┼───────────────┼───────────────┼──────────────────┘
```

### 10.2 新增 HAL 接口 / New HAL Interfaces

| HAL 接口 | 用途 | 关键类型 |
|---|---|---|
| **IFusionEngine** | 视觉-激光雷达融合 (早/晚/深度融合) | `FusionConfig`, `FusedDetection`, `FusedPointCloud`, `SynchronizedFrameSet` |
| **ISyncEngine** | 多传感器时间同步 | `SyncConfig`, `SynchronizedFrameSet`, `SyncMethod` |
| **IOperatorRegistry** | 异构算子注册表 | `OperatorImplementation`, `Tensor`, `ComputeBackendType` |
| **IComputeContext** | 任务级计算资源隔离 | `ComputeResourceLimits`, `Tensor`, `MemoryType` |
| **IPlugin** | 动态算法插件接口 | `PluginManifest`, `PluginPort`, `PluginType` |
| **IPluginManager** | 插件生命周期管理 | `loadPlugin`, `unloadPlugin`, `hotSwapPlugin` |
| **ITaskScheduler** | 基于 DAG 的任务调度 | `TaskGraph`, `TaskNode`, `TaskEdge`, `TaskExecutionContext` |

### 10.3 统一帧缓冲 / Unified FrameBuffer

UPEP `FrameBuffer` 统一图像和点云数据，支持零拷贝语义：

```cpp
struct FrameBuffer {
    FrameMetadata metadata;           // 时间戳, frame_id, sensor_id, frame_type
    
    // 图像数据 (IMAGE, DEPTH_MAP)
    std::vector<uint8_t> image_data;
    bool image_owns_data = true;
    
    // 点云数据 (POINT_CLOUD)
    PointCloudLayout pc_layout;       // 字段: x,y,z,intensity,ring,timestamp
    std::vector<uint8_t> point_data;
    bool pc_owns_data = true;
    
    // 同步
    FenceHandle acquire_fence;
    FenceHandle release_fence;
    
    // 多传感器同步
    uint64_t sync_group_id = 0;
    uint64_t reference_timestamp_ns = 0;
    
    // 引用计数用于零拷贝共享
    std::atomic<uint32_t> ref_count{1};
};
```

### 10.4 多传感器同步 / Multi-Sensor Synchronization

`ISyncEngine` 提供硬件触发、PTP 和软件时间戳同步：

```cpp
struct SynchronizedFrameSet {
    uint64_t sync_timestamp_ns;       // 参考时间戳 (PTP 主时钟)
    uint64_t sync_group_id;
    
    FrameBufferPtr camera_frame;      // 主相机
    FrameBufferPtr depth_frame;       // 深度/双目
    FrameBufferPtr lidar_frame;       // 激光雷达点云
    FrameBufferPtr imu_frame;         // IMU 数据
    
    std::map<std::string, CalibrationData> calibrations;
    
    bool hasCamera() const { return camera_frame && camera_frame->isImage(); }
    bool hasDepth() const { return depth_frame && depth_frame->isImage(); }
    bool hasLidar() const { return lidar_frame && lidar_frame->isPointCloud(); }
    bool hasImu() const { return imu_frame && imu_frame->isImage(); }
};
```

### 10.5 异构计算抽象 / Heterogeneous Compute Abstraction

`IOperatorRegistry` 和 `IComputeContext` 提供后端无关的算子执行：

```cpp
enum class ComputeBackendType { CPU, CUDA, OPENCL, VULKAN, TENSORRT, DLA, CUDLA, RKNN, NPU_GENERIC };

struct Tensor {
    TensorDesc desc;                  // dtype, layout, shape
    BufferHandle buffer;              // 不透明缓冲句柄
    MemoryType mem_type = MemoryType::HOST;
    void* host_ptr = nullptr;
};

struct OperatorImplementation {
    OperatorSignature signature;      // 输入, 输出, 属性
    ComputeBackendType backend;
    int priority;                     // 越高 = 优先
    OperatorImplFunc impl;            // std::function(inputs, outputs, attrs, stream)
};

// 自动后端选择
OperatorRegistry::execute("add", inputs, outputs, attrs, ComputeBackendType::CPU, stream);
```

### 10.6 动态插件管理器 / Dynamic Plugin Manager

`IPluginManager` 支持运行时算法加载/卸载/热切换：

```cpp
// 运行时加载插件
auto instance = plugin_mgr->loadPlugin("libdynalgo_algo_stereo_match.so", "stereo_match");

// 热切换运行中插件，无需停止管道
plugin_mgr->hotSwapPlugin("stereo_match", "libdynalgo_algo_stereo_match_v2.so");

// 完成后卸载
plugin_mgr->unloadPlugin("stereo_match");
```

插件清单定义 I/O 契约和资源需求：

```cpp
struct PluginManifest {
    std::string name, version, vendor;
    PluginType type;  // STEREO_MATCH, LIDAR_SEGMENTATION, IMAGE_DETECTION, 等
    
    std::vector<PluginPort> inputs, outputs;  // 张量规格
    ResourceRequirements resources;            // 内存, GPU, NPU 需求
    std::vector<std::string> required_operators;  // 算子注册表依赖
};
```

### 10.8 DAG 任务调度器 / DAG Task Scheduler

`ITaskScheduler` 以 DAG 形式执行感知管道，支持资源感知调度：

```cpp
struct TaskGraph {
    std::map<std::string, TaskNode> nodes;  // 插件实例 + 依赖
    std::vector<TaskEdge> edges;            // 任务间数据流
    
    // 验证
    bool validate(std::string& error_msg) const;  // 环检测
    std::vector<std::string> topologicalSort() const;
};

struct TaskNode {
    std::string plugin_instance;    // 插件实例名
    std::vector<std::string> dependencies;  // 上游任务 ID
    ResourceRequirements resources;
    ComputeBackendType preferred_backend;
    int priority = 0;
    uint32_t timeout_ms = 5000;
};

// 执行感知管道
auto graph_id = scheduler->submitGraph(graph);
scheduler->setTaskCallback([](graph_id, task_id, status, result) {
    // 处理完成
});
scheduler->start();
```

### 10.9 多模态融合策略 / Multi-Modal Fusion Strategies

| 策略 | 说明 | 适用场景 |
|---|---|---|
| **早期融合** | 将激光雷达点投影到图像平面，像素级融合 | 稠密深度补全 |
| **后期融合** | 分别检测 → 关联 → 目标级融合 | 3D 目标检测 |
| **深度融合** | 通过神经网络进行特征级融合 | 端到端感知 |
| **混合融合** | 上述策略组合 | 复杂场景 |

### 10.10 目录结构更新 / Directory Structure Updates

```
DynamicAlgoCam/
├── include/dynalgo/hal/
│   ├── multimodal_fusion_hal.hpp    # 新增: IFusionEngine, ISyncEngine
│   ├── heterogeneous_compute_hal.hpp # 新增: IOperatorRegistry, IComputeContext
│   ├── plugin_manager_hal.hpp       # 新增: IPlugin, IPluginManager, ITaskScheduler
│   └── ... (现有 HAL)
├── app/core/
│   ├── multimodal_fusion_hal.cpp    # CPU 实现
│   ├── heterogeneous_compute_hal.cpp
│   ├── plugin_manager_hal.cpp
│   └── ... (现有 core)
```

### 10.10 配置扩展 / Configuration Extensions

板卡配置现支持激光雷达和融合：

```yaml
board:
  name: "nvidia_jetson_agx_orin_devkit"
  platform: "aarch64_nvidia"
  
  cameras:
    - id: "cam0"
      connector: "J13 (CSI-A)"
      vendor: "nvsipl"
      sensor_config: "imx728"
      
  lidars:
    - id: "lidar0"
      connector: "J14 (CSI-B)"
      vendor: "robosense"
      sensor_config: "rs_helios"
      coordinate_frame: "sensor"
      
  fusion:
    enabled: true
    fusion_type: "late"
    lidar_to_camera_extrinsics: [0.1, 0.0, 0.05, 0.0, 0.0, 0.0]
```

---

## 4. HAL 接口规范 / HAL Interface Specification

### 4.1 设计规则 / Design Rules

| 规则 | 要求 |
|---|---|
| **纯虚接口** | 所有 HAL 接口 = 抽象 C++ 类，虚析构 |
| **禁用异常** | 错误处理用 `Result<T>` 或错误码 |
| **ABI 边界禁用 STL** | 用 `dynalgo::hal::Vector`、`String`、`Span` |
| **版本化** | `hal_types.hpp` 中定义 `DYNALGO_HAL_VERSION_MAJOR/MINOR` |
| **线程安全** | 所有 HAL 方法必须线程安全 |
| **零拷贝** | 缓冲区按句柄传递；所有权明确 |

### 4.2 核心 HAL 类型 / Core HAL Types

```cpp
// include/dynalgo/hal/hal_types.hpp
namespace dynalgo::hal {

constexpr uint32_t HAL_VERSION_MAJOR = 1;
constexpr uint32_t HAL_VERSION_MINOR = 0;

// 结果类型 (无异常)
template<typename T>
class Result {
    uint32_t error_code_;
    T value_;
public:
    static Result ok(T v) { Result r; r.error_code_ = 0; r.value_ = std::move(v); return r; }
    static Result err(uint32_t code) { Result r; r.error_code_ = code; return r; }
    bool is_ok() const { return error_code_ == 0; }
    uint32_t error() const { return error_code_; }
    T& value() { return value_; }
    const T& value() const { return value_; }
};

enum class ErrorCode : uint32_t {
    OK = 0,
    INVALID_ARG = 1,
    NOT_INITIALIZED = 2,
    ALREADY_INITIALIZED = 3,
    NOT_SUPPORTED = 4,
    TIMEOUT = 5,
    OUT_OF_MEMORY = 6,
    DEVICE_ERROR = 7,
    PERMISSION_DENIED = 8,
    VENDOR_BASE = 0x10000  // 厂商专用错误从此开始
};

// 缓冲区句柄 (不透明，平台特定)
struct BufferHandle {
    uint64_t handle = 0;      // NvSciBufObj / dma-buf fd / V4L2 buffer index
    uint32_t platform_id = 0;
    void* reserved = nullptr;
    bool is_valid() const { return handle != 0; }
};

// 同步栅栏句柄
struct FenceHandle {
    uint64_t handle = 0;      // NvSciSyncFence / sync_file fd
    uint32_t platform_id = 0;
    bool is_valid() const { return handle != 0; }
};

// 像素格式 (兼容 V4L2)
enum class PixelFormat : uint32_t {
    Y8 = 0x38595659, NV12 = 0x3231564E, YUYV = 0x56595559,
    RGB24 = 0x33324247, BGR24 = 0x33324242, RGBA32 = 0x41524742,
    RAW10 = 0x30315741, RAW12 = 0x32315741,
    H264 = 0x34363248, H265 = 0x35363248, METADATA = 0x4154414D,
};

// 帧元数据
struct FrameMetadata {
    uint64_t timestamp_ns = 0;
    uint32_t frame_id = 0;
    uint32_t sensor_id = 0;
    PixelFormat format = PixelFormat::Y8;
    uint32_t width = 0, height = 0, stride = 0;
    BufferHandle buffer;
    FenceHandle acquire_fence;    // 访问前等待
    FenceHandle release_fence;    // 完成后信号
    void* vendor_data = nullptr;
    size_t vendor_data_size = 0;
};

} // namespace dynalgo::hal
```

### 4.3 Camera HAL / 相机 HAL

```cpp
// include/dynalgo/hal/camera_hal.hpp
namespace dynalgo::hal {

struct CameraConfig {
    std::string device_id;            // 来自板卡配置的逻辑设备 ID
    uint32_t width = 1920, height = 1080, fps = 30;
    PixelFormat format = PixelFormat::NV12;
    uint32_t buffer_count = 4;
    bool enable_hw_sync = true;       // 硬件帧同步 (GPIO/MIPI)
    void* vendor_params = nullptr;
    size_t vendor_params_size = 0;
};

struct MultiCameraConfig {
    std::vector<CameraConfig> cameras;
    bool synchronized = true;
    uint32_t master_camera = 0;
};

class ICameraHAL {
public:
    virtual ~ICameraHAL() = default;

    // 能力查询
    virtual Result<std::vector<StreamConfig>> getSupportedStreams(
        const std::string& device_id) = 0;

    // 生命周期
    virtual ResultVoid initialize(const CameraConfig& config) = 0;
    virtual ResultVoid initializeMulti(const MultiCameraConfig& config) = 0;
    virtual ResultVoid start() = 0;
    virtual ResultVoid stop() = 0;
    virtual ResultVoid deinitialize() = 0;

    // 帧获取 (阻塞 + 超时)
    virtual Result<FrameMetadata> acquireFrame(uint32_t timeout_ms = 1000) = 0;
    virtual Result<std::vector<FrameMetadata>> acquireFrames(uint32_t timeout_ms = 1000) = 0;

    // 异步回调
    using FrameCallback = std::function<void(const FrameMetadata&)>;
    virtual ResultVoid registerCallback(FrameCallback cb) = 0;
    virtual ResultVoid unregisterCallback() = 0;

    // 缓冲管理 (零拷贝)
    virtual Result<BufferHandle> importBuffer(const BufferHandle& external) = 0;
    virtual ResultVoid releaseBuffer(const BufferHandle& buffer) = 0;

    // 控制 (曝光、增益、对焦等)
    virtual ResultVoid setControl(uint32_t control_id, int64_t value) = 0;
    virtual Result<int64_t> getControl(uint32_t control_id) = 0;

    // 传感器信息
    virtual Result<std::string> getSensorName() const = 0;
    virtual Result<std::string> getDriverVersion() const = 0;

    // 平台特定扩展
    virtual ResultVoid vendorCommand(uint32_t cmd_id,
                                     const void* in_data, size_t in_size,
                                     void* out_data, size_t out_size) = 0;
};

// 工厂 (动态加载)
class CameraHALFactory {
public:
    static Result<ICameraHAL*> create(const std::string& platform,
                                      const std::string& vendor);
    static void destroy(ICameraHAL* hal);
};

} // namespace dynalgo::hal
```

### 4.4 其他 HAL / Other HALs

```cpp
// encoder_hal.hpp
struct EncodeConfig {
    uint32_t width, height, fps, bitrate_bps;
    PixelFormat input_format = PixelFormat::NV12;
    PixelFormat output_format = PixelFormat::H264;
    uint32_t gop_size = 30, profile = 100, level = 51;
    bool enable_sps_pps_per_idr = true;
};

struct BitstreamBuffer {
    BufferHandle buffer;
    uint32_t size = 0;
    bool is_keyframe = false;
    uint64_t timestamp_ns = 0;
    FenceHandle fence;
};

class IEncoderHAL {
public:
    virtual ~IEncoderHAL() = default;
    virtual ResultVoid initialize(const EncodeConfig&) = 0;
    virtual ResultVoid start() = 0;
    virtual ResultVoid stop() = 0;
    virtual ResultVoid deinitialize() = 0;
    virtual ResultVoid encodeFrame(const FrameMetadata&, BitstreamBuffer&) = 0;
    virtual ResultVoid flush() = 0;
};

// inference_hal.hpp
struct ModelConfig {
    std::string model_path;
    std::string precision = "FP16";
    int device_id = 0, dla_core = -1;
    uint32_t max_batch = 1, workspace_size_mb = 512;
};

struct TensorInfo {
    std::string name;
    std::vector<int32_t> shape;
    PixelFormat format;
    BufferHandle buffer;
};

struct InferenceResult {
    std::vector<TensorInfo> outputs;
    uint64_t latency_us = 0;
    FenceHandle fence;
};

class IInferenceHAL {
public:
    virtual ~IInferenceHAL() = default;
    virtual ResultVoid initialize(const ModelConfig&) = 0;
    virtual ResultVoid infer(const std::vector<TensorInfo>&, InferenceResult&) = 0;
    virtual ResultVoid inferAsync(const std::vector<TensorInfo>&,
                                  std::function<void(InferenceResult)>) = 0;
    virtual Result<std::vector<TensorInfo>> getInputSpecs() = 0;
    virtual Result<std::vector<TensorInfo>> getOutputSpecs() = 0;
};
```

---

## 5. 平台/厂商/板卡配置 / Platform/Vendor/Board Configuration

### 5.1 平台配置 (每 SoC 架构)

```yaml
# bsp/configs/platforms/aarch64_jetson_agx_orin.yaml
platform:
  name: "aarch64_nvidia"
  arch: "aarch64"
  vendor: "nvidia"
  soc: "orin"
  soc_revision: "A02"

  defaults:
    camera: "nvsipl"
    encoder: "nvmedia"
    inference: "tensorrt"
    display: "nvdisplay"
    actuator: "gpio"

  resources:
    max_camera_channels: 16
    max_encode_channels: 8
    max_inference_streams: 4
    gpu_memory_mb: 3072
    dla_cores: 2

  features:
    hardware_isp: true
    hardware_encoder: true
    hardware_decoder: true
    dla: true
    ptp: true
    gpio: true
    can: true
```

### 5.2 厂商配置 (每厂商实现)

```yaml
# bsp/configs/vendors/nvsipl/imx728.yaml
vendor:
  name: "nvsipl"
  platform: "aarch64_nvidia"
  library: "libdynalgo_hal_camera_nvsipl.so"

  sensor:
    name: "IMX728"
    resolution: "3840x2160"
    fps: 30
    interface: "MIPI CSI-2 4-lane"
    pixel_format: "RAW12"
    hdr: "DOL 3-exposure"

  platform_cfg: "platforms/nvidia/orin/imx728_max96712.yaml"

  isp:
    mode: "auto"
    ae_target: 18
    awb_mode: "auto"
    ccm: "standard"

  sync:
    master: true
    gpio_pin: 12
    trigger_mode: "hardware"

  buffers:
    count: 6
    layout: "block_linear"
    cuda_compatible: true
```

### 5.3 板卡配置 (每载板)

```yaml
# bsp/configs/boards/nvidia_jetson_agx_orin_devkit.yaml
board:
  name: "nvidia_jetson_agx_orin_devkit"
  platform: "aarch64_nvidia"
  revision: "A01"

  cameras:
    - id: "cam0"
      connector: "J13 (CSI-A)"
      vendor: "nvsipl"
      sensor_config: "imx728"
      position: "front_center"
      orientation: 0
      intrinsics: "calibration/imx728_front.yml"
      extrinsics: "calibration/extrinsics_front.yml"

    - id: "cam1"
      connector: "J14 (CSI-B)"
      vendor: "nvsipl"
      sensor_config: "imx623"
      position: "rear_center"
      orientation: 180
      intrinsics: "calibration/imx623_rear.yml"
      extrinsics: "calibration/extrinsics_rear.yml"

  actuators:
    - id: "actuator0"
      type: "gimbal"
      interface: "gpio_pwm"
      gpio_pan: 18
      gpio_tilt: 19
      vendor: "gpio"

  sync:
    ptp_enabled: true
    ptp_interface: "eth0"
    gpio_sync_pin: 12
    gpio_sync_polarity: "rising"

  power:
    mode: "MAXN"
    thermal_zone: "cpu-gpu"
    max_temp_c: 85

  storage:
    data_path: "/data/dynalgo"
    cache_path: "/tmp/dynalgo"
    log_path: "/var/log/dynalgo"
```

---

## 6. 构建系统集成 / Build System Integration

### 6.1 顶层 CMake / Top-Level CMake

```cmake
# CMakeLists.txt - 平台自动检测
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64")
    set(DYNALGO_PLATFORM "x86_64")
    set(DYNALGO_DEFAULT_VENDOR_CAMERA "uvc")
    set(DYNALGO_DEFAULT_VENDOR_ENCODER "ffmpeg")
    set(DYNALGO_DEFAULT_VENDOR_INFERENCE "tensorrt")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    if(EXISTS "/etc/nv_tegra_release" OR EXISTS "/proc/device-tree/compatible")
        set(DYNALGO_PLATFORM "aarch64_nvidia")
        set(DYNALGO_DEFAULT_VENDOR_CAMERA "nvsipl")
        set(DYNALGO_DEFAULT_VENDOR_ENCODER "nvmedia")
        set(DYNALGO_DEFAULT_VENDOR_INFERENCE "tensorrt")
    else()
        set(DYNALGO_PLATFORM "aarch64_generic")
        set(DYNALGO_DEFAULT_VENDOR_CAMERA "v4l2")
        set(DYNALGO_DEFAULT_VENDOR_ENCODER "ffmpeg")
        set(DYNALGO_DEFAULT_VENDOR_INFERENCE "onnxruntime")
    endif()
# ... mips64, riscv64 类似
endif()

# 厂商可覆盖
set(DYNALGO_VENDOR_CAMERA "${DYNALGO_DEFAULT_VENDOR_CAMERA}" CACHE STRING "Camera vendor")
set(DYNALGO_VENDOR_ENCODER "${DYNALGO_DEFAULT_VENDOR_ENCODER}" CACHE STRING "Encoder vendor")
set(DYNALGO_VENDOR_INFERENCE "${DYNALGO_DEFAULT_VENDOR_INFERENCE}" CACHE STRING "Inference vendor")

include(cmake/platforms/${DYNALGO_PLATFORM}.cmake)
add_subdirectory(hal)
add_subdirectory(app)
```

### 6.2 平台 CMake / Platform CMake

```cmake
# cmake/platforms/aarch64_nvidia.cmake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a -mtune=cortex-a78ae")

find_package(CUDA 12.2 REQUIRED)
set(CUDA_ARCHITECTURES "87")

find_library(NVSIPL_LIB nvsipl PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVMEDIA_IEP_LIB nvmedia_iep_sci PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVSCIBUF_LIB nvscibuf PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVSCISYNC_LIB nvscisync PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVSCISTREAM_LIB nvscistream PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_package(TensorRT 8.6 REQUIRED)

add_compile_definitions(
    DYNALGO_PLATFORM_AARCH64_NVIDIA=1
    DYNALGO_HAVE_NVSIPL=1
    DYNALGO_HAVE_NVMEDIA=1
    DYNALGO_HAVE_NVSCIBUF=1
    DYNALGO_HAVE_NVSCISYNC=1
    DYNALGO_HAVE_CUDLA=1
)

set(HAL_CAMERA_LIB "dynalgo_hal_camera_nvsipl")
set(HAL_ENCODER_LIB "dynalgo_hal_encoder_nvmedia")
set(HAL_INFERENCE_LIB "dynalgo_hal_inference_tensorrt")
set(HAL_DISPLAY_LIB "dynalgo_hal_display_nvdisplay")
set(HAL_ACTUATOR_LIB "dynalgo_hal_actuator_gpio")
```

### 6.3 HAL CMake / HAL Layer CMake

```cmake
# hal/CMakeLists.txt - 按选择的厂商构建共享库
add_subdirectory(${DYNALGO_PLATFORM}/camera/${DYNALGO_VENDOR_CAMERA})
add_subdirectory(${DYNALGO_PLATFORM}/encoder/${DYNALGO_VENDOR_ENCODER})
add_subdirectory(${DYNALGO_PLATFORM}/inference/${DYNALGO_VENDOR_INFERENCE})
add_subdirectory(${DYNALGO_PLATFORM}/display/${DYNALGO_VENDOR_DISPLAY})
add_subdirectory(${DYNALGO_PLATFORM}/actuator/${DYNALGO_VENDOR_ACTUATOR})

install(DIRECTORY ../include/dynalgo/hal/ DESTINATION include/dynalgo/hal)
install(TARGETS ${HAL_CAMERA_LIB} ${HAL_ENCODER_LIB} ${HAL_INFERENCE_LIB}
                ${HAL_DISPLAY_LIB} ${HAL_ACTUATOR_LIB}
        LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
```

### 6.4 厂商 HAL CMake 示例

```cmake
# hal/aarch64_nvidia/camera/nvsipl/CMakeLists.txt
add_library(dynalgo_hal_camera_nvsipl SHARED
    nvsipl_camera_hal.cpp
    nvsipl_producer.cpp
    nvsipl_consumer_cuda.cpp
    nvsipl_consumer_enc.cpp
    nvsipl_consumer_display.cpp
    nvsipl_platform_cfg.cpp
)

target_link_libraries(dynalgo_hal_camera_nvsipl PRIVATE
    ${NVSIPL_LIB} ${NVMEDIA_LIB} ${NVMEDIA_IEP_LIB}
    ${NVSCIBUF_LIB} ${NVSCISYNC_LIB} ${NVSCISTREAM_LIB}
    ${CUDA_LIBRARIES} ${CUDLA_LIB}
    dynalgo_hal_common
)

set_target_properties(dynalgo_hal_camera_nvsipl PROPERTIES
    VERSION ${DYNALGO_VERSION}
    SOVERSION ${HAL_VERSION_MAJOR}
    CXX_STANDARD 17
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)
```

---

## 7. 应用层使用 / Application Layer Usage

### 7.1 HAL 工厂 / HAL Factory

```cpp
// app/core/dynalgo_hal_factory.cpp
namespace dynalgo {

class HALFactory {
public:
    struct HALBundle {
        std::unique_ptr<hal::ICameraHAL> camera;
        std::unique_ptr<hal::IEncoderHAL> encoder;
        std::unique_ptr<hal::IInferenceHAL> inference;
        std::unique_ptr<hal::IDisplayHAL> display;
        std::unique_ptr<hal::IActuatorHAL> actuator;
    };

    static Result<HALBundle> createFromConfig(const PlatformConfig& platform_cfg,
                                               const BoardConfig& board_cfg) {
        HALBundle bundle;
        auto cam_result = loadCameraHAL(platform_cfg, board_cfg);
        if (!cam_result.is_ok()) return Result<HALBundle>::err(cam_result.error());
        bundle.camera = std::move(cam_result.value());
        // ... encoder, inference, display, actuator
        return Result<HALBundle>::ok(std::move(bundle));
    }

private:
    static Result<std::unique_ptr<hal::ICameraHAL>> loadCameraHAL(
        const PlatformConfig& platform_cfg, const BoardConfig& board_cfg) {
        std::string vendor = platform_cfg.getVendor("camera");
        std::string lib_name = "libdynalgo_hal_camera_" + vendor + ".so";

        void* handle = dlopen(lib_name.c_str(), RTLD_NOW | RTLD_LOCAL);
        // dlsym create/destroy, 初始化, 返回 unique_ptr with custom deleter
    }
};

} // namespace dynalgo
```

### 7.2 应用代码 (仅 HAL) / Application Code (HAL-only)

```cpp
// app/core/dynalgo_capture_session.cpp
namespace dynalgo {

class CaptureSession {
public:
    CaptureSession(const HALFactory::HALBundle& hal) : hal_(hal) {}

    bool start() {
        hal_.camera->start();
        if (hal_.encoder) hal_.encoder->start();
        if (hal_.inference) hal_.inference->initialize(model_config_);
        running_ = true;
        capture_thread_ = std::thread(&CaptureSession::captureLoop, this);
        return true;
    }

private:
    void captureLoop() {
        while (running_) {
            auto frame_result = hal_.camera->acquireFrame(33);
            if (!frame_result.is_ok()) continue;

            FrameMetadata frame = std::move(frame_result.value());
            processFrame(frame);
            hal_.camera->releaseBuffer(frame.buffer);  // 归还缓冲
        }
    }

    void processFrame(const FrameMetadata& frame) {
        if (hal_.encoder) {
            hal::BitstreamBuffer bitstream;
            hal_.encoder->encodeFrame(frame, bitstream);
        }
        if (hal_.inference) {
            hal::TensorInfo input;
            input.buffer = frame.buffer;
            input.format = frame.format;
            hal_.inference->inferAsync({input},
                [this, frame](hal::InferenceResult result) {
                    onInferenceDone(frame, result);
                });
        }
        if (hal_.display) hal_.display->presentFrame(frame);
    }

    HALFactory::HALBundle hal_;
    std::thread capture_thread_;
    std::atomic<bool> running_{false};
};

} // namespace dynalgo
```

---

## 8. 部署与交付 / Deployment & Delivery

### 8.1 发布包结构

```
dynalgo-cam-2.0.0-linux-aarch64/
├── bin/dynamic_algo_cam
├── lib/libdynalgo_core.so
├── lib/hal/
│   ├── libdynalgo_hal_camera_nvsipl.so
│   ├── libdynalgo_hal_encoder_nvmedia.so
│   ├── libdynalgo_hal_inference_tensorrt.so
│   ├── libdynalgo_hal_display_nvdisplay.so
│   └── libdynalgo_hal_actuator_gpio.so
├── config/
│   ├── platform.yaml
│   ├── board.yaml
│   └── vendor/nvsipl/imx728.yaml
├── calibration/imx728_front.yml
├── systemd/dynalgo-cam.service
└── manifest.json
```

### 8.2 manifest.json

```json
{
  "package": "dynalgo-cam",
  "version": "2.0.0",
  "platform": "aarch64_nvidia",
  "soc": "orin",
  "hal_version": "1.0",
  "dependencies": {
    "cuda": "12.2", "tensorrt": "8.6", "nvsipl": "6.0.8",
    "nvmedia": "6.0.8", "nvscibuf": "1.0", "nvscisync": "1.0"
  },
  "hal_plugins": {
    "camera": "nvsipl", "encoder": "nvmedia",
    "inference": "tensorrt", "display": "nvdisplay", "actuator": "gpio"
  }
}
```

---

## 9. 安全与认证 / Safety & Certification

| 方面 | 要求 | 实现 |
|---|---|---|
| **内存安全** | 热路径零动态分配 | 预分配缓冲池、`std::array` |
| **确定性** | 有界最坏延迟 | 实时调度 (SCHED_FIFO)、CPU 亲和性 |
| **故障隔离** | 单 HAL 故障不崩溃 | 进程隔离 (每 HAL 独立进程) |
| **看门狗** | 每管线硬件看门狗 | NvSIPL watchdog + 应用心跳 |
| **配置校验** | 启动时模式验证 | JSON Schema / YAML 校验 |
| **审计追踪** | 结构化日志 | `ILoggerHAL` 带严级、组件、时间戳 |
| **安全启动** | 签名 HAL 插件 | `dlopen` 签名验证 |

---

## 10. 迁移路线图 / Migration Roadmap

| 阶段 | 周期 | 范围 | 交付 |
|---|---|---|---|
| **0. 基础设施** | 2周 | HAL 类型系统、错误码、缓冲/同步抽象 | `include/dynalgo/hal/`, `hal/common/` |
| **1. x86 HAL 移植** | 3周 | 现有 x86 驱动 → HAL 插件 | `hal/x86_64/` (uvc, orbbec, robosense, stereo) |
| **2. ARM/NVIDIA HAL** | 4周 | NvSIPL, NvMedia, TensorRT, CUDLA HAL | `hal/aarch64_nvidia/` (7+ 模块) |
| **3. 应用重构** | 2周 | `CaptureSession` 等仅依赖 HAL | `app/core/` 重构 |
| **4. 配置系统** | 2周 | 平台/厂商/板卡 YAML、校验、加载器 | `bsp/configs/`, `dynalgo_config_manager.cpp` |
| **5. MIPS/RISC-V 骨架** | 2周 | 目录结构、存根 HAL、CI 集成 | `hal/mips64_loongson/`, `hal/riscv64_starfive/` |
| **6. 构建/部署** | 2周 | 交叉编译、打包、systemd、OTA | `scripts/`, `manifest.json`, Docker |
| **7. 测试/认证** | 3周 | 单元/集成/性能测试、安全分析 | `tests/`, 安全文档 |
| **8. 文档** | 1周 | HAL 规范、移植指南、API 参考 | `docs/architecture/` |

**总计: ~21 周完成全多平台工业级发布**

---

## 第一阶段实现状态 / Phase 1 Implementation Status (✅ 已完成 / COMPLETED)

| 任务 | 状态 | 文件 | 构建验证 |
|---|---|---|---|
| HAL 目录结构 | ✅ 完成 | `include/dynalgo/hal/`, `hal/common/`, `hal/x86_64/`, `hal/aarch64_nvidia/` | ✅ |
| `hal_types.hpp` - 核心类型 | ✅ 完成 | `Result<T>`, `ErrorCode`, `BufferHandle`, `FenceHandle`, `PixelFormat`, `FrameMetadata` | ✅ |
| `camera_hal.hpp` | ✅ 完成 | `ICameraHAL`, `CameraConfig`, `MultiCameraConfig`, `CameraHALFactory` | ✅ |
| `encoder_hal.hpp` | ✅ 完成 | `IEncoderHAL`, `EncodeConfig`, `BitstreamBuffer` | ✅ |
| `inference_hal.hpp` | ✅ 完成 | `IInferenceHAL`, `ModelConfig`, `TensorInfo`, `InferenceResult` | ✅ |
| `display_hal.hpp` | ✅ 完成 | `IDisplayHAL`, `DisplayConfig`, `PresentConfig` | ✅ |
| `actuator_hal.hpp` | ✅ 完成 | `IActuatorHAL`, `ActuatorConfig`, `ActuatorCommand` | ✅ |
| `sensor_hal.hpp` | ✅ 完成 | `ISensorHAL`, `SensorConfig`, `SensorData` (std::variant) | ✅ |
| `logger_hal.hpp` | ✅ 完成 | `ILoggerHAL`, `LogConfig`, `LogDestination` (位运算) | ✅ |
| `time_hal.hpp` | ✅ 完成 | `ITimeHAL`, `TimeConfig`, `PTPConfig` | ✅ |
| HAL 通用组件 (`buffer_manager`, `sync_manager`, `plugin_loader`, `hal_base`) | ✅ 完成 | 零拷贝缓冲池、同步原语、动态插件加载 | ✅ |
| 平台 CMake 配置 (x86_64, aarch64_nvidia, aarch64_generic, mips64_loongson, riscv64_starfive) | ✅ 完成 | 自动检测、厂商默认值、依赖查找 | ✅ |
| HAL CMakeLists.txt | ✅ 完成 | 自动平台检测、厂商选择、共享库构建 | ✅ |
| x86_64 厂商存根 (uvc, orbbec, robosense, ffmpeg, vaapi, tensorrt, sdl2, generic_serial, sensor, spdlog, chrono) | ✅ 完成 | 所有 9 种 HAL 类型的存根实现 | ✅ **已构建** |
| aarch64_nvidia 厂商存根 (nvsipl, v4l2, nvmedia, nvenc, tensorrt, cudla, nvdisplay, gpio, iio, spdlog, ptp) | ✅ 完成 | 所有 9 种 HAL 类型的存根实现 | ✅ |

**构建验证**: 所有 9 个 x86_64 HAL 目标构建成功：
- `dynalgo_hal_common` (静态库)
- `dynalgo_hal_camera_uvc`
- `dynalgo_hal_encoder_ffmpeg`
- `dynalgo_hal_inference_tensorrt`
- `dynalgo_hal_display_sdl2`
- `dynalgo_hal_actuator_generic_serial`
- `dynalgo_hal_sensor_generic`
- `dynalgo_hal_logger_spdlog`
- `dynalgo_hal_time_chrono`

---

## x86_64 厂商 HAL 实现 (已完成) / x86_64 Vendor HAL Implementations (Completed)

以下厂商特定 HAL 实现已在 x86_64 平台完成：

| HAL / 厂商 | 实现文件 | 功能 | 构建状态 |
|---|---|---|---|
| **Orbbec Camera HAL** | `hal/x86_64/camera/orbbec/orbbec_camera_hal.cpp` | • Gemini 305, 305g (GMSL2), 335L, 336L<br>• 多相机: IR 立体对 (IR_LEFT + IR_RIGHT)<br>• 335L/336L 硬件 D2C 对齐<br>• 通过 dlopen 动态加载 SDK<br>• GMSL2 连接支持<br>• 标定加载 (内参/外参) | ✅ **已构建** |
| **RoboSense AC1 Camera HAL** | `hal/x86_64/camera/robosense/robosense_camera_hal.cpp` | • RoboSense RoboX AC1 (相机 + LiDAR 一体化)<br>• 彩色 (1920x1080@30), 深度 (640x480@30), LiDAR 点云, IMU<br>• 硬件同步支持 (GPIO/PTP)<br>• 固定流规格符合 AC1 规格 | ✅ **已构建** |
| **RoboSense LiDAR Sensor HAL** | `hal/x86_64/sensor/robosense/robosense_lidar_hal.cpp` | • 实现 `ISensorHAL` 用于 AC1 LiDAR<br>• 输出 `FrameBuffer` with `FrameType::POINT_CLOUD`<br>• `SyncConfig` 支持 HW/PTP/软件同步<br>• LiDAR 点云 + IMU 回调 | ✅ **已构建** |
| **Stereo Camera HAL** | `hal/x86_64/camera/stereo/stereo_camera_hal.cpp` | • 桥接 `UvcStereoCamera` 驱动到 `ICameraHAL`<br>• 双 UVC 相机硬件同步<br>• OpenCV 标定矫正 (`cv::initUndistortRectifyMap`, `cv::remap`)<br>• SGBM 立体匹配深度/视差计算<br>• 从 OpenCV YAML/XML 加载标定 | ✅ **已构建** |

---

## x86_64 CMake 配置 / x86_64 CMake Configuration

以下 CMake 选项控制 x86_64 HAL 构建 (x86_64 平台默认 ON)：

```cmake
option(ENABLE_ORBBEC_HAL "Build Orbbec Camera HAL (305/305g/335L/336L)" ON)
option(ENABLE_ROBOSENSE_HAL "Build RoboSense AC1 Camera HAL" ON)
option(ENABLE_STEREO_HAL "Build Generic UVC Stereo Camera HAL" ON)
option(ENABLE_ROBOSENSE_LIDAR_HAL "Build RoboSense AC1 LiDAR Sensor HAL" ON)
```

**库输出**: 所有 HAL 共享库构建到 `./lib/dynalgo/hal/` (构建目录本地)。

---

## HAL 工厂注册 / HAL Factory Registration

厂商插件在应用启动时通过 `HALFactory::registerX86_64Vendors()` 注册：

```cpp
// app/core/dynalgo_hal_factory.cpp
void HALFactory::registerX86_64Vendors() {
    // Camera HALs
    hal::CameraHALFactory::registerVendor("x86_64", "orbbec",
        []() -> hal::ICameraHAL* { return new dynalgo::OrbbecCameraHAL(); },
        [](hal::ICameraHAL* p) { delete p; });
    
    hal::CameraHALFactory::registerVendor("x86_64", "robosense",
        []() -> hal::ICameraHAL* { return new dynalgo::RobosenseCameraHAL(); },
        [](hal::ICameraHAL* p) { delete p; });
    
    hal::CameraHALFactory::registerVendor("x86_64", "stereo",
        []() -> hal::ICameraHAL* { return new dynalgo::StereoCameraHAL(); },
        [](hal::ICameraHAL* p) { delete p; });
    
    // Sensor HALs
    hal::SensorHALFactory::registerVendor("x86_64", "robosense_lidar",
        []() -> hal::ISensorHAL* { return new dynalgo::RobosenseLidarHAL(); },
        [](hal::ISensorHAL* p) { delete p; });
}
```

库通过 `dlopen`/`dlsym` 从 `./lib/dynalgo/hal/` 动态加载。

---

## 配置 Schema / Configuration Schema

板卡配置 schema 已扩展以支持多传感器：

```yaml
board:
  name: "x86_devkit"
  platform: "x86_64"
  
  cameras:
    - id: "cam0"
      connector: "/dev/video0"
      vendor: "orbbec"
      sensor_config: "gemini_305"
      connection_type: "usb3"           # usb3, gmsl2, ethernet, pcie
      role: "main"                      # main, stereo_left, stereo_right, depth, ir
      streams:
        - type: "color"
          width: 1920
          height: 1080
          fps: 30
          format: "NV12"
        - type: "depth"
          width: 1280
          height: 800
          fps: 30
          format: "Y16"
          hw_d2c: true
        - type: "ir_left"
          width: 640
          height: 400
          fps: 30
          format: "Y8"
        - type: "ir_right"
          width: 640
          height: 400
          fps: 30
          format: "Y8"
      orbbec_mode: "standard"           # standard, 305g_gmsl2
      disable_ir_left: false
      
  lidars:
    - id: "lidar0"
      connector: "/dev/ttyUSB0"
      vendor: "robosense"
      sensor_config: "rs_ac1"
      connection_type: "usb3"
      coordinate_frame: "sensor"
      streams:
        - type: "points"
          fps: 10
          format: "POINT"
          
  sync:
    sync_method: "hardware_trigger"     # hardware_trigger, ptp, software_timestamp
    sync_group_id: 1
    max_time_diff_ns: 1000000
    enable_interpolation: true
```

---

## 构建验证 / Build Verification

所有 x86_64 HAL 目标构建成功：

```bash
# Build all x86 HALs
cmake -B build -DENABLE_ORBBEC_HAL=ON -DENABLE_ROBOSENSE_HAL=ON -DENABLE_STEREO_HAL=ON -DENABLE_ROBOSENSE_LIDAR_HAL=ON
cmake --build build --target dynalgo_hal_camera_orbbec dynalgo_hal_camera_robosense dynalgo_hal_camera_stereo dynalgo_hal_sensor_robosense_lidar

# Verify libraries
ls build/lib/dynalgo/hal/
# libdynalgo_hal_camera_orbbec.so
# libdynalgo_hal_camera_robosense.so
# libdynalgo_hal_camera_stereo.so
# libdynalgo_hal_sensor_robosense_lidar.so
```

---

## 11. 附录：厂商插件接口 / Appendix: Vendor Plugin Interface

每个 HAL 厂商插件仅导出 **两个 C 符号**：

```cpp
// nvsipl_camera_hal.cpp
extern "C" {
dynalgo::hal::ICameraHAL* dynalgo_hal_camera_create() {
    return new dynalgo::NvSIPLCameraHAL();
}
void dynalgo_hal_camera_destroy(dynalgo::hal::ICameraHAL* ptr) {
    delete ptr;
}
}
```

**插件加载**: `HALFactory` 用 `dlopen`/`dlsym` → 应用零依赖厂商代码。

---

## 12. 术语表 / Glossary

| 术语 | 定义 |
|---|---|
| **HAL** | Hardware Abstraction Layer — App 与平台间的稳定接口 |
| **CS** | Chip Support — 平台特定的 HAL 实现 |
| **BSP** | Board Support Package — 板级配置 (pinmux、标定、电源) |
| **Platform** | SoC 架构类 (x86_64, aarch64_nvidia, mips64_loongson, riscv64_starfive) |
| **Vendor** | 提供 HAL 实现的芯商 (nvidia, orbbec, robosense, generic) |
| **BufferHandle** | 跨平台不透明缓冲引用 (NvSciBufObj, dma-buf fd, V4L2 index) |
| **FenceHandle** | 不透明同步原语 (NvSciSyncFence, sync_file fd) |
| **Zero-Copy** | 无数据拷贝的缓冲所有权转移 |

---

*文档版本: 2.0 | 更新: 2026-09-08 | 密级: 内部 - 设计规范*