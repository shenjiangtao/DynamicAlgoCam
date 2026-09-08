# DynamicAlgoCam Industrial-Grade Multi-Platform Architecture / 工业级多平台架构设计

---

## 1. Architectural Vision / 架构愿景

**Goal**: Build a **production-grade, safety-certifiable** computer vision platform following **industrial software delivery standards** (AUTOSAR-like layering, Android HAL patterns, Linux V4L2/IIO subsystems).
**目标**: 构建 **生产级、可安全认证** 的计算机视觉平台，遵循 **工业软件交付标准**（类 AUTOSAR 分层、Android HAL 模式、Linux V4L2/IIO 子系统）。

### Core Principles / 核心原则

| Principle / 原则 | Description / 说明 |
|---|---|
| **App-Hardware Decoupling** | App layer knows **only** abstract HAL interfaces; zero hardware headers / App 层仅依赖抽象 HAL 接口，零硬件头文件依赖 |
| **HAL as Contract** | HAL = **stable C/C++ ABI** per major version; vendors implement, app consumes / HAL = **稳定 C/C++ ABI**；厂商实现，应用消费 |
| **Platform → Vendor → Board** | 3-level hierarchy: **Platform (SoC arch)** → **Vendor (Silicon vendor)** → **Board (Carrier board)** / 3 级层级：平台 → 厂商 → 板卡 |
| **Configuration over Code** | Hardware bring-up via **YAML/JSON config**, not code changes / 硬件带板通过 **配置文件** 而非代码修改 |
| **Safety by Design** | Deterministic memory, bounded latency, fault containment / 确定性内存、有界延迟、故障隔离 |
| **Supplier Independence** | Multiple vendors per platform; hot-swappable at build/deploy / 每平台多厂商；构建/部署时热插拔 |

---

## 2. Layered Architecture / 分层架构

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

## 3. Directory Structure / 目录结构

```
DynamicAlgoCam/
├── CMakeLists.txt                    # Top-level: platform detection, vendor selection
├── cmake/
│   ├── platforms/                    # Platform detection & defaults
│   │   ├── x86_64.cmake
│   │   ├── aarch64_nvidia.cmake
│   │   ├── mips64_loongson.cmake
│   │   └── riscv64_starfive.cmake
│   ├── vendors/                      # Vendor-specific find modules
│   │   ├── FindNvSIPL.cmake
│   │   ├── FindNvMedia.cmake
│   │   ├── FindOrbbecSDK.cmake
│   │   ├── FindRoboSense.cmake
│   │   └── FindLoongsonGPU.cmake
│   └── toolchains/                   # Cross-compile toolchains
│       ├── jetson-aarch64.cmake
│       ├── loongson-mips64.cmake
│       └── starfive-riscv64.cmake
│
├── include/                          # PUBLIC HAL HEADERS (stable ABI)
│   └── dynalgo/
│       ├── hal/
│       │   ├── camera_hal.hpp        # ICameraHAL, CameraConfig, FrameSet
│       │   ├── encoder_hal.hpp       # IEncoderHAL, EncodeConfig, Bitstream
│       │   ├── inference_hal.hpp     # IInferenceHAL, ModelConfig, Tensor
│       │   ├── display_hal.hpp       # IDisplayHAL, DisplayConfig
│       │   ├── actuator_hal.hpp      # IActuatorHAL, ActuatorConfig
│       │   ├── sensor_hal.hpp        # ISensorHAL (IMU, GPS, etc.)
│       │   ├── logger_hal.hpp        # ILoggerHAL
│       │   ├── time_hal.hpp          # ITimeHAL (monotonic, PTP, GPS)
│       │   └── hal_types.hpp         # Common HAL types, error codes
│       ├── core/
│       │   ├── dynalgo_frame.hpp     # DynalgoFrame (HAL-agnostic)
│       │   ├── dynalgo_types.hpp     # Core types (detection, bbox, 3D)
│       │   └── dynalgo_errors.hpp    # Error codes (HAL + App)
│       └── config/
│           ├── platform_config.hpp   # PlatformConfig (YAML schema)
│           └── vendor_config.hpp     # VendorConfig (per-vendor params)
│
├── hal/                              # HAL IMPLEMENTATIONS (per platform/vendor)
│   ├── CMakeLists.txt                # Adds subdirs based on DYNALGO_PLATFORM
│   ├── common/                       # Shared HAL utilities, base classes
│   │   ├── hal_base.hpp
│   │   ├── buffer_manager.hpp        # NvSciBuf/V4L2/dma-buf unified wrapper
│   │   ├── sync_manager.hpp          # NvSciSync/fence/pthread unified wrapper
│   │   └── plugin_loader.hpp         # Dynamic library loading for vendors
│   │
│   ├── x86_64/                       # x86 PLATFORM
│   │   ├── CMakeLists.txt
│   │   ├── camera/
│   │   │   ├── uvc/                  # VENDOR: Generic UVC
│   │   │   │   ├── uvc_camera_hal.cpp
│   │   │   │   ├── uvc_camera_hal.hpp
│   │   │   │   └── CMakeLists.txt
│   │   │   ├── orbbec/               # VENDOR: Orbbec
│   │   │   │   ├── orbbec_camera_hal.cpp
│   │   │   │   └── CMakeLists.txt
│   │   │   ├── robosense/            # VENDOR: RoboSense
│   │   │   │   ├── rs_camera_hal.cpp
│   │   │   │   └── CMakeLists.txt
│   │   │   ├── realsense/            # VENDOR: Intel RealSense
│   │   │   └── stereo/               # VENDOR: Generic stereo UVC
│   │   ├── encoder/
│   │   │   ├── ffmpeg/               # VENDOR: FFmpeg SW encode
│   │   │   ├── vaapi/                # VENDOR: Intel VAAPI HW encode
│   │   │   └── nvenc/                # VENDOR: NVIDIA NVENC (x86 dGPU)
│   │   ├── inference/
│   │   │   ├── tensorrt/             # VENDOR: TensorRT x86
│   │   │   ├── onnxruntime/          # VENDOR: ONNX Runtime
│   │   │   └── openvino/             # VENDOR: Intel OpenVINO
│   │   ├── display/
│   │   │   ├── sdl2/                 # VENDOR: SDL2
│   │   │   ├── wayland/              # VENDOR: Wayland/DRM
│   │   │   └── x11/                  # VENDOR: X11
│   │   └── actuator/
│   │       ├── generic_serial/       # VENDOR: Serial/USB actuator
│   │       └── can/                  # VENDOR: CAN bus actuator
│   │
│   ├── aarch64_nvidia/               # ARM PLATFORM: NVIDIA Jetson/Drive
│   │   ├── CMakeLists.txt
│   │   ├── camera/
│   │   │   ├── nvsipl/               # VENDOR: NVIDIA SIPL (primary)
│   │   │   │   ├── nvsipl_camera_hal.cpp
│   │   │   │   ├── nvsipl_camera_hal.hpp
│   │   │   │   ├── nvsipl_producer.cpp
│   │   │   │   ├── nvsipl_consumer_cuda.cpp
│   │   │   │   ├── nvsipl_consumer_enc.cpp
│   │   │   │   ├── nvsipl_consumer_display.cpp
│   │   │   │   ├── nvsipl_platform_cfg.cpp
│   │   │   │   └── CMakeLists.txt
│   │   │   ├── v4l2/                 # VENDOR: Generic V4L2 fallback
│   │   │   └── argus/                # VENDOR: libargus (legacy)
│   │   ├── encoder/
│   │   │   ├── nvmedia/              # VENDOR: NvMedia IEP (HW H.264/265)
│   │   │   └── nvenc/                # VENDOR: NVENC (Jetson Orin)
│   │   ├── inference/
│   │   │   ├── tensorrt/             # VENDOR: TensorRT + DLA
│   │   │   ├── cudla/                # VENDOR: CUDLA direct
│   │   │   └── triton/               # VENDOR: Triton Inference Server
│   │   ├── display/
│   │   │   ├── nvdisplay/            # VENDOR: NvDisplay/DRM
│   │   │   └── wfd/                  # VENDOR: OpenWFD
│   │   └── actuator/
│   │       ├── gpio/                 # VENDOR: GPIO PWM
│   │       └── can/                  # VENDOR: CAN (via mttcan)
│   │
│   ├── mips64_loongson/              # MIPS PLATFORM: Loongson
│   │   ├── CMakeLists.txt
│   │   ├── camera/
│   │   │   ├── v4l2/                 # VENDOR: V4L2
│   │   │   └── loongson_isp/         # VENDOR: Loongson ISP
│   │   ├── encoder/
│   │   │   └── loongson_venC/        # VENDOR: Loongson Video Encoder
│   │   ├── inference/
│   │   │   └── loongson_npu/         # VENDOR: Loongson NPU
│   │   └── ...
│   │
│   └── riscv64_starfive/             # RISC-V PLATFORM: StarFive/T-Head
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
├── app/                              # APPLICATION LAYER (HAL-only deps)
│   ├── CMakeLists.txt
│   ├── core/
│   │   ├── dynalgo_capture_session.cpp    # Uses ICameraHAL only
│   │   ├── dynalgo_pipeline_manager.cpp   # Pipeline orchestration
│   │   ├── dynalgo_frame_manager.cpp      # Frame lifecycle (HAL buffers)
│   │   └── dynalgo_config_manager.cpp     # Platform/Vendor/Board config
│   ├── algo/
│   │   ├── engagement/
│   │   │   ├── dynalgo_engagement_loop.cpp
│   │   │   ├── dynalgo_target_selector.cpp
│   │   │   └── dynalgo_track_bundle.cpp
│   │   ├── perception/
│   │   │   ├── detection_postprocess.cpp
│   │   │   ├── nms.cpp                   # Algorithm only, no HW
│   │   │   └── tracker.cpp
│   │   └── fusion/
│   │       ├── sensor_fusion.cpp
│   │       └── calibration.cpp
│   ├── dynamic_algo_cam/
│   │   ├── dynamic_algo_cam.cpp          # Main entry, HAL factory init
│   │   └── cli_parser.cpp
│   └── tools/
│       ├── calibration_tool.cpp
│       └── benchmark_tool.cpp
│
├── bsp/                                # BOARD SUPPORT PACKAGES
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
│   ├── build.sh                        # Host build
│   ├── build_cross.sh                  # Cross-compile (platform arg)
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
│   │   ├── HAL_SPEC.md                 # HAL interface specification
│   │   ├── PLATFORM_PORTING_GUIDE.md   # Adding new platform
│   │   ├── VENDOR_PORTING_GUIDE.md     # Adding new vendor
│   │   ├── BOARD_BRINGUP_GUIDE.md      # Board config
│   │   └── SAFETY_CERTIFICATION_GUIDE.md
│   ├── dynamic_algo_cam/
│   │   ├── MULTI_ARCH_PORTING_PLAN.md  # This document
│   │   ├── MULTI_ARCH_PORTING_PLAN_CN.md
│   │   └── ...
│   └── api/
│       └── hal_api_reference.md
│
└── vendors/                            # 3rd party SDKs (git submodules)
    ├── OrbbecSDK/
    ├── RoboSense/
    └── (NvSIPL/NvMedia from system SDK)
```

---

## 4. HAL Interface Specification / HAL 接口规范

### 4.1 Design Rules / 设计规则

| Rule / 规则 | Requirement / 要求 |
|---|---|
| **Pure Virtual** | All HAL interfaces = abstract C++ classes with virtual destructor / 纯虚类，虚析构 |
| **No Exceptions** | Error handling via `Result<T>` or error codes / 禁用异常，用 `Result<T>` 或错误码 |
| **No STL in ABI** | Use `dynalgo::hal::Vector`, `String`, `Span` / ABI 边界禁用 STL，用自定义容器 |
| **Versioned** | `DYNALGO_HAL_VERSION_MAJOR/MINOR` in `hal_types.hpp` / 版本号 |
| **Thread-Safe** | All HAL methods must be thread-safe / 所有 HAL 方法线程安全 |
| **Zero-Copy** | Buffers passed by handle; ownership explicit / 零拷贝，句柄传递，所有权明确 |

### 4.2 Core HAL Types / 核心 HAL 类型

```cpp
// include/dynalgo/hal/hal_types.hpp
#pragma once
#include <cstdint>
#include <cstddef>

namespace dynalgo::hal {

// Version
constexpr uint32_t HAL_VERSION_MAJOR = 1;
constexpr uint32_t HAL_VERSION_MINOR = 0;

// Result type (no exceptions)
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

using ResultVoid = Result<void>;

// Error codes (extend per HAL)
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
    VENDOR_BASE = 0x10000  // Vendor-specific errors start here
};

// Buffer handle (opaque, platform-specific)
struct BufferHandle {
    uint64_t handle = 0;      // NvSciBufObj / dma-buf fd / V4L2 buffer index
    uint32_t platform_id = 0; // Platform identifier
    void* reserved = nullptr; // Vendor-specific
    bool is_valid() const { return handle != 0; }
};

// Sync fence handle
struct FenceHandle {
    uint64_t handle = 0;      // NvSciSyncFence / sync_file fd
    uint32_t platform_id = 0;
    bool is_valid() const { return handle != 0; }
};

// Pixel formats (V4L2 compatible)
enum class PixelFormat : uint32_t {
    Y8 = 0x38595659,          // 'Y8  '
    Y16 = 0x36315959,         // 'Y16 '
    NV12 = 0x3231564E,        // 'NV12'
    NV21 = 0x3132564E,        // 'NV21'
    YUYV = 0x56595559,        // 'YUYV'
    UYVY = 0x59565955,        // 'UYVY'
    RGB24 = 0x33324247,       // 'RGB3'
    BGR24 = 0x33324242,       // 'BGR3'
    RGBA32 = 0x41524742,      // 'RGBA'
    BGRA32 = 0x41524742,      // 'BGRA'
    RAW8 = 0x38574152,        // 'RAW8'
    RAW10 = 0x30315741,       // 'RAW10'
    RAW12 = 0x32315741,       // 'RAW12'
    H264 = 0x34363248,        // 'H264'
    H265 = 0x35363248,        // 'H265'
    METADATA = 0x4154414D,    // 'META'
};

// Frame metadata
struct FrameMetadata {
    uint64_t timestamp_ns = 0;        // Monotonic/PTT timestamp
    uint32_t frame_id = 0;
    uint32_t sensor_id = 0;
    PixelFormat format = PixelFormat::Y8;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    BufferHandle buffer;
    FenceHandle acquire_fence;        // Wait before access
    FenceHandle release_fence;        // Signal after done
    // Vendor-specific extension
    void* vendor_data = nullptr;
    size_t vendor_data_size = 0;
};

} // namespace dynalgo::hal
```

### 4.3 Camera HAL / 相机 HAL

```cpp
// include/dynalgo/hal/camera_hal.hpp
#pragma once
#include "hal_types.hpp"
#include <functional>
#include <vector>
#include <string>

namespace dynalgo::hal {

struct CameraConfig {
    std::string device_id;            // Logical device ID from board config
    uint32_t width = 1920;
    uint32_t height = 1080;
    uint32_t fps = 30;
    PixelFormat format = PixelFormat::NV12;
    uint32_t buffer_count = 4;        // Ring buffer depth
    bool enable_hw_sync = true;       // Hardware frame sync (GPIO/MIPI)
    // Vendor-specific parameters (opaque)
    void* vendor_params = nullptr;
    size_t vendor_params_size = 0;
};

struct StreamConfig {
    uint32_t stream_id = 0;
    PixelFormat format = PixelFormat::NV12;
    uint32_t width = 1920;
    uint32_t height = 1080;
    // Cropping, scaling, etc.
};

struct MultiCameraConfig {
    std::vector<CameraConfig> cameras;
    bool synchronized = true;         // Hardware sync across cameras
    uint32_t master_camera = 0;       // Master for sync
};

class ICameraHAL {
public:
    virtual ~ICameraHAL() = default;

    // Capability query
    virtual Result<std::vector<StreamConfig>> getSupportedStreams(
        const std::string& device_id) = 0;

    // Lifecycle
    virtual ResultVoid initialize(const CameraConfig& config) = 0;
    virtual ResultVoid initializeMulti(const MultiCameraConfig& config) = 0;
    virtual ResultVoid start() = 0;
    virtual ResultVoid stop() = 0;
    virtual ResultVoid deinitialize() = 0;

    // Frame acquisition (blocking with timeout)
    virtual Result<FrameMetadata> acquireFrame(uint32_t timeout_ms = 1000) = 0;
    virtual Result<std::vector<FrameMetadata>> acquireFrames(
        uint32_t timeout_ms = 1000) = 0;  // For multi-cam sync

    // Async callback mode
    using FrameCallback = std::function<void(const FrameMetadata&)>;
    virtual ResultVoid registerCallback(FrameCallback cb) = 0;
    virtual ResultVoid unregisterCallback() = 0;

    // Buffer management (zero-copy)
    virtual Result<BufferHandle> importBuffer(const BufferHandle& external) = 0;
    virtual ResultVoid releaseBuffer(const BufferHandle& buffer) = 0;

    // Controls (exposure, gain, focus, etc.)
    virtual ResultVoid setControl(uint32_t control_id, int64_t value) = 0;
    virtual Result<int64_t> getControl(uint32_t control_id) = 0;

    // Sensor info
    virtual Result<std::string> getSensorName() const = 0;
    virtual Result<std::string> getDriverVersion() const = 0;

    // Platform-specific extensions
    virtual ResultVoid vendorCommand(uint32_t cmd_id,
                                     const void* in_data, size_t in_size,
                                     void* out_data, size_t out_size) = 0;
};

// Factory (loaded dynamically)
class CameraHALFactory {
public:
    using CreateFunc = ICameraHAL* (*)();
    using DestroyFunc = void (*)(ICameraHAL*);

    static Result<ICameraHAL*> create(const std::string& platform,
                                      const std::string& vendor);
    static void destroy(ICameraHAL* hal);
};

} // namespace dynalgo::hal
```

### 4.4 Other HALs / 其他 HAL

```cpp
// encoder_hal.hpp
struct EncodeConfig {
    uint32_t width, height;
    uint32_t fps;
    uint32_t bitrate_bps;
    PixelFormat input_format = PixelFormat::NV12;
    PixelFormat output_format = PixelFormat::H264;  // or H265
    uint32_t gop_size = 30;
    uint32_t profile = 100;  // Baseline=66, Main=77, High=100
    uint32_t level = 51;     // H.264 level
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
    virtual ResultVoid initialize(const EncodeConfig& config) = 0;
    virtual ResultVoid start() = 0;
    virtual ResultVoid stop() = 0;
    virtual ResultVoid deinitialize() = 0;
    virtual ResultVoid encodeFrame(const FrameMetadata& input,
                                   BitstreamBuffer& output) = 0;
    virtual ResultVoid flush() = 0;
};

// inference_hal.hpp
struct ModelConfig {
    std::string model_path;           // .engine, .onnx, .plan
    std::string precision = "FP16";   // FP32, FP16, INT8
    int device_id = 0;                // GPU/DLA core
    int dla_core = -1;                // -1 = GPU, 0/1 = DLA
    uint32_t max_batch = 1;
    uint32_t workspace_size_mb = 512;
};

struct TensorInfo {
    std::string name;
    std::vector<int32_t> shape;       // NCHW or NHWC
    PixelFormat format;               // Data type
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
    virtual ResultVoid initialize(const ModelConfig& config) = 0;
    virtual ResultVoid infer(const std::vector<TensorInfo>& inputs,
                             InferenceResult& output) = 0;
    virtual ResultVoid inferAsync(const std::vector<TensorInfo>& inputs,
                                  std::function<void(InferenceResult)> cb) = 0;
    virtual Result<std::vector<TensorInfo>> getInputSpecs() = 0;
    virtual Result<std::vector<TensorInfo>> getOutputSpecs() = 0;
};
```

---

## 5. Platform/Vendor/Board Configuration / 平台/厂商/板卡配置

### 5.1 Platform Config (per SoC architecture) / 平台配置

```yaml
# bsp/configs/platforms/aarch64_jetson_agx_orin.yaml
platform:
  name: "aarch64_nvidia"
  arch: "aarch64"
  vendor: "nvidia"
  soc: "orin"
  soc_revision: "A02"

  # Default vendors for this platform
  defaults:
    camera: "nvsipl"
    encoder: "nvmedia"
    inference: "tensorrt"
    display: "nvdisplay"
    actuator: "gpio"

  # Resource limits
  resources:
    max_camera_channels: 16
    max_encode_channels: 8
    max_inference_streams: 4
    gpu_memory_mb: 3072
    dla_cores: 2

  # Features
  features:
    hardware_isp: true
    hardware_encoder: true
    hardware_decoder: true
    dla: true
    ptp: true
    gpio: true
    can: true
```

### 5.2 Vendor Config (per vendor implementation) / 厂商配置

```yaml
# bsp/configs/vendors/nvsipl/imx728.yaml
vendor:
  name: "nvsipl"
  platform: "aarch64_nvidia"
  library: "libdynalgo_hal_camera_nvsipl.so"

  # Sensor-specific
  sensor:
    name: "IMX728"
    resolution: "3840x2160"
    fps: 30
    interface: "MIPI CSI-2 4-lane"
    pixel_format: "RAW12"
    hdr: "DOL 3-exposure"

  # PlatformCfg reference (NvSIPL)
  platform_cfg: "platforms/nvidia/orin/imx728_max96712.yaml"

  # ISP tuning
  isp:
    mode: "auto"
    ae_target: 18
    awb_mode: "auto"
    ccm: "standard"

  # Synchronization
  sync:
    master: true
    gpio_pin: 12
    trigger_mode: "hardware"

  # Buffer configuration
  buffers:
    count: 6
    layout: "block_linear"
    cuda_compatible: true
```

### 5.3 Board Config (per carrier board) / 板卡配置

```yaml
# bsp/configs/boards/nvidia_jetson_agx_orin_devkit.yaml
board:
  name: "nvidia_jetson_agx_orin_devkit"
  platform: "aarch64_nvidia"
  revision: "A01"

  # Camera connectors
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

    - id: "cam2"
      connector: "J15 (CSI-C)"
      vendor: "nvsipl"
      sensor_config: "ar0820"
      position: "left_side"
      orientation: 90
      intrinsics: "calibration/ar0820_left.yml"
      extrinsics: "calibration/extrinsics_left.yml"

    - id: "cam3"
      connector: "J16 (CSI-D)"
      vendor: "nvsipl"
      sensor_config: "ar0820"
      position: "right_side"
      orientation: 270
      intrinsics: "calibration/ar0820_right.yml"
      extrinsics: "calibration/extrinsics_right.yml"

  # Actuators
  actuators:
    - id: "actuator0"
      type: "gimbal"
      interface: "gpio_pwm"
      gpio_pan: 18
      gpio_tilt: 19
      vendor: "gpio"

    - id: "actuator1"
      type: "trigger"
      interface: "gpio"
      gpio_pin: 20
      vendor: "gpio"

  # Synchronization
  sync:
    ptp_enabled: true
    ptp_interface: "eth0"
    gpio_sync_pin: 12
    gpio_sync_polarity: "rising"

  # Power/thermal
  power:
    mode: "MAXN"
    thermal_zone: "cpu-gpu"
    max_temp_c: 85

  # Storage
  storage:
    data_path: "/data/dynalgo"
    cache_path: "/tmp/dynalgo"
    log_path: "/var/log/dynalgo"
```

---

## 6. Build System Integration / 构建系统集成

### 6.1 Top-Level CMake / 顶层 CMake

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.20)
project(DynamicAlgoCam VERSION 2.0.0 LANGUAGES CXX C)

# === PLATFORM DETECTION ===
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64")
    set(DYNALGO_PLATFORM "x86_64" CACHE STRING "Target platform")
    set(DYNALGO_DEFAULT_VENDOR_CAMERA "uvc")
    set(DYNALGO_DEFAULT_VENDOR_ENCODER "ffmpeg")
    set(DYNALGO_DEFAULT_VENDOR_INFERENCE "tensorrt")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    # Detect NVIDIA vs generic ARM
    if(EXISTS "/etc/nv_tegra_release" OR EXISTS "/proc/device-tree/compatible")
        set(DYNALGO_PLATFORM "aarch64_nvidia" CACHE STRING "Target platform")
        set(DYNALGO_DEFAULT_VENDOR_CAMERA "nvsipl")
        set(DYNALGO_DEFAULT_VENDOR_ENCODER "nvmedia")
        set(DYNALGO_DEFAULT_VENDOR_INFERENCE "tensorrt")
    else()
        set(DYNALGO_PLATFORM "aarch64_generic" CACHE STRING "Target platform")
        set(DYNALGO_DEFAULT_VENDOR_CAMERA "v4l2")
        set(DYNALGO_DEFAULT_VENDOR_ENCODER "ffmpeg")
        set(DYNALGO_DEFAULT_VENDOR_INFERENCE "onnxruntime")
    endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "mips")
    set(DYNALGO_PLATFORM "mips64_loongson" CACHE STRING "Target platform")
    set(DYNALGO_DEFAULT_VENDOR_CAMERA "v4l2")
    set(DYNALGO_DEFAULT_VENDOR_ENCODER "loongson_venc")
    set(DYNALGO_DEFAULT_VENDOR_INFERENCE "loongson_npu")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "riscv")
    set(DYNALGO_PLATFORM "riscv64_starfive" CACHE STRING "Target platform")
    set(DYNALGO_DEFAULT_VENDOR_CAMERA "v4l2")
    set(DYNALGO_DEFAULT_VENDOR_ENCODER "starfive_venc")
    set(DYNALGO_DEFAULT_VENDOR_INFERENCE "thead_npu")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Vendor overrides (can be set via command line)
set(DYNALGO_VENDOR_CAMERA "${DYNALGO_DEFAULT_VENDOR_CAMERA}" CACHE STRING "Camera vendor")
set(DYNALGO_VENDOR_ENCODER "${DYNALGO_DEFAULT_VENDOR_ENCODER}" CACHE STRING "Encoder vendor")
set(DYNALGO_VENDOR_INFERENCE "${DYNALGO_DEFAULT_VENDOR_INFERENCE}" CACHE STRING "Inference vendor")
set(DYNALGO_VENDOR_DISPLAY "default" CACHE STRING "Display vendor")
set(DYNALGO_VENDOR_ACTUATOR "default" CACHE STRING "Actuator vendor")

message(STATUS "DYNALGO: Platform=${DYNALGO_PLATFORM}")
message(STATUS "DYNALGO: Vendors: camera=${DYNALGO_VENDOR_CAMERA} encoder=${DYNALGO_VENDOR_ENCODER} inference=${DYNALGO_VENDOR_INFERENCE}")

# Include platform config
include(cmake/platforms/${DYNALGO_PLATFORM}.cmake)

# === HAL LAYER ===
add_subdirectory(hal)

# === APP LAYER (depends only on HAL headers) ===
add_subdirectory(app)

# === BSP (configs only, no build) ===
# add_subdirectory(bsp)  # Optional: install configs

# === TESTS ===
option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
```

### 6.2 Platform CMake / 平台 CMake

```cmake
# cmake/platforms/aarch64_nvidia.cmake
# Platform-specific flags, dependencies, defaults

# Compiler flags for Orin (Ampere + ARMv8.2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a -mtune=cortex-a78ae")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8.2-a -mtune=cortex-a78ae")

# Find NVIDIA SDK components
find_package(CUDA 12.2 REQUIRED)
set(CUDA_ARCHITECTURES "87" CACHE STRING "Orin compute capability")

find_library(NVSIPL_LIB nvsipl PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVMEDIA_LIB nvmedia PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVMEDIA_IEP_LIB nvmedia_iep_sci PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVSCIBUF_LIB nvscibuf PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVSCISYNC_LIB nvscisync PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVSCISTREAM_LIB nvscistream PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVSCIEVENT_LIB nvscievent PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVMEDIA_2D_LIB nvmedia2d PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(TEGRA_WFD_LIB tegrawfd PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(CUDLA_LIB cudla PATHS /usr/lib/aarch64-linux-gnu/tegra)

# TensorRT
find_package(TensorRT 8.6 REQUIRED)

# Platform compile definitions
add_compile_definitions(
    DYNALGO_PLATFORM_AARCH64_NVIDIA=1
    DYNALGO_HAVE_NVSIPL=1
    DYNALGO_HAVE_NVMEDIA=1
    DYNALGO_HAVE_NVSCIBUF=1
    DYNALGO_HAVE_NVSCISYNC=1
    DYNALGO_HAVE_CUDLA=1
)

# Set HAL vendor library names
set(HAL_CAMERA_LIB "dynalgo_hal_camera_nvsipl")
set(HAL_ENCODER_LIB "dynalgo_hal_encoder_nvmedia")
set(HAL_INFERENCE_LIB "dynalgo_hal_inference_tensorrt")
set(HAL_DISPLAY_LIB "dynalgo_hal_display_nvdisplay")
set(HAL_ACTUATOR_LIB "dynalgo_hal_actuator_gpio")
```

### 6.3 HAL CMake / HAL 层 CMake

```cmake
# hal/CMakeLists.txt
# Build selected vendor implementations as shared libraries

# Camera HAL
add_subdirectory(${DYNALGO_PLATFORM}/camera/${DYNALGO_VENDOR_CAMERA})

# Encoder HAL
add_subdirectory(${DYNALGO_PLATFORM}/encoder/${DYNALGO_VENDOR_ENCODER})

# Inference HAL
add_subdirectory(${DYNALGO_PLATFORM}/inference/${DYNALGO_VENDOR_INFERENCE})

# Display HAL
if(DYNALGO_VENDOR_DISPLAY STREQUAL "default")
    # Select platform default
    if(DYNALGO_PLATFORM STREQUAL "aarch64_nvidia")
        set(DYNALGO_VENDOR_DISPLAY "nvdisplay")
    elseif(DYNALGO_PLATFORM STREQUAL "x86_64")
        set(DYNALGO_VENDOR_DISPLAY "sdl2")
    endif()
endif()
add_subdirectory(${DYNALGO_PLATFORM}/display/${DYNALGO_VENDOR_DISPLAY})

# Actuator HAL
if(DYNALGO_VENDOR_ACTUATOR STREQUAL "default")
    if(DYNALGO_PLATFORM STREQUAL "aarch64_nvidia")
        set(DYNALGO_VENDOR_ACTUATOR "gpio")
    elseif(DYNALGO_PLATFORM STREQUAL "x86_64")
        set(DYNALGO_VENDOR_ACTUATOR "generic_serial")
    endif()
endif()
add_subdirectory(${DYNALGO_PLATFORM}/actuator/${DYNALGO_VENDOR_ACTUATOR})

# Install HAL headers
install(DIRECTORY ../include/dynalgo/hal/
        DESTINATION include/dynalgo/hal
        FILES_MATCHING PATTERN "*.hpp")

# Install HAL libraries
install(TARGETS ${HAL_CAMERA_LIB} ${HAL_ENCODER_LIB} ${HAL_INFERENCE_LIB}
                ${HAL_DISPLAY_LIB} ${HAL_ACTUATOR_LIB}
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
```

### 6.4 Vendor HAL CMake Example / 厂商 HAL CMake 示例

```cmake
# hal/aarch64_nvidia/camera/nvsipl/CMakeLists.txt
cmake_minimum_required(VERSION 3.20)

# NvSIPL Camera HAL shared library
add_library(dynalgo_hal_camera_nvsipl SHARED
    nvsipl_camera_hal.cpp
    nvsipl_producer.cpp
    nvsipl_consumer_cuda.cpp
    nvsipl_consumer_enc.cpp
    nvsipl_consumer_display.cpp
    nvsipl_platform_cfg.cpp
)

target_include_directories(dynalgo_hal_camera_nvsipl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${NVSIPL_INCLUDE_DIRS}
    ${NVMEDIA_INCLUDE_DIRS}
    ${NVSCIBUF_INCLUDE_DIRS}
    ${NVSCISYNC_INCLUDE_DIRS}
    ${NVSCISTREAM_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ../../../../include  # For dynalgo::hal types
)

target_link_libraries(dynalgo_hal_camera_nvsipl PRIVATE
    ${NVSIPL_LIB}
    ${NVMEDIA_LIB}
    ${NVMEDIA_IEP_LIB}
    ${NVSCIBUF_LIB}
    ${NVSCISYNC_LIB}
    ${NVSCISTREAM_LIB}
    ${NVSCIEVENT_LIB}
    ${NVMEDIA_2D_LIB}
    ${TEGRA_WFD_LIB}
    ${CUDA_LIBRARIES}
    ${CUDLA_LIB}
    dynalgo_hal_common  # Common HAL utilities
)

# Version & ABI
set_target_properties(dynalgo_hal_camera_nvsipl PROPERTIES
    VERSION ${DYNALGO_VERSION}
    SOVERSION ${HAL_VERSION_MAJOR}
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

# Install
install(TARGETS dynalgo_hal_camera_nvsipl
        LIBRARY DESTINATION lib/dynalgo/hal
        RUNTIME DESTINATION bin)
```

---

## 7. Application Layer Usage / 应用层使用

### 7.1 HAL Factory / HAL 工厂

```cpp
// app/core/dynalgo_hal_factory.cpp
#include <dynalgo/hal/camera_hal.hpp>
#include <dynalgo/hal/encoder_hal.hpp>
#include <dynalgo/hal/inference_hal.hpp>
#include <dynalgo/hal/display_hal.hpp>
#include <dynalgo/hal/actuator_hal.hpp>
#include <dynalgo/config/platform_config.hpp>
#include <dlfcn.h>  // Dynamic loading

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

        // Load camera HAL
        auto cam_result = loadCameraHAL(platform_cfg, board_cfg);
        if (!cam_result.is_ok()) return Result<HALBundle>::err(cam_result.error());
        bundle.camera = std::move(cam_result.value());

        // Load encoder HAL (optional)
        if (platform_cfg.encoder.enabled) {
            auto enc_result = loadEncoderHAL(platform_cfg);
            if (enc_result.is_ok()) bundle.encoder = std::move(enc_result.value());
        }

        // Load inference HAL (optional)
        if (platform_cfg.inference.enabled) {
            auto inf_result = loadInferenceHAL(platform_cfg);
            if (inf_result.is_ok()) bundle.inference = std::move(inf_result.value());
        }

        // ... display, actuator

        return Result<HALBundle>::ok(std::move(bundle));
    }

private:
    static Result<std::unique_ptr<hal::ICameraHAL>> loadCameraHAL(
        const PlatformConfig& platform_cfg, const BoardConfig& board_cfg) {

        std::string vendor = platform_cfg.getVendor("camera");
        std::string lib_name = "libdynalgo_hal_camera_" + vendor + ".so";

        void* handle = dlopen(lib_name.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (!handle) {
            return Result<std::unique_ptr<hal::ICameraHAL>>::err(
                static_cast<uint32_t>(hal::ErrorCode::NOT_SUPPORTED));
        }

        using CreateFunc = hal::ICameraHAL* (*)();
        using DestroyFunc = void (*)(hal::ICameraHAL*);

        CreateFunc create = reinterpret_cast<CreateFunc>(dlsym(handle, "dynalgo_hal_camera_create"));
        DestroyFunc destroy = reinterpret_cast<DestroyFunc>(dlsym(handle, "dynalgo_hal_camera_destroy"));

        if (!create || !destroy) {
            dlclose(handle);
            return Result<std::unique_ptr<hal::ICameraHAL>>::err(
                static_cast<uint32_t>(hal::ErrorCode::NOT_SUPPORTED));
        }

        auto hal = std::unique_ptr<hal::ICameraHAL>(create(),
            [handle, destroy](hal::ICameraHAL* ptr) {
                destroy(ptr);
                dlclose(handle);
            });

        // Initialize with board config
        hal::CameraConfig cfg = board_cfg.getCameraConfig(0);  // First camera
        auto init_result = hal->initialize(cfg);
        if (!init_result.is_ok()) {
            return Result<std::unique_ptr<hal::ICameraHAL>>::err(init_result.error());
        }

        return Result<std::unique_ptr<hal::ICameraHAL>>::ok(std::move(hal));
    }
};

} // namespace dynalgo
```

### 7.2 Application Code (HAL-only) / 应用代码

```cpp
// app/core/dynalgo_capture_session.cpp
#include <dynalgo/hal/camera_hal.hpp>
#include <dynalgo/hal/encoder_hal.hpp>
#include <dynalgo/hal/inference_hal.hpp>
#include <dynalgo/core/dynalgo_frame_manager.hpp>

namespace dynalgo {

class CaptureSession {
public:
    CaptureSession(const HALFactory::HALBundle& hal) : hal_(hal) {}

    bool start() {
        // Start camera
        auto result = hal_.camera->start();
        if (!result.is_ok()) {
            LOG_ERROR("Camera start failed: {}", result.error());
            return false;
        }

        // Start encoder if available
        if (hal_.encoder) {
            hal_.encoder->start();
        }

        // Start inference if available
        if (hal_.inference) {
            hal_.inference->initialize(model_config_);
        }

        running_ = true;
        capture_thread_ = std::thread(&CaptureSession::captureLoop, this);
        return true;
    }

    void stop() {
        running_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
        hal_.camera->stop();
        if (hal_.encoder) hal_.encoder->stop();
    }

private:
    void captureLoop() {
        while (running_) {
            // Acquire frame (blocking, with timeout)
            auto frame_result = hal_.camera->acquireFrame(33);  // ~30fps
            if (!frame_result.is_ok()) {
                if (frame_result.error() == static_cast<uint32_t>(hal::ErrorCode::TIMEOUT)) {
                    continue;
                }
                LOG_ERROR("Acquire frame failed: {}", frame_result.error());
                break;
            }

            FrameMetadata frame = std::move(frame_result.value());

            // Process frame through pipeline
            processFrame(frame);

            // Release buffer back to HAL
            hal_.camera->releaseBuffer(frame.buffer);
        }
    }

    void processFrame(const FrameMetadata& frame) {
        // 1. Encode (async)
        if (hal_.encoder) {
            hal::BitstreamBuffer bitstream;
            hal_.encoder->encodeFrame(frame, bitstream);
            // Handle bitstream (stream, record)
        }

        // 2. Inference (async)
        if (hal_.inference) {
            hal::TensorInfo input;
            input.buffer = frame.buffer;
            input.format = frame.format;
            input.shape = {1, frame.height, frame.width, 3};  // NHWC

            hal_.inference->inferAsync({input},
                [this, frame](hal::InferenceResult result) {
                    // Post-process detections
                    onInferenceDone(frame, result);
                });
        }

        // 3. Display
        if (hal_.display) {
            hal_.display->presentFrame(frame);
        }
    }

    void onInferenceDone(const FrameMetadata& frame, const hal::InferenceResult& result) {
        // Convert to app types, run tracking, engagement loop
        // ...
    }

    HALFactory::HALBundle hal_;
    std::thread capture_thread_;
    std::atomic<bool> running_{false};
    hal::ModelConfig model_config_;
};

} // namespace dynalgo
```

---

## 8. Deployment & Delivery / 部署与交付

### 8.1 Release Package Structure / 发布包结构

```
dynalgo-cam-2.0.0-linux-aarch64/
├── bin/
│   ├── dynamic_algo_cam              # Main executable
│   ├── calibration_tool
│   └── benchmark_tool
├── lib/
│   ├── libdynalgo_core.so            # App core
│   ├── libdynalgo_algo.so            # Algorithms
│   └── hal/                          # HAL plugins (loaded at runtime)
│       ├── libdynalgo_hal_camera_nvsipl.so
│       ├── libdynalgo_hal_encoder_nvmedia.so
│       ├── libdynalgo_hal_inference_tensorrt.so
│       ├── libdynalgo_hal_display_nvdisplay.so
│       └── libdynalgo_hal_actuator_gpio.so
├── config/
│   ├── platform.yaml                 # Platform config (selected at deploy)
│   ├── board.yaml                    # Board config (selected at deploy)
│   └── vendor/                       # Vendor configs
│       ├── nvsipl/imx728.yaml
│       └── ...
├── calibration/
│   ├── imx728_front.yml
│   └── ...
├── scripts/
│   ├── install.sh
│   ├── uninstall.sh
│   └── run_dynalgo.sh
├── systemd/
│   └── dynalgo-cam.service
├── docs/
│   ├── RELEASE_NOTES.md
│   ├── USER_GUIDE.md
│   └── API_REFERENCE.md
└── manifest.json                     # Version, checksums, dependencies
```

### 8.2 manifest.json

```json
{
  "package": "dynalgo-cam",
  "version": "2.0.0",
  "platform": "aarch64_nvidia",
  "soc": "orin",
  "build_date": "2026-09-08",
  "git_commit": "abc1234",
  "hal_version": "1.0",
  "dependencies": {
    "cuda": "12.2",
    "tensorrt": "8.6",
    "nvsipl": "6.0.8",
    "nvmedia": "6.0.8",
    "nvscibuf": "1.0",
    "nvscisync": "1.0",
    "nvscistream": "1.0"
  },
  "hal_plugins": {
    "camera": "nvsipl",
    "encoder": "nvmedia",
    "inference": "tensorrt",
    "display": "nvdisplay",
    "actuator": "gpio"
  },
  "checksums": {
    "bin/dynamic_algo_cam": "sha256:...",
    "lib/hal/libdynalgo_hal_camera_nvsipl.so": "sha256:..."
  }
}
```

### 8.3 Installation Script / 安装脚本

```bash
#!/bin/bash
# scripts/install.sh

set -e

INSTALL_PREFIX="/opt/dynalgo"
PLATFORM=$(cat /etc/dynalgo_platform 2>/dev/null || echo "auto")

# Detect platform if auto
if [ "$PLATFORM" = "auto" ]; then
    ARCH=$(uname -m)
    if [ "$ARCH" = "aarch64" ] && [ -f /etc/nv_tegra_release ]; then
        PLATFORM="aarch64_nvidia"
    elif [ "$ARCH" = "x86_64" ]; then
        PLATFORM="x86_64"
    else
        echo "Unsupported platform: $ARCH"
        exit 1
    fi
fi

echo "Installing DynamicAlgoCam for platform: $PLATFORM"

# Create directories
mkdir -p $INSTALL_PREFIX/{bin,lib/hal,config/board,config/vendor,calibration,logs}

# Copy binaries
cp bin/* $INSTALL_PREFIX/bin/
cp lib/*.so $INSTALL_PREFIX/lib/
cp lib/hal/*.so $INSTALL_PREFIX/lib/hal/

# Select board config
if [ -f "config/boards/${BOARD}.yaml" ]; then
    cp "config/boards/${BOARD}.yaml" $INSTALL_PREFIX/config/board.yaml
else
    echo "Warning: No board config for $BOARD, using generic"
    cp "config/platforms/${PLATFORM}.yaml" $INSTALL_PREFIX/config/board.yaml
fi

# Copy platform config
cp "config/platforms/${PLATFORM}.yaml" $INSTALL_PREFIX/config/platform.yaml

# Copy vendor configs
cp -r config/vendor/* $INSTALL_PREFIX/config/vendor/

# Copy calibration data
cp calibration/* $INSTALL_PREFIX/calibration/

# Install systemd service
cp systemd/dynalgo-cam.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable dynalgo-cam

# Set up udev rules for cameras
cp scripts/99-dynalgo-cameras.rules /etc/udev/rules.d/
udevadm control --reload-rules

echo "Installation complete. Run: systemctl start dynalgo-cam"
```

---

## 9. Safety & Certification Considerations / 安全与认证考量

| Aspect / 方面 | Requirement / 要求 | Implementation / 实现 |
|---|---|---|
| **Memory Safety** | No dynamic alloc in hot path / 热路径零动态分配 | Pre-allocated buffer pools, `std::array` |
| **Determinism** | Bounded worst-case latency / 有界最坏延迟 | Real-time scheduling (SCHED_FIFO), CPU affinity |
| **Fault Containment** | Single HAL failure ≠ system crash / 单 HAL 故障不崩溃 | Process isolation (separate processes per HAL) |
| **Watchdog** | HW watchdog for each pipeline / 每管线硬件看门狗 | NvSIPL watchdog + app-level heartbeat |
| **Config Validation** | Schema validation at startup / 启动时模式验证 | JSON Schema / YAML validation |
| **Audit Trail** | Structured logging for traceability / 结构化日志追踪 | `ILoggerHAL` with severity, component, timestamp |
| **Secure Boot** | Signed HAL plugins / 签名 HAL 插件 | `dlopen` with signature verification |

---

## 10. Migration Roadmap / 迁移路线图

| Phase / 阶段 | Duration / 周期 | Scope / 范围 | Deliverable / 交付 |
|---|---|---|---|
| **0. Foundation** | 2 weeks | HAL type system, error codes, buffer/sync abstractions | `include/dynalgo/hal/`, `hal/common/` |
| **1. x86 HAL Port** | 3 weeks | Move existing x86 drivers to HAL plugins | `hal/x86_64/` (uvc, orbbec, robosense, stereo) |
| **2. ARM/NVIDIA HAL** | 4 weeks | NvSIPL, NvMedia, TensorRT, CUDLA HAL plugins | `hal/aarch64_nvidia/` (7+ modules) |
| **3. App Refactor** | 2 weeks | `CaptureSession`, `PipelineManager` using HAL only | `app/core/` refactored |
| **4. Config System** | 2 weeks | Platform/Vendor/Board YAML, validation, loader | `bsp/configs/`, `app/core/dynalgo_config_manager.cpp` |
| **5. MIPS/RISC-V Skeleton** | 2 weeks | Directory structure, stub HALs, CI integration | `hal/mips64_loongson/`, `hal/riscv64_starfive/` |
| **6. Build/Deploy** | 2 weeks | Cross-compile, packaging, systemd, OTA | `scripts/`, `manifest.json`, Docker |
| **7. Test/Cert** | 3 weeks | Unit/integration/perf tests, safety analysis | `tests/`, safety case docs |
| **8. Documentation** | 1 week | HAL spec, porting guides, API reference | `docs/architecture/` |

**Total: ~21 weeks for full multi-platform industrial release**

---

## Phase 1 Implementation Status / 第一阶段实现状态 (✅ COMPLETED / 已完成)

| Task / 任务 | Status / 状态 | Files / 文件 | Build Verified / 构建验证 |
|---|---|---|---|
| HAL Directory Structure | ✅ Done | `include/dynalgo/hal/`, `hal/common/`, `hal/x86_64/`, `hal/aarch64_nvidia/` | ✅ |
| `hal_types.hpp` - Core Types | ✅ Done | `Result<T>`, `ErrorCode`, `BufferHandle`, `FenceHandle`, `PixelFormat`, `FrameMetadata` | ✅ |
| `camera_hal.hpp` | ✅ Done | `ICameraHAL`, `CameraConfig`, `MultiCameraConfig`, `CameraHALFactory` | ✅ |
| `encoder_hal.hpp` | ✅ Done | `IEncoderHAL`, `EncodeConfig`, `BitstreamBuffer` | ✅ |
| `inference_hal.hpp` | ✅ Done | `IInferenceHAL`, `ModelConfig`, `TensorInfo`, `InferenceResult` | ✅ |
| `display_hal.hpp` | ✅ Done | `IDisplayHAL`, `DisplayConfig`, `PresentConfig` | ✅ |
| `actuator_hal.hpp` | ✅ Done | `IActuatorHAL`, `ActuatorConfig`, `ActuatorCommand` | ✅ |
| `sensor_hal.hpp` | ✅ Done | `ISensorHAL`, `SensorConfig`, `SensorData` (std::variant) | ✅ |
| `logger_hal.hpp` | ✅ Done | `ILoggerHAL`, `LogConfig`, `LogDestination` (bitwise ops) | ✅ |
| `time_hal.hpp` | ✅ Done | `ITimeHAL`, `TimeConfig`, `PTPConfig` | ✅ |
| HAL Common (`buffer_manager`, `sync_manager`, `plugin_loader`, `hal_base`) | ✅ Done | Zero-copy buffer pool, sync primitives, dynamic plugin loading | ✅ |
| Platform CMake Configs (x86_64, aarch64_nvidia, aarch64_generic, mips64_loongson, riscv64_starfive) | ✅ Done | Auto-detection, vendor defaults, dependency find | ✅ |
| HAL CMakeLists.txt | ✅ Done | Auto platform detection, vendor selection, shared lib builds | ✅ |
| x86_64 Vendor Stubs (uvc, orbbec, robosense, ffmpeg, vaapi, tensorrt, sdl2, generic_serial, sensor, spdlog, chrono) | ✅ Done | All 9 HAL types with stub implementations | ✅ **BUILT** |
| aarch64_nvidia Vendor Stubs (nvsipl, v4l2, nvmedia, nvenc, tensorrt, cudla, nvdisplay, gpio, iio, spdlog, ptp) | ✅ Done | All 9 HAL types with stub implementations | ✅ |

**Build Verification**: All 9 x86_64 HAL targets built successfully:
- `dynalgo_hal_common` (static)
- `dynalgo_hal_camera_uvc`
- `dynalgo_hal_encoder_ffmpeg`
- `dynalgo_hal_inference_tensorrt`
- `dynalgo_hal_display_sdl2`
- `dynalgo_hal_actuator_generic_serial`
- `dynalgo_hal_sensor_generic`
- `dynalgo_hal_logger_spdlog`
- `dynalgo_hal_time_chrono`

---

## 11. Appendix: Vendor Plugin Interface / 附录：厂商插件接口

Each HAL vendor plugin exports **exactly two C symbols**:

```cpp
// In each vendor HAL implementation (e.g., nvsipl_camera_hal.cpp)
#include <dynalgo/hal/camera_hal.hpp>

extern "C" {

// Create instance
dynalgo::hal::ICameraHAL* dynalgo_hal_camera_create() {
    return new dynalgo::NvSIPLCameraHAL();
}

// Destroy instance
void dynalgo_hal_camera_destroy(dynalgo::hal::ICameraHAL* ptr) {
    delete ptr;
}

} // extern "C"
```

**Plugin Loading**: `HALFactory` uses `dlopen`/`dlsym` → zero app dependency on vendor code.
**插件加载**：`HALFactory` 使用 `dlopen`/`dlsym` → 应用零依赖厂商代码。

---

## 12. Glossary / 术语表

| Term / 术语 | Definition / 定义 |
|---|---|
| **HAL** | Hardware Abstraction Layer — stable interface between App and platform / 硬件抽象层 |
| **CS** | Chip Support — platform-specific HAL implementations / 芯片支持包 |
| **BSP** | Board Support Package — board-level config (pinmux, calibration, power) / 板级支持包 |
| **Platform** | SoC architecture class (x86_64, aarch64_nvidia, mips64_loongson, riscv64_starfive) / 平台 |
| **Vendor** | Silicon/IP vendor providing HAL implementation (nvidia, orbbec, robosense, generic) / 厂商 |
| **BufferHandle** | Opaque cross-platform buffer reference (NvSciBufObj, dma-buf fd, V4L2 index) / 缓冲句柄 |
| **FenceHandle** | Opaque synchronization primitive (NvSciSyncFence, sync_file fd) / 栅栏句柄 |
| **Zero-Copy** | Buffer ownership transfer without data copy / 零拷贝（所有权转移无数据拷贝） |

---

*Document Version: 2.0 | Last Updated: 2026-09-08 | Classification: Internal - Design Specification*