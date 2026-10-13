# DynamicAlgoCam NVIDIA ARM Platform Porting Plan

## Executive Summary

Extend DynamicAlgoCam from x86-only (UVC/V4L2, Orbbec, RoboSense) to support NVIDIA ARM platforms (Jetson Orin, Drive Orin) using **NvSIPL** (NVIDIA Safety Image Processing Library) for camera pipeline, **NvSciBuf/NvSciSync** for zero-copy IPC, **CUDA** for acceleration, and **NvMedia** for encoding.

**Target Platforms**: Jetson AGX Orin, Orin NX, Orin Nano, Drive Orin (aarch64, Linux for Tegra / DriveOS)
**Source Reference**: `/HDD/jtshen/github/nvsipl-multicast-6.0.8.0/multicast/` (NvSIPL 6.0.8 multicast sample)

---

## Architecture Comparison

| Component | DynamicAlgoCam (x86) | NVIDIA ARM (NvSIPL) |
|-----------|---------------------|---------------------|
| **Camera Interface** | UVC/V4L2, OrbbecSDK, rs_driver | NvSIPL (INvSIPLCamera, INvSIPLClient) |
| **ISP** | Host CPU / vendor SDK | NVIDIA HW ISP (ICP/ISP/ACP/CDI) |
| **Buffer Management** | `std::vector<uint8_t>`, V4L2 buffers | NvSciBuf (NvSciBufObj, NvSciBufAttrList) |
| **Synchronization** | `std::mutex`, condition variables | NvSciSync (NvSciSyncObj, NvSciSyncFence) |
| **IPC/Streaming** | FFmpeg, SDL2, custom | NvSciStream (multi-process, zero-copy) |
| **GPU Acceleration** | Optional CUDA (ENABLE_CUDA) | Native CUDA + CUDLA (DLA) |
| **Video Encoding** | FFmpeg (SW) | NvMedia IEP (HW H.264/H.265) |
| **Inference** | TensorRT/ONNX (x86) | TensorRT + CUDLA (aarch64) |
| **Sensor Config** | Runtime USB descriptors | PlatformCfg (JSON), CameraModuleInfo |

---

## Phase 1: Build System & Platform Detection (Week 1-2) ✅ COMPLETED

### 1.1 CMake Platform Detection
```cmake
# In root CMakeLists.txt
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    set(DYNALGO_PLATFORM_ARM64 ON)
    # Detect L4T / DriveOS
    if(EXISTS "/etc/nv_tegra_release")
        set(DYNALGO_PLATFORM_JETSON ON)
        file(READ "/etc/nv_tegra_release" L4T_VERSION)
        string(REGEX MATCH "R([0-9]+)" _ ${L4T_VERSION})
        set(L4T_MAJOR ${CMAKE_MATCH_1})
    elseif(EXISTS "/etc/driveos-release")
        set(DYNALGO_PLATFORM_DRIVE ON)
    endif()
endif()

option(ENABLE_NVSIPL "Enable NVIDIA SIPL camera support (ARM64 only)" ${DYNALGO_PLATFORM_ARM64})
option(ENABLE_NVMEDIA "Enable NvMedia encode/decode (ARM64 only)" ${DYNALGO_PLATFORM_ARM64})
option(ENABLE_CUDLA "Enable CUDLA/DLA acceleration (ARM64 only)" ${DYNALGO_PLATFORM_ARM64})
```

### 1.2 Cross-Compilation Toolchain
```cmake
# cmake/toolchains/aarch64_nvidia.cmake (IMPLEMENTED)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# DriveOS SDK paths (container: /drive/drive-linux/)
set(CMAKE_SYSROOT /drive/drive-linux)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.4)

# Compiler flags for Orin (Ampere + ARMv8.2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a -mtune=cortex-a76")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8.2-a -mtune=cortex-a76")
```

### 1.3 Dependency Discovery ✅ IMPLEMENTED
```cmake
# cmake/platforms/aarch64_nvidia.cmake
# NVIDIA SDK libraries (DriveOS / JetPack)
set(NVIDIA_SDK_PATHS
    /drive/drive-linux/lib-target
    /drive/drive-linux/lib
    /usr/lib/aarch64-linux-gnu/tegra
    /usr/lib/aarch64-linux-gnu
    /usr/local/lib
    /opt/nvidia/driveos
)

# Include directories
include_directories(/drive/drive-linux/include)

# NvSIPL & NvMedia
find_library(NVSIPL_LIB nvsipl PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSCIBUF_LIB nvscibuf PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSCISYNC_LIB nvscisync PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSCISTREAM_LIB nvscistream PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSCIEVENT_LIB nvscievent PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSICIPC_LIB nvsciipc PATHS ${NVIDIA_SDK_PATHS})
find_library(NVSCICOMMON_LIB nvsciCommon PATHS ${NVIDIA_SDK_PATHS})
find_library(TEGRA_WFD_LIB tegrawfd PATHS ${NVIDIA_SDK_PATHS})
find_library(CUDLA_LIB nvcudla PATHS ${NVIDIA_SDK_PATHS})
find_library(NVML_LIB nvidia-ml PATHS ${NVIDIA_SDK_PATHS})

# NvMedia component libraries (no single libnvmedia.so in container)
find_library(NVMEDIA_2D_LIB nvmedia2d PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_IEP_LIB nvmedia_iep_sci PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_DLA_LIB nvmedia_dla PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_EGLSTREAM_LIB nvmedia_eglstream PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_IDE_PARSER_LIB nvmedia_ide_parser PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_IDE_SCI_LIB nvmedia_ide_sci PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_IJPD_SCI_LIB nvmedia_ijpd_sci PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_IJPE_SCI_LIB nvmedia_ijpe_sci PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_IOFA_SCI_LIB nvmedia_iofa_sci PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_TENSOR_LIB nvmedia_tensor PATHS ${NVIDIA_SDK_PATHS})
find_library(NVMEDIA_LDC_LIB nvmedialdc PATHS ${NVIDIA_SDK_PATHS})
set(NVMEDIA_LIBS ${NVMEDIA_2D_LIB} ${NVMEDIA_IEP_LIB} ${NVMEDIA_DLA_LIB} ...)
```

### 1.4 Docker Build Environment ✅ VALIDATED
```bash
# Container: harbor-ad2.nioint.com/nad/conandocker:v6.0.7.0-0.0.31
# Entrypoint: /bin/bash (must override: --entrypoint /bin/bash -c "cmd")
# Cross-compiler: aarch64-linux-gnu-g++ 9.4.0
# CUDA: 11.4 at /usr/local/cuda-11.4
# Conan profiles: orin, nt2_ccc, nt2_mazu (armv8, gcc 9.3, toolchain-orin/6.0.5.0)

docker run --rm --entrypoint /bin/bash \
  -v /HDD/jtshen/github/DynamicAlgoCam:/workspace \
  harbor-ad2.nioint.com/nad/conandocker:v6.0.7.0-0.0.31 -c "
    cd /workspace/hal/aarch64_nvidia/camera/nvsipl/build &&
    cmake -DCMAKE_TOOLCHAIN_FILE=/workspace/cmake/platforms/aarch64_nvidia.cmake .. &&
    make -j4
  "
```

---

## Phase 2: NvSIPL HAL Implementation (Week 2-4) ✅ COMPLETED

### 2.1 Directory Structure (HAL-based, not app/driver/)
```
hal/
├── aarch64_nvidia/
│   ├── CMakeLists.txt
│   ├── camera/
│   │   └── nvsipl/                    # VENDOR: NVIDIA SIPL (primary)
│   │       ├── CMakeLists.txt
│   │       ├── nvsipl_camera_hal.cpp  # ICameraHAL implementation
│   │       ├── nvsipl_platform_cfg.cpp # PlatformCfg parser (NIO/AD camera.json)
│   │       ├── third_party/
│   │       │   └── nlohmann_json.hpp  # Vendored JSON parser (no nlohmann_json dep)
│   │       └── nvsipl_platform_cfg.hpp
```

### 2.2 HAL Interface Compliance ✅ IMPLEMENTED
```cpp
// hal/aarch64_nvidia/camera/nvsipl/nvsipl_camera_hal.cpp
// Implements: dynalgo::hal::ICameraHAL (stable ABI from include/dynalgo/hal/camera_hal.hpp)

class NvSIPLCameraHAL : public dynalgo::hal::ICameraHAL {
public:
    // Capability query
    Result<std::vector<StreamConfig>> getSupportedStreams(device_id) override;
    Result<SensorInfo> getSensorInfo(device_id) override;
    
    // Lifecycle
    ResultVoid initialize(const CameraConfig& config) override;      // SetPlatformCfg + Init
    ResultVoid start() override;                                      // INvSIPLCamera::Start()
    ResultVoid stop() override;                                       // INvSIPLCamera::Stop()
    ResultVoid deinitialize() override;                               // Deinit + cleanup
    
    // Frame acquisition (blocking + async)
    Result<FrameMetadata> acquireFrame(timeout_ms) override;          // Ring buffer
    Result<std::vector<FrameMetadata>> acquireFrames(timeout_ms) override;
    ResultVoid registerCallback(FrameCallback cb) override;           // Async mode
    ResultVoid unregisterCallback() override;
    
    // Buffer management (zero-copy via NvSciBuf)
    Result<BufferHandle> importBuffer(external) override;             // NvSciBufObj handle
    ResultVoid releaseBuffer(buffer) override;
    
    // Controls (via NvSIPL auto-control plugins)
    ResultVoid setControl(device_id, CameraControl, value) override;
    Result<int64_t> getControl(device_id, CameraControl) override;
    
    // Platform info
    Result<std::string> getName() const override { return "NvSIPL Camera HAL"; }
    Result<std::string> getVendor() const override { return "nvidia"; }
    Result<std::string> getVersion() const override { return "6.0.8"; }
};

// Factory registration
extern "C" {
dynalgo::hal::ICameraHAL* dynalgo_hal_camera_create() { return new dynalgo::NvSIPLCameraHAL(); }
void dynalgo_hal_camera_destroy(dynalgo::hal::ICameraHAL* ptr) { delete ptr; }
}
```

### 2.3 SIPL Camera Wrapper (`nvsipl_camera_hal.cpp`) ✅
- Wraps `nvsipl::INvSIPLCamera` from NvSIPL SDK
- Implements `ICameraHAL` interface
- Handles `PlatformCfg` loading (NIO/AD JSON format), sensor enumeration
- Manages frame queues via `INvSIPLFrameCompletionQueue` (ICP/ISP0/ISP1)
- Pipeline notifications via `INvSIPLNotificationQueue` (ICP/ISP/ACP/CDI events)
- Callback → `FrameMetadata` conversion using NvSciBuf CPU mapping
- Thread-safe frame ring buffer with condition variables

### 2.4 Frame Queue Handlers ✅
```cpp
// PipelineFrameQueueHandler - processes ICP/ISP0/ISP1 frame completion queues
SIPLStatus Init(sensor_id, queues, FrameCallback);
void Deinit();
ThreadFunc() {
    // Get buffers from INvSIPLFrameCompletionQueue::Get(INvSIPLBuffer*&, timeout)
    // Convert INvSIPLNvMBuffer → FrameMetadata (NvSciBufObj handle + metadata)
    // Callback(user) + Release() buffers
}

// PipelineNotificationHandler - processes pipeline events
NotificationType values from NvSIPLPipelineNotifier:
  NOTIF_INFO_ICP_PROCESSING_DONE (0)
  NOTIF_INFO_ISP_PROCESSING_DONE (1)
  NOTIF_INFO_ACP_PROCESSING_DONE (2)
  NOTIF_INFO_CDI_PROCESSING_DONE (3)
  NOTIF_INFO_ICP_AUTH_SUCCESS (4)
  NOTIF_WARN_ICP_FRAME_DROP (100)
  NOTIF_WARN_ICP_FRAME_DISCONTINUITY (101)
  NOTIF_WARN_ICP_CAPTURE_TIMEOUT (102)
  NOTIF_ERROR_ICP_BAD_INPUT_STREAM (200)
  NOTIF_ERROR_ICP_CAPTURE_FAILURE (201)
  NOTIF_ERROR_ICP_EMB_DATA_PARSE_FAILURE (202)
  NOTIF_ERROR_ISP_PROCESSING_FAILURE (203)
  NOTIF_ERROR_ACP_PROCESSING_FAILURE (204)
  NOTIF_ERROR_CDI_SET_SENSOR_CTRL_FAILURE (205)
  NOTIF_ERROR_INTERNAL_FAILURE (300)
  NOTIF_ERROR_ICP_AUTH_FAILURE (211)
```

---

## Phase 3: NvSIPL Platform Configuration Parser (Week 3) ✅ COMPLETED

### 3.1 NIO/AD camera.json Format Support ✅
```json
// NIO/AD camera.json (12 cameras: Front120, DMS, Front30, Rear, SideView_FL/FR/RL/RR, Park_Right/Left/Rear/Front)
{
  "sensorConfig": [
    {
      "cameraName": "NIO_P02F120A1_20BIT_RGGB_FLIP",
      "sensorProtocol": "camera.gmsl",
      "dataSourceName": "F120",
      "sensorParameters": "camera-name=NIO_P02F120A1_20BIT_RGGB_FLIP,interface=csi-e,deserializer=MAX96712DES1,link=0,output-format=raw+processed,CPHY-mode=1,skip-eeprom=1,fifo-size=4,enable-subframe=1,output-timestamp=raw,disable-subframe=0,slave=0,isp-mode=yuv420-uint8-bl",
      "width": 3840,
      "height": 2160,
      "frameRate": 30,
      "rawImageEnable": 1,
      "encoderEnable": 0,
      "position": "front_120",
      "orientation": 0
    }
  ]
}
```

### 3.2 Parser Implementation (`nvsipl_platform_cfg.cpp`) ✅
```cpp
// Key functions:
static bool ParseFromJson(json_file, device_blocks);           // Entry point
static bool ParseSensorConfigArray(sensor_config_array, blocks); // NIO sensorConfig[]
static std::optional<GmslCameraParams> ParseGmslParams(str);    // key=value CSV parser
static nvsipl::PlatformCfg BuildRealPlatformCfg(device_blocks); // Query + Direct fallback

// NIO Query Flow (preferred for real hardware):
// 1. INvSIPLQuery::GetInstance()
// 2. ParseDatabase()
// 3. GetPlatformCfg("NIO_CCC" or "NIO_MAZU_C_EBD", platform_cfg)
// 4. ApplyMask(platform_cfg, deserializer_masks) - bitwise link enable masks
//    Mask: LINK_0=0x0001, LINK_1=0x0010, LINK_2=0x0100, LINK_3=0x1000

// Direct Construction Fallback:
// Populate PlatformCfg::deviceBlockList with fixed arrays (MAX_DEVICEBLOCKS_PER_PLATFORM=6)
```

### 3.3 GMSL Camera Parameters Mapping ✅
| NIO Parameter | Struct Field | Example |
|---|---|---|
| `camera-name` | `camera_name` | `NIO_P02F120A1_20BIT_RGGB_FLIP` |
| `interface` | `interface` | `csi-e` |
| `deserializer` | `deserializer` | `MAX96712DES1` |
| `link` | `link` | `0` |
| `output-format` | `output_format` | `raw+processed` |
| `CPHY-mode` | `cphy_mode` | `1` |
| `skip-eeprom` | `skip_eeprom` | `true` |
| `fifo-size` | `fifo_size` | `4` |
| `enable-subframe` | `enable_subframe` | `true` |
| `output-timestamp` | `output_timestamp` | `raw` |
| `disable-subframe` | `disable_subframe` | `false` |
| `slave` | `slave_mode` | `false` |
| `isp-mode` | `isp_mode` | `yuv420-uint8-bl` |

---

## Phase 4: Zero-Copy Pipeline Integration (Week 4-5) 📋 PLANNED

### 4.1 NvSciBuf Integration in FrameMetadata
```cpp
// FrameMetadata.buffer.handle = NvSciBufObj (uint64_t)
// FrameMetadata.buffer.platform_id = 0x1 (NVIDIA platform)
// FrameMetadata.acquire_fence = NvSciSyncFence (wait before access)
// FrameMetadata.release_fence = NvSciSyncFence (signal after done)
```

### 4.2 Pipeline Topology
```
[GMSL Sensor] → [MAX96712 Deserializer] → [MIPI CSI-2] → [NvSIPL ICP/ISP]
                                                        │
                    ┌───────────────────────────────────┼───────────────────────────────────┐
                    ▼                                   ▼                                   ▼
              [ICP Raw Bayer]                      [ISP0 NV12]                          [ISP1 NV12]
              (NvSciBufObj)                        (NvSciBufObj)                        (NvSciBufObj)
                    │                                   │                                   │
                    ▼                                   ▼                                   ▼
              [CUDA Consumer]                       [Encoder]                             [Display]
              (cudaImportExternalMemory)            (NvMedia IEP)                        (WFD/DRM)
                    │                                   │                                   │
                    ▼                                   ▼                                   ▼
              [Inference: TensorRT]              [H.264 Stream]                       [HDMI/DP]
```

---

## Phase 5: CUDA/CUDLA Acceleration (Week 5-6) 📋 PLANNED

### 5.1 CUDA Preprocessing Kernels
```cpp
// app/algo/cuda_preprocess.cu
__global__ void nv12_to_rgb_kernel(const uint8_t* y, const uint8_t* uv,
                                   uint8_t* rgb, int width, int height, int pitch);
__global__ void letterbox_resize_kernel(const uint8_t* src, uint8_t* dst,
                                        int srcW, int srcH, int dstW, int dstH);
__global__ void nms_kernel(float* boxes, float* scores, int numBoxes,
                           float iouThresh, float scoreThresh, int* keep, int* keepCount);
```

### 5.2 CUDLA Integration (DLA)
```cpp
// app/model_backends/tensorrt/trt_dla_backend.cpp
class TRTDLABackend : public DynalgoModelBackend {
    // Load TensorRT engine with DLA enabled
    // Builder config: setDefaultDeviceType(kDLA), setDLACore(0)
    // Input: NvSciBufObj → CUDA external memory → TensorRT
};
```

---

## Phase 6: NvMedia Encoder Integration (Week 6) 📋 PLANNED

### 6.1 Encoder Backend
```cpp
// hal/aarch64_nvidia/encoder/nvmedia/nvmedia_encoder.cpp
class NvMediaEncoder : public IEncoderHAL {
    bool init(width, height, fps, bitrate, codec);
    bool encode(NvSciBufObj inputBuf, BitstreamBuffer& output);
    bool flush();
    void deinit();
private:
    NvMediaIEP* m_iep = nullptr;
    NvSciSyncObj m_eofSyncObj = nullptr;
};
```

---

## Phase 7: Configuration & Deployment (Week 7) 📋 PLANNED

### 7.1 Platform Configuration (JSON) ✅ SUPPORTED
```json
// NIO/AD camera.json format (12 cameras)
// Supported cameras: Front120, DMS, Front30, Rear, SideView_FL/FR/RL/RR, Park_Right/Left/Rear/Front
// sensorParameters: comma-separated key=value for GMSL config
```

### 7.2 Runtime Config
```cpp
// app/dynamic_algo_cam/dynamic_algo_cam.cpp
CameraConfig config;
config.device_id = "nvsipl://0";           // Logical ID from board config
config.vendor = "nvidia";
config.width = 3840;
config.height = 2160;
config.fps = 30;
config.format = PixelFormat::NV12;
config.buffer_count = 4;
config.enable_hw_sync = true;

auto hal = CameraHALFactory::create("aarch64_nvidia", "nvsipl");
hal->initialize(config);
hal->start();
```

### 7.3 Container Deployment
```dockerfile
# Dockerfile.jetson
FROM nvcr.io/nvidia/l4t-base:r36.2.0
RUN apt-get update && apt-get install -y \
    libnvsipl-dev libnvmedia-dev libnvscibuf-dev \
    libnvscisync-dev libnvscistream-dev \
    cuda-toolkit-12-2 tensorrt libcudla-dev
# Copy DynamicAlgoCam build
COPY build/linux_aarch64/lib/dynalgo/hal/libdynalgo_hal_camera_nvsipl.so /usr/local/lib/dynalgo/hal/
COPY build/linux_aarch64/bin/dynamic_algo_cam /usr/local/bin/
```

---

## Phase 8: Testing & Validation (Week 8) 📋 PLANNED

### 8.1 Unit Tests
- NvSciBuf import/export round-trip
- NvSciSync fence signaling/wait
- SIPL camera open/close/start/stop
- CUDA kernel correctness (NMS, resize, color convert)

### 8.2 Integration Tests
- Multi-camera synchronization (hardware trigger via GPIO)
- End-to-end latency: sensor → ISP → CUDA → inference → actuator
- Encoder quality/bitrate validation
- Display pipeline (WFD/HDMI) zero-copy verification

### 8.3 Performance Benchmarks
| Metric | Target (Orin AGX) |
|--------|-------------------|
| ISP throughput | 4x 4K@30fps / 8x 1080p@30fps |
| CUDA preprocess | < 2ms/frame (1080p) |
| TensorRT inference | < 10ms/frame (YOLOv8n) |
| NvMedia encode | < 3ms/frame (1080p@30 H.264) |
| End-to-end latency | < 50ms (sensor → actuator) |

---

## Implementation Status Summary

| Component | Status | Location |
|-----------|--------|----------|
| CMake platform detection | ✅ Done | `cmake/platforms/aarch64_nvidia.cmake` |
| Cross-compilation toolchain | ✅ Done | `cmake/toolchains/aarch64_nvidia.cmake` |
| NvSIPL HAL (ICameraHAL) | ✅ Done | `hal/aarch64_nvidia/camera/nvsipl/nvsipl_camera_hal.cpp` |
| PlatformCfg parser (NIO JSON) | ✅ Done | `hal/aarch64_nvidia/camera/nvsipl/nvsipl_platform_cfg.cpp` |
| GMSL params parser | ✅ Done | `ParseGmslParams()` |
| NIO Query flow (INvSIPLQuery) | ✅ Done | `BuildPlatformCfgViaQuery()` |
| Direct PlatformCfg construction | ✅ Done | `BuildPlatformCfgDirect()` |
| Vendored nlohmann/json | ✅ Done | `third_party/nlohmann_json.hpp` |
| Docker build validation | ✅ Done | aarch64 ELF objects produced |
| Frame queue handlers | ✅ Done | `PipelineFrameQueueHandler`, `PipelineNotificationHandler` |
| NvSciBuf zero-copy | 📋 Planned | `FrameMetadata.buffer.handle = NvSciBufObj` |
| CUDA consumer | 📋 Planned | `nvsipl_consumer_cuda.cpp` |
| NvMedia encoder | 📋 Planned | `hal/aarch64_nvidia/encoder/nvmedia/` |
| TensorRT + DLA inference | 📋 Planned | `hal/aarch64_nvidia/inference/tensorrt/` |
| Multi-camera sync | 📋 Planned | Hardware GPIO trigger |

---

## Key Technical Decisions

1. **HAL-based architecture**: Implementation in `hal/aarch64_nvidia/camera/nvsipl/` (not `app/driver/`) to maintain platform/vendor/board hierarchy
2. **Vendored nlohmann/json**: Single-header `third_party/nlohmann_json.hpp` avoids external dependency in container
3. **jsoncpp for JSON parsing**: Uses OrbbecSDK's jsoncpp vendor (`#include <json/json.h>`)
4. **No yaml-cpp dependency**: YAML parsing returns default config with warning (container lacks yaml-cpp)
5. **NIO Query flow preferred**: `INvSIPLQuery::GetPlatformCfg` + `ApplyMask` for real hardware; direct construction as fallback
6. **sensorParameters only**: CCC parameters (`cccParameters`) not used; standard hardware uses `sensorParameters` field
7. **Container build verified**: aarch64 ELF objects produced with `aarch64-linux-gnu-g++ 9.4.0`

---

## Dependencies (DriveOS 6.0.7 / JetPack 6.x)

| Package | Version | Source |
|---------|---------|--------|
| NvSIPL | 6.0.x | DriveOS SDK (`/drive/drive-linux/include`, `/drive/drive-linux/lib-target`) |
| NvMedia | 6.0.x | Component libs (no single libnvmedia.so) |
| CUDA | 11.4 | Container `/usr/local/cuda-11.4` |
| TensorRT | 8.6+ | JetPack / container (optional) |
| CUDLA | 12.2+ | DriveOS (optional in container) |
| libnvscibuf | 6.0.x | `/drive/drive-linux/lib-target/libnvscibuf.so` |
| libnvscisync | 6.0.x | `/drive/drive-linux/lib-target/libnvscisync.so` |
| libnvscistream | 6.0.x | `/drive/drive-linux/lib-target/libnvscistream.so` |

---

## Deliverables

1. **Source**: `hal/aarch64_nvidia/camera/nvsipl/` (complete NvSIPL HAL stack)
2. **Build**: Cross-compilation toolchain, CMake integration ✅
3. **Config**: NIO/AD `camera.json` parser with GMSL params ✅
4. **Binary**: `libdynalgo_hal_camera_nvsipl.so` aarch64 shared library ✅ (compiled)
5. **Container**: Dockerfile for Jetson/Drive deployment 📋
6. **Docs**: This document + sensor bring-up checklist ✅

---

## Timeline Summary

| Week | Phase | Key Deliverable | Status |
|------|-------|-----------------|--------|
| 1-2 | Build System | CMake ARM64 detection, toolchain, dependency find | ✅ Done |
| 2-4 | NvSIPL HAL | `ICameraHAL`, SIPL camera, frame handlers, notifications | ✅ Done |
| 3 | PlatformCfg Parser | NIO JSON, GMSL params, NIO Query flow | ✅ Done |
| 4-5 | Zero-Copy Pipeline | NvSciBuf/Sync integration, pipeline topology | 📋 Planned |
| 5-6 | CUDA/CUDLA | Preprocess kernels, DLA inference backend | 📋 Planned |
| 6 | NvMedia Encoder | HW H.264/H.265 encoder plugin | 📋 Planned |
| 7 | Config/Deploy | PlatformCfg, Docker, runtime config | 📋 Partially Done |
| 8 | Test/Validate | Unit/integration tests, benchmarks, docs | 📋 Planned |

**Total: ~8 weeks for MVP on Jetson AGX Orin / Drive Orin**