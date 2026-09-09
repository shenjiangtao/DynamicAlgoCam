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
| **Sensor Config** | Runtime USB descriptors | PlatformCfg (YAML), CameraModuleInfo |

---

## Phase 1: Build System & Platform Detection (Week 1-2)

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
# cmake/toolchains/jetson-aarch64.cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# L4T sysroot (adjust for JetPack version)
set(CMAKE_SYSROOT /opt/nvidia/jetson-sdk/l4t/sysroot)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
```

### 1.3 Dependency Discovery
```cmake
# Find NvSIPL, NvMedia, NvSciBuf, NvSciSync, NvSciStream
find_library(NVSIPL_LIB nvsipl PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVMEDIA_IEP_LIB nvmedia_iep_sci PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVSCIBUF_LIB nvscibuf PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVSCISYNC_LIB nvscisync PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVSCISTREAM_LIB nvscistream PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVSCIEVENT_LIB nvscievent PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVSICIPC_LIB nvsciipc PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(NVMEDIA_2D_LIB nvmedia2d PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(TEGRA_WFD_LIB tegrawfd PATHS /usr/lib/aarch64-linux-gnu/tegra)

# CUDA on ARM
find_package(CUDA REQUIRED)
set(CUDA_ARCHITECTURES "87;87" CACHE STRING "Ampere (Orin) compute capability")
```

---

## Phase 2: NvSIPL Driver Abstraction (Week 2-4)

### 2.1 Directory Structure
```
app/driver/
├── nvsipl/
│   ├── CMakeLists.txt
│   ├── nvsipl_adapter.hpp       # Abstract interface (like stereo_adapter.hpp)
│   ├── nvsipl_camera.cpp        # CSiplCamera wrapper
│   ├── nvsipl_producer.cpp      # CSIPLProducer → NvSciStream producer
│   ├── nvsipl_consumer_cuda.cpp # CCudaConsumer → GPU processing
│   ├── nvsipl_consumer_enc.cpp  # CEncConsumer → NvMedia IEP encoding
│   ├── nvsipl_consumer_display.cpp # CDisplayConsumer → WFD/DRM
│   ├── nvsipl_platform_cfg.cpp  # PlatformCfg parsing (YAML)
│   └── nvsipl_utils.cpp         # Error handling, logging
```

### 2.2 Abstract Interface (`nvsipl_adapter.hpp`)
```cpp
#pragma once
#include "dynalgo_frame.hpp"
#include "dynalgo_types.hpp"

namespace dynalgo {

struct NvSIPLConfig {
    std::string platformConfigFile;   // PlatformCfg YAML (e.g., platform_cfg_orin.yml)
    std::string sensorModuleConfig;   // CameraModuleInfo selector
    uint32_t numCameras = 1;
    bool enableISP = true;
    bool enableEncoder = false;
    bool enableDisplay = false;
    bool enableInference = false;
    int cudaDeviceId = 0;
};

struct NvSIPLFrameSet {
    uint64_t timestampUs = 0;
    uint32_t frameId = 0;
    uint32_t sensorId = 0;

    // ISP outputs
    DynalgoFrame isp0;      // NV12 block-linear (ISP0)
    DynalgoFrame isp1;      // NV12 pitch-linear (ISP1)
    DynalgoFrame icpRaw;    // Raw Bayer (ICP)

    // Encoded output
    DynalgoFrame encoded;   // H.264/H.265 bitstream

    // Inference output
    std::vector<DynalgoDetectionResult> detections;

    // Metadata
    NvSIPLImageMetadata meta;
};

class INvSIPLCamera {
public:
    virtual ~INvSIPLCamera() = default;
    virtual bool open(const NvSIPLConfig& cfg) = 0;
    virtual bool close() = 0;
    virtual bool isOpen() const = 0;
    virtual bool grab(NvSIPLFrameSet& outFrame, int timeoutMs = 1000) = 0;
    virtual bool startStreaming(std::function<void(const NvSIPLFrameSet&)> cb) = 0;
    virtual bool stopStreaming() = 0;
    virtual void setCallback(std::function<void(const NvSIPLFrameSet&)> cb) = 0;
};

class NvSIPLCameraFactory {
public:
    static std::unique_ptr<INvSIPLCamera> create();
    static std::vector<std::string> discoverPlatformConfigs();
    static std::vector<CameraModuleInfo> discoverSensors(const std::string& platformCfg);
};

} // namespace dynalgo
```

### 2.3 SIPL Camera Wrapper (`nvsipl_camera.cpp`)
- Wrap `CSiplCamera` from multicast sample
- Implement `INvSIPLCamera` interface
- Handle `PlatformCfg` loading, sensor enumeration
- Manage `CPipelineFrameQueueHandler`, `CPipelineNotificationHandler`, `CDeviceBlockNotificationHandler`
- Callback → `NvSIPLFrameSet` conversion using NvSciBuf CPU mapping

### 2.4 SIPL Producer (`nvsipl_producer.cpp`)
- Wrap `CSIPLProducer` → NvSciStream block producer
- Register NvSciBufObj with `INvSIPLCamera::RegisterImages()`
- Register NvSciSyncObj (EOF/PRE) with camera
- Handle `NvSciStreamBlockPacketFenceSet` for synchronization

### 2.5 CUDA Consumer (`nvsipl_consumer_cuda.cpp`)
- Wrap `CCudaConsumer` → NvSciStream consumer
- Import NvSciBufObj as CUDA external memory (`cudaImportExternalMemory`)
- Block-linear → pitch-linear conversion via mipmapped arrays
- CUDA stream synchronization with NvSciSyncFence
- Optional: CUDLA inference (car detection example)

### 2.6 Encoder Consumer (`nvsipl_consumer_enc.cpp`)
- Wrap `CEncConsumer` → NvMedia IEP
- `NvMediaIEPFillNvSciBufAttrList` for buffer negotiation
- `NvMediaIEPFeedFrame` + `NvMediaIEPGetBits` for encoding
- EOF sync fence for downstream consumers

### 2.7 Display Consumer (`nvsipl_consumer_display.cpp`)
- Wrap `CDisplayConsumer` → OpenWFD / DRM/KMS
- Zero-copy display via NvSciBuf

---

## Phase 3: Zero-Copy Pipeline Integration (Week 4-5)

### 3.1 NvSciBuf Integration in `DynalgoFrame`
```cpp
// Extend dynalgo_frame.hpp for ARM/NvSciBuf
struct DynalgoFrame {
    // ... existing fields ...
    
#ifdef DYNALOGO_HAVE_NVSCIBUF
    NvSciBufObj sciBufObj = nullptr;      // NvSciBuf object handle
    NvSciBufAttrList attrList = nullptr;  // Buffer attributes
    bool ownsSciBuf = false;              // Ownership flag
#endif
};

// Helper: Map NvSciBufObj to CPU pointer
bool mapNvSciBufToCpu(NvSciBufObj obj, void** cpuPtr, size_t* size);

// Helper: Get NvSciBuf attributes (width, height, format, layout)
bool queryNvSciBufAttrs(NvSciBufObj obj, NvSciBufAttrList* attrList);
```

### 3.2 NvSciSync Integration
```cpp
// Sync primitives for inter-op synchronization
class NvSciSyncWrapper {
public:
    static bool createSignaler(NvSciSyncObj* obj, NvSciSyncAttrList attrs);
    static bool createWaiter(NvSciSyncObj* obj, NvSciSyncAttrList attrs);
    static bool wait(NvSciSyncObj obj, NvSciSyncFence* fence, uint64_t timeoutNs);
    static bool signal(NvSciSyncObj obj, NvSciSyncFence* fence);
    static void destroy(NvSciSyncObj obj);
};
```

### 3.3 Pipeline Topology
```
[Sensor] → [NvSIPL ICP/ISP] → [NvSciBuf] ──┬─→ [CUDA Consumer] → [Inference] → [Actuator]
                                           ├─→ [Encoder Consumer] → [H.264 Stream]
                                           └─→ [Display Consumer] → [WFD/HDMI]
```

---

## Phase 4: CUDA/CUDLA Acceleration (Week 5-6)

### 4.1 CUDA Preprocessing Kernels
```cpp
// app/algo/cuda_preprocess.cu
__global__ void nv12_to_rgb_kernel(const uint8_t* y, const uint8_t* uv,
                                   uint8_t* rgb, int width, int height, int pitch);
__global__ void letterbox_resize_kernel(const uint8_t* src, uint8_t* dst,
                                        int srcW, int srcH, int dstW, int dstH);
__global__ void nms_kernel(float* boxes, float* scores, int numBoxes,
                           float iouThresh, float scoreThresh, int* keep, int* keepCount);
```

### 4.2 CUDLA Integration (DLA)
```cpp
// app/model_backends/tensorrt/trt_dla_backend.cpp
class TRTDLABackend : public DynalgoModelBackend {
    // Load TensorRT engine with DLA enabled
    // Builder config: setDefaultDeviceType(kDLA), setDLACore(0)
    // Input: NvSciBufObj → CUDA external memory → TensorRT
};
```

### 4.3 Build Integration
```cmake
# app/algo/CMakeLists.txt
if(ENABLE_CUDA AND DYNALGO_PLATFORM_ARM64)
    find_package(CUDA REQUIRED)
    cuda_add_library(dynalgo_cuda_preprocess
        cuda_preprocess.cu
        nms_kernel.cu
    )
    target_link_libraries(dynalgo_cuda_preprocess ${CUDA_LIBRARIES} cudart)
    target_compile_options(dynalgo_cuda_preprocess PRIVATE
        -gencode arch=compute_87,code=sm_87  # Orin Ampere
    )
endif()

if(ENABLE_CUDLA)
    find_library(CUDLA_LIB cudla PATHS /usr/lib/aarch64-linux-gnu/tegra)
    target_link_libraries(dynalgo_model_backend ${CUDLA_LIB})
endif()
```

---

## Phase 5: NvMedia Encoder Integration (Week 6)

### 5.1 Encoder Backend
```cpp
// app/plugins/nvmedia_encoder.hpp
class NvMediaEncoder : public IEncoder {
public:
    bool init(int width, int height, int fps, int bitrate, Codec codec);
    bool encode(NvSciBufObj inputBuf, std::vector<uint8_t>& outputBitstream);
    bool flush();
    void deinit();

private:
    NvMediaIEP* m_iep = nullptr;
    NvSciSyncObj m_eofSyncObj = nullptr;
    // ... encoder config
};
```

### 5.2 Pipeline Integration
- Add encoder as optional consumer in `NvSIPLConfig`
- Wire NvSciSync EOF fence from ISP → Encoder
- Output bitstream → network streaming / file

---

## Phase 6: Configuration & Deployment (Week 7)

### 6.1 Platform Configuration (YAML)
```yaml
# configs/nvsipl/platform_cfg_orin.yml
platformCfg:
  deviceBlocks:
    - name: "deser_0"
      cdi: "max96712"
      cdiConfig: "max96712_tpg_yuv"
      cameraModules:
        - name: "imx728"
          sensor: "imx728"
          linkIndex: 0
          position: "front_center"
          intrinsics: "calib/imx728_front.yml"
```

### 6.2 Runtime Config
```cpp
// app/dynamic_algo_cam/dynamic_algo_cam.cpp
if (platform == "jetson" || platform == "drive") {
    auto nvsiplCam = NvSIPLCameraFactory::create();
    NvSIPLConfig cfg;
    cfg.platformConfigFile = "configs/nvsipl/platform_cfg_orin.yml";
    cfg.enableISP = true;
    cfg.enableEncoder = true;
    cfg.enableInference = true;
    nvsiplCam->open(cfg);
    // Register with CaptureSession
}
```

### 6.3 Container / Rootfs Setup
```dockerfile
# Dockerfile.jetson
FROM nvcr.io/nvidia/l4t-base:r36.2.0
RUN apt-get update && apt-get install -y \
    libnvsipl-dev libnvmedia-dev libnvscibuf-dev \
    libnvscisync-dev libnvscistream-dev \
    cuda-toolkit-12-2 tensorrt libcudla-dev
# Copy DynamicAlgoCam build
COPY build/linux_aarch64/bin/dynamic_algo_cam /usr/local/bin/
```

---

## Phase 7: Testing & Validation (Week 8)

### 7.1 Unit Tests
- NvSciBuf import/export round-trip
- NvSciSync fence signaling/wait
- SIPL camera open/close/start/stop
- CUDA kernel correctness (NMS, resize, color convert)

### 7.2 Integration Tests
- Multi-camera synchronization (hardware trigger via GPIO)
- End-to-end latency: sensor → ISP → CUDA → inference → actuator
- Encoder quality/bitrate validation
- Display pipeline (WFD/HDMI) zero-copy verification

### 7.3 Performance Benchmarks
| Metric | Target (Orin AGX) |
|--------|-------------------|
| ISP throughput | 4x 4K@30fps / 8x 1080p@30fps |
| CUDA preprocess | < 2ms/frame (1080p) |
| TensorRT inference | < 10ms/frame (YOLOv8n) |
| NvMedia encode | < 3ms/frame (1080p@30 H.264) |
| End-to-end latency | < 50ms (sensor → actuator) |

---

## Migration Checklist

### Code Changes Required
- [ ] Root CMakeLists.txt: ARM64 detection, NvSIPL/NvMedia options
- [ ] `cmake/toolchains/jetson-aarch64.cmake` (new)
- [ ] `app/driver/nvsipl/` (new directory with 7 files)
- [ ] `app/include/dynalgo_frame.hpp`: NvSciBuf fields
- [ ] `app/core/dynalgo_capture_session.cpp`: NvSIPL camera registration
- [ ] `app/algo/`: CUDA kernels, CUDLA backend
- [ ] `app/plugins/`: NvMedia encoder plugin
- [ ] `configs/nvsipl/`: PlatformCfg YAML files
- [ ] `scripts/build_jetson.sh`: Cross-compile script
- [ ] `Dockerfile.jetson`: Deployment container
- [ ] Documentation: ARM porting guide, sensor bring-up

### Dependencies (JetPack 6.x / DriveOS 6.x)
| Package | Version | Source |
|---------|---------|--------|
| NvSIPL | 6.0.x | DriveOS / JetPack |
| NvMedia | 6.0.x | DriveOS / JetPack |
| CUDA | 12.2+ | JetPack |
| TensorRT | 8.6+ | JetPack |
| CUDLA | 12.2+ | JetPack |
| libnvscibuf | 6.0.x | DriveOS / JetPack |
| libnvscisync | 6.0.x | DriveOS / JetPack |
| libnvscistream | 6.0.x | DriveOS / JetPack |

### Sensor Bring-Up (Per Camera Module)
1. Obtain PlatformCfg from NVIDIA / sensor vendor
2. Calibrate intrinsics/extrinsics → YAML
3. Validate ICP/ISP/ACP/CDI pipeline
4. Tune ISP parameters (AE, AWB, CCM)
5. Verify frame sync across cameras

---

## Risk Mitigation

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| NvSIPL API changes | Medium | High | Pin DriveOS/JetPack version; abstraction layer |
| NvSciBuf compatibility | Low | High | Test on target L4T version; fallback to CPU copy |
| CUDA/DLA memory pressure | Medium | Medium | Pool allocation; limit concurrent buffers |
| Sensor driver availability | Medium | High | Maintain supported sensor list; generic CSI fallback |
| Cross-compile complexity | High | Medium | Docker-based build; CI on ARM runners |

---

## Deliverables

1. **Source**: `app/driver/nvsipl/` (complete NvSIPL driver stack)
2. **Build**: Cross-compilation toolchain, CMake integration
3. **Config**: PlatformCfg YAML for Jetson Orin + common sensors (IMX728, IMX623, AR0820)
4. **Binary**: `dynamic_algo_cam` aarch64 executable
5. **Container**: Dockerfile for Jetson deployment
6. **Docs**: `NVIDIA_ARM_PORTING_GUIDE.md`, sensor bring-up checklist
7. **CI**: GitHub Actions / Jetson CI for ARM builds

---

## Timeline Summary

| Week | Phase | Key Deliverable |
|------|-------|-----------------|
| 1-2 | Build System | CMake ARM64 detection, toolchain, dependency find |
| 2-4 | NvSIPL Driver | `INvSIPLCamera`, SIPL camera, producer, consumers |
| 4-5 | Zero-Copy Pipeline | NvSciBuf/Sync integration, pipeline topology |
| 5-6 | CUDA/CUDLA | Preprocess kernels, DLA inference backend |
| 6 | NvMedia Encoder | HW H.264/H.265 encoder plugin |
| 7 | Config/Deploy | PlatformCfg, Docker, runtime config |
| 8 | Test/Validate | Unit/integration tests, benchmarks, docs |

**Total: ~8 weeks for MVP on Jetson AGX Orin**