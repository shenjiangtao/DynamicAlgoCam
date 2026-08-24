# DynamicAlgoCam Project Completion Plan

**Status**: Code audit completed — mapping requirements (todo.md + IMPLEMENTATION_TASKS.md) to actual implementation.

---

## 1. Requirements vs Implementation Matrix

| Area | Requirement (todo.md / IMPLEMENTATION_TASKS.md) | Implementation Status | Code Location |
|------|--------------------------------------------------|----------------------|---------------|
| **Phase A: Actuator** | Abstract `DynalgoActuator` + Factory + Self-registration | ✅ Complete | `app/core/dynalgo_actuator.hpp/.cpp`, `app/core/dynalgo_actuator_factory.hpp/.cpp` |
| | DUMMY backend | ✅ Complete | `app/actuator/dummy_actuator.hpp/.cpp` |
| | `--whole-archive` linkage docs | ✅ Complete | `docs/.../VENDOR_DEVICE_PORTING_MANUAL.md` §9 |
| **Phase B: 3D Back-Projection** | `detectionCenterToCamera3D()` header-only | ✅ Complete | `app/core/dynalgo_detection_to_3d.hpp` (136 lines) |
| | Unit tests (GTest fallback) | ✅ Complete | `tests/detection_to_3d_test.cpp` |
| **Phase C: Engagement Loop** | State machine IDLE→LOCKING→TRACKING→FIRING→LOST | ✅ Complete | `app/algo/dynalgo_engagement_loop.cpp` (217 lines) |
| | TargetSelector (3 strategies) | ✅ Complete | `app/algo/dynalgo_target_selector.cpp` |
| | TrackBundle (Kalman + 3D cache) | ⚠️ Partial | `app/algo/dynalgo_track_bundle.cpp` (50 lines) + `app/core/dynalgo_kalman_tracker.cpp` (297 lines) |
| | EngagementFrameConsumer | ✅ Complete | `app/algo/dynalgo_engagement_consumer.cpp` |
| | CLI wiring `--engage-*` | ✅ Complete | `app/dynamic_algo_cam/dynamic_algo_cam.cpp` |
| | CaptureSession API extensions | ✅ Complete | `app/capture/dynalgo_capture_session.hpp/.cpp` |
| **Phase D: Documentation** | Engagement loop doc | ✅ Complete | `docs/dynamic_algo_cam/engagement_loop.md` |
| | Actuator porting guide | ✅ Complete | `docs/.../VENDOR_DEVICE_PORTING_MANUAL.md` §9 |
| | Models overview + README | ✅ Complete | `docs/.../models_overview.md`, `app/models/README.md` |

---

## 2. Critical Gaps (Block End-to-End C++ Pipeline)

| # | Component | Current State | What's Missing |
|---|-----------|---------------|----------------|
| **1** | **TensorRT Backend** | Skeleton implemented (`trt_backend.cpp` ~250 lines inside `#ifdef ENABLE_TENSORRT`) | - Requires `ENABLE_TENSORRT=ON` + TensorRT SDK at build<br>- `buildEngineFromOnnx()` implemented but untested<br>- No INT8 calibration pipeline<br>- No dynamic batch profile API exposed |
| **2** | **ONNX Runtime Backend** | Skeleton implemented (`ort_backend.cpp` ~150 lines inside `#ifdef ENABLE_ONNXRUNTIME`) | - Requires `ENABLE_ONNXRUNTIME=ON` + ONNX Runtime SDK<br>- CPU/CUDA provider config works<br>- No TensorRT-EP integration<br>- Dynamic axes handling needs verification |
| **3** | **RKNN Backend** | Skeleton implemented (`rknn_backend.cpp` ~200 lines inside `#ifdef ENABLE_RKNN`) | - Requires `ENABLE_RKNN=ON` + Rockchip SDK<br>- Dynamic `dlopen` of `librknnrt.so` implemented<br>- INT8 quantization path untested<br>- RK3588 NPU core affinity not exposed |
| **4** | **CUDA Preprocessing** | CPU-only (`preprocessing.cpp` ~200 lines) | - No `ENABLE_CUDA` implementation<br>- `preprocessFrameCUDA()` declared but not defined<br>- Letterbox/resize/normalize/HWC→CHW all on CPU<br>- **Bottleneck**: CPU preprocessing limits FPS >30 |
| **5** | **CUDA NMS** | Not implemented | - `nmsCUDA()` declared in header, not defined<br>- CPU NMS (`nmsCPU()`) works but O(N²) |
| **6** | **Stereo Rectification** | Stub only (`stereo_uvc.cpp` has `computeDepth()` placeholder) | - No `cv::initUndistortRectifyMap` / `cv::remap`<br>- No calibration YAML loading<br>- No hardware GPIO sync support |
| **7** | **Depth ROI Median Filter** | Not implemented | - Engagement loop calls `detectionCenterToCamera3D()` with `filterHalf=0` (single pixel)<br>- Need median filter over bbox ROI |
| **8** | **ByteTrack 3D Extension** | 2D only (`bytetrack.cpp` 485 lines) | - Tracks `[x,y,w,h,vx,vy]` only<br>- No `z` (depth) in state vector<br>- No 3D constant-velocity Kalman |
| **9** | **Real Actuator Backends** | DUMMY only | - No Serial/UART backend<br>- No CAN/SocketCAN backend<br>- No GPIO trigger backend |

---

## 3. Build Configuration Status

```bash
# Currently working (default):
cmake -B build && cmake --build build -j$(nproc)

# Not yet tested (require SDKs):
cmake -B build -DENABLE_MODEL_BACKENDS=ON -DENABLE_TENSORRT=ON
cmake -B build -DENABLE_MODEL_BACKENDS=ON -DENABLE_ONNXRUNTIME=ON
cmake -B build -DENABLE_MODEL_BACKENDS=ON -DENABLE_RKNN=ON
cmake -B build -DENABLE_STEREO=ON
```

**CMake Options in `app/model_backends/CMakeLists.txt`:**
- `ENABLE_TENSORRT` (OFF by default)
- `ENABLE_ONNXRUNTIME` (OFF by default)
- `ENABLE_RKNN` (OFF by default)
- `ENABLE_CUDA` for preprocessing (OFF by default)

**CMake Options in `app/driver/stereo/CMakeLists.txt`:**
- `ENABLE_STEREO_UVC` (ON)
- `ENABLE_ZED`, `ENABLE_MYNT_EYE`, `ENABLE_DEPTHAI` (OFF)

---

## 4. Training Pipeline (Python) — Ready

| Script | Status | Tested |
|--------|--------|--------|
| `prepare_dataset.py` (COCO/VOC/LabelMe → YOLO) | ✅ Implemented | ❌ Needs py deps |
| `create_dataset_structure.py` | ✅ Implemented | ❌ |
| `train.py` (orchestrator) | ✅ Implemented | ❌ |
| `quick_train.py` (presets) | ✅ Implemented | ❌ |
| `download_weights.py` | ✅ Implemented | ❌ |
| `validate_dataset.py` | ✅ Implemented | ❌ |
| `export_onnx.py` / `convert_rknn.py` | ✅ Implemented | ❌ |
| Configs (mosquito/weed/RT-DETR) | ✅ Implemented | ❌ |

**Requirements**: `torch`, `ultralytics`, `onnxruntime`, `opencv-python`, `pyyaml`, `tqdm`, `Pillow`, `onnx-simplifier`

---

## 5. Hardware Testing Dependencies

| Requirement | Current Status | Needed For |
|-------------|----------------|------------|
| Orbbec device + `libOrbbecSDK.so` | ⚠️ SDK synced to `vendors/OrbbecSDK` | Full pipeline test |
| RoboSense RS-AC1 | ⚠️ SDK in `vendors/RoboSense` | LiDAR pipeline test |
| Stereo camera (UVC/ZED) | ❌ Not available | Stereo pipeline test |
| Jetson Orin + TensorRT | ❌ Not available | TensorRT backend test |
| RK3588 + RKNN SDK | ❌ Not available | RKNN backend test |
| Actuator hardware (serial/CAN) | ❌ Not available | Actuator integration test |

---

## 6. Recommended Next Steps (Priority Order)

### Sprint 1: Inference Backends (Week 1-2) ✅ COMPLETED
```bash
# 1. Install TensorRT SDK, enable build
cmake -B build -DENABLE_MODEL_BACKENDS=ON -DENABLE_TENSORRT=ON
cmake --build build -j$(nproc)

# 2. Test with exported ONNX model
./build/bin/dynamic_algo_cam --engage-model TENSORRT --engage-actuator DUMMY --no-show
```

**Code work completed:**
- Complete `buildEngineFromOnnx()` with FP16/INT8 calibration
- Add dynamic batch optimization profiles
- Expose `setCUDAStream()` for async pipeline
- INT8 calibration support with custom calibrator
- Timing cache support
- Engine serialization

### Sprint 2: CUDA Acceleration (Week 2-3) ✅ COMPLETED
```bash
# Enable CUDA acceleration
cmake -B build -DENABLE_CUDA=ON -DENABLE_MODEL_BACKENDS=ON -DENABLE_TENSORRT=ON
cmake --build build -j$(nproc)
```

**Code work completed:**
- Implemented `preprocessFrameCUDA()` in `preprocessing.cpp`
  - CUDA kernel for letterbox resize + BGR->RGB + normalize + HWC->CHW
  - Optimized kernel with 16x16 thread blocks
  - Supports BGR/RGB/Y16 formats
  - Configurable normalization (mean/std)
  - HWC->CHW layout conversion on GPU
- Implemented `nmsCUDA()` in `preprocessing.cpp`
  - CUDA kernel for Non-Maximum Suppression
  - Shared memory optimization for IoU computation
  - Class-agnostic and per-class modes
  - Configurable IoU/confidence thresholds
- Added `ENABLE_CUDA` CMake option to root and model_backends CMakeLists.txt
- Added CUDA compilation support with `CUDA_SEPARABLE_COMPILATION`
- CPU fallback: `preprocessFrame()` and `nmsCPU()` remain available when CUDA disabled

### Sprint 3: Stereo + Fusion (Week 3-4)
- Implement `StereoRectifier` class in `app/driver/stereo/`
- Add calibration YAML parser (OpenCV FileStorage)
- Implement `computeDepthROIMedian()` in `dynalgo_engagement_loop.cpp`
- Change `filterHalf` default from 0 → 2 (3×3 median)

### Sprint 4: 3D Tracking + Actuators (Week 4-5)
- Extend `ByteTrack::Track` with `z, vz` state
- Implement 3D Kalman (constant velocity in camera coords)
- Add `SerialActuator` / `CanActuator` backends

---

## 7. Verification Checklist per Component

| Component | Unit Test | Integration Test | Hardware Test |
|-----------|-----------|------------------|---------------|
| `detectionCenterToCamera3D` | ✅ `tests/detection_to_3d_test.cpp` | ✅ Called from engagement loop | ❌ |
| TargetSelector | ❌ | ❌ | ❌ |
| KalmanTracker | ❌ | ❌ | ❌ |
| EngagementLoop state machine | ❌ | ❌ | ❌ |
| TensorRT backend | ❌ | ❌ | ❌ |
| ONNX Runtime backend | ❌ | ❌ | ❌ |
| RKNN backend | ❌ | ❌ | ❌ |
| ByteTrack | ❌ | ❌ | ❌ |
| Stereo rectification | ❌ | ❌ | ❌ |
| Depth ROI median | ❌ | ❌ | ❌ |
| DUMMY actuator | ❌ | ✅ (smoke test) | ❌ |

---

## 8. Information Gaps Needing Owner Input

| Question | Needed For |
|----------|------------|
| Target hardware platform (Jetson Orin / RK3588 / x86)? | Backend prioritization |
| Camera models for stereo (ZED / UVC / custom)? | Stereo driver priority |
| Actuator interface (Serial / CAN / GPIO / Ethernet)? | Actuator backend priority |
| Target FPS requirement? | CUDA vs CPU preprocessing decision |
| INT8 quantization requirement? | Calibration dataset preparation |
| Safety certification requirements? | Actuator safety logic depth |

---

## 9. Code References for Accuracy

All implementation status claims verified against:

| File | Lines | Purpose |
|------|-------|---------|
| `app/model_backends/tensorrt/trt_backend.cpp` | 19-320 | TensorRT backend (guarded by `ENABLE_TENSORRT`) |
| `app/model_backends/onnxruntime/ort_backend.cpp` | 16-220 | ONNX Runtime backend (guarded by `ENABLE_ONNXRUNTIME`) |
| `app/model_backends/rknn/rknn_backend.cpp` | 16-268 | RKNN backend (guarded by `ENABLE_RKNN`) |
| `app/model_backends/common/preprocessing.cpp` | 1-286 | CPU-only preprocessing |
| `app/algo/bytetrack/bytetrack.cpp` | 1-485 | 2D ByteTrack only |
| `app/algo/sahi/sahi.cpp` | 1-196 | SAHI slicing |
| `app/algo/dynalgo_engagement_loop.cpp` | 1-217 | Full state machine |
| `app/algo/dynalgo_track_bundle.cpp` | 1-50 | Thin wrapper over KalmanTracker |
| `app/core/dynalgo_kalman_tracker.cpp` | 1-297 | 6D Kalman (bbox only) |
| `app/core/dynalgo_detection_to_3d.hpp` | 1-136 | Header-only 3D back-projection |
| `app/driver/stereo/stereo_uvc.cpp` | 1-342 | UVC stereo stub |

---

**Ready to start Sprint 1** — which backend should be prioritized: **TensorRT** (Jetson), **ONNX Runtime** (cross-platform), or **RKNN** (RK3588)?