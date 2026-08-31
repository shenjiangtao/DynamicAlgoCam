# DynamicAlgoCam

DynamicAlgoCam is a camera-driven dynamic algorithm loading framework. It provides a unified perception and execution platform built on stereo cameras and LiDAR cameras, supporting modular algorithm loading, runtime task switching, and visual-LiDAR fusion.

**Key capabilities:**

- Modular algorithm loading with runtime task switching
- Visual + LiDAR fusion for high-precision target detection and environment perception
- Rapid deployment and validation for domain-specific tasks (e.g., mosquito control, weeding, inspection)
- Event-driven recording with IMU-aware depth alignment
- Perceive → Locate → Estimate → Control engagement loop (Phase C)

## Supported Devices

| Device | VID | Sensors | Output Files |
|---|---|---|---|
| Orbbec Gemini 305 | `2bc5:0840` | Color, Depth, IR L+R | `.h264`, `.raw`, fusion `.h264` |
| Orbbec Gemini 335L | `2bc5:0804` | Color, Depth, IR L+R, IMU | `.h264`, `.raw`, `.txt`, fusion `.h264` |
| Orbbec Gemini 336L | `2bc5:0807` | Color, Depth, IR L+R, IMU | `.h264`, `.raw`, `.txt`, fusion `.h264` |
| RoboSense RS-AC1 | `3840:1010` | Color, Depth (point cloud), Point Cloud | `.h264`, `.raw`, fusion `.h264`, `*_point_raw_*.raw` |

See [docs/device_comparison.md](docs/device_comparison.md) for detailed hardware specs, stream profiles, and output format differences.

## Architecture

DynamicAlgoCam uses a **layered architecture** with strict downward-only dependencies. Vendor SDK headers are isolated in the driver layer; all upper layers remain SDK-agnostic.

```
┌──────────────────────────────────────────────────────────────────────────────────────┐
│                              dynamic_algo_cam (executable)                           │
│                                                                                      │
│  Entry point: CLI parsing → device discovery → session setup → frame pipeline →      │
│  optional engagement loop (Phase C) → graceful shutdown on SIGINT/SIGTERM            │
├──────────────────────────────────────────────────────────────────────────────────────┤
│                         Algorithm Layer (Phase C) — OPTIONAL (--engage-*)           │
│                          ┌─────────────────────────────────┐                         │
│  dynalgo_algo            │  DynalgoEngagementLoop          │  --engage-model         │
│  (static lib)            │  ┌───────────┐ ┌───────────┐   │  --engage-actuator      │
│                          │  │ IDLE      │→│ LOCKING   │   │                         │
│                          │  └───────────┘ └───────────┘   │  State machine:         │
│                          │       ↑               ↓         │  perceive → locate →    │
│                          │  ┌───────────┐ ┌───────────┐   │  estimate → control     │
│                          │  │ LOST      │←│ FIRING    │   │                         │
│                          │  └───────────┘ └───────────┘   │  Thresholds:            │
│                          │       ↑               ↓         │  LOCKING→TRACKING=3     │
│                          │  ┌───────────┐ ┌───────────┐   │  TRACKING→LOST=5        │
│                          │  │ IDLE      │←│ TRACKING  │   │  FIRING cooldown=1s     │
│                          │  └───────────┘ └───────────┘   │                         │
│                          └─────────────────────────────────┘                         │
│                                                                                      │
│  ┌──────────────────────┐  ┌──────────────────────┐  ┌────────────────────────┐     │
│  │ TargetSelector        │  │ TrackBundle           │  │ DummyModelBackend     │     │
│  │ (pickTarget)          │  │ (KF + 3D cache)       │  │ (DUMMY self-registrar)│     │
│  │                       │  │                        │  │                        │     │
│  │ Strategies:           │  │ 6D constant-velocity   │  │ Produces synthetic     │     │
│  │ • HIGHEST_SCORE       │  │ Kalman filter          │  │ center-frame detection │     │
│  │ • NEAREST_DEPTH       │  │ [cx,cy,w,h,vx,vy]     │  │ for dry-run testing    │     │
│  │ • LARGEST_AREA        │  │ init/update/predict    │  │                        │     │
│  └──────────────────────┘  └──────────────────────┘  └────────────────────────┘     │
│                                                                                      │
│  ┌──────────────────────┐  ┌──────────────────────┐  ┌────────────────────────┐     │
│  │ EngagementFrameConsumer│ │ DynalgoModelBackend   │  │ DynalgoKalmanTracker  │     │
│  │ (FrameConsumer)       │  │ (abstract interface)  │  │ (app/core)            │     │
│  │                        │  │                        │  │                        │     │
│  │ consume(shared_ptr     │  │ Types: NONE, DUMMY,   │  │ 6D KF [cx,cy,w,h,     │     │
│  │   <DynalgoFrameSet>)   │  │ YOLOV8_PY,            │  │  vx,vy]               │     │
│  │ → loop_->onFrame()     │  │ ONNXRUNTIME, TENSORRT │  │ init/update/predict   │     │
│  └──────────────────────┘  └──────────────────────┘  └────────────────────────┘     │
├──────────────────────────────────────────────────────────────────────────────────────┤
│                           Capture Layer (dynalgo_capture)                           │
│                                                                                      │
│  ┌──────────────────────┐  ┌──────────────────────┐  ┌────────────────────────┐     │
│  │ DynalgoCaptureSession │  │ DynalgoCaptureConfig  │  │ DynalgoFrameConsumer  │     │
│  │                        │  │                        │  │ (abstract interface)  │     │
│  │ • setupPipeline()      │  │ • CLI arg parsing      │  │                        │     │
│  │ • depthIntrinsic()     │  │ • D2C profile selection│  │ consume(FrameSet)     │     │
│  │ • depthScale()         │  │ • --engage-* options    │  │ stopTask()            │     │
│  │ • addFrameConsumer()   │  │                        │  │                        │     │
│  │ • setEngagementLoop()  │  │ DynalgoFrameQueue      │  └────────────────────────┘     │
│  └──────────────────────┘  │ DynalgoStreamTasks      │                                 │
│                            │ DynalgoH264Encoder      │                                 │
│  DynalgoSDLViewer          │ DynalgoStreamIO          │  Event-driven recording:       │
│  DynalgoColorConvert       │ EventWindow              │  ENABLE_EVENT_SIM → EventSim   │
│  (optional OpenCV plugin)  │ eventSink lambda         │                                │
│                            └──────────────────────┘                                 │
├──────────────────────────────────────────────────────────────────────────────────────┤
│                      Actuator Layer (dynalgo_actuators) — OPTIONAL (--engage-*)     │
│                          ┌─────────────────────────────────┐                         │
│  dynalgo_actuators       │  DynalgoActuator (abstract)     │  --engage-actuator      │
│  (static lib)            │  ┌────────────────────────────┐ │                         │
│                          │  │ config(): dryRun, device,  │ │  Types: NONE, DUMMY,    │
│                          │  │ channel, ip, port, baud    │ │  LASER_GENERIC,         │
│                          │  └────────────────────────────┘ │  GIMBAL_GENERIC          │
│                          │                                  │                         │
│                          │  DummyActuator (all no-op)       │  Safety: dryRun=true     │
│                          │  self-registrar via              │  by default              │
│                          │  registerActuator()              │                         │
│                          └─────────────────────────────────┘                         │
├──────────────────────────────────────────────────────────────────────────────────────┤
│                           Driver Layer (dynalgo_drivers)                             │
│                          ┌─────────────────────────────────┐                         │
│  dynalgo_drivers         │  DynalgoDevice (abstract)        │                         │
│  (static lib)            │  DynalgoPipeline (abstract)      │                         │
│                          │  DynalgoContext (abstract)        │                         │
│                          │  DynalgoDriverFactory             │                         │
│                          └─────────────────────────────────┘                         │
│                                                                                      │
│  ┌──────────────────────┐  ┌──────────────────────┐                                │
│  │ Orbbec Adapter        │  │ RoboSense Adapter     │                                │
│  │ (ENABLE_ORBBEC)       │  │ (ENABLE_RS_AC1)       │                                │
│  │                        │  │                        │                                │
│  │ DynalgoObDevice       │  │ DynalgoRsDevice       │                                │
│  │ DynalgoObAdapter      │  │ DynalgoRsAdapter      │                                │
│  │ DynalgoObFrameAdapter │  │ DynalgoRsFrameAdapter │                                │
│  │ DynalgoObSpec         │  │ DynalgoRsSpec         │                                │
│  │ DynalgoObValidator    │  │ DynalgoRsValidator    │                                │
│  └──────────────────────┘  └──────────────────────┘                                │
│                                                                                      │
│  Vendor SDK isolation: OrbbecSDK headers → app/driver/orbbec/ only                  │
│                        rs_driver headers  → app/driver/robosense/ only              │
├──────────────────────────────────────────────────────────────────────────────────────┤
│                            Core Layer (dynalgo_core)                                 │
│                          ┌─────────────────────────────────┐                         │
│  dynalgo_core            │  SDK-neutral types & interfaces  │  No vendor deps         │
│  (static lib)            │  No OrbbecSDK / rs_driver        │                         │
│                          └─────────────────────────────────┘                         │
│                                                                                      │
│  Types:  DynalgoFrameType, DynalgoFormat, DynalgoIntrinsic, DynalgoAlignMode        │
│  Frames: DynalgoFrame, DynalgoFrameSet (getFrame accessor)                          │
│  Abstracts: DynalgoDevice, DynalgoPipeline, DynalgoContext                          │
│  Model: DynalgoModelBackend, DynalgoModelConfig, DynalgoDetectionResult             │
│  Actuator: DynalgoActuator, DynalgoActuatorConfig, DynalgoActuatorType              │
│  Factory: createModelBackend(), createActuator() (self-registration)                │
│  Infra: DYNALGO_LOG_*, signalHandler(), mkdirp(), getTimestampMs()                  │
│  Algo: DynalgoKalmanTracker (6D KF), detectionCenterToCamera3D()                   │
└──────────────────────────────────────────────────────────────────────────────────────┘
```

### Design Principles

- **Strict layering**: Dependencies flow downward only. Upper layers never reference vendor SDKs directly.
- **Vendor isolation**: OrbbecSDK and rs_driver headers are confined to `app/driver/<vendor>/`. All other code communicates through abstract `DynalgoDevice` / `DynalgoPipeline` / `DynalgoContext` interfaces.
- **Self-registration**: Model backends and actuators register themselves at static-init time via `registerModelBackend()` / `registerActuator()`. New backends only need to be added to the appropriate static lib's CMakeLists.txt.
- **Dry-run safety**: Actuator `dryRun=true` by default. All control actions are no-ops until explicitly enabled.

### Data Flow

```
Device (USB)
    │
    ▼
Driver Layer (Orbbec / RoboSense)
    │  raw frames: COLOR, DEPTH, IR_L, IR_R, IMU, POINT
    ▼
Capture Session
    │  ├── FrameQueue → StreamTasks → H264Encoder → StreamIO (file)
    │  ├── SDLViewer (preview)
    │  └── FrameConsumer chain (optional)
    │         │
    │         ▼
    │    EngagementFrameConsumer
    │         │
    │         ▼
    │    EngagementLoop.onFrame()
    │         │
    │         ├── TargetSelector.pickTarget(detections, strategy)
    │         ├── TrackBundle.init/update (Kalman filter)
    │         ├── detectionCenterToCamera3D() → 3D fix
    │         └── Actuator.fire() (if state == FIRING)
    │
    ▼
Output Files (.h264, .raw, .txt, .csv)
```

### app/ Directory Structure

The `app/` directory contains all C++ source code organized in a **strictly layered architecture** with downward-only dependencies:

```
app/
├── core/                    # Core Layer (dynalgo_core)
│   ├── dynalgo_*_factory.*  # Factory + self-registration
│   ├── dynalgo_*.hpp        # Frame, Types, Model, Actuator, Device abstractions
│   └── utils*               # Logging, timestamps, threading
│
├── driver/                  # Driver Layer (dynalgo_drivers)
│   ├── orbbec/              # OrbbecSDK adapter
│   ├── robosense/           # RoboSense rs_driver adapter
│   └── stereo/              # Stereo camera abstraction (optional)
│
├── capture/                 # Capture Layer (dynalgo_capture)
│   ├── dynalgo_capture_session.*    # Session orchestration
│   ├── dynalgo_h264_encoder.*       # H.264 encoding
│   ├── dynalgo_stream_io.*          # File writing
│   ├── dynalgo_frame_queue.*        # Lock-free SPSC queues
│   ├── dynalgo_stream_tasks.*       # Stream task management
│   ├── dynalgo_sdl_viewer.*         # SDL preview
│   ├── dynalgo_color_convert.*      # Color space conversion
│   └── dynalgo_frame_consumer.*     # FrameConsumer chain
│
├── algo/                    # Algorithm Layer (dynalgo_algo) — optional
│   ├── bytetrack/           # ByteTrack multi-object tracking
│   ├── sahi/                # SAHI sliced inference for small objects
│   ├── dynalgo_engagement_loop.*    # Perceive→Locate→Estimate→Control
│   ├── dynalgo_engagement_consumer.*
│   ├── dynalgo_target_selector.*    # Target selection strategies
│   ├── dynalgo_track_bundle.*       # Kalman + 3D cache
│   └── dummy_model_backend.*        # Dry-run model stub
│
├── model_backends/          # Inference Backends (optional, ENABLE_MODEL_BACKENDS)
│   ├── common/              # Shared preprocessing, NMS, postprocess
│   ├── tensorrt/            # TensorRT C++ backend
│   ├── onnxruntime/         # ONNX Runtime C++ backend
│   └── rknn/                # RKNN Runtime C++ backend
│
├── actuator/                # Actuator Layer (optional, --engage-actuator)
│   ├── dynalgo_actuator.*   # Abstract interface
│   ├── dummy_actuator.*     # Dry-run implementation
│   └── CMakeLists.txt
│
├── dynamic_algo_cam/        # Main executable entry point
│   └── dynamic_algo_cam.cpp
│
├── plugins/                 # Optional plugins
│   └── opencv/              # OpenCV color conversion plugin
│
├── models/                  # Vendored Python model packages (NOT built by CMake)
│   └── yolov8/              # Ultralytics YOLOv8 (GPL-3.0)
│
├── training/                # Python training pipeline (NOT built by CMake)
│   ├── scripts/             # train, export, validate, convert
│   └── configs/             # YOLOv11 mosquito/weed, RT-DETR configs
│
└── tools/                   # Python post-processing tools
    └── parse_*.py, stream_*.py, evaluate_*.py
```

**Dependency Flow (strictly downward):**

```
dynamic_algo_cam
       │
       ▼
┌──────────────────┐
│  capture + algo  │  ← depend on core + actuator
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│     driver       │  ← depends on core
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│      core        │  ← zero vendor dependencies
└──────────────────┘
```

**Key Principles:**
- **Vendor isolation**: OrbbecSDK / rs_driver headers only in `app/driver/<vendor>/`
- **Self-registration**: Model backends & actuators register at static-init via factory hooks
- **Dry-run safety**: `dryRun=true` by default; all control actions are no-ops until explicitly enabled
- **Optional layers**: `algo/`, `model_backends/`, `actuator/`, `driver/stereo/` only linked when corresponding CMake options or CLI flags are enabled

### UML Sequence Diagram

The following sequence diagram illustrates the key interactions during a capture session with the engagement loop enabled (`--engage-model DUMMY --engage-actuator DUMMY`):

```mermaid
sequenceDiagram
    autonumber
    actor User
    participant CLI as dynamic_algo_cam
    participant Factory as DynalgoDriverFactory
    participant Device as DynalgoDevice (Orbbec/RS)
    participant Pipeline as DynalgoPipeline
    participant Session as DynalgoCaptureSession
    participant Encoder as DynalgoH264Encoder
    participant Writer as DynalgoStreamIO
    participant Viewer as DynalgoSDLViewer
    participant Consumer as EngagementFrameConsumer
    participant Loop as DynalgoEngagementLoop
    participant Model as DynalgoModelBackend
    participant Selector as TargetSelector
    participant Tracker as DynalgoTrackBundle
    participant Actuator as DynalgoActuator

    rect rgb(240, 248, 255)
        note right of CLI: Startup & Device Discovery
    end
    User->>CLI: ./dynamic_algo_cam --engage-model DUMMY --engage-actuator DUMMY
    CLI->>Factory: discoverDevices()
    Factory->>Device: scan USB / enumerate
    Device-->>Factory: vector<DiscoveredDevice>
    Factory-->>CLI: discovered devices

    rect rgb(255, 248, 240)
        note right of CLI: Pipeline Setup
    end
    CLI->>Session: setupPipeline(device, config)
    Session->>Pipeline: setupPipeline(config)
    Pipeline->>Device: enable streams (color, depth, IR, IMU)
    Device-->>Pipeline: sensor profiles + intrinsics
    Pipeline-->>Session: DynalgoSensorInfo (depthIntrinsic, depthScale)
    Session->>Model: createModelBackend(DUMMY)
    Session->>Actuator: createActuator(DUMMY)
    Session->>Loop: new EngagementLoop(model, actuator, selector, tracker)
    Session->>Consumer: new EngagementFrameConsumer(loop)
    Session->>Session: addFrameConsumer(consumer)

    rect rgb(240, 255, 240)
        note right of Pipeline: Capture Loop (per frame)
    end
    loop for each frame
        Pipeline->>Device: waitForFrames() / callback
        Device-->>Pipeline: raw vendor frames
        Pipeline->>Pipeline: convert to DynalgoFrameSet (deep copy)
        Pipeline-->>Session: videoCallback(frameSet)

        par Recording Path
            Session->>Encoder: encode(frameSet.color, frameSet.depth, ...)
            Encoder-->>Writer: H.264 packets
            Writer->>Writer: write to .h264 files
        and Preview Path
            Session->>Viewer: render(frameSet)
        and Engagement Path (optional)
            Session->>Consumer: consume(frameSet)
            Consumer->>Loop: onFrame(frameSet)
            Loop->>Model: infer(frameSet.color)
            Model-->>Loop: vector<DynalgoDetectionResult>
            Loop->>Selector: pickTarget(detections, strategy)
            Selector-->>Loop: optional<DynalgoDetectionResult>
            alt target found
                Loop->>Tracker: update(detection, depthFrame, intr, scale)
                Tracker->>Tracker: Kalman predict/update
                Tracker-->>Loop: hasFix(), lastX/Y/Z()
                Loop->>Loop: stateMachine(IDLE→LOCKING→TRACKING→FIRING)
                alt state == FIRING
                    Loop->>Actuator: aimAt(X, Y, Z)
                    Loop->>Actuator: fire(durationMs)
                end
            else no target
                Loop->>Loop: stateMachine(TRACKING→LOST→IDLE)
            end
        end
    end

    rect rgb(255, 240, 248)
        note right of CLI: Shutdown (Ctrl+C / SIGTERM)
    end
    User->>CLI: SIGINT / SIGTERM
    CLI->>Session: stop()
    Session->>Consumer: stopTask()
    Session->>Pipeline: stop()
    Pipeline->>Device: stop streams
    Session->>Encoder: flush()
    Session->>Writer: close files
    Session->>Actuator: close()
    Session->>Model: (cleanup)
    CLI->>User: "All recordings saved to: <dir>"
```

> **Note**: The engagement loop path is only active when both `--engage-model` and `--engage-actuator` are provided. Without these flags, the `FrameConsumer` chain remains empty and the capture session behaves identically to the baseline (zero overhead).

## Build

### Prerequisites

| Dependency | Required | Source |
|---|---|---|
| CMake >= 3.10 | Yes | System package |
| C++14 compiler (C++17 for algo) | Yes | GCC / Clang |
| FFmpeg (libavcodec, libavutil, libswscale, libavformat, libswresample) | Yes | pkg-config |
| SDL2 | Yes | pkg-config |
| pthreads | Yes | System |
| OpenCV | No | Optional — enables `dynalgo_opencv_plugin` |
| GTest | No | Optional — enables `tests/` (if `BUILD_TESTS=ON`) |

### Vendor SDK Options

Options are declared in the **root** `CMakeLists.txt`. At least one must be ON.

| CMake Option | Default | Effect |
|---|---|---|
| `ENABLE_ORBBEC` | ON | Build OrbbecSDK adapter, link `ob::OrbbecSDK` |
| `ENABLE_RS_AC1` | ON | Build rs_driver adapter, link `usb-ac-static` / `uvc-ac-static` |

### Model Backend Options

Options are declared in `app/model_backends/CMakeLists.txt`. These are optional and require corresponding SDKs.

| CMake Option | Default | Effect |
|---|---|---|
| `ENABLE_MODEL_BACKENDS` | OFF | Enable model inference backends |
| `ENABLE_TENSORRT` | OFF | Build TensorRT backend (requires CUDA + TensorRT SDK) |
| `ENABLE_ONNXRUNTIME` | OFF | Build ONNX Runtime backend (requires ONNX Runtime SDK) |
| `ENABLE_RKNN` | OFF | Build RKNN backend (requires Rockchip RKNN SDK) |

### CUDA Acceleration Options

Options are declared in `app/model_backends/common/CMakeLists.txt` and root `CMakeLists.txt`. These enable GPU-accelerated preprocessing and NMS.

| CMake Option | Default | Effect |
|---|---|---|
| `ENABLE_CUDA` | OFF | Enable CUDA acceleration for preprocessing and NMS (requires CUDA Toolkit) |

When `ENABLE_CUDA=ON`, the following operations are accelerated on GPU:
- **Preprocessing**: Letterbox resize + BGR→RGB conversion + normalization + HWC→CHW layout conversion
- **NMS (Non-Maximum Suppression)**: Parallel IoU computation with shared memory optimization

### Stereo Camera Options

Options are declared in `app/driver/stereo/CMakeLists.txt`. These enable stereo camera support.

| CMake Option | Default | Effect |
|---|---|---|
| `ENABLE_STEREO` | OFF | Enable stereo camera support (includes ENABLE_STEREO_UVC) |
| `ENABLE_STEREO_UVC` | ON | Enable generic UVC stereo camera driver |
| `ENABLE_ZED` | OFF | Enable Stereolabs ZED SDK support |
| `ENABLE_MYNT_EYE` | OFF | Enable MYNT EYE SDK support |
| `ENABLE_DEPTHAI` | OFF | Enable Luxonis DepthAI/OAK support |

When `ENABLE_STEREO=ON`, the stereo driver layer is built with support for:
- Generic UVC stereo cameras (dual UVC devices)
- Calibration loading from OpenCV YAML/XML
- Stereo rectification (undistort + rectify)
- SGBM stereo matching for disparity/depth

### Build Steps

```bash
mkdir -p build && cd build

# Default: both vendors enabled
cmake ..

# Orbbec only
cmake .. -DENABLE_RS_AC1=OFF

# RS-AC1 only
cmake .. -DENABLE_ORBBEC=OFF

# With TensorRT backend (requires CUDA + TensorRT SDK)
cmake .. -DENABLE_MODEL_BACKENDS=ON -DENABLE_TENSORRT=ON

# With ONNX Runtime backend
cmake .. -DENABLE_MODEL_BACKENDS=ON -DENABLE_ONNXRUNTIME=ON

# With RKNN backend (requires Rockchip RKNN SDK)
cmake .. -DENABLE_MODEL_BACKENDS=ON -DENABLE_RKNN=ON

# All backends
cmake .. -DENABLE_MODEL_BACKENDS=ON -DENABLE_TENSORRT=ON -DENABLE_ONNXRUNTIME=ON -DENABLE_RKNN=ON

# With CUDA acceleration for preprocessing/NMS
cmake .. -DENABLE_CUDA=ON -DENABLE_MODEL_BACKENDS=ON -DENABLE_TENSORRT=ON

# All backends with CUDA acceleration
cmake .. -DENABLE_CUDA=ON -DENABLE_MODEL_BACKENDS=ON -DENABLE_TENSORRT=ON -DENABLE_ONNXRUNTIME=ON -DENABLE_RKNN=ON

# With stereo camera support
cmake .. -DENABLE_STEREO=ON

# With stereo + CUDA + TensorRT
cmake .. -DENABLE_STEREO=ON -DENABLE_CUDA=ON -DENABLE_MODEL_BACKENDS=ON -DENABLE_TENSORRT=ON

cmake --build . -j$(nproc)
```

The executable is `build/bin/dynamic_algo_cam`. Install with `cmake --install .` (installs to `bin/`).

**Note:** The Algorithm Layer (`dynalgo_algo`) and Actuator Layer (`dynalgo_actuators`) are built as static libraries but are only linked into the executable when the `--engage-model` and/or `--engage-actuator` CLI flags are provided. Without these flags, the executable behaves identically to the baseline (no engagement loop overhead).

### Runtime Library Path

OrbbecSDK ships `libOrbbecSDK.so`. Either install it to a system path or set:
```bash
export LD_LIBRARY_PATH=/path/to/orbbeccamera/build/linux_x86_64/lib:$LD_LIBRARY_PATH
```

RS-AC1 dependencies are statically linked — no runtime `.so` needed for rs_driver.

## Usage

```bash
# Record all connected devices
./dynamic_algo_cam

# Filter by device name substring
./dynamic_algo_cam -c "305" "336L"

# Custom save directory
./dynamic_algo_cam -s /HDD/dynalgo_capture

# Adjust D2C fusion parameters
./dynamic_algo_cam --alpha 0.7 --depth-min 0.2 --depth-max 3.0

# Headless (no SDL window)
./dynamic_algo_cam --no-show

# Disable D2C fusion output
./dynamic_algo_cam --no-fusion

# Dry-run engagement loop (DUMMY model + DUMMY actuator)
./dynamic_algo_cam --engage-model DUMMY --engage-actuator DUMMY --no-show

# Dry-run with synthetic events
./dynamic_algo_cam --engage-model DUMMY --engage-actuator DUMMY --enable-event-sim

# Engagement loop with TensorRT backend
./dynamic_algo_cam --engage-model TENSORRT --engage-actuator DUMMY --engage-model-path model.engine --no-show

# Engagement loop with ONNX Runtime backend
./dynamic_algo_cam --engage-model ONNXRUNTIME --engage-actuator DUMMY --engage-model-path model.onnx --no-show

# Engagement loop with RKNN backend
./dynamic_algo_cam --engage-model RKNN --engage-actuator DUMMY --engage-model-path model.rknn --no-show

# Stereo camera capture with calibration
./dynamic_algo_cam --stereo-config stereo_calib.yaml --stereo-compute-depth --no-show
```

### CLI Parameters

| Parameter | Default | Description |
|---|---|---|
| `-c <name...>` | All devices | Filter by device name substring |
| `-s <dir>` | `capture_output/` | Save directory |
| `--alpha VAL` | 0.5 | Depth overlay opacity (0=raw color, 1=raw depth colorized) |
| `--depth-min M` | 0.3 | Min depth for jet colormap (meters) |
| `--depth-max M` | 5.0 | Max depth for jet colormap (meters) |
| `--no-fusion` | off | Skip D2C fusion output |
| `--no-show` | off | Disable SDL preview window |
| `--engage-model TYPE` | (empty) | Model backend: NONE, DUMMY, YOLOV8_PY, ONNXRUNTIME, TENSORRT, RKNN |
| `--engage-actuator TYPE` | (empty) | Actuator backend: NONE, DUMMY, LASER_GENERIC, GIMBAL_GENERIC |
| `--engage-model-path PATH` | (empty) | Path to model weights (used by non-DUMMY backends) |
| `--help` | — | Show help |

### Output Structure

```
<saveDir>/<sessionTimestamp>/<deviceName>/
├── <name>_color_<ts>.h264
├── <name>_depth_<ts>.h264
├── <name>_depth_raw_<ts>.raw
├── <name>_ir_left_<ts>.h264     (Orbbec only)
├── <name>_ir_right_<ts>.h264    (Orbbec only)
├── <name>_imu_<ts>.txt          (if device has IMU)
├── <name>_d2c_fused_<ts>.h264   (if fusion enabled)
└── <name>_point_raw_<ts>.raw    (RS-AC1 only)
```

### Stopping

`Ctrl+C` or `SIGTERM` triggers graceful shutdown: stop SDK pipelines → flush encoders → close files.

## Pre-Capture Setup

### USB Buffer (Required for Multi-Device)

```bash
# Check current value
cat /sys/module/usbcore/parameters/usbfs_memory_mb

# Set to 256 MB (temporary)
echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb

# Permanent
echo "options usbcore usbfs_memory_mb=256" | sudo tee /etc/modprobe.d/usbcore.conf
```

### udev Rules (Orbbec)

```bash
sudo cp vendors/OrbbecSDK/scripts/99-obsensor-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### RS-AC1 USB Unbind (Required)

RS-AC1 uses a custom libuvc. The kernel `uvcvideo` driver must be unbound:

```bash
# Find device
lsusb | grep 3840:1010

# Unbind (replace with actual bus/device path)
echo '1-5' | sudo tee /sys/bus/usb/drivers/uvcvideo/unbind 2>/dev/null || true
```

## Post-Processing Tools

```bash
# Depth raw parser
python3 app/tools/parse_depth_raw.py <file.raw> --stats
python3 app/tools/parse_depth_raw.py <file.raw> --output vis --all

# Point cloud raw parser (RS-AC1)
python3 app/tools/parse_point_raw.py <file.raw> --info
python3 app/tools/parse_point_raw.py <file.raw> --frame 0

# H.264 playback
ffplay -f h264 <file>.h264

# Wrap H.264 into MP4
ffmpeg -y -fflags +genpts -r 30 -i <file>.h264 -c copy output.mp4
```

## Documentation

| Document | Content |
|---|---|
| [docs/device_comparison.md](docs/device_comparison.md) | Hardware specs, stream profiles, output format differences per device |
| [docs/VENDOR_DEVICE_PORTING_MANUAL.md](docs/VENDOR_DEVICE_PORTING_MANUAL.md) | How to add a new vendor device adapter |
| [docs/dynamic_algo_cam/use_guide.md](docs/dynamic_algo_cam/use_guide.md) | Detailed usage guide (Chinese) |
| [docs/dynamic_algo_cam/troubleshooting.md](docs/dynamic_algo_cam/troubleshooting.md) | Troubleshooting reference |
| [docs/dynamic_algo_cam/dynamic_algo_cam_technical_reference.md](docs/dynamic_algo_cam/dynamic_algo_cam_technical_reference.md) | Technical reference (architecture, algorithms, data formats) |
| [docs/dynamic_algo_cam/models_overview.md](docs/dynamic_algo_cam/models_overview.md) | Algorithm / inference model packages under `app/models/` (layout, usage, license disclosures) |
| [docs/dynamic_algo_cam/DEVELOPMENT_PLAN.md](docs/dynamic_algo_cam/DEVELOPMENT_PLAN.md) | Development plan (Phase A–D milestones) |
| [docs/dynamic_algo_cam/IMPLEMENTATION_TASKS.md](docs/dynamic_algo_cam/IMPLEMENTATION_TASKS.md) | Implementation task checklist with commit hashes |

## Packaging

```bash
./scripts/package.sh [--output DIR] [--skip-libs]
```

Creates a self-contained tar.gz with the binary, SDK libraries, udev rules, and launcher scripts.

## License

MIT License. See vendor SDKs under `vendors/` for their respective licenses.

### Embedded model packages — additional license disclosures

This project vendors third-party algorithm / inference model packages under
`app/models/`. Each subdirectory ships its upstream `LICENSE` and **is governed
by that license, not the project MIT license**:

| Subdir | License | Notes |
|---|---|---|
| `app/models/yolov8/` | **GPL-3.0** ([`app/models/yolov8/LICENSE`](app/models/yolov8/LICENSE)) | GPL-3.0 is **not compatible** with MIT. ANY distribution that incorporates YOLOv8 source code triggers GPL-3.0 obligations (corresponding source availability, notice propagation, copyleft terms). Removing or not exercising the YOLOv8 code avoids these obligations. |

See [`app/models/README.md`](app/models/README.md) for the rationale and rules for adding further model packages.
