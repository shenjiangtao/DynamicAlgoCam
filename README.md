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

### Build Steps

```bash
mkdir -p build && cd build

# Default: both vendors enabled
cmake ..

# Orbbec only
cmake .. -DENABLE_RS_AC1=OFF

# RS-AC1 only
cmake .. -DENABLE_ORBBEC=OFF

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
| `--engage-model TYPE` | (empty) | Model backend: NONE, DUMMY, YOLOV8_PY, ONNXRUNTIME, TENSORRT |
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
