# DynamicAlgoCam

DynamicAlgoCam 表示相机驱动的动态算法加载框架。DynamicAlgoCam 是一个基于 双目相机 和 激光雷达相机 的动态算法加载平台，旨在为多种任务提供统一的感知与执行框架。

目标包括：

- 提供 模块化算法加载机制，支持在运行时切换或添加任务算法。
- 利用 视觉 + 激光雷达融合，实现高精度目标检测与环境感知。
- 面向 特定任务场景（如灭蚊、除草、巡检），快速部署与验证。

## Supported Devices

| Device | VID | Sensors | Output Files |
|---|---|---|---|
| Orbbec Gemini 305 | `2bc5:0840` | Color, Depth, IR L+R | `.h264`, `.raw`, fusion `.h264` |
| Orbbec Gemini 335L | `2bc5:0804` | Color, Depth, IR L+R, IMU | `.h264`, `.raw`, `.txt`, fusion `.h264` |
| Orbbec Gemini 336L | `2bc5:0807` | Color, Depth, IR L+R, IMU | `.h264`, `.raw`, `.txt`, fusion `.h264` |
| RoboSense RS-AC1 | `3840:1010` | Color, Depth (point cloud), Point Cloud | `.h264`, `.raw`, fusion `.h264`, `*_point_raw_*.raw` |

See [docs/device_comparison.md](docs/device_comparison.md) for detailed hardware specs, stream profiles, and output format differences.

## Architecture

```
┌─────────────────────────────────────────────────┐
│  nio_multi_capture  (executable)                │
├─────────────────────────────────────────────────┤
│  nio_capture         (capture logic + encoding) │
├─────────────────────────────────────────────────┤
│  nio_drivers         (vendor SDK adapters)       │
├─────────────────────────────────────────────────┤
│  nio_core            (SDK-neutral types)        │
└─────────────────────────────────────────────────┘
  + nio_opencv_plugin (optional, if OpenCV found)
```

Vendor SDK headers (`libobsensor/`, `rs_driver/`) are **only** allowed in `app/driver/`. All other layers are SDK-agnostic, communicating through abstract `NioDevice` / `NioPipeline` / `NioContext` interfaces.

## Build

### Prerequisites

| Dependency | Required | Source |
|---|---|---|
| CMake >= 3.10 | Yes | System package |
| C++14 compiler | Yes | GCC / Clang |
| FFmpeg (libavcodec, libavutil, libswscale, libavformat, libswresample) | Yes | pkg-config |
| SDL2 | Yes | pkg-config |
| pthreads | Yes | System |
| OpenCV | No | Optional — enables `nio_opencv_plugin` |

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

The executable is `build/nio_multi_capture`. Install with `cmake --install .` (installs to `bin/`).

### Runtime Library Path

OrbbecSDK ships `libOrbbecSDK.so`. Either install it to a system path or set:
```bash
export LD_LIBRARY_PATH=/path/to/orbbeccamera/build/linux_x86_64/lib:$LD_LIBRARY_PATH
```

RS-AC1 dependencies are statically linked — no runtime `.so` needed for rs_driver.

## Usage

```bash
# Record all connected devices
./nio_multi_capture

# Filter by device name substring
./nio_multi_capture -c "305" "336L"

# Custom save directory
./nio_multi_capture -s /HDD/nio_capture

# Adjust D2C fusion parameters
./nio_multi_capture --alpha 0.7 --depth-min 0.2 --depth-max 3.0

# Headless (no SDL window)
./nio_multi_capture --no-show

# Disable D2C fusion output
./nio_multi_capture --no-fusion
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
| [docs/nio_multi_capture/use_guide.md](docs/nio_multi_capture/use_guide.md) | Detailed usage guide (Chinese) |
| [docs/nio_multi_capture/troubleshooting.md](docs/nio_multi_capture/troubleshooting.md) | Troubleshooting reference |
| [docs/nio_multi_capture/nio_multi_capture_technical_reference.md](docs/nio_multi_capture/nio_multi_capture_technical_reference.md) | Technical reference (architecture, algorithms, data formats) |

## Packaging

```bash
./scripts/package.sh [--output DIR] [--skip-libs]
```

Creates a self-contained tar.gz with the binary, SDK libraries, udev rules, and launcher scripts.

## License

MIT License. See vendor SDKs under `vendors/` for their respective licenses.
