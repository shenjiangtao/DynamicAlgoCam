# NIO Multi-Device Capture — Hardware Technical Comparison

> Orbbec Gemini 305 / 335L / 336L vs RoboSense RoboX AC1

## 1. Overview

| | Gemini 305 | Gemini 335L | Gemini 336L | RoboX AC1 |
|---|---|---|---|---|
| **Vendor** | Orbbec | Orbbec | Orbbec | RoboSense |
| **VID:PID** | `2BC5:0840` | `2BC5:0804` | `2BC5:0807` | `3840:1010` |
| **Device Family** | G305 | G330L | G330L | RS-AC1 |
| **SDK** | OrbbecSDK v2 | OrbbecSDK v2 | OrbbecSDK v2 | rs_driver (custom) |
| **Interface** | USB 2.1/3.0 | USB 2.1/3.0 | USB 2.1/3.0 | USB 3.0 only |
| **Sensor Type** | Structured-light depth camera | Structured-light depth camera | Structured-light depth camera | Solid-state flash lidar |

## 2. Sensor Capabilities

| Capability | Gemini 305 | Gemini 335L | Gemini 336L | RoboX AC1 |
|---|---|---|---|---|
| **Color (RGB)** | Yes | Yes | Yes | Yes |
| **Depth** | Yes | Yes | Yes | Yes (point-cloud derived) |
| **IR** | Yes (IR_LEFT + IR_RIGHT) | Yes (IR_LEFT + IR_RIGHT) | Yes (IR_LEFT + IR_RIGHT) | No |
| **IMU (Accel+Gyro)** | **No** | Yes (~200 Hz) | Yes (~200 Hz) | **Yes** (~100 Hz, via rs_driver HID) |
| **Point Cloud** | No (depth map only) | No (depth map only) | No (depth map only) | **Yes** (27,648 pts/frame) |

### Key Implications

- **Gemini 305** has no IMU — no `*_imu_*.txt` output file is created.
- **Gemini 335L/336L** have IMU — accel and gyro at ~200 Hz → `*_imu_*.txt` with full CSV data.
- **RoboX AC1** IMU delivers ~100 Hz accel + gyro data via rs_driver HID callback. Temperature field is 0.0 (rs_driver does not expose temperature). IMU data quality: gyro noise ~0.01 rad/s (vs 335L ~0.002 rad/s).
- **RoboX AC1** has no IR sensors — no `*_ir_*.h264` / `*_ir_left_*.h264` / `*_ir_right_*.h264` files are created.
- **RoboX AC1** is the only device that outputs point cloud data → `*_point_raw_*.raw` file.

## 3. Stream Profiles

### 3.1 Color

| | Gemini 305 | Gemini 335L | Gemini 336L | RoboX AC1 |
|---|---|---|---|---|
| **Default Resolution** | 848×530 | 1280×720 | 1280×720 | 1920×1080 |
| **Format** | YUYV | MJPG | MJPG | NV12 |
| **FPS** | 10 (USB2) / 30 (GMSL) | 10 (USB2) / 30 (GMSL) | 10 (USB2) / 30 (GMSL) | 30 |
| **NioFormat** | `YUYV` | `MJPG` | `MJPG` | `NV12` |

- 305 prefers YUYV on both USB and GMSL (`app/driver/orbbec/nio_ob_device.cpp:85`).
- 335L/336L prefer MJPG on USB (higher compression at 1280×720).
- AC1 NV12 is decoded via sws_scale before H.264 encode (`app/capture/nio_capture_session.cpp:83`).

### 3.2 Depth

| | Gemini 305 | Gemini 335L | Gemini 336L | RoboX AC1 |
|---|---|---|---|---|
| **Resolution** | 848×530 | 848×480 | 848×480 | 96×288 (27,648 pts) |
| **Format** | Y16 (uint16) | Y16 (uint16) | Y16 (uint16) | Y16 (synthetic 2D grid) |
| **FPS** | 10 (USB2) / 30 | 10 (USB2) / 30 | 10 (USB2) / 30 | 10 |
| **Depth Scale** | 0.001–0.0001 m/step | 0.001–0.0001 m/step | 0.001–0.0001 m/step | 0.005 m/step (5 mm) |
| **Max Range** | ~5 m | ~5 m | ~5 m | **200 m** |
| **Depth File** | `*_depth_raw_*.raw` + `*_depth_*.h264` | Same | Same | `*_depth_raw_*.raw` + `*_depth_*.h264` + `*_point_raw_*.raw` |

- OB depth raw: `NIO_DEPTH_RAW` binary container, 2 bytes/pixel.
- AC1 depth raw: Same container format, but pixels are organized as 96×288 grid from lidar points.
- AC1 additionally outputs `*_point_raw_*.raw`: `NIO_POINT_CLOUD_RAW` binary with 26B/point (float xyz + float intensity + uint16 ring + double timestamp).
- OB depth granularity: 1 mm (depthScale=0.001) with 4 precision levels down to 0.1 mm.
- AC1 depth granularity: 5 mm (depthScale=5), lidar-native distance resolution 5 mm.

### 3.3 IR

| | Gemini 305 | Gemini 335L | Gemini 336L | RoboX AC1 |
|---|---|---|---|---|
| **IR Type** | IR_LEFT + IR_RIGHT | IR_LEFT + IR_RIGHT | IR_LEFT + IR_RIGHT | — |
| **Resolution** | 848×530 | 848×480 | 848×480 | — |
| **Format** | Y8 | Y8 | Y8 | — |
| **FPS** | 10 (USB2) / 30 (GMSL) | 10 (USB2) / 30 (GMSL) | 10 (USB2) / 30 (GMSL) | — |
| **Output** | `*_ir_left_*.h264` + `*_ir_right_*.h264` | Same | Same | — |

- 305g (GMSL) may disable IR_LEFT as a hardware quirk (`app/driver/orbbec/nio_ob_device.cpp:224-228`).

### 3.4 IMU

| | Gemini 305 | Gemini 335L | Gemini 336L | RoboX AC1 |
|---|---|---|---|---|
| **Accelerometer** | No | Yes | Yes | Yes (~100 Hz) |
| **Gyroscope** | No | Yes | Yes | Yes (~100 Hz) |
| **Sample Rate** | — | ~200 Hz | ~200 Hz | ~100 Hz |
| **Full Scale (Accel)** | — | 2g / 8g | 2g / 8g | Fixed |
| **Full Scale (Gyro)** | — | 250 dps | 250 dps | Fixed |
| **Temperature** | — | Yes (~30°C) | Yes (~30°C) | No (0.0) |
| **Output** | — | `*_imu_*.txt` | `*_imu_*.txt` | `*_imu_*.txt` |

- IMU CSV format: `# host_ts_ms,type,device_ts_us,x,y,z,temperature`
- Type field: `ACCEL` or `GYRO`
- OB IMU source: `SOURCE_PORT_USB_HID` (G330Device.cpp:641-694)
- AC1 IMU is active via rs_driver HID — ~100 Hz accel+gyro. Temperature field is always 0.0.

### 3.5 Point Cloud (AC1 only)

| Property | Value |
|---|---|
| Point type | `PointXYZIRT` |
| Fields | `x y z intensity ring timestamp` |
| Grid | 96×288 = 27,648 points/frame |
| Wire format | 23 bytes/point (float xyz + uint8 intensity + uint16 ring + double timestamp) |
| Raw container format | 26 bytes/point (intensity promoted to float32 for alignment) |
| Distance range | 0.2–200 m |
| Distance resolution | 5 mm |
| Vector base | 32768 (direction normalization divisor) |
| FPS | 10 |
| Output | `*_point_raw_*.raw` (`NIO_POINT_CLOUD_RAW` container) |

Container binary layout (68B file header + 32B frame header + data per frame):
```
[File Header - 68 bytes, frame 0 only]
  [20B] magic = "NIO_POINT_CLOUD_RAW\0"
  [4B]  version = 1
  [4B]  fieldCount = 6
  [40B] fieldNames = "x\0y\0z\0intensity\0ring\0timestamp" (padded)

[Frame Header - 32 bytes, every frame]
  [8B] frameIndex (uint64)
  [8B] timestampUs (uint64)
  [4B] pointCount (uint32)
  [4B] pointDataBytes (uint32) = pointCount × 26
  [8B] reserved (uint64, zero)

[Frame Data - pointCount × 26 bytes each]
  float x(4) + float y(4) + float z(4) + float intensity(4) + uint16 ring(2) + double timestamp(8)
```

Parse with: `python3 app/tools/parse_point_raw.py <file.raw> [--frame N|--all|--info]`

## 4. D2C Depth-to-Color Fusion

| | Gemini 305 | Gemini 335L | Gemini 336L | RoboX AC1 |
|---|---|---|---|---|
| **HW D2C** | Yes (if profile matches) | Yes (if profile matches) | Yes (if profile matches) | Always HW |
| **Fallback** | SW align via `ob::Align` | Same | Same | — |
| **Fusion Resolution** | Color resolution | Color resolution | Color resolution | 1920×1080 |
| **Fusion FPS** | `min(colorFps, depthFps)` | Same | Same | `min(30, 10) = 10` |
| **Alpha** | 0.5 (default) | 0.5 | 0.5 | 0.5 |
| **Depth Range for Colormap** | 0.3–5 m | 0.3–5 m | 0.3–5 m | 0.3–5 m |
| **Output** | `*_d2c_fused_*.h264` | Same | Same | `*_d2c_fused_*.h264` |

- OB D2C: firmware-provided profile list checked at pipeline start. If no HW match, falls back to SW alignment.
- AC1 D2C: always HW (lidar-camera factory-calibrated). Upsampled from 96×288 to 1920×1080 — visible block artifacts at close range.

## 5. Output File Summary

| File | Gemini 305 | Gemini 335L | Gemini 336L | RoboX AC1 | Format |
|---|---|---|---|---|---|
| `*_color_*.h264` | Yes | Yes | Yes | Yes | H.264 encoded |
| `*_depth_*.h264` | Yes | Yes | Yes | Yes | H.264 encoded |
| `*_depth_raw_*.raw` | Yes | Yes | Yes | Yes | `NIO_DEPTH_RAW` binary |
| `*_ir_left_*.h264` | Yes | Yes | Yes | — | H.264 encoded |
| `*_ir_right_*.h264` | Yes | Yes | Yes | — | H.264 encoded |
| `*_imu_*.txt` | — | Yes | Yes | Yes | CSV (`host_ts_ms,type,...`) |
| `*_d2c_fused_*.h264` | Yes | Yes | Yes | Yes | H.264 encoded |
| `*_point_raw_*.raw` | — | — | — | Yes | `NIO_POINT_CLOUD_RAW` binary |
| `*_depth_intrinsic_*.json` | Yes | Yes | Yes | Yes | JSON (depth+color+lidar) |

### Intrinsic JSON Differences

**Orbbec devices** produce:
```json
{
  "depth": {"fx":373.68, "fy":373.68, "cx":424, "cy":240, "width":848, "height":480},
  "color": {"fx":368.43, "fy":368.43, "cx":640, "cy":360, "width":1280, "height":720},
  "depth_scale": 1,
  "device": "Orbbec_Gemini_335L_CP26363000FB"
}
```

**RoboX AC1** produces (no focal-length intrinsics, lidar section added):
```json
{
  "depth": {"fx":0, "fy":0, "cx":0, "cy":0, "width":0, "height":0},
  "color": {"fx":0, "fy":0, "cx":0, "cy":0, "width":0, "height":0},
  "depth_scale": 5,
  "lidar": {
    "type": "RS-AC1",
    "point_grid_width": 96,
    "point_grid_height": 288,
    "distance_min_m": 0.2,
    "distance_max_m": 200.0,
    "distance_resolution_m": 0.005,
    "vector_base": 32768,
    "point_fields": "x y z intensity ring timestamp",
    "point_type": "PointXYZIRT"
  },
  "device": "RoboSense_AC1_1111bfa90090"
}
```

## 6. System Requirements & Prerequisites

| | Gemini 305/335L/336L | RoboX AC1 |
|---|---|---|
| **USB** | USB 2.0+ (USB 3.0 recommended) | **USB 3.0 required** |
| **Kernel driver** | `uvcvideo` (auto-binds) | Must **unbind** `uvcvideo` before use (custom libuvc) |
| **udev rule** | Orbbec SDK includes one | `SUBSYSTEM=="usb", ATTR{idVendor}=="3840", ATTR{idProduct}=="1010", MODE="0666"` at `/etc/udev/rules.d/99-robosense-ac1.rules` |
| **SDK libs** | `libOrbbecSDK.so` | `librsac_usb.so` + `libuvc.so` + `libusb-1.0.so` |
| **Compile flags** | — | `-DENABLE_RS_AC1 -DENABLE_USB -DENABLE_IMU_PARSE -DENABLE_IMAGE_PARSE` |
| **CMake** | `ENABLE_ORBBEC=ON` (root CMakeLists.txt) | `ENABLE_RS_AC1=ON` (root CMakeLists.txt, default ON) |
| **Device serial** | Read by OrbbecSDK | USB string descriptor `iSerialNumber` (e.g. `1111bfa90090`) |

### AC1 USB Unbind (required before capture)

```bash
# Find the device
lsusb | grep 3840:1010
# Bus 002 Device 010: ID 3840:1010

# Unbind kernel UVC driver (replace bus/device numbers)
echo '1-5' | sudo tee /sys/bus/usb/drivers/uvcvideo/unbind 2>/dev/null || true

# Verify: the device should still be visible but no uvcvideo binding
lsusb -v -d 3840:1010 | grep bcdUSB
# bcdUSB            3.00
```

## 7. Troubleshooting

| Symptom | Device | Cause | Fix |
|---|---|---|---|
| "Device is not found, please check!" | AC1 | `device_uuid` mismatch (bus-device instead of serial) | Use USB serial number descriptor as UUID (`app/driver/robosense/nio_rs_device.cpp:323`) |
| "Device is not found, please check!" | AC1 | USB permissions (`crw-rw-r-- root root`) | Add udev rule with `MODE="0666"` |
| "Wrong Input Type 4" | AC1 | `ENABLE_USB` not defined at compile time | Ensure `-DENABLE_USB` in compile definitions (`app/driver/CMakeLists.txt:44`) |
| All AC1 output files 0-byte | AC1 | Pipeline init failed (above errors) | Fix above issues first |
| IMU temperature=0.0 | AC1 | rs_driver does not expose temperature data | Expected — temperature field always 0.0 |
| Core dump on exit | AC1 | `stop()` called on uninitialized driver | Fixed: `running_=false` before thread joins (`app/driver/robosense/nio_rs_device.cpp:202`) |
| Block artifacts in D2C | AC1 | 96×288 upsampled to 1920×1080 | Inherent limitation; accept or adjust alpha/depth range |
| IR files empty | AC1 | AC1 has no IR sensors | Expected — IR file creation is skipped (`hasIR=false`) |
| Depth raw magic mismatch | AC1 | Old 16B magic truncation in writer | Fixed: magic renamed to `NIO_DEPTH_RAW` (vendor-neutral); parsers accept both old `ORBBEC_DEPTH_RAW` and new `NIO_DEPTH_RAW` |
| MJPEG format warnings | OB | `yuvj422p` deprecated pixel format | Harness auto-recreates sws context with correct range (`app/capture/nio_h264_encoder.cpp:326`) |

## 8. Typical Output Sizes (15 s capture)

| File | Gemini 335L (USB3) | RoboX AC1 |
|---|---|---|
| `color.h264` | ~8 MB | ~4 MB |
| `depth.h264` | ~8 MB | ~100–150 KB (low-res grid) |
| `depth_raw.raw` | ~230 MB | ~6 MB (96×288×2B × ~150 frames) |
| `ir_left.h264` | ~8 MB | — |
| `ir_right.h264` | ~8 MB | — |
| `imu.txt` | ~300–500 KB | — |
| `d2c_fused.h264` | ~2–3 MB | ~4–5 MB |
| `point_raw.raw` | — | ~75–100 MB (27,648 pts × 26B × ~120 frames) |
| **Total** | **~280 MB** | **~90 MB** |

## 9. Device Schema in Code

```
NioDevice (interface)
├── ObDevice   — wraps ob::Device, queries OrbbecSDK sensor list
│                 getSensorInfo() → dynamic from SDK
│                 hasIRSensor() → si.hasIR || si.hasIRLeft || si.hasIRRight
│                 setupPipeline() → selects profiles, returns NioSensorInfo
│
└── RsDevice   — hardcoded specs for RoboSense AC1 (from nio_rs_spec.hpp::AC1)
                  getSensorInfo() → static (hasAccel=true, hasGyro=true)
                  hasIRSensor() → false
                  setupPipeline() → returns fixed NioSensorInfo
                  validateStream() → uses rs::AC1::COLOR/DEPTH constants

NioContext (interface)
├── ObContext  — ob::Context 自动枚举
└── RsContext  — libusb 手动扫描，匹配 VID=0x3840/PID=0x1010

ConfigValidator (abstract)
├── ObValidator — validates Orbbec stream config, depth precision, device VID
└── RsValidator — validates AC1 fixed parameters (1920x1080/30fps, etc.)

DriverFactory — discovers devices with optional vendor filter (DriverVendor::ALL/ORBBEC/ROBOSENSE)
```

Output file creation is gated by `NioSensorInfo` flags:

| File | Gate |
|---|---|
| `*_color_*.h264` | `hasColor && colorFormat != UNKNOWN` |
| `*_depth_*.h264` + `*_depth_raw_*.raw` | `hasDepth && depthFormat != UNKNOWN` |
| `*_ir_left_*.h264` | `hasIRLeft && irLeftFormat != UNKNOWN` |
| `*_ir_right_*.h264` | `hasIRRight && irRightFormat != UNKNOWN` |
| `*_imu_*.txt` | `hasIMU()` = `hasAccel && hasGyro` |
| `*_point_raw_*.raw` | `pipeline->isPointCloudDepth()` |
| `*_d2c_fused_*.h264` | `canFuse` (= hasColor && hasDepth) |

## 10. Key Source References

| Component | Path |
|---|---|
| OB device adapter | `app/driver/orbbec/nio_ob_device.cpp` |
| OB format adapter | `app/driver/orbbec/nio_ob_adapter.hpp` |
| RS device adapter | `app/driver/robosense/nio_rs_device.cpp` |
| RS frame adapter | `app/driver/robosense/nio_rs_frame_adapter.hpp` |
| Capture session (file creation) | `app/capture/nio_capture_session.cpp` |
| Stream I/O (depth/point raw formats) | `app/capture/nio_stream_io.cpp` |
| Stream tasks | `app/capture/nio_stream_tasks.cpp` |
| Multi-capture main | `app/nio_multi_capture/nio_multi_capture.cpp` |
| Sensor info struct | `app/core/nio_types.hpp` |
| RS driver CMake | `app/driver/CMakeLists.txt` |
| Depth raw parser | `app/tools/parse_depth_raw.py` |
| Point raw parser | `app/tools/parse_point_raw.py` |
| OB SDK PID table | `src/device/DevicePids.hpp` |
| OB G305 sensor init | `src/device/gemini305/G305Device.cpp:538` |
| OB G330 sensor init | `src/device/gemini330/G330Device.cpp:380` |
| OB G330 IMU init | `src/device/gemini330/G330Device.cpp:641-694` |
| RS AC1 decoder | `RoboSense/rs_driver-dev_opt_AC1/src/rs_driver/driver/decoder/decoder_RSAC1.hpp` |
| RS input USB | `RoboSense/rs_driver-dev_opt_AC1/src/rs_driver/driver/input/input_usb.cpp` |
| RS AC1 spec | `app/driver/robosense/nio_rs_spec.hpp` | AC1 hardware constants (resolution, fps, format, USB ID) |
| Orbbec spec | `app/driver/orbbec/nio_ob_spec.hpp` | Depth precision mapping, color format policy |
| Type system | `app/core/nio_types.hpp` | `nio::types` namespace (NioFormat, NioFrameType) |
| Driver factory | `app/driver/nio_driver_factory.hpp` | `discoverDevices()` with vendor filter |
| Config validator | `app/core/nio_config_validator.hpp` | `ConfigValidator` interface + `createValidator()` factory |
| Orbbec validator | `app/driver/orbbec/nio_ob_validator.hpp` | Stream/depth-precision/device validation |
| RoboSense validator | `app/driver/robosense/nio_rs_validator.hpp` | AC1 fixed-parameter validation |
