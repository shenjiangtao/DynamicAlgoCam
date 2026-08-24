# Vendor Device Porting Manual

## 1. Architecture Overview

```
app/
├── core/           # SDK-neutral: DynalgoDevice, DynalgoPipeline, DynalgoFrame, DynalgoTypes
├── driver/         # SDK-specific adapters (ONLY layer that includes vendor SDK headers)
│   ├── orbbec/     # Orbbec SDK adapter
│   ├── robosense/  # RoboSense rs_driver adapter
│   └── dynalgo_driver_factory.hpp/cpp  # Device discovery factory
├── capture/         # CaptureSession — uses DynalgoDevice/DynalgoPipeline only
├── plugins/opencv/  # OpenCV utilities — uses DynalgoFrame/DynalgoFormat only
└── dynamic_algo_cam/ # Main app — uses discoverDevices() only
```

**Hard rule:** `ENABLE_ORBBEC` / `ENABLE_RS_AC1` macros, vendor SDK headers
(`libobsensor/`, `rs_driver/`, etc.), and vendor-specific constants
(e.g. `OB_DEVICE_VID = 0x2bc5`, vendor VID/PID device-type checks) are ONLY
allowed in `app/driver/`. Everything else in `app/` is SDK-agnostic.
`app/core/` must compile cleanly with zero vendor SDK dependencies.

---

## 2. Abstract Interfaces (app/core/dynalgo_device.hpp)

Every vendor adapter must implement these three classes:

### 2.0 Supporting Types & Callbacks

| Type | Definition | Purpose |
|------|------------|---------|
| `DynalgoStreamConfig` | `{frameType, width, height, fps, format, enabled}` | Per-stream configuration passed to `enableStream()` |
| `DynalgoAlignMode` | `NONE / HW / SW` | D2C alignment mode |
| `DynalgoVideoCallback` | `function<void(shared_ptr<DynalgoFrameSet>)>` | Video frame callback type |
| `DynalgoImuCallback` | `function<void(const vector<DynalgoImuSample>&)>` | IMU sample callback type |

**Framework call order** (`app/capture/dynalgo_capture_session.cpp:25-53`):

```cpp
// CaptureSession::setup() execution order:
device_->timerSyncWithHost();                      // clock sync (exception = warning only)
device_->enableGlobalTimestamp(true);              // if supported
sensorInfo_ = device_->setupPipeline(*pipeline_);   // CORE: populates all sensor info
// sensorInfo_ drives all subsequent encoder/file creation decisions
```

### 2.1 DynalgoDevice

| Method | Return | Purpose |
|--------|--------|---------|
| `getDeviceInfo()` | `DynalgoDeviceInfo` | name, serialNumber, vid, pid, connectionType |
| `timerSyncWithHost()` | void | Sync device clock to host |
| `isGlobalTimestampSupported()` | bool | Query global timestamp capability |
| `enableGlobalTimestamp(bool)` | void | Enable/disable global timestamp |
| `getSensorInfo()` | `DynalgoSensorInfo` | Cached sensor presence + profile summary |
| `getIntProperty(int)` | int32_t | Read integer property (e.g. depth precision) |
| `hasIRSensor()` | bool | True if device has any IR sensor |
| **`setupPipeline(DynalgoPipeline&)`** | `DynalgoSensorInfo` | **Critical**: enumerate sensors, select profiles, enable streams, apply quirks, detect D2C mode, return resolved sensor info |

### 2.2 DynalgoPipeline

| Method | Return | Purpose |
|--------|--------|---------|
| `enableStream(DynalgoStreamConfig)` | void | Configure stream (type, width, height, fps, format) |
| `disableStream(DynalgoFrameType)` | void | Disable a stream type |
| `setAggregateAllTypeFrameRequire(bool)` | void | Frame aggregation mode |
| `setAlignMode(DynalgoAlignMode)` | void | Set D2C alignment mode (HW/SW/NONE) |
| `checkHWD2CSupport(...)` | bool | Check HW D2C feasibility for given profiles |
| `enableFrameSync()` | void | Enable hardware frame sync |
| **`start(DynalgoVideoCallback)`** | bool | **Critical**: start pipeline; callback receives `shared_ptr<DynalgoFrameSet>` |
| `startImu(DynalgoImuCallback)` | bool | Start IMU streaming; callback receives `vector<DynalgoImuSample>` |
| `stop()` | void | Stop video pipeline |
| `stopImu()` | void | Stop IMU pipeline |
| `getDevice()` | `shared_ptr<DynalgoDevice>` | Return underlying device |
| `isPointCloudDepth()` | bool | Override to `true` if depth is 3D point cloud (default: false) |
| `getAlignMode()` | `DynalgoAlignMode` | Current alignment mode |
| `getD2CAlignFilter()` | `shared_ptr<DynalgoD2CAlign>` | Return SW alignment filter (default: nullptr) |

### 2.3 DynalgoContext

| Method | Return | Purpose |
|--------|--------|---------|
| `getDeviceCount()` | uint32_t | Number of connected devices |
| `getDevice(uint32_t)` | `shared_ptr<DynalgoDevice>` | Get device by index |

### 2.4 DynalgoD2CAlign (for SW D2C)

| Method | Return | Purpose |
|--------|--------|---------|
| `process(shared_ptr<void>, DynalgoAlignedFrame&)` | bool | Take type-erased native FrameSet, perform alignment, fill output struct |

---

## 3. Value Types (app/core/dynalgo_types.hpp, dynalgo_frame.hpp)

### DynalgoFormat

```
UNKNOWN, Y8, Y16, YUYV, UYVY, YUY2, MJPG, MJPEG, NV12, NV21, I420,
RGB, BGR, RGBA, BGRA, H264, H265, HEVC, POINT, RGB888
```

Helper functions:
- `nioFormatBpp(DynalgoFormat)` → bytes per pixel (Y8=1, Y16=2, RGB=3, RGBA=4, others=0)
- `nioFormatRawSize(DynalgoFormat, w, h)` → raw buffer size in bytes

### DynalgoFrameType

```
COLOR, DEPTH, IR, IR_LEFT, IR_RIGHT, ACCEL, GYRO,
COLOR_LEFT, COLOR_RIGHT, CONFIDENCE, POINT, COUNT
```

### DynalgoFrame

| Field | Type | Description |
|-------|------|-------------|
| `type` | DynalgoFrameType | Sensor source |
| `format` | DynalgoFormat | Pixel format |
| `width`, `height` | int | Frame dimensions (0 for IMU/point) |
| `timestampUs` | uint64_t | Microsecond timestamp |
| `depthScale` | float | Meters per raw unit (default 1.0) |
| `data` | `vector<uint8_t>` | Owning pixel buffer (deep copy) |
| `rawData()` | `const uint8_t*` | Pointer into data |
| `dataSize()` | uint32_t | Byte count |

### DynalgoFrameSet

| Method | Description |
|--------|-------------|
| `getFrame(DynalgoFrameType)` | Get frame pointer (nullptr if absent) |
| `setFrame(DynalgoFrameType, DynalgoFrame)` | Insert frame (takes ownership) |
| `allFrames()` | Iterate all frames |
| `nativeFrameSet` | `shared_ptr<void>` — opaque native FrameSet for alignment operations |

### DynalgoSensorInfo

| Field | Description |
|-------|-------------|
| `hasColor`, `hasDepth`, `hasIR`, `hasIRLeft`, `hasIRRight`, `hasAccel`, `hasGyro` | Sensor presence |
| `colorFormat`, `depthFormat`, `irFormat`, `irLeftFormat`, `irRightFormat` | Resolved DynalgoFormat |
| `colorW/H/Fps`, `depthW/H/Fps`, `irW/H/Fps`, `irLW/H/Fps`, `irRW/H/Fps` | Resolved profiles |
| `depthIntrinsic`, `colorIntrinsic` | `DynalgoIntrinsic` (fx, fy, cx, cy, width, height) |
| `depthScale` | Meters per raw depth unit |

### DynalgoAlignedFrame

| Field | Description |
|-------|-------------|
| `colorData`, `colorSize`, `colorTs` | Raw color after alignment |
| `depthData`, `depthSize`, `depthTs` | Raw depth after alignment |
| `depthScale` | Depth scale for the aligned frame |

---

## 4. Existing Adapter Reference

### 4.1 Orbbec Adapter (app/driver/orbbec/)

| File | Contents | Lines |
|------|----------|-------|
| `dynalgo_ob_adapter.hpp` | `obFormatToDynalgo()`, `dynalgoFormatToOb()`, `obFrameTypeToDynalgo()`, `obFrameTypeToDynalgo()`, `obFrameTypeToDynalgoSensor()`, `obIntrinsicToDynalgo()`, `dynalgoIntrinsicToOb()`, `selectBestProfile()`, `isLiDARDevice()`, `OB_DEVICE_VID` (0x2bc5), `isGemini305Device()`, `isGemini305gDevice()`, `isAstraMiniDevice()` | ~210 |
| `dynalgo_ob_device.hpp` | `ObDevice`, `ObPipeline`, `ObContext` declarations | ~106 |
| `dynalgo_ob_device.cpp` | Full implementation: device enumeration, sensor setup, HW/SW D2C, pipeline start/stop, IMU pipeline | ~447 |
| `dynalgo_ob_frame_adapter.hpp` | `obFrameSetToDynalgo()` (deep copy all video frames), `obImuToDynalgoSamples()` | ~118 |
| `dynalgo_ob_d2c_align.hpp` | `ObD2CAlign : DynalgoD2CAlign` — wraps `ob::Align` | ~54 |

**Key patterns:**

- `ObDevice::setupPipeline()` downcasts `DynalgoPipeline&` to `ObPipeline&`, iterates SDK sensor list, calls `selectBestProfile()` for each, enables streams on `ob::Config`, applies device quirks (e.g. Gemini 305g disables IR_LEFT), checks HW D2C, returns fully populated `DynalgoSensorInfo`.
- `ObPipeline::start()` wraps SDK callback: calls `obFrameSetToDynalgo(obFs)` for deep copy, attaches `nativeFrameSet = static_pointer_cast<void>(obFs)`, then calls user callback.
- `ObPipeline::getD2CAlignFilter()` returns `make_shared<ObD2CAlign>(alignFilter_)` if SW mode, else nullptr.
- `ObD2CAlign::process()` restores `ob::FrameSet` from `nativeFrameSet`, calls `ob::Align::process()`, extracts color/depth pointers into `DynalgoAlignedFrame`.

### 4.2 RoboSense Adapter (app/driver/robosense/)

| File | Contents | Lines |
|------|----------|-------|
| `dynalgo_rs_adapter.hpp` | `rsFrameFormatToDynalgo()`, `dynalgoFormatToRsFrameFormat()`, `rsImuToDynalgoSamples()` | ~75 |
| `dynalgo_rs_device.hpp` | `RsDevice`, `RsPipeline`, `RsContext` declarations | ~163 |
| `dynalgo_rs_device.cpp` | Full implementation: USB discovery, dual get/put callback queues, FrameSet synthesis, point-to-depth conversion | ~373 |
| `dynalgo_rs_frame_adapter.hpp` | `rsDepthToDynalgoFrame()`, `rsPointToDynalgoFrame()`, `rsImageToDynalgoFrame()` | ~121 |

**Key patterns:**

- `RsDevice` has fixed sensor info (no IR, depth=96×288@10 Y16, color=1920×1080@30 NV12).
- `RsPipeline` uses `LidarDriver` with dual get/put callback queues (`freeCloudQueue` ↔ `stuffedCloudQueue`, etc.). Three processing threads drain stuffed queues and synthesize `DynalgoFrameSet` when both color and depth arrive.
- `rsDepthToDynalgoFrame()` synthesizes a 96×288 Y16 depth map from 3D point cloud (distance / 0.005 → uint16, 0 for invalid/NaN).
- `RsPipeline::isPointCloudDepth()` returns `true`; `getAlignMode()` always returns `HW`.
- `RsContext::scanDevices()` uses libusb to find VID=0x3840/PID=0x1010 devices, reads serial from USB descriptor, falls back to busnum-devnum UUID.
- `RsPipeline::start()` must launch processing threads **before** `driver_.start()` so threads are ready when first data arrives.

---

## 5. Step-by-Step Porting Checklist

### Phase 1: Create Adapter Files

For a new vendor "XYZ", create under `app/driver/xyz/`:

```
app/driver/xyz/
├── dynalgo_xyz_.hpp       # Type conversions: XYZ↔Dynalgo
├── dynalgo_xyz_.hpp # Frame conversion: XYZ frame → DynalgoFrame
├── dynalgo_xyz_.hpp        # XyzDevice, XyzPipeline, XyzContext declarations
└── dynalgo_xyz_.cpp        # Full implementation
```

If the device supports SW D2C alignment, also create:
```
├── dynalgo_xyz_.hpp     # XyzD2CAlign : DynalgoD2CAlign
```

### Phase 2: Implement Type Mappings (dynalgo_xyz_.hpp)

Required conversions:

| From | To | Notes |
|------|----|-------|
| Vendor format enum | `DynalgoFormat` | Map all vendor pixel formats |
| `DynalgoFormat` | Vendor format enum | Reverse map (may be lossy) |
| Vendor frame type | `DynalgoFrameType` | Map sensor types |
| `DynalgoFrameType` | Vendor sensor type | For stream enable/disable |
| Vendor intrinsic | `DynalgoIntrinsic` | Copy fx/fy/cx/cy/width/height |
| `DynalgoIntrinsic` | Vendor intrinsic | Reverse map |

Helper functions you may need:
- `selectBestProfile()` — adapt from `dynalgo_ob_adapter.hpp` if the vendor SDK supports profile enumeration, or hardcode if fixed.
- `isLiDARDevice()` — if vendor supports LiDAR sensors, add to adapter; else skip.

### Phase 3: Implement Frame Conversion (dynalgo_xyz_.hpp)

You must provide at minimum:

```cpp
// Convert vendor video frame(s) → DynalgoFrameSet (deep copy pixel data)
DynalgoFrameSet xyzFrameSetToDynalgo(VendorFrameSetType vendorFs);

// Convert vendor IMU data → vector<DynalgoImuSample>
std::vector<DynalgoImuSample> xyzImuToDynalgoSamples(VendorImuType vendorImu);
```

Critical rules for `xyzFrameSetToDynalgo`:
1. **Deep copy pixel data**: `nf.data.assign(data, data + size)` — do NOT hold SDK buffer pointers.
2. **Set all DynalgoFrame fields**: type, format, width, height, timestampUs, depthScale (for depth frames), data.
3. **Attach native FrameSet** if SW D2C alignment needs it later: `nioFs.nativeFrameSet = static_pointer_cast<void>(vendorFs)`. Otherwise leave null.
4. **Handle missing frames gracefully**: if a stream doesn't exist in the vendor FrameSet, skip it — `DynalgoFrameSet` defaults to empty.

### Phase 4: Implement Device Classes (dynalgo_xyz_.hpp/cpp)

#### XyzDevice : DynalgoDevice

```cpp
class XyzDevice : public DynalgoDevice {
public:
    explicit XyzDevice(VendorDeviceHandle dev);

    DynalgoDeviceInfo getDeviceInfo() const override;
    void timerSyncWithHost() override;
    bool isGlobalTimestampSupported() const override;
    void enableGlobalTimestamp(bool enable) override;
    DynalgoSensorInfo getSensorInfo() const override;
    int32_t getIntProperty(int propertyId) override;
    bool hasIRSensor() const override;
    DynalgoSensorInfo setupPipeline(DynalgoPipeline& pipeline) override;

    // Expose vendor handle for pipeline construction
    VendorDeviceHandle xyzDevice() const { return xyzDevice_; }
private:
    VendorDeviceHandle xyzDevice_;
};
```

**`setupPipeline()` is the most complex method.** It must:

1. Downcast `DynalgoPipeline&` to `XyzPipeline&`.
2. Enumerate vendor sensor list.
3. For each sensor type, select the best streaming profile (resolution, format, fps).
4. Enable the chosen profile on the vendor pipeline config.
5. Populate `DynalgoSensorInfo` with resolved values.
6. Apply any device-specific quirks (disable broken streams, etc.).
7. Check D2C alignment capability → call `pipeline.setAlignMode()`.

   **D2C-HW-aware depth profile selection (Orbbec reference):** before picking the
   depth profile, call the vendor equivalent of `getD2CDepthProfileList(colorProfile,
   ALIGN_D2C_HW_MODE)` to obtain the HW-D2C-supported depth profile set and pass it
   to `selectBestProfile()`'s `hwD2CSupportedProfiles` parameter — profiles appearing
   in that set get +800 score, maximising the chance HW alignment kicks in instead of
   falling back to software. See Technical Reference §7.1.1.

8. Return the fully populated `DynalgoSensorInfo`.

#### XyzPipeline : DynalgoPipeline

```cpp
class XyzPipeline : public DynalgoPipeline {
public:
    explicit XyzPipeline(std::shared_ptr<XyzDevice> device);

    void enableStream(const DynalgoStreamConfig& cfg) override;
    void disableStream(DynalgoFrameType type) override;
    // ... all DynalgoPipeline virtual methods ...

    bool start(DynalgoVideoCallback callback) override;
    void stop() override;
    std::shared_ptr<DynalgoD2CAlign> getD2CAlignFilter() const override;
private:
    std::shared_ptr<XyzDevice> xyzDevice_;
    VendorPipelineHandle xyzPipeline_;
    DynalgoVideoCallback videoCallback_;
    // ...
};
```

**`start()` critical pattern:**

```cpp
bool XyzPipeline::start(DynalgoVideoCallback callback) {
    videoCallback_ = callback;
    try {
        vendorPipeline_->start(vendorConfig_, [this](VendorFrameSetType vfs) {
            if (vfs) {
                auto dynalgoFs = std::make_shared<DynalgoFrameSet>(xyzFrameSetToDynalgo(vfs));
                // Only attach nativeFrameSet if SW D2C alignment needs it:
                dynalgoFs->nativeFrameSet = std::static_pointer_cast<void>(vfs);
                videoCallback_(dynalgoFs);
            }
        });
        return true;
    } catch (VendorError& e) {
        DYNALGO_LOG_ERROR_S("Pipeline start failed: " << e.what());
        return false;
    }
}
```

**Asynchronous sources** (like RoboSense where point cloud and image arrive on separate threads):
- Use processing threads to drain vendor queues.
- Maintain `colorFrame_`, `depthFrame_`, `colorReady_`, `depthReady_` with a mutex.
- When both ready, call `tryEmitFrameSet()` to assemble and emit `DynalgoFrameSet`.

#### XyzContext : DynalgoContext

- Implement device discovery (USB scan, SDK context query, etc.).
- `getDeviceCount()` → number of connected devices.
- `getDevice(index)` → `make_shared<XyzDevice>(vendorDev)`.

### Phase 5: Register in Driver Factory (dynalgo_driver_factory.cpp)

```cpp
#ifdef ENABLE_XYZ
#include "xyz/dynalgo_xyz_device.hpp"
#endif

std::vector<DiscoveredDevice> discoverDevices() {
    std::vector<DiscoveredDevice> result;

    // ... existing ENABLE_ORBBEC block ...
    // ... existing ENABLE_RS_AC1 block ...

#ifdef ENABLE_XYZ
    XyzContext xyzCtx;
    uint32_t xyzCount = xyzCtx.getDeviceCount();
    for (uint32_t i = 0; i < xyzCount; i++) {
        auto dynalgoDev = xyzCtx.getDevice(i);
        auto xyzDev = std::dynamic_pointer_cast<XyzDevice>(dynalgoDev);
        if (!xyzDev)
            continue;
        DiscoveredDevice dd;
        dd.device = dynalgoDev;
        dd.pipeline = std::make_shared<XyzPipeline>(xyzDev);
        result.push_back(std::move(dd));
    }
#endif

    return result;
}
```

### Phase 6: Update CMake

**`option()` declarations go in the root `CMakeLists.txt`**, not in `app/driver/CMakeLists.txt`.
The `add_subdirectory(vendors/XYZ)` call and its cache variable setup also go in root `CMakeLists.txt`,
gated by `if(ENABLE_XYZ)`. See the existing `ENABLE_ORBBEC` / `ENABLE_RS_AC1` blocks for reference.

In `app/driver/CMakeLists.txt`, add a new `if(ENABLE_XYZ)` block for source files, include paths,
compile definitions, and link libraries (following the existing Orbbec/RoboSense pattern):

Add a new `if(ENABLE_XYZ)` block following the existing pattern:

```cmake
# Root CMakeLists.txt — option declaration + vendor subdirectory
option(ENABLE_XYZ "Enable XYZ device support" OFF)

if(ENABLE_XYZ)
    set(XYZ_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vendors/XYZ" CACHE PATH "" FORCE)
    # ... vendor-specific cache variables ...
    add_subdirectory(vendors/XYZ)
endif()

# app/driver/CMakeLists.txt — source, includes, definitions, links
if(ENABLE_XYZ)
    target_include_directories(dynalgo_drivers PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/xyz
    )

    target_sources(dynalgo_drivers PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/xyz/dynalgo_xyz_.hpp
        ${CMAKE_CURRENT_LIST_DIR}/xyz/dynalgo_xyz_.hpp
        ${CMAKE_CURRENT_LIST_DIR}/xyz/dynalgo_xyz_.hpp
        ${CMAKE_CURRENT_LIST_DIR}/xyz/dynalgo_xyz_.cpp
        ${CMAKE_CURRENT_LIST_DIR}/dynalgo_driver_factory.hpp
        ${CMAKE_CURRENT_LIST_DIR}/dynalgo_driver_factory.cpp
    )

    target_compile_definitions(dynalgo_drivers PUBLIC
        ENABLE_XYZ
    )

    target_link_libraries(dynalgo_drivers PUBLIC
        XYZ::SDK
    )

    message(STATUS "XYZ support enabled")
endif()
```

Also ensure the driver factory source (`dynalgo_driver_factory.hpp/.cpp`) is included
when **any** vendor is enabled (it sits in the shared `if(ENABLE_ORBBEC OR ENABLE_RS_AC1)` block;
extend the condition to `if(ENABLE_ORBBEC OR ENABLE_RS_AC1 OR ENABLE_XYZ)`).

---

## 6. D2C Alignment Strategy

| Scenario | Pattern | Implementation |
|----------|---------|----------------|
| **HW D2C only** (e.g. RS-AC1) | `getAlignMode()` returns `HW`, `getD2CAlignFilter()` returns nullptr | CaptureSession extracts color/depth data directly from `DynalgoFrameSet`; no SW alignment step |
| **SW D2C only** | `getAlignMode()` returns `SW`, `getD2CAlignFilter()` returns valid `DynalgoD2CAlign*` | CaptureSession calls `alignFilter->process(dynalgoFs->nativeFrameSet, out)`; nativeFrameSet must be populated |
| **HW preferred, SW fallback** (e.g. Orbbec) | `setupPipeline()` queries HW capability; falls back to SW if unavailable | ObPipeline::checkHWD2CSupport() → setAlignMode(HW or SW) |
| **No D2C** | `getAlignMode()` returns `NONE`, `getD2CAlignFilter()` returns nullptr | No alignment performed |

**Important:** `nativeFrameSet` is a transitional escape valve. It exists because `DynalgoD2CAlign::process()` needs the original SDK frame to perform vendor-specific alignment. If your vendor's alignment can work on raw pixel data alone, you can implement `DynalgoD2CAlign::process()` to work without `nativeFrameSet` (pass `nullptr`, operate on `DynalgoFrame::data` directly).

**Fusion decision flow** (`app/capture/dynalgo_capture_session.cpp:164-208`):

```cpp
canFuse_ = cfg_.enableFusion && sensorInfo_.hasColor && sensorInfo_.hasDepth;
hwD2CMode_ = (pipeline_->getAlignMode() == DynalgoAlignMode::HW);
if (!hwD2CMode_) {
    alignFilter = pipeline_->getD2CAlignFilter();  // SW mode: get alignment filter
}
```

---

## 7. Data Flow Diagram

```
Vendor SDK callback
  │
  ▼  xyzFrameSetToDynalgo() — deep copy pixel data
  │  dynalgoFs->nativeFrameSet = static_pointer_cast<void>(vendorFs)  [if SW D2C]
  │
DynalgoFrameSet (owns pixel data + optional opaque native handle)
  │
  ▼  videoCallback_(dynalgoFs)  —  zero blocking in SDK callback
  │
VideoFrameQueue (bounded SPSC, capacity 8)
  │
  ▼  VideoConsumerThread pops
  │
  ├──► FusionTask (HW: enqueueColor/enqueueDepth from DynalgoFrame)
  │                        (SW: enqueueNioFrameSet → alignFilter->process())
  ├──► ColorFrameConsumer → H264 encoder + viewer
  ├──► DepthFrameConsumer → jet colormap H264 + raw file + viewer
  ├──► IRFrameConsumer → H264 + viewer
   └──► PointCloudConsumer → PCD file
```

File creation gating conditions (`app/capture/dynalgo_capture_session.cpp:59-158`):

| File | Gate condition |
|------|---------------|
| `*_color_*.h264` | `hasColor && colorFormat != UNKNOWN` |
| `*_depth_*.h264` + `*_depth_raw_*.raw` | `hasDepth && depthFormat != UNKNOWN` |
| `*_ir_left_*.h264` | `hasIRLeft` |
| `*_ir_right_*.h264` | `hasIRRight` |
| `*_imu_*.txt` | `hasAccel && hasGyro` |
| `*_point_raw_*.raw` | `pipeline->isPointCloudDepth()` |
| `*_d2c_fused_*.h264` | `canFuse` (= hasColor && hasDepth) |

---

## 8. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Build error: `DYNALGO_DEVICE_VID / isGemini305*` / `isAstraMini*` in core | Orbbec-specific vid/pid checks leaked to core | Move to `app/driver/VENDOR/dynalgo_xyz_.hpp` — use `VENDOR_DEVICE_VID` constant + inline functions |
| Build error: `ob::` / `OB_` in non-driver file | SDK type used outside driver | Replace with `Nio*` equivalent; move SDK-specific logic to adapter |
| Segfault in consumer thread | `DynalgoFrame::data` is empty or `rawData()` returns nullptr | Verify `xyzFrameSetToDynalgo()` copies pixel data via `nf.data.assign()` |
| SW D2C returns garbage | `nativeFrameSet` is null or wrong type | Ensure `start()` sets `dynalgoFs->nativeFrameSet = static_pointer_cast<void>(vendorFs)` and `XyzD2CAlign::process()` casts back correctly |
| Duplicate frames / mixed timestamps | Vendor callback fires on wrong thread / not synchronized | Use mutex + ready flags for async sources (see RsPipeline pattern) |
| Pipeline start fails silently | Exception swallowed in `start()` | Check `DYNALGO_LOG_ERROR_S` output; verify vendor config is complete before calling vendor start |
| `setupPipeline()` returns all-zero `DynalgoSensorInfo` | Downcast to concrete pipeline failed | Ensure factory creates correct `XyzPipeline` type paired with `XyzDevice` |
| USB memory error with multi-device | `usbfs_memory_mb` too low | `echo 256 \| sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb` |
| `discoverDevices()` link error | Driver factory not compiled | Ensure `app/driver/CMakeLists.txt:94` condition includes new vendor macro |

---

## 9. Actuator Porting Guide

> Actuators are the output end of the perceive–control loop. Phase A introduced the `DynalgoActuator` abstract base class with a self-registration factory; Phase C wires it into the EngagementLoop. Adding a new actuator only requires implementing the abstract interface and calling `registerActuator()` at static init.

### 9.1 Architecture Location

```
app/
├── core/
│   ├── dynalgo_actuator.hpp          # DynalgoActuatorType, DynalgoActuatorConfig, DynalgoActuator abstract
│   ├── dynalgo_actuator_factory.hpp  # createActuator(), registerActuator()
│   └── dynalgo_actuator_factory.cpp  # implementation
├── actuator/
│   ├── dummy_actuator.hpp/cpp        # DUMMY backend (self-registration demo)
│   └── CMakeLists.txt                # builds dynalgo_actuators static lib
├── algo/
│   └── dynalgo_engagement_loop.cpp   # calls actuator->aimAt(X,Y,Z) → fire()
└── dynamic_algo_cam/
    └── dynamic_algo_cam.cpp          # CLI wiring --engage-actuator
```

### 9.2 Abstract Interface (app/core/dynalgo_actuator.hpp)

```cpp
enum class DynalgoActuatorType {
    NONE, DUMMY, LASER_GENERIC, GIMBAL_GENERIC
};

struct DynalgoActuatorConfig {
    bool dryRun = true;           // safety default: true = all control is no-op
    std::string device;           // serial / network device path
    int channel = 0;              // channel number
    std::string ip;               // network IP
    uint16_t port = 0;            // network port
    int baud = 115200;            // serial baud rate
};

class DynalgoActuator {
public:
    virtual ~DynalgoActuator() = default;

    // Initialize: parse cfg, open device. dryRun=true → log only, return true
    virtual bool load(const DynalgoActuatorConfig& cfg) = 0;

    // Open/connect device
    virtual bool open() = 0;

    // Aim: camera optical-center XYZ meters (from TrackBundle::lastX/Y/Z())
    virtual bool aimAt(float x, float y, float z) = 0;

    // Fire: durationMs = pulse duration. dryRun=true → log only
    virtual bool fire(int durationMs) = 0;

    // Close/disconnect device
    virtual bool close() = 0;

    // Backend name for log identification
    virtual const char* name() const noexcept = 0;
};
```

**Key points:**
- `dryRun` defaults to `true` — all real hardware ops must be inside `if (!cfg.dryRun) { ... }`
- `aimAt()` coordinate system: **camera optical-center XYZ meters** (Phase B `detectionCenterToCamera3D()` output)
- `fire()` duration unit: **milliseconds**
- All methods return `bool`: on failure EngagementLoop logs `DYNALGO_LOG_WARN_S` and continues

### 9.3 Self-Registration Pattern (DUMMY Demo)

`app/actuator/dummy_actuator.cpp` end:

```cpp
namespace {
struct DummyRegistrar {
    DummyRegistrar() {
        dynalgo::registerActuator(
            dynalgo::DynalgoActuatorType::DUMMY,
            [](const dynalgo::DynalgoActuatorConfig& cfg) {
                return std::make_unique<dynalgo::DummyActuator>(cfg);
            });
    }
} g_dummyRegistrar;
} // namespace
```

**Critical requirement:**
1. Anonymous namespace + static instance ensures registrar TU isn't optimized away at link time
2. Consumer (main program) linking `libdynalgo_actuators.a` **must** use `--whole-archive`:

```cmake
# app/dynamic_algo_cam/CMakeLists.txt or root CMakeLists.txt
target_link_libraries(dynamic_algo_cam PRIVATE
    "-Wl,--whole-archive" dynalgo::actuators "-Wl,--no-whole-archive"
)
```

Without this, the registrar TU is discarded → `createActuator(DUMMY)` returns `nullptr`.

### 9.4 New Actuator Implementation Checklist

For a new actuator "XYZ" under `app/actuator/xyz/`:

```
app/actuator/xyz/
├── dynalgo_xyz_actuator.hpp
├── dynalgo_xyz_actuator.cpp
└── CMakeLists.txt (optional, or add to app/actuator/CMakeLists.txt)
```

#### Implementation Steps

1. **Inherit `DynalgoActuator`** and implement all 6 pure virtual methods
2. **Constructor** stores `DynalgoActuatorConfig`
3. **`load()`** parses cfg; `dryRun=true` → log only, return `true`
4. **`open()`** real connect to serial/network; `dryRun=true` → log only
5. **`aimAt(x,y,z)`** sends command to hardware; `dryRun=true` → log only
6. **`fire(durationMs)`** triggers emission; `dryRun=true` → log only
7. **`close()`** closes connection; `dryRun=true` → log only
8. **`name()`** returns `"XYZ"`
9. **Self-registration block** (same pattern as DUMMY) at end of `.cpp`
10. **Update `app/actuator/CMakeLists.txt`** to include source file

#### DUMMY Reference Implementation

```cpp
// dummy_actuator.hpp
class DummyActuator : public DynalgoActuator {
public:
    explicit DummyActuator(const DynalgoActuatorConfig& cfg);
    bool load(const DynalgoActuatorConfig& cfg) override;
    bool open() override;
    bool aimAt(float x, float y, float z) override;
    bool fire(int durationMs) override;
    bool close() override;
    const char* name() const noexcept override { return "DUMMY"; }
private:
    DynalgoActuatorConfig cfg_;
};
```

```cpp
// dummy_actuator.cpp
bool DummyActuator::load(const DynalgoActuatorConfig& cfg) {
    cfg_ = cfg;
    DYNALGO_LOG_INFO_S("DummyActuator::load dryRun=" << (cfg_.dryRun ? "on" : "off"));
    return true;
}
bool DummyActuator::open() {
    DYNALGO_LOG_INFO_S("DummyActuator::open dryRun=" << (cfg_.dryRun ? "on" : "off"));
    return true;
}
bool DummyActuator::aimAt(float x, float y, float z) {
    DYNALGO_LOG_INFO_S("DummyActuator::aimAt(" << x << "," << y << "," << z << ") dryRun=" << (cfg_.dryRun ? "on" : "off"));
    return true;
}
bool DummyActuator::fire(int durationMs) {
    DYNALGO_LOG_INFO_S("DummyActuator::fire durationMs=" << durationMs << " dryRun=" << (cfg_.dryRun ? "on" : "off"));
    return true;
}
bool DummyActuator::close() {
    DYNALGO_LOG_INFO_S("DummyActuator::close dryRun=" << (cfg_.dryRun ? "on" : "off"));
    return true;
}
```

### 9.5 CMake Configuration

`app/actuator/CMakeLists.txt`:

```cmake
add_library(dynalgo_actuators STATIC
    dummy_actuator.cpp
    # add new actuator sources here:
    # xyz/dynalgo_xyz_actuator.cpp
)

target_include_directories(dynalgo_actuators PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(dynalgo_actuators PUBLIC
    dynalgo::core
)

# alias for consumers
add_library(dynalgo::actuators ALIAS dynalgo_actuators)
```

Root `app/CMakeLists.txt` (after `add_subdirectory(capture)`):

```cmake
add_subdirectory(actuator)
```

### 9.6 CLI Wiring (app/dynamic_algo_cam/dynamic_algo_cam.cpp)

```cpp
// parse --engage-actuator
std::string engageActuator = config.engageActuator; // "DUMMY" / "LASER_GENERIC" etc.
DynalgoActuatorType actuatorType = DynalgoActuatorType::NONE;
if (engageActuator == "DUMMY") actuatorType = DynalgoActuatorType::DUMMY;
else if (engageActuator == "LASER_GENERIC") actuatorType = DynalgoActuatorType::LASER_GENERIC;
// ...

// create actuator (requires --whole-archive for self-registration)
auto actuator = createActuator(actuatorType);
if (actuator) {
    DynalgoActuatorConfig actuatorCfg;
    actuatorCfg.dryRun = true; // default safe
    actuator->load(actuatorCfg);
    actuator->open();
    // pass to EngagementLoop constructor
}
```

### 9.7 Full Call Chain

```
dynamic_algo_cam.cpp
    → createActuator(type)                    [factory lookup]
        → DummyActuator(cfg)                  [construct]
            → actuator->load(cfg)             [parse config]
            → actuator->open()                [connect hardware]
            → actuator->aimAt(X,Y,Z)          [aim: optical-center XYZ]
            → actuator->fire(durationMs)      [fire: milliseconds]
            → actuator->close()               [disconnect]
```

### 9.8 Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `createActuator()` returns `nullptr` | Missing `--whole-archive` at link | `target_link_libraries(... "-Wl,--whole-archive" dynalgo::actuators "-Wl,--no-whole-archive")` |
| Actuator does nothing (no logs) | `dryRun=true` by default | `load()/open()` log dryRun state — check logs |
| Build error: `ob::` / `rs::` in actuator code | Vendor SDK leaked to actuator layer | Actuator layer must be SDK-agnostic; depend only on `dynalgo_core` |
| `aimAt` coordinates wrong | Coordinate system mismatch | Confirm input is **camera optical-center XYZ meters** (TrackBundle output) |
| Link error: `registerActuator` multiply defined | Multiple TUs define same registrar | Use anonymous-namespace static instance; ensure each backend registers once |

---

## 10. Model Backend Porting Guide

> Model backends are the inference engines for the perceive–locate–estimate–control loop. Phase C introduced the `DynalgoModelBackend` abstract base class with a self-registration factory; Phase C wires it into the EngagementLoop. Adding a new model backend only requires implementing the abstract interface and calling `registerModelBackend()` at static init.

### 10.1 Architecture Location

```
app/
├── core/
│   ├── dynalgo_model.hpp           # DynalgoModelType, DynalgoModelConfig, DynalgoDetectionResult, DynalgoModelBackend abstract
│   ├── dynalgo_model_factory.hpp   # createModelBackend(), registerModelBackend()
│   └── dynalgo_model_factory.cpp   # implementation
├── model_backends/
│   ├── common/
│   │   ├── preprocessing.hpp/cpp   # Letterbox, NMS, postprocess (shared)
│   │   └── CMakeLists.txt
│   ├── tensorrt/
│   │   ├── trt_backend.hpp/cpp     # TensorRT C++ backend
│   │   └── CMakeLists.txt
│   ├── onnxruntime/
│   │   ├── ort_backend.hpp/cpp     # ONNX Runtime C++ backend
│   │   └── CMakeLists.txt
│   ├── rknn/
│   │   ├── rknn_backend.hpp/cpp    # RKNN Runtime C++ backend
│   │   └── CMakeLists.txt
│   └── CMakeLists.txt              # Meta-library with ENABLE_* options
├── algo/
│   └── dynalgo_engagement_loop.cpp # calls model->infer(frame, detections)
└── dynamic_algo_cam/
    └── dynamic_algo_cam.cpp        # CLI wiring --engage-model
```

### 10.2 Abstract Interface (app/core/dynalgo_model.hpp)

```cpp
enum class DynalgoModelType {
    NONE, DUMMY, YOLOV8_PY, ONNXRUNTIME, TENSORRT
};

struct DynalgoModelConfig {
    std::string modelPath;       // weights / model file or model name
    std::string deviceHint;      // e.g. "cpu", "gpu", "cuda:0", empty = default
    float confThreshold = 0.25f; // minimum score to keep a detection
    float iouThreshold = 0.45f;  // NMS IoU threshold (when applicable)
};

class DynalgoModelBackend {
public:
    virtual ~DynalgoModelBackend() = default;

    // Load weights / initialise backend. Returns true on success. Must be called once before infer().
    virtual bool load(const DynalgoModelConfig& cfg) = 0;

    // Run inference on one frame. Results appended to `out`. Thread-safe implementations should support multi-threaded calls.
    virtual bool infer(const DynalgoFrame& frame, std::vector<DynalgoDetectionResult>& out) = 0;

    // Return human-readable backend name
    virtual const char* name() const = 0;
};
```

### 10.3 TensorRT Backend (app/model_backends/tensorrt/)

The TensorRT backend (`trt_backend.hpp/cpp`) provides high-performance inference on NVIDIA GPUs with full TensorRT feature support.

#### Configuration (TrtBackendConfig)

```cpp
struct TrtBackendConfig {
    std::string modelPath;           // Path to .engine or .onnx file
    bool fp16 = true;                // Enable FP16 mode
    bool int8 = false;               // Enable INT8 mode (requires calibration)
    std::string calibrationCache;    // Path to calibration cache file
    std::string timingCache;         // Path to timing cache file
    int maxBatchSize = 1;            // Maximum batch size
    size_t workspaceSize = 1 << 30;  // Workspace size (1GB default)
    int deviceId = 0;                // GPU device ID
    bool enableDynamicBatch = true;  // Enable dynamic batch optimization profile
    cudaStream_t externalStream = 0; // External CUDA stream for async pipeline
};
```

#### Key Features

| Feature | Description |
|---------|-------------|
| **FP16** | `config.fp16 = true` enables FP16 mode (requires GPU with FP16 support) |
| **INT8 Quantization** | `config.int8 = true` + `calibrationCache` + calibrator provider for INT8 |
| **Dynamic Batch** | `config.enableDynamicBatch = true` + `maxBatchSize` creates optimization profiles |
| **Timing Cache** | `config.timingCache` path for kernel selection timing cache (load/save) |
| **Calibration Cache** | `config.calibrationCache` path for INT8 calibration cache (load/save) |
| **Async Pipeline** | `config.externalStream` or `setCUDAStream()` for async CUDA stream integration |
| **Engine Serialization** | `serializeEngine(path)` exports built engine for deployment |

#### Building Engine from ONNX

```cpp
auto backend = createModelBackend(DynalgoModelType::TENSORRT);
auto* trt = dynamic_cast<TrtBackend*>(backend.get());

TrtBackendConfig config;
config.modelPath = "model.onnx";
config.fp16 = true;
config.int8 = false;
config.maxBatchSize = 4;
config.workspaceSize = 1 << 30; // 1GB
config.timingCache = "model_timing.cache";
config.enableDynamicBatch = true;
config.externalStream = myCudaStream; // For async pipeline

if (!trt->buildEngineFromOnnx(config)) {
    // Handle error
}
```

#### INT8 Quantization

```cpp
// Implement custom calibrator
class MyCalibrator : public IInt8CalibratorProvider {
    bool getBatch(void* bindings[], const char* names[], int nbBindings) override {
        // Fill bindings with calibration data
        return true; // or false when done
    }
    // ... implement other methods
};

MyCalibrator calibrator;
TrtBackendConfig config;
config.modelPath = "model.onnx";
config.int8 = true;
config.calibrationCache = "model_int8.cache";
config.maxBatchSize = 4;

if (!trt->buildEngineWithCalibration(config, &calibrator)) {
    // Handle error
}
```

#### Async Pipeline Integration

```cpp
// Create CUDA stream for async operations
cudaStream_t myStream;
cudaStreamCreate(&myStream);

// Configure backend
trt->setCUDAStream(myStream);

// In inference loop
while (running) {
    // Preprocess on host or separate stream
    // ...

    // Infer with external stream
    trt->infer(frame, detections); // Uses external stream internally

    // Synchronize only when results needed
    cudaStreamSynchronize(myStream);
    
    // Process detections...
}
```

#### Self-Registration Pattern

```cpp
// In trt_backend.cpp
std::unique_ptr<DynalgoModelBackend> createTrtBackend() {
    return std::make_unique<TrtBackend>();
}

// Static registration (in anonymous namespace at file end)
namespace {
struct TrtRegistrar {
    TrtRegistrar() {
        dynalgo::registerModelBackend(
            dynalgo::DynalgoModelType::TENSORRT,
            []() { return std::make_unique<TrtBackend>(); });
    }
} g_trtRegistrar;
}
```

### 10.4 CMake Configuration

`app/model_backends/CMakeLists.txt`:

```cmake
option(ENABLE_TENSORRT "Enable TensorRT backend" OFF)
if(ENABLE_TENSORRT)
    find_package(CUDA REQUIRED)
    find_package(TensorRT REQUIRED)
    add_subdirectory(tensorrt)
endif()

option(ENABLE_ONNXRUNTIME "Enable ONNX Runtime backend" OFF)
if(ENABLE_ONNXRUNTIME)
    find_package(ONNXRuntime REQUIRED)
    add_subdirectory(onnxruntime)
endif()

option(ENABLE_RKNN "Enable RKNN backend" OFF)
if(ENABLE_RKNN)
    add_subdirectory(rknn)
endif()

# Meta-library
if(ENABLE_TENSORRT OR ENABLE_ONNXRUNTIME OR ENABLE_RKNN)
    add_library(dynalgo_model_backends STATIC model_backends_dummy.cpp)
    target_link_libraries(dynalgo_model_backends PUBLIC dynalgo::core dynalgo::model_backends_common)
    if(ENABLE_TENSORRT)
        target_link_libraries(dynalgo_model_backends PUBLIC dynalgo::model_backend_trt)
    endif()
    # ...
endif()
```

Build with:

```bash
cmake -B build -DENABLE_MODEL_BACKENDS=ON -DENABLE_TENSORRT=ON
cmake --build build -j$(nproc)
```

### 10.5 CLI Wiring (app/dynamic_algo_cam/dynamic_algo_cam.cpp)

```cpp
// parse --engage-model
std::string engageModel = config.engageModel; // "DUMMY", "ONNXRUNTIME", "TENSORRT"
DynalgoModelType modelType = DynalgoModelType::NONE;
if (engageModel == "TENSORRT") modelType = DynalgoModelType::TENSORRT;
else if (engageModel == "ONNXRUNTIME") modelType = DynalgoModelType::ONNXRUNTIME;
// ...

auto model = createModelBackend(modelType);
if (model) {
    DynalgoModelConfig modelCfg;
    modelCfg.modelPath = config.engageModelPath;
    modelCfg.deviceHint = "cuda:0";
    modelCfg.confThreshold = 0.25f;
    model->load(modelCfg);
    // pass to EngagementLoop constructor
}
```

Usage:

```bash
./dynamic_algo_cam --engage-model TENSORRT --engage-actuator DUMMY --engage-model-path model.engine --no-show
```

---

## 11. Unresolved / Open Items

1. **`DynalgoFrameSet::nativeFrameSet`** is a transitional escape valve. Goal: remove once all D2C alignment can operate on `DynalgoFrame::data` alone or through a cleaner abstraction.
2. **`DynalgoPipeline::enableStream(DynalgoStreamConfig)`** is currently a no-op for Orbbec (streams are enabled inside `setupPipeline()`). A future refactor should make stream enable/disable fully configurable through this API.
3. **Device property IDs** (`getIntProperty(int)`) pass raw vendor property IDs. Should be abstracted to `DynalgoPropertyID` enum if more vendors need it.
4. **Point cloud depth** (`isPointCloudDepth() == true`) is currently only used by RS-AC1. The 96×288 synthetic depth map is vendor-specific. A future depth abstraction may be needed for vendors with different point cloud layouts.
5. **`app/core/utils_c.h` and `utils_c.c`** contain C API functions (timestamp, key press). They are SDK-agnostic. The former `DYNALGO_DEVICE_VID 0x2bc5` and Orbbec vid/pid device-type checks have been moved to `app/driver/orbbec/dynalgo_ob_adapter.hpp` as `OB_DEVICE_VID`, `isGemini305Device()`, `isGemini305gDevice()`, `isAstraMiniDevice()`.
6. **FATAL_ERROR guard**: root `CMakeLists.txt` requires at least one `ENABLE_*` option to be ON. When adding a new vendor option, update the guard condition from `if(NOT ENABLE_ORBBEC AND NOT ENABLE_RS_AC1)` to also include the new option.
