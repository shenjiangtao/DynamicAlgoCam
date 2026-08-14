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
| `dynalgo_ob_adapter.hpp` | `obFormatToNio()`, `nioFormatToOb()`, `obFrameTypeToNio()`, `nioFrameTypeToOb()`, `nioFrameTypeToObSensor()`, `obIntrinsicToNio()`, `nioIntrinsicToOb()`, `selectBestProfile()`, `isLiDARDevice()`, `OB_DEVICE_VID` (0x2bc5), `isGemini305Device()`, `isGemini305gDevice()`, `isAstraMiniDevice()` | ~210 |
| `dynalgo_ob_device.hpp` | `ObDevice`, `ObPipeline`, `ObContext` declarations | ~106 |
| `dynalgo_ob_device.cpp` | Full implementation: device enumeration, sensor setup, HW/SW D2C, pipeline start/stop, IMU pipeline | ~447 |
| `dynalgo_ob_frame_adapter.hpp` | `obFrameSetToNio()` (deep copy all video frames), `obImuToNioSamples()` | ~118 |
| `dynalgo_ob_d2c_align.hpp` | `ObD2CAlign : DynalgoD2CAlign` — wraps `ob::Align` | ~54 |

**Key patterns:**

- `ObDevice::setupPipeline()` downcasts `DynalgoPipeline&` to `ObPipeline&`, iterates SDK sensor list, calls `selectBestProfile()` for each, enables streams on `ob::Config`, applies device quirks (e.g. Gemini 305g disables IR_LEFT), checks HW D2C, returns fully populated `DynalgoSensorInfo`.
- `ObPipeline::start()` wraps SDK callback: calls `obFrameSetToNio(obFs)` for deep copy, attaches `nativeFrameSet = static_pointer_cast<void>(obFs)`, then calls user callback.
- `ObPipeline::getD2CAlignFilter()` returns `make_shared<ObD2CAlign>(alignFilter_)` if SW mode, else nullptr.
- `ObD2CAlign::process()` restores `ob::FrameSet` from `nativeFrameSet`, calls `ob::Align::process()`, extracts color/depth pointers into `DynalgoAlignedFrame`.

### 4.2 RoboSense Adapter (app/driver/robosense/)

| File | Contents | Lines |
|------|----------|-------|
| `dynalgo_rs_adapter.hpp` | `rsFrameFormatToNio()`, `nioFormatToRsFrameFormat()`, `rsImuToNioSamples()` | ~75 |
| `dynalgo_rs_device.hpp` | `RsDevice`, `RsPipeline`, `RsContext` declarations | ~163 |
| `dynalgo_rs_device.cpp` | Full implementation: USB discovery, dual get/put callback queues, FrameSet synthesis, point-to-depth conversion | ~373 |
| `dynalgo_rs_frame_adapter.hpp` | `rsDepthToNioFrame()`, `rsPointToNioFrame()`, `rsImageToNioFrame()` | ~121 |

**Key patterns:**

- `RsDevice` has fixed sensor info (no IR, depth=96×288@10 Y16, color=1920×1080@30 NV12).
- `RsPipeline` uses `LidarDriver` with dual get/put callback queues (`freeCloudQueue` ↔ `stuffedCloudQueue`, etc.). Three processing threads drain stuffed queues and synthesize `DynalgoFrameSet` when both color and depth arrive.
- `rsDepthToNioFrame()` synthesizes a 96×288 Y16 depth map from 3D point cloud (distance / 0.005 → uint16, 0 for invalid/NaN).
- `RsPipeline::isPointCloudDepth()` returns `true`; `getAlignMode()` always returns `HW`.
- `RsContext::scanDevices()` uses libusb to find VID=0x3840/PID=0x1010 devices, reads serial from USB descriptor, falls back to busnum-devnum UUID.
- `RsPipeline::start()` must launch processing threads **before** `driver_.start()` so threads are ready when first data arrives.

---

## 5. Step-by-Step Porting Checklist

### Phase 1: Create Adapter Files

For a new vendor "XYZ", create under `app/driver/xyz/`:

```
app/driver/xyz/
├── dynalgo_xyz_.hpp       # Type conversions: XYZ↔Nio
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
DynalgoFrameSet xyzFrameSetToNio(VendorFrameSetType vendorFs);

// Convert vendor IMU data → vector<DynalgoImuSample>
std::vector<DynalgoImuSample> xyzImuToNioSamples(VendorImuType vendorImu);
```

Critical rules for `xyzFrameSetToNio`:
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
                auto nioFs = std::make_shared<DynalgoFrameSet>(xyzFrameSetToNio(vfs));
                // Only attach nativeFrameSet if SW D2C alignment needs it:
                nioFs->nativeFrameSet = std::static_pointer_cast<void>(vfs);
                videoCallback_(nioFs);
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
        auto nioDev = xyzCtx.getDevice(i);
        auto xyzDev = std::dynamic_pointer_cast<XyzDevice>(nioDev);
        if (!xyzDev)
            continue;
        DiscoveredDevice dd;
        dd.device = nioDev;
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
| **SW D2C only** | `getAlignMode()` returns `SW`, `getD2CAlignFilter()` returns valid `DynalgoD2CAlign*` | CaptureSession calls `alignFilter->process(nioFs->nativeFrameSet, out)`; nativeFrameSet must be populated |
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
  ▼  xyzFrameSetToNio() — deep copy pixel data
  │  nioFs->nativeFrameSet = static_pointer_cast<void>(vendorFs)  [if SW D2C]
  │
DynalgoFrameSet (owns pixel data + optional opaque native handle)
  │
  ▼  videoCallback_(nioFs)  —  zero blocking in SDK callback
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
| Segfault in consumer thread | `DynalgoFrame::data` is empty or `rawData()` returns nullptr | Verify `xyzFrameSetToNio()` copies pixel data via `nf.data.assign()` |
| SW D2C returns garbage | `nativeFrameSet` is null or wrong type | Ensure `start()` sets `nioFs->nativeFrameSet = static_pointer_cast<void>(vendorFs)` and `XyzD2CAlign::process()` casts back correctly |
| Duplicate frames / mixed timestamps | Vendor callback fires on wrong thread / not synchronized | Use mutex + ready flags for async sources (see RsPipeline pattern) |
| Pipeline start fails silently | Exception swallowed in `start()` | Check `DYNALGO_LOG_ERROR_S` output; verify vendor config is complete before calling vendor start |
| `setupPipeline()` returns all-zero `DynalgoSensorInfo` | Downcast to concrete pipeline failed | Ensure factory creates correct `XyzPipeline` type paired with `XyzDevice` |
| USB memory error with multi-device | `usbfs_memory_mb` too low | `echo 256 \| sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb` |
| `discoverDevices()` link error | Driver factory not compiled | Ensure `app/driver/CMakeLists.txt:94` condition includes new vendor macro |

---

## 9. Unresolved / Open Items

1. **`DynalgoFrameSet::nativeFrameSet`** is a transitional escape valve. Goal: remove once all D2C alignment can operate on `DynalgoFrame::data` alone or through a cleaner abstraction.
2. **`DynalgoPipeline::enableStream(DynalgoStreamConfig)`** is currently a no-op for Orbbec (streams are enabled inside `setupPipeline()`). A future refactor should make stream enable/disable fully configurable through this API.
3. **Device property IDs** (`getIntProperty(int)`) pass raw vendor property IDs. Should be abstracted to `DynalgoPropertyID` enum if more vendors need it.
4. **Point cloud depth** (`isPointCloudDepth() == true`) is currently only used by RS-AC1. The 96×288 synthetic depth map is vendor-specific. A future depth abstraction may be needed for vendors with different point cloud layouts.
5. **`app/core/utils_c.h` and `utils_c.c`** contain C API functions (timestamp, key press). They are SDK-agnostic. The former `DYNALGO_DEVICE_VID 0x2bc5` and Orbbec vid/pid device-type checks have been moved to `app/driver/orbbec/dynalgo_ob_adapter.hpp` as `OB_DEVICE_VID`, `isGemini305Device()`, `isGemini305gDevice()`, `isAstraMiniDevice()`.
6. **FATAL_ERROR guard**: root `CMakeLists.txt` requires at least one `ENABLE_*` option to be ON. When adding a new vendor option, update the guard condition from `if(NOT ENABLE_ORBBEC AND NOT ENABLE_RS_AC1)` to also include the new option.
