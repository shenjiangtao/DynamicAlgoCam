# Engagement Loop (Phase C)

The Engagement Loop is the **perceive → locate → estimate → control** closed loop that runs only when `--engage-model` and `--engage-actuator` CLI flags are provided. Without these flags, the executable behaves identically to the baseline capture-only mode.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           EngagementLoop (dynalgo_algo)                     │
│                                                                             │
│  ┌─────────────────┐    ┌─────────────────┐    ┌────────────────────────┐  │
│  │ DynalgoModel    │    │ TargetSelector  │    │ DynalgoTrackBundle     │  │
│  │ Backend         │───▶│ (pickTarget)    │───▶│ (Kalman + 3D cache)    │  │
│  │                 │    │                 │    │                        │  │
│  │ detect()        │    │ Strategies:     │    │ init/update/predict()  │  │
│  │ → Dynalgo       │    │ • HIGHEST_SCORE │    │ lastX/Y/Z()/hasFix()   │  │
│  │ DetectionResult │    │ • NEAREST_DEPTH │    │                        │  │
│  └─────────────────┘    │ • LARGEST_AREA  │    └───────────┬────────────┘  │
│                          └─────────────────┘                │               │
│                                                             ▼               │
│  ┌─────────────────┐    ┌─────────────────┐    ┌────────────────────────┐  │
│  │ EngagementFrame │    │ DetectionCenter │    │ DynalgoActuator        │  │
│  │ Consumer        │───▶│ ToCamera3D      │───▶│ (abstract)             │  │
│  │ (FrameConsumer) │    │ (Phase B util)  │    │                        │  │
│  │                 │    │                 │    │ aimAt(X,Y,Z)           │  │
│  │ consume()       │    │ detection +     │    │ fire(durationMs)       │  │
│  │ → loop.onFrame()│    │ depth + intr    │    │                        │  │
│  └─────────────────┘    │ → (X,Y,Z) meters│    └────────────────────────┘  │
│                          └─────────────────┘                                │
└─────────────────────────────────────────────────────────────────────────────┘
```

## State Machine

```
IDLE ──(detection)──▶ LOCKING ──(3 stable frames)──▶ TRACKING
  ▲                                                 │
  │                                                 ▼
  │                                    ┌─────────────────────┐
  └────────(5 lost frames)────────────▶│       FIRING        │
                                        │ (aimAt → fire)      │
                                        └──────────┬──────────┘
                                                   │
                                        (cooldown 1000ms)
                                                   │
                                                   ▼
                                              LOST ──▶ IDLE
```

| State | Entry Condition | Exit Condition | Action |
|-------|-----------------|----------------|--------|
| `IDLE` | Start / LOST timeout | Any detection | Wait |
| `LOCKING` | Detection in IDLE | 3 consecutive stable detections → `TRACKING`<br/>No detection for 5 frames → `IDLE` | Accumulate detections |
| `TRACKING` | 3 stable detections in LOCKING | Detection lost for 5 frames → `LOST`<br/>3D fix + TRACKING → `FIRING` | Kalman update, predict |
| `FIRING` | 3D fix in TRACKING | Cooldown (1000ms) → `TRACKING` | `actuator->aimAt(X,Y,Z)` → `fire(durationMs)` |
| `LOST` | 5 frames no detection in TRACKING | Auto → `IDLE` | Reset tracker |

**Thresholds (compile-time constants in `dynalgo_engagement_loop.cpp`):**
- `LOCKING_TO_TRACKING_FRAMES = 3`
- `TRACKING_TO_LOST_FRAMES = 5`
- `FIRE_COOLDOWN_MS = 1000`

## Integration

### CLI Activation

```bash
# Dry-run with DUMMY model + DUMMY actuator
./dynamic_algo_cam --engage-model DUMMY --engage-actuator DUMMY --no-show

# With synthetic event simulation
./dynamic_algo_cam --engage-model DUMMY --engage-actuator DUMMY --enable-event-sim
```

### Component Wiring (in `dynamic_algo_cam.cpp`)

```cpp
// 1. Create model backend (from --engage-model flag)
auto model = createModelBackend(modelType, modelPath);

// 2. Create actuator (from --engage-actuator flag)
auto actuator = createActuator(actuatorType);

// 3. Create selector + track bundle
auto selector = std::make_unique<TargetSelector>(strategy);
auto trackBundle = std::make_unique<DynalgoTrackBundle>();

// 4. Create engagement loop
auto loop = std::make_unique<DynalgoEngagementLoop>(
    std::move(model), std::move(actuator),
    std::move(selector), std::move(trackBundle));

// 5. Create frame consumer and inject into session
auto consumer = std::make_unique<DynalgoEngagementFrameConsumer>(loop.get());
session->addFrameConsumer(std::move(consumer));

// 6. Store ownership (session deletes on destruction)
session->setEngagementLoop(loop.release(), model.release(), actuator.release());
```

### CaptureSession API (no signature changes to existing methods)

```cpp
// In dynalgo_capture_session.hpp
void addFrameConsumer(std::unique_ptr<FrameConsumer> consumer);
const DynalgoIntrinsic& depthIntrinsic() const;
float depthScale() const;
void setEngagementLoop(DynalgoEngagementLoop* loop,
                       DynalgoModelBackend* model,
                       DynalgoActuator* actuator);
```

The `FrameConsumer` chain is iterated in `videoConsumerLoop()` after all stream tasks. The `EngagementFrameConsumer` calls `loop_->onFrame(*frameSet)` for each fused frame set.

## Safety

| Mechanism | Implementation |
|-----------|----------------|
| **Dry-run default** | `DynalgoActuatorConfig::dryRun = true` — all control actions are no-ops |
| **No external deps** | `DynalgoActuator` / `DynalgoModelBackend` are pure abstract interfaces in `dynalgo_core` |
| **Self-registration** | Backends register via `registerModelBackend()` / `registerActuator()` at static init |
| **Opt-in only** | Loop is **not constructed** unless both `--engage-model` and `--engage-actuator` are provided |
| **No signature changes** | Existing `CaptureSession::setup/start/startVideoPipeline/stop` unchanged |

## Precondition for 3D Fix

The `detectionCenterToCamera3D()` function (Phase B) requires:

- **D2C-aligned depth frame** (Y16) — depth must be registered to color camera intrinsics
- Valid `DynalgoIntrinsic` (fx, fy, cx, cy > 0)
- Valid `depthScale` (from sensor info)
- Detection bbox within frame bounds

If any check fails, the function returns `false` and `TrackBundle` retains its last cached 3D fix.

## Building

```bash
# Default build includes dynalgo_algo and dynalgo_actuators static libs
cmake --build build -j$(nproc)

# Libraries produced:
#   build/lib/libdynalgo_algo.a
#   build/lib/libdynalgo_actuators.a
```

The main executable links these libraries. When `--engage-*` flags are absent, the engagement loop code is linked but never instantiated (zero runtime overhead).

## Extending

### New Model Backend

1. Implement `DynalgoModelBackend` in `app/models/<name>/`
2. Add to `app/models/CMakeLists.txt` (or separate static lib)
3. Register via `registerModelBackend(DynalgoModelType::YOUR_TYPE, creator)`

### New Actuator

1. Implement `DynalgoActuator` in `app/actuator/<name>/`
2. Add to `app/actuator/CMakeLists.txt`
3. Register via `registerActuator(DynalgoActuatorType::YOUR_TYPE, creator)`
4. **Link with `--whole-archive`** (see `dynalgo_actuator_factory.hpp`)

```cmake
target_link_libraries(dynamic_algo_cam PRIVATE
    "-Wl,--whole-archive" dynalgo::actuators "-Wl,--no-whole-archive"
)
```

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `createActuator(DUMMY)` returns `nullptr` | Missing `--whole-archive` | Add linker flags above |
| Loop stays in `IDLE` | No detections from model | Check model backend `detect()` output |
| Loop oscillates `LOCKING`↔`IDLE` | Unstable detections | Increase `LOCKING_TO_TRACKING_FRAMES` or improve model confidence |
| `FIRING` never reached | No 3D fix | Verify D2C alignment + depthScale + intrinsics |
| Actuator commands do nothing | `dryRun=true` (default) | Set `dryRun=false` in config (Phase C only uses DUMMY which ignores) |

## Files

| File | Role |
|------|------|
| `app/algo/dynalgo_engagement_loop.{hpp,cpp}` | State machine + orchestration |
| `app/algo/dynalgo_engagement_consumer.{hpp,cpp}` | FrameConsumer adapter |
| `app/algo/dynalgo_target_selector.{hpp,cpp}` | `pickTarget()` strategies |
| `app/algo/dynalgo_track_bundle.{hpp,cpp}` | Kalman + 3D cache |
| `app/algo/dummy_model_backend.{hpp,cpp}` | DUMMY self-registrar |
| `app/actuator/dummy_actuator.{hpp,cpp}` | DUMMY actuator self-registrar |
| `app/core/dynalgo_detection_to_3d.hpp` | Phase B 3D projection (header-only) |
| `app/core/dynalgo_actuator.{hpp,cpp}` | Actuator abstract + factory |
| `app/core/dynalgo_model.hpp` | Model backend abstract + factory |
| `app/capture/dynalgo_capture_session.{hpp,cpp}` | `addFrameConsumer()`, accessors |
| `app/dynamic_algo_cam/dynamic_algo_cam.cpp` | CLI wiring |

## License

MIT — same as main project. No third-party dependencies introduced in this layer.