# Migration Notice

All NIO application utility code has been moved from this directory to `app/`,
reorganized into a layered architecture:

| Layer | Directory | Library | Contents |
|-------|-----------|---------|----------|
| Core | `app/core/` | `nio_core` | Types, frame, device interfaces, logging, utilities |
| Driver | `app/driver/orbbec/` | `nio_drivers` | Orbbec SDK adapter |
| Driver | `app/driver/robosense/` | `nio_drivers` | RoboSense RS-AC1 adapter |
| Capture | `app/capture/` | `nio_capture` | Encoding, file I/O, session, viewer |
| Plugin | `app/plugins/opencv/` | `nio_opencv_plugin` | OpenCV color conversion (optional) |
| Tools | `app/tools/` | — | Offline Python/shell scripts |
| App | `app/nio_multi_capture/` | executable | Application entry point |

The old monolithic `ob_examples_utils` library is replaced by:
- `nio::core` — SDK-neutral, no external deps
- `nio::drivers` — device adapters (OrbbecSDK + optional rs_driver)
- `nio::capture` — recording pipeline (FFmpeg, SDL2)

If you need the old flat library, check git history before this commit.
