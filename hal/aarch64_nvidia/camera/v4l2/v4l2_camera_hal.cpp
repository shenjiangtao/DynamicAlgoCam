#include <dynalgo/hal/camera_hal.hpp>

namespace dynalgo {
class V4L2CameraHAL : public hal::ICameraHAL {
public:
    V4L2CameraHAL() = default;
    ~V4L2CameraHAL() override = default;
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::ICameraHAL* dynalgo_hal_camera_create() { return new dynalgo::V4L2CameraHAL(); }
void dynalgo_hal_camera_destroy(dynalgo::hal::ICameraHAL* ptr) { delete ptr; }
}
