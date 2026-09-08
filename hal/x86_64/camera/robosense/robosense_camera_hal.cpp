#include <dynalgo/hal/camera_hal.hpp>

namespace dynalgo {
class RobosenseCameraHAL : public hal::ICameraHAL {
public:
    RobosenseCameraHAL() = default;
    ~RobosenseCameraHAL() override = default;
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::ICameraHAL* dynalgo_hal_camera_create() { return new dynalgo::RobosenseCameraHAL(); }
void dynalgo_hal_camera_destroy(dynalgo::hal::ICameraHAL* ptr) { delete ptr; }
}
