#include <dynalgo/hal/camera_hal.hpp>

namespace dynalgo {
class OrbbecCameraHAL : public hal::ICameraHAL {
public:
    OrbbecCameraHAL() = default;
    ~OrbbecCameraHAL() override = default;
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::ICameraHAL* dynalgo_hal_camera_create() { return new dynalgo::OrbbecCameraHAL(); }
void dynalgo_hal_camera_destroy(dynalgo::hal::ICameraHAL* ptr) { delete ptr; }
}
