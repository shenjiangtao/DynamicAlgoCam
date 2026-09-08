#include <dynalgo/hal/camera_hal.hpp>

namespace dynalgo {
class StereoCameraHAL : public hal::ICameraHAL {
public:
    StereoCameraHAL() = default;
    ~StereoCameraHAL() override = default;
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::ICameraHAL* dynalgo_hal_camera_create() { return new dynalgo::StereoCameraHAL(); }
void dynalgo_hal_camera_destroy(dynalgo::hal::ICameraHAL* ptr) { delete ptr; }
}
