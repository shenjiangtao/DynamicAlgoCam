#include <dynalgo/hal/camera_hal.hpp>

namespace dynalgo {
class NvSIPLCameraHAL : public hal::ICameraHAL {
public:
    NvSIPLCameraHAL() = default;
    ~NvSIPLCameraHAL() override = default;
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::ICameraHAL* dynalgo_hal_camera_create() { return new dynalgo::NvSIPLCameraHAL(); }
void dynalgo_hal_camera_destroy(dynalgo::hal::ICameraHAL* ptr) { delete ptr; }
}
