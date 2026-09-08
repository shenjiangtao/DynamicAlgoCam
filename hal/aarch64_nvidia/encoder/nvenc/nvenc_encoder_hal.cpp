#include <dynalgo/hal/encoder_hal.hpp>

namespace dynalgo {
class NVENCEncoderHAL : public hal::IEncoderHAL {
public:
    NVENCEncoderHAL() = default;
    ~NVENCEncoderHAL() override = default;
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::IEncoderHAL* dynalgo_hal_encoder_create() { return new dynalgo::NVENCEncoderHAL(); }
void dynalgo_hal_encoder_destroy(dynalgo::hal::IEncoderHAL* ptr) { delete ptr; }
}
