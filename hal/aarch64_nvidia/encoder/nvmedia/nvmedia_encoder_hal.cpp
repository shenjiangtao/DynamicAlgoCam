#include <dynalgo/hal/encoder_hal.hpp>

namespace dynalgo {
class NvMediaEncoderHAL : public hal::IEncoderHAL {
public:
    NvMediaEncoderHAL() = default;
    ~NvMediaEncoderHAL() override = default;
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::IEncoderHAL* dynalgo_hal_encoder_create() { return new dynalgo::NvMediaEncoderHAL(); }
void dynalgo_hal_encoder_destroy(dynalgo::hal::IEncoderHAL* ptr) { delete ptr; }
}
