#include <dynalgo/hal/encoder_hal.hpp>

namespace dynalgo {
class NvencEncoderHAL : public hal::IEncoderHAL {
public:
    NvencEncoderHAL() = default;
    ~NvencEncoderHAL() override = default;
    // TODO: Implement all pure virtual methods
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::IEncoderHAL* dynalgo_hal_encoder_create() { return new dynalgo::NvencEncoderHAL(); }
void dynalgo_hal_encoder_destroy(dynalgo::hal::IEncoderHAL* ptr) { delete ptr; }
}