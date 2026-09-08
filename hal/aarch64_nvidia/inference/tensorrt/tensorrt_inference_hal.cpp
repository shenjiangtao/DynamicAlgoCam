#include <dynalgo/hal/inference_hal.hpp>

namespace dynalgo {
class TensorRTInferenceHAL : public hal::IInferenceHAL {
public:
    TensorRTInferenceHAL() = default;
    ~TensorRTInferenceHAL() override = default;
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::IInferenceHAL* dynalgo_hal_inference_create() { return new dynalgo::TensorRTInferenceHAL(); }
void dynalgo_hal_inference_destroy(dynalgo::hal::IInferenceHAL* ptr) { delete ptr; }
}
