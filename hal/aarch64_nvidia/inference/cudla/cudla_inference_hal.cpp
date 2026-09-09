#include <dynalgo/hal/inference_hal.hpp>

namespace dynalgo {
class CudlaInferenceHAL : public hal::IInferenceHAL {
public:
    CudlaInferenceHAL() = default;
    ~CudlaInferenceHAL() override = default;
    // TODO: Implement all pure virtual methods
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::IInferenceHAL* dynalgo_hal_inference_create() { return new dynalgo::CudlaInferenceHAL(); }
void dynalgo_hal_inference_destroy(dynalgo::hal::IInferenceHAL* ptr) { delete ptr; }
}