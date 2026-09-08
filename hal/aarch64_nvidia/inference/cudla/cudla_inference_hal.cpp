#include <dynalgo/hal/inference_hal.hpp>

namespace dynalgo {
class CUDLAInferenceHAL : public hal::IInferenceHAL {
public:
    CUDLAInferenceHAL() = default;
    ~CUDLAInferenceHAL() override = default;
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::IInferenceHAL* dynalgo_hal_inference_create() { return new dynalgo::CUDLAInferenceHAL(); }
void dynalgo_hal_inference_destroy(dynalgo::hal::IInferenceHAL* ptr) { delete ptr; }
}
