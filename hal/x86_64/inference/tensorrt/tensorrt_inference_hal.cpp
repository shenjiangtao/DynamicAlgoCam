#include <dynalgo/hal/inference_hal.hpp>

namespace dynalgo {
class TensorRTInferenceHAL : public hal::IInferenceHAL {
public:
    TensorRTInferenceHAL() = default;
    ~TensorRTInferenceHAL() override = default;

    hal::ResultVoid initialize(const hal::ModelConfig&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid deinitialize() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<hal::ModelInfo> getModelInfo() const override { return hal::Result<hal::ModelInfo>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid infer(const std::vector<hal::TensorInfo>&, hal::InferenceResult&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid inferAsync(const std::vector<hal::TensorInfo>&, std::function<void(hal::Result<hal::InferenceResult>)>) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid inferBatch(const std::vector<std::vector<hal::TensorInfo>>&, std::vector<hal::InferenceResult>&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<std::vector<hal::TensorInfo>> getInputSpecs() const override { return hal::Result<std::vector<hal::TensorInfo>>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<std::vector<hal::TensorInfo>> getOutputSpecs() const override { return hal::Result<std::vector<hal::TensorInfo>>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setInput(const std::string&, const hal::TensorInfo&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<hal::TensorInfo> getOutput(const std::string&) override { return hal::Result<hal::TensorInfo>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<hal::InferenceStats> getStats() override { return hal::Result<hal::InferenceStats>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid resetStats() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setProfile(const std::string&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<std::vector<std::string>> getAvailableProfiles() override { return hal::Result<std::vector<std::string>>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<std::string> getName() const override { return hal::Result<std::string>::ok("TensorRT Inference HAL"); }
    hal::Result<std::string> getVendor() const override { return hal::Result<std::string>::ok("nvidia"); }
    hal::Result<std::string> getVersion() const override { return hal::Result<std::string>::ok("1.0.0"); }
    hal::ResultVoid vendorCommand(uint32_t, const void*, size_t, void*, size_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    bool isInitialized() const override { return false; }
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::IInferenceHAL* dynalgo_hal_inference_create() { return new dynalgo::TensorRTInferenceHAL(); }
void dynalgo_hal_inference_destroy(dynalgo::hal::IInferenceHAL* ptr) { delete ptr; }
}
