#pragma once

#include "hal_types.hpp"
#include <string>
#include <vector>
#include <functional>
#include <map>

namespace dynalgo::hal {

enum class InferencePrecision : uint32_t {
    FP32 = 0,
    FP16 = 1,
    INT8 = 2,
    INT4 = 3,
    BF16 = 4,
    TF32 = 5,
};

enum class InferenceDevice : uint32_t {
    AUTO = 0,
    GPU = 1,
    DLA_0 = 2,
    DLA_1 = 3,
    CPU = 4,
    NPU = 5,
    DSP = 6,
};

enum class ModelFormat : uint32_t {
    ONNX = 0,
    TENSORRT_ENGINE = 1,
    TFLITE = 2,
    PYTORCH = 3,
    CAFFE = 4,
    OPENVINO_IR = 5,
    NCNN = 6,
    MNN = 7,
};

struct TensorInfo {
    std::string name;
    std::vector<int32_t> shape;
    PixelFormat format = PixelFormat::UNKNOWN;
    BufferHandle buffer;
    FenceHandle fence;
    std::vector<uint32_t> strides;
    size_t offset = 0;
    size_t size = 0;

    size_t get_element_count() const {
        size_t count = 1;
        for (auto dim : shape) count *= dim;
        return count;
    }
    size_t get_byte_size() const {
        return get_element_count() * (pixel_format_bpp(format) / 8);
    }
};

struct ModelConfig {
    std::string model_path;
    ModelFormat format = ModelFormat::ONNX;
    InferencePrecision precision = InferencePrecision::FP16;
    InferenceDevice device = InferenceDevice::AUTO;
    int32_t device_id = 0;
    int32_t dla_core = -1;
    uint32_t max_batch_size = 1;
    uint32_t workspace_size_mb = 512;
    bool enable_fp16 = true;
    bool enable_int8 = false;
    std::string calibration_cache;
    std::vector<std::string> input_names;
    std::vector<std::string> output_names;
    std::map<std::string, std::vector<int32_t>> input_shapes;
    std::map<std::string, std::string> custom_options;
    void* vendor_params = nullptr;
    size_t vendor_params_size = 0;
};

struct ModelInfo {
    std::string name;
    std::string version;
    std::vector<TensorInfo> inputs;
    std::vector<TensorInfo> outputs;
    size_t param_count = 0;
    size_t flops = 0;
    InferencePrecision precision = InferencePrecision::FP16;
    InferenceDevice device = InferenceDevice::AUTO;
};

struct InferenceResult {
    std::vector<TensorInfo> outputs;
    uint64_t latency_us = 0;
    uint64_t preprocess_us = 0;
    uint64_t inference_us = 0;
    uint64_t postprocess_us = 0;
    FenceHandle fence;
    uint32_t frame_id = 0;
    std::string error_message;
    bool success = true;
};

struct InferenceStats {
    uint64_t total_inferences = 0;
    uint64_t total_latency_us = 0;
    uint64_t min_latency_us = UINT64_MAX;
    uint64_t max_latency_us = 0;
    float avg_fps = 0.0f;
    uint64_t errors = 0;
};

class IInferenceHAL {
public:
    virtual ~IInferenceHAL() = default;

    virtual ResultVoid initialize(const ModelConfig& config) = 0;
    virtual ResultVoid deinitialize() = 0;

    virtual Result<ModelInfo> getModelInfo() const = 0;

    virtual ResultVoid infer(const std::vector<TensorInfo>& inputs,
                             InferenceResult& output) = 0;

    virtual ResultVoid inferAsync(const std::vector<TensorInfo>& inputs,
                                  std::function<void(Result<InferenceResult>)> callback) = 0;

    virtual ResultVoid inferBatch(const std::vector<std::vector<TensorInfo>>& batch_inputs,
                                  std::vector<InferenceResult>& batch_outputs) = 0;

    virtual Result<std::vector<TensorInfo>> getInputSpecs() const = 0;
    virtual Result<std::vector<TensorInfo>> getOutputSpecs() const = 0;

    virtual ResultVoid setInput(const std::string& name, const TensorInfo& tensor) = 0;
    virtual Result<TensorInfo> getOutput(const std::string& name) = 0;

    virtual Result<InferenceStats> getStats() = 0;
    virtual ResultVoid resetStats() = 0;

    virtual ResultVoid setProfile(const std::string& profile_name) = 0;
    virtual Result<std::vector<std::string>> getAvailableProfiles() = 0;

    virtual Result<std::string> getName() const = 0;
    virtual Result<std::string> getVendor() const = 0;
    virtual Result<std::string> getVersion() const = 0;

    virtual ResultVoid vendorCommand(uint32_t cmd_id,
                                     const void* in_data, size_t in_size,
                                     void* out_data, size_t out_size) = 0;

    virtual bool isInitialized() const = 0;
};

class InferenceHALFactory {
public:
    using CreateFunc = IInferenceHAL* (*)();
    using DestroyFunc = void (*)(IInferenceHAL*);

    static Result<IInferenceHAL*> create(const std::string& platform, const std::string& vendor);
    static Result<IInferenceHAL*> create(const ModelConfig& config);
    static void destroy(IInferenceHAL* hal);

    static std::vector<std::string> getSupportedVendors(const std::string& platform);
    static void registerVendor(const std::string& platform, const std::string& vendor,
                               CreateFunc create, DestroyFunc destroy);
};

} // namespace dynalgo::hal