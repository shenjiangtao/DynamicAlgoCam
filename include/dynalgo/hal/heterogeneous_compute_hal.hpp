/*
 * heterogenous_compute_hal.hpp - Heterogeneous compute abstraction layer
 * UPEP: GPU/NPU/CPU operator abstraction, algorithm plugins declare compute needs,
 * framework schedules to appropriate hardware backend
 */
#pragma once

#include "hal_types.hpp"
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <variant>
#include <unordered_map>

namespace dynalgo::hal {

// ============================================================================
// UPEP: Compute Backend Types
// ============================================================================

enum class ComputeBackendType : uint32_t {
    CPU = 0,
    CUDA = 1,
    OPENCL = 2,
    VULKAN = 3,
    TENSORRT = 4,
    DLA = 5,
    CUDLA = 6,
    RKNN = 7,
    NPU_GENERIC = 8,
    VENDOR_BASE = 0x10000,
};

enum class DataType : uint32_t {
    UNKNOWN = 0,
    FLOAT32 = 1,
    FLOAT16 = 2,
    INT8 = 3,
    INT16 = 4,
    INT32 = 5,
    UINT8 = 6,
    UINT16 = 7,
    UINT32 = 8,
    BOOL = 9,
    COMPLEX64 = 10,
    COMPLEX128 = 11,
};

enum class MemoryType : uint32_t {
    HOST = 0,
    DEVICE = 1,
    UNIFIED = 2,
    PINNED = 3,
    DMA_BUF = 4,
    NVSCI_BUF = 5,
};

enum class TensorLayout : uint32_t {
    UNKNOWN = 0,
    NCHW = 1,
    NHWC = 2,
    CHW = 3,
    HWC = 4,
    OIHW = 5,
    HWIO = 6,
    STRUCTURED = 7,     // For point clouds and structured data
    CUSTOM = 0xFFFF,
};

// ============================================================================
// UPEP: Tensor Descriptor (ABI-stable, no STL in public API)
// ============================================================================

struct TensorDesc {
    DataType dtype = DataType::FLOAT32;
    TensorLayout layout = TensorLayout::NCHW;
    std::vector<int64_t> shape;  // Dynamic shape
    int64_t num_elements() const {
        int64_t n = 1;
        for (auto d : shape) n *= d;
        return n;
    }
    size_t byte_size() const {
        size_t dt_size = 0;
        switch (dtype) {
            case DataType::FLOAT32: case DataType::INT32: case DataType::UINT32: dt_size = 4; break;
            case DataType::FLOAT16: case DataType::INT16: case DataType::UINT16: dt_size = 2; break;
            case DataType::INT8: case DataType::UINT8: case DataType::BOOL: dt_size = 1; break;
            case DataType::COMPLEX64: dt_size = 8; break;
            case DataType::COMPLEX128: dt_size = 16; break;
            default: dt_size = 4;
        }
        return num_elements() * dt_size;
    }
};

struct Tensor {
    TensorDesc desc;
    BufferHandle buffer;
    MemoryType mem_type = MemoryType::HOST;
    void* host_ptr = nullptr;  // Valid only for HOST memory
    
    // Convenience accessors
    template<typename T>
    T* data() { 
        if (host_ptr) return static_cast<T*>(host_ptr);
        if (buffer.handle) return reinterpret_cast<T*>(buffer.handle);
        return nullptr; 
    }
    
    template<typename T>
    const T* data() const { 
        if (host_ptr) return static_cast<const T*>(host_ptr);
        if (buffer.handle) return reinterpret_cast<const T*>(buffer.handle);
        return nullptr; 
    }
};

// ============================================================================
// UPEP: Operator Registry
// ============================================================================

enum class OperatorCategory : uint32_t {
    UNKNOWN = 0,
    CONVOLUTION = 1,
    POOLING = 2,
    ACTIVATION = 3,
    NORMALIZATION = 4,
    ELEMENTWISE = 5,
    REDUCTION = 6,
    MATRIX_MUL = 7,
    TRANSFORM = 8,
    POINT_CLOUD = 9,    // Downsample, ICP, registration
    STEREO = 10,        // Disparity, depth, rectification
    CUSTOM = 0xFFFF,
};

struct OperatorAttrs {
    std::map<std::string, std::variant<int64_t, float, double, std::string, bool>> attrs;
    
    template<typename T>
    T get(const std::string& key, T default_val = T{}) const {
        auto it = attrs.find(key);
        if (it != attrs.end()) {
            try { return std::get<T>(it->second); }
            catch (...) { return default_val; }
        }
        return default_val;
    }
    
    void set(const std::string& key, const std::variant<int64_t, float, double, std::string, bool>& val) {
        attrs[key] = val;
    }
};

struct OperatorSignature {
    std::string name;
    OperatorCategory category;
    std::vector<TensorDesc> inputs;
    std::vector<TensorDesc> outputs;
    OperatorAttrs attrs;
    std::string backend_requirements;  // JSON string for backend-specific needs
};

using OperatorImplFunc = std::function<ResultVoid(
    const std::vector<Tensor>& inputs,
    std::vector<Tensor>& outputs,
    const OperatorAttrs& attrs,
    void* stream  // Backend-specific stream/context
)>;

struct OperatorImplementation {
    OperatorSignature signature;
    ComputeBackendType backend;
    int priority = 0;  // Higher = preferred
    OperatorImplFunc impl;
    std::string vendor;  // Vendor-specific implementation
};

// ============================================================================
// UPEP: Compute Context (per-task resource isolation)
// ============================================================================

struct ComputeResourceLimits {
    int64_t max_memory_bytes = 0;      // 0 = unlimited
    int max_compute_units = 0;          // 0 = unlimited
    double max_power_watts = 0.0;
    int preferred_dla_core = -1;        // -1 = any
    bool require_cuda_stream = false;
};

class IComputeContext {
public:
    virtual ~IComputeContext() = default;
    
    virtual ResultVoid initialize(const ComputeResourceLimits& limits) = 0;
    virtual ResultVoid allocateTensor(Tensor& tensor, MemoryType preferred_mem = MemoryType::DEVICE) = 0;
    virtual ResultVoid freeTensor(Tensor& tensor) = 0;
    virtual ResultVoid copyTensor(const Tensor& src, Tensor& dst) = 0;  // Host<->Device, Device<->Device
    virtual ResultVoid synchronize() = 0;  // Wait for all pending ops
    
    // Stream management for async execution
    virtual Result<void*> createStream() = 0;
    virtual ResultVoid destroyStream(void* stream) = 0;
    virtual ResultVoid synchronizeStream(void* stream) = 0;
    
    // Query
    virtual Result<int64_t> getFreeMemory() const = 0;
    virtual Result<int64_t> getTotalMemory() const = 0;
    virtual ComputeBackendType getBackendType() const = 0;
};

class ComputeContextFactory {
public:
    using CreateFunc = IComputeContext* (*)(ComputeBackendType);
    using DestroyFunc = void (*)(IComputeContext*);
    
    static Result<IComputeContext*> create(ComputeBackendType backend, const ComputeResourceLimits& limits = {});
    static void destroy(IComputeContext* ctx);
    
    static std::vector<ComputeBackendType> getAvailableBackends();
    static void registerBackend(ComputeBackendType backend, CreateFunc create, DestroyFunc destroy);
};

// ============================================================================
// UPEP: Operator Registry (Global)
// ============================================================================

class IOperatorRegistry {
public:
    virtual ~IOperatorRegistry() = default;
    
    // Register operator implementation
    virtual ResultVoid registerOperator(const OperatorImplementation& impl) = 0;
    virtual ResultVoid unregisterOperator(const std::string& name, ComputeBackendType backend) = 0;
    
    // Query
    virtual std::vector<OperatorImplementation> getImplementations(
        const std::string& op_name, 
        ComputeBackendType preferred_backend = ComputeBackendType::CPU) const = 0;
    
    // Find best implementation for given signature and backend
    virtual Result<OperatorImplementation> resolveOperator(
        const OperatorSignature& sig,
        ComputeBackendType preferred_backend = ComputeBackendType::CPU,
        const ComputeResourceLimits& limits = {}) const = 0;
    
    // List all registered operators
    virtual std::vector<std::string> listOperators() const = 0;
    virtual std::vector<ComputeBackendType> getOperatorBackends(const std::string& op_name) const = 0;
};

class OperatorRegistry {
public:
    static IOperatorRegistry& instance();
    
    // Convenience: register CPU implementation
    static ResultVoid registerCPUOperator(const OperatorImplementation& impl) {
        return instance().registerOperator(impl);
    }
    
    // Execute operator with auto backend selection
    static ResultVoid execute(
        const std::string& op_name,
        const std::vector<Tensor>& inputs,
        std::vector<Tensor>& outputs,
        const OperatorAttrs& attrs = {},
        ComputeBackendType preferred = ComputeBackendType::CPU,
        void* stream = nullptr) {
        // Resolve best implementation
        OperatorSignature sig;
        sig.name = op_name;
        sig.inputs.reserve(inputs.size());
        sig.outputs.reserve(outputs.size());
        for (auto& t : inputs) sig.inputs.push_back(t.desc);
        for (auto& t : outputs) sig.outputs.push_back(t.desc);
        sig.attrs = attrs;
        
        auto resolve_result = instance().resolveOperator(sig, preferred);
        if (!resolve_result.is_ok()) return ResultVoid::err(resolve_result.error());
        
        auto impl = resolve_result.value();
        if (impl.impl) {
            return impl.impl(inputs, outputs, attrs, stream);
        }
        return ResultVoid::err(static_cast<uint32_t>(ErrorCode::NOT_IMPLEMENTED));
    }
};

} // namespace dynalgo::hal