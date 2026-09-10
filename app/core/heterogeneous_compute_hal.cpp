/*
 * heterogeneous_compute_hal.cpp - Heterogeneous compute abstraction implementation
 * UPEP: GPU/NPU/CPU operator abstraction
 */
#include <dynalgo/hal/heterogeneous_compute_hal.hpp>

#include <mutex>
#include <cstring>
#include <algorithm>
#include <iostream>

namespace dynalgo::hal {

// ============================================================================
// CPU Compute Context Implementation
// ============================================================================

class CPUComputeContext : public IComputeContext {
public:
    CPUComputeContext() = default;
    
    ResultVoid initialize(const ComputeResourceLimits& limits) override {
        m_limits = limits;
        return ResultVoid::ok();
    }
    
    ResultVoid allocateTensor(Tensor& tensor, MemoryType preferred_mem) override {
        if (tensor.desc.num_elements() <= 0) {
            return ResultVoid::err(static_cast<uint32_t>(ErrorCode::INVALID_ARG));
        }
        size_t size = tensor.desc.byte_size();
        tensor.host_ptr = std::aligned_alloc(64, size);
        if (!tensor.host_ptr) {
            return ResultVoid::err(static_cast<uint32_t>(ErrorCode::OUT_OF_MEMORY));
        }
        tensor.mem_type = MemoryType::HOST;
        tensor.buffer.handle = reinterpret_cast<uint64_t>(tensor.host_ptr);
        tensor.buffer.platform_id = 0;  // CPU
        return ResultVoid::ok();
    }
    
    ResultVoid freeTensor(Tensor& tensor) override {
        if (tensor.host_ptr) {
            std::free(tensor.host_ptr);
            tensor.host_ptr = nullptr;
        }
        tensor.buffer.handle = 0;
        tensor.buffer.platform_id = 0;
        return ResultVoid::ok();
    }
    
    ResultVoid copyTensor(const Tensor& src, Tensor& dst) override {
        if (src.desc.byte_size() != dst.desc.byte_size()) {
            return ResultVoid::err(static_cast<uint32_t>(ErrorCode::INVALID_ARG));
        }
        if (src.host_ptr && dst.host_ptr) {
            std::memcpy(dst.host_ptr, src.host_ptr, src.desc.byte_size());
        } else {
            return ResultVoid::err(static_cast<uint32_t>(ErrorCode::NOT_SUPPORTED));
        }
        return ResultVoid::ok();
    }
    
    ResultVoid synchronize() override {
        return ResultVoid::ok();
    }
    
    Result<void*> createStream() override {
        // CPU doesn't need streams, return dummy
        return Result<void*>::ok(nullptr);
    }
    
    ResultVoid destroyStream(void* stream) override {
        (void)stream;
        return ResultVoid::ok();
    }
    
    ResultVoid synchronizeStream(void* stream) override {
        (void)stream;
        return ResultVoid::ok();
    }
    
    Result<int64_t> getFreeMemory() const override {
        // Simplified - return a large value
        return Result<int64_t>::ok(1024LL * 1024 * 1024 * 16);  // 16GB
    }
    
    Result<int64_t> getTotalMemory() const override {
        return Result<int64_t>::ok(1024LL * 1024 * 1024 * 32);  // 32GB
    }
    
    ComputeBackendType getBackendType() const override {
        return ComputeBackendType::CPU;
    }

private:
    ComputeResourceLimits m_limits;
};

struct CPUComputeContextDeleter {
    void operator()(IComputeContext* ctx) const {
        delete ctx;
    }
};

std::unique_ptr<IComputeContext, CPUComputeContextDeleter> g_cpu_context;

// ============================================================================
// Operator Registry Implementation
// ============================================================================

class OperatorRegistryImpl : public IOperatorRegistry {
public:
    ResultVoid registerOperator(const OperatorImplementation& impl) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_operators[impl.signature.name].push_back(impl);
        // Sort by priority (higher first)
        auto& vec = m_operators[impl.signature.name];
        std::sort(vec.begin(), vec.end(), [](const auto& a, const auto& b) {
            return a.priority > b.priority;
        });
        return ResultVoid::ok();
    }
    
    ResultVoid unregisterOperator(const std::string& name, ComputeBackendType backend) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_operators.find(name);
        if (it == m_operators.end()) {
            return ResultVoid::err(static_cast<uint32_t>(ErrorCode::NOT_FOUND));
        }
        auto& vec = it->second;
        vec.erase(std::remove_if(vec.begin(), vec.end(), 
            [backend](const OperatorImplementation& impl) { return impl.backend == backend; }),
            vec.end());
        if (vec.empty()) m_operators.erase(it);
        return ResultVoid::ok();
    }
    
    std::vector<OperatorImplementation> getImplementations(
        const std::string& op_name, 
        ComputeBackendType preferred_backend) const override {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_operators.find(op_name);
        if (it == m_operators.end()) return {};
        
        std::vector<OperatorImplementation> result;
        for (const auto& impl : it->second) {
            if (preferred_backend == ComputeBackendType::CPU || impl.backend == preferred_backend) {
                result.push_back(impl);
            }
        }
        return result;
    }
    
    Result<OperatorImplementation> resolveOperator(
        const OperatorSignature& sig,
        ComputeBackendType preferred_backend,
        const ComputeResourceLimits& limits) const override {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_operators.find(sig.name);
        if (it == m_operators.end()) {
            return Result<OperatorImplementation>::err(static_cast<uint32_t>(ErrorCode::NOT_FOUND));
        }
        
        const auto& candidates = it->second;
        if (candidates.empty()) {
            return Result<OperatorImplementation>::err(static_cast<uint32_t>(ErrorCode::NOT_FOUND));
        }
        
        // Filter by backend preference
        std::vector<const OperatorImplementation*> filtered;
        for (const auto& impl : candidates) {
            if (preferred_backend == ComputeBackendType::CPU || impl.backend == preferred_backend) {
                // Check resource limits
                bool ok = true;
                // Add resource checking logic here if needed
                if (ok) filtered.push_back(&impl);
            }
        }
        
        if (filtered.empty()) {
            // Fall back to any available
            for (const auto& impl : candidates) {
                filtered.push_back(&impl);
            }
        }
        
        if (filtered.empty()) {
            return Result<OperatorImplementation>::err(static_cast<uint32_t>(ErrorCode::NOT_SUPPORTED));
        }
        
        // Return highest priority
        return Result<OperatorImplementation>::ok(*filtered[0]);
    }
    
    std::vector<std::string> listOperators() const override {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::vector<std::string> result;
        result.reserve(m_operators.size());
        for (const auto& pair : m_operators) {
            result.push_back(pair.first);
        }
        return result;
    }
    
    std::vector<ComputeBackendType> getOperatorBackends(const std::string& op_name) const override {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_operators.find(op_name);
        if (it == m_operators.end()) return {};
        
        std::vector<ComputeBackendType> backends;
        for (const auto& impl : it->second) {
            if (std::find(backends.begin(), backends.end(), impl.backend) == backends.end()) {
                backends.push_back(impl.backend);
            }
        }
        return backends;
    }

private:
    mutable std::mutex m_mutex;
    std::unordered_map<std::string, std::vector<OperatorImplementation>> m_operators;
};

std::unique_ptr<OperatorRegistryImpl> g_operator_registry;

// ============================================================================
// Factory Functions
// ============================================================================

Result<IComputeContext*> ComputeContextFactory::create(ComputeBackendType backend, const ComputeResourceLimits& limits) {
    switch (backend) {
        case ComputeBackendType::CPU: {
            if (!g_cpu_context) {
                g_cpu_context = std::unique_ptr<IComputeContext, CPUComputeContextDeleter>(new CPUComputeContext());
            }
            auto result = g_cpu_context->initialize(limits);
            if (!result.is_ok()) return Result<IComputeContext*>::err(result.error());
            return Result<IComputeContext*>::ok(g_cpu_context.get());
        }
        case ComputeBackendType::CUDA:
        case ComputeBackendType::TENSORRT:
        case ComputeBackendType::DLA:
        case ComputeBackendType::CUDLA:
        case ComputeBackendType::RKNN:
        case ComputeBackendType::NPU_GENERIC:
        case ComputeBackendType::OPENCL:
        case ComputeBackendType::VULKAN:
            return Result<IComputeContext*>::err(static_cast<uint32_t>(ErrorCode::NOT_IMPLEMENTED));
        default:
            return Result<IComputeContext*>::err(static_cast<uint32_t>(ErrorCode::NOT_SUPPORTED));
    }
}

void ComputeContextFactory::destroy(IComputeContext* ctx) {
    // CPU context is singleton, don't destroy
    (void)ctx;
}

std::vector<ComputeBackendType> ComputeContextFactory::getAvailableBackends() {
    std::vector<ComputeBackendType> backends = {ComputeBackendType::CPU};
    // TODO: Detect available GPU/NPU backends
    return backends;
}

void ComputeContextFactory::registerBackend(ComputeBackendType, CreateFunc, DestroyFunc) {
    // TODO: Implement dynamic backend registration
}

IOperatorRegistry& OperatorRegistry::instance() {
    if (!g_operator_registry) {
        g_operator_registry = std::make_unique<OperatorRegistryImpl>();
    }
    return *g_operator_registry;
}

// ============================================================================
// Built-in CPU Operator Implementations
// ============================================================================

namespace {

// Element-wise addition
ResultVoid cpu_add(const std::vector<Tensor>& inputs, std::vector<Tensor>& outputs, 
                   const OperatorAttrs&, void*) {
    if (inputs.size() != 2 || outputs.size() != 1) {
        return ResultVoid::err(static_cast<uint32_t>(ErrorCode::INVALID_ARG));
    }
    const auto& a = inputs[0];
    const auto& b = inputs[1];
    auto& out = outputs[0];
    
    if (a.desc.num_elements() != b.desc.num_elements() || a.desc.num_elements() != out.desc.num_elements()) {
        return ResultVoid::err(static_cast<uint32_t>(ErrorCode::INVALID_ARG));
    }
    
    size_t n = a.desc.num_elements();
    if (a.desc.dtype == DataType::FLOAT32 && b.desc.dtype == DataType::FLOAT32) {
        const float* pa = a.data<float>();
        const float* pb = b.data<float>();
        float* pout = out.data<float>();
        for (size_t i = 0; i < n; ++i) pout[i] = pa[i] + pb[i];
    } else if (a.desc.dtype == DataType::INT32 && b.desc.dtype == DataType::INT32) {
        const int32_t* pa = a.data<int32_t>();
        const int32_t* pb = b.data<int32_t>();
        int32_t* pout = out.data<int32_t>();
        for (size_t i = 0; i < n; ++i) pout[i] = pa[i] + pb[i];
    } else {
        return ResultVoid::err(static_cast<uint32_t>(ErrorCode::NOT_SUPPORTED));
    }
    return ResultVoid::ok();
}

// ReLU activation
ResultVoid cpu_relu(const std::vector<Tensor>& inputs, std::vector<Tensor>& outputs,
                    const OperatorAttrs&, void*) {
    if (inputs.size() != 1 || outputs.size() != 1) {
        return ResultVoid::err(static_cast<uint32_t>(ErrorCode::INVALID_ARG));
    }
    const auto& in = inputs[0];
    auto& out = outputs[0];
    
    if (in.desc.num_elements() != out.desc.num_elements()) {
        return ResultVoid::err(static_cast<uint32_t>(ErrorCode::INVALID_ARG));
    }
    
    size_t n = in.desc.num_elements();
    if (in.desc.dtype == DataType::FLOAT32) {
        const float* pin = in.data<float>();
        float* pout = out.data<float>();
        for (size_t i = 0; i < n; ++i) pout[i] = std::max(0.0f, pin[i]);
    } else {
        return ResultVoid::err(static_cast<uint32_t>(ErrorCode::NOT_SUPPORTED));
    }
    return ResultVoid::ok();
}

// Point cloud downsampling (voxel grid)
ResultVoid cpu_pointcloud_downsample(const std::vector<Tensor>& inputs, std::vector<Tensor>& outputs,
                                     const OperatorAttrs& attrs, void*) {
    if (inputs.size() != 1 || outputs.size() != 1) {
        return ResultVoid::err(static_cast<uint32_t>(ErrorCode::INVALID_ARG));
    }
    
    float voxel_size = attrs.get<float>("voxel_size", 0.1f);
    // Simplified implementation - just copy for now
    // Real implementation would do voxel grid filtering
    return cpu_add(inputs, outputs, attrs, nullptr);  // Placeholder
}

} // anonymous namespace

// ============================================================================
// Registration Function
// ============================================================================

void registerBuiltinOperators() {
    auto& registry = OperatorRegistry::instance();
    
    // Register CPU operators
    OperatorImplementation add_impl;
    add_impl.signature = {"add", OperatorCategory::ELEMENTWISE, 
                          {TensorDesc{DataType::FLOAT32, TensorLayout::NCHW, {1, -1, -1, -1}},
                           TensorDesc{DataType::FLOAT32, TensorLayout::NCHW, {1, -1, -1, -1}}},
                          {TensorDesc{DataType::FLOAT32, TensorLayout::NCHW, {1, -1, -1, -1}}},
                          {}};
    add_impl.backend = ComputeBackendType::CPU;
    add_impl.priority = 100;
    add_impl.impl = cpu_add;
    registry.registerOperator(add_impl);
    
    OperatorImplementation relu_impl;
    relu_impl.signature = {"relu", OperatorCategory::ACTIVATION,
                          {TensorDesc{DataType::FLOAT32, TensorLayout::NCHW, {1, -1, -1, -1}}},
                          {TensorDesc{DataType::FLOAT32, TensorLayout::NCHW, {1, -1, -1, -1}}},
                          {}};
    relu_impl.backend = ComputeBackendType::CPU;
    relu_impl.priority = 100;
    relu_impl.impl = cpu_relu;
    registry.registerOperator(relu_impl);
    
    OperatorImplementation pc_downsample_impl;
    pc_downsample_impl.signature.name = "pointcloud_downsample";
    pc_downsample_impl.signature.category = OperatorCategory::POINT_CLOUD;
    pc_downsample_impl.signature.inputs = {TensorDesc{DataType::FLOAT32, TensorLayout::STRUCTURED, {-1, 4}}};
    pc_downsample_impl.signature.outputs = {TensorDesc{DataType::FLOAT32, TensorLayout::STRUCTURED, {-1, 4}}};
    pc_downsample_impl.signature.attrs = {};
    pc_downsample_impl.backend = ComputeBackendType::CPU;
    pc_downsample_impl.priority = 100;
    pc_downsample_impl.impl = cpu_pointcloud_downsample;
    registry.registerOperator(pc_downsample_impl);
}

} // namespace dynalgo::hal