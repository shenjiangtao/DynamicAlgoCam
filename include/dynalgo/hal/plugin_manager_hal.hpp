/*
 * plugin_manager_hal.hpp - Dynamic algorithm plugin manager
 * UPEP: Runtime plugin loading, unloading, hot-swap, DAG task scheduling
 */
#pragma once

#include "hal_types.hpp"
#include "heterogeneous_compute_hal.hpp"
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <queue>
#include <unordered_map>

namespace dynalgo::hal {

// ============================================================================
// UPEP: Plugin Metadata and Manifest
// ============================================================================

enum class PluginType : uint32_t {
    UNKNOWN = 0,
    STEREO_MATCH = 1,
    LIDAR_SEGMENTATION = 2,
    IMAGE_DETECTION = 3,
    MULTIMODAL_FUSION = 4,
    TRACKING = 5,
    SLAM = 6,
    CUSTOM = 0xFFFF,
};

enum class PluginState : uint32_t {
    UNLOADED = 0,
    LOADING = 1,
    LOADED = 2,
    INITIALIZING = 3,
    READY = 4,
    RUNNING = 5,
    STOPPING = 6,
    ERROR = 7,
};

struct ResourceRequirements {
    int64_t min_memory_mb = 0;
    int64_t max_memory_mb = 0;
    int min_compute_units = 0;
    bool requires_gpu = false;
    bool requires_npu = false;
    std::string preferred_backend;  // "cuda", "tensorrt", "rk3588_npu", etc.
    int dla_core = -1;
};

struct TensorSpec {
    std::string name;
    DataType dtype;
    TensorLayout layout;
    std::vector<int64_t> shape;  // -1 for dynamic dimensions
};

struct PluginPort {
    std::string name;
    std::vector<TensorSpec> tensors;
    bool required = true;
};

struct PluginManifest {
    std::string name;
    std::string version;
    std::string vendor;
    PluginType type;
    std::string description;
    
    std::vector<PluginPort> inputs;
    std::vector<PluginPort> outputs;
    
    ResourceRequirements resources;
    
    // Dependencies
    std::vector<std::string> required_operators;  // Operator names from registry
    std::vector<std::string> required_plugins;    // Other plugin names
    
    // Security
    std::string signature;  // SHA256 of plugin binary
    bool verified = false;
    
    // Configuration schema (JSON)
    std::string config_schema;
};

// ============================================================================
// UPEP: Plugin Instance Interface
// ============================================================================

class IPlugin {
public:
    virtual ~IPlugin() = default;
    
    virtual ResultVoid initialize(const std::string& config_json) = 0;
    virtual ResultVoid deinitialize() = 0;
    
    virtual ResultVoid start() = 0;
    virtual ResultVoid stop() = 0;
    
    // Execute one inference step
    virtual ResultVoid execute(
        const std::map<std::string, FrameBufferPtr>& inputs,
        std::map<std::string, FrameBufferPtr>& outputs,
        void* stream = nullptr) = 0;
    
    // Async execution
    virtual ResultVoid executeAsync(
        const std::map<std::string, FrameBufferPtr>& inputs,
        std::function<void(const std::map<std::string, FrameBufferPtr>&, ResultVoid)> callback,
        void* stream = nullptr) = 0;
    
    // State management
    virtual PluginState getState() const = 0;
    virtual Result<std::string> getLastError() const = 0;
    
    // Configuration
    virtual ResultVoid setConfig(const std::string& config_json) = 0;
    virtual Result<std::string> getConfig() const = 0;
    
    // Metadata
    virtual Result<PluginManifest> getManifest() const = 0;
    
    // Resource usage
    virtual Result<int64_t> getMemoryUsage() const = 0;
    virtual Result<double> getComputeUtilization() const = 0;
    
    // Vendor extension
    virtual ResultVoid vendorCommand(uint32_t cmd_id,
                                     const void* in_data, size_t in_size,
                                     void* out_data, size_t out_size) = 0;
};

// ============================================================================
// UPEP: Plugin Manager
// ============================================================================

class IPluginManager {
public:
    virtual ~IPluginManager() = default;
    
    // Load/unload plugins
    virtual Result<std::string> loadPlugin(const std::string& plugin_path,
                                           const std::string& instance_name = "") = 0;
    virtual ResultVoid unloadPlugin(const std::string& instance_name) = 0;
    
    // Hot-swap: atomic replace running plugin
    virtual ResultVoid hotSwapPlugin(const std::string& old_instance,
                                     const std::string& new_plugin_path) = 0;
    
    // Instance management
    virtual Result<IPlugin*> getPlugin(const std::string& instance_name) = 0;
    virtual std::vector<std::string> listPlugins() const = 0;
    virtual std::vector<std::string> listPluginsByType(PluginType type) const = 0;
    
    // Plugin discovery
    virtual std::vector<std::string> discoverPlugins(const std::string& plugin_dir) = 0;
    virtual Result<PluginManifest> getPluginManifest(const std::string& plugin_path) = 0;
    
    // Security
    virtual ResultVoid verifyPluginSignature(const std::string& plugin_path) = 0;
    virtual void setTrustedKeys(const std::vector<std::string>& public_keys) = 0;
    
    // Resource monitoring
    virtual Result<void> getResourceUsage(const std::string& instance_name,
                                          int64_t& memory_mb, double& compute_pct) = 0;
};

class PluginManagerFactory {
public:
    using CreateFunc = IPluginManager* (*)();
    using DestroyFunc = void (*)(IPluginManager*);
    
    static Result<IPluginManager*> create();
    static void destroy(IPluginManager* mgr);
};

// ============================================================================
// UPEP: DAG Task Graph
// ============================================================================

struct TaskNode {
    std::string id;
    std::string plugin_instance;  // Plugin instance name
    std::string name;             // Human-readable name
    
    std::vector<std::string> dependencies;  // Task IDs that must complete first
    std::vector<std::string> outputs;       // Output tensor names
    
    // Execution config
    int priority = 0;
    int max_retries = 3;
    uint32_t timeout_ms = 5000;
    bool async = false;
    
    // Resource hints
    ResourceRequirements resources;
    ComputeBackendType preferred_backend = ComputeBackendType::CPU;
};

struct TaskEdge {
    std::string from_task;
    std::string to_task;
    std::string output_port;   // Output from 'from_task'
    std::string input_port;    // Input to 'to_task'
};

struct TaskGraph {
    std::string name;
    std::string version;
    
    std::map<std::string, TaskNode> nodes;
    std::vector<TaskEdge> edges;
    
    // Graph metadata
    std::map<std::string, std::string> metadata;
    
    // Validation
    bool validate(std::string& error_msg) const;
    std::vector<std::string> topologicalSort() const;
};

// ============================================================================
// UPEP: Task Execution Context
// ============================================================================

struct TaskExecutionContext {
    std::string graph_id;
    std::string execution_id;
    
    std::map<std::string, FrameBufferPtr> data_store;  // Tensor store
    std::map<std::string, TaskNode> task_states;       // Task execution state
    
    // Synchronization
    std::mutex mutex;
    std::condition_variable cv;
    std::atomic<int> completed_tasks{0};
    std::atomic<int> failed_tasks{0};
    std::atomic<bool> cancelled{false};
    
    // Timing
    uint64_t start_time_ns = 0;
    std::map<std::string, uint64_t> task_start_times;
    std::map<std::string, uint64_t> task_end_times;
};

enum class TaskStatus : uint32_t {
    PENDING = 0,
    READY = 1,
    RUNNING = 2,
    COMPLETED = 3,
    FAILED = 4,
    CANCELLED = 5,
    SKIPPED = 6,
};

struct TaskExecutionResult {
    TaskStatus status = TaskStatus::PENDING;
    std::string error_message;
    uint64_t execution_time_us = 0;
    int64_t memory_peak_mb = 0;
    double compute_utilization = 0.0;
};

// ============================================================================
// UPEP: Task Scheduler
// ============================================================================

class ITaskScheduler {
public:
    virtual ~ITaskScheduler() = default;
    
    // Graph management
    virtual Result<std::string> submitGraph(const TaskGraph& graph) = 0;
    virtual ResultVoid cancelGraph(const std::string& graph_id) = 0;
    virtual ResultVoid pauseGraph(const std::string& graph_id) = 0;
    virtual ResultVoid resumeGraph(const std::string& graph_id) = 0;
    
    // Execution control
    virtual ResultVoid start() = 0;
    virtual ResultVoid stop() = 0;
    
    // Monitoring
    virtual Result<TaskExecutionResult> getTaskResult(
        const std::string& graph_id, const std::string& task_id) = 0;
    virtual std::vector<TaskStatus> getGraphStatus(const std::string& graph_id) = 0;
    virtual Result<double> getGraphProgress(const std::string& graph_id) = 0;
    
    // Callbacks
    using TaskCallback = std::function<void(const std::string& graph_id,
                                            const std::string& task_id,
                                            TaskStatus status,
                                            const TaskExecutionResult& result)>;
    virtual ResultVoid setTaskCallback(TaskCallback cb) = 0;
    virtual ResultVoid setGraphCallback(std::function<void(const std::string&, bool)> cb) = 0;
    
    // Resource management
    virtual ResultVoid setMaxConcurrentTasks(int max_tasks) = 0;
    virtual ResultVoid setResourceLimits(const ResourceRequirements& limits) = 0;
};

class TaskSchedulerFactory {
public:
    using CreateFunc = ITaskScheduler* (*)(IPluginManager*);
    using DestroyFunc = void (*)(ITaskScheduler*);
    
    static Result<ITaskScheduler*> create(IPluginManager* plugin_mgr);
    static void destroy(ITaskScheduler* scheduler);
};

} // namespace dynalgo::hal