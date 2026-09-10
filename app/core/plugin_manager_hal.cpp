/*
 * plugin_manager_hal.cpp - Dynamic plugin manager and task scheduler implementation
 * UPEP: Runtime plugin loading, hot-swap, DAG task scheduling
 */
#include <dynalgo/hal/plugin_manager_hal.hpp>

#include <dlfcn.h>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>
#include <atomic>
#include <filesystem>
#include <queue>

namespace dynalgo::hal {

// ============================================================================
// CPU Plugin Implementation
// ============================================================================

class CPUPlugin : public IPlugin {
public:
    CPUPlugin(const std::string& lib_path, const std::string& instance_name)
        : m_lib_path(lib_path), m_instance_name(instance_name), m_state(PluginState::UNLOADED) {}
    
    ~CPUPlugin() {
        if (m_handle) dlclose(m_handle);
    }
    
    ResultVoid initialize(const std::string& config_json) override {
        if (m_state != PluginState::LOADED && m_state != PluginState::UNLOADED) {
            return ResultVoid::err(static_cast<uint32_t>(ErrorCode::INVALID_STATE));
        }
        m_state = PluginState::INITIALIZING;
        m_config = config_json;
        m_state = PluginState::READY;
        return ResultVoid::ok();
    }
    
    ResultVoid deinitialize() override {
        m_state = PluginState::UNLOADED;
        return ResultVoid::ok();
    }
    
    ResultVoid start() override {
        if (m_state != PluginState::READY) {
            return ResultVoid::err(static_cast<uint32_t>(ErrorCode::INVALID_STATE));
        }
        m_state = PluginState::RUNNING;
        return ResultVoid::ok();
    }
    
    ResultVoid stop() override {
        if (m_state != PluginState::RUNNING) {
            return ResultVoid::err(static_cast<uint32_t>(ErrorCode::INVALID_STATE));
        }
        m_state = PluginState::READY;
        return ResultVoid::ok();
    }
    
    ResultVoid execute(const std::map<std::string, FrameBufferPtr>& inputs,
                       std::map<std::string, FrameBufferPtr>& outputs,
                       void* stream) override {
        if (m_state != PluginState::RUNNING) {
            return ResultVoid::err(static_cast<uint32_t>(ErrorCode::INVALID_STATE));
        }
        // Placeholder - real implementation would call plugin's execute function
        return ResultVoid::ok();
    }
    
    ResultVoid executeAsync(const std::map<std::string, FrameBufferPtr>& inputs,
                            std::function<void(const std::map<std::string, FrameBufferPtr>&, ResultVoid)> callback,
                            void* stream) override {
        // Execute synchronously for now
        auto result = execute(inputs, const_cast<std::map<std::string, FrameBufferPtr>&>(inputs), stream);
        callback(inputs, result);
        return ResultVoid::ok();
    }
    
    PluginState getState() const override { return m_state; }
    Result<std::string> getLastError() const override { return Result<std::string>::ok(m_last_error); }
    
    ResultVoid setConfig(const std::string& config_json) override {
        m_config = config_json;
        return ResultVoid::ok();
    }
    
    Result<std::string> getConfig() const override { return Result<std::string>::ok(m_config); }
    
    Result<PluginManifest> getManifest() const override { return Result<PluginManifest>::ok(m_manifest); }
    
    Result<int64_t> getMemoryUsage() const override { return Result<int64_t>::ok(0); }
    Result<double> getComputeUtilization() const override { return Result<double>::ok(0.0); }
    
    ResultVoid vendorCommand(uint32_t, const void*, size_t, void*, size_t) override {
        return ResultVoid::err(static_cast<uint32_t>(ErrorCode::NOT_IMPLEMENTED));
    }
    
    // Internal: load from shared library
    ResultVoid loadLibrary() {
        m_handle = dlopen(m_lib_path.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (!m_handle) {
            m_last_error = dlerror();
            return ResultVoid::err(static_cast<uint32_t>(ErrorCode::NOT_FOUND));
        }
        
        // Load manifest
        using GetManifestFunc = PluginManifest* (*)();
        GetManifestFunc get_manifest = reinterpret_cast<GetManifestFunc>(dlsym(m_handle, "dynalgo_plugin_get_manifest"));
        if (get_manifest) {
            m_manifest = *get_manifest();
        }
        
        m_state = PluginState::LOADED;
        return ResultVoid::ok();
    }
    
    std::string getInstanceName() const { return m_instance_name; }

private:
    std::string m_lib_path;
    std::string m_instance_name;
    void* m_handle = nullptr;
    PluginManifest m_manifest;
    std::string m_config;
    PluginState m_state;
    std::string m_last_error;
};

// ============================================================================
// CPU Plugin Manager Implementation
// ============================================================================

class CPUPluginManager : public IPluginManager {
public:
    CPUPluginManager() = default;
    ~CPUPluginManager() {
        for (auto& pair : m_plugins) {
            unloadPlugin(pair.first);
        }
    }
    
    Result<std::string> loadPlugin(const std::string& plugin_path,
                                   const std::string& instance_name) override {
        std::string name = instance_name.empty() ? 
            std::filesystem::path(plugin_path).stem().string() : instance_name;
        
        std::lock_guard<std::mutex> lock(m_mutex);
        
        if (m_plugins.find(name) != m_plugins.end()) {
            return Result<std::string>::err(static_cast<uint32_t>(ErrorCode::ALREADY_INITIALIZED));
        }
        
        auto plugin = std::make_unique<CPUPlugin>(plugin_path, name);
        auto load_result = plugin->loadLibrary();
        if (!load_result.is_ok()) {
            return Result<std::string>::err(load_result.error());
        }
        
        m_plugins[name] = std::move(plugin);
        return Result<std::string>::ok(name);
    }
    
    ResultVoid unloadPlugin(const std::string& instance_name) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        auto it = m_plugins.find(instance_name);
        if (it == m_plugins.end()) {
            return ResultVoid::err(static_cast<uint32_t>(ErrorCode::NOT_FOUND));
        }
        
        it->second->stop();
        it->second->deinitialize();
        m_plugins.erase(it);
        return ResultVoid::ok();
    }
    
    ResultVoid hotSwapPlugin(const std::string& old_instance,
                             const std::string& new_plugin_path) override {
        // Get old plugin config
        std::string config;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            auto it = m_plugins.find(old_instance);
            if (it == m_plugins.end()) {
                return ResultVoid::err(static_cast<uint32_t>(ErrorCode::NOT_FOUND));
            }
            auto config_result = it->second->getConfig();
            if (config_result.is_ok()) config = config_result.value();
            it->second->stop();
        }
        
        // Unload old
        unloadPlugin(old_instance);
        
        // Load new with same name
        auto load_result = loadPlugin(new_plugin_path, old_instance);
        if (!load_result.is_ok()) return ResultVoid::err(load_result.error());
        
        // Apply config
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            auto it = m_plugins.find(old_instance);
            if (it != m_plugins.end() && !config.empty()) {
                it->second->setConfig(config);
            }
        }
        
        return ResultVoid::ok();
    }
    
    Result<IPlugin*> getPlugin(const std::string& instance_name) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_plugins.find(instance_name);
        if (it == m_plugins.end()) {
            return Result<IPlugin*>::err(static_cast<uint32_t>(ErrorCode::NOT_FOUND));
        }
        return Result<IPlugin*>::ok(it->second.get());
    }
    
    std::vector<std::string> listPlugins() const override {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::vector<std::string> result;
        result.reserve(m_plugins.size());
        for (const auto& pair : m_plugins) {
            result.push_back(pair.first);
        }
        return result;
    }
    
    std::vector<std::string> listPluginsByType(PluginType type) const override {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::vector<std::string> result;
        for (const auto& pair : m_plugins) {
            auto manifest_result = pair.second->getManifest();
            if (manifest_result.is_ok() && manifest_result.value().type == type) {
                result.push_back(pair.first);
            }
        }
        return result;
    }
    
    std::vector<std::string> discoverPlugins(const std::string& plugin_dir) override {
        std::vector<std::string> result;
        if (!std::filesystem::exists(plugin_dir)) return result;
        
        for (const auto& entry : std::filesystem::directory_iterator(plugin_dir)) {
            if (entry.path().extension() == ".so" || entry.path().extension() == ".dylib") {
                result.push_back(entry.path().string());
            }
        }
        return result;
    }
    
    Result<PluginManifest> getPluginManifest(const std::string& plugin_path) override {
        void* handle = dlopen(plugin_path.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (!handle) {
            return Result<PluginManifest>::err(static_cast<uint32_t>(ErrorCode::NOT_FOUND));
        }
        
        using GetManifestFunc = PluginManifest* (*)();
        GetManifestFunc get_manifest = reinterpret_cast<GetManifestFunc>(dlsym(handle, "dynalgo_plugin_get_manifest"));
        PluginManifest manifest;
        if (get_manifest) {
            manifest = *get_manifest();
        }
        dlclose(handle);
        return Result<PluginManifest>::ok(manifest);
    }
    
    ResultVoid verifyPluginSignature(const std::string& plugin_path) override {
        // TODO: Implement signature verification
        (void)plugin_path;
        return ResultVoid::err(static_cast<uint32_t>(ErrorCode::NOT_IMPLEMENTED));
    }
    
    void setTrustedKeys(const std::vector<std::string>& public_keys) override {
        m_trusted_keys = public_keys;
    }
    
    Result<void> getResourceUsage(const std::string& instance_name,
                                  int64_t& memory_mb, double& compute_pct) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_plugins.find(instance_name);
        if (it == m_plugins.end()) {
            return Result<void>::err(static_cast<uint32_t>(ErrorCode::NOT_FOUND));
        }
        
        auto mem_result = it->second->getMemoryUsage();
        auto compute_result = it->second->getComputeUtilization();
        
        if (mem_result.is_ok()) memory_mb = mem_result.value();
        if (compute_result.is_ok()) compute_pct = compute_result.value();
        
        return Result<void>::ok();
    }

private:
    mutable std::mutex m_mutex;
    std::unordered_map<std::string, std::unique_ptr<CPUPlugin>> m_plugins;
    std::vector<std::string> m_trusted_keys;
};

// ============================================================================
// CPU Task Scheduler Implementation
// ============================================================================

class CPUTaskScheduler : public ITaskScheduler {
public:
    CPUTaskScheduler(IPluginManager* plugin_mgr) : m_plugin_mgr(plugin_mgr), m_running(false) {}
    ~CPUTaskScheduler() { stop(); }
    
    Result<std::string> submitGraph(const TaskGraph& graph) override {
        std::string graph_id = generateGraphId();
        
        std::lock_guard<std::mutex> lock(m_mutex);
        
        // Validate graph
        std::string error_msg;
        if (!graph.validate(error_msg)) {
            return Result<std::string>::err(static_cast<uint32_t>(ErrorCode::INVALID_ARG));
        }
        
        // Store graph
        m_graphs[graph_id] = graph;
        
        // Create execution context
        auto ctx = std::make_shared<TaskExecutionContext>();
        ctx->graph_id = graph_id;
        ctx->execution_id = generateExecutionId();
        ctx->start_time_ns = now_ns();
        
        // Initialize task states
        for (const auto& pair : graph.nodes) {
            ctx->task_states[pair.first] = pair.second;
        }
        
        m_contexts[graph_id] = ctx;
        
        // Start execution if scheduler is running
        if (m_running) {
            scheduleReadyTasks(graph_id);
        }
        
        return Result<std::string>::ok(graph_id);
    }
    
    ResultVoid cancelGraph(const std::string& graph_id) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_contexts.find(graph_id);
        if (it != m_contexts.end()) {
            it->second->cancelled = true;
            it->second->cv.notify_all();
        }
        return ResultVoid::ok();
    }
    
    ResultVoid pauseGraph(const std::string& graph_id) override {
        // TODO: Implement pause
        (void)graph_id;
        return ResultVoid::err(static_cast<uint32_t>(ErrorCode::NOT_IMPLEMENTED));
    }
    
    ResultVoid resumeGraph(const std::string& graph_id) override {
        // TODO: Implement resume
        (void)graph_id;
        return ResultVoid::err(static_cast<uint32_t>(ErrorCode::NOT_IMPLEMENTED));
    }
    
    ResultVoid start() override {
        m_running = true;
        m_worker_thread = std::thread(&CPUTaskScheduler::workerLoop, this);
        return ResultVoid::ok();
    }
    
    ResultVoid stop() override {
        m_running = false;
        if (m_worker_thread.joinable()) {
            m_worker_thread.join();
        }
        return ResultVoid::ok();
    }
    
    Result<TaskExecutionResult> getTaskResult(const std::string& graph_id,
                                              const std::string& task_id) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_results.find(graph_id + ":" + task_id);
        if (it == m_results.end()) {
            return Result<TaskExecutionResult>::err(static_cast<uint32_t>(ErrorCode::NOT_FOUND));
        }
        return Result<TaskExecutionResult>::ok(it->second);
    }
    
    std::vector<TaskStatus> getGraphStatus(const std::string& graph_id) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_contexts.find(graph_id);
        if (it == m_contexts.end()) return {};
        
        std::vector<TaskStatus> statuses;
        for (const auto& pair : it->second->task_states) {
            // In a real implementation, we'd track actual task status
            statuses.push_back(TaskStatus::PENDING);
        }
        return statuses;
    }
    
    Result<double> getGraphProgress(const std::string& graph_id) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_contexts.find(graph_id);
        if (it == m_contexts.end()) return Result<double>::err(static_cast<uint32_t>(ErrorCode::NOT_FOUND));
        
        int total = it->second->task_states.size();
        if (total == 0) return Result<double>::ok(1.0);
        
        int completed = it->second->completed_tasks.load();
        return Result<double>::ok(static_cast<double>(completed) / total);
    }
    
    ResultVoid setTaskCallback(TaskCallback cb) override {
        m_task_callback = cb;
        return ResultVoid::ok();
    }
    
    ResultVoid setGraphCallback(std::function<void(const std::string&, bool)> cb) override {
        m_graph_callback = cb;
        return ResultVoid::ok();
    }
    
    ResultVoid setMaxConcurrentTasks(int max_tasks) override {
        m_max_concurrent_tasks = std::max(1, max_tasks);
        return ResultVoid::ok();
    }
    
    ResultVoid setResourceLimits(const ResourceRequirements& limits) override {
        m_resource_limits = limits;
        return ResultVoid::ok();
    }

private:
    std::string generateGraphId() {
        static std::atomic<uint64_t> counter{0};
        return "graph_" + std::to_string(++counter) + "_" + std::to_string(now_ns());
    }
    
    std::string generateExecutionId() {
        static std::atomic<uint64_t> counter{0};
        return "exec_" + std::to_string(++counter);
    }
    
    void workerLoop() {
        while (m_running) {
            std::vector<std::string> ready_graphs;
            
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                for (const auto& pair : m_contexts) {
                    if (!pair.second->cancelled) {
                        ready_graphs.push_back(pair.first);
                    }
                }
            }
            
            for (const auto& graph_id : ready_graphs) {
                scheduleReadyTasks(graph_id);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    void scheduleReadyTasks(const std::string& graph_id) {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        auto ctx_it = m_contexts.find(graph_id);
        if (ctx_it == m_contexts.end()) return;
        
        auto graph_it = m_graphs.find(graph_id);
        if (graph_it == m_graphs.end()) return;
        
        auto& ctx = ctx_it->second;
        auto& graph = graph_it->second;
        
        // Find ready tasks (dependencies met, not yet running)
        for (auto& pair : ctx->task_states) {
            const std::string& task_id = pair.first;
            auto& task = pair.second;
            
            // Check if already completed or running
            // In a real implementation, we'd track task status separately
            
            // Check dependencies
            bool deps_met = true;
            for (const auto& dep : task.dependencies) {
                auto dep_it = ctx->task_states.find(dep);
                if (dep_it == ctx->task_states.end()) {
                    deps_met = false;
                    break;
                }
                // In real impl, check dep status
            }
            
            if (deps_met) {
                // Execute task
                executeTask(graph_id, task_id, ctx);
            }
        }
    }
    
    void executeTask(const std::string& graph_id, const std::string& task_id,
                     std::shared_ptr<TaskExecutionContext> ctx) {
        auto graph_it = m_graphs.find(graph_id);
        if (graph_it == m_graphs.end()) return;
        
        auto& graph = graph_it->second;
        auto task_it = graph.nodes.find(task_id);
        if (task_it == graph.nodes.end()) return;
        
        const auto& task = task_it->second;
        
        // Get plugin
        auto plugin_result = m_plugin_mgr->getPlugin(task.plugin_instance);
        if (!plugin_result.is_ok()) {
            // Handle error
            return;
        }
        
        IPlugin* plugin = plugin_result.value();
        
        // Prepare inputs from data store
        std::map<std::string, FrameBufferPtr> inputs;
        const auto& task_graph = m_graphs[graph_id];
        for (const auto& edge : task_graph.edges) {
            if (edge.to_task == task_id) {
                auto it = ctx->data_store.find(edge.output_port);
                if (it != ctx->data_store.end()) {
                    inputs[edge.input_port] = it->second;
                }
            }
        }
        
        // Execute
        std::map<std::string, FrameBufferPtr> outputs;
        auto start_time = now_ns();
        auto result = plugin->execute(inputs, outputs);
        auto end_time = now_ns();
        
        // Store outputs
        for (const auto& pair : outputs) {
            ctx->data_store[pair.first] = pair.second;
        }
        
        // Record result
        TaskExecutionResult exec_result;
        exec_result.status = result.is_ok() ? TaskStatus::COMPLETED : TaskStatus::FAILED;
        exec_result.error_message = result.is_ok() ? "" : "Execution failed";
        exec_result.execution_time_us = (end_time - start_time) / 1000;
        
        std::string key = graph_id + ":" + task_id;
        m_results[key] = exec_result;
        
        ctx->completed_tasks.fetch_add(1);
        
        // Callback
        if (m_task_callback) {
            m_task_callback(graph_id, task_id, exec_result.status, exec_result);
        }
        
        // Check if graph complete
        if (ctx->completed_tasks.load() >= static_cast<int>(ctx->task_states.size())) {
            if (m_graph_callback) {
                m_graph_callback(graph_id, true);
            }
        }
    }

    IPluginManager* m_plugin_mgr;
    std::atomic<bool> m_running;
    std::thread m_worker_thread;
    std::mutex m_mutex;
    
    std::unordered_map<std::string, TaskGraph> m_graphs;
    std::unordered_map<std::string, std::shared_ptr<TaskExecutionContext>> m_contexts;
    std::unordered_map<std::string, TaskExecutionResult> m_results;
    
    TaskCallback m_task_callback;
    std::function<void(const std::string&, bool)> m_graph_callback;
    
    int m_max_concurrent_tasks = 4;
    ResourceRequirements m_resource_limits;
};

// ============================================================================
// Factory Functions
// ============================================================================

Result<IPluginManager*> PluginManagerFactory::create() {
    static std::unique_ptr<IPluginManager> g_manager(new CPUPluginManager());
    return Result<IPluginManager*>::ok(g_manager.get());
}

void PluginManagerFactory::destroy(IPluginManager* mgr) {
    (void)mgr;
    // Singleton, don't destroy
}

Result<ITaskScheduler*> TaskSchedulerFactory::create(IPluginManager* plugin_mgr) {
    static std::unique_ptr<ITaskScheduler> g_scheduler(new CPUTaskScheduler(plugin_mgr));
    return Result<ITaskScheduler*>::ok(g_scheduler.get());
}

void TaskSchedulerFactory::destroy(ITaskScheduler* scheduler) {
    (void)scheduler;
    // Singleton, don't destroy
}

// ============================================================================
// TaskGraph Validation
// ============================================================================

bool TaskGraph::validate(std::string& error_msg) const {
    // Check for cycles using topological sort
    std::unordered_map<std::string, int> in_degree;
    std::unordered_map<std::string, std::vector<std::string>> adj;
    
    for (const auto& pair : nodes) {
        in_degree[pair.first] = 0;
    }
    
    for (const auto& edge : edges) {
        if (nodes.find(edge.from_task) == nodes.end()) {
            error_msg = "Edge references unknown task: " + edge.from_task;
            return false;
        }
        if (nodes.find(edge.to_task) == nodes.end()) {
            error_msg = "Edge references unknown task: " + edge.to_task;
            return false;
        }
        adj[edge.from_task].push_back(edge.to_task);
        in_degree[edge.to_task]++;
    }
    
    // Kahn's algorithm
    std::queue<std::string> q;
    for (const auto& pair : in_degree) {
        if (pair.second == 0) q.push(pair.first);
    }
    
    int visited = 0;
    while (!q.empty()) {
        auto u = q.front(); q.pop();
        visited++;
        for (const auto& v : adj[u]) {
            if (--in_degree[v] == 0) q.push(v);
        }
    }
    
    if (visited != static_cast<int>(nodes.size())) {
        error_msg = "Graph contains cycle";
        return false;
    }
    
    return true;
}

std::vector<std::string> TaskGraph::topologicalSort() const {
    std::unordered_map<std::string, int> in_degree;
    std::unordered_map<std::string, std::vector<std::string>> adj;
    
    for (const auto& pair : nodes) {
        in_degree[pair.first] = 0;
    }
    
    for (const auto& edge : edges) {
        adj[edge.from_task].push_back(edge.to_task);
        in_degree[edge.to_task]++;
    }
    
    std::queue<std::string> q;
    for (const auto& pair : in_degree) {
        if (pair.second == 0) q.push(pair.first);
    }
    
    std::vector<std::string> result;
    while (!q.empty()) {
        auto u = q.front(); q.pop();
        result.push_back(u);
        for (const auto& v : adj[u]) {
            if (--in_degree[v] == 0) q.push(v);
        }
    }
    
    return result;
}

} // namespace dynalgo::hal