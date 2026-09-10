/*
 * multimodal_fusion_hal.cpp - Multi-modal fusion engine implementation
 * UPEP: Vision-LiDAR fusion, temporal-spatial synchronization
 */
#include <dynalgo/hal/multimodal_fusion_hal.hpp>

#include <mutex>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace dynalgo::hal {

// ============================================================================
// CPU Sync Engine Implementation
// ============================================================================

class CPUSyncEngine : public ISyncEngine {
public:
    CPUSyncEngine() = default;
    
    ResultVoid initialize(const SyncConfig& config) override {
        m_config = config;
        m_sync_group_id = config.sync_group_id;
        return ResultVoid::ok();
    }
    
    ResultVoid deinitialize() override {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_frame_buffers.clear();
        return ResultVoid::ok();
    }
    
    ResultVoid addFrame(const FrameBufferPtr& frame, const std::string& sensor_id) override {
        if (!frame) return ResultVoid::err(static_cast<uint32_t>(ErrorCode::INVALID_ARG));
        
        std::lock_guard<std::mutex> lock(m_mutex);
        
        // Store frame directly - timestamp adjustment handled in fusion
        m_frame_buffers[sensor_id].push_back(frame);
        
        // Keep only recent frames (last 100)
        if (m_frame_buffers[sensor_id].size() > 100) {
            m_frame_buffers[sensor_id].erase(m_frame_buffers[sensor_id].begin());
        }
        
        return ResultVoid::ok();
    }
    
    Result<SynchronizedFrameSetPtr> getSynchronizedSet(
        uint64_t target_timestamp_ns, uint32_t timeout_ms) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        SynchronizedFrameSetPtr result = std::make_shared<SynchronizedFrameSet>();
        result->sync_timestamp_ns = target_timestamp_ns;
        result->sync_group_id = m_sync_group_id;
        
        bool found_any = false;
        
        for (auto& pair : m_frame_buffers) {
            const std::string& sensor_id = pair.first;
            auto& frames = pair.second;
            
            if (frames.empty()) continue;
            
            // Find closest frame to target timestamp
            FrameBufferPtr best_frame = nullptr;
            uint64_t best_diff = UINT64_MAX;
            
            for (auto& frame : frames) {
                uint64_t frame_ts = frame->metadata.timestamp_ns;
                uint64_t diff = (frame_ts > target_timestamp_ns) ? 
                    frame_ts - target_timestamp_ns : target_timestamp_ns - frame_ts;
                
                if (diff < best_diff && diff <= m_config.max_time_diff_ns) {
                    best_diff = diff;
                    best_frame = frame;
                }
            }
            
            if (best_frame) {
                found_any = true;
                
                // Assign to appropriate slot based on sensor type
                if (sensor_id.find("camera") != std::string::npos || sensor_id.find("cam") != std::string::npos) {
                    if (!result->camera_frame) result->camera_frame = best_frame;
                } else if (sensor_id.find("depth") != std::string::npos) {
                    if (!result->depth_frame) result->depth_frame = best_frame;
                } else if (sensor_id.find("lidar") != std::string::npos) {
                    if (!result->lidar_frame) result->lidar_frame = best_frame;
                } else if (sensor_id.find("imu") != std::string::npos) {
                    if (!result->imu_frame) result->imu_frame = best_frame;
                } else {
                    // Default to camera if unknown
                    if (!result->camera_frame) result->camera_frame = best_frame;
                }
            }
        }
        
        if (!found_any) {
            return Result<SynchronizedFrameSetPtr>::err(static_cast<uint32_t>(ErrorCode::NO_DATA));
        }
        
        return Result<SynchronizedFrameSetPtr>::ok(std::move(result));
    }
    
    ResultVoid registerCallback(SyncCallback cb) override {
        m_callback = cb;
        return ResultVoid::ok();
    }
    
    ResultVoid unregisterCallback() override {
        m_callback = nullptr;
        return ResultVoid::ok();
    }
    
    Result<int64_t> getTimeOffset(const std::string& sensor_id) const override {
        auto it = m_time_offsets.find(sensor_id);
        if (it != m_time_offsets.end()) {
            return Result<int64_t>::ok(it->second);
        }
        return Result<int64_t>::ok(0);
    }
    
    ResultVoid setTimeOffset(const std::string& sensor_id, int64_t offset_ns) override {
        m_time_offsets[sensor_id] = offset_ns;
        return ResultVoid::ok();
    }
    
    Result<double> getSyncAccuracy() const override {
        // Simplified - return configured max time diff as accuracy estimate
        return Result<double>::ok(static_cast<double>(m_config.max_time_diff_ns));
    }

private:
    SyncConfig m_config;
    uint64_t m_sync_group_id = 0;
    std::map<std::string, std::vector<FrameBufferPtr>> m_frame_buffers;
    std::map<std::string, int64_t> m_time_offsets;
    SyncCallback m_callback;
    mutable std::mutex m_mutex;
};

// ============================================================================
// CPU Fusion Engine Implementation (Late Fusion)
// ============================================================================

class CPUFusionEngine : public IFusionEngine {
public:
    CPUFusionEngine() = default;
    
    ResultVoid initialize(const FusionConfig& config,
                          const std::map<std::string, CalibrationData>& calibrations) override {
        m_config = config;
        m_calibrations = calibrations;
        return ResultVoid::ok();
    }
    
    ResultVoid deinitialize() override {
        return ResultVoid::ok();
    }
    
    Result<SynchronizedFrameSetPtr> fuse(const SynchronizedFrameSetPtr& input) override {
        // Late fusion: synchronize and return
        // In a real implementation, this would:
        // 1. Project LiDAR points to camera image plane
        // 2. Associate detections
        // 3. Fuse at object level
        
        if (!input) {
            return Result<SynchronizedFrameSetPtr>::err(static_cast<uint32_t>(ErrorCode::INVALID_ARG));
        }
        
        // For now, just pass through with calibrations attached
        auto result = std::make_shared<SynchronizedFrameSet>(*input);
        result->calibrations = m_calibrations;
        return Result<SynchronizedFrameSetPtr>::ok(std::move(result));
    }
    
    Result<std::vector<FusedDetection>> getDetections(const SynchronizedFrameSetPtr& fused) override {
        // Placeholder - real implementation would:
        // 1. Run camera detection (YOLO, etc.)
        // 2. Run LiDAR detection (PointPillars, etc.)
        // 3. Fuse using late fusion strategy
        return Result<std::vector<FusedDetection>>::ok({});
    }
    
    Result<FusedPointCloud> getFusedPointCloud(const SynchronizedFrameSetPtr& fused) override {
        FusedPointCloud result;
        
        if (fused->hasLidar() && fused->hasDepth()) {
            // Early fusion: combine LiDAR sparse points with depth map dense points
            // Use the first available frame's dimensions as reference
            if (fused->lidar_frame) {
                result.width = fused->lidar_frame->metadata.width;
                result.height = fused->lidar_frame->metadata.height;
            } else if (fused->depth_frame) {
                result.width = fused->depth_frame->metadata.width;
                result.height = fused->depth_frame->metadata.height;
            }
            
            // Get camera intrinsic from calibration
            for (const auto& cal : m_calibrations) {
                if (cal.second.frame == CoordinateFrame::CAMERA) {
                    result.camera_intrinsic = cal.second.intrinsic;
                    result.lidar_to_camera = cal.second.sensor_to_camera;
                    break;
                }
            }
        }
        
        return Result<FusedPointCloud>::ok(std::move(result));
    }
    
    ResultVoid setConfig(const FusionConfig& config) override {
        m_config = config;
        return ResultVoid::ok();
    }
    
    Result<FusionConfig> getConfig() const override {
        return Result<FusionConfig>::ok(m_config);
    }
    
    ResultVoid updateCalibration(const std::string& sensor_id, const CalibrationData& calib) override {
        m_calibrations[sensor_id] = calib;
        return ResultVoid::ok();
    }
    
    ResultVoid vendorCommand(uint32_t cmd_id, const void* in_data, size_t in_size,
                             void* out_data, size_t out_size) override {
        return ResultVoid::err(static_cast<uint32_t>(ErrorCode::NOT_IMPLEMENTED));
    }

private:
    FusionConfig m_config;
    std::map<std::string, CalibrationData> m_calibrations;
};

// ============================================================================
// Factory Functions
// ============================================================================

Result<ISyncEngine*> SyncEngineFactory::create(const std::string& platform, const std::string& vendor) {
    (void)platform; (void)vendor;
    // For now, always return CPU implementation
    static std::unique_ptr<ISyncEngine> g_sync_engine(new CPUSyncEngine());
    return Result<ISyncEngine*>::ok(g_sync_engine.get());
}

void SyncEngineFactory::destroy(ISyncEngine* engine) {
    // CPU engine is singleton
    (void)engine;
}

std::vector<std::string> SyncEngineFactory::getSupportedVendors(const std::string& platform) {
    (void)platform;
    return {"cpu"};
}

void SyncEngineFactory::registerVendor(const std::string&, const std::string&,
                                       CreateFunc, DestroyFunc) {
    // TODO: Implement dynamic registration
}

Result<IFusionEngine*> FusionEngineFactory::create(const std::string& platform, const std::string& vendor) {
    (void)platform; (void)vendor;
    static std::unique_ptr<IFusionEngine> g_fusion_engine(new CPUFusionEngine());
    return Result<IFusionEngine*>::ok(g_fusion_engine.get());
}

Result<IFusionEngine*> FusionEngineFactory::create(const FusionConfig& config) {
    static std::unique_ptr<IFusionEngine> g_fusion_engine(new CPUFusionEngine());
    return Result<IFusionEngine*>::ok(g_fusion_engine.get());
}

void FusionEngineFactory::destroy(IFusionEngine* engine) {
    (void)engine;
}

std::vector<std::string> FusionEngineFactory::getSupportedVendors(const std::string& platform) {
    (void)platform;
    return {"cpu"};
}

void FusionEngineFactory::registerVendor(const std::string&, const std::string&,
                                         CreateFunc, DestroyFunc) {
    // TODO: Implement dynamic registration
}

} // namespace dynalgo::hal