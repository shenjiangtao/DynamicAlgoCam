/*
 * multimodal_fusion_hal.hpp - Multi-modal fusion engine HAL
 * UPEP: Vision-LiDAR fusion, temporal-spatial synchronization, coordinate transformation
 */
#pragma once

#include "hal_types.hpp"
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <map>

namespace dynalgo::hal {

// ============================================================================
// UPEP: Fusion Results
// ============================================================================

struct FusedDetection {
    // 2D detection (from camera)
    float bbox_x = 0, bbox_y = 0, bbox_w = 0, bbox_h = 0;
    float confidence = 0.0f;
    int class_id = -1;
    std::string class_name;
    
    // 3D position (fused)
    float x = 0, y = 0, z = 0;        // In vehicle frame (meters)
    float length = 0, width = 0, height = 0;
    float yaw = 0;                     // Heading in vehicle frame
    
    // Fusion metadata
    float camera_confidence = 0.0f;
    float lidar_confidence = 0.0f;
    int camera_det_id = -1;
    int lidar_det_id = -1;
    
    // Tracking
    uint64_t track_id = 0;
    float track_score = 0.0f;
};

struct FusedPointCloud {
    FrameBufferPtr point_cloud;       // Fused point cloud (camera depth + LiDAR)
    std::vector<float> depth_map;     // Dense depth map (width * height)
    uint32_t width = 0, height = 0;
    CameraIntrinsic camera_intrinsic;
    Transform3D lidar_to_camera;      // Transform used for fusion
};

enum class FusionStrategy : uint32_t {
    EARLY = 0,        // Point cloud projected to image plane
    LATE = 1,         // Separate detections fused at object level
    DEEP = 2,         // Feature-level fusion (requires NN)
    HYBRID = 3,       // Combination of above
};

struct FusionConfig {
    FusionStrategy strategy = FusionStrategy::LATE;
    
    // Projection parameters
    float max_projection_distance = 100.0f;  // Max distance for LiDAR->camera projection
    float min_projection_distance = 0.5f;
    float projection_padding = 5.0f;         // Pixels around projected point
    
    // Association parameters
    float iou_threshold_2d = 0.5f;
    float distance_threshold_3d = 1.0f;      // Meters
    float yaw_threshold = 0.5f;              // Radians
    
    // Filtering
    float min_detection_confidence = 0.3f;
    float nms_iou_threshold = 0.5f;
    
    // Coordinate frames
    CoordinateFrame output_frame = CoordinateFrame::VEHICLE;
};

// ============================================================================
// UPEP: Fusion Engine Interface
// ============================================================================

class IFusionEngine {
public:
    virtual ~IFusionEngine() = default;
    
    virtual ResultVoid initialize(const FusionConfig& config,
                                  const std::map<std::string, CalibrationData>& calibrations) = 0;
    virtual ResultVoid deinitialize() = 0;
    
    // UPEP: Main fusion entry point
    virtual Result<SynchronizedFrameSetPtr> fuse(
        const SynchronizedFrameSetPtr& input) = 0;
    
    // Get fused detections
    virtual Result<std::vector<FusedDetection>> getDetections(
        const SynchronizedFrameSetPtr& fused) = 0;
    
    // Get fused point cloud
    virtual Result<FusedPointCloud> getFusedPointCloud(
        const SynchronizedFrameSetPtr& fused) = 0;
    
    // Config
    virtual ResultVoid setConfig(const FusionConfig& config) = 0;
    virtual Result<FusionConfig> getConfig() const = 0;
    
    // Calibration updates
    virtual ResultVoid updateCalibration(const std::string& sensor_id, 
                                         const CalibrationData& calib) = 0;
    
    // Vendor extension
    virtual ResultVoid vendorCommand(uint32_t cmd_id,
                                     const void* in_data, size_t in_size,
                                     void* out_data, size_t out_size) = 0;
};

class FusionEngineFactory {
public:
    using CreateFunc = IFusionEngine* (*)();
    using DestroyFunc = void (*)(IFusionEngine*);
    
    static Result<IFusionEngine*> create(const std::string& platform, 
                                         const std::string& vendor);
    static Result<IFusionEngine*> create(const FusionConfig& config);
    static void destroy(IFusionEngine* engine);
    
    static std::vector<std::string> getSupportedVendors(const std::string& platform);
    static void registerVendor(const std::string& platform, const std::string& vendor,
                               CreateFunc create, DestroyFunc destroy);
};

// ============================================================================
// UPEP: Synchronization Engine (Time-space alignment)
// ============================================================================

class ISyncEngine {
public:
    virtual ~ISyncEngine() = default;
    
    virtual ResultVoid initialize(const SyncConfig& config) = 0;
    virtual ResultVoid deinitialize() = 0;
    
    // Add frame to sync buffer
    virtual ResultVoid addFrame(const FrameBufferPtr& frame, 
                                const std::string& sensor_id) = 0;
    
    // Get synchronized frame set (blocking with timeout)
    virtual Result<SynchronizedFrameSetPtr> getSynchronizedSet(
        uint64_t target_timestamp_ns,
        uint32_t timeout_ms = 100) = 0;
    
    // Async callback mode
    using SyncCallback = std::function<void(const SynchronizedFrameSetPtr&)>;
    virtual ResultVoid registerCallback(SyncCallback cb) = 0;
    virtual ResultVoid unregisterCallback() = 0;
    
    // Time sync
    virtual Result<int64_t> getTimeOffset(const std::string& sensor_id) const = 0;
    virtual ResultVoid setTimeOffset(const std::string& sensor_id, int64_t offset_ns) = 0;
    
    // Statistics
    virtual Result<double> getSyncAccuracy() const = 0;  // RMS time error in ns
};

class SyncEngineFactory {
public:
    using CreateFunc = ISyncEngine* (*)();
    using DestroyFunc = void (*)(ISyncEngine*);
    
    static Result<ISyncEngine*> create(const std::string& platform,
                                       const std::string& vendor);
    static void destroy(ISyncEngine* engine);
    
    static std::vector<std::string> getSupportedVendors(const std::string& platform);
    static void registerVendor(const std::string& platform, const std::string& vendor,
                               CreateFunc create, DestroyFunc destroy);
};

} // namespace dynalgo::hal