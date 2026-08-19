// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_rs_device.cpp — RoboSense RS-AC1 DynalgoDevice/DynalgoPipeline/DynalgoContext implementation.

#include "dynalgo_rs_device.hpp"

#include "dynalgo_log.hpp"
#include "dynalgo_thread.hpp"

#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace dynalgo {

// === RsDevice ===

// [构造函数 / Constructor]
// 中文: 构造函数，保存设备索引和 UUID
// English: Constructor, stores device index and UUID
RsDevice::RsDevice(uint32_t deviceIndex, const std::string& deviceUuid)
: deviceIndex_(deviceIndex), deviceUuid_(deviceUuid) {}

// [函数说明 / Function Description]
// 中文: 获取设备信息（名称、VID、PID、连接类型、序列号，带缓存）
// English: Get device info (name, VID, PID, connection type, serial number, with caching)
DynalgoDeviceInfo RsDevice::getDeviceInfo() const {
    if (!deviceInfoQueried_) {
        cachedDevInfo_.name = "RoboSense_AC1";
        cachedDevInfo_.vid = rs::AC1::USB_ID.vid;
        cachedDevInfo_.pid = rs::AC1::USB_ID.pid;
        cachedDevInfo_.connectionType = "USB3.0";
        if (!deviceUuid_.empty())
            cachedDevInfo_.serialNumber = deviceUuid_;
        deviceInfoQueried_ = true;
    }
    return cachedDevInfo_;
}

// [函数说明 / Function Description]
// 中文: 与主机同步时间戳（空实现）
// English: Sync timestamp with host (no-op)
void RsDevice::timerSyncWithHost() {}

// [函数说明 / Function Description]
// 中文: 检查是否支持全局时间戳（RS-AC1 支持）
// English: Check if global timestamp is supported (RS-AC1 supports it)
bool RsDevice::isGlobalTimestampSupported() const {
    return true;
}

// [函数说明 / Function Description]
// 中文: 启用/禁用全局时间戳（空实现）
// English: Enable/disable global timestamp (no-op)
void RsDevice::enableGlobalTimestamp(bool) {}

// [函数说明 / Function Description]
// 中文: 获取传感器信息（颜色、深度、IMU 等固定规格）
// English: Get sensor info (fixed specs for color, depth, IMU, etc.)
DynalgoSensorInfo RsDevice::getSensorInfo() const {
    DynalgoSensorInfo si;
    si.hasColor = true;
    si.hasDepth = true;
    si.hasIR = false;
    si.hasIRLeft = false;
    si.hasIRRight = false;
    si.hasAccel = true;
    si.hasGyro = true;

    si.colorFormat = rs::AC1::COLOR.format;
    si.colorW = rs::AC1::COLOR.resolution.width;
    si.colorH = rs::AC1::COLOR.resolution.height;
    si.colorFps = rs::AC1::COLOR.fps;

    si.depthFormat = rs::AC1::DEPTH.format;
    si.depthW = rs::AC1::DEPTH.resolution.width;
    si.depthH = rs::AC1::DEPTH.resolution.height;
    si.depthFps = rs::AC1::DEPTH.fps;

    return si;
}

// [函数说明 / Function Description]
// 中文: 获取整数属性（返回 0）
// English: Get integer property (returns 0)
int32_t RsDevice::getIntProperty(int) {
    return 0;
}

// [函数说明 / Function Description]
// 中文: 配置管道（返回固定传感器信息和深度比例）
// English: Configure pipeline (returns fixed sensor info and depth scale)
DynalgoSensorInfo RsDevice::setupPipeline(DynalgoPipeline& /*pipeline*/) {
    DynalgoSensorInfo si = getSensorInfo();
    si.depthScale = rs::AC1::DEPTH_SCALE;
    return si;
}

// === RsPipeline ===

// [构造函数 / Constructor]
// 中文: 构造函数，保存设备共享指针
// English: Constructor, stores device shared pointer
RsPipeline::RsPipeline(std::shared_ptr<RsDevice> device) : rsDevice_(std::move(device)) {}

// [函数说明 / Function Description]
// 中文: 启用流（颜色流配置格式/分辨率/帧率，IMU 配置帧率）
// English: Enable stream (color stream config format/resolution/fps, IMU config fps)
void RsPipeline::enableStream(const DynalgoStreamConfig& cfg) {
    if (cfg.frameType == DynalgoFrameType::COLOR) {
        enableImage_ = true;
        imageFormat_ = cfg.format != DynalgoFormat::UNKNOWN ? cfg.format : DynalgoFormat::NV12;
        if (cfg.width > 0)
            imageWidth_ = cfg.width;
        if (cfg.height > 0)
            imageHeight_ = cfg.height;
        if (cfg.fps > 0)
            imageFps_ = cfg.fps;
    } else if (cfg.frameType == DynalgoFrameType::ACCEL || cfg.frameType == DynalgoFrameType::GYRO) {
        if (cfg.fps > 0)
            imuFps_ = cfg.fps;
    }
}

// [函数说明 / Function Description]
// 中文: 禁用流（颜色流）
// English: Disable stream (color stream)
void RsPipeline::disableStream(DynalgoFrameType type) {
    if (type == DynalgoFrameType::COLOR)
        enableImage_ = false;
}

// [函数说明 / Function Description]
// 中文: 设置聚合模式（空实现）
// English: Set aggregate mode (no-op)
void RsPipeline::setAggregateAllTypeFrameRequire(bool) {}

// [函数说明 / Function Description]
// 中文: 设置对齐模式（空实现，RS-AC1 固定硬件对齐）
// English: Set align mode (no-op, RS-AC1 fixed HW alignment)
void RsPipeline::setAlignMode(DynalgoAlignMode) {}

// [函数说明 / Function Description]
// 中文: 检查硬件 D2C 支持（总是返回 true）
// English: Check HW D2C support (always returns true)
bool RsPipeline::checkHWD2CSupport(int, int, DynalgoFormat, int, int, DynalgoFormat, int) {
    return true;
}

// [函数说明 / Function Description]
// 中文: 启用帧同步（空实现）
// English: Enable frame sync (no-op)
void RsPipeline::enableFrameSync() {}

// [函数说明 / Function Description]
// 中文: 启动视频流（初始化驱动参数、注册回调、启动处理线程）
// English: Start video stream (init driver params, register callbacks, start processing threads)
bool RsPipeline::start(DynalgoVideoCallback callback) {
    videoCallback_ = callback;

    // Configure driver parameters.
    robosense::lidar::RSDriverParam param;
    param.input_type = robosense::lidar::InputType::USB;
    param.lidar_type = robosense::lidar::LidarType::RS_AC1;
    param.input_param.enable_image = enableImage_;
    param.input_param.image_format = nioFormatToRsFrameFormat(imageFormat_);
    param.input_param.image_width = imageWidth_;
    param.input_param.image_height = imageHeight_;
    param.input_param.image_fps = imageFps_;
    param.input_param.imu_fps = imuFps_;
    if (!rsDevice_->deviceUuid().empty())
        param.input_param.device_uuid = rsDevice_->deviceUuid();

    // Register dual callbacks — put lambdas push to class-member stuffed queues.
    driver_.regPointCloudCallback(
        [this]() -> std::shared_ptr<RsPointCloudMsg> {
            auto msg = freeCloudQueue_.pop();
            return msg ? msg : std::make_shared<RsPointCloudMsg>();
        },
        [this](std::shared_ptr<RsPointCloudMsg> msg) { stuffedCloudQueue_.push(msg); });

    if (enableImage_) {
        driver_.regImageDataCallback(
            [this]() -> std::shared_ptr<robosense::lidar::ImageData> {
                auto msg = freeImageQueue_.pop();
                return msg ? msg : std::make_shared<robosense::lidar::ImageData>();
            },
            [this](const std::shared_ptr<robosense::lidar::ImageData>& msg) { stuffedImageQueue_.push(msg); });
    }

    driver_.regImuDataCallback(
        [this]() -> std::shared_ptr<robosense::lidar::ImuData> {
            auto msg = freeImuQueue_.pop();
            return msg ? msg : std::make_shared<robosense::lidar::ImuData>();
        },
        [this](const std::shared_ptr<robosense::lidar::ImuData>& msg) { stuffedImuQueue_.push(msg); });

    driver_.regExceptionCallback(
        [](const robosense::lidar::Error& err) { DYNALGO_LOG_WARN_S("RS-AC1 driver error: " << err.toString()); });

    if (!driver_.init(param)) {
        DYNALGO_LOG_ERROR("RS-AC1 driver init failed");
        return false;
    }

    running_ = true;

    // Launch processing threads (before driver_.start() so they're
    // ready when the first stuffed messages arrive).
    cloudThread_ = std::thread(&RsPipeline::processCloud, this);
    if (enableImage_)
        imageThread_ = std::thread(&RsPipeline::processImageData, this);
    if (imuCallback_)
        imuThread_ = std::thread(&RsPipeline::processImu, this);

    if (!driver_.start()) {
        DYNALGO_LOG_ERROR("RS-AC1 driver start failed");
        running_ = false;
        if (cloudThread_.joinable())
            cloudThread_.join();
        if (imageThread_.joinable())
            imageThread_.join();
        if (imuThread_.joinable())
            imuThread_.join();
        return false;
    }

    started_ = true;
    DYNALGO_LOG_INFO_S("RS-AC1 pipeline started: " << rsDevice_->deviceUuid());
    return true;
}

// [函数说明 / Function Description]
// 中文: 启动 IMU 流（延迟绑定 IMU 线程）
// English: Start IMU stream (late-bind IMU thread)
bool RsPipeline::startImu(DynalgoImuCallback callback) {
    imuCallback_ = callback;
    imuStarted_ = true;
    if (started_ && !imuThread_.joinable() && running_.load()) {
        imuThread_ = std::thread(&RsPipeline::processImu, this);
        DYNALGO_LOG_INFO_S("RS-AC1 IMU thread started (late-bind)");
    }
    return true;
}

// [函数说明 / Function Description]
// 中文: 停止视频流（停止驱动、等待线程结束）
// English: Stop video stream (stop driver, join threads)
void RsPipeline::stop() {
    if (started_) {
        running_ = false;
        driver_.stop();
        started_ = false;
    }
    running_ = false;
    if (cloudThread_.joinable())
        cloudThread_.join();
    if (imageThread_.joinable())
        imageThread_.join();
    if (imuThread_.joinable())
        imuThread_.join();
}

// [函数说明 / Function Description]
// 中文: 停止 IMU 流（RS-AC1 无法独立停止 IMU）
// English: Stop IMU stream (RS-AC1 cannot stop IMU independently)
void RsPipeline::stopImu() {
    // IMU cannot be stopped independently for RS-AC1.
    imuStarted_ = false;
}

// [函数说明 / Function Description]
// 中文: 获取关联的设备
// English: Get associated device
std::shared_ptr<DynalgoDevice> RsPipeline::getDevice() const {
    return rsDevice_;
}

// --- Processing threads ---

// [函数说明 / Function Description]
// 中文: 处理点云队列（生成深度帧和点云帧，尝试发射 FrameSet）
// English: Process cloud queue (generate depth frame and point frame, try emit FrameSet)
void RsPipeline::processCloud() {
    setThreadName("rs_cloud");
    DYNALGO_LOG_DEBUG("RS-AC1 cloud processing thread started");
    while (running_.load()) {
        auto msg = stuffedCloudQueue_.popWait(10000); // 10ms timeout for real-time
        if (!msg)
            continue;
        auto depthFrame = std::make_shared<DynalgoFrame>(rsDepthToNioFrame(msg));
        auto pointFrame = std::make_shared<DynalgoFrame>(rsPointToNioFrame(msg));
        {
            std::lock_guard<std::mutex> lk(syncMtx_);
            depthFrame_ = depthFrame;
            pointFrame_ = pointFrame;
            depthReady_ = true;
            tryEmitFrameSet();
        }
        freeCloudQueue_.push(msg);
    }
    DYNALGO_LOG_DEBUG("RS-AC1 cloud processing thread stopped");
}

// [函数说明 / Function Description]
// 中文: 处理图像数据队列（生成颜色帧，尝试发射 FrameSet）
// English: Process image data queue (generate color frame, try emit FrameSet)
void RsPipeline::processImageData() {
    setThreadName("rs_image");
    DYNALGO_LOG_DEBUG("RS-AC1 image processing thread started");
    while (running_.load()) {
        auto msg = stuffedImageQueue_.popWait(10000); // 10ms timeout for real-time
        if (!msg)
            continue;
        auto colorFrame = std::make_shared<DynalgoFrame>(rsImageToNioFrame(msg));
        {
            std::lock_guard<std::mutex> lk(syncMtx_);
            colorFrame_ = colorFrame;
            colorReady_ = true;
            tryEmitFrameSet();
        }
        freeImageQueue_.push(msg);
    }
    DYNALGO_LOG_DEBUG("RS-AC1 image processing thread stopped");
}

// [函数说明 / Function Description]
// 中文: 处理 IMU 队列（格式化为 CSV 并推送回调）
// English: Process IMU queue (format as CSV and push via callback)
void RsPipeline::processImu() {
    setThreadName("rs_imu");
    DYNALGO_LOG_DEBUG("RS-AC1 IMU processing thread started");
    while (running_.load()) {
        auto msg = stuffedImuQueue_.popWait(10000); // 10ms timeout for real-time
        if (!msg)
            continue;
        emitImuData(msg);
        freeImuQueue_.push(msg);
    }
    DYNALGO_LOG_DEBUG("RS-AC1 IMU processing thread stopped");
}

// --- FrameSet synthesis ---

// [函数说明 / Function Description]
// 中文: 尝试发射 FrameSet（颜色+深度都就绪时，重用最新深度帧）
// English: Try emit FrameSet (when both color and depth ready, reuse latest depth frame)
void RsPipeline::tryEmitFrameSet() {
    // Must be called with syncMtx_ held.
    // Emit only when a fresh depth frame is available (D2C fusion trigger).
    // Both depth and color must be ready to guarantee aligned data.
    // Emit when a fresh colour frame arrives, using the latest depth frame if available.
    if (colorReady_ && depthReady_) {
        auto nioFs = std::make_shared<DynalgoFrameSet>();
        // Use the latest colour frame.
        nioFs->setFrame(DynalgoFrameType::COLOR, *colorFrame_);
        // Use the latest depth frame.
        nioFs->setFrame(DynalgoFrameType::DEPTH, *depthFrame_);
        if (pointFrame_)
            nioFs->setFrame(DynalgoFrameType::POINT, *pointFrame_);
        if (videoCallback_)
            videoCallback_(nioFs);
        // Reset colour readiness; keep depthReady_ true to reuse latest depth for next colour frame.
        colorReady_ = false;
        return;
    }
    // If only one of the streams is ready, wait for the other before emitting.
    // No emission occurs here.
}

// [函数说明 / Function Description]
// 中文: 格式化 IMU 数据并推送回调
// English: Format IMU data and push via callback
void RsPipeline::emitImuData(const std::shared_ptr<robosense::lidar::ImuData>& imu) {
    if (!imu || !imuCallback_)
        return;

    auto samples = rsImuToNioSamples(imu);
    if (!samples.empty())
        imuCallback_(samples);
}

// === RsContext ===

static constexpr uint16_t RS_AC1_VID = rs::AC1::USB_ID.vid;
static constexpr uint16_t RS_AC1_PID = rs::AC1::USB_ID.pid;

// [构造函数 / Constructor]
// 中文: 构造函数，初始化 libusb 上下文
// English: Constructor, initializes libusb context
RsContext::RsContext() {
    libusb_init(&usbCtx_);
}

// [析构函数 / Destructor]
// 中文: 析构函数，清理 libusb 上下文
// English: Destructor, cleans up libusb context
RsContext::~RsContext() {
    if (usbCtx_)
        libusb_exit(usbCtx_);
}

// [函数说明 / Function Description]
// 中文: 扫描 USB 总线查找 RS-AC1 设备，缓存 VID/PID 匹配项
// English: Scan USB bus for RS-AC1 devices; cache VID/PID matches
void RsContext::scanDevices() {
    if (scanned_)
        return;

    libusb_device** list = nullptr;
    ssize_t cnt = libusb_get_device_list(usbCtx_, &list);
    if (cnt <= 0) {
        scanned_ = true;
        return;
    }

    for (ssize_t i = 0; i < cnt; i++) {
        struct libusb_device_descriptor desc;
        int rc = libusb_get_device_descriptor(list[i], &desc);
        if (rc != 0)
            continue;
        if (desc.idVendor == RS_AC1_VID && desc.idProduct == RS_AC1_PID) {
            std::string uuid;
            libusb_device_handle* handle = nullptr;
            if (libusb_open(list[i], &handle) == 0 && desc.iSerialNumber > 0) {
                unsigned char buf[256] = {};
                int len = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, buf, sizeof(buf));
                if (len > 0)
                    uuid = std::string(reinterpret_cast<char*>(buf), static_cast<size_t>(len));
                libusb_close(handle);
            }
            if (uuid.empty()) {
                uint8_t busNum = libusb_get_bus_number(list[i]);
                uint8_t devNum = libusb_get_device_address(list[i]);
                std::ostringstream oss;
                oss << static_cast<int>(busNum) << "-" << static_cast<int>(devNum);
                uuid = oss.str();
            }
            deviceUuids_.push_back(uuid);
        }
    }

    libusb_free_device_list(list, 1);
    scanned_ = true;
}

// [函数说明 / Function Description]
// 中文: 获取设备数量
// English: Get device count
uint32_t RsContext::getDeviceCount() {
    scanDevices();
    return static_cast<uint32_t>(deviceUuids_.size());
}

// [函数说明 / Function Description]
// 中文: 根据索引获取设备
// English: Get device by index
std::shared_ptr<DynalgoDevice> RsContext::getDevice(uint32_t index) {
    scanDevices();
    if (index >= deviceUuids_.size())
        return nullptr;
    return std::make_shared<RsDevice>(index, deviceUuids_[index]);
}

} // namespace dynalgo

