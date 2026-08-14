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

RsDevice::RsDevice(uint32_t deviceIndex, const std::string& deviceUuid)
: deviceIndex_(deviceIndex), deviceUuid_(deviceUuid) {}

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

void RsDevice::timerSyncWithHost() {}

bool RsDevice::isGlobalTimestampSupported() const {
    return true;
}

void RsDevice::enableGlobalTimestamp(bool) {}

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

int32_t RsDevice::getIntProperty(int) {
    return 0;
}

DynalgoSensorInfo RsDevice::setupPipeline(DynalgoPipeline& /*pipeline*/) {
    DynalgoSensorInfo si = getSensorInfo();
    si.depthScale = rs::AC1::DEPTH_SCALE;
    return si;
}

// === RsPipeline ===

RsPipeline::RsPipeline(std::shared_ptr<RsDevice> device) : rsDevice_(std::move(device)) {}

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

void RsPipeline::disableStream(DynalgoFrameType type) {
    if (type == DynalgoFrameType::COLOR)
        enableImage_ = false;
}

void RsPipeline::setAggregateAllTypeFrameRequire(bool) {}

void RsPipeline::setAlignMode(DynalgoAlignMode) {}

bool RsPipeline::checkHWD2CSupport(int, int, DynalgoFormat, int, int, DynalgoFormat, int) {
    return true;
}

void RsPipeline::enableFrameSync() {}

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

bool RsPipeline::startImu(DynalgoImuCallback callback) {
    imuCallback_ = callback;
    imuStarted_ = true;
    if (started_ && !imuThread_.joinable() && running_.load()) {
        imuThread_ = std::thread(&RsPipeline::processImu, this);
        DYNALGO_LOG_INFO_S("RS-AC1 IMU thread started (late-bind)");
    }
    return true;
}

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

void RsPipeline::stopImu() {
    // IMU cannot be stopped independently for RS-AC1.
    imuStarted_ = false;
}

std::shared_ptr<DynalgoDevice> RsPipeline::getDevice() const {
    return rsDevice_;
}

// --- Processing threads ---

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

RsContext::RsContext() {
    libusb_init(&usbCtx_);
}

RsContext::~RsContext() {
    if (usbCtx_)
        libusb_exit(usbCtx_);
}

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

uint32_t RsContext::getDeviceCount() {
    scanDevices();
    return static_cast<uint32_t>(deviceUuids_.size());
}

std::shared_ptr<DynalgoDevice> RsContext::getDevice(uint32_t index) {
    scanDevices();
    if (index >= deviceUuids_.size())
        return nullptr;
    return std::make_shared<RsDevice>(index, deviceUuids_[index]);
}

} // namespace dynalgo

