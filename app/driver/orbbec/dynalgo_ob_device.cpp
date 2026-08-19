// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_ob_device.cpp — Orbbec SDK DynalgoDevice/DynalgoPipeline/DynalgoContext impl.

#include "dynalgo_ob_device.hpp"
#include "dynalgo_common.hpp"
#include "dynalgo_log.hpp"

using namespace dynalgo::orbbec;
namespace dynalgo {


// [构造函数 / Constructor]
// 中文: 构造函数，保存 ob::Device 共享指针
// English: Constructor, stores ob::Device shared pointer
ObDevice::ObDevice(std::shared_ptr<ob::Device> device) : obDevice_(std::move(device)) {}

// [函数说明 / Function Description]
// 中文: 获取设备信息（名称、序列号、VID、PID、连接类型）
// English: Get device info (name, serial number, VID, PID, connection type)
DynalgoDeviceInfo ObDevice::getDeviceInfo() const {
    auto di = obDevice_->getDeviceInfo();
    DynalgoDeviceInfo info;
    info.name = di->getName();
    info.serialNumber = di->getSerialNumber();
    info.vid = di->getVid();
    info.pid = di->getPid();
    info.connectionType = di->getConnectionType();
    return info;
}

// [函数说明 / Function Description]
// 中文: 与主机同步时间戳
// English: Sync timestamp with host
void ObDevice::timerSyncWithHost() {
    obDevice_->timerSyncWithHost();
}

// [函数说明 / Function Description]
// 中文: 检查是否支持全局时间戳
// English: Check if global timestamp is supported
bool ObDevice::isGlobalTimestampSupported() const {
    return obDevice_->isGlobalTimestampSupported();
}

// [函数说明 / Function Description]
// 中文: 启用/禁用全局时间戳
// English: Enable/disable global timestamp
void ObDevice::enableGlobalTimestamp(bool enable) {
    obDevice_->enableGlobalTimestamp(enable);
}

// [函数说明 / Function Description]
// 中文: 获取传感器信息（带缓存）
// English: Get sensor info (with caching)
DynalgoSensorInfo ObDevice::getSensorInfo() const {
    if (sensorInfoCached_)
        return cachedSensorInfo_;

    auto sensorList = obDevice_->getSensorList();
    DynalgoSensorInfo si;

    for (uint32_t s = 0; s < sensorList->getCount(); s++) {
        auto sensorType = sensorList->getSensorType(s);
        switch (sensorType) {
        case OB_SENSOR_COLOR:
            si.hasColor = true;
            break;
        case OB_SENSOR_DEPTH:
            si.hasDepth = true;
            break;
        case OB_SENSOR_IR:
            si.hasIR = true;
            break;
        case OB_SENSOR_IR_LEFT:
            si.hasIRLeft = true;
            break;
        case OB_SENSOR_IR_RIGHT:
            si.hasIRRight = true;
            break;
        case OB_SENSOR_ACCEL:
            si.hasAccel = true;
            break;
        case OB_SENSOR_GYRO:
            si.hasGyro = true;
            break;
        default:
            break;
        }
    }

    cachedSensorInfo_ = si;
    sensorInfoCached_ = true;
    return si;
}

// [函数说明 / Function Description]
// 中文: 获取整数属性
// English: Get integer property
int32_t ObDevice::getIntProperty(int propertyId) {
    return obDevice_->getIntProperty(static_cast<OBPropertyID>(propertyId));
}

// [函数说明 / Function Description]
// 中文: 检查是否有 IR 传感器
// English: Check if IR sensor exists
bool ObDevice::hasIRSensor() const {
    auto si = getSensorInfo();
    return si.hasIR || si.hasIRLeft || si.hasIRRight;
}

// [函数说明 / Function Description]
// 中文: 配置管道流（颜色、深度、IR 等），返回实际启用的传感器信息
// English: Configure pipeline streams (color, depth, IR, etc.), return enabled sensor info
DynalgoSensorInfo ObDevice::setupPipeline(DynalgoPipeline& pipeline) {
    auto* obPipe = dynamic_cast<ObPipeline*>(&pipeline);
    if (!obPipe) {
        DYNALGO_LOG_ERROR("setupPipeline: pipeline is not ObPipeline");
        return DynalgoSensorInfo{};
    }

    auto devInfo = obDevice_->getDeviceInfo();
    auto pid = devInfo->getPid();
    auto vid = devInfo->getVid();
    bool is305 = dynalgo::isGemini305Device(vid, pid);
    bool is335L336L = dynalgo::isGemini335L336LDevice(vid, pid);
    DynalgoFormat colorPreferredFmt = getPreferredColorFormat(is305, is335L336L);

    DynalgoSensorInfo si;
    auto sensorList = obDevice_->getSensorList();
    auto obCfg = obPipe->obConfig();

    for (uint32_t s = 0; s < sensorList->getCount(); s++) {
        auto sensorType = sensorList->getSensorType(s);
        auto sensor = sensorList->getSensor(s);
        auto profileList = sensor->getStreamProfileList();

        switch (sensorType) {
        case OB_SENSOR_COLOR:
            {
                si.hasColor = true;
                OBFormat obPreferred = nioFormatToOb(colorPreferredFmt);
                auto profile = selectBestProfile(profileList, obPreferred);
                DynalgoFormat resolvedFmt = DynalgoFormat::UNKNOWN;
                if (profile) {
                    resolvedFmt = obFormatToNio(profile->getFormat());
                    if (resolvedFmt == DynalgoFormat::UNKNOWN) {
                        for (uint32_t k = 0; k < profileList->getCount(); k++) {
                            try {
                                auto p = profileList->getProfile(k)->as<ob::VideoStreamProfile>();
                                if (p && p->getFormat() != OB_FORMAT_UNKNOWN) {
                                    profile = p;
                                    resolvedFmt = obFormatToNio(p->getFormat());
                                    break;
                                }
                            } catch (...) {
                                DYNALGO_LOG_WARN_S("Color profile fallback getProfile(" << k << ") threw");
                            }
                        }
                    }
                }
                if (profile && resolvedFmt != DynalgoFormat::UNKNOWN) {
                    obCfg->enableStream(profile);
                    obPipe->setColorProfile(profile);
                    si.colorFormat = resolvedFmt;
                    si.colorW = profile->getWidth();
                    si.colorH = profile->getHeight();
                    si.colorFps = profile->getFps();
                    try {
                        si.colorIntrinsic = obIntrinsicToNio(profile->as<ob::VideoStreamProfile>()->getIntrinsic());
                    } catch (...) {
                        DYNALGO_LOG_WARN_S("Color intrinsic not available");
                    }
                } else {
                    si.hasColor = false;
                }
                break;
            }
        case OB_SENSOR_DEPTH:
            {
                si.hasDepth = true;
                std::shared_ptr<ob::StreamProfileList> hwD2CProfiles;
                auto colorProfile = obPipe->colorProfile();
                if (colorProfile) {
                    try {
                        hwD2CProfiles = obPipe->obPipeline()->getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
                    } catch (...) {
                        DYNALGO_LOG_WARN_S("getD2CDepthProfileList threw — depth selection without HW-D2C preference");
                    }
                }
                auto profile = selectBestProfile(profileList, OB_FORMAT_Y16, hwD2CProfiles);
                DynalgoFormat resolvedFmt = DynalgoFormat::UNKNOWN;
                if (profile) {
                    resolvedFmt = obFormatToNio(profile->getFormat());
                    if (resolvedFmt == DynalgoFormat::UNKNOWN) {
                        for (uint32_t k = 0; k < profileList->getCount(); k++) {
                            try {
                                auto p = profileList->getProfile(k)->as<ob::VideoStreamProfile>();
                                if (p && p->getFormat() != OB_FORMAT_UNKNOWN) {
                                    profile = p;
                                    resolvedFmt = obFormatToNio(p->getFormat());
                                    break;
                                }
                            } catch (...) {
                                DYNALGO_LOG_WARN_S("Depth profile fallback getProfile(" << k << ") threw");
                            }
                        }
                    }
                }
                if (profile && resolvedFmt != DynalgoFormat::UNKNOWN) {
                    obCfg->enableStream(profile);
                    obPipe->setDepthProfile(profile);
                    si.depthFormat = resolvedFmt;
                    si.depthW = profile->getWidth();
                    si.depthH = profile->getHeight();
                    si.depthFps = profile->getFps();
                    try {
                        si.depthIntrinsic = obIntrinsicToNio(profile->as<ob::VideoStreamProfile>()->getIntrinsic());
                    } catch (...) {
                        DYNALGO_LOG_WARN_S("Depth intrinsic not available");
                    }
                } else {
                    si.hasDepth = false;
                }
                // Depth scale
                try {
                    int32_t level = obDevice_->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
                    si.depthScale = depthLevelToScale(level);
                    DYNALGO_LOG_INFO_S("Depth scale: " << si.depthScale << " (precision level " << level << ")");
                } catch (...) {
                    DYNALGO_LOG_WARN_S("Depth precision level query threw — using default scale "
                                   << DEFAULT_DEPTH_SCALE);
                    si.depthScale = DEFAULT_DEPTH_SCALE;
                }
                break;
            }
        case OB_SENSOR_IR:
            {
                si.hasIR = true;
                auto profile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if (profile) {
                    si.irFormat = obFormatToNio(profile->getFormat());
                    if (si.irFormat == DynalgoFormat::UNKNOWN)
                        si.irFormat = DynalgoFormat::Y8;
                    obCfg->enableStream(profile);
                    si.irW = profile->getWidth();
                    si.irH = profile->getHeight();
                    si.irFps = profile->getFps();
                } else {
                    si.hasIR = false;
                }
                break;
            }
        case OB_SENSOR_IR_LEFT:
            {
                si.hasIRLeft = true;
                auto profile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if (profile) {
                    si.irLeftFormat = obFormatToNio(profile->getFormat());
                    if (si.irLeftFormat == DynalgoFormat::UNKNOWN)
                        si.irLeftFormat = DynalgoFormat::Y8;
                    obCfg->enableStream(profile);
                    si.irLW = profile->getWidth();
                    si.irLH = profile->getHeight();
                    si.irLFps = profile->getFps();
                } else {
                    si.hasIRLeft = false;
                }
                break;
            }
        case OB_SENSOR_IR_RIGHT:
            {
                si.hasIRRight = true;
                auto profile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if (profile) {
                    si.irRightFormat = obFormatToNio(profile->getFormat());
                    if (si.irRightFormat == DynalgoFormat::UNKNOWN)
                        si.irRightFormat = DynalgoFormat::Y8;
                    obCfg->enableStream(profile);
                    si.irRW = profile->getWidth();
                    si.irRH = profile->getHeight();
                    si.irRFps = profile->getFps();
                } else {
                    si.hasIRRight = false;
                }
                break;
            }
        case OB_SENSOR_ACCEL:
            si.hasAccel = true;
            break;
        case OB_SENSOR_GYRO:
            si.hasGyro = true;
            break;
        default:
            break;
        }
    }

    // Device quirks
    if (dynalgo::isGemini305gDevice(devInfo->getVid(), devInfo->getPid(), devInfo->getConnectionType())) {
        obPipe->disableStream(DynalgoFrameType::IR_LEFT);
        si.hasIRLeft = false;
        DYNALGO_LOG_INFO_S("Gemini 305g: disabled IR_LEFT");
    }

    // HW D2C check
    if (si.hasColor && si.hasDepth) {
        bool hwD2CSupported = obPipe->checkHWD2CSupport(si.colorW, si.colorH, si.colorFormat, si.depthW, si.depthH,
                                                        si.depthFormat, si.depthFps);
        pipeline.setAlignMode(hwD2CSupported ? DynalgoAlignMode::HW : DynalgoAlignMode::SW);
        DYNALGO_LOG_INFO_S("HW D2C: " << (hwD2CSupported ? "supported" : "not supported"));
    }

    cachedSensorInfo_ = si;
    sensorInfoCached_ = true;
return si;
}

// === ObPipeline ===

// [构造函数 / Constructor]
// 中文: 构造函数，创建 ob::Pipeline 和 ob::Config，设置聚合模式为 ANY_SITUATION
// English: Constructor, creates ob::Pipeline and ob::Config, sets aggregate mode to ANY_SITUATION
ObPipeline::ObPipeline(std::shared_ptr<ob::Device> device) : obDevice_(device) {
    obPipeline_ = std::make_shared<ob::Pipeline>(device);
    obConfig_ = std::make_shared<ob::Config>();
    obConfig_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
}

// [函数说明 / Function Description]
// 中文: 启用流（需要 ob::VideoStreamProfile，由 CaptureSession 实现）
// English: Enable stream (requires ob::VideoStreamProfile, implemented in CaptureSession)
void ObPipeline::enableStream(const DynalgoStreamConfig& /*cfg*/) {
    // 流启用需要 ob::VideoStreamProfile，由 CaptureSession 中具体实现。
    // Phase 4 重构时将完整实现。
}

// [函数说明 / Function Description]
// 中文: 禁用流
// English: Disable stream
void ObPipeline::disableStream(DynalgoFrameType type) {
    obConfig_->disableStream(nioFrameTypeToObSensor(type));
}

// [函数说明 / Function Description]
// 中文: 设置聚合模式（任何情况都输出 FrameSet，保持融合 FPS 接近颜色传感器速率）
// English: Set aggregate mode (emit FrameSet as soon as any stream arrives, keeps fused FPS near color rate)
void ObPipeline::setAggregateAllTypeFrameRequire(bool /*require*/) {
    // Emit a FrameSet as soon as any stream arrives (colour, depth, IR, etc.).
    // This mode keeps the fused FPS close to the colour sensor rate (≈30 fps).
    obConfig_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
}

// [函数说明 / Function Description]
// 中文: 设置对齐模式（硬件 D2C 或软件对齐）
// English: Set align mode (HW D2C or SW alignment)
void ObPipeline::setAlignMode(DynalgoAlignMode mode) {
    if (mode == DynalgoAlignMode::HW) {
        obConfig_->setAlignMode(ALIGN_D2C_HW_MODE);
        hwD2CMode_ = true;
    } else if (mode == DynalgoAlignMode::SW) {
        alignFilter_ = std::make_shared<ob::Align>(OB_STREAM_COLOR);
        hwD2CMode_ = false;
    } else {
        hwD2CMode_ = false;
    }
}

void ObPipeline::setPointCloudEnabled(bool enable) {
    pcdEnabled_ = enable;
    if (enable) {
        pointCloudFilter_ = std::make_shared<ob::PointCloudFilter>();
        pointCloudFilter_->setCreatePointFormat(OB_FORMAT_POINT);
    }
}

// [函数说明 / Function Description]
// 中文: 检查硬件 D2C 支持
// English: Check HW D2C support
bool ObPipeline::checkHWD2CSupport(int /*colorW*/, int /*colorH*/, DynalgoFormat /*colorFmt*/, int /*depthW*/,
                                   int /*depthH*/, DynalgoFormat /*depthFmt*/, int /*depthFps*/) {
    if (!colorProfile_ || !depthProfile_)
        return false;
    auto hwD2CDepthProfiles = obPipeline_->getD2CDepthProfileList(colorProfile_, ALIGN_D2C_HW_MODE);
    if (!hwD2CDepthProfiles || hwD2CDepthProfiles->getCount() == 0)
        return false;

    auto depthVsp = depthProfile_;
    auto count = hwD2CDepthProfiles->getCount();
    for (uint32_t i = 0; i < count; i++) {
        auto sp = hwD2CDepthProfiles->getProfile(i);
        auto vsp = sp->as<ob::VideoStreamProfile>();
        if (vsp->getWidth() == depthVsp->getWidth() && vsp->getHeight() == depthVsp->getHeight() &&
            vsp->getFormat() == depthVsp->getFormat() && vsp->getFps() == depthVsp->getFps())
            return true;
    }
    return false;
}

// [函数说明 / Function Description]
// 中文: 启用帧同步
// English: Enable frame sync
void ObPipeline::enableFrameSync() {
    try {
        obPipeline_->enableFrameSync();
    } catch (...) {
        DYNALGO_LOG_WARN_S("enableFrameSync threw — running unsynchronized");
    }
}

// [函数说明 / Function Description]
// 中文: 启动视频流（回调中处理点云、对齐、转换为 DynalgoFrameSet）
// English: Start video stream (callback processes point cloud, alignment, converts to DynalgoFrameSet)
bool ObPipeline::start(DynalgoVideoCallback callback) {
    videoCallback_ = std::move(callback);
    try {
        obPipeline_->start(obConfig_, [this](std::shared_ptr<ob::FrameSet> obFs) {
            if (obFs) {
                if (pointCloudFilter_) {
                    try {
                        auto pointFrame = pointCloudFilter_->process(obFs);
                        if (pointFrame) {
                            obFs->pushFrame(pointFrame);
                        }
                    } catch (...) {
                        DYNALGO_LOG_WARN_S("pointCloudFilter_->process threw");
                    }
                }
                if (alignFilter_) {
                    try {
                        auto aligned = alignFilter_->process(obFs);
                        auto alignedFS = aligned ? std::dynamic_pointer_cast<ob::FrameSet>(aligned) : nullptr;
                        if (alignedFS)
                            obFs = alignedFS;
                    } catch (...) {
                        DYNALGO_LOG_WARN_S("alignFilter_->process threw — using unaligned frames");
                    }
                }
                auto nioFs = std::make_shared<DynalgoFrameSet>(obFrameSetToNio(obFs));
                videoCallback_(nioFs);
            }
        });
        return true;
    } catch (ob::Error& e) {
        DYNALGO_LOG_ERROR_S("Pipeline start failed: " << e.what());
        videoCallback_ = nullptr;
        return false;
    }
}

// [函数说明 / Function Description]
// 中文: 启动 IMU 流
// English: Start IMU stream
bool ObPipeline::startImu(DynalgoImuCallback callback) {
    if (!imuPipeline_) {
        imuPipeline_ = std::make_shared<ob::Pipeline>(obDevice_);
        imuConfig_ = std::make_shared<ob::Config>();
        imuConfig_->enableAccelStream();
        imuConfig_->enableGyroStream();
        imuConfig_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    }

    try {
        imuCallback_ = callback;
        imuPipeline_->start(imuConfig_, [this](std::shared_ptr<ob::FrameSet> obFs) {
            if (obFs) {
                auto samples = obImuToNioSamples(obFs);
                if (!samples.empty())
                    imuCallback_(samples);
            }
        });
        imuStarted_ = true;
        return true;
    } catch (ob::Error& e) {
        DYNALGO_LOG_ERROR_S("IMU pipeline start failed: " << e.what());
        return false;
    }
}

// [函数说明 / Function Description]
// 中文: 停止视频流
// English: Stop video stream
void ObPipeline::stop() {
    if (obPipeline_) {
        try {
            obPipeline_->stop();
        } catch (...) {
            DYNALGO_LOG_WARN_S("obPipeline_->stop() threw");
        }
    }
}

// [函数说明 / Function Description]
// 中文: 停止 IMU 流
// English: Stop IMU stream
void ObPipeline::stopImu() {
    if (imuStarted_ && imuPipeline_) {
        try {
            imuPipeline_->stop();
        } catch (...) {
            DYNALGO_LOG_WARN_S("imuPipeline_->stop() threw");
        }
        imuStarted_ = false;
    }
}

// [函数说明 / Function Description]
// 中文: 获取关联的设备
// English: Get associated device
std::shared_ptr<DynalgoDevice> ObPipeline::getDevice() const {
    return std::make_shared<ObDevice>(obDevice_);
}

// [函数说明 / Function Description]
// 中文: 获取当前对齐模式
// English: Get current align mode
DynalgoAlignMode ObPipeline::getAlignMode() const {
    if (hwD2CMode_)
        return DynalgoAlignMode::HW;
    if (alignFilter_)
        return DynalgoAlignMode::SW;
    return DynalgoAlignMode::NONE;
}

// === ObContext ===

bool ObContext::sdkInitialized_ = false;

// [函数说明 / Function Description]
// 中文: 初始化 Orbbec SDK（设置扩展目录）
// English: Initialize Orbbec SDK (set extensions directory)
void ObContext::initSDK(const std::string& extensionsDir) {
    if (sdkInitialized_)
        return;
    if (!extensionsDir.empty()) {
        ob::Context::setExtensionsDirectory(extensionsDir.c_str());
        DYNALGO_LOG_INFO_S("ObContext::initSDK: extensions directory set to " << extensionsDir);
    }
    sdkInitialized_ = true;
}

// [构造函数 / Constructor]
// 中文: 构造函数，使用配置文件路径初始化 ob::Context
// English: Constructor, initializes ob::Context with config file path
ObContext::ObContext(const std::string& configPath) : ctx_(configPath.c_str()) {}

// [函数说明 / Function Description]
// 中文: 获取设备数量
// English: Get device count
uint32_t ObContext::getDeviceCount() {
    return ctx_.queryDeviceList()->getCount();
}

// [函数说明 / Function Description]
// 中文: 根据索引获取设备
// English: Get device by index
std::shared_ptr<DynalgoDevice> ObContext::getDevice(uint32_t index) {
    auto dev = ctx_.queryDeviceList()->getDevice(index);
    return std::make_shared<ObDevice>(dev);
}

} // namespace dynalgo
