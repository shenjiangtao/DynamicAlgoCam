// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_ob_device.cpp — Orbbec SDK NioDevice/NioPipeline/NioContext impl.

#include "nio_ob_device.hpp"
#include "nio_common.hpp"
#include "nio_log.hpp"

namespace nio {

ObDevice::ObDevice(std::shared_ptr<ob::Device> device) : obDevice_(std::move(device)) {}

NioDeviceInfo ObDevice::getDeviceInfo() const {
    auto di = obDevice_->getDeviceInfo();
    NioDeviceInfo info;
    info.name = di->getName();
    info.serialNumber = di->getSerialNumber();
    info.vid = di->getVid();
    info.pid = di->getPid();
    info.connectionType = di->getConnectionType();
    return info;
}

void ObDevice::timerSyncWithHost() {
    obDevice_->timerSyncWithHost();
}

bool ObDevice::isGlobalTimestampSupported() const {
    return obDevice_->isGlobalTimestampSupported();
}

void ObDevice::enableGlobalTimestamp(bool enable) {
    obDevice_->enableGlobalTimestamp(enable);
}

NioSensorInfo ObDevice::getSensorInfo() const {
    if (sensorInfoCached_)
        return cachedSensorInfo_;

    auto sensorList = obDevice_->getSensorList();
    NioSensorInfo si;

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

int32_t ObDevice::getIntProperty(int propertyId) {
    return obDevice_->getIntProperty(static_cast<OBPropertyID>(propertyId));
}

bool ObDevice::hasIRSensor() const {
    auto si = getSensorInfo();
    return si.hasIR || si.hasIRLeft || si.hasIRRight;
}

NioSensorInfo ObDevice::setupPipeline(NioPipeline& pipeline) {
    auto* obPipe = dynamic_cast<ObPipeline*>(&pipeline);
    if (!obPipe) {
        NIO_LOG_ERROR("setupPipeline: pipeline is not ObPipeline");
        return NioSensorInfo{};
    }

    auto devInfo = obDevice_->getDeviceInfo();
    auto pid = devInfo->getPid();
    auto vid = devInfo->getVid();
    bool is305 = nio::isGemini305Device(vid, pid);
    bool is335L336L = nio::isGemini335L336LDevice(vid, pid);
    NioFormat colorPreferredFmt = (is305 || is335L336L) ? NioFormat::YUYV : NioFormat::MJPG;

    NioSensorInfo si;
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
                NioFormat resolvedFmt = NioFormat::UNKNOWN;
                if (profile) {
                    resolvedFmt = obFormatToNio(profile->getFormat());
                    if (resolvedFmt == NioFormat::UNKNOWN) {
                        for (uint32_t k = 0; k < profileList->getCount(); k++) {
                            try {
                                auto p = profileList->getProfile(k)->as<ob::VideoStreamProfile>();
                                if (p && p->getFormat() != OB_FORMAT_UNKNOWN) {
                                    profile = p;
                                    resolvedFmt = obFormatToNio(p->getFormat());
                                    break;
                                }
                            } catch (...) {
                            }
                        }
                    }
                }
                if (profile && resolvedFmt != NioFormat::UNKNOWN) {
                    obCfg->enableStream(profile);
                    obPipe->setColorProfile(profile);
                    si.colorFormat = resolvedFmt;
                    si.colorW = profile->getWidth();
                    si.colorH = profile->getHeight();
                    si.colorFps = profile->getFps();
                    try {
                        si.colorIntrinsic = obIntrinsicToNio(profile->as<ob::VideoStreamProfile>()->getIntrinsic());
                    } catch (...) {
                        NIO_LOG_WARN_S("Color intrinsic not available");
                    }
                } else {
                    si.hasColor = false;
                }
                break;
            }
        case OB_SENSOR_DEPTH:
            {
                si.hasDepth = true;
                auto profile = selectBestProfile(profileList, OB_FORMAT_Y16);
                NioFormat resolvedFmt = NioFormat::UNKNOWN;
                if (profile) {
                    resolvedFmt = obFormatToNio(profile->getFormat());
                    if (resolvedFmt == NioFormat::UNKNOWN) {
                        for (uint32_t k = 0; k < profileList->getCount(); k++) {
                            try {
                                auto p = profileList->getProfile(k)->as<ob::VideoStreamProfile>();
                                if (p && p->getFormat() != OB_FORMAT_UNKNOWN) {
                                    profile = p;
                                    resolvedFmt = obFormatToNio(p->getFormat());
                                    break;
                                }
                            } catch (...) {
                            }
                        }
                    }
                }
                if (profile && resolvedFmt != NioFormat::UNKNOWN) {
                    obCfg->enableStream(profile);
                    obPipe->setDepthProfile(profile);
                    si.depthFormat = resolvedFmt;
                    si.depthW = profile->getWidth();
                    si.depthH = profile->getHeight();
                    si.depthFps = profile->getFps();
                    try {
                        si.depthIntrinsic = obIntrinsicToNio(profile->as<ob::VideoStreamProfile>()->getIntrinsic());
                    } catch (...) {
                        NIO_LOG_WARN_S("Depth intrinsic not available");
                    }
                } else {
                    si.hasDepth = false;
                }
                // Depth scale
                try {
                    int32_t level = obDevice_->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
                    switch (level) {
                    case 0:
                        si.depthScale = 0.001f;
                        break;
                    case 1:
                        si.depthScale = 0.0005f;
                        break;
                    case 2:
                        si.depthScale = 0.00025f;
                        break;
                    case 3:
                        si.depthScale = 0.0001f;
                        break;
                    default:
                        si.depthScale = 0.001f;
                        break;
                    }
                    std::cout << " Depth scale: " << si.depthScale << " (precision level " << level << ")" << std::endl;
                } catch (...) {
                    si.depthScale = 0.001f;
                }
                break;
            }
        case OB_SENSOR_IR:
            {
                si.hasIR = true;
                auto profile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if (profile) {
                    si.irFormat = obFormatToNio(profile->getFormat());
                    if (si.irFormat == NioFormat::UNKNOWN)
                        si.irFormat = NioFormat::Y8;
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
                    if (si.irLeftFormat == NioFormat::UNKNOWN)
                        si.irLeftFormat = NioFormat::Y8;
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
                    if (si.irRightFormat == NioFormat::UNKNOWN)
                        si.irRightFormat = NioFormat::Y8;
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
    if (nio::isGemini305gDevice(devInfo->getVid(), devInfo->getPid(), devInfo->getConnectionType())) {
        obPipe->disableStream(NioFrameType::IR_LEFT);
        si.hasIRLeft = false;
        std::cout << "  Gemini 305g: disabled IR_LEFT" << std::endl;
    }

    // HW D2C check
    if (si.hasColor && si.hasDepth) {
        bool hwD2CSupported = obPipe->checkHWD2CSupport(si.colorW, si.colorH, si.colorFormat, si.depthW, si.depthH,
                                                        si.depthFormat, si.depthFps);
        pipeline.setAlignMode(hwD2CSupported ? NioAlignMode::HW : NioAlignMode::SW);
        std::cout << "  HW D2C: " << (hwD2CSupported ? "supported" : "not supported") << std::endl;
    }

    cachedSensorInfo_ = si;
    sensorInfoCached_ = true;
    return si;
}

// === ObPipeline ===

ObPipeline::ObPipeline(std::shared_ptr<ob::Device> device) : obDevice_(device) {
    obPipeline_ = std::make_shared<ob::Pipeline>(device);
    obConfig_ = std::make_shared<ob::Config>();
    obConfig_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
}

void ObPipeline::enableStream(const NioStreamConfig& /*cfg*/) {
    // 流启用需要 ob::VideoStreamProfile，由 CaptureSession 中具体实现。
    // Phase 4 重构时将完整实现。
}

void ObPipeline::disableStream(NioFrameType type) {
    obConfig_->disableStream(nioFrameTypeToObSensor(type));
}

void ObPipeline::setAggregateAllTypeFrameRequire(bool /*require*/) {
    obConfig_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
}

void ObPipeline::setAlignMode(NioAlignMode mode) {
    if (mode == NioAlignMode::HW) {
        obConfig_->setAlignMode(ALIGN_D2C_HW_MODE);
        hwD2CMode_ = true;
    } else if (mode == NioAlignMode::SW) {
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

bool ObPipeline::checkHWD2CSupport(int /*colorW*/, int /*colorH*/, NioFormat /*colorFmt*/, int /*depthW*/,
                                   int /*depthH*/, NioFormat /*depthFmt*/, int /*depthFps*/) {
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

void ObPipeline::enableFrameSync() {
    try {
        obPipeline_->enableFrameSync();
    } catch (...) {
    }
}

bool ObPipeline::start(NioVideoCallback callback) {
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
                    }
                }
                if (alignFilter_) {
                    try {
                        auto aligned = alignFilter_->process(obFs);
                        auto alignedFS = aligned ? std::dynamic_pointer_cast<ob::FrameSet>(aligned) : nullptr;
                        if (alignedFS)
                            obFs = alignedFS;
                    } catch (...) {
                    }
                }
                auto nioFs = std::make_shared<NioFrameSet>(obFrameSetToNio(obFs));
                videoCallback_(nioFs);
            }
        });
        videoCallback_ = callback;
        return true;
    } catch (ob::Error& e) {
        NIO_LOG_ERROR_S("Pipeline start failed: " << e.what());
        return false;
    }
}

bool ObPipeline::startImu(NioImuCallback callback) {
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
        NIO_LOG_ERROR_S("IMU pipeline start failed: " << e.what());
        return false;
    }
}

void ObPipeline::stop() {
    if (obPipeline_) {
        try {
            obPipeline_->stop();
        } catch (...) {
        }
    }
}

void ObPipeline::stopImu() {
    if (imuStarted_ && imuPipeline_) {
        try {
            imuPipeline_->stop();
        } catch (...) {
        }
        imuStarted_ = false;
    }
}

std::shared_ptr<NioDevice> ObPipeline::getDevice() const {
    return std::make_shared<ObDevice>(obDevice_);
}

NioAlignMode ObPipeline::getAlignMode() const {
    if (hwD2CMode_)
        return NioAlignMode::HW;
    if (alignFilter_)
        return NioAlignMode::SW;
    return NioAlignMode::NONE;
}

// === ObContext ===

bool ObContext::sdkInitialized_ = false;

void ObContext::initSDK(const std::string& extensionsDir) {
    if (sdkInitialized_)
        return;
    if (!extensionsDir.empty()) {
        ob::Context::setExtensionsDirectory(extensionsDir.c_str());
        std::cout << "ObContext::initSDK: extensions directory set to " << extensionsDir << std::endl;
    }
    sdkInitialized_ = true;
}

ObContext::ObContext(const std::string& configPath) : ctx_(configPath.c_str()) {}

uint32_t ObContext::getDeviceCount() {
    return ctx_.queryDeviceList()->getCount();
}

std::shared_ptr<NioDevice> ObContext::getDevice(uint32_t index) {
    auto dev = ctx_.queryDeviceList()->getDevice(index);
    return std::make_shared<ObDevice>(dev);
}

} // namespace nio
