// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_ob_device.cpp — Orbbec SDK NioDevice/NioPipeline/NioContext impl.

#include "nio_ob_device.hpp"
#include "nio_common.hpp"
#include "nio_log.hpp"

namespace nio {

// === ObDevice ===

ObDevice::ObDevice(std::shared_ptr<ob::Device> device)
: obDevice_(std::move(device)) {}

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
        case OB_SENSOR_COLOR:    si.hasColor = true; break;
        case OB_SENSOR_DEPTH:    si.hasDepth = true; break;
        case OB_SENSOR_IR:       si.hasIR = true; break;
        case OB_SENSOR_IR_LEFT:  si.hasIRLeft = true; break;
        case OB_SENSOR_IR_RIGHT: si.hasIRRight = true; break;
        case OB_SENSOR_ACCEL:   si.hasAccel = true; break;
        case OB_SENSOR_GYRO:    si.hasGyro = true; break;
        default: break;
        }
    }

    cachedSensorInfo_ = si;
    sensorInfoCached_ = true;
    return si;
}

int32_t ObDevice::getIntProperty(int propertyId) {
    return obDevice_->getIntProperty(static_cast<OBPropertyID>(propertyId));
}

// === ObPipeline ===

ObPipeline::ObPipeline(std::shared_ptr<ob::Device> device)
: obDevice_(device) {
    obPipeline_ = std::make_shared<ob::Pipeline>(device);
    obConfig_ = std::make_shared<ob::Config>();
    obConfig_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
}

void ObPipeline::enableStream(const NioStreamConfig &/*cfg*/) {
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

bool ObPipeline::checkHWD2CSupport(int /*colorW*/, int /*colorH*/, NioFormat /*colorFmt*/,
                                    int /*depthW*/, int /*depthH*/, NioFormat /*depthFmt*/, int /*depthFps*/) {
    // 需要ob::VideoStreamProfile，Phase 4完整实现时从device获取
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
                auto nioFs = std::make_shared<NioFrameSet>(obFrameSetToNio(obFs));
                nioFs->nativeFrameSet = std::static_pointer_cast<void>(obFs);
                videoCallback_(nioFs);
            }
        });
        videoCallback_ = callback;
        return true;
    } catch (ob::Error &e) {
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
                auto nioFs = std::make_shared<NioFrameSet>(obFrameSetToNio(obFs));
                imuCallback_(nioFs);
            }
        });
        imuStarted_ = true;
        return true;
    } catch (ob::Error &e) {
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

// === ObContext ===

ObContext::ObContext() {}

uint32_t ObContext::getDeviceCount() {
    return ctx_.queryDeviceList()->getCount();
}

std::shared_ptr<NioDevice> ObContext::getDevice(uint32_t index) {
    auto dev = ctx_.queryDeviceList()->getDevice(index);
    return std::make_shared<ObDevice>(dev);
}

} // namespace nio
