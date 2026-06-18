// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_capture_session.cpp — CaptureSession implementation.

#include "nio_capture_session.hpp"
#include "nio_log.hpp"

namespace nio {

CaptureSession::CaptureSession(std::shared_ptr<ob::Device> device, const std::string& safeName,
                               const std::string& deviceOutputDir, const CaptureConfig& cfg)
: device_(device), safeName_(safeName), deviceOutputDir_(deviceOutputDir), cfg_(cfg) {
    devId_ = safeName_;
    if (devId_.size() > 8)
        devId_ = devId_.substr(0, 8);
}

bool CaptureSession::setup() {
    auto devInfo = device_->getDeviceInfo();

    try {
        device_->timerSyncWithHost();
    } catch (ob::Error& e) {
        std::cerr << "Timer sync warning: " << e.what() << std::endl;
        NIO_LOG_WARN_S("Timer sync failed for " << safeName_ << ": " << e.what());
    }

    if (device_->isGlobalTimestampSupported()) {
        try {
            device_->enableGlobalTimestamp(true);
        } catch (...) {
        }
    }

    videoPipeline_ = std::make_shared<ob::Pipeline>(device_);
    videoConfig_ = std::make_shared<ob::Config>();
    videoConfig_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

    sensorFiles_ = std::make_shared<SensorFiles>();

    if (!enumerateSensors())
        return false;

    startTs_ = getTimestampMs();
    baseName_ = deviceOutputDir_ + "/" + safeName_;

    createEncodersAndTasks();
    setupFusion();
    writeIntrinsicJson();

    return true;
}

bool CaptureSession::enumerateSensors() {
    auto devInfo = device_->getDeviceInfo();
    auto pid = devInfo->getPid();
    auto vid = devInfo->getVid();
    auto sensorList = device_->getSensorList();

    bool is305 = ob_smpl::isGemini305Device(vid, pid);
    OBFormat colorPreferredFmt = is305 ? OB_FORMAT_YUYV : OB_FORMAT_MJPG;

    for (uint32_t s = 0; s < sensorList->getCount(); s++) {
        auto sensorType = sensorList->getSensorType(s);
        auto sensor = sensorList->getSensor(s);
        auto profileList = sensor->getStreamProfileList();

        switch (sensorType) {
        case OB_SENSOR_COLOR:
            {
                sensorInfo_.hasColor = true;
                sensorInfo_.colorProfile = selectBestProfile(profileList, colorPreferredFmt);
                if (sensorInfo_.colorProfile) {
                    sensorInfo_.colorFormat = sensorInfo_.colorProfile->getFormat();
                    if (sensorInfo_.colorFormat == OB_FORMAT_UNKNOWN) {
                        for (uint32_t k = 0; k < profileList->getCount(); k++) {
                            try {
                                auto p = profileList->getProfile(k)->as<ob::VideoStreamProfile>();
                                if (p && p->getFormat() != OB_FORMAT_UNKNOWN) {
                                    sensorInfo_.colorProfile = p;
                                    sensorInfo_.colorFormat = p->getFormat();
                                    break;
                                }
                            } catch (...) {
                            }
                        }
                    }
                    if (sensorInfo_.colorFormat != OB_FORMAT_UNKNOWN) {
                        videoConfig_->enableStream(sensorInfo_.colorProfile);
                        sensorInfo_.colorW = sensorInfo_.colorProfile->getWidth();
                        sensorInfo_.colorH = sensorInfo_.colorProfile->getHeight();
                        sensorInfo_.colorFps = sensorInfo_.colorProfile->getFps();
                    } else {
                        sensorInfo_.hasColor = false;
                        std::cout << " Color: no usable format found, skipping" << std::endl;
                    }
                } else {
                    sensorInfo_.hasColor = false;
                }
                if (sensorInfo_.hasColor) {
                    std::cout << " Color: " << sensorInfo_.colorW << "x" << sensorInfo_.colorH << "@"
                              << sensorInfo_.colorFps << " format=" << sensorInfo_.colorFormat << std::endl;
                    NIO_LOG_INFO_S("Color stream: " << sensorInfo_.colorW << "x" << sensorInfo_.colorH << "@"
                                                    << sensorInfo_.colorFps << " format=" << sensorInfo_.colorFormat);
                    try {
                        if (sensorInfo_.colorProfile)
                            sensorInfo_.colorIntrinsic =
                                sensorInfo_.colorProfile->as<ob::VideoStreamProfile>()->getIntrinsic();
                    } catch (...) {
                        NIO_LOG_WARN_S("Color intrinsic not available for " << safeName_);
                    }
                }
                break;
            }
        case OB_SENSOR_DEPTH:
            {
                sensorInfo_.hasDepth = true;
                sensorInfo_.depthProfile = selectBestProfile(profileList, OB_FORMAT_Y16);
                if (sensorInfo_.depthProfile) {
                    sensorInfo_.depthFormat = sensorInfo_.depthProfile->getFormat();
                    if (sensorInfo_.depthFormat == OB_FORMAT_UNKNOWN) {
                        for (uint32_t k = 0; k < profileList->getCount(); k++) {
                            try {
                                auto p = profileList->getProfile(k)->as<ob::VideoStreamProfile>();
                                if (p && p->getFormat() != OB_FORMAT_UNKNOWN) {
                                    sensorInfo_.depthProfile = p;
                                    sensorInfo_.depthFormat = p->getFormat();
                                    break;
                                }
                            } catch (...) {
                            }
                        }
                    }
                    if (sensorInfo_.depthFormat != OB_FORMAT_UNKNOWN) {
                        videoConfig_->enableStream(sensorInfo_.depthProfile);
                        sensorInfo_.depthW = sensorInfo_.depthProfile->getWidth();
                        sensorInfo_.depthH = sensorInfo_.depthProfile->getHeight();
                        sensorInfo_.depthFps = sensorInfo_.depthProfile->getFps();
                    } else {
                        sensorInfo_.hasDepth = false;
                        std::cout << " Depth: no usable format found, skipping" << std::endl;
                    }
                } else {
                    sensorInfo_.hasDepth = false;
                }
                if (sensorInfo_.hasDepth) {
                    std::cout << " Depth: " << sensorInfo_.depthW << "x" << sensorInfo_.depthH << "@"
                              << sensorInfo_.depthFps << " format=" << sensorInfo_.depthFormat << std::endl;
                    NIO_LOG_INFO_S("Depth stream: " << sensorInfo_.depthW << "x" << sensorInfo_.depthH << "@"
                                                    << sensorInfo_.depthFps << " format=" << sensorInfo_.depthFormat);
                }
                try {
                    int32_t precisionLevel = device_->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
                    switch (precisionLevel) {
                    case 0:
                        depthScale_ = 0.001f;
                        break;
                    case 1:
                        depthScale_ = 0.0005f;
                        break;
                    case 2:
                        depthScale_ = 0.00025f;
                        break;
                    case 3:
                        depthScale_ = 0.0001f;
                        break;
                    default:
                        depthScale_ = 0.001f;
                        break;
                    }
                    std::cout << " Depth scale: " << depthScale_ << " (precision level " << precisionLevel << ")"
                              << std::endl;
                    NIO_LOG_INFO_S("Depth scale: " << depthScale_ << " precision_level=" << precisionLevel);
                } catch (...) {
                    depthScale_ = 0.001f;
                    std::cout << " Depth scale: 0.001 (default)" << std::endl;
                }
                try {
                    if (sensorInfo_.depthProfile)
                        sensorInfo_.depthIntrinsic =
                            sensorInfo_.depthProfile->as<ob::VideoStreamProfile>()->getIntrinsic();
                } catch (...) {
                    NIO_LOG_WARN_S("Depth intrinsic not available for " << safeName_);
                }
                break;
            }
        case OB_SENSOR_IR:
            {
                sensorInfo_.hasIR = true;
                sensorInfo_.irProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if (sensorInfo_.irProfile) {
                    sensorInfo_.irFormat = sensorInfo_.irProfile->getFormat();
                    if (sensorInfo_.irFormat == OB_FORMAT_UNKNOWN)
                        sensorInfo_.irFormat = OB_FORMAT_Y8;
                    videoConfig_->enableStream(sensorInfo_.irProfile);
                    sensorInfo_.irW = sensorInfo_.irProfile->getWidth();
                    sensorInfo_.irH = sensorInfo_.irProfile->getHeight();
                    sensorInfo_.irFps = sensorInfo_.irProfile->getFps();
                } else {
                    sensorInfo_.hasIR = false;
                }
                if (sensorInfo_.hasIR) {
                    std::cout << " IR: " << sensorInfo_.irW << "x" << sensorInfo_.irH << "@" << sensorInfo_.irFps
                              << " format=" << sensorInfo_.irFormat << std::endl;
                    NIO_LOG_INFO_S("IR stream: " << sensorInfo_.irW << "x" << sensorInfo_.irH << "@"
                                                 << sensorInfo_.irFps << " format=" << sensorInfo_.irFormat);
                }
                break;
            }
        case OB_SENSOR_IR_LEFT:
            {
                sensorInfo_.hasIRLeft = true;
                sensorInfo_.irLeftProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if (sensorInfo_.irLeftProfile) {
                    sensorInfo_.irLeftFormat = sensorInfo_.irLeftProfile->getFormat();
                    if (sensorInfo_.irLeftFormat == OB_FORMAT_UNKNOWN)
                        sensorInfo_.irLeftFormat = OB_FORMAT_Y8;
                    videoConfig_->enableStream(sensorInfo_.irLeftProfile);
                    sensorInfo_.irLW = sensorInfo_.irLeftProfile->getWidth();
                    sensorInfo_.irLH = sensorInfo_.irLeftProfile->getHeight();
                    sensorInfo_.irLFps = sensorInfo_.irLeftProfile->getFps();
                } else {
                    sensorInfo_.hasIRLeft = false;
                }
                if (sensorInfo_.hasIRLeft) {
                    std::cout << " IR Left: " << sensorInfo_.irLW << "x" << sensorInfo_.irLH << "@"
                              << sensorInfo_.irLFps << " format=" << sensorInfo_.irLeftFormat << std::endl;
                    NIO_LOG_INFO_S("IR Left stream: " << sensorInfo_.irLW << "x" << sensorInfo_.irLH << "@"
                                                      << sensorInfo_.irLFps << " format=" << sensorInfo_.irLeftFormat);
                }
                break;
            }
        case OB_SENSOR_IR_RIGHT:
            {
                sensorInfo_.hasIRRight = true;
                sensorInfo_.irRightProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if (sensorInfo_.irRightProfile) {
                    sensorInfo_.irRightFormat = sensorInfo_.irRightProfile->getFormat();
                    if (sensorInfo_.irRightFormat == OB_FORMAT_UNKNOWN)
                        sensorInfo_.irRightFormat = OB_FORMAT_Y8;
                    videoConfig_->enableStream(sensorInfo_.irRightProfile);
                    sensorInfo_.irRW = sensorInfo_.irRightProfile->getWidth();
                    sensorInfo_.irRH = sensorInfo_.irRightProfile->getHeight();
                    sensorInfo_.irRFps = sensorInfo_.irRightProfile->getFps();
                } else {
                    sensorInfo_.hasIRRight = false;
                }
                if (sensorInfo_.hasIRRight) {
                    std::cout << " IR Right: " << sensorInfo_.irRW << "x" << sensorInfo_.irRH << "@"
                              << sensorInfo_.irRFps << " format=" << sensorInfo_.irRightFormat << std::endl;
                    NIO_LOG_INFO_S("IR Right stream: " << sensorInfo_.irRW << "x" << sensorInfo_.irRH << "@"
                                                       << sensorInfo_.irRFps
                                                       << " format=" << sensorInfo_.irRightFormat);
                }
                break;
            }
        case OB_SENSOR_ACCEL:
            sensorInfo_.hasAccel = true;
            break;
        case OB_SENSOR_GYRO:
            sensorInfo_.hasGyro = true;
            break;
        default:
            break;
        }
    }

    auto devInfo2 = device_->getDeviceInfo();
    if (ob_smpl::isGemini305gDevice(devInfo2->getVid(), devInfo2->getPid(), devInfo2->getConnectionType())) {
        videoConfig_->disableStream(OB_SENSOR_IR_LEFT);
        sensorInfo_.hasIRLeft = false;
        std::cout << "  Gemini 305g: disabled IR_LEFT" << std::endl;
        NIO_LOG_INFO("Gemini 305g detected, disabled IR_LEFT stream");
    }

    if (sensorInfo_.hasColor && sensorInfo_.hasDepth && sensorInfo_.colorProfile && sensorInfo_.depthProfile) {
        bool hwD2CSupported = checkHWD2CAlign();
        if (hwD2CSupported) {
            videoConfig_->setAlignMode(ALIGN_D2C_HW_MODE);
            hwD2CMode_ = true;
            std::cout << "  HW D2C: supported, using hardware depth-to-color alignment" << std::endl;
            NIO_LOG_INFO_S("HW D2C supported for " << safeName_ << ", using ALIGN_D2C_HW_MODE");
        } else {
            std::cout << "  HW D2C: not supported, using software alignment" << std::endl;
            NIO_LOG_INFO_S("HW D2C not supported for " << safeName_ << ", using SW alignment");
        }
    }

    return true;
}

void CaptureSession::createEncodersAndTasks() {
    auto sf = sensorFiles_;

    if (sensorInfo_.hasColor && sensorInfo_.colorFormat != OB_FORMAT_UNKNOWN) {
        sf->color = createStreamEncoder(baseName_ + "_color_" + startTs_ + ".h264", sensorInfo_.colorFormat,
                                        sensorInfo_.colorW, sensorInfo_.colorH, sensorInfo_.colorFps, nullptr, false);
        auto task = std::make_shared<EncodeStreamTask>(devId_ + "_color_enc", sf->color);
        task->start();
        frameConsumers_.push_back(
            std::unique_ptr<FrameConsumer>(new ColorFrameConsumer(task, nullptr, -1, ViewerChannel::COLOR, sf)));
        NIO_LOG_INFO_S("Color output: " << baseName_ + "_color_" + startTs_ + ".h264"
                                        << " fmt=" << sensorInfo_.colorFormat);
    }
    if (sensorInfo_.hasDepth && sensorInfo_.depthFormat != OB_FORMAT_UNKNOWN) {
        sf->depth = createStreamEncoder(baseName_ + "_depth_" + startTs_ + ".h264", sensorInfo_.depthFormat,
                                        sensorInfo_.depthW, sensorInfo_.depthH, sensorInfo_.depthFps, nullptr, false);
        sf->depthRawFile =
            std::make_shared<std::ofstream>(baseName_ + "_depth_raw_" + startTs_ + ".raw", std::ios::binary);
        auto encTask = std::make_shared<EncodeStreamTask>(devId_ + "_depth_enc", sf->depth);
        encTask->start();
        auto rawTask = std::make_shared<DepthRawTask>(devId_ + "_depth_raw", sf->depthRawFile, sensorInfo_.depthW,
                                                      sensorInfo_.depthH, depthScale_);
        rawTask->start();
        frameConsumers_.push_back(std::unique_ptr<FrameConsumer>(new DepthFrameConsumer(
            encTask, rawTask, nullptr, -1, ViewerChannel::DEPTH, sf, depthScale_, cfg_.depthMinM, cfg_.depthMaxM)));
        NIO_LOG_INFO_S("Depth output: " << baseName_ + "_depth_" + startTs_ + ".h264" << " + raw");
    }
    if (sensorInfo_.hasIR && sensorInfo_.irFormat != OB_FORMAT_UNKNOWN) {
        sf->ir = createStreamEncoder(baseName_ + "_ir_" + startTs_ + ".h264", sensorInfo_.irFormat, sensorInfo_.irW,
                                     sensorInfo_.irH, sensorInfo_.irFps, nullptr, false);
        auto task = std::make_shared<EncodeStreamTask>(devId_ + "_ir_enc", sf->ir);
        task->start();
        frameConsumers_.push_back(
            std::unique_ptr<FrameConsumer>(new IRFrameConsumer(OB_FRAME_IR, task, nullptr, -1, ViewerChannel::IR, sf)));
        NIO_LOG_INFO_S("IR output: " << baseName_ + "_ir_" + startTs_ + ".h264");
    }
    if (sensorInfo_.hasIRLeft && sensorInfo_.irLeftFormat != OB_FORMAT_UNKNOWN) {
        sf->irLeft = createStreamEncoder(baseName_ + "_ir_left_" + startTs_ + ".h264", sensorInfo_.irLeftFormat,
                                         sensorInfo_.irLW, sensorInfo_.irLH, sensorInfo_.irLFps, nullptr, false);
        auto task = std::make_shared<EncodeStreamTask>(devId_ + "_irl_enc", sf->irLeft);
        task->start();
        frameConsumers_.push_back(std::unique_ptr<FrameConsumer>(
            new IRFrameConsumer(OB_FRAME_IR_LEFT, task, nullptr, -1, ViewerChannel::IR_LEFT, sf)));
        NIO_LOG_INFO_S("IR Left output: " << baseName_ + "_ir_left_" + startTs_ + ".h264");
    }
    if (sensorInfo_.hasIRRight && sensorInfo_.irRightFormat != OB_FORMAT_UNKNOWN) {
        sf->irRight = createStreamEncoder(baseName_ + "_ir_right_" + startTs_ + ".h264", sensorInfo_.irRightFormat,
                                          sensorInfo_.irRW, sensorInfo_.irRH, sensorInfo_.irRFps, nullptr, false);
        auto task = std::make_shared<EncodeStreamTask>(devId_ + "_irr_enc", sf->irRight);
        task->start();
        frameConsumers_.push_back(std::unique_ptr<FrameConsumer>(
            new IRFrameConsumer(OB_FRAME_IR_RIGHT, task, nullptr, -1, ViewerChannel::IR_RIGHT, sf)));
        NIO_LOG_INFO_S("IR Right output: " << baseName_ + "_ir_right_" + startTs_ + ".h264");
    }
    if (sensorInfo_.hasAccel || sensorInfo_.hasGyro) {
        sf->imuFile = std::make_shared<std::ofstream>(baseName_ + "_imu_" + startTs_ + ".txt");
        *sf->imuFile << "# host_ts_ms,type,device_ts_us,x,y,z,temperature\n";
        sf->imuFile->flush();
        imuTask_ = std::make_shared<ImuStreamTask>(devId_ + "_imu", sf->imuFile);
        imuTask_->start();
        NIO_LOG_INFO_S("IMU output: " << baseName_ + "_imu_" + startTs_ + ".txt");
    }
}

void CaptureSession::setupFusion() {
    canFuse_ = cfg_.enableFusion && sensorInfo_.hasColor && sensorInfo_.hasDepth;
    if (!canFuse_) {
        if (cfg_.enableFusion && !sensorInfo_.hasColor) {
            std::cout << " D2C Fusion: skipped (no color sensor)" << std::endl;
            NIO_LOG_DEBUG_S("D2C Fusion skipped for " << safeName_ << ": no color sensor");
        } else if (cfg_.enableFusion && !sensorInfo_.hasDepth) {
            std::cout << " D2C Fusion: skipped (no depth sensor)" << std::endl;
            NIO_LOG_DEBUG_S("D2C Fusion skipped for " << safeName_ << ": no depth sensor");
        }
        return;
    }

    if (!hwD2CMode_)
        alignFilter_ = std::make_shared<ob::Align>(OB_STREAM_COLOR);

    fusedFps_ = std::min(sensorInfo_.colorFps, sensorInfo_.depthFps);

    std::string fusedPath = baseName_ + "_d2c_fused_" + startTs_ + ".h264";
    auto fusedFile = std::make_shared<std::ofstream>(fusedPath, std::ios::binary);

    auto fusedEncoder = std::make_shared<H264Encoder>();
    if (!fusedEncoder->initRGB(sensorInfo_.colorW, sensorInfo_.colorH, fusedFps_)) {
        std::cerr << "  Failed to init fused H264 encoder for " << safeName_ << std::endl;
        NIO_LOG_ERROR_S("Failed to init fused H264 encoder for " << safeName_ << " " << sensorInfo_.colorW << "x"
                                                                 << sensorInfo_.colorH << "@" << fusedFps_);
        canFuse_ = false;
        return;
    }

    mjpgRes_ = std::make_shared<MjpgDecoderRes>();
    mjpgRes_->init(sensorInfo_.colorW, sensorInfo_.colorH, sensorInfo_.colorFormat);
    fusionTask_ = std::make_shared<FusionStreamTask>(devId_ + "_fusion", sensorInfo_.colorW, sensorInfo_.colorH,
                                                     sensorInfo_.colorFormat, fusedFps_, fusedEncoder, fusedFile,
                                                     fusedMtx_, alignFilter_, hwD2CMode_, cfg_.alpha, cfg_.depthMinM,
                                                     cfg_.depthMaxM, depthScale_, mjpgRes_);
    fusionTask_->start();
    std::cout << "  D2C Fusion: " << sensorInfo_.colorW << "x" << sensorInfo_.colorH << "@" << fusedFps_
              << " alpha=" << cfg_.alpha << " depth=[" << cfg_.depthMinM << "m, " << cfg_.depthMaxM << "m]"
              << " mode=" << (hwD2CMode_ ? "HW" : "SW") << std::endl;
    NIO_LOG_INFO_S("D2C Fusion enabled: " << sensorInfo_.colorW << "x" << sensorInfo_.colorH << "@" << fusedFps_
                                          << " alpha=" << cfg_.alpha << " depthRange=" << cfg_.depthMinM << "m-"
                                          << cfg_.depthMaxM << "m"
                                          << " mode=" << (hwD2CMode_ ? "HW" : "SW") << " output=" << fusedPath);
}

void CaptureSession::writeIntrinsicJson() {
    if (!sensorInfo_.hasDepth)
        return;

    std::string intrinsicPath = baseName_ + "_depth_intrinsic_" + startTs_ + ".json";
    std::ofstream jf(intrinsicPath);
    if (jf.is_open()) {
        jf << "{\n";
        jf << "  \"depth\": {\"fx\":" << sensorInfo_.depthIntrinsic.fx << ",\"fy\":" << sensorInfo_.depthIntrinsic.fy
           << ",\"cx\":" << sensorInfo_.depthIntrinsic.cx << ",\"cy\":" << sensorInfo_.depthIntrinsic.cy
           << ",\"width\":" << sensorInfo_.depthIntrinsic.width << ",\"height\":" << sensorInfo_.depthIntrinsic.height
           << "},\n";
        jf << "  \"color\": {\"fx\":" << sensorInfo_.colorIntrinsic.fx << ",\"fy\":" << sensorInfo_.colorIntrinsic.fy
           << ",\"cx\":" << sensorInfo_.colorIntrinsic.cx << ",\"cy\":" << sensorInfo_.colorIntrinsic.cy
           << ",\"width\":" << sensorInfo_.colorIntrinsic.width << ",\"height\":" << sensorInfo_.colorIntrinsic.height
           << "},\n";
        jf << "  \"depth_scale\":" << depthScale_ << ",\n";
        jf << "  \"device\":\"" << safeName_ << "\"\n";
        jf << "}\n";
        std::cout << " Intrinsic: " << intrinsicPath << std::endl;
        NIO_LOG_INFO_S("Intrinsic JSON: " << intrinsicPath);
    }
}

bool CaptureSession::checkHWD2CAlign() {
    auto hwD2CDepthProfiles = videoPipeline_->getD2CDepthProfileList(sensorInfo_.colorProfile, ALIGN_D2C_HW_MODE);
    if (!hwD2CDepthProfiles || hwD2CDepthProfiles->getCount() == 0)
        return false;

    auto depthVsp = sensorInfo_.depthProfile->as<ob::VideoStreamProfile>();
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

// ---------------------------------------------------------------------------
// Video consumer thread: dequeues FrameSets and dispatches to workers
// ---------------------------------------------------------------------------

void CaptureSession::videoConsumerLoop() {
    setThreadName(devId_ + "_vcons");
    NIO_LOG_DEBUG_S("Video consumer started: " << safeName_);

    while (consumersRunning_.load()) {
        std::shared_ptr<ob::FrameSet> frameSet;
        if (!videoQueue_.pop(frameSet))
            continue;
        if (!frameSet)
            continue;

        if (canFuse_ && fusionTask_) {
            if (hwD2CMode_) {
                auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
                auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
                if (colorFrame && depthFrame) {
                    fusionTask_->enqueueColor(colorFrame->getData(), colorFrame->getDataSize(),
                                              colorFrame->getTimeStampUs());
                    float dsForFusion = depthScale_;
                    try {
                        auto depthF = depthFrame->as<ob::DepthFrame>();
                        if (depthF)
                            dsForFusion = depthF->getValueScale();
                    } catch (...) {
                    }
                    fusionTask_->enqueueDepth(depthFrame->getData(), depthFrame->getDataSize(),
                                              depthFrame->getTimeStampUs(), dsForFusion);
                }
            } else {
                fusionTask_->enqueueFrameSet(frameSet);
            }
        }

        for (auto& fc : frameConsumers_)
            fc->consume(frameSet);
    }

    NIO_LOG_DEBUG_S("Video consumer stopped: " << safeName_);
}

// ---------------------------------------------------------------------------
// IMU consumer thread: dequeues CSV lines and writes to ImuStreamTask
// ---------------------------------------------------------------------------

void CaptureSession::imuConsumerLoop() {
    setThreadName(devId_ + "_icons");
    NIO_LOG_DEBUG_S("IMU consumer started: " << safeName_);

    while (consumersRunning_.load()) {
        std::string line;
        if (!imuQueue_.pop(line))
            continue;
        if (imuTask_)
            imuTask_->enqueueLine(std::move(line));
    }

    NIO_LOG_DEBUG_S("IMU consumer stopped: " << safeName_);
}

// ---------------------------------------------------------------------------
// Pipeline start / stop
// ---------------------------------------------------------------------------

void CaptureSession::startVideoPipeline(SDLViewer& viewer, bool noShow) {
    viewer_ = &viewer;

    if (!noShow) {
        OBFormat depthSlotFmt = OB_FORMAT_Y16;
        int depthSlotW = sensorInfo_.depthW;
        int depthSlotH = sensorInfo_.depthH;
        if (sensorInfo_.hasDepth && hwD2CMode_ && sensorInfo_.hasColor) {
            depthSlotW = sensorInfo_.colorW;
            depthSlotH = sensorInfo_.colorH;
        }
        std::string camType = device_->getDeviceInfo()->getName();
        std::replace(camType.begin(), camType.end(), ' ', '_');
        auto devInfo = device_->getDeviceInfo();
        viewerIdx_ = viewer.addDevice(safeName_, camType, devInfo->getSerialNumber(), sensorInfo_.hasColor,
                                      sensorInfo_.colorFormat, sensorInfo_.colorW, sensorInfo_.colorH,
                                      sensorInfo_.hasDepth, depthSlotFmt, depthSlotW, depthSlotH, sensorInfo_.hasIR,
                                      sensorInfo_.irW, sensorInfo_.irH, sensorInfo_.hasIRLeft, sensorInfo_.irLW,
                                      sensorInfo_.irLH, sensorInfo_.hasIRRight, sensorInfo_.irRW, sensorInfo_.irRH);
    }

    try {
        videoPipeline_->enableFrameSync();
    } catch (...) {
    }

    if (viewerIdx_ >= 0) {
        for (auto& fc : frameConsumers_)
            fc->setViewer(viewer_, viewerIdx_);
    }

    consumersRunning_ = true;
    videoConsumerThread_ = std::thread(&CaptureSession::videoConsumerLoop, this);

    try {
        videoPipeline_->start(videoConfig_, [this](std::shared_ptr<ob::FrameSet> frameSet) {
            if (frameSet)
                videoQueue_.push(std::move(frameSet));
        });
    } catch (ob::Error& e) {
        std::cerr << " Pipeline start failed for " << safeName_ << ": " << e.what() << std::endl;
        NIO_LOG_ERROR_S("Pipeline start failed for " << safeName_ << ": " << e.what());
        consumersRunning_ = false;
        videoQueue_.shutdown();
        if (videoConsumerThread_.joinable())
            videoConsumerThread_.join();
        videoPipeline_.reset();
        return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void CaptureSession::startImuPipeline() {
    if (!hasIMU())
        return;

    NIO_LOG_INFO_S("Starting IMU pipeline for " << safeName_);
    auto imuDev = videoPipeline_->getDevice();
    imuPipeline_ = std::make_shared<ob::Pipeline>(imuDev);
    std::shared_ptr<ob::Config> imuConfig = std::make_shared<ob::Config>();
    imuConfig->enableAccelStream();
    imuConfig->enableGyroStream();
    imuConfig->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

    imuConsumerThread_ = std::thread(&CaptureSession::imuConsumerLoop, this);

    imuPipeline_->start(imuConfig, [this](std::shared_ptr<ob::FrameSet> frameSet) {
        if (!frameSet)
            return;

        auto accelFrameRaw = frameSet->getFrame(OB_FRAME_ACCEL);
        auto gyroFrameRaw = frameSet->getFrame(OB_FRAME_GYRO);

        if (accelFrameRaw) {
            try {
                auto accelFrame = accelFrameRaw->as<ob::AccelFrame>();
                auto val = accelFrame->getValue();
                auto ts = accelFrame->getTimeStampUs();
                auto temp = accelFrame->getTemperature();
                auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count();
                std::ostringstream oss;
                oss << nowMs << ",ACCEL," << ts << "," << val.x << "," << val.y << "," << val.z << "," << temp << "\n";
                imuQueue_.push(oss.str());
            } catch (...) {
            }
            std::lock_guard<std::mutex> lock(sensorFiles_->countMtx);
            sensorFiles_->frameCounts[OB_FRAME_ACCEL]++;
        }

        if (gyroFrameRaw) {
            try {
                auto gyroFrame = gyroFrameRaw->as<ob::GyroFrame>();
                auto val = gyroFrame->getValue();
                auto ts = gyroFrame->getTimeStampUs();
                auto temp = gyroFrame->getTemperature();
                auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count();
                std::ostringstream oss;
                oss << nowMs << ",GYRO," << ts << "," << val.x << "," << val.y << "," << val.z << "," << temp << "\n";
                imuQueue_.push(oss.str());
            } catch (...) {
            }
            std::lock_guard<std::mutex> lock(sensorFiles_->countMtx);
            sensorFiles_->frameCounts[OB_FRAME_GYRO]++;
        }
    });
}

void CaptureSession::stop() {
    consumersRunning_ = false;
    videoQueue_.shutdown();
    imuQueue_.shutdown();

    if (videoPipeline_)
        videoPipeline_->stop();
    if (hasIMU() && imuPipeline_)
        imuPipeline_->stop();

    if (videoConsumerThread_.joinable())
        videoConsumerThread_.join();
    if (imuConsumerThread_.joinable())
        imuConsumerThread_.join();

    for (auto& fc : frameConsumers_)
        fc->stopTask();
    if (fusionTask_)
        fusionTask_->stop();
    if (imuTask_)
        imuTask_->stop();

    auto& sf = sensorFiles_;
    if (sf->color && sf->color->encoder)
        sf->color->encoder->close();
    if (sf->depth && sf->depth->encoder)
        sf->depth->encoder->close();
    if (sf->ir && sf->ir->encoder)
        sf->ir->encoder->close();
    if (sf->irLeft && sf->irLeft->encoder)
        sf->irLeft->encoder->close();
    if (sf->irRight && sf->irRight->encoder)
        sf->irRight->encoder->close();

    if (sf->color && sf->color->file)
        sf->color->file->close();
    if (sf->depth && sf->depth->file)
        sf->depth->file->close();
    if (sf->ir && sf->ir->file)
        sf->ir->file->close();
    if (sf->irLeft && sf->irLeft->file)
        sf->irLeft->file->close();
    if (sf->irRight && sf->irRight->file)
        sf->irRight->file->close();
    if (sf->depthRawFile)
        sf->depthRawFile->close();
    if (sf->imuFile)
        sf->imuFile->close();

    mjpgRes_.reset();

    std::cout << "Stopped: " << safeName_ << std::endl;
    NIO_LOG_INFO_S("Stopped device: " << safeName_);
}

uint64_t CaptureSession::getAndResetFrameCount(OBFrameType type) {
    std::lock_guard<std::mutex> lock(sensorFiles_->countMtx);
    auto it = sensorFiles_->frameCounts.find(type);
    if (it == sensorFiles_->frameCounts.end())
        return 0;
    uint64_t count = it->second;
    it->second = 0;
    return count;
}

uint64_t CaptureSession::getAndResetFusionCount() {
    if (fusionTask_)
        return fusionTask_->frameCount.exchange(0);
    return 0;
}

void CaptureSession::reportFps(uint64_t reportDurationMs) {
    std::map<OBFrameType, uint64_t> tempCounts;
    {
        std::lock_guard<std::mutex> lock(sensorFiles_->countMtx);
        if (!sensorFiles_->frameCounts.empty()) {
            tempCounts = sensorFiles_->frameCounts;
            for (auto& item : sensorFiles_->frameCounts)
                item.second = 0;
        }
    }

    std::cout << "[" << safeName_ << "] ";
    if (tempCounts.empty() && !fusionTask_) {
        std::cout << "Recording... waiting for frames";
    } else {
        std::cout << "Recording... FPS: ";
        std::string sep;
        for (const auto& item : tempCounts) {
            auto name = ob::TypeHelper::convertOBFrameTypeToString(item.first);
            float rate = (reportDurationMs > 0) ? (item.second / (reportDurationMs / 1000.0f)) : 0.0f;
            std::cout << std::fixed << std::setprecision(1) << sep << name << "=" << rate;
            sep = ", ";
            NIO_LOG_TRACE_S("[" << safeName_ << "] " << name << "=" << std::fixed << std::setprecision(1) << rate);
        }
        if (fusionTask_) {
            uint64_t fusedCount = fusionTask_->frameCount.exchange(0);
            float fusedRate = (reportDurationMs > 0) ? (fusedCount / (reportDurationMs / 1000.0f)) : 0.0f;
            std::cout << sep << "fused=" << std::fixed << std::setprecision(1) << fusedRate;
            NIO_LOG_TRACE_S("[" << safeName_ << "] fused=" << std::fixed << std::setprecision(1) << fusedRate);
        }
    }
    std::cout << std::endl;
}

} // namespace nio
