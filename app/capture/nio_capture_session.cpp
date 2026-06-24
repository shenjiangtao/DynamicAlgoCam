// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_capture_session.cpp — CaptureSession implementation.

#include "nio_capture_session.hpp"
#include "nio_log.hpp"
#include "nio_sdl_viewer.hpp"

namespace nio {

CaptureSession::CaptureSession(std::shared_ptr<NioDevice> device, std::shared_ptr<NioPipeline> pipeline,
                               const std::string& safeName, const std::string& deviceOutputDir,
                               const CaptureConfig& cfg)
: device_(std::move(device))
, pipeline_(std::move(pipeline))
, safeName_(safeName)
, deviceOutputDir_(deviceOutputDir)
, cfg_(cfg) {
    devId_ = safeName_;
    if (devId_.size() > 8)
        devId_ = devId_.substr(0, 8);
}

bool CaptureSession::setup() {
    try {
        device_->timerSyncWithHost();
    } catch (std::exception& e) {
        std::cerr << "Timer sync warning: " << e.what() << std::endl;
        NIO_LOG_WARN_S("Timer sync failed for " << safeName_ << ": " << e.what());
    }

    if (device_->isGlobalTimestampSupported()) {
        try {
            device_->enableGlobalTimestamp(true);
        } catch (...) {
        }
    }

    sensorInfo_ = device_->setupPipeline(*pipeline_);
    depthScale_ = sensorInfo_.depthScale;

    sensorFiles_ = std::make_shared<SensorFiles>();

    startTs_ = getTimestampMs();
    baseName_ = deviceOutputDir_ + "/" + safeName_;

    createEncodersAndTasks();
    setupFusion();
    writeIntrinsicJson();

    return true;
}

// ---------------------------------------------------------------------------
// Encoder / task creation
// ---------------------------------------------------------------------------

void CaptureSession::createEncodersAndTasks() {
    createColorEncoder();
    createDepthEncoder();
    if (sensorInfo_.hasIR && sensorInfo_.irFormat != NioFormat::UNKNOWN)
        createIREncoder(NioFrameType::IR, "_ir_", sensorInfo_.irFormat, sensorInfo_.irW, sensorInfo_.irH,
                        sensorInfo_.irFps, ViewerChannel::IR);
    if (sensorInfo_.hasIRLeft && sensorInfo_.irLeftFormat != NioFormat::UNKNOWN)
        createIREncoder(NioFrameType::IR_LEFT, "_ir_left_", sensorInfo_.irLeftFormat, sensorInfo_.irLW,
                        sensorInfo_.irLH, sensorInfo_.irLFps, ViewerChannel::IR_LEFT);
    if (sensorInfo_.hasIRRight && sensorInfo_.irRightFormat != NioFormat::UNKNOWN)
        createIREncoder(NioFrameType::IR_RIGHT, "_ir_right_", sensorInfo_.irRightFormat, sensorInfo_.irRW,
                        sensorInfo_.irRH, sensorInfo_.irRFps, ViewerChannel::IR_RIGHT);
    createImuTask();
    createPcdTask();
}

void CaptureSession::createColorEncoder() {
    if (!sensorInfo_.hasColor || sensorInfo_.colorFormat == NioFormat::UNKNOWN)
        return;
    auto sf = sensorFiles_;
    sf->color = createStreamEncoder(baseName_ + "_color_" + startTs_ + ".h264", sensorInfo_.colorFormat,
                                    sensorInfo_.colorW, sensorInfo_.colorH, sensorInfo_.colorFps, nullptr, false);
    auto task = std::make_shared<EncodeStreamTask>(devId_ + "_color_enc", sf->color);
    task->start();
    frameConsumers_.push_back(
        std::unique_ptr<FrameConsumer>(new ColorFrameConsumer(task, nullptr, -1, ViewerChannel::COLOR, sf)));
    NIO_LOG_INFO_S("Color output: " << baseName_ + "_color_" + startTs_ + ".h264"
                                    << " fmt=" << nioFormatToStr(sensorInfo_.colorFormat));
}

void CaptureSession::createDepthEncoder() {
    if (!sensorInfo_.hasDepth || sensorInfo_.depthFormat == NioFormat::UNKNOWN)
        return;
    auto sf = sensorFiles_;
    sf->depth = createStreamEncoder(baseName_ + "_depth_" + startTs_ + ".h264", NioFormat::RGB, sensorInfo_.depthW,
                                    sensorInfo_.depthH, sensorInfo_.depthFps, nullptr, false);
    sf->depthRawFile = std::make_shared<std::ofstream>(baseName_ + "_depth_raw_" + startTs_ + ".raw", std::ios::binary);
    auto encTask = std::make_shared<EncodeStreamTask>(devId_ + "_depth_enc", sf->depth);
    encTask->start();
    auto rawTask = std::make_shared<DepthRawTask>(devId_ + "_depth_raw", sf->depthRawFile, sensorInfo_.depthW,
                                                  sensorInfo_.depthH, depthScale_);
    rawTask->start();
    frameConsumers_.push_back(std::unique_ptr<FrameConsumer>(
        new DepthFrameConsumer(encTask, rawTask, nullptr, -1, ViewerChannel::DEPTH, sf, depthScale_, cfg_.depthMinM,
                               cfg_.depthMaxM, sensorInfo_.depthW, sensorInfo_.depthH)));
    NIO_LOG_INFO_S("Depth output: " << baseName_ + "_depth_" + startTs_ + ".h264" << " + raw (jet RGB encoded)");
}

void CaptureSession::createIREncoder(NioFrameType type, const std::string& suffix, NioFormat fmt, int w, int h, int fps,
                                     ViewerChannel ch) {
    auto sf = sensorFiles_;
    auto se = createStreamEncoder(baseName_ + suffix + startTs_ + ".h264", fmt, w, h, fps, nullptr, false);
    if (type == NioFrameType::IR)
        sf->ir = se;
    else if (type == NioFrameType::IR_LEFT)
        sf->irLeft = se;
    else
        sf->irRight = se;

    std::string taskId = devId_ + "_" +
                         (type == NioFrameType::IR      ? "ir" :
                          type == NioFrameType::IR_LEFT ? "irl" :
                                                          "irr") +
                         "_enc";
    auto task = std::make_shared<EncodeStreamTask>(taskId, se);
    task->start();
    frameConsumers_.push_back(std::unique_ptr<FrameConsumer>(new IRFrameConsumer(type, task, nullptr, -1, ch, sf)));
    NIO_LOG_INFO_S("IR" << (type == NioFrameType::IR_LEFT  ? " Left" :
                            type == NioFrameType::IR_RIGHT ? " Right" :
                                                             "")
                        << " output: " << baseName_ + suffix + startTs_ + ".h264");
}

void CaptureSession::createImuTask() {
    if (!hasIMU())
        return;
    auto sf = sensorFiles_;
    sf->imuFile = std::make_shared<std::ofstream>(baseName_ + "_imu_" + startTs_ + ".txt");
    *sf->imuFile << "# host_ts_ms,type,device_ts_us,x,y,z,temperature\n";
    sf->imuFile->flush();
    imuTask_ = std::make_shared<ImuStreamTask>(devId_ + "_imu", sf->imuFile);
    imuTask_->start();
    NIO_LOG_INFO_S("IMU output: " << baseName_ + "_imu_" + startTs_ + ".txt");
}

void CaptureSession::createPcdTask() {
    if (!pipeline_ || !pipeline_->isPointCloudDepth())
        return;

    std::string pcdPath = baseName_ + "_point_raw_" + startTs_ + ".raw";
    auto sf = sensorFiles_;
    sf->pcdFile = openBufferedFile(pcdPath, std::ios::binary);

    pcdTask_ = std::make_shared<PcdStreamTask>(devId_ + "_pcd", sf->pcdFile);
    pcdTask_->start();

    frameConsumers_.push_back(std::unique_ptr<FrameConsumer>(new PointcloudFrameConsumer(pcdTask_, sf)));

    NIO_LOG_INFO_S("PCD point cloud output: " << pcdPath);
    std::cout << "  PCD point cloud: " << pcdPath << std::endl;
}

// ---------------------------------------------------------------------------
// Fusion
// ---------------------------------------------------------------------------

void CaptureSession::setupFusion() {
    canFuse_ = cfg_.enableFusion && sensorInfo_.hasColor && sensorInfo_.hasDepth;
    if (!canFuse_) {
        if (cfg_.enableFusion && !sensorInfo_.hasColor) {
            std::cout << " D2C Fusion: skipped (no color sensor)" << std::endl;
        } else if (cfg_.enableFusion && !sensorInfo_.hasDepth) {
            std::cout << " D2C Fusion: skipped (no depth sensor)" << std::endl;
        }
        return;
    }

    hwD2CMode_ = (pipeline_->getAlignMode() == NioAlignMode::HW);

    std::shared_ptr<NioD2CAlign> alignFilter;
    if (!hwD2CMode_) {
        alignFilter = pipeline_->getD2CAlignFilter();
    }

    fusedFps_ = std::min(sensorInfo_.colorFps, sensorInfo_.depthFps);

    std::string fusedPath = baseName_ + "_d2c_fused_" + startTs_ + ".h264";
    auto fusedFile = std::make_shared<std::ofstream>(fusedPath, std::ios::binary);

    auto fusedEncoder = std::make_shared<H264Encoder>();
    if (!fusedEncoder->initRGB(sensorInfo_.colorW, sensorInfo_.colorH, fusedFps_)) {
        std::cerr << "  Failed to init fused H264 encoder for " << safeName_ << std::endl;
        canFuse_ = false;
        return;
    }

    mjpgRes_ = std::make_shared<MjpgDecoderRes>();
    mjpgRes_->init(sensorInfo_.colorW, sensorInfo_.colorH, sensorInfo_.colorFormat);
    fusionTask_ = std::make_shared<FusionStreamTask>(devId_ + "_fusion", sensorInfo_.colorW, sensorInfo_.colorH,
                                                     sensorInfo_.colorFormat, fusedFps_, fusedEncoder, fusedFile,
                                                     fusedMtx_, alignFilter, hwD2CMode_, cfg_.alpha, cfg_.depthMinM,
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

// ---------------------------------------------------------------------------
// Intrinsic JSON
// ---------------------------------------------------------------------------

void CaptureSession::writeIntrinsicJson() {
    if (!sensorInfo_.hasDepth)
        return;

    std::string intrinsicPath = baseName_ + "_depth_intrinsic_" + startTs_ + ".json";
    std::ofstream jf(intrinsicPath);
    if (jf.is_open()) {
        bool isPointDepth = pipeline_ && pipeline_->isPointCloudDepth();

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

        if (isPointDepth) {
            jf << "  \"lidar\": {\n"
               << "    \"type\": \"RS-AC1\",\n"
               << "    \"point_grid_width\": 96,\n"
               << "    \"point_grid_height\": 288,\n"
               << "    \"distance_min_m\": 0.2,\n"
               << "    \"distance_max_m\": 200.0,\n"
               << "    \"distance_resolution_m\": 0.005,\n"
               << "    \"vector_base\": 32768,\n"
               << "    \"point_fields\": \"x y z intensity ring timestamp\",\n"
               << "    \"point_type\": \"PointXYZIRT\"\n"
               << "  },\n";
        }

        jf << "  \"device\":\"" << safeName_ << "\"\n";
        jf << "}\n";
        std::cout << " Intrinsic: " << intrinsicPath << std::endl;
    }
}

// ---------------------------------------------------------------------------
// Viewer setup
// ---------------------------------------------------------------------------

void CaptureSession::setupViewerSlot(SDLViewer& viewer) {
    NioFormat depthSlotFmt = NioFormat::Y16;
    int depthSlotW = sensorInfo_.depthW;
    int depthSlotH = sensorInfo_.depthH;
    if (sensorInfo_.hasDepth && hwD2CMode_ && sensorInfo_.hasColor && !pipeline_->isPointCloudDepth()) {
        depthSlotW = sensorInfo_.colorW;
        depthSlotH = sensorInfo_.colorH;
    }
    bool hasPoint = pipeline_ && pipeline_->isPointCloudDepth();
    auto devInfo = device_->getDeviceInfo();
    std::string camType = devInfo.name;
    std::replace(camType.begin(), camType.end(), ' ', '_');
    viewerIdx_ = viewer.addDevice(safeName_, camType, devInfo.serialNumber, sensorInfo_.hasColor,
                                  sensorInfo_.colorFormat, sensorInfo_.colorW, sensorInfo_.colorH, sensorInfo_.hasDepth,
                                  depthSlotFmt, depthSlotW, depthSlotH, sensorInfo_.hasIR, sensorInfo_.irW,
                                  sensorInfo_.irH, sensorInfo_.hasIRLeft, sensorInfo_.irLW, sensorInfo_.irLH,
                                  sensorInfo_.hasIRRight, sensorInfo_.irRW, sensorInfo_.irRH, hasPoint);
}

// ---------------------------------------------------------------------------
// Consumer thread loops
// ---------------------------------------------------------------------------

void CaptureSession::videoConsumerLoop() {
    setThreadName(devId_ + "_vcons");
    NIO_LOG_DEBUG_S("Video consumer started: " << safeName_);

    while (consumersRunning_.load()) {
        std::shared_ptr<NioFrameSet> nioFs;
        if (!videoQueue_.pop(nioFs))
            continue;
        if (!nioFs)
            continue;

        if (canFuse_ && fusionTask_) {
            if (hwD2CMode_) {
                auto colorFrame = nioFs->getFrame(NioFrameType::COLOR);
                auto depthFrame = nioFs->getFrame(NioFrameType::DEPTH);
                if (colorFrame && depthFrame) {
                    fusionTask_->enqueueColor(colorFrame->rawData(), colorFrame->dataSize(), colorFrame->timestampUs);
                    fusionTask_->enqueueDepth(depthFrame->rawData(), depthFrame->dataSize(), depthFrame->timestampUs,
                                              depthFrame->depthScale);
                }
            } else {
                fusionTask_->enqueueNioFrameSet(nioFs);
            }
        }

        for (auto& fc : frameConsumers_)
            fc->consume(nioFs);
    }

    NIO_LOG_DEBUG_S("Video consumer stopped: " << safeName_);
}

// IMU callback: format NioImuSample → CSV lines and push to imuQueue_.
static void onImuSamples(const std::vector<NioImuSample>& samples, ImuFrameQueue& imuQueue,
                         const std::shared_ptr<SensorFiles>& sensorFiles) {
    auto nowMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count();

    for (const auto& s : samples) {
        std::ostringstream oss;
        oss << nowMs << "," << nioFrameTypeToStr(s.type) << "," << s.timestampUs << "," << std::fixed
            << std::setprecision(6) << s.x << "," << s.y << "," << s.z << "," << s.temperature << "\n";
        imuQueue.push(oss.str());
        std::lock_guard<std::mutex> lock(sensorFiles->countMtx);
        sensorFiles->frameCounts[s.type]++;
    }
}

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

    if (!noShow)
        setupViewerSlot(viewer);

    pipeline_->enableFrameSync();

    if (viewerIdx_ >= 0) {
        for (auto& fc : frameConsumers_)
            fc->setViewer(viewer_, viewerIdx_);
    }

    consumersRunning_ = true;
    videoConsumerThread_ = std::thread(&CaptureSession::videoConsumerLoop, this);

    bool ok = pipeline_->start([this](std::shared_ptr<NioFrameSet> nioFs) {
        if (nioFs)
            videoQueue_.push(std::move(nioFs));
    });

    if (!ok) {
        std::cerr << " Pipeline start failed for " << safeName_ << std::endl;
        NIO_LOG_ERROR_S("Pipeline start failed for " << safeName_);
        consumersRunning_ = false;
        videoQueue_.shutdown();
        if (videoConsumerThread_.joinable())
            videoConsumerThread_.join();
        pipeline_.reset();
        return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void CaptureSession::startImuPipeline() {
    if (!hasIMU())
        return;

    NIO_LOG_INFO_S("Starting IMU pipeline for " << safeName_);
    consumersRunning_ = true;
    imuConsumerThread_ = std::thread(&CaptureSession::imuConsumerLoop, this);

    pipeline_->startImu(
        [this](const std::vector<NioImuSample>& samples) { onImuSamples(samples, imuQueue_, sensorFiles_); });
}

void CaptureSession::stop() {
    consumersRunning_ = false;
    videoQueue_.shutdown();
    imuQueue_.shutdown();

    pipeline_->stop();
    pipeline_->stopImu();

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
    if (pcdTask_)
        pcdTask_->stop();

    // closeEncoders / closeFiles
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
    if (sf->pcdFile)
        sf->pcdFile->close();

    mjpgRes_.reset();

    std::cout << "Stopped: " << safeName_ << std::endl;
    NIO_LOG_INFO_S("Stopped device: " << safeName_);
}

// ---------------------------------------------------------------------------
// FPS reporting
// ---------------------------------------------------------------------------

uint64_t CaptureSession::getAndResetFrameCount(NioFrameType type) {
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
    std::map<NioFrameType, uint64_t> tempCounts;
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
            auto name = nioFrameTypeToStr(item.first);
            float rate = (reportDurationMs > 0) ? (item.second / (reportDurationMs / 1000.0f)) : 0.0f;
            std::cout << std::fixed << std::setprecision(1) << sep << name << "=" << rate;
            sep = ", ";
        }
        if (fusionTask_) {
            uint64_t fusedCount = fusionTask_->frameCount.exchange(0);
            float fusedRate = (reportDurationMs > 0) ? (fusedCount / (reportDurationMs / 1000.0f)) : 0.0f;
            std::cout << sep << "fused=" << std::fixed << std::setprecision(1) << fusedRate;
        }
    }
    std::cout << std::endl;
}

} // namespace nio
