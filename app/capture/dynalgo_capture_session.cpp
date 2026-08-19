// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_capture_session.cpp — CaptureSession implementation.

#include "dynalgo_capture_session.hpp"
#include "dynalgo_log.hpp"
#include "dynalgo_sdl_viewer.hpp"
#include "../algo/dynalgo_engagement_loop.hpp"

namespace dynalgo {

CaptureSession::CaptureSession(std::shared_ptr<DynalgoDevice> device, std::shared_ptr<DynalgoPipeline> pipeline,
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
        DYNALGO_LOG_WARN_S("Timer sync failed for " << safeName_ << ": " << e.what());
    }

    if (device_->isGlobalTimestampSupported()) {
        try {
            device_->enableGlobalTimestamp(true);
        } catch (...) {
            DYNALGO_LOG_WARN_S("enableGlobalTimestamp threw for " << safeName_);
        }
    }

    sensorInfo_ = device_->setupPipeline(*pipeline_);
    depthScale_ = sensorInfo_.depthScale;

    if (cfg_.depthToPcd)
        pipeline_->setPointCloudEnabled(true);

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
    if (sensorInfo_.hasIR && sensorInfo_.irFormat != DynalgoFormat::UNKNOWN)
        createIREncoder(DynalgoFrameType::IR, "_ir_", sensorInfo_.irFormat, sensorInfo_.irW, sensorInfo_.irH,
                        sensorInfo_.irFps, ViewerChannel::IR);
    if (sensorInfo_.hasIRLeft && sensorInfo_.irLeftFormat != DynalgoFormat::UNKNOWN)
        createIREncoder(DynalgoFrameType::IR_LEFT, "_ir_left_", sensorInfo_.irLeftFormat, sensorInfo_.irLW,
                        sensorInfo_.irLH, sensorInfo_.irLFps, ViewerChannel::IR_LEFT);
    if (sensorInfo_.hasIRRight && sensorInfo_.irRightFormat != DynalgoFormat::UNKNOWN)
        createIREncoder(DynalgoFrameType::IR_RIGHT, "_ir_right_", sensorInfo_.irRightFormat, sensorInfo_.irRW,
                        sensorInfo_.irRH, sensorInfo_.irRFps, ViewerChannel::IR_RIGHT);
    createImuTask();
    createPcdTask();
}

void CaptureSession::createColorEncoder() {
    if (!sensorInfo_.hasColor || sensorInfo_.colorFormat == DynalgoFormat::UNKNOWN)
        return;
    auto sf = sensorFiles_;
    sf->color = createStreamEncoder((filePrefix_.empty()?baseName_:filePrefix_) + "_color_" + startTs_ + ".h264", sensorInfo_.colorFormat,
                                    sensorInfo_.colorW, sensorInfo_.colorH, sensorInfo_.colorFps, nullptr, false);
    auto task = std::make_shared<EncodeStreamTask>(devId_ + "_color_enc", sf->color);
    task->start();
    frameConsumers_.push_back(
        std::unique_ptr<FrameConsumer>(new ColorFrameConsumer(task, nullptr, -1, ViewerChannel::COLOR, sf)));
    DYNALGO_LOG_INFO_S("Color output: " << (filePrefix_.empty()?baseName_:filePrefix_) + "_color_" + startTs_ + ".h264"
                                    << " fmt=" << nioFormatToStr(sensorInfo_.colorFormat));
}

void CaptureSession::createDepthEncoder() {
    if (!sensorInfo_.hasDepth || sensorInfo_.depthFormat == DynalgoFormat::UNKNOWN)
        return;
    auto sf = sensorFiles_;
    sf->depth = createStreamEncoder((filePrefix_.empty()?baseName_:filePrefix_) + "_depth_" + startTs_ + ".h264", DynalgoFormat::RGB, sensorInfo_.depthW,
                                    sensorInfo_.depthH, sensorInfo_.depthFps, nullptr, false);
    sf->depthRawFile = std::make_shared<std::ofstream>((filePrefix_.empty()?baseName_:filePrefix_) + "_depth_raw_" + startTs_ + ".raw", std::ios::binary);
    auto encTask = std::make_shared<EncodeStreamTask>(devId_ + "_depth_enc", sf->depth);
    encTask->start();
    auto rawTask = std::make_shared<DepthRawTask>(devId_ + "_depth_raw", sf->depthRawFile, sensorInfo_.depthW,
                                                  sensorInfo_.depthH, depthScale_);
    rawTask->start();
    frameConsumers_.push_back(std::unique_ptr<FrameConsumer>(
        new DepthFrameConsumer(encTask, rawTask, nullptr, -1, ViewerChannel::DEPTH, sf, depthScale_, cfg_.depthMinM,
                               cfg_.depthMaxM, sensorInfo_.depthW, sensorInfo_.depthH)));
    DYNALGO_LOG_INFO_S("Depth output: " << (filePrefix_.empty()?baseName_:filePrefix_) + "_depth_" + startTs_ + ".h264" << " + raw (jet RGB encoded)");
}

void CaptureSession::createIREncoder(DynalgoFrameType type, const std::string& suffix, DynalgoFormat fmt, int w, int h, int fps,
                                     ViewerChannel ch) {
    auto sf = sensorFiles_;
    auto se = createStreamEncoder(baseName_ + suffix + startTs_ + ".h264", fmt, w, h, fps, nullptr, false);
    if (type == DynalgoFrameType::IR)
        sf->ir = se;
    else if (type == DynalgoFrameType::IR_LEFT)
        sf->irLeft = se;
    else
        sf->irRight = se;

    std::string taskId = devId_ + "_" +
                         (type == DynalgoFrameType::IR      ? "ir" :
                          type == DynalgoFrameType::IR_LEFT ? "irl" :
                                                          "irr") +
                         "_enc";
    auto task = std::make_shared<EncodeStreamTask>(taskId, se);
    task->start();
    frameConsumers_.push_back(std::unique_ptr<FrameConsumer>(new IRFrameConsumer(type, task, nullptr, -1, ch, sf)));
    DYNALGO_LOG_INFO_S("IR" << (type == DynalgoFrameType::IR_LEFT  ? " Left" :
                            type == DynalgoFrameType::IR_RIGHT ? " Right" :
                                                             "")
                        << " output: " << baseName_ + suffix + startTs_ + ".h264");
}

void CaptureSession::createImuTask() {
    if (!hasIMU())
        return;
    auto sf = sensorFiles_;
    sf->imuFile = std::make_shared<std::ofstream>((filePrefix_.empty()?baseName_:filePrefix_) + "_imu_" + startTs_ + ".txt");
    *sf->imuFile << "# host_ts_ms,type,device_ts_us,x,y,z,temperature\n";
    sf->imuFile->flush();
    imuTask_ = std::make_shared<ImuStreamTask>(devId_ + "_imu", sf->imuFile);
    imuTask_->start();
    DYNALGO_LOG_INFO_S("IMU output: " << (filePrefix_.empty()?baseName_:filePrefix_) + "_imu_" + startTs_ + ".txt");
}

void CaptureSession::createPcdTask() {
    if (!pipeline_ || !pipeline_->isPcdEnabled())
        return;

    std::string pcdDir = (filePrefix_.empty()?baseName_:filePrefix_) + "_pcd_" + startTs_;
    std::string pcdBase = safeName_;

    if (cfg_.pcdMode == PcdMode::Single) {
        pcdTask_ = std::make_shared<PcdSingleTask>(devId_ + "_pcd", pcdDir, pcdBase);
        pcdTask_->start();
        frameConsumers_.push_back(std::unique_ptr<FrameConsumer>(
            new PointcloudFrameConsumer(pcdTask_, sensorFiles_, sensorInfo_.depthIntrinsic, depthScale_)));
        DYNALGO_LOG_INFO_S("PCD point cloud output (single): " << pcdDir << "/");
    } else {
        pcdTask_ = std::make_shared<PcdStreamTask>(devId_ + "_pcd", pcdDir, pcdBase);
        pcdTask_->start();
        frameConsumers_.push_back(std::unique_ptr<FrameConsumer>(
            new PointcloudFrameConsumer(pcdTask_, sensorFiles_, sensorInfo_.depthIntrinsic, depthScale_)));
        DYNALGO_LOG_INFO_S("PCD point cloud output (stream): " << pcdDir << "/" << pcdBase << ".pcs");
    }
}

// ---------------------------------------------------------------------------
// Fusion
// ---------------------------------------------------------------------------

void CaptureSession::setupFusion() {
    canFuse_ = cfg_.enableFusion && sensorInfo_.hasColor && sensorInfo_.hasDepth;
    if (!canFuse_) {
        if (cfg_.enableFusion && !sensorInfo_.hasColor) {
            DYNALGO_LOG_WARN_S("D2C Fusion: skipped (no color sensor) for " << safeName_);
        } else if (cfg_.enableFusion && !sensorInfo_.hasDepth) {
            DYNALGO_LOG_WARN_S("D2C Fusion: skipped (no depth sensor) for " << safeName_);
        }
        return;
    }

    hwD2CMode_ = (pipeline_->getAlignMode() == DynalgoAlignMode::HW);

    fusedFps_ = std::min(sensorInfo_.colorFps, sensorInfo_.depthFps);

    std::string fusedPath = (filePrefix_.empty()?baseName_:filePrefix_) + "_d2c_fused_" + startTs_ + ".h264";
    auto fusedFile = std::make_shared<std::ofstream>(fusedPath, std::ios::binary);

    auto fusedEncoder = std::make_shared<H264Encoder>();
    if (!fusedEncoder->initRGB(sensorInfo_.colorW, sensorInfo_.colorH, fusedFps_)) {
        DYNALGO_LOG_ERROR_S("Failed to init fused H264 encoder for " << safeName_);
        canFuse_ = false;
        return;
    }

    mjpgRes_ = std::make_shared<MjpgDecoderRes>();
    mjpgRes_->init(sensorInfo_.colorW, sensorInfo_.colorH, sensorInfo_.colorFormat);
    fusionTask_ = std::make_shared<FusionStreamTask>(
        devId_ + "_fusion", sensorInfo_.colorW, sensorInfo_.colorH, sensorInfo_.colorFormat, fusedFps_, fusedEncoder,
        fusedFile, fusedMtx_, hwD2CMode_, cfg_.alpha, cfg_.depthMinM, cfg_.depthMaxM, depthScale_, mjpgRes_);
    fusionTask_->start();
    DYNALGO_LOG_INFO_S("D2C Fusion enabled: " << sensorInfo_.colorW << "x" << sensorInfo_.colorH << "@" << fusedFps_
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

    std::string intrinsicPath = (filePrefix_.empty()?baseName_:filePrefix_) + "_depth_intrinsic_" + startTs_ + ".json";
    std::ofstream jf(intrinsicPath);
    if (jf.is_open()) {
        bool isPointDepth = pipeline_ && pipeline_->isPcdEnabled();

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
        DYNALGO_LOG_INFO_S("Intrinsic saved: " << intrinsicPath);
    }
}

// ---------------------------------------------------------------------------
// Viewer setup
// ---------------------------------------------------------------------------

void CaptureSession::setupViewerSlot(SDLViewer& viewer) {
    DynalgoFormat depthSlotFmt = DynalgoFormat::Y16;
    int depthSlotW = sensorInfo_.depthW;
    int depthSlotH = sensorInfo_.depthH;
    if (sensorInfo_.hasDepth && sensorInfo_.hasColor) {
        depthSlotW = sensorInfo_.colorW;
        depthSlotH = sensorInfo_.colorH;
    }
    bool hasPoint = pipeline_ && pipeline_->isPcdEnabled();
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
    DYNALGO_LOG_DEBUG_S("Video consumer started: " << safeName_);

    while (consumersRunning_.load()) {
        std::shared_ptr<DynalgoFrameSet> nioFs;
        if (!videoQueue_.pop(nioFs, 5))
            continue;
        if (!nioFs)
            continue;

        if (canFuse_ && fusionTask_) {
            if (hwD2CMode_) {
                auto colorFrame = nioFs->getFrame(DynalgoFrameType::COLOR);
                auto depthFrame = nioFs->getFrame(DynalgoFrameType::DEPTH);
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

    DYNALGO_LOG_DEBUG_S("Video consumer stopped: " << safeName_);
}

// IMU callback: format DynalgoImuSample → CSV lines and push to imuQueue_.
static void onImuSamples(const std::vector<DynalgoImuSample>& samples, ImuFrameQueue& imuQueue,
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
    DYNALGO_LOG_DEBUG_S("IMU consumer started: " << safeName_);

    while (consumersRunning_.load()) {
        std::string line;
        if (!imuQueue_.pop(line))
            continue;
        if (imuTask_)
            imuTask_->enqueueLine(std::move(line));
    }

    DYNALGO_LOG_DEBUG_S("IMU consumer stopped: " << safeName_);
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

    bool ok = pipeline_->start([this](std::shared_ptr<DynalgoFrameSet> nioFs) {
        if (nioFs) {
            videoQueue_.push(std::move(nioFs));
            // Notify start-video caller that the pipeline is delivering frames.
            {
                std::lock_guard<std::mutex> lk(firstFrameMtx_);
                firstFrameSeen_ = true;
            }
            firstFrameCv_.notify_one();
        }
    });

    if (!ok) {
        DYNALGO_LOG_ERROR_S("Pipeline start failed for " << safeName_);
        consumersRunning_ = false;
        videoQueue_.shutdown();
        if (videoConsumerThread_.joinable())
            videoConsumerThread_.join();
        pipeline_.reset();
        return;
    }

    // Wait until the SDK callback delivers its first frame (event-driven),
    // or 2s safety-net timeout if the device is slow to initialize.
    // Replaces the prior unconditional std::this_thread::sleep_for(500ms).
    {
        std::unique_lock<std::mutex> lk(firstFrameMtx_);
        firstFrameCv_.wait_for(lk, std::chrono::seconds(2),
                               [this]() { return firstFrameSeen_; });
    }
    if (!firstFrameSeen_)
        DYNALGO_LOG_WARN_S("First-frame wait timed out (2s) for " << safeName_
                          << " — continuing; pipeline may still be warming up");
}

void CaptureSession::startImuPipeline() {
    if (!hasIMU())
        return;

    DYNALGO_LOG_INFO_S("Starting IMU pipeline for " << safeName_);
    consumersRunning_ = true;
    imuConsumerThread_ = std::thread(&CaptureSession::imuConsumerLoop, this);

    pipeline_->startImu(
        [this](const std::vector<DynalgoImuSample>& samples) { onImuSamples(samples, imuQueue_, sensorFiles_); });
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

    mjpgRes_.reset();

    DYNALGO_LOG_INFO_S("Stopped device: " << safeName_);
}

// ---------------------------------------------------------------------------
// FPS reporting
// ---------------------------------------------------------------------------

uint64_t CaptureSession::getAndResetFrameCount(DynalgoFrameType type) {
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
    std::map<DynalgoFrameType, uint64_t> tempCounts;
    {
        std::lock_guard<std::mutex> lock(sensorFiles_->countMtx);
        if (!sensorFiles_->frameCounts.empty()) {
            tempCounts = sensorFiles_->frameCounts;
            for (auto& item : sensorFiles_->frameCounts)
                item.second = 0;
        }
    }

    std::ostringstream fpsLine;
    fpsLine << "[" << safeName_ << "] ";
    if (tempCounts.empty() && !fusionTask_) {
        fpsLine << "Recording... waiting for frames";
    } else {
        fpsLine << "Recording... FPS: ";
        std::string sep;
        for (const auto& item : tempCounts) {
            auto name = nioFrameTypeToStr(item.first);
            float rate = (reportDurationMs > 0) ? (item.second / (reportDurationMs / 1000.0f)) : 0.0f;
            fpsLine << std::fixed << std::setprecision(1) << sep << name << "=" << rate;
            sep = ", ";
        }
        if (fusionTask_) {
            uint64_t fusedCount = fusionTask_->frameCount.exchange(0);
            float fusedRate = (reportDurationMs > 0) ? (fusedCount / (reportDurationMs / 1000.0f)) : 0.0f;
            fpsLine << sep << "fused=" << std::fixed << std::setprecision(1) << fusedRate;
        }
    }
    DYNALGO_LOG_INFO_S(fpsLine.str());
}

void CaptureSession::addFrameConsumer(std::unique_ptr<FrameConsumer> consumer)
{
    if (!consumer)
        return;
    frameConsumers_.push_back(std::move(consumer));
    DYNALGO_LOG_INFO_S("Added FrameConsumer to session " << safeName_ << " (total: " << frameConsumers_.size() << ")");
}

void CaptureSession::setEngagementLoop(DynalgoEngagementLoop* loop,
                                       DynalgoModelBackend* model,
                                       DynalgoActuator* actuator)
{
    // Delete any existing engagement loop components first
    delete engageLoop_;
    delete engageModelBackend_;
    delete engageActuator_;

    engageLoop_ = loop;
    engageModelBackend_ = model;
    engageActuator_ = actuator;
    DYNALGO_LOG_INFO_S("Engagement loop stored for session " << safeName_);
}

CaptureSession::~CaptureSession() {
    delete engageLoop_;
    delete engageModelBackend_;
    delete engageActuator_;
}

} // namespace dynalgo
