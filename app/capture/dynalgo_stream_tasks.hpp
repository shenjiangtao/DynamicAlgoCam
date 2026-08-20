// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_stream_tasks.hpp — StreamTask subclasses for per-stream capture threads.
//
// EncodeStreamTask: H264 encode + file write for color/depth/IR streams.
// DepthRawTask: Y16 raw file write with per-frame header.
// FusionStreamTask: D2C alignment + alpha-blend + fused H264 encode.
// ImuStreamTask: IMU accel/gyro CSV line writer.

#pragma once

#include "dynalgo_color_convert.hpp"
#include "dynalgo_common.hpp"
#include "dynalgo_device.hpp"
#include "dynalgo_frame.hpp"
#include "dynalgo_h264_encoder.hpp"
#include "dynalgo_stream_io.hpp"
#include "dynalgo_thread.hpp"
#include "dynalgo_types.hpp"

#include <atomic>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace dynalgo {

// [类说明 / Class Description]
// 中文: 编码流任务 - H264编码+文件写入，用于彩色/深度/红外流
// English: EncodeStreamTask: H264 encode + file write for color/depth/IR streams
class EncodeStreamTask : public StreamTask
{
public:
    // [方法说明 / Method Description]
    // 中文: 构造函数，初始化名称和流编码器
    // English: Constructor, initializes name and stream encoder
    EncodeStreamTask(const std::string& name, std::shared_ptr<StreamEncoder> se);
    std::atomic<uint64_t> frameCount{ 0 };

protected:
    // [方法说明 / Method Description]
    // 中文: 处理帧数据，重写基类方法
    // English: Process frame data, override base class method
    void processFrame(const FrameBlob& blob) override;

private:
    std::shared_ptr<StreamEncoder> se_;
};

// [类说明 / Class Description]
// 中文: 深度原始数据写入任务，带帧头信息
// English: DepthRawTask: Y16 raw file write with per-frame header
class DepthRawTask : public StreamTask
{
public:
    // [方法说明 / Method Description]
    // 中文: 构造函数
    // English: Constructor
    DepthRawTask(const std::string& name, std::shared_ptr<std::ofstream> file, int width, int height, float depthScale);

protected:
    // [方法说明 / Method Description]
    // 中文: 处理帧数据，重写基类方法
    // English: Process frame data, override base class method
    void processFrame(const FrameBlob& blob) override;

private:
    std::shared_ptr<std::ofstream> file_;
    int width_;
    int height_;
    float depthScale_;
    std::mutex fileMtx_;
    uint64_t frameIdx_ = 0;
};

// [类说明 / Class Description]
// 中文: D2C融合任务 - 执行深度到彩色对齐、alpha混合和融合H264编码
// English: FusionStreamTask: D2C alignment + alpha-blend + fused H264 encode
class FusionStreamTask : public StreamTask
{
public:
    // [方法说明 / Method Description]
    // 中文: 构造函数，初始化颜色/深度缓冲、编码器和混合参数
    // English: Constructor, initializes color/depth buffers, encoder and blend params
    FusionStreamTask(const std::string& name, int colorW, int colorH, DynalgoFormat colorFormat, int fusedFps,
                     std::shared_ptr<H264Encoder> fusedEncoder, std::shared_ptr<std::ofstream> fusedFile,
                     std::mutex& fusedMtx, bool hwD2CMode, float alpha,
                     float depthMinM, float depthMaxM, float depthScale, std::shared_ptr<MjpgDecoderRes> mjpgRes);

    std::atomic<uint64_t> frameCount{ 0 };

    // [方法说明 / Method Description]
    // 中文: 入队Dynalgo帧集，用于D2C融合
    // English: Enqueue Dynalgo frame set for D2C fusion
    void enqueueDynalgoFrameSet(std::shared_ptr<DynalgoFrameSet> frameSet);
    // [方法说明 / Method Description]
    // 中文: 入队彩色帧数据
    // English: Enqueue color frame data
    void enqueueColor(const uint8_t* data, uint32_t size, uint64_t timestampUs);
    // [方法说明 / Method Description]
    // 中文: 入队深度帧数据
    // English: Enqueue depth frame data
    void enqueueDepth(const uint8_t* data, uint32_t size, uint64_t timestampUs, float depthScale = 1.0f);

protected:
    // [方法说明 / Method Description]
    // 中文: 融合任务帧处理，调用onIdle执行混合操作
    // English: FusionStreamTask frame processing, calls onIdle to execute blend operation
    void processFrame(const FrameBlob& blob) override;
    // [方法说明 / Method Description]
    // 中文: 空闲时执行混合（硬件D2C模式）
    // English: Execute blend on idle (HW D2C mode)
    void onIdle() override;
    // [方法说明 / Method Description]
    // 中文: 硬件D2C模式混合操作
    // English: Hardware D2C mode blend operation
    void onIdleHwD2C();
    // [方法说明 / Method Description]
    // 中文: 软件D2C模式混合操作
    // English: Software D2C mode blend operation
    void onIdleSwD2C();

private:
    // [方法说明 / Method Description]
    // 中文: 执行彩色深度混合
    // English: Execute color-depth blend
    void doBlend(const uint8_t* colorData, uint32_t colorSize, uint64_t colorTs, const uint8_t* depthData,
                 uint32_t depthSize, uint64_t depthTs, float depthScale);

    int colorW_;
    int colorH_;
    DynalgoFormat colorFormat_;
    std::shared_ptr<H264Encoder> fusedEncoder_;
    std::shared_ptr<std::ofstream> fusedFile_;
    std::mutex& fusedMtx_;
    bool hwD2CMode_;

    float alpha_;
    float depthMinM_;
    float depthMaxM_;
    float depthScale_;

    std::shared_ptr<MjpgDecoderRes> mjpgRes_;
    std::shared_ptr<std::vector<uint8_t>> colorRGBBuf_;
    std::shared_ptr<std::vector<uint8_t>> fusedRGBBuf_;

    struct FrameBuf
    {
        std::vector<uint8_t> data;
        uint32_t size = 0;
        uint64_t timestampUs = 0;
        float depthScale = 1.0f;
    };

    std::mutex colorMtx_;
    FrameBuf latestColor_;
    std::atomic<bool> colorReady_{ false };

    std::mutex depthMtx_;
    FrameBuf latestDepth_;
    std::atomic<bool> depthReady_{ false };

    std::mutex frameSetMtx_;
    std::shared_ptr<DynalgoFrameSet> latestFrameSet_;
    std::atomic<bool> frameSetReady_{ false };
};

// [类说明 / Class Description]
// 中文: IMU加速度/陀螺仪CSV行写入任务
// English: ImuStreamTask: IMU accel/gyro CSV line writer
class ImuStreamTask : public StreamTask
{
public:
    // [方法说明 / Method Description]
    // 中文: 构造函数，初始化IMU输出文件
    // English: Constructor, initializes IMU output file
    ImuStreamTask(const std::string& name, std::shared_ptr<std::ofstream> imuFile);

    // [方法说明 / Method Description]
    // 中文: 入队IMU CSV行数据
    // English: Enqueue IMU CSV line data
    void enqueueLine(std::string line);

protected:
    // [方法说明 / Method Description]
    // 中文: IMU任务帧处理，将CSV行写入IMU输出文件
    // English: IMU task frame processing, write CSV line to IMU output file
    void processFrame(const FrameBlob& blob) override;

private:
    std::shared_ptr<std::ofstream> imuFile_;
    std::mutex fileMtx_;
};

// [类说明 / Class Description]
// 中文: 统一点云写入器，由Mode驱动：Single=每帧独立PCD文件，Stream=连续.pcs流
// English: PcdWriterTask: unified point-cloud writer driven by Mode. Mode::Single → one standalone PCD v0.7 file per frame. Mode::Stream → continuous .pcs stream with trailing index
class PcdWriterTask : public StreamTask
{
public:
    enum class Mode { Single, Stream };

    // [方法说明 / Method Description]
    // 中文: 构造函数，支持单帧PCD和连续PCS流两种模式
    // English: Constructor, supports single PCD and continuous PCS stream modes
    PcdWriterTask(const std::string& name, const std::string& outputDir, const std::string& baseName, Mode mode);
    ~PcdWriterTask();
    std::atomic<uint64_t> frameCount{ 0 };

protected:
    // [方法说明 / Method Description]
    // 中文: 点云写入任务帧处理，支持单帧PCD和连续PCS流两种模式
    // English: Pointcloud writer task frame processing, supports single PCD and continuous PCS stream modes
    void processFrame(const FrameBlob& blob) override;
    // [方法说明 / Method Description]
    // 中文: 停止时写入流索引（仅Stream模式）
    // English: Write stream index on stop (Stream mode only)
    void onStop() override;

private:
    std::string outputDir_;
    std::string baseName_;
    Mode mode_;
    std::mutex fileMtx_;
    PcdStream pcdStream_;  // only used in Stream mode
};

// [类说明 / Class Description]
// 中文: PcdSingleTask/PcdStreamTask: 选择PcdWriterTask Mode的薄子类，保持3参数构造签名
// English: PcdSingleTask / PcdStreamTask: thin subclasses that select PcdWriterTask's Mode in their constructor — preserves the existing 3-arg constructor signature
class PcdSingleTask : public PcdWriterTask
{
public:
    // [方法说明 / Method Description]
    // 中文: 单帧PCD写入任务构造函数，选择Mode::Single模式
    // English: Single PCD writer task constructor, selects Mode::Single mode
    PcdSingleTask(const std::string& name, const std::string& outputDir, const std::string& baseName)
    : PcdWriterTask(name, outputDir, baseName, Mode::Single) {}
};

class PcdStreamTask : public PcdWriterTask
{
public:
    // [方法说明 / Method Description]
    // 中文: 连续PCS流写入任务构造函数，选择Mode::Stream模式
    // English: Continuous PCS stream writer task constructor, selects Mode::Stream mode
    PcdStreamTask(const std::string& name, const std::string& outputDir, const std::string& baseName)
    : PcdWriterTask(name, outputDir, baseName, Mode::Stream) {}
};

} // namespace dynalgo
