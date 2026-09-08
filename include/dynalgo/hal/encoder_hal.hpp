#pragma once

#include "hal_types.hpp"
#include <string>
#include <vector>
#include <functional>

namespace dynalgo::hal {

enum class EncoderCodec : uint32_t {
    H264_BASELINE = 0,
    H264_MAIN = 1,
    H264_HIGH = 2,
    H265_MAIN = 3,
    H265_MAIN10 = 4,
    MJPEG = 5,
    VP8 = 6,
    VP9 = 7,
    AV1 = 8,
};

enum class EncoderRateControl : uint32_t {
    CBR = 0,
    VBR = 1,
    CONSTANT_QP = 2,
    CAPPED_VBR = 3,
    LOW_LATENCY = 4,
};

enum class EncoderPreset : uint32_t {
    ULTRAFAST = 0,
    SUPERFAST = 1,
    VERYFAST = 2,
    FASTER = 3,
    FAST = 4,
    MEDIUM = 5,
    SLOW = 6,
    SLOWER = 7,
    VERYSLOW = 8,
    LOSSLESS = 9,
};

struct EncodeConfig {
    EncoderCodec codec = EncoderCodec::H264_HIGH;
    uint32_t width = 1920;
    uint32_t height = 1080;
    uint32_t fps = 30;
    uint32_t bitrate_bps = 8000000;
    EncoderRateControl rate_control = EncoderRateControl::CBR;
    uint32_t gop_size = 30;
    uint32_t idr_interval = 300;
    uint32_t num_b_frames = 0;
    uint32_t profile = 100;
    uint32_t level = 51;
    EncoderPreset preset = EncoderPreset::FAST;
    bool enable_sps_pps_per_idr = true;
    bool enable_aud = true;
    bool enable_sei = false;
    PixelFormat input_format = PixelFormat::NV12;
    uint32_t input_buffer_count = 4;
    void* vendor_params = nullptr;
    size_t vendor_params_size = 0;
};

struct BitstreamBuffer {
    BufferHandle buffer;
    uint32_t size = 0;
    uint32_t offset = 0;
    bool is_keyframe = false;
    bool is_config = false;
    uint64_t timestamp_ns = 0;
    uint32_t frame_id = 0;
    FenceHandle fence;
    std::vector<uint32_t> nal_sizes;
};

struct EncoderStats {
    uint64_t frames_encoded = 0;
    uint64_t bytes_produced = 0;
    uint64_t total_encode_time_us = 0;
    uint32_t current_qp = 0;
    uint32_t avg_qp = 0;
    float current_fps = 0.0f;
    uint64_t dropped_frames = 0;
    uint64_t error_frames = 0;
};

class IEncoderHAL {
public:
    virtual ~IEncoderHAL() = default;

    virtual ResultVoid initialize(const EncodeConfig& config) = 0;
    virtual ResultVoid start() = 0;
    virtual ResultVoid stop() = 0;
    virtual ResultVoid deinitialize() = 0;

    virtual ResultVoid encodeFrame(const FrameMetadata& input, BitstreamBuffer& output) = 0;
    virtual ResultVoid encodeFrameAsync(const FrameMetadata& input,
                                        std::function<void(Result<BitstreamBuffer>)> callback) = 0;

    virtual ResultVoid flush() = 0;
    virtual ResultVoid forceIDR() = 0;

    virtual ResultVoid setBitrate(uint32_t bitrate_bps) = 0;
    virtual ResultVoid setFramerate(uint32_t fps) = 0;
    virtual ResultVoid setGopSize(uint32_t gop_size) = 0;
    virtual ResultVoid setQP(uint32_t qp_i, uint32_t qp_p, uint32_t qp_b) = 0;

    virtual Result<EncoderStats> getStats() = 0;
    virtual ResultVoid resetStats() = 0;

    virtual Result<std::string> getName() const = 0;
    virtual Result<std::string> getVendor() const = 0;
    virtual Result<std::string> getVersion() const = 0;

    virtual ResultVoid vendorCommand(uint32_t cmd_id,
                                     const void* in_data, size_t in_size,
                                     void* out_data, size_t out_size) = 0;

    virtual bool isRunning() const = 0;
};

class EncoderHALFactory {
public:
    using CreateFunc = IEncoderHAL* (*)();
    using DestroyFunc = void (*)(IEncoderHAL*);

    static Result<IEncoderHAL*> create(const std::string& platform, const std::string& vendor);
    static Result<IEncoderHAL*> create(const EncodeConfig& config);
    static void destroy(IEncoderHAL* hal);

    static std::vector<std::string> getSupportedVendors(const std::string& platform);
    static void registerVendor(const std::string& platform, const std::string& vendor,
                               CreateFunc create, DestroyFunc destroy);
};

} // namespace dynalgo::hal