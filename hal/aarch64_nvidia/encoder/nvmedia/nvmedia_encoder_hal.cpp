/*
 * NvMedia Encoder HAL for NVIDIA Jetson/Drive platforms
 * Based on NVIDIA CEncConsumer reference implementation
 */
#include <dynalgo/hal/encoder_hal.hpp>
#include <dynalgo/hal/hal_types.hpp>

#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <chrono>
#include <queue>
#include <cstring>

#ifdef DYNALGO_HAVE_NVMEDIA
#include <nvmedia_iep.h>
#include <nvmedia_image.h>
#else
// Mock types for compilation on non-NVIDIA platforms
namespace nvmedia {
    using NvMediaStatus = int;
    constexpr int NVMEDIA_STATUS_OK = 0;
    constexpr int NVMEDIA_STATUS_ERROR = -1;
    constexpr int NVMEDIA_ENCODE_H264 = 0;
    constexpr int NVMEDIA_ENCODE_H265 = 1;
    constexpr int NVMEDIA_ENCODE_PROFILE_AUTOSELECT = 0;
    constexpr int NVMEDIA_ENCODE_LEVEL_AUTOSELECT = 0;
    constexpr int NVMEDIA_ENCODE_CONFIG_H264_ENABLE_OUTPUT_AUD = 1;
    constexpr int NVMEDIA_ENCODE_SPSPPS_REPEAT_INTRA_FRAMES = 1;
    constexpr int NVMEDIA_ENCODE_H264_ADAPTIVE_TRANSFORM_AUTOSELECT = 0;
    constexpr int NVMEDIA_ENCODE_H264_BDIRECT_MODE_DISABLE = 0;
    constexpr int NVMEDIA_ENCODE_H264_ENTROPY_CODING_MODE_CAVLC = 0;
    constexpr int NVMEDIA_ENC_PRESET_HP = 1;
    constexpr int NVMEDIA_ENCODE_PARAMS_RC_CONSTQP = 0;
    constexpr int NVMEDIA_ENCODE_PIC_TYPE_IDR = 1;
    constexpr int NVMEDIA_ENCODE_PIC_TYPE_AUTOSELECT = 0;
    constexpr int NVMEDIA_ENCODE_PIC_FLAG_OUTPUT_SPSPPS = 1;
    constexpr int NVMEDIA_ENCODER_INSTANCE_0 = 0;
    constexpr int NVMEDIA_SIGNALER = 0;
    constexpr int NVMEDIA_WAITER = 1;
    constexpr int NVMEDIA_EOFSYNCOBJ = 0;
    constexpr int NVMEDIA_PRESYNCOBJ = 1;
    constexpr int NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING = 0;
    constexpr int NVMEDIA_ENCODE_TIMEOUT_INFINITE = -1;
    constexpr int NVMEDIA_STATUS_NONE_PENDING = -2;
    constexpr int NVMEDIA_FALSE = 0;
    constexpr int NVMEDIA_TRUE = 1;
    
    struct NvMediaEncodeConfigH264 {
        int features = 0;
        int gopLength = 30;
        int idrPeriod = 300;
        int repeatSPSPPS = 0;
        int adaptiveTransformMode = 0;
        int bdirectMode = 0;
        int entropyCodingMode = 0;
        int encPreset = 0;
        struct { int rateControlMode = 0; struct { struct { int qpIntra = 32; int qpInterP = 35; int qpInterB = 25; } const_qp; } params; } rcParams;
        int numBFrames = 0;
    };
    struct NvMediaEncodeConfigH264VUIParams { int timingInfoPresentFlag = 0; };
    struct NvMediaEncodeInitializeParamsH264 {
        int profile = 0;
        int level = 0;
        int encodeHeight = 1080;
        int encodeWidth = 1920;
        int useBFramesAsRef = 0;
        int frameRateDen = 1;
        int frameRateNum = 30;
        int maxNumRefFrames = 1;
        int enableExternalMEHints = 0;
        int enableAllIFrames = 0;
    };
    struct NvMediaBitstreamBuffer { void* bitstream = nullptr; int bitstreamSize = 0; };
    struct NvMediaIEP { };
    using NvMediaIEPHandle = NvMediaIEP*;
    
    NvMediaIEPHandle NvMediaIEPCreate(int, void*, void*, int, int) { return nullptr; }
    NvMediaStatus NvMediaIEPDestroy(NvMediaIEPHandle) { return NVMEDIA_STATUS_OK; }
    NvMediaStatus NvMediaIEPSetConfiguration(NvMediaIEPHandle, void*) { return NVMEDIA_STATUS_OK; }
    NvMediaStatus NvMediaIEPFillNvSciBufAttrList(int, void*) { return NVMEDIA_STATUS_OK; }
    NvMediaStatus NvMediaIEPFillNvSciSyncAttrList(NvMediaIEPHandle, void*, int) { return NVMEDIA_STATUS_OK; }
    NvMediaStatus NvMediaIEPRegisterNvSciBufObj(NvMediaIEPHandle, void*) { return NVMEDIA_STATUS_OK; }
    NvMediaStatus NvMediaIEPUnregisterNvSciBufObj(NvMediaIEPHandle, void*) { return NVMEDIA_STATUS_OK; }
    NvMediaStatus NvMediaIEPRegisterNvSciSyncObj(NvMediaIEPHandle, int, void*) { return NVMEDIA_STATUS_OK; }
    NvMediaStatus NvMediaIEPUnregisterNvSciSyncObj(NvMediaIEPHandle, void*) { return NVMEDIA_STATUS_OK; }
    NvMediaStatus NvMediaIEPInsertPreNvSciSyncFence(NvMediaIEPHandle, void*) { return NVMEDIA_STATUS_OK; }
    NvMediaStatus NvMediaIEPSetNvSciSyncObjforEOF(NvMediaIEPHandle, void*) { return NVMEDIA_STATUS_OK; }
    NvMediaStatus NvMediaIEPFeedFrame(NvMediaIEPHandle, void*, void*, int) { return NVMEDIA_STATUS_OK; }
    NvMediaStatus NvMediaIEPGetEOFNvSciSyncFence(NvMediaIEPHandle, void*, void**) { return NVMEDIA_STATUS_OK; }
    NvMediaStatus NvMediaIEPBitsAvailable(NvMediaIEPHandle, int*, int, int) { return NVMEDIA_STATUS_OK; }
    NvMediaStatus NvMediaIEPGetBits(NvMediaIEPHandle, int*, int, void**, void*) { return NVMEDIA_STATUS_OK; }
}
#endif

namespace dynalgo {

class NvMediaEncoderHAL : public hal::IEncoderHAL {
public:
    NvMediaEncoderHAL() = default;
    ~NvMediaEncoderHAL() override { deinitialize(); }

    hal::ResultVoid initialize(const hal::EncodeConfig& config) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_initialized) return hal::ResultVoid::err(hal::ErrorCode::ALREADY_INITIALIZED);
        
        m_config = config;
        
#ifdef DYNALGO_HAVE_NVMEDIA
        // Configure encoder
        memset(&m_encode_config, 0, sizeof(m_encode_config));
        m_encode_config.h264VUIParameters = new NvMediaEncodeConfigH264VUIParams();
        m_encode_config.h264VUIParameters->timingInfoPresentFlag = 1;
        
        m_encode_config.features = NVMEDIA_ENCODE_CONFIG_H264_ENABLE_OUTPUT_AUD;
        m_encode_config.gopLength = config.gop_size;
        m_encode_config.idrPeriod = config.idrPeriod;
        m_encode_config.repeatSPSPPS = NVMEDIA_ENCODE_SPSPPS_REPEAT_INTRA_FRAMES;
        m_encode_config.adaptiveTransformMode = NVMEDIA_ENCODE_H264_ADAPTIVE_TRANSFORM_AUTOSELECT;
        m_encode_config.bdirectMode = NVMEDIA_ENCODE_H264_BDIRECT_MODE_DISABLE;
        m_encode_config.entropyCodingMode = NVMEDIA_ENCODE_H264_ENTROPY_CODING_MODE_CAVLC;
        m_encode_config.encPreset = NVMEDIA_ENC_PRESET_HP;
        
        m_encode_config.rcParams.rateControlMode = NVMEDIA_ENCODE_PARAMS_RC_CONSTQP;
        m_encode_config.rcParams.params.const_qp.constQP.qpIntra = 32;
        m_encode_config.rcParams.params.const_qp.constQP.qpInterP = 35;
        m_encode_config.rcParams.params.const_qp.constQP.qpInterB = 25;
        m_encode_config.rcParams.numBFrames = 0;
        
        NvMediaEncodeInitializeParamsH264 init_params = {0};
        init_params.profile = NVMEDIA_ENCODE_PROFILE_AUTOSELECT;
        init_params.level = NVMEDIA_ENCODE_LEVEL_AUTOSELECT;
        init_params.encodeHeight = config.height;
        init_params.encodeWidth = config.width;
        init_params.useBFramesAsRef = 0;
        init_params.frameRateDen = 1;
        init_params.frameRateNum = config.fps;
        init_params.maxNumRefFrames = 1;
        init_params.enableExternalMEHints = NVMEDIA_FALSE;
        init_params.enableAllIFrames = NVMEDIA_FALSE;
        
        m_iep = NvMediaIEPCreate(NVMEDIA_ENCODE_H264, &init_params, nullptr, 0, NVMEDIA_ENCODER_INSTANCE_0);
        if (!m_iep) return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
        
        nvmedia::NvMediaStatus status = NvMediaIEPSetConfiguration(m_iep, &m_encode_config);
        if (status != NVMEDIA_STATUS_OK) return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
        
        // Fill buffer attributes
        void* buf_attr_list = nullptr; // Would be created from reconciled NvSciBufAttrList
        status = NvMediaIEPFillNvSciBufAttrList(NVMEDIA_ENCODER_INSTANCE_0, buf_attr_list);
        if (status != NVMEDIA_STATUS_OK) return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
        
        // Fill sync attributes
        // NvMediaIEPFillNvSciSyncAttrList(m_iep, signaler_attr_list, NVMEDIA_SIGNALER);
        // NvMediaIEPFillNvSciSyncAttrList(m_iep, waiter_attr_list, NVMEDIA_WAITER);
#endif
        
        m_initialized = true;
        return hal::ResultVoid::ok();
    }

    hal::ResultVoid start() override {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_initialized) return hal::ResultVoid::err(hal::ErrorCode::NOT_INITIALIZED);
        if (m_running) return hal::ResultVoid::ok();
        
#ifdef DYNALGO_HAVE_NVMEDIA
        // Register EOF sync object
        // NvMediaIEPSetNvSciSyncObjforEOF(m_iep, m_eof_sync_obj);
#endif
        
        m_running = true;
        m_encode_thread = std::thread(&NvMediaEncoderHAL::encodeLoop, this);
        return hal::ResultVoid::ok();
    }

    hal::ResultVoid stop() override {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_running) return hal::ResultVoid::ok();
        
        m_running = false;
        if (m_encode_thread.joinable()) m_encode_thread.join();
        return hal::ResultVoid::ok();
    }

    void encodeLoop() {
        while (m_running.load()) {
            // In a real implementation, this would wait for input frames and encode them
            // For now, just sleep to simulate work
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
    }

    hal::ResultVoid deinitialize() override {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_initialized) return hal::ResultVoid::ok();
        
        if (m_running) stop();
        
#ifdef DYNALGO_HAVE_NVMEDIA
        if (m_iep) {
            // Unregister sync objects
            // NvMediaIEPUnregisterNvSciSyncObj(m_iep, m_eof_sync_obj);
            // for each waiter sync obj...
            
            // Unregister buffers
            for (auto buf : m_buf_objs) {
                NvMediaIEPUnregisterNvSciBufObj(m_iep, buf);
            }
            
            if (m_encode_config.h264VUIParameters) {
                delete m_encode_config.h264VUIParameters;
                m_encode_config.h264VUIParameters = nullptr;
            }
            
            NvMediaIEPDestroy(m_iep);
            m_iep = nullptr;
        }
#endif
        
        m_initialized = false;
        return hal::ResultVoid::ok();
    }

    hal::ResultVoid encodeFrame(const hal::FrameMetadata& input, hal::BitstreamBuffer& output) override {
        if (!m_initialized) return hal::ResultVoid::err(hal::ErrorCode::NOT_INITIALIZED);
        
#ifdef DYNALGO_HAVE_NVMEDIA
        // Insert pre-fence if provided
        if (input.acquire_fence.is_valid()) {
            nvmedia::NvMediaStatus status = NvMediaIEPInsertPreNvSciSyncFence(m_iep, &input.acquire_fence);
            if (status != NVMEDIA_STATUS_OK) return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
        }
        
        // Get NvSciBufObj from input buffer
        void* nvsci_buf = reinterpret_cast<void*>(input.buffer.handle);
        
        // Register buffer if not already registered
        // NvMediaIEPRegisterNvSciBufObj(m_iep, nvsci_buf);
        
        // Encode frame
        NvMediaEncodePicParamsH264 pic_params = {0};
        pic_params.pictureType = (m_frame_count == 0) ? NVMEDIA_ENCODE_PIC_TYPE_IDR : NVMEDIA_ENCODE_PIC_TYPE_AUTOSELECT;
        pic_params.encodePicFlags = NVMEDIA_ENCODE_PIC_FLAG_OUTPUT_SPSPPS;
        pic_params.nextBFrames = 0;
        
        nvmedia::NvMediaStatus status = NvMediaIEPFeedFrame(m_iep, nvsci_buf, &pic_params, NVMEDIA_ENCODER_INSTANCE_0);
        if (status != NVMEDIA_STATUS_OK) return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
        
        // Get EOF fence
        void* eof_fence = nullptr;
        nvmedia::NvMediaStatus fence_status = NvMediaIEPGetEOFNvSciSyncFence(m_iep, m_eof_sync_obj, &eof_fence);
        if (fence_status != NVMEDIA_STATUS_OK) return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
        
        // Get encoded bits
        int bytes_available = 0;
        status = NvMediaIEPBitsAvailable(m_iep, &bytes_available, NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING, NVMEDIA_ENCODE_TIMEOUT_INFINITE);
        if (status != NVMEDIA_STATUS_OK && status != NVMEDIA_STATUS_NONE_PENDING) {
            return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
        }
        
        if (status == NVMEDIA_STATUS_OK) {
            std::vector<uint8_t> bitstream(bytes_available);
            NvMediaBitstreamBuffer bs = {0};
            bs.bitstream = bitstream.data();
            bs.bitstreamSize = bytes_available;
            
            int bytes_written = 0;
            status = NvMediaIEPGetBits(m_iep, &bytes_written, 1, &bs, nullptr);
            if (status != NVMEDIA_STATUS_OK && status != NVMEDIA_STATUS_NONE_PENDING) {
                return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
            }
            
            // Copy to output buffer
            if (output.buffer.is_valid()) {
                // Copy bitstream to output buffer
                memcpy(reinterpret_cast<void*>(output.buffer.handle), bitstream.data(), bytes_written);
                output.size = bytes_written;
                output.is_keyframe = (m_frame_count == 0);
                output.timestamp_ns = dynalgo::hal::now_ns();
                output.frame_id = m_frame_count++;
            }
        }
        
        m_frame_count++;
#endif
        
        return hal::ResultVoid::ok();
    }

    hal::ResultVoid encodeFrameAsync(const hal::FrameMetadata& input, std::function<void(hal::Result<hal::BitstreamBuffer>)> callback) override {
        // For async, queue the frame and call callback when done
        auto result = encodeFrame(input, m_async_output);
        if (result.is_ok()) {
            callback(hal::Result<hal::BitstreamBuffer>::ok(m_async_output));
        } else {
            callback(hal::Result<hal::BitstreamBuffer>::err(result.error()));
        }
        return hal::ResultVoid::ok();
    }

    hal::ResultVoid flush() override {
        // Wait for all pending frames to complete
        return hal::ResultVoid::ok();
    }

    hal::ResultVoid forceIDR() override {
#ifdef DYNALGO_HAVE_NVMEDIA
        // Force next frame to be IDR
#endif
        return hal::ResultVoid::ok();
    }

    hal::ResultVoid setBitrate(uint32_t bitrate_bps) override {
#ifdef DYNALGO_HAVE_NVMEDIA
        m_encode_config.rcParams.params.const_qp.constQP.qpIntra = 32; // Simplified
        NvMediaIEPSetConfiguration(m_iep, &m_encode_config);
#endif
        return hal::ResultVoid::ok();
    }

    hal::ResultVoid setFramerate(uint32_t fps) override { return hal::ResultVoid::ok(); }
    hal::ResultVoid setGopSize(uint32_t gop_size) override { return hal::ResultVoid::ok(); }
    hal::ResultVoid setQP(uint32_t qp_i, uint32_t qp_p, uint32_t qp_b) override { return hal::ResultVoid::ok(); }

    hal::Result<hal::EncoderStats> getStats() override {
        hal::EncoderStats stats;
        stats.frames_encoded = m_frame_count;
        stats.bytes_produced = m_total_bytes;
        stats.current_fps = 0.0f;
        stats.current_qp = 0;
        stats.avg_qp = 0;
        return hal::Result<hal::EncoderStats>::ok(stats);
    }

    hal::ResultVoid resetStats() override { return hal::ResultVoid::ok(); }

    hal::Result<std::string> getName() const override { return hal::Result<std::string>::ok("NvMedia Encoder HAL"); }
    hal::Result<std::string> getVendor() const override { return hal::Result<std::string>::ok("nvidia"); }
    hal::Result<std::string> getVersion() const override { return hal::Result<std::string>::ok("1.0.0"); }
    hal::ResultVoid vendorCommand(uint32_t, const void*, size_t, void*, size_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_SUPPORTED); }
    bool isRunning() const override { return m_running; }

private:
    std::mutex m_mutex;
    hal::EncodeConfig m_config;
    
#ifdef DYNALGO_HAVE_NVMEDIA
    nvmedia::NvMediaIEPHandle m_iep = nullptr;
    nvmedia::NvMediaEncodeConfigH264 m_encode_config;
    void* m_eof_sync_obj = nullptr;
    std::vector<void*> m_buf_objs;
    void* m_async_output = nullptr;
#endif
    
    std::thread m_encode_thread;
    std::atomic<bool> m_initialized{false};
    std::atomic<bool> m_running{false};
    uint64_t m_frame_count = 0;
    uint64_t m_total_bytes = 0;
    hal::BitstreamBuffer m_async_output;
};

} // namespace dynalgo

extern "C" {
dynalgo::hal::IEncoderHAL* dynalgo_hal_encoder_create() { return new dynalgo::NvMediaEncoderHAL(); }
void dynalgo_hal_encoder_destroy(dynalgo::hal::IEncoderHAL* ptr) { delete ptr; }
}