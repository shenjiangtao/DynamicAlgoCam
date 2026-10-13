/*
 * Copyright (c) 2024, DynamicAlgoCam. All rights reserved.
 *
 * NvSIPL H.265 Encoder Consumer for NVIDIA Jetson/Drive platforms.
 * Based on NVIDIA CSiplCamera reference implementation (nvsipl-multicast-6.0.8.0/CEncConsumer.cpp).
 */

#ifndef NVSIPL_ENC_CONSUMER_HPP
#define NVSIPL_ENC_CONSUMER_HPP

#ifdef DYNALGO_HAVE_NVSIPL

#include <NvSIPLClient.hpp>
#include <nvmedia_iep.h>
#include <nvscibuf.h>
#include <nvscisync.h>

#include <memory>
#include <string>
#include <vector>

namespace dynalgo {
namespace nvsipl {

// Use NVIDIA's SIPLStatus from nvsipl namespace
using ::nvsipl::SIPLStatus;
using ::nvsipl::NVSIPL_STATUS_OK;
using ::nvsipl::NVSIPL_STATUS_BAD_ARGUMENT;
using ::nvsipl::NVSIPL_STATUS_ERROR;
using ::nvsipl::NVSIPL_STATUS_TIMED_OUT;
using ::nvsipl::NVSIPL_STATUS_EOF;
using ::nvsipl::NVSIPL_STATUS_OUT_OF_MEMORY;
using ::nvsipl::NVSIPL_STATUS_NOT_INITIALIZED;

class NvSIPLEncConsumer {
public:
    struct EncoderConfig {
        std::string output_file;
        uint32_t width = 3840;
        uint32_t height = 2160;
        uint32_t fps = 30;
        uint32_t bitrate = 10000000;  // 10 Mbps default
        uint32_t gop_length = 30;
        uint32_t idr_period = 300;
        uint32_t qp_intra = 32;
        uint32_t qp_inter_p = 35;
        uint32_t qp_inter_b = 25;
        bool enable_file_dump = false;
        uint32_t start_frame = 0;
        uint32_t end_frame = UINT32_MAX;
    };

    NvSIPLEncConsumer(uint32_t sensor_id, const EncoderConfig& config);
    ~NvSIPLEncConsumer();

    // Initialize encoder with NvSciBuf attributes from upstream
    SIPLStatus Initialize(NvSciBufAttrList buf_attr_list);

    // Encode a single frame from NvSciBufObj (zero-copy)
    SIPLStatus EncodeFrame(NvSciBufObj sci_buf_obj, 
                           uint8_t** output_buffer,
                           size_t* output_size,
                           NvSciSyncFence* post_fence);

    // Set up buffer attributes for NvSciBuf reconciliation
    static SIPLStatus FillNvSciBufAttrList(NvSciBufAttrList& buf_attr_list);

    // Set up sync attributes for NvSciSync
    SIPLStatus FillNvSciSyncAttrList(NvSciSyncAttrList& signaler_attr_list,
                                      NvSciSyncAttrList& waiter_attr_list);

    // Register NvSciBufObj with encoder
    SIPLStatus RegisterNvSciBufObj(NvSciBufObj buf_obj);

    // Register sync objects
    SIPLStatus RegisterSignalSyncObj(NvSciSyncObj signal_sync_obj);
    SIPLStatus RegisterWaiterSyncObj(NvSciSyncObj waiter_sync_obj);

    // Insert pre-fence
    SIPLStatus InsertPreFence(NvSciSyncFence& pre_fence);

    // Set EOF sync object
    SIPLStatus SetEofSyncObj();

    // Cleanup sync objects
    SIPLStatus UnregisterSyncObjs();

    // Check if encoder is initialized
    bool IsInitialized() const { return m_initialized; }

    // Get sensor ID
    uint32_t GetSensorId() const { return m_sensor_id; }

private:
    // Initialize NvMediaIEP encoder
    SIPLStatus InitEncoder(NvSciBufAttrList buf_attr_list);

    // Configure encoder parameters (H.265)
    SIPLStatus ConfigureEncoder();

    // Destroy encoder
    void DestroyEncoder();

    struct DestroyNvMediaIEP {
        void operator()(NvMediaIEP* p) const {
            if (p) NvMediaIEPDestroy(p);
        }
    };

    uint32_t m_sensor_id;
    EncoderConfig m_config;
    std::unique_ptr<NvMediaIEP, DestroyNvMediaIEP> m_encoder{nullptr};
    std::vector<NvSciBufObj> m_sci_buf_objs;
    NvSciSyncObj m_signal_sync_obj = nullptr;
    FILE* m_output_file = nullptr;
    NvMediaEncodeConfigH265 m_encode_config_h265{};
    uint8_t* m_encoded_buffer = nullptr;
    size_t m_encoded_bytes = 0;
    uint32_t m_frame_num = 0;
    bool m_initialized = false;
    const uint32_t MAX_NUM_PACKETS = 8;
};

} // namespace nvsipl
} // namespace dynalgo

#else // DYNALGO_HAVE_NVSIPL

// Mock for compilation without NvSIPL
namespace dynalgo {
namespace nvsipl {
    using SIPLStatus = int;
    constexpr int NVSIPL_STATUS_OK = 0;
    constexpr int NVSIPL_STATUS_BAD_ARGUMENT = -1;
    constexpr int NVSIPL_STATUS_ERROR = -2;
    constexpr int NVSIPL_STATUS_TIMED_OUT = -3;
    constexpr int NVSIPL_STATUS_EOF = -4;
    constexpr int NVSIPL_STATUS_OUT_OF_MEMORY = -5;
    constexpr int NVSIPL_STATUS_NOT_INITIALIZED = -6;

    class NvSIPLEncConsumer {
    public:
        struct EncoderConfig {
            std::string output_file;
            uint32_t width = 3840;
            uint32_t height = 2160;
            uint32_t fps = 30;
            uint32_t bitrate = 10000000;
            uint32_t gop_length = 30;
            uint32_t idr_period = 300;
            uint32_t qp_intra = 32;
            uint32_t qp_inter_p = 35;
            uint32_t qp_inter_b = 25;
            bool enable_file_dump = false;
            uint32_t start_frame = 0;
            uint32_t end_frame = UINT32_MAX;
        };

        NvSIPLEncConsumer(uint32_t sensor_id, const EncoderConfig& config) {}
        ~NvSIPLEncConsumer() {}
        SIPLStatus Initialize(void*) { return NVSIPL_STATUS_OK; }
        SIPLStatus EncodeFrame(void*, void**, size_t*, void*) { return NVSIPL_STATUS_OK; }
        static SIPLStatus FillNvSciBufAttrList(void*) { return NVSIPL_STATUS_OK; }
        SIPLStatus FillNvSciSyncAttrList(void*, void*) { return NVSIPL_STATUS_OK; }
        SIPLStatus RegisterNvSciBufObj(void*) { return NVSIPL_STATUS_OK; }
        SIPLStatus RegisterSignalSyncObj(void*) { return NVSIPL_STATUS_OK; }
        SIPLStatus RegisterWaiterSyncObj(void*) { return NVSIPL_STATUS_OK; }
        SIPLStatus InsertPreFence(void*) { return NVSIPL_STATUS_OK; }
        SIPLStatus SetEofSyncObj() { return NVSIPL_STATUS_OK; }
        SIPLStatus UnregisterSyncObjs() { return NVSIPL_STATUS_OK; }
        bool IsInitialized() const { return false; }
        uint32_t GetSensorId() const { return 0; }
    };

} // namespace nvsipl
} // namespace dynalgo

#endif // DYNALGO_HAVE_NVSIPL

#endif // NVSIPL_ENC_CONSUMER_HPP