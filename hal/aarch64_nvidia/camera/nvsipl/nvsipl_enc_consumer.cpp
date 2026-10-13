/*
 * Copyright (c) 2024, DynamicAlgoCam. All rights reserved.
 *
 * NvSIPL H.265 Encoder Consumer Implementation
 * Based on NVIDIA CSiplCamera reference implementation (nvsipl-multicast-6.0.8.0/CEncConsumer.cpp).
 */

#include "nvsipl_enc_consumer.hpp"

#ifdef DYNALGO_HAVE_NVSIPL

#include <cstdio>
#include <cstring>
#include <iostream>

#define LOG_DBG(fmt, ...) fprintf(stderr, "[NVPL_ENC_DBG] " fmt "\n", ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) fprintf(stderr, "[NVPL_ENC_INFO] " fmt "\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) fprintf(stderr, "[NVPL_ENC_WARN] " fmt "\n", ##__VA_ARGS__)
#define LOG_ERR(fmt, ...) fprintf(stderr, "[NVPL_ENC_ERR] " fmt "\n", ##__VA_ARGS__)

namespace dynalgo {
namespace nvsipl {

NvSIPLEncConsumer::NvSIPLEncConsumer(uint32_t sensor_id, const EncoderConfig& config)
    : m_sensor_id(sensor_id), m_config(config) {
    m_sci_buf_objs.resize(MAX_NUM_PACKETS, nullptr);
    LOG_INFO("NvSIPLEncConsumer created for sensor %u", m_sensor_id);
}

NvSIPLEncConsumer::~NvSIPLEncConsumer() {
    LOG_INFO("NvSIPLEncConsumer destroyed for sensor %u", m_sensor_id);
    
    // Cleanup NvSciBufObjs
    for (NvSciBufObj buf_obj : m_sci_buf_objs) {
        if (buf_obj && m_encoder) {
            NvMediaIEPUnregisterNvSciBufObj(m_encoder.get(), buf_obj);
        }
    }
    
    UnregisterSyncObjs();
    DestroyEncoder();
    
    if (m_output_file) {
        fflush(m_output_file);
        fclose(m_output_file);
        m_output_file = nullptr;
    }
    
    if (m_encoded_buffer) {
        delete[] m_encoded_buffer;
        m_encoded_buffer = nullptr;
    }
}

SIPLStatus NvSIPLEncConsumer::Initialize(NvSciBufAttrList buf_attr_list) {
    if (m_initialized) {
        LOG_WARN("Encoder already initialized for sensor %u", m_sensor_id);
        return NVSIPL_STATUS_OK;
    }
    
    SIPLStatus status = InitEncoder(buf_attr_list);
    if (status != NVSIPL_STATUS_OK) {
        LOG_ERR("Failed to initialize encoder for sensor %u", m_sensor_id);
        return status;
    }
    
    m_initialized = true;
    LOG_INFO("Encoder initialized for sensor %u: %ux%u @ %ufps", 
             m_sensor_id, m_config.width, m_config.height, m_config.fps);
    return NVSIPL_STATUS_OK;
}

SIPLStatus NvSIPLEncConsumer::InitEncoder(NvSciBufAttrList buf_attr_list) {
    NvMediaEncodeInitializeParamsH265 encoder_init_params;
    memset(&encoder_init_params, 0, sizeof(encoder_init_params));
    
    encoder_init_params.profile = NVMEDIA_ENCODE_PROFILE_AUTOSELECT;
    encoder_init_params.level = NVMEDIA_ENCODE_LEVEL_AUTOSELECT;
    encoder_init_params.encodeWidth = m_config.width;
    encoder_init_params.encodeHeight = m_config.height;
    encoder_init_params.useBFramesAsRef = 0;
    encoder_init_params.frameRateDen = 1;
    encoder_init_params.frameRateNum = m_config.fps;
    encoder_init_params.maxNumRefFrames = 1;
    encoder_init_params.enableExternalMEHints = NVMEDIA_FALSE;
    encoder_init_params.enableAllIFrames = NVMEDIA_FALSE;
    
    m_encoder.reset(NvMediaIEPCreate(NVMEDIA_IMAGE_ENCODE_HEVC,  // H.265/HEVC codec
                                     &encoder_init_params,       // init params
                                     buf_attr_list,              // reconciled attr list
                                     0,                          // maxOutputBuffering
                                     NVMEDIA_ENCODER_INSTANCE_0));
    
    if (!m_encoder) {
        LOG_ERR("NvMediaIEPCreate failed for H.265 encoder");
        return NVSIPL_STATUS_ERROR;
    }
    
    // Open output file if dumping enabled
    if (m_config.enable_file_dump && !m_config.output_file.empty()) {
        m_output_file = fopen(m_config.output_file.c_str(), "wb");
        if (!m_output_file) {
            LOG_WARN("Failed to open output file: %s", m_config.output_file.c_str());
        } else {
            LOG_INFO("Encoder output file: %s", m_config.output_file.c_str());
        }
    }
    
    // Configure encoder parameters
    return ConfigureEncoder();
}

SIPLStatus NvSIPLEncConsumer::ConfigureEncoder() {
    memset(&m_encode_config_h265, 0, sizeof(NvMediaEncodeConfigH265));
    
    m_encode_config_h265.h265VUIParameters = 
        (NvMediaEncodeConfigH265VUIParams*)calloc(1, sizeof(NvMediaEncodeConfigH265VUIParams));
    if (!m_encode_config_h265.h265VUIParameters) {
        LOG_ERR("Failed to allocate H.265 VUI parameters");
        return NVSIPL_STATUS_OUT_OF_MEMORY;
    }
    m_encode_config_h265.h265VUIParameters->vuiTimingInfoPresentFlag = true;
    
    // H.265 encoder configuration
    m_encode_config_h265.features = NVMEDIA_ENCODE_CONFIG_H265_ENABLE_OUTPUT_AUD;
    m_encode_config_h265.gopLength = m_config.gop_length;
    m_encode_config_h265.idrPeriod = m_config.idr_period;
    m_encode_config_h265.repeatSPSPPS = NVMEDIA_ENCODE_SPSPPS_REPEAT_INTRA_FRAMES;
    m_encode_config_h265.encPreset = NVMEDIA_ENC_PRESET_HP;  // High Performance preset
    
    // Rate control: Constant QP
    m_encode_config_h265.rcParams.rateControlMode = NVMEDIA_ENCODE_PARAMS_RC_CONSTQP;
    m_encode_config_h265.rcParams.params.const_qp.constQP.qpIntra = m_config.qp_intra;
    m_encode_config_h265.rcParams.params.const_qp.constQP.qpInterP = m_config.qp_inter_p;
    m_encode_config_h265.rcParams.params.const_qp.constQP.qpInterB = m_config.qp_inter_b;
    m_encode_config_h265.rcParams.numBFrames = 0;  // No B-frames for low latency
    
    NvMediaStatus nvm_status = NvMediaIEPSetConfiguration(m_encoder.get(), &m_encode_config_h265);
    if (nvm_status != NVMEDIA_STATUS_OK) {
        LOG_ERR("NvMediaIEPSetConfiguration failed for H.265: %d", nvm_status);
        return NVSIPL_STATUS_ERROR;
    }
    
    LOG_INFO("H.265 encoder configured: GOP=%u, IDR=%u, QP(I/P/B)=%u/%u/%u, fps=%u",
             m_config.gop_length, m_config.idr_period,
             m_config.qp_intra, m_config.qp_inter_p, m_config.qp_inter_b,
             m_config.fps);
    
    return NVSIPL_STATUS_OK;
}

void NvSIPLEncConsumer::DestroyEncoder() {
    if (m_encode_config_h265.h265VUIParameters) {
        free(m_encode_config_h265.h265VUIParameters);
        m_encode_config_h265.h265VUIParameters = nullptr;
    }
    m_encoder.reset();
}

SIPLStatus NvSIPLEncConsumer::FillNvSciBufAttrList(NvSciBufAttrList& buf_attr_list) {
    NvMediaStatus nvm_status = NvMediaIEPFillNvSciBufAttrList(NVMEDIA_ENCODER_INSTANCE_0, buf_attr_list);
    if (nvm_status != NVMEDIA_STATUS_OK) {
        LOG_ERR("NvMediaIEPFillNvSciBufAttrList failed: %d", nvm_status);
        return NVSIPL_STATUS_ERROR;
    }
    
    // Set buffer attributes for encoder input (read-only access to YUV frames)
    NvSciBufAttrValAccessPerm access_perm = NvSciBufAccessPerm_Readonly;
    NvSciBufType buf_type = NvSciBufType_Image;
    bool need_cpu_access = false;
    bool is_enable_cpu_cache = false;
    
    NvSciBufAttrKeyValuePair attributes[] = {
        { NvSciBufGeneralAttrKey_RequiredPerm, &access_perm, sizeof(access_perm) },
        { NvSciBufGeneralAttrKey_Types, &buf_type, sizeof(buf_type) },
        { NvSciBufGeneralAttrKey_NeedCpuAccess, &need_cpu_access, sizeof(need_cpu_access) },
        { NvSciBufGeneralAttrKey_EnableCpuCache, &is_enable_cpu_cache, sizeof(is_enable_cpu_cache) }
    };
    
    NvSciError sci_err = NvSciBufAttrListSetAttrs(buf_attr_list, attributes, 
                                                   sizeof(attributes) / sizeof(NvSciBufAttrKeyValuePair));
    if (sci_err != NvSciError_Success) {
        LOG_ERR("NvSciBufAttrListSetAttrs failed: %d", sci_err);
        return NVSIPL_STATUS_ERROR;
    }
    
    return NVSIPL_STATUS_OK;
}

SIPLStatus NvSIPLEncConsumer::FillNvSciSyncAttrList(NvSciSyncAttrList& signaler_attr_list,
                                                     NvSciSyncAttrList& waiter_attr_list) {
    if (!m_encoder) {
        LOG_ERR("Encoder not initialized");
        return NVSIPL_STATUS_NOT_INITIALIZED;
    }
    
    NvMediaStatus nvm_status = NvMediaIEPFillNvSciSyncAttrList(m_encoder.get(), 
                                                                signaler_attr_list, 
                                                                NVMEDIA_SIGNALER);
    if (nvm_status != NVMEDIA_STATUS_OK) {
        LOG_ERR("Signaler NvMediaIEPFillNvSciSyncAttrList failed: %d", nvm_status);
        return NVSIPL_STATUS_ERROR;
    }
    
    nvm_status = NvMediaIEPFillNvSciSyncAttrList(m_encoder.get(), 
                                                  waiter_attr_list, 
                                                  NVMEDIA_WAITER);
    if (nvm_status != NVMEDIA_STATUS_OK) {
        LOG_ERR("Waiter NvMediaIEPFillNvSciSyncAttrList failed: %d", nvm_status);
        return NVSIPL_STATUS_ERROR;
    }
    
    return NVSIPL_STATUS_OK;
}

SIPLStatus NvSIPLEncConsumer::RegisterNvSciBufObj(NvSciBufObj buf_obj) {
    if (!m_encoder) {
        LOG_ERR("Encoder not initialized");
        return NVSIPL_STATUS_NOT_INITIALIZED;
    }
    
    NvMediaStatus nvm_status = NvMediaIEPRegisterNvSciBufObj(m_encoder.get(), buf_obj);
    if (nvm_status != NVMEDIA_STATUS_OK) {
        LOG_ERR("NvMediaIEPRegisterNvSciBufObj failed: %d", nvm_status);
        return NVSIPL_STATUS_ERROR;
    }
    
    return NVSIPL_STATUS_OK;
}

SIPLStatus NvSIPLEncConsumer::RegisterSignalSyncObj(NvSciSyncObj signal_sync_obj) {
    if (!m_encoder) {
        LOG_ERR("Encoder not initialized");
        return NVSIPL_STATUS_NOT_INITIALIZED;
    }
    
    m_signal_sync_obj = signal_sync_obj;
    
    NvMediaStatus nvm_status = NvMediaIEPRegisterNvSciSyncObj(m_encoder.get(), 
                                                               NVMEDIA_EOFSYNCOBJ, 
                                                               signal_sync_obj);
    if (nvm_status != NVMEDIA_STATUS_OK) {
        LOG_ERR("NvMediaIEPRegisterNvSciSyncObj for EOF failed: %d", nvm_status);
        return NVSIPL_STATUS_ERROR;
    }
    
    return NVSIPL_STATUS_OK;
}

SIPLStatus NvSIPLEncConsumer::RegisterWaiterSyncObj(NvSciSyncObj waiter_sync_obj) {
    if (!m_encoder) {
        LOG_ERR("Encoder not initialized");
        return NVSIPL_STATUS_NOT_INITIALIZED;
    }
    
    NvMediaStatus nvm_status = NvMediaIEPRegisterNvSciSyncObj(m_encoder.get(), 
                                                               NVMEDIA_PRESYNCOBJ, 
                                                               waiter_sync_obj);
    if (nvm_status != NVMEDIA_STATUS_OK) {
        LOG_ERR("NvMediaIEPRegisterNvSciSyncObj for PRE failed: %d", nvm_status);
        return NVSIPL_STATUS_ERROR;
    }
    
    return NVSIPL_STATUS_OK;
}

SIPLStatus NvSIPLEncConsumer::InsertPreFence(NvSciSyncFence& pre_fence) {
    if (!m_encoder) {
        LOG_ERR("Encoder not initialized");
        return NVSIPL_STATUS_NOT_INITIALIZED;
    }
    
    NvMediaStatus nvm_status = NvMediaIEPInsertPreNvSciSyncFence(m_encoder.get(), &pre_fence);
    if (nvm_status != NVMEDIA_STATUS_OK) {
        LOG_ERR("NvMediaIEPInsertPreNvSciSyncFence failed: %d", nvm_status);
        return NVSIPL_STATUS_ERROR;
    }
    
    return NVSIPL_STATUS_OK;
}

SIPLStatus NvSIPLEncConsumer::SetEofSyncObj() {
    if (!m_encoder) {
        LOG_ERR("Encoder not initialized");
        return NVSIPL_STATUS_NOT_INITIALIZED;
    }
    
    NvMediaStatus nvm_status = NvMediaIEPSetNvSciSyncObjforEOF(m_encoder.get(), m_signal_sync_obj);
    if (nvm_status != NVMEDIA_STATUS_OK) {
        LOG_ERR("NvMediaIEPSetNvSciSyncObjforEOF failed: %d", nvm_status);
        return NVSIPL_STATUS_ERROR;
    }
    
    return NVSIPL_STATUS_OK;
}

SIPLStatus NvSIPLEncConsumer::UnregisterSyncObjs() {
    if (!m_encoder) {
        return NVSIPL_STATUS_NOT_INITIALIZED;
    }
    
    if (m_signal_sync_obj) {
        NvMediaStatus nvm_status = NvMediaIEPUnregisterNvSciSyncObj(m_encoder.get(), m_signal_sync_obj);
        if (nvm_status != NVMEDIA_STATUS_OK) {
            LOG_ERR("NvMediaIEPUnregisterNvSciSyncObj for EOF failed: %d", nvm_status);
        }
        m_signal_sync_obj = nullptr;
    }
    
    return NVSIPL_STATUS_OK;
}

SIPLStatus NvSIPLEncConsumer::EncodeFrame(NvSciBufObj sci_buf_obj,
                                           uint8_t** output_buffer,
                                           size_t* output_size,
                                           NvSciSyncFence* post_fence) {
    if (!m_encoder) {
        LOG_ERR("Encoder not initialized");
        return NVSIPL_STATUS_NOT_INITIALIZED;
    }
    
    NvMediaEncodePicParamsH265 encode_pic_params;
    memset(&encode_pic_params, 0, sizeof(NvMediaEncodePicParamsH265));
    
    // First frame is IDR, subsequent frames are auto-selected
    encode_pic_params.pictureType = (m_frame_num == m_config.start_frame) ? 
                                    NVMEDIA_ENCODE_PIC_TYPE_IDR : NVMEDIA_ENCODE_PIC_TYPE_AUTOSELECT;
    encode_pic_params.encodePicFlags = NVMEDIA_ENCODE_PIC_FLAG_OUTPUT_SPSPPS;
    encode_pic_params.nextBFrames = 0;
    
    NvMediaStatus nvm_status = NvMediaIEPFeedFrame(m_encoder.get(),             // encoder
                                                   sci_buf_obj,                 // input frame
                                                   &encode_pic_params,          // encoder parameters
                                                   NVMEDIA_ENCODER_INSTANCE_0); // encoder instance
    if (nvm_status != NVMEDIA_STATUS_OK) {
        LOG_ERR("NvMediaIEPFeedFrame failed: %d", nvm_status);
        return NVSIPL_STATUS_ERROR;
    }
    
    // Get EOF fence
    nvm_status = NvMediaIEPGetEOFNvSciSyncFence(m_encoder.get(), m_signal_sync_obj, post_fence);
    if (nvm_status != NVMEDIA_STATUS_OK) {
        LOG_ERR("NvMediaIEPGetEOFNvSciSyncFence failed: %d", nvm_status);
        return NVSIPL_STATUS_ERROR;
    }
    
    // Wait for encoded data
    bool encode_done = false;
    while (!encode_done) {
        NvMediaBitstreamBuffer bitstreams = { 0 };
        uint32_t bytes_available = 0;
        uint32_t bytes_written = 0;
        
        nvm_status = NvMediaIEPBitsAvailable(m_encoder.get(), &bytes_available,
                                             NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING, 
                                             NVMEDIA_ENCODE_TIMEOUT_INFINITE);
        
        switch (nvm_status) {
            case NVMEDIA_STATUS_OK: {
                // Allocate buffer for encoded data
                if (m_encoded_buffer) {
                    delete[] m_encoded_buffer;
                }
                m_encoded_buffer = new (std::nothrow) uint8_t[bytes_available];
                if (!m_encoded_buffer) {
                    LOG_ERR("Out of memory allocating encoded buffer");
                    return NVSIPL_STATUS_OUT_OF_MEMORY;
                }
                
                bitstreams.bitstream = m_encoded_buffer;
                bitstreams.bitstreamSize = bytes_available;
                std::fill(m_encoded_buffer, m_encoded_buffer + bytes_available, 0xE5);
                
                nvm_status = NvMediaIEPGetBits(m_encoder.get(), &bytes_written, 1, &bitstreams, nullptr);
                if (nvm_status != NVMEDIA_STATUS_OK && nvm_status != NVMEDIA_STATUS_NONE_PENDING) {
                    LOG_ERR("Error getting encoded bits: %d", nvm_status);
                    return NVSIPL_STATUS_ERROR;
                }
                
                if (bytes_written != bytes_available) {
                    LOG_ERR("Byte count mismatch: %u vs %u", bytes_available, bytes_written);
                    return NVSIPL_STATUS_ERROR;
                }
                
                *output_buffer = m_encoded_buffer;
                *output_size = static_cast<size_t>(bytes_available);
                m_encoded_bytes = bytes_available;
                encode_done = true;
                break;
            }
            
            case NVMEDIA_STATUS_PENDING:
                LOG_ERR("Encoded data pending (should not happen with blocking)");
                return NVSIPL_STATUS_ERROR;
                
            case NVMEDIA_STATUS_NONE_PENDING:
                LOG_ERR("No encoded data pending");
                return NVSIPL_STATUS_ERROR;
                
            default:
                LOG_ERR("Error in NvMediaIEPBitsAvailable: %d", nvm_status);
                return NVSIPL_STATUS_ERROR;
        }
    }
    
    m_frame_num++;
    return NVSIPL_STATUS_OK;
}

} // namespace nvsipl
} // namespace dynalgo

#else // DYNALGO_HAVE_NVSIPL

namespace dynalgo {
namespace nvsipl {

NvSIPLEncConsumer::NvSIPLEncConsumer(uint32_t sensor_id, const EncoderConfig& config) {}
NvSIPLEncConsumer::~NvSIPLEncConsumer() {}
int NvSIPLEncConsumer::Initialize(void*) { return 0; }
int NvSIPLEncConsumer::EncodeFrame(void*, void**, size_t*, void*) { return 0; }
int NvSIPLEncConsumer::FillNvSciBufAttrList(void*) { return 0; }
int NvSIPLEncConsumer::FillNvSciSyncAttrList(void*, void*) { return 0; }
int NvSIPLEncConsumer::RegisterNvSciBufObj(void*) { return 0; }
int NvSIPLEncConsumer::RegisterSignalSyncObj(void*) { return 0; }
int NvSIPLEncConsumer::RegisterWaiterSyncObj(void*) { return 0; }
int NvSIPLEncConsumer::InsertPreFence(void*) { return 0; }
int NvSIPLEncConsumer::SetEofSyncObj() { return 0; }
int NvSIPLEncConsumer::UnregisterSyncObjs() { return 0; }
bool NvSIPLEncConsumer::IsInitialized() const { return false; }
uint32_t NvSIPLEncConsumer::GetSensorId() const { return 0; }

} // namespace nvsipl
} // namespace dynalgo

#endif // DYNALGO_HAVE_NVSIPL