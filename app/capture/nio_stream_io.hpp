// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_stream_io.hpp — Stream I/O utilities: H.264 file writing with buffered
// I/O, depth raw file writing, StreamEncoder / SensorFiles management.
//
// StreamEncoder: bundles H264Encoder + output file + mutex + metadata.
//   - isNativeH264: if true, the camera already outputs H.264 NALs (e.g.
//     Orbbec Gemini 2's OB_FORMAT_H264) — no re-encoding needed, just
//     write NALs directly (drop frames until first keyframe).
//   - writeSEI: whether to embed copyright + timestamp SEI NAL units.
//
// SensorFiles: per-device collection of StreamEncoders (color/depth/IR/
//   IRLeft/IRRight) + depth raw file + IMU file + frame counters.
//
// All file writes use 4 MB buffered I/O (NIO_FILE_BUF_SIZE) to reduce
// syscall overhead for high-throughput video recording.

#pragma once

#include "nio_h264_encoder.hpp"

#include "nio_types.hpp"

#include <fstream>
#include <mutex>
#include <map>
#include <memory>
#include <string>
#include <cstdint>

namespace nio {

// 4 MB file buffer — reduces syscall frequency for high-bitrate H.264 writes
static const int NIO_FILE_BUF_SIZE = 4 * 1024 * 1024;

// StreamEncoder: bundles encoder + output file + per-stream mutex + metadata.
// For native H.264 streams (camera outputs H.264 directly), encoder is null
// and NALs are written verbatim after the first keyframe is seen.
// For FFV1 lossless streams (Y16/Y8/YUYV/MJPG), ffv1Encoder is non-null and
// file is null (FFV1Encoder writes directly to MKV via libavformat).
// For other formats (RGB/etc.), H264Encoder is used with a separate .h264 file.
struct StreamEncoder {
std::shared_ptr<H264Encoder> encoder; // null for native H.264 streams
std::shared_ptr<std::ofstream> file; // output .h264 file
std::mutex mtx; // protects file writes
bool h264KeyFrameWritten = false; // native H.264: skip until first IDR/SPS/PPS
bool isNativeH264 = false; // true = camera outputs H.264 directly
bool writeSEI = true; // embed copyright/timestamp SEI NALs
    NioFormat srcFormat = NioFormat::UNKNOWN;
    int width = 0;
    int height = 0;
    int fps = 30;
std::string sensorTag; // for logging/error messages
std::shared_ptr<char[]> fileBuf; // holds the 4 MB file buffer alive
};

// SensorFiles: per-device collection of stream encoders and raw data files.
// Each sensor type (color, depth, IR, etc.) gets its own StreamEncoder.
// depthRawFile / imuFile have their own mutexes for concurrent writes.
struct SensorFiles {
    std::shared_ptr<StreamEncoder> color;
    std::shared_ptr<StreamEncoder> depth;
    std::shared_ptr<StreamEncoder> ir;
    std::shared_ptr<StreamEncoder> irLeft;
    std::shared_ptr<StreamEncoder> irRight;
    std::shared_ptr<std::ofstream> depthRawFile;  // .raw binary depth output
    std::shared_ptr<std::ofstream> imuFile;        // IMU CSV/binary output
    std::mutex depthRawMtx;
    std::mutex imuMtx;
    std::shared_ptr<std::ofstream> pcdFile;        // .pcd binary point cloud output
    std::mutex pcdMtx;

    NioFrameCounts frameCounts;   // per-type frame counter
    std::mutex countMtx;
};

// isH264KeyFrame: scan NAL units for IDR(5)/SPS(7)/PPS(8) types
bool isH264KeyFrame(const uint8_t *data, uint32_t size);

// writeH264StartCode: emit 4-byte Annex B start code (00 00 00 01)
void writeH264StartCode(std::ofstream &f);

// writeH264Frame: write H.264 NALs to file; skips frames until first keyframe
void writeH264Frame(std::ofstream &file, const uint8_t *data, uint32_t size,
                    bool &keyFrameWritten, std::mutex &mtx);

// writeDepthRawWithHeader: write depth frame with ORBBEC_DEPTH_RAW header
// (magic, width, height, bpp, scale, frameSize, timestamp) on frame 0
void writeDepthRawWithHeader(std::ofstream &file, const uint8_t *data, uint32_t size,
                             int width, int height, float scale,
                             uint64_t frameIndex, std::mutex &mtx,
                             uint64_t deviceTsUs = 0);

// writePointRawWithHeader: write point-cloud frame with NIO_POINT_CLOUD_RAW
// container. Frame 0 writes file header (magic + version + point fields metadata
// + start timestamp). Each frame writes a per-frame header (frameIndex +
// timestampUs + pointCount + dataBytes) followed by binary point data.
// Each point is 26 bytes: float x,y,z,intensity + uint16_t ring + double timestamp.
void writePointRawWithHeader(std::ofstream &file, const uint8_t *data, uint32_t size,
                              uint64_t frameIndex, std::mutex &mtx,
                              uint64_t deviceTsUs = 0);

// openBufferedFile: open ofstream with large user-space buffer for fast writes
std::shared_ptr<std::ofstream> openBufferedFile(const std::string &path,
                                                 std::ios_base::openmode mode = std::ios::binary,
                                                 int bufSize = NIO_FILE_BUF_SIZE,
                                                 std::shared_ptr<char[]> *outBuf = nullptr);

// createStreamEncoder: factory — creates encoder + buffered file for a stream
// For native H.264/H.265: no encoder, just a file.  For MJPEG/YUYV/etc:
// creates H264Encoder.  Falls back to raw file if encoder init fails.
std::shared_ptr<StreamEncoder> createStreamEncoder(const std::string &filePath,
                                                    NioFormat format, int w, int h, int fps,
                                                    const char *seiUuid = "nio@orbbec-fusio",
                                                    bool writeSEI = true);

// writeStreamFrame: dispatch to native-H264 write or encode+write
void writeStreamFrame(StreamEncoder *se, const uint8_t *data, uint32_t size,
                      uint64_t deviceTsUs = 0);

} // namespace nio
