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

#include <cstdint>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace nio {

// 4 MB file buffer — reduces syscall frequency for high-bitrate H.264 writes
// Defined in nio_stream_io.cpp (extern + single definition) so the value
// lives in one TU only; avoids `inline constexpr` which would require C++17
// (project standard is C++14).
extern const int NIO_FILE_BUF_SIZE;

// StreamEncoder: bundles encoder + output file + per-stream mutex + metadata.
// For native H.264 streams (camera outputs H.264 directly), encoder is null
// and NALs are written verbatim after the first keyframe is seen.
// For FFV1 lossless streams (Y16/Y8/YUYV/MJPG), ffv1Encoder is non-null and
// file is null (FFV1Encoder writes directly to MKV via libavformat).
// For other formats (RGB/etc.), H264Encoder is used with a separate .h264 file.
struct StreamEncoder
{
    std::shared_ptr<H264Encoder> encoder; // null for native H.264 streams
    std::shared_ptr<std::ofstream> file;  // output .h264 file
    std::mutex mtx;                       // protects file writes
    bool h264KeyFrameWritten = false;     // native H.264: skip until first IDR/SPS/PPS
    bool isNativeH264 = false;            // true = camera outputs H.264 directly
    bool writeSEI = true;                 // embed copyright/timestamp SEI NALs
    NioFormat srcFormat = NioFormat::UNKNOWN;
    int width = 0;
    int height = 0;
    int fps = 30;
    std::string sensorTag;           // for logging/error messages
    std::shared_ptr<char[]> fileBuf; // holds the 4 MB file buffer alive
};

// SensorFiles: per-device collection of stream encoders and raw data files.
// Each sensor type (color, depth, IR, etc.) gets its own StreamEncoder.
// depthRawFile / imuFile have their own mutexes for concurrent writes.
struct SensorFiles
{
    std::shared_ptr<StreamEncoder> color;
    std::shared_ptr<StreamEncoder> depth;
    std::shared_ptr<StreamEncoder> ir;
    std::shared_ptr<StreamEncoder> irLeft;
    std::shared_ptr<StreamEncoder> irRight;
    std::shared_ptr<std::ofstream> depthRawFile; // .raw binary depth output
    std::shared_ptr<std::ofstream> imuFile;      // IMU CSV/binary output
    std::mutex depthRawMtx;
    std::mutex imuMtx;

    NioFrameCounts frameCounts; // per-type frame counter
    std::mutex countMtx;
};

// isH264KeyFrame: scan NAL units for IDR(5)/SPS(7)/PPS(8) types
bool isH264KeyFrame(const uint8_t* data, uint32_t size);

// writeH264StartCode: emit 4-byte Annex B start code (00 00 00 01)
void writeH264StartCode(std::ofstream& f);

// writeH264Frame: write H.264 NALs to file; skips frames until first keyframe
void writeH264Frame(std::ofstream& file, const uint8_t* data, uint32_t size, bool& keyFrameWritten, std::mutex& mtx);

// writeDepthRawWithHeader: write depth frame with NIO_DEPTH_RAW header
// (magic, width, height, bpp, scale, frameSize, timestamp) on frame 0
void writeDepthRawWithHeader(std::ofstream& file, const uint8_t* data, uint32_t size, int width, int height,
                             float scale, uint64_t frameIndex, std::mutex& mtx, uint64_t deviceTsUs = 0);

// mkdirRecursive: create all directories in path (like mkdir -p)
void mkdirRecursive(const std::string& path);

// writePcdFile: write one PCD v0.7 binary file per frame (legacy, one file per frame).
// Input wire format is self-describing (see PcdLayout::serialize):
//   [4B srcPointSize] [4B numFields] [4B pointCount]
//   [numFields * 24B PcdFieldDesc entries]
//   [pointCount * srcPointSize packed point data]
// Output PCD header and binary data are generated from the field descriptors.
// File path = outputDir / baseName_<deviceTsUs>.pcd
void writePcdFile(const std::string& outputDir, const std::string& baseName, const uint8_t* data, uint32_t size,
                  std::mutex& mtx, uint64_t deviceTsUs = 0);

// PcdStream: continuous PCD stream file (.pcs format).
//
// File layout:
//   [Header]
//     16B  magic = "NIO_PCD_STREAM\0"
//     4B   numFields  (uint32)
//     4B   srcPointSize (uint32)
//     4B   pcdPointSize (uint32)
//     numFields * 24B  PcdFieldDesc entries
//   [Frame 0]
//     8B   timestampUs (uint64)
//     4B   pointCount  (uint32)
//     pointCount * pcdPointSize bytes  (converted PCD binary data)
//   [Frame 1]
//     ...
//   [Trailing Index]  (written on close)
//     8B   dataStartOffset (uint64) — byte offset of first frame from file start
//     4B   numFrames (uint32)
//     numFrames * 16B entries:
//       8B  timestampUs
//       8B  frameOffset (byte offset from file start to this frame's timestampUs)
struct PcdStream
{
    std::shared_ptr<std::ofstream> file;
    std::shared_ptr<char[]> fileBuf;
    PcdLayout layout;
    uint64_t dataStartOffset = 0; // byte offset of first frame data after header
    bool headerWritten = false;

    struct IndexEntry
    {
        uint64_t timestampUs;
        uint64_t offset;
    };
    std::vector<IndexEntry> index;
};

// writePcdStreamHeader: write the .pcs file header (magic + field descriptors).
// Must be called once with the first frame's wire data to extract the layout.
bool writePcdStreamHeader(PcdStream& stream, const uint8_t* wireData, uint32_t wireSize);

// writePcdStreamFrame: append one frame of converted point data to the .pcs file.
// Extracts layout from wireData on first call (auto-writes header if needed).
// Records an index entry for random access.
bool writePcdStreamFrame(PcdStream& stream, const uint8_t* wireData, uint32_t wireSize, uint64_t deviceTsUs);

// writePcdStreamIndex: write the trailing index table and close the file.
void writePcdStreamIndex(PcdStream& stream);

// openBufferedFile: open ofstream with large user-space buffer for fast writes
std::shared_ptr<std::ofstream> openBufferedFile(const std::string& path,
                                                std::ios_base::openmode mode = std::ios::binary,
                                                int bufSize = NIO_FILE_BUF_SIZE,
                                                std::shared_ptr<char[]>* outBuf = nullptr);

// createStreamEncoder: factory — creates encoder + buffered file for a stream
// For native H.264/H.265: no encoder, just a file.  For MJPEG/YUYV/etc:
// creates H264Encoder.  Falls back to raw file if encoder init fails.
std::shared_ptr<StreamEncoder> createStreamEncoder(const std::string& filePath, NioFormat format, int w, int h, int fps,
                                                   const char* seiUuid = "jiangtao.shen@ad", bool writeSEI = true);

// writeStreamFrame: dispatch to native-H264 write or encode+write
void writeStreamFrame(StreamEncoder* se, const uint8_t* data, uint32_t size, uint64_t deviceTsUs = 0);

} // namespace nio
