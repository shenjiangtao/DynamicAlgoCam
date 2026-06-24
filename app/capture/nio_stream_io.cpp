// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_stream_io.cpp — Stream I/O implementation with 4 MB buffered file writes.
//
// Sections:
//   1. isH264KeyFrame / writeH264StartCode / writeH264Frame — native H.264 NAL handling
//   2. writeDepthRawWithHeader — ORBBEC_DEPTH_RAW format writer
//   2b. writePcdFile — PCD v0.7 binary per-frame point cloud writer
//   3. openBufferedFile — large-buffer ofstream factory
//   4. createStreamEncoder / writeStreamFrame — encoder+file management

#include "nio_stream_io.hpp"
#include "nio_common.hpp"
#include "nio_log.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

namespace nio {

// === Section 1: H.264 NAL handling ===

// isH264KeyFrame: scan byte stream for Annex B start codes, check NAL type.
// Returns true if any IDR(5), SPS(7), or PPS(8) NAL unit is found.
bool isH264KeyFrame(const uint8_t* data, uint32_t size) {
    if (size < 5)
        return false;
    const uint8_t* ptr = data;
    const uint8_t* end = data + size;
    while (ptr + 4 < end) {
        if (ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 0 && ptr[3] == 1) {
            uint8_t nalType = ptr[4] & 0x1F;
            if (nalType == 5 || nalType == 7 || nalType == 8)
                return true;
        }
        ptr++;
    }
    return false;
}

void writeH264StartCode(std::ofstream& f) {
    const uint8_t sc[] = { 0x00, 0x00, 0x00, 0x01 };
    f.write(reinterpret_cast<const char*>(sc), 4);
}

// writeH264Frame: write NAL data to file.  Skips all frames before the
// first keyframe (IDR/SPS/PPS) to ensure the file starts with a decodable
// stream.  Prepends Annex B start code if the data doesn't already have one.
void writeH264Frame(std::ofstream& file, const uint8_t* data, uint32_t size, bool& keyFrameWritten, std::mutex& mtx) {
    if (!file.is_open())
        return;
    bool hasStartCode = (size >= 4 && data[0] == 0 && data[1] == 0 && ((data[2] == 0 && data[3] == 1) || data[2] == 1));
    std::lock_guard<std::mutex> lock(mtx);
    if (!keyFrameWritten) {
        if (isH264KeyFrame(data, size))
            keyFrameWritten = true;
        else
            return;
    }
    if (hasStartCode) {
        file.write(reinterpret_cast<const char*>(data), size);
    } else {
        writeH264StartCode(file);
        file.write(reinterpret_cast<const char*>(data), size);
    }
}

// === Section 2: Depth raw file writer with ORBBEC_DEPTH_RAW header ===
// Header layout: "ORBBEC_DEPTH_RAW" (16B) | width (4B) | height (4B) |
// bpp (4B) | scale (4B float) | frameSize (4B) | timestamp (8B)
// Subsequent frames are raw Y16 pixel data, each followed by flush.

void writeDepthRawWithHeader(std::ofstream& file, const uint8_t* data, uint32_t size, int width, int height,
                             float scale, uint64_t frameIndex, std::mutex& mtx, uint64_t deviceTsUs) {
    std::lock_guard<std::mutex> lock(mtx);
    if (!file.is_open())
        return;

    if (frameIndex == 0) {
        const char magic[] = "ORBBEC_DEPTH_RAW";
        file.write(magic, 16);

        uint32_t w32 = static_cast<uint32_t>(width);
        uint32_t h32 = static_cast<uint32_t>(height);
        file.write(reinterpret_cast<const char*>(&w32), 4);
        file.write(reinterpret_cast<const char*>(&h32), 4);

        uint32_t bpp = 2;
        file.write(reinterpret_cast<const char*>(&bpp), 4);

        file.write(reinterpret_cast<const char*>(&scale), 4);

        uint32_t frameSize = width * height * 2;
        file.write(reinterpret_cast<const char*>(&frameSize), 4);

        uint64_t ts = deviceTsUs ? deviceTsUs :
                                   static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                             std::chrono::system_clock::now().time_since_epoch())
                                                             .count());
        file.write(reinterpret_cast<const char*>(&ts), 8);
    }

    file.write(reinterpret_cast<const char*>(data), size);
    file.flush();
}

// === Section 2b: PCD file writer ===

// Writes a single PCD v0.7 binary file for one frame of point cloud data.
// Input wire format: [4B pointCount (uint32)] + pointCount * 23B
//   per point: float x(4) + float y(4) + float z(4) + uint8 intensity(1) +
//              uint16 ring(2) + double timestamp(8) = 23 bytes
// Output PCD fields: x y z intensity ring timestamp (F F F F U F)
//   intensity is expanded from uint8 → float32 for standard PCD compatibility.

void writePcdFile(const std::string& outputDir, const std::string& baseName, uint64_t frameIndex, const uint8_t* data,
                  uint32_t size, std::mutex& mtx, uint64_t /*deviceTsUs*/) {
    if (size < sizeof(uint32_t))
        return;

    uint32_t pointCount = 0;
    std::memcpy(&pointCount, data, sizeof(uint32_t));
    const uint8_t* pointData = data + sizeof(uint32_t);

    constexpr size_t srcPointSize = 4 + 4 + 4 + 1 + 2 + 8;
    size_t expectedSrcBytes = static_cast<size_t>(pointCount) * srcPointSize;
    if (size - sizeof(uint32_t) < expectedSrcBytes)
        return;

    std::string filepath;
    {
        std::lock_guard<std::mutex> lock(mtx);
        static bool dirCreated = false;
        if (!dirCreated) {
            std::string path = outputDir;
            for (size_t pos = 0; pos < path.size();) {
                pos = path.find('/', pos + 1);
                if (pos == std::string::npos)
                    pos = path.size();
                mkdir(path.substr(0, pos).c_str(), 0755);
            }
            dirCreated = true;
        }
    }

    char fname[512];
    std::snprintf(fname, sizeof(fname), "%s/%s_%06lu.pcd", outputDir.c_str(), baseName.c_str(),
                  static_cast<unsigned long>(frameIndex));
    filepath = fname;

    std::ofstream pcd(filepath, std::ios::binary);
    if (!pcd.is_open())
        return;

    char header[1024];
    int hdrLen = std::snprintf(header, sizeof(header),
                               "# .PCD v0.7 - Point Cloud Data file format\n"
                               "VERSION 0.7\n"
                               "FIELDS x y z intensity ring timestamp\n"
                               "SIZE 4 4 4 4 2 8\n"
                               "TYPE F F F F U F\n"
                               "COUNT 1 1 1 1 1 1\n"
                               "WIDTH %u\n"
                               "HEIGHT 1\n"
                               "VIEWPOINT 0 0 0 1 0 0 0\n"
                               "POINTS %u\n"
                               "DATA binary\n",
                               pointCount, pointCount);
    pcd.write(header, hdrLen);

    constexpr size_t dstPointSize = 4 + 4 + 4 + 4 + 2 + 8;
    std::vector<uint8_t> buf(static_cast<size_t>(pointCount) * dstPointSize);
    uint8_t* dst = buf.data();

    const uint8_t* ptr = pointData;
    for (uint32_t i = 0; i < pointCount; ++i) {
        float x, y, z;
        uint8_t rawIntensity;
        uint16_t ring;
        double ptTs;
        std::memcpy(&x, ptr, 4);
        ptr += 4;
        std::memcpy(&y, ptr, 4);
        ptr += 4;
        std::memcpy(&z, ptr, 4);
        ptr += 4;
        std::memcpy(&rawIntensity, ptr, 1);
        ptr += 1;
        std::memcpy(&ring, ptr, 2);
        ptr += 2;
        std::memcpy(&ptTs, ptr, 8);
        ptr += 8;

        float intensity = static_cast<float>(rawIntensity);
        std::memcpy(dst, &x, 4);
        dst += 4;
        std::memcpy(dst, &y, 4);
        dst += 4;
        std::memcpy(dst, &z, 4);
        dst += 4;
        std::memcpy(dst, &intensity, 4);
        dst += 4;
        std::memcpy(dst, &ring, 2);
        dst += 2;
        std::memcpy(dst, &ptTs, 8);
        dst += 8;
    }

    pcd.write(reinterpret_cast<const char*>(buf.data()), buf.size());
    pcd.close();
}

// === Section 3: Buffered file factory ===

// openBufferedFile: open an ofstream with a user-space buffer of bufSize bytes.
// The buffer is heap-allocated and shared via outBuf so it outlives the ofstream.
std::shared_ptr<std::ofstream> openBufferedFile(const std::string& path, std::ios_base::openmode mode, int bufSize,
                                                std::shared_ptr<char[]>* outBuf) {
    auto f = std::make_shared<std::ofstream>(path, mode);
    if (f->is_open() && bufSize > 0) {
        auto buf = std::shared_ptr<char[]>(new char[bufSize]);
        f->rdbuf()->pubsetbuf(buf.get(), bufSize);
        if (outBuf)
            *outBuf = buf;
    }
    return f;
}

// === Section 4: StreamEncoder factory and frame writer ===

// createStreamEncoder: create encoder + buffered file for a stream.
// For native H.264/H.265: no software encoder needed — just write NALs.
// For other formats (MJPEG/YUYV/RGB/etc.): create H264Encoder.
// If encoder init fails, fall back to raw binary file (no encoding).
std::shared_ptr<StreamEncoder> createStreamEncoder(const std::string& filePath, NioFormat format, int w, int h, int fps,
                                                   const char* seiUuid, bool writeSEI) {
    auto se = std::make_shared<StreamEncoder>();
    se->srcFormat = format;
    se->width = w;
    se->height = h;
    se->fps = fps;
    se->sensorTag = filePath;
    se->writeSEI = writeSEI;

    if (format == NioFormat::H264 || format == NioFormat::H265 || format == NioFormat::HEVC) {
        se->isNativeH264 = true;
        se->file = openBufferedFile(filePath, std::ios::binary, NIO_FILE_BUF_SIZE, &se->fileBuf);
        return se;
    }

    se->encoder = std::make_shared<H264Encoder>();
    if (!se->encoder->init(w, h, fps, format, 4000000, seiUuid)) {
        std::cerr << " Failed to init H264 encoder for format=" << nioFormatToStr(format) << " " << w << "x" << h
                  << std::endl;
        NIO_LOG_WARN_S("Fallback: H264 encoder init failed for " << filePath << " format=" << nioFormatToStr(format)
                                                                 << " " << w << "x" << h << "@" << fps);
        se->encoder.reset();
        se->file = openBufferedFile(filePath, std::ios::binary, NIO_FILE_BUF_SIZE, &se->fileBuf);
        return se;
    }

    se->file = openBufferedFile(filePath, std::ios::binary, NIO_FILE_BUF_SIZE, &se->fileBuf);
    return se;
}

// writeStreamFrame: dispatch frame data to the appropriate writer.
// - native H.264: writeH264Frame (skip until keyframe)
// - software-encoded: H264Encoder::encode (MJPEG decode + sws + x264)
// - fallback: raw binary write (when encoder init failed)
void writeStreamFrame(StreamEncoder* se, const uint8_t* data, uint32_t size, uint64_t deviceTsUs) {
    if (!se || !se->file || !se->file->is_open())
        return;

    if (se->isNativeH264) {
        writeH264Frame(*se->file, data, size, se->h264KeyFrameWritten, se->mtx);
    } else if (se->encoder) {
        se->encoder->encode(data, size, *se->file, se->mtx, deviceTsUs, se->writeSEI);
    } else {
        std::lock_guard<std::mutex> lock(se->mtx);
        se->file->write(reinterpret_cast<const char*>(data), size);
        se->file->flush();
    }
}

} // namespace nio
