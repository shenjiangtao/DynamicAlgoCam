// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_stream_io.cpp — Stream I/O implementation with 4 MB buffered file writes.
//
// Sections:
//   1. isH264KeyFrame / writeH264StartCode / writeH264Frame — native H.264 NAL handling
//   2. writeDepthRawWithHeader — NIO_DEPTH_RAW format writer
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

// 4 MB file buffer — single definition backing the extern declaration in
// the matching header.
const int nio::NIO_FILE_BUF_SIZE = 4 * 1024 * 1024;

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

// === Section 2: Depth raw file writer with NIO_DEPTH_RAW header ===
// Header layout: "NIO_DEPTH_RAW" (16B, null-padded) | width (4B) | height (4B) |
// bpp (4B) | scale (4B float) | frameSize (4B) | timestamp (8B)
// Subsequent frames are raw Y16 pixel data, each followed by flush.

void writeDepthRawWithHeader(std::ofstream& file, const uint8_t* data, uint32_t size, int width, int height,
                             float scale, uint64_t frameIndex, std::mutex& mtx, uint64_t deviceTsUs) {
    std::lock_guard<std::mutex> lock(mtx);
    if (!file.is_open())
        return;

    if (frameIndex == 0) {
        const char magic[] = "NIO_DEPTH_RAW";
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
}

// === Section 2b: PCD file writer ===

// Writes a single PCD v0.7 binary file for one frame of point cloud data.
// Input wire format (self-describing):
//   [4B srcPointSize] [4B numFields] [4B pointCount]
//   [numFields * 24B PcdFieldDesc entries]
//   [pointCount * srcPointSize packed point data]
// Each point's fields are described by PcdFieldDesc.  If pcdSize != srcSize
// for a field, the source value is cast to the PCD type (e.g. uint8
// intensity -> float32 intensity).

namespace {

// convertPcdPoints: convert wire point data to PCD binary format.
// Returns the converted data buffer (or empty on error).
std::vector<uint8_t> convertPcdPoints(const PcdLayout& layout, uint32_t pointCount, const uint8_t* pointData) {
    size_t srcBytes = static_cast<size_t>(pointCount) * layout.srcPointSize;
    uint32_t dstPointSize = layout.pcdPointSize();

    bool needsConvert = false;
    for (const auto& f : layout.fields) {
        if (f.pcdSize != f.srcSize) {
            needsConvert = true;
            break;
        }
    }

    if (!needsConvert) {
        return std::vector<uint8_t>(pointData, pointData + srcBytes);
    }

    std::vector<uint8_t> buf(static_cast<size_t>(pointCount) * dstPointSize);
    uint8_t* __restrict__ dstPtr = buf.data();
    const uint8_t* __restrict__ srcBase = pointData;

    for (uint32_t i = 0; i < pointCount; ++i) {
        const uint8_t* srcPt = srcBase + static_cast<size_t>(i) * layout.srcPointSize;
        for (const auto& f : layout.fields) {
            const uint8_t* srcField = srcPt + f.srcOffset;
            if (f.pcdSize == f.srcSize) {
                std::memcpy(dstPtr, srcField, f.pcdSize);
            } else if (f.srcSize == 1 && f.pcdSize == 4 && f.pcdType == 'F') {
                float val = static_cast<float>(*srcField);
                std::memcpy(dstPtr, &val, 4);
            } else if (f.srcSize == 1 && f.pcdSize == 4 && f.pcdType == 'U') {
                uint32_t val = static_cast<uint32_t>(*srcField);
                std::memcpy(dstPtr, &val, 4);
            } else if (f.srcSize == 2 && f.pcdSize == 4 && f.pcdType == 'F') {
                uint16_t tmp;
                std::memcpy(&tmp, srcField, 2);
                float val = static_cast<float>(tmp);
                std::memcpy(dstPtr, &val, 4);
            } else {
                size_t copyN = std::min(static_cast<size_t>(f.srcSize), static_cast<size_t>(f.pcdSize));
                std::memcpy(dstPtr, srcField, copyN);
                if (copyN < static_cast<size_t>(f.pcdSize))
                    std::memset(dstPtr + copyN, 0, f.pcdSize - copyN);
            }
            dstPtr += f.pcdSize;
        }
    }
    return buf;
}

} // anonymous namespace

void mkdirRecursive(const std::string& path) {
    mkdirp(path);
}

void writePcdFile(const std::string& outputDir, const std::string& baseName, const uint8_t* data, uint32_t size,
                  std::mutex& mtx, uint64_t deviceTsUs) {
    PcdLayout layout;
    uint32_t pointCount = 0;
    size_t hdrBytes = PcdLayout::deserialize(data, size, layout, pointCount);
    if (hdrBytes == 0 || pointCount == 0)
        return;

    size_t expectedPointBytes = static_cast<size_t>(pointCount) * layout.srcPointSize;
    if (size - hdrBytes < expectedPointBytes)
        return;

    const uint8_t* pointData = data + hdrBytes;

    {
        std::lock_guard<std::mutex> lock(mtx);
        static bool dirCreated = false;
        if (!dirCreated) {
            mkdirRecursive(outputDir);
            dirCreated = true;
        }
    }

    char fname[512];
    std::snprintf(fname, sizeof(fname), "%s/%s_%lu.pcd", outputDir.c_str(), baseName.c_str(),
                  static_cast<unsigned long>(deviceTsUs));

    std::ofstream pcd(fname, std::ios::binary);
    if (!pcd.is_open())
        return;

    std::string fieldsLine, sizeLine, typeLine, countLine;
    for (size_t i = 0; i < layout.fields.size(); ++i) {
        const auto& f = layout.fields[i];
        if (i) {
            fieldsLine += ' ';
            sizeLine += ' ';
            typeLine += ' ';
            countLine += ' ';
        }
        fieldsLine += f.name;
        sizeLine += std::to_string(f.pcdSize);
        typeLine += f.pcdType;
        countLine += '1';
    }

    char header[2048];
    int hdrLen = std::snprintf(header, sizeof(header),
                               "# .PCD v0.7 - Point Cloud Data file format\n"
                               "VERSION 0.7\n"
                               "FIELDS %s\n"
                               "SIZE %s\n"
                               "TYPE %s\n"
                               "COUNT %s\n"
                               "WIDTH %u\n"
                               "HEIGHT 1\n"
                               "VIEWPOINT 0 0 0 1 0 0 0\n"
                               "POINTS %u\n"
                               "DATA binary\n",
                               fieldsLine.c_str(), sizeLine.c_str(), typeLine.c_str(), countLine.c_str(), pointCount,
                               pointCount);
    pcd.write(header, hdrLen);

    auto converted = convertPcdPoints(layout, pointCount, pointData);
    pcd.write(reinterpret_cast<const char*>(converted.data()), static_cast<std::streamsize>(converted.size()));
    pcd.close();
}

// === Section 2c: PCD stream writer (.pcs format) ===

bool writePcdStreamHeader(PcdStream& stream, const uint8_t* wireData, uint32_t wireSize) {
    PcdLayout layout;
    uint32_t pointCount = 0;
    size_t hdrBytes = PcdLayout::deserialize(wireData, wireSize, layout, pointCount);
    if (hdrBytes == 0)
        return false;

    stream.layout = layout;

    const char magic[] = "NIO_PCD_STREAM";
    stream.file->write(magic, 16);

    uint32_t n = static_cast<uint32_t>(layout.fields.size());
    stream.file->write(reinterpret_cast<const char*>(&n), 4);
    stream.file->write(reinterpret_cast<const char*>(&layout.srcPointSize), 4);
    uint32_t pcdPtSz = layout.pcdPointSize();
    stream.file->write(reinterpret_cast<const char*>(&pcdPtSz), 4);

    if (!layout.fields.empty())
        stream.file->write(reinterpret_cast<const char*>(layout.fields.data()), n * sizeof(PcdFieldDesc));

    stream.dataStartOffset = 16 + 4 + 4 + 4 + n * sizeof(PcdFieldDesc);
    stream.headerWritten = true;
    return true;
}

bool writePcdStreamFrame(PcdStream& stream, const uint8_t* wireData, uint32_t wireSize, uint64_t deviceTsUs) {
    if (!stream.file || !stream.file->is_open())
        return false;

    if (!stream.headerWritten) {
        if (!writePcdStreamHeader(stream, wireData, wireSize))
            return false;
    }

    // Deserialize wire data just to get pointCount and point data location
    // (layout is already stored in stream.layout from the header write)
    PcdLayout wireLayout;
    uint32_t pointCount = 0;
    size_t hdrBytes = PcdLayout::deserialize(wireData, wireSize, wireLayout, pointCount);
    if (hdrBytes == 0 || pointCount == 0)
        return false;

    size_t expectedPointBytes = static_cast<size_t>(pointCount) * wireLayout.srcPointSize;
    if (wireSize - hdrBytes < expectedPointBytes)
        return false;

    const uint8_t* pointData = wireData + hdrBytes;

    uint64_t frameOffset = static_cast<uint64_t>(stream.file->tellp());

    stream.file->write(reinterpret_cast<const char*>(&deviceTsUs), 8);
    stream.file->write(reinterpret_cast<const char*>(&pointCount), 4);

    auto converted = convertPcdPoints(stream.layout, pointCount, pointData);
    stream.file->write(reinterpret_cast<const char*>(converted.data()), static_cast<std::streamsize>(converted.size()));

    stream.index.push_back({ deviceTsUs, frameOffset });
    return true;
}

void writePcdStreamIndex(PcdStream& stream) {
    if (!stream.file || !stream.file->is_open())
        return;

    stream.file->write(reinterpret_cast<const char*>(&stream.dataStartOffset), 8);
    uint32_t numFrames = static_cast<uint32_t>(stream.index.size());
    stream.file->write(reinterpret_cast<const char*>(&numFrames), 4);

    for (const auto& entry : stream.index) {
        stream.file->write(reinterpret_cast<const char*>(&entry.timestampUs), 8);
        stream.file->write(reinterpret_cast<const char*>(&entry.offset), 8);
    }
    stream.file->flush();
    stream.file->close();
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
    }
}

} // namespace nio
