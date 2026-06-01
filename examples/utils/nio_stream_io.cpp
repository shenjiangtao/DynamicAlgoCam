// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_stream_io.cpp — Stream I/O implementation with 4 MB buffered file writes.

#include "nio_stream_io.hpp"
#include "nio_common.hpp"
#include "nio_log.hpp"

#include <iostream>
#include <vector>
#include <cstring>
#include <chrono>
#include <cstdlib>

namespace nio {

bool isH264KeyFrame(const uint8_t *data, uint32_t size) {
    if(size < 5) return false;
    const uint8_t *ptr = data;
    const uint8_t *end = data + size;
    while(ptr + 4 < end) {
        if(ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 0 && ptr[3] == 1) {
            uint8_t nalType = ptr[4] & 0x1F;
            if(nalType == 5 || nalType == 7 || nalType == 8) return true;
        }
        ptr++;
    }
    return false;
}

void writeH264StartCode(std::ofstream &f) {
    const uint8_t sc[] = {0x00, 0x00, 0x00, 0x01};
    f.write(reinterpret_cast<const char *>(sc), 4);
}

void writeH264Frame(std::ofstream &file, const uint8_t *data, uint32_t size,
                     bool &keyFrameWritten, std::mutex &mtx) {
    if(!file.is_open()) return;
    bool hasStartCode = (size >= 4 && data[0] == 0 && data[1] == 0 &&
                         ((data[2] == 0 && data[3] == 1) || data[2] == 1));
    std::lock_guard<std::mutex> lock(mtx);
    if(!keyFrameWritten) {
        if(isH264KeyFrame(data, size)) keyFrameWritten = true;
        else return;
    }
    if(hasStartCode) {
        file.write(reinterpret_cast<const char *>(data), size);
    } else {
        writeH264StartCode(file);
        file.write(reinterpret_cast<const char *>(data), size);
    }
}

void writeDepthRawWithHeader(std::ofstream &file, const uint8_t *data, uint32_t size,
                              int width, int height, float scale,
                              uint64_t frameIndex, std::mutex &mtx,
                              uint64_t deviceTsUs) {
    std::lock_guard<std::mutex> lock(mtx);
    if(!file.is_open()) return;

    if(frameIndex == 0) {
        const char magic[] = "ORBBEC_DEPTH_RAW";
        file.write(magic, 16);

        uint32_t w32 = static_cast<uint32_t>(width);
        uint32_t h32 = static_cast<uint32_t>(height);
        file.write(reinterpret_cast<const char *>(&w32), 4);
        file.write(reinterpret_cast<const char *>(&h32), 4);

        uint32_t bpp = 2;
        file.write(reinterpret_cast<const char *>(&bpp), 4);

        file.write(reinterpret_cast<const char *>(&scale), 4);

        uint32_t frameSize = width * height * 2;
        file.write(reinterpret_cast<const char *>(&frameSize), 4);

        uint64_t ts = deviceTsUs ? deviceTsUs
            : static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        file.write(reinterpret_cast<const char *>(&ts), 8);
    }

    file.write(reinterpret_cast<const char *>(data), size);
    file.flush();
}

std::shared_ptr<std::ofstream> openBufferedFile(const std::string &path,
    std::ios_base::openmode mode, int bufSize, std::shared_ptr<char[]> *outBuf) {
    auto f = std::make_shared<std::ofstream>(path, mode);
    if(f->is_open() && bufSize > 0) {
        auto buf = std::shared_ptr<char[]>(new char[bufSize]);
        f->rdbuf()->pubsetbuf(buf.get(), bufSize);
        if(outBuf) *outBuf = buf;
    }
    return f;
}

std::shared_ptr<StreamEncoder> createStreamEncoder(const std::string &filePath,
    OBFormat format, int w, int h, int fps,
    const char *seiUuid, bool writeSEI) {
    auto se = std::make_shared<StreamEncoder>();
    se->srcFormat = format;
    se->width = w;
    se->height = h;
    se->fps = fps;
    se->sensorTag = filePath;
    se->writeSEI = writeSEI;

    if(format == OB_FORMAT_H264 || format == OB_FORMAT_H265 || format == OB_FORMAT_HEVC) {
        se->isNativeH264 = true;
        se->file = openBufferedFile(filePath, std::ios::binary, NIO_FILE_BUF_SIZE, &se->fileBuf);
        return se;
    }

    se->encoder = std::make_shared<H264Encoder>();
    if(!se->encoder->init(w, h, fps, format, 4000000, seiUuid)) {
        std::cerr << " Failed to init H264 encoder for format=" << format
                  << " " << w << "x" << h << std::endl;
        NIO_LOG_WARN_S("Fallback: H264 encoder init failed for " << filePath
                       << " format=" << format << " " << w << "x" << h << "@" << fps);
        se->encoder.reset();
        se->file = openBufferedFile(filePath, std::ios::binary, NIO_FILE_BUF_SIZE, &se->fileBuf);
        return se;
    }

    se->file = openBufferedFile(filePath, std::ios::binary, NIO_FILE_BUF_SIZE, &se->fileBuf);
    return se;
}

void writeStreamFrame(StreamEncoder *se, const uint8_t *data, uint32_t size,
                      uint64_t deviceTsUs) {
    if(!se || !se->file || !se->file->is_open()) return;

    if(se->isNativeH264) {
        writeH264Frame(*se->file, data, size, se->h264KeyFrameWritten, se->mtx);
    } else if(se->encoder) {
        se->encoder->encode(data, size, *se->file, se->mtx, deviceTsUs, se->writeSEI);
    } else {
        std::lock_guard<std::mutex> lock(se->mtx);
        se->file->write(reinterpret_cast<const char *>(data), size);
        se->file->flush();
    }
}

} // namespace nio
