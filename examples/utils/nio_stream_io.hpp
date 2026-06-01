// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_stream_io.hpp — Stream I/O utilities: H.264 file writing with buffered
// I/O, depth raw file writing, StreamEncoder / SensorFiles management.

#pragma once

#include "nio_h264_encoder.hpp"

#include <fstream>
#include <mutex>
#include <map>
#include <memory>
#include <string>
#include <cstdint>

#include <libobsensor/ObSensor.hpp>

namespace nio {

static const int NIO_FILE_BUF_SIZE = 4 * 1024 * 1024;

struct StreamEncoder {
    std::shared_ptr<H264Encoder> encoder;
    std::shared_ptr<std::ofstream> file;
    std::mutex mtx;
    bool h264KeyFrameWritten = false;
    bool isNativeH264 = false;
    bool writeSEI = true;
    OBFormat srcFormat = OB_FORMAT_UNKNOWN;
    int width = 0;
    int height = 0;
    int fps = 30;
    std::string sensorTag;
    std::shared_ptr<char[]> fileBuf;
};

struct SensorFiles {
    std::shared_ptr<StreamEncoder> color;
    std::shared_ptr<StreamEncoder> depth;
    std::shared_ptr<StreamEncoder> ir;
    std::shared_ptr<StreamEncoder> irLeft;
    std::shared_ptr<StreamEncoder> irRight;
    std::shared_ptr<std::ofstream> depthRawFile;
    std::shared_ptr<std::ofstream> imuFile;
    std::mutex depthRawMtx;
    std::mutex imuMtx;

    std::map<OBFrameType, uint64_t> frameCounts;
    std::mutex countMtx;
};

bool isH264KeyFrame(const uint8_t *data, uint32_t size);

void writeH264StartCode(std::ofstream &f);

void writeH264Frame(std::ofstream &file, const uint8_t *data, uint32_t size,
                     bool &keyFrameWritten, std::mutex &mtx);

void writeDepthRawWithHeader(std::ofstream &file, const uint8_t *data, uint32_t size,
                              int width, int height, float scale,
                              uint64_t frameIndex, std::mutex &mtx);

std::shared_ptr<std::ofstream> openBufferedFile(const std::string &path,
    std::ios_base::openmode mode = std::ios::binary,
    int bufSize = NIO_FILE_BUF_SIZE,
    std::shared_ptr<char[]> *outBuf = nullptr);

std::shared_ptr<StreamEncoder> createStreamEncoder(const std::string &filePath,
    OBFormat format, int w, int h, int fps,
    const char *seiUuid = "nio@orbbec-fusio",
    bool writeSEI = true);

void writeStreamFrame(StreamEncoder *se, const uint8_t *data, uint32_t size);

} // namespace nio
