// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// preprocessing.cpp — Shared preprocessing implementation.

#include "preprocessing.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>

namespace dynalgo {

LetterboxResult computeLetterbox(int srcW, int srcH, int dstW, int dstH) {
    LetterboxResult lb;
    float scale = std::min(static_cast<float>(dstW) / srcW,
                          static_cast<float>(dstH) / srcH);
    lb.newW = static_cast<int>(std::round(srcW * scale));
    lb.newH = static_cast<int>(std::round(srcH * scale));
    lb.padLeft = (dstW - lb.newW) / 2;
    lb.padTop = (dstH - lb.newH) / 2;
    lb.scale = scale;
    return lb;
}

static void resizeBilinear(const uint8_t* src, int srcW, int srcH, int srcStride,
                          uint8_t* dst, int dstW, int dstH, int dstStride) {
    float invScaleX = static_cast<float>(srcW) / dstW;
    float invScaleY = static_cast<float>(srcH) / dstH;

    for (int y = 0; y < dstH; ++y) {
        float srcY = (y + 0.5f) * invScaleY - 0.5f;
        int y0 = std::max(0, std::min(srcH - 1, static_cast<int>(std::floor(srcY))));
        int y1 = std::max(0, std::min(srcH - 1, y0 + 1));
        float wy = srcY - y0;

        for (int x = 0; x < dstW; ++x) {
            float srcX = (x + 0.5f) * invScaleX - 0.5f;
            int x0 = std::max(0, std::min(srcW - 1, static_cast<int>(std::floor(srcX))));
            int x1 = std::max(0, std::min(srcW - 1, x0 + 1));
            float wx = srcX - x0;

            for (int c = 0; c < 3; ++c) {
                float v00 = src[y0 * srcStride + x0 * 3 + c];
                float v01 = src[y0 * srcStride + x1 * 3 + c];
                float v10 = src[y1 * srcStride + x0 * 3 + c];
                float v11 = src[y1 * srcStride + x1 * 3 + c];

                float v0 = v00 * (1 - wx) + v01 * wx;
                float v1 = v10 * (1 - wx) + v11 * wx;
                float v = v0 * (1 - wy) + v1 * wy;

                dst[y * dstStride + x * 3 + c] = static_cast<uint8_t>(std::round(v));
            }
        }
    }
}

std::vector<float> preprocessFrame(const DynalgoFrame& frame,
                                   const PreprocessConfig& cfg,
                                   std::optional<LetterboxResult>* lbOut) {
    // Only support Y16 (depth) or BGR/RGB color frames for now
    if (frame.format != DynalgoFormat::BGR &&
        frame.format != DynalgoFormat::RGB &&
        frame.format != DynalgoFormat::RGB888 &&
        frame.format != DynalgoFormat::Y16) {
        return {};
    }

    int srcW = frame.width;
    int srcH = frame.height;
    int channels = (frame.format == DynalgoFormat::Y16) ? 1 : 3;

    LetterboxResult lb;
    if (cfg.letterbox) {
        lb = computeLetterbox(srcW, srcH, cfg.inputWidth, cfg.inputHeight);
    } else {
        lb = {cfg.inputWidth, cfg.inputHeight, 0, 0,
              std::min(static_cast<float>(cfg.inputWidth) / srcW,
                       static_cast<float>(cfg.inputHeight) / srcH)};
    }

    if (lbOut) *lbOut = lb;

    // Step 1: Resize with letterbox padding
    std::vector<uint8_t> resized(cfg.inputHeight * cfg.inputWidth * channels);
    std::fill(resized.begin(), resized.end(), 114); // padding value (YOLO default)

    if (frame.format == DynalgoFormat::Y16) {
        // Depth: 16-bit -> 8-bit for visualization, or keep 16-bit
        const uint16_t* src16 = reinterpret_cast<const uint16_t*>(frame.data.data());
        for (int y = 0; y < lb.newH; ++y) {
            int srcY = static_cast<int>(std::round(y / lb.scale));
            if (srcY >= srcH) srcY = srcH - 1;
            for (int x = 0; x < lb.newW; ++x) {
                int srcX = static_cast<int>(std::round(x / lb.scale));
                if (srcX >= srcW) srcX = srcW - 1;
                resized[(lb.padTop + y) * cfg.inputWidth + lb.padLeft + x] =
                    static_cast<uint8_t>(src16[srcY * srcW + srcX] >> 8);
            }
        }
    } else {
        // Color: BGR/RGB 24-bit
        const uint8_t* src = frame.data.data();
        int srcStride = srcW * 3;
        for (int y = 0; y < lb.newH; ++y) {
            int srcY = static_cast<int>(std::round(y / lb.scale));
            if (srcY >= srcH) srcY = srcH - 1;
            int dstOffset = ((lb.padTop + y) * cfg.inputWidth + lb.padLeft) * 3;
            int srcOffset = srcY * srcStride;
            if (cfg.bgrToRgb) {
                // BGR -> RGB during copy
                for (int x = 0; x < lb.newW; ++x) {
                    resized[dstOffset + x * 3 + 0] = src[srcOffset + x * 3 + 2]; // R
                    resized[dstOffset + x * 3 + 1] = src[srcOffset + x * 3 + 1]; // G
                    resized[dstOffset + x * 3 + 2] = src[srcOffset + x * 3 + 0]; // B
                }
            } else {
                std::memcpy(&resized[dstOffset], &src[srcOffset], lb.newW * 3);
            }
        }
    }

    // Step 2: Convert to float tensor (CHW or HWC)
    std::vector<float> tensor(cfg.inputHeight * cfg.inputWidth * channels);

    if (cfg.hwcToChw) {
        // HWC -> CHW
        for (int c = 0; c < channels; ++c) {
            for (int y = 0; y < cfg.inputHeight; ++y) {
                for (int x = 0; x < cfg.inputWidth; ++x) {
                    int hwcIdx = (y * cfg.inputWidth + x) * channels + c;
                    int chwIdx = c * cfg.inputHeight * cfg.inputWidth + y * cfg.inputWidth + x;
                    float val = resized[hwcIdx];
                    if (cfg.normalize) {
                        val = (val / 255.0f - cfg.mean[c]) / cfg.std[c];
                    }
                    tensor[chwIdx] = val;
                }
            }
        }
    } else {
        // Keep HWC
        for (size_t i = 0; i < resized.size(); ++i) {
            float val = resized[i];
            int c = i % channels;
            if (cfg.normalize) {
                val = (val / 255.0f - cfg.mean[c]) / cfg.std[c];
            }
            tensor[i] = val;
        }
    }

    return tensor;
}

// CPU NMS implementation
std::vector<int> nmsCPU(const std::vector<DetectionBox>& boxes, const NMSConfig& cfg) {
    std::vector<int> indices(boxes.size());
    std::iota(indices.begin(), indices.end(), 0);

    // Sort by confidence descending
    std::sort(indices.begin(), indices.end(),
        [&boxes](int a, int b) { return boxes[a].score > boxes[b].score; });

    std::vector<int> keep;
    std::vector<bool> suppressed(boxes.size(), false);

    for (size_t i = 0; i < indices.size(); ++i) {
        int idx = indices[i];
        if (suppressed[idx]) continue;
        if (boxes[idx].score < cfg.confThreshold) break;

        keep.push_back(idx);
        if (keep.size() >= static_cast<size_t>(cfg.maxDetections)) break;

        for (size_t j = i + 1; j < indices.size(); ++j) {
            int idx2 = indices[j];
            if (suppressed[idx2]) continue;

            if (cfg.classAgnostic || boxes[idx].classId == boxes[idx2].classId) {
                // Compute IoU
                float x1 = std::max(boxes[idx].x1, boxes[idx2].x1);
                float y1 = std::max(boxes[idx].y1, boxes[idx2].y1);
                float x2 = std::min(boxes[idx].x2, boxes[idx2].x2);
                float y2 = std::min(boxes[idx].y2, boxes[idx2].y2);

                float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
                float area1 = (boxes[idx].x2 - boxes[idx].x1) * (boxes[idx].y2 - boxes[idx].y1);
                float area2 = (boxes[idx2].x2 - boxes[idx2].x1) * (boxes[idx2].y2 - boxes[idx2].y1);
                float iou = inter / (area1 + area2 - inter + 1e-6f);

                if (iou > cfg.iouThreshold) {
                    suppressed[idx2] = true;
                }
            }
        }
    }

    return keep;
}

std::vector<DynalgoDetectionResult> postprocessDetections(
    const float* output,
    int numDetections,
    int numClasses,
    const LetterboxResult& lb,
    const NMSConfig& cfg,
    int srcW, int srcH,
    bool xywhFormat) {

    std::vector<DetectionBox> boxes;
    boxes.reserve(numDetections);

    for (int i = 0; i < numDetections; ++i) {
        const float* det = output + i * (5 + numClasses); // x,y,w,h,conf + class scores

        float cx = det[0];
        float cy = det[1];
        float w = det[2];
        float h = det[3];
        float conf = det[4];

        if (conf < cfg.confThreshold) continue;

        // Find best class
        int bestClass = 0;
        float bestScore = 0.0f;
        for (int c = 0; c < numClasses; ++c) {
            float score = det[5 + c] * conf;
            if (score > bestScore) {
                bestScore = score;
                bestClass = c;
            }
        }

        if (bestScore < cfg.confThreshold) continue;

        float x1, y1, x2, y2;
        if (xywhFormat) {
            x1 = cx - w * 0.5f;
            y1 = cy - h * 0.5f;
            x2 = cx + w * 0.5f;
            y2 = cy + h * 0.5f;
        } else {
            x1 = cx; y1 = cy; x2 = w; y2 = h;
        }

        // Remove letterbox padding and scale back to original
        x1 = (x1 - lb.padLeft) / lb.scale;
        y1 = (y1 - lb.padTop) / lb.scale;
        x2 = (x2 - lb.padLeft) / lb.scale;
        y2 = (y2 - lb.padTop) / lb.scale;

        // Clamp to image bounds
        x1 = std::max(0.0f, std::min(static_cast<float>(srcW), x1));
        y1 = std::max(0.0f, std::min(static_cast<float>(srcH), y1));
        x2 = std::max(0.0f, std::min(static_cast<float>(srcW), x2));
        y2 = std::max(0.0f, std::min(static_cast<float>(srcH), y2));

        if (x2 <= x1 || y2 <= y1) continue;

        boxes.push_back({x1, y1, x2, y2, bestScore, bestClass});
    }

    // Apply NMS
    std::vector<int> keep = nmsCPU(boxes, cfg);

    // Convert to DynalgoDetectionResult
    std::vector<DynalgoDetectionResult> results;
    results.reserve(keep.size());
    for (int idx : keep) {
        const auto& b = boxes[idx];
        DynalgoDetectionResult r;
        r.classId = b.classId;
        r.score = b.score;
        r.x = b.x1;
        r.y = b.y1;
        r.w = b.x2 - b.x1;
        r.h = b.y2 - b.y1;
        results.push_back(r);
    }

    return results;
}

} // namespace dynalgo