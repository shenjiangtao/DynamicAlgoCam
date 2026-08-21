// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// sahi.cpp — SAHI C++ implementation.

#include "sahi.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace dynalgo {

SahiInference::SahiInference(const SahiConfig& cfg, InferFunc inferFunc)
    : cfg_(cfg), inferFunc_(std::move(inferFunc)) {}

std::vector<Slice> SahiInference::generateSlices(int imgW, int imgH) const {
    std::vector<Slice> slices;

    int sliceW = cfg_.sliceWidth;
    int sliceH = cfg_.sliceHeight;

    // Auto-adjust slice size if image is smaller
    if (cfg_.autoSliceResolution) {
        sliceW = std::min(sliceW, imgW);
        sliceH = std::min(sliceH, imgH);
    }

    int strideW = static_cast<int>(sliceW * (1.0f - cfg_.overlapWidthRatio));
    int strideH = static_cast<int>(sliceH * (1.0f - cfg_.overlapHeightRatio));
    strideW = std::max(1, strideW);
    strideH = std::max(1, strideH);

    for (int y = 0; y < imgH; y += strideH) {
        for (int x = 0; x < imgW; x += strideW) {
            int w = std::min(sliceW, imgW - x);
            int h = std::min(sliceH, imgH - y);

            if (w < 32 || h < 32) continue; // skip too small slices

            slices.push_back({x, y, w, h, imgW, imgH});

            if (slices.size() >= static_cast<size_t>(cfg_.maxSlices)) {
                return slices;
            }
        }
    }

    return slices;
}

std::vector<Slice> SahiInference::getSlices(int imgW, int imgH) const {
    return generateSlices(imgW, imgH);
}

DynalgoFrame SahiInference::extractSlice(const DynalgoFrame& frame, const Slice& slice) const {
    DynalgoFrame sliceFrame = frame;
    sliceFrame.width = slice.w;
    sliceFrame.height = slice.h;

    // Only support BGR/RGB/RGB888 for now
    int channels = (frame.format == DynalgoFormat::BGR ||
                    frame.format == DynalgoFormat::RGB ||
                    frame.format == DynalgoFormat::RGB888) ? 3 : 1;
    int srcStride = frame.width * channels;
    int dstStride = slice.w * channels;

    sliceFrame.data.resize(slice.h * dstStride);

    const uint8_t* src = frame.data.data();
    uint8_t* dst = sliceFrame.data.data();

    for (int y = 0; y < slice.h; ++y) {
        int srcY = slice.y + y;
        if (srcY >= frame.height) break;
        std::memcpy(dst + y * dstStride,
                    src + srcY * srcStride + slice.x * channels,
                    slice.w * channels);
    }

    return sliceFrame;
}

DynalgoDetectionResult SahiInference::mapDetection(const DynalgoDetectionResult& det,
                                                    const Slice& slice) const {
    DynalgoDetectionResult mapped = det;
    mapped.x += slice.x;
    mapped.y += slice.y;
    // w, h unchanged
    return mapped;
}

std::vector<int> SahiInference::nmsMerge(const std::vector<SahiDetection>& dets,
                                          float iouThr, float confThr) const {
    struct Box {
        float x1, y1, x2, y2, score;
        int classId;
        int idx;
    };

    std::vector<Box> boxes;
    boxes.reserve(dets.size());

    for (size_t i = 0; i < dets.size(); ++i) {
        const auto& d = dets[i].detection;
        if (d.score < confThr) continue;
        boxes.push_back({d.x, d.y, d.x + d.w, d.y + d.h, d.score, d.classId, static_cast<int>(i)});
    }

    std::sort(boxes.begin(), boxes.end(),
        [](const Box& a, const Box& b) { return a.score > b.score; });

    std::vector<int> keep;
    std::vector<bool> suppressed(boxes.size(), false);

    for (size_t i = 0; i < boxes.size(); ++i) {
        if (suppressed[i]) continue;
        keep.push_back(boxes[i].idx);

        for (size_t j = i + 1; j < boxes.size(); ++j) {
            if (suppressed[j]) continue;
            if (boxes[i].classId != boxes[j].classId) continue;

            float x1 = std::max(boxes[i].x1, boxes[j].x1);
            float y1 = std::max(boxes[i].y1, boxes[j].y1);
            float x2 = std::min(boxes[i].x2, boxes[j].x2);
            float y2 = std::min(boxes[i].y2, boxes[j].y2);

            float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
            float area1 = (boxes[i].x2 - boxes[i].x1) * (boxes[i].y2 - boxes[i].y1);
            float area2 = (boxes[j].x2 - boxes[j].x1) * (boxes[j].y2 - boxes[j].y1);
            float iou = inter / (area1 + area2 - inter + 1e-6f);

            if (iou > iouThr) suppressed[j] = true;
        }
    }

    return keep;
}

std::vector<DynalgoDetectionResult> SahiInference::mergeDetections(
    const std::vector<SahiDetection>& sliceDetections,
    int srcW, int srcH) const {

    // Filter by confidence
    std::vector<SahiDetection> filtered;
    filtered.reserve(sliceDetections.size());
    for (const auto& d : sliceDetections) {
        if (d.detection.score >= cfg_.confThreshold) {
            filtered.push_back(d);
        }
    }

    if (filtered.empty()) return {};

    // Apply NMS across all slices
    std::vector<int> keep = nmsMerge(filtered, cfg_.iouThreshold, cfg_.confThreshold);

    std::vector<DynalgoDetectionResult> merged;
    merged.reserve(keep.size());
    for (int idx : keep) {
        merged.push_back(filtered[idx].detection);
    }

    return merged;
}

std::vector<DynalgoDetectionResult> SahiInference::infer(const DynalgoFrame& frame) {
    std::vector<Slice> slices = generateSlices(frame.width, frame.height);

    if (slices.empty()) {
        // Fallback: run on full image
        std::vector<DynalgoDetectionResult> out;
        inferFunc_(frame, out);
        return out;
    }

    std::vector<SahiDetection> allDetections;
    allDetections.reserve(slices.size() * 10);

    for (size_t i = 0; i < slices.size(); ++i) {
        DynalgoFrame sliceFrame = extractSlice(frame, slices[i]);
        std::vector<DynalgoDetectionResult> sliceOut;

        if (inferFunc_(sliceFrame, sliceOut)) {
            for (auto& det : sliceOut) {
                if (det.score >= cfg_.confThreshold) {
                    allDetections.push_back({mapDetection(det, slices[i]), static_cast<int>(i)});
                }
            }
        }
    }

    return mergeDetections(allDetections, frame.width, frame.height);
}

} // namespace dynalgo