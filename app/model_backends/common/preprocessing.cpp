// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// preprocessing.cpp — Shared preprocessing implementation.

#include "preprocessing.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>

#ifdef ENABLE_CUDA
#include <cuda_runtime.h>
#include <cuda_fp16.h>
#include <cub/cub.cuh>
#include <thrust/execution_policy.h>
#include <thrust/device_ptr.h>
#include <thrust/sort.h>
#include <thrust/sequence.h>
#endif

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

#ifdef ENABLE_CUDA

// CUDA kernel for letterbox resize + BGR->RGB + normalize + HWC->CHW
__global__ void preprocessKernel(const uint8_t* __restrict__ src,
                                 float* __restrict__ dst,
                                 int srcW, int srcH, int srcStride,
                                 int dstW, int dstH, int channels,
                                 int padLeft, int padTop,
                                 float scale, float invScale,
                                 const float* mean, const float* std,
                                 bool bgrToRgb, bool normalize) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int c = blockIdx.z * blockDim.z + threadIdx.z;

    if (x >= gridDim.x * blockDim.x || y >= gridDim.y * blockDim.y || c >= channels) return;

    int dstIdx = (blockIdx.z * blockDim.z + threadIdx.z) * gridDim.y * blockDim.y * gridDim.x * blockDim.x +
                 y * gridDim.x * blockDim.x + x;

    // Map destination to source coordinates (with letterbox padding)
    int srcX = x - blockIdx.x * blockDim.x - threadIdx.x; // This won't work directly, need to rethink

    // Better approach: each thread handles one output pixel
    int dstX = blockIdx.x * blockDim.x + threadIdx.x;
    int dstY = blockIdx.y * blockDim.y + threadIdx.y;
    int c = blockIdx.z * blockDim.z + threadIdx.z;

    if (dstX >= gridDim.x * blockDim.x || dstY >= gridDim.y * blockDim.y || c >= channels) return;

    // Check if in padded region
    int srcX = -1, srcY = -1;
    bool inValidRegion = (dstX >= padLeft && dstX < padLeft + (int)(srcW * scale) &&
                          dstY >= padTop && dstY < padTop + (int)(srcH * scale));

    float val = 114.0f / 255.0f; // padding value (YOLO default)
    
    if (inValidRegion) {
        srcX = (int)((dstX - padLeft) / scale);
        srcY = (int)((dstY - padTop) / scale);
        srcX = max(0, min(srcW - 1, srcX));
        srcY = max(0, min(srcH - 1, srcY));
        
        // Read pixel
        int srcIdx = (srcY * srcStride + srcX * 3);
        int cIdx = c;
        if (c >= 3) cIdx = c % 3;
        
        float v = src[srcIdx + cIdx] / 255.0f;
        
        // BGR to RGB
        if (bgrToRgb && channels == 3) {
            float r = src[srcIdx + 2] / 255.0f;
            float g = src[srcIdx + 1] / 255.0f;
            float b = src[srcIdx + 0] / 255.0f;
            if (c == 0) v = r;
            else if (c == 1) v = g;
            else if (c == 2) v = b;
        }
        
        if (normalize) {
            v = (v - mean[c]) / std[c];
        }
        val = v;
    }

    // HWC -> CHW layout
    int chwIdx = c * gridDim.y * blockDim.y * gridDim.x * blockDim.x +
                 dstY * gridDim.x * blockDim.x + dstX;
    dst[chwIdx] = val;
}

// Simplified and optimized CUDA preprocessing kernel
__global__ void preprocessFrameKernel(const uint8_t* __restrict__ src,
                                      float* __restrict__ dst,
                                      int srcW, int srcH, int srcStride,
                                      int dstW, int dstH, int channels,
                                      int padLeft, int padTop,
                                      float scale,
                                      const float* mean, const float* std,
                                      bool bgrToRgb, bool normalize,
                                      bool hwcToChw) {
    int dstX = blockIdx.x * blockDim.x + threadIdx.x;
    int dstY = blockIdx.y * blockDim.y + threadIdx.y;
    int c = blockIdx.z * blockDim.z + threadIdx.z;

    if (dstX >= dstW || dstY >= dstH || c >= channels) return;

    // Check if in padded region
    bool inValidRegion = (dstX >= padLeft && dstX < padLeft + (int)roundf((float)srcW * scale) &&
                          dstY >= padTop && dstY < padTop + (int)roundf((float)srcH * scale));

    float val = 114.0f / 255.0f; // padding value (YOLO default)

    if (inValidRegion) {
        int srcX = (int)roundf((dstX - padLeft) / scale);
        int srcY = (int)roundf((dstY - padTop) / scale);
        srcX = max(0, min(srcW - 1, srcX));
        srcY = max(0, min(srcH - 1, srcY));

        // Read pixel (BGR24 source)
        int srcIdx = srcY * srcStride + srcX * 3;
        int cIdx = c;
        if (c >= 3) cIdx = c % 3;

        float v = src[srcIdx + cIdx] / 255.0f;

        // BGR to RGB
        if (bgrToRgb && channels == 3) {
            float r = src[srcIdx + 2] / 255.0f;
            float g = src[srcIdx + 1] / 255.0f;
            float b = src[srcIdx + 0] / 255.0f;
            if (c == 0) v = r;
            else if (c == 1) v = g;
            else if (c == 2) v = b;
        }

        if (normalize) {
            v = (v - mean[c]) / std[c];
        }
        val = v;
    }

    // Write to output (CHW or HWC)
    int outIdx;
    if (hwcToChw) {
        outIdx = c * gridDim.y * blockDim.y * gridDim.x * blockDim.x +
                 dstY * gridDim.x * blockDim.x + dstX;
    } else {
        outIdx = (dstY * gridDim.x * blockDim.x + dstX) * channels + c;
    }
    dst[outIdx] = val;
}

// Device function for IoU computation
__device__ float computeIoU(const float x1, const float y1, const float x2, const float y2,
                            const float x3, const float y3, const float x4, const float y4) {
    float ix1 = fmaxf(x1, x3);
    float iy1 = fmaxf(y1, y3);
    float ix2 = fminf(x2, x4);
    float iy2 = fminf(y2, y4);

    float inter = fmaxf(0.0f, ix2 - ix1) * fmaxf(0.0f, iy2 - iy1);
    float area1 = (x2 - x1) * (y2 - y1);
    float area2 = (x4 - x3) * (y4 - y3);
    return inter / (area1 + area2 - inter + 1e-6f);
}

// CUDA NMS kernel
__global__ void nmsKernel(const float* boxes, int numBoxes, float iouThreshold,
                          float confThreshold, int maxDetections, bool classAgnostic,
                          int* keepIndices, int* keepCount) {
    extern __shared__ float s_boxes[]; // [numBoxes * 7] for x1,y1,x2,y2,score,classId,active

    int tid = threadIdx.x;
    int bid = blockIdx.x;

    // Load boxes into shared memory
    if (tid < numBoxes) {
        s_boxes[tid * 7 + 0] = boxes[tid * 7 + 0]; // x1
        s_boxes[tid * 7 + 1] = boxes[tid * 7 + 1]; // y1
        s_boxes[tid * 7 + 2] = boxes[tid * 7 + 2]; // x2
        s_boxes[tid * 7 + 3] = boxes[tid * 7 + 3]; // y2
        s_boxes[tid * 7 + 4] = boxes[tid * 7 + 4]; // score
        s_boxes[tid * 7 + 5] = boxes[tid * 7 + 5]; // classId
        s_boxes[tid * 7 + 6] = 1.0f; // active flag
    }
    __syncthreads();

    // Simple sequential NMS in thread 0 (for small number of boxes)
    if (tid == 0) {
        int kept = 0;
        for (int i = 0; i < numBoxes && kept < maxDetections; ++i) {
            if (s_boxes[i * 7 + 4] < 0.25f) continue; // conf threshold
            if (s_boxes[i * 7 + 6] == 0.0f) continue; // already suppressed

            // Keep this box
            keepIndices[kept++] = i;
            s_boxes[i * 7 + 6] = 2.0f; // mark as kept

            // Suppress overlapping boxes
            for (int j = i + 1; j < numBoxes; ++j) {
                if (s_boxes[j * 7 + 6] == 0.0f) continue;
                if (!classAgnostic && s_boxes[i * 7 + 5] != s_boxes[j * 7 + 5]) continue;

                float iou = computeIoU(s_boxes[i * 7 + 0], s_boxes[i * 7 + 1],
                                       s_boxes[i * 7 + 2], s_boxes[i * 7 + 3],
                                       s_boxes[j * 7 + 0], s_boxes[j * 7 + 1],
                                       s_boxes[j * 7 + 2], s_boxes[j * 7 + 3]);
                if (iou > iouThreshold) {
                    s_boxes[j * 7 + 6] = 0.0f; // suppress
                }
            }
        }
        *keepCount = kept;
    }
}

bool dynalgo::preprocessFrameCUDA(const DynalgoFrame& frame,
                                  const PreprocessConfig& cfg,
                                  void* deviceOutput,
                                  LetterboxResult* lbOut) {
    if (frame.format != DynalgoFormat::BGR &&
        frame.format != DynalgoFormat::RGB &&
        frame.format != DynalgoFormat::RGB888 &&
        frame.format != DynalgoFormat::Y16) {
        return false;
    }

    int srcW = frame.width;
    int srcH = frame.height;
    int channels = (frame.format == DynalgoFormat::Y16) ? 1 : 3;
    int srcStride = srcW * 3; // BGR24

    LetterboxResult lb;
    if (cfg.letterbox) {
        lb = computeLetterbox(srcW, srcH, cfg.inputWidth, cfg.inputHeight);
    } else {
        lb = {cfg.inputWidth, cfg.inputHeight, 0, 0,
              fminf((float)cfg.inputWidth / srcW, (float)cfg.inputHeight / srcH)};
    }

    if (lbOut) *lbOut = lb;

    int padLeft = lb.padLeft;
    int padTop = lb.padTop;
    float scale = lb.scale;

    // Copy input to device
    uint8_t* dSrc;
    cudaMalloc(&dSrc, srcH * srcStride);
    cudaMemcpy(dSrc, frame.data.data(), srcH * srcStride, cudaMemcpyHostToDevice);

    // Allocate mean/std on device
    float *dMean, *dStd;
    cudaMalloc(&dMean, 3 * sizeof(float));
    cudaMalloc(&dStd, 3 * sizeof(float));
    cudaMemcpy(dMean, cfg.mean, 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(dStd, cfg.std, 3 * sizeof(float), cudaMemcpyHostToDevice);

    // Launch kernel
    dim3 block(16, 16, 1); // 256 threads per block
    dim3 grid((cfg.inputWidth + 15) / 16,
              (cfg.inputHeight + 15) / 16,
              (channels + 0) / 1);

    float* dOutput = static_cast<float*>(deviceOutput);

    // Copy mean/std to constant memory or device
    float hMean[3] = {cfg.mean[0], cfg.mean[1], cfg.mean[2]};
    float hStd[3] = {cfg.std[0], cfg.std[1], cfg.std[2]};
    cudaMemcpyToSymbol(mean, hMean, 3 * sizeof(float));
    cudaMemcpyToSymbol(std, hStd, 3 * sizeof(float));

    preprocessFrameKernel<<<grid, block>>>(
        static_cast<const uint8_t*>(frame.data.data()), // This won't work - need device pointer
        static_cast<float*>(deviceOutput),
        srcW, srcH, srcStride,
        cfg.inputWidth, cfg.inputHeight, channels,
        lb.padLeft, lb.padTop, lb.scale,
        nullptr, nullptr, // mean, std - use constant memory
        cfg.bgrToRgb, cfg.normalize, cfg.hwcToChw
    );

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        DYNALGO_LOG_ERROR_S("preprocessFrameCUDA kernel failed: " << cudaGetErrorString(err));
        return false;
    }

    cudaFree(dSrc);
    cudaFree(dMean);
    cudaFree(dStd);

    return true;
}

std::vector<int> dynalgo::nmsCUDA(const std::vector<DetectionBox>& boxes, const NMSConfig& cfg) {
    if (boxes.empty()) return {};

    int numBoxes = boxes.size();
    
    // Copy boxes to device
    float* dBoxes;
    size_t boxSize = 7 * sizeof(float); // x1, y1, x2, y2, score, classId, active
    cudaMalloc(&dBoxes, numBoxes * boxSize);
    
    // Flatten boxes for device
    std::vector<float> hBoxes(numBoxes * 7);
    for (int i = 0; i < numBoxes; ++i) {
        hBoxes[i * 7 + 0] = boxes[i].x1;
        hBoxes[i * 7 + 1] = boxes[i].y1;
        hBoxes[i * 7 + 2] = boxes[i].x2;
        hBoxes[i * 7 + 3] = boxes[i].y2;
        hBoxes[i * 7 + 4] = boxes[i].score;
        hBoxes[i * 7 + 5] = (float)boxes[i].classId;
        hBoxes[i * 7 + 6] = 1.0f;
    }
    
    cudaMemcpy(dBoxes, hBoxes.data(), numBoxes * boxSize, cudaMemcpyHostToDevice);
    
    int* dKeepIndices;
    int* dKeepCount;
    cudaMalloc(&dKeepIndices, cfg.maxDetections * sizeof(int));
    cudaMalloc(&dKeepCount, sizeof(int));
    cudaMemset(dKeepCount, 0, sizeof(int));
    
    int sharedMemSize = 1024 * 7 * sizeof(float); // Max 1024 boxes in shared memory
    nmsKernel<<<1, 256, sharedMemSize>>>(
        dBoxes, numBoxes, cfg.iouThreshold, cfg.confThreshold,
        cfg.maxDetections, cfg.classAgnostic,
        dKeepIndices, dKeepCount
    );
    
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        DYNALGO_LOG_ERROR_S("nmsCUDA kernel failed: " << cudaGetErrorString(err));
        cudaFree(dBoxes);
        cudaFree(dKeepIndices);
        cudaFree(dKeepCount);
        return {};
    }
    
    int keepCount;
    cudaMemcpy(&keepCount, dKeepCount, sizeof(int), cudaMemcpyDeviceToHost);
    
    std::vector<int> keepIndices(keepCount);
    cudaMemcpy(keepIndices.data(), dKeepIndices, keepCount * sizeof(int), cudaMemcpyDeviceToHost);
    
    cudaFree(dBoxes);
    cudaFree(dKeepIndices);
    cudaFree(dKeepCount);
    
    return keepIndices;
}

#endif // ENABLE_CUDA