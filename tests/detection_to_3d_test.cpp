// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// detection_to_3d_test.cpp — Unit tests for detectionCenterToCamera3D().
//
// Two cases (per IMPLEMENTATION_TASKS.md B2.1):
//   (a) detection perfectly centred on a 1280x800 frame, all depth = 1000mm
//       → expected back-projection (0, 0, 1.0) metres.
//   (b) detection offset from the principal point on a 1280x800 frame, all
//       depth = 2000mm → expected back-projection matches the closed-form
//       pinhole predicted value within Euclidean error < 1e-4.
//
// Build: only built when root CMake option BUILD_TESTS=ON and GTest is found;
// otherwise the root CMake skips the entire `tests/` subdirectory (see root
// CMakeLists.txt lines 107-116). When built, this TU exercises the header
// in `app/core/dynalgo_detection_to_3d.hpp` by populating DynalgoFrame /
// DynalgoIntrinsic / DynalgoDetectionResult by hand (no SDK needed).

#include "dynalgo_detection_to_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace {

// Build a 1280x800 Y16 depth frame filled with `depthMm` everywhere.
// Returns a DynalgoFrame with format=Y16, width=1280, height=800, and a
// w*h*2-byte `data` buffer holding `depthMm` in every pixel.
DynalgoFrame makeUniformDepth(int w, int h, uint16_t depthMm)
{
    DynalgoFrame f;
    f.type = DynalgoFrameType::DEPTH;
    f.format = DynalgoFormat::Y16;
    f.width = w;
    f.height = h;
    f.depthScale = 0.001f;  // Y16 millimetres → metres
    f.data.resize(static_cast<size_t>(w) * h * sizeof(uint16_t));
    auto* y16 = reinterpret_cast<uint16_t*>(f.data.data());
    for (size_t i = 0; i < static_cast<size_t>(w) * h; ++i)
        y16[i] = depthMm;
    f.present = true;
    return f;
}

} // namespace

// Centre detection on a 1280x800 frame, uniform depth 1000 mm → expect (0,0,1.0) m.
TEST(DetectionTo3D, CenteredBoxUniformDepth1m)
{
    const int w = 1280, h = 800;
    auto depth = makeUniformDepth(w, h, 1000);

    DynalgoIntrinsic k;
    k.fx = 640.0f;
    k.fy = 640.0f;
    k.cx = 640.0f;
    k.cy = 400.0f;
    k.width = w;
    k.height = h;

    DynalgoDetectionResult det;
    det.x = 600.0f;   // box top-left X
    det.y = 360.0f;   // box top-left Y
    det.w = 80.0f;
    det.h = 80.0f;    // so centre = (640, 400) == principal point exactly
    det.score = 1.0f;

    float X = -1.0f, Y = -1.0f, Z = -1.0f;
    bool ok = detectionCenterToCamera3D(depth, k, 0.001f, 0, det, X, Y, Z);

    ASSERT_TRUE(ok);
    EXPECT_NEAR(X, 0.0f, 1e-5f);
    EXPECT_NEAR(Y, 0.0f, 1e-5f);
    EXPECT_NEAR(Z, 1.0f, 1e-5f);
}

// Offset detection on a 1280x800 frame, uniform depth 2000 mm → expect
// back-projected point matches closed-form prediction within Euclidean error < 1e-4.
TEST(DetectionTo3D, OffsetBoxUniformDepth2m)
{
    const int w = 1280, h = 800;
    auto depth = makeUniformDepth(w, h, 2000);

    DynalgoIntrinsic k;
    k.fx = 700.0f;
    k.fy = 700.0f;
    k.cx = 640.0f;
    k.cy = 400.0f;
    k.width = w;
    k.height = h;

    DynalgoDetectionResult det;
    det.x = 730.0f;  // box top-left X
    det.y = 360.0f;  // box top-left Y
    det.w = 60.0f;
    det.h = 80.0f;   // centre = (760, 400)

    // Closed-form expected value:
    //   Z = 2000 * 0.001 = 2.0
    //   X = (u - cx) * Z / fx = (760 - 640) * 2 / 700 = 240/700 ≈ 0.342857142857
    //   Y = (v - cy) * Z / fy = (400 - 400) * 2 / 700 = 0
    const float expZ = 2.0f;
    const float expX = (760.0f - 640.0f) * expZ / 700.0f;
    const float expY = 0.0f;

    float X = -1.0f, Y = -1.0f, Z = -1.0f;
    bool ok = detectionCenterToCamera3D(depth, k, 0.001f, 0, det, X, Y, Z);

    ASSERT_TRUE(ok);
    const float dx = X - expX, dy = Y - expY, dz = Z - expZ;
    const float euc = std::sqrt(dx * dx + dy * dy + dz * dz);
    EXPECT_LT(euc, 1e-4f);
}

// Centre inside valid grid but a uniform-zero depth window  → function returns
// false ("no 3D fix this frame"); out-params left unchanged.
TEST(DetectionTo3D, AllZeroDepthReturnsFalse)
{
    const int w = 640, h = 480;
    auto depth = makeUniformDepth(w, h, /*depthMm=*/0);

    DynalgoIntrinsic k;
    k.fx = k.fy = 500.0f;
    k.cx = 320.0f;
    k.cy = 240.0f;
    k.width = w;
    k.height = h;

    DynalgoDetectionResult det;
    det.x = 280; det.y = 200; det.w = 80; det.h = 80;

    float X = 42.0f, Y = 43.0f, Z = 44.0f;
    bool ok = detectionCenterToCamera3D(depth, k, 0.001f, 0, det, X, Y, Z);
    EXPECT_FALSE(ok);
    EXPECT_FLOAT_EQ(X, 42.0f);  // unchanged
    EXPECT_FLOAT_EQ(Y, 43.0f);
    EXPECT_FLOAT_EQ(Z, 44.0f);
}

// Wrong format (RGB instead of Y16) → false.
TEST(DetectionTo3D, WrongFormatReturnsFalse)
{
    const int w = 640, h = 480;
    DynalgoFrame depth;
    depth.type = DynalgoFrameType::DEPTH;
    depth.format = DynalgoFormat::RGB;   // NOT Y16 — should reject
    depth.width = w;
    depth.height = h;
    depth.depthScale = 0.001f;
    depth.data.resize(static_cast<size_t>(w) * h * 3, 0x80);

    DynalgoIntrinsic k;
    k.fx = k.fy = 500.0f;
    k.cx = 320.0f;
    k.cy = 240.0f;
    k.width = w;
    k.height = h;

    DynalgoDetectionResult det;
    det.x = 280; det.y = 200; det.w = 80; det.h = 80;

    float X, Y, Z;
    EXPECT_FALSE(detectionCenterToCamera3D(depth, k, 0.001f, 0, det, X, Y, Z));
}

// Median window (filterHalf=1) should stay deterministic on uniform depth.
TEST(DetectionTo3D, MedianWindowUniformDepth)
{
    const int w = 320, h = 240;
    auto depth = makeUniformDepth(w, h, /*depthMm=*/500);

    DynalgoIntrinsic k;
    k.fx = k.fy = 400.0f;
    k.cx = 160.0f;
    k.cy = 120.0f;
    k.width = w;
    k.height = h;

    DynalgoDetectionResult det;
    det.x = 140; det.y = 100; det.w = 40; det.h = 40;  // centre = (160,120)

    float X, Y, Z;
    bool ok = detectionCenterToCamera3D(depth, k, 0.001f, /*filterHalf=*/1, det, X, Y, Z);
    ASSERT_TRUE(ok);
    EXPECT_NEAR(X, 0.0f, 1e-5f);
    EXPECT_NEAR(Y, 0.0f, 1e-5f);
    EXPECT_NEAR(Z, 0.5f, 1e-5f);  // 500 mm * 0.001 = 0.5 m
}
