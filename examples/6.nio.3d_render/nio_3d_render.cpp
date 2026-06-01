// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_3d_render.cpp — Software 3D point-cloud renderer with IMU-based
// camera pose tracking. Fuses color + depth into a rendered view,
// encoded to H.264. Uses shared nio:: utilities from examples/utils/.

#include <libobsensor/ObSensor.hpp>
#include "utils.hpp"
#include "nio_log.hpp"
#include "nio_common.hpp"
#include "nio_h264_encoder.hpp"
#include "nio_stream_io.hpp"
#include "nio_color_convert.hpp"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <map>
#include <vector>
#include <algorithm>
#include <cstring>
#include <csignal>
#include <chrono>
#include <cmath>
#include <numeric>

using namespace nio;

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3 &o) const { return Vec3(x+o.x, y+o.y, z+o.z); }
    Vec3 operator-(const Vec3 &o) const { return Vec3(x-o.x, y-o.y, z-o.z); }
    Vec3 operator*(float s) const { return Vec3(x*s, y*s, z*s); }
    float dot(const Vec3 &o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3 &o) const {
        return Vec3(y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x);
    }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const {
        float l = length();
        return l > 1e-9f ? Vec3(x/l, y/l, z/l) : Vec3(0, 1, 0);
    }
};

struct Mat3x3 {
    float m[3][3];
    Mat3x3() { memset(m, 0, sizeof(m)); m[0][0] = m[1][1] = m[2][2] = 1.0f; }
    static Mat3x3 identity() { return Mat3x3(); }
    static Mat3x3 rotationX(float angleRad) {
        Mat3x3 r;
        float c = std::cos(angleRad), s = std::sin(angleRad);
        r.m[0][0]=1; r.m[0][1]=0; r.m[0][2]=0;
        r.m[1][0]=0; r.m[1][1]=c; r.m[1][2]=-s;
        r.m[2][0]=0; r.m[2][1]=s; r.m[2][2]=c;
        return r;
    }
    static Mat3x3 rotationY(float angleRad) {
        Mat3x3 r;
        float c = std::cos(angleRad), s = std::sin(angleRad);
        r.m[0][0]=c; r.m[0][1]=0; r.m[0][2]=s;
        r.m[1][0]=0; r.m[1][1]=1; r.m[1][2]=0;
        r.m[2][0]=-s; r.m[2][1]=0; r.m[2][2]=c;
        return r;
    }
    static Mat3x3 rotationZ(float angleRad) {
        Mat3x3 r;
        float c = std::cos(angleRad), s = std::sin(angleRad);
        r.m[0][0]=c; r.m[0][1]=-s; r.m[0][2]=0;
        r.m[1][0]=s; r.m[1][1]=c; r.m[1][2]=0;
        r.m[2][0]=0; r.m[2][1]=0; r.m[2][2]=1;
        return r;
    }
    Mat3x3 operator*(const Mat3x3 &o) const {
        Mat3x3 r;
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) {
            r.m[i][j] = 0;
            for(int k=0;k<3;k++) r.m[i][j] += m[i][k] * o.m[k][j];
        }
        return r;
    }
    Vec3 operator*(const Vec3 &v) const {
        return Vec3(
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
        );
    }
};

struct Camera3D {
    Vec3 pos;
    Mat3x3 rot;
    float fov;
    float nearP, farP;

    Camera3D() : pos(0, -1.0f, 0), fov(70.0f), nearP(0.05f), farP(15.0f) {}

    Vec3 forward() const { return rot * Vec3(0, 0, 1); }
    Vec3 up() const { return rot * Vec3(0, 1, 0); }
    Vec3 right() const { return rot * Vec3(1, 0, 0); }
};

struct IMUState {
    std::mutex mtx;
    Vec3 accel;
    Vec3 gyro;
    bool valid;

    IMUState() : valid(false) {}
    void update(const Vec3 &a, const Vec3 &g) {
        std::lock_guard<std::mutex> lock(mtx);
        accel = a; gyro = g; valid = true;
    }
    bool get(Vec3 &a, Vec3 &g) {
        std::lock_guard<std::mutex> lock(mtx);
        a = accel; g = gyro; return valid;
    }
};

struct DeviceRender {
    std::shared_ptr<ob::Pipeline> videoPipeline;
    std::shared_ptr<ob::Pipeline> imuPipeline;
    std::string deviceName;
    bool hasIMU;
    float depthScale;
    OBFormat colorFormat;
    int colorW, colorH;
    int depthW, depthH;

    std::shared_ptr<MjpgDecoderRes> mjpgRes;
    std::shared_ptr<IMUState> imuState;
    std::shared_ptr<std::atomic<uint64_t>> renderFrameCount;

    std::shared_ptr<ob::Align> alignFilter;

    OBCameraIntrinsic depthIntrinsic;
    OBCameraDistortion depthDistortion;
    OBExtrinsic depthToColorExtrinsic;
    bool hasIntrinsics;

    std::mutex latestDataMtx;
    std::condition_variable latestDataCV;
    bool latestDataReady;
    std::vector<uint16_t> latestDepth;
    int latestDepthW, latestDepthH;
    float latestScale;
    std::vector<uint8_t> latestColorRGB;
    int latestColorW, latestColorH;
    std::vector<uint8_t> latestIRLeftData;
    int latestIRLeftW, latestIRLeftH;
    std::vector<uint8_t> latestIRRightData;
    int latestIRRightW, latestIRRightH;

    DeviceRender() : hasIMU(false), depthScale(0.001f), colorFormat(OB_FORMAT_UNKNOWN),
    colorW(0), colorH(0), depthW(0), depthH(0), hasIntrinsics(false),
    latestDataReady(false), latestDepthW(0), latestDepthH(0), latestScale(0.001f),
    latestColorW(0), latestColorH(0), latestIRLeftW(0), latestIRLeftH(0),
    latestIRRightW(0), latestIRRightH(0) {}
};

struct RenderConfig {
    int outW, outH, outFps;
    float autoRotateSpeed;
    float camOrbitRadius;
    float camHeight;
    float camFov;
    float depthMinM, depthMaxM;
    float depthAlpha;
    int pointSkip;
    std::vector<std::string> deviceFilter;
};

struct ScenePoint {
    Vec3 pos;
    uint8_t r, g, b;
    uint8_t irIntensity;
};

class Scene3DRenderer {
public:
    Scene3DRenderer(int outW, int outH, const RenderConfig &cfg)
        : outW_(outW), outH_(outH), cfg_(cfg) {
        frameBuf_.resize(outW * outH * 3, 0);
        zBuf_.resize(outW * outH, 1e10f);
        camera_.fov = cfg.camFov;
        camera_.pos = Vec3(0, cfg.camHeight, -cfg.camOrbitRadius);
    }

    void updateIMUOrientation(const Vec3 &accel, const Vec3 &gyro, float dt) {
        Vec3 gravDir = accel.normalized();
        float tiltX = std::atan2(-gravDir.x, gravDir.z);
        float tiltZ = std::atan2(-gravDir.y, std::sqrt(gravDir.x*gravDir.x + gravDir.z*gravDir.z));
        float gyroMag = gyro.length();
        if(gyroMag > 0.001f) {
            imuOrientation_ = imuOrientation_
                * Mat3x3::rotationY(gyro.y * dt * 0.05f)
                * Mat3x3::rotationX(gyro.x * dt * 0.05f)
                * Mat3x3::rotationZ(gyro.z * dt * 0.05f);
        }
        Mat3x3 gravCorr = Mat3x3::rotationX(tiltX * 0.02f) * Mat3x3::rotationZ(tiltZ * 0.02f);
        imuOrientation_ = gravCorr * imuOrientation_;
    }

    void addDepthPoints(const uint16_t *depthData, int depthW, int depthH, float scale,
        const OBCameraIntrinsic &intrinsic, const OBExtrinsic &extrinsic,
        const uint8_t *colorRGB, int colorW, int colorH, OBFormat /*colorFmt*/,
        const uint8_t *irLeftData, int irLeftW, int irLeftH,
        const uint8_t * /*irRightData*/, int /*irRightW*/, int /*irRightH*/) {

        float depthMinMm = cfg_.depthMinM * 1000.0f;
        float depthMaxMm = cfg_.depthMaxM * 1000.0f;
        int skip = cfg_.pointSkip;
        if(skip < 1) skip = 1;

        float fx = intrinsic.fx;
        float fy = intrinsic.fy;
        float cx = intrinsic.cx;
        float cy = intrinsic.cy;

        float rot[9];
        float trans[3];
        for(int i=0; i<9; i++) rot[i] = extrinsic.rot[i];
        for(int i=0; i<3; i++) trans[i] = extrinsic.trans[i];

        for(int v = 0; v < depthH; v += skip) {
            for(int u = 0; u < depthW; u += skip) {
                uint16_t rawVal = depthData[v * depthW + u];
                if(rawVal == 0) continue;
                float depthMm = rawVal * scale;
                if(depthMm < depthMinMm || depthMm > depthMaxMm) continue;

                float x = (u - cx) * depthMm / fx;
                float y = (v - cy) * depthMm / fy;
                float z = depthMm;

                float wx = rot[0]*x + rot[1]*y + rot[2]*z + trans[0];
                float wy = rot[3]*x + rot[4]*y + rot[5]*z + trans[1];
                float wz = rot[6]*x + rot[7]*y + rot[8]*z + trans[2];

                float wxm = wx / 1000.0f;
                float wym = wy / 1000.0f;
                float wzm = wz / 1000.0f;

                ScenePoint pt;
                pt.pos = Vec3(wxm, wym, wzm);

                if(colorRGB && colorW > 0 && colorH > 0) {
                    float cu = fx * wxm * 1000.0f / wzm + cx;
                    float cv = fy * wym * 1000.0f / wzm + cy;
                    int ci = static_cast<int>(cu);
                    int cj = static_cast<int>(cv);
                    if(ci >= 0 && ci < colorW && cj >= 0 && cj < colorH) {
                        int idx = (cj * colorW + ci) * 3;
                        pt.r = colorRGB[idx + 0];
                        pt.g = colorRGB[idx + 1];
                        pt.b = colorRGB[idx + 2];
                    } else {
                        float norm = (depthMm - depthMinMm) / (depthMaxMm - depthMinMm);
                        norm = std::max(0.0f, std::min(1.0f, norm));
                        uint8_t v8 = static_cast<uint8_t>(norm * 255.0f);
                        jetColormap(v8, pt.r, pt.g, pt.b);
                    }
                } else {
                    float norm = (depthMm - depthMinMm) / (depthMaxMm - depthMinMm);
                    norm = std::max(0.0f, std::min(1.0f, norm));
                    uint8_t v8 = static_cast<uint8_t>(norm * 255.0f);
                    jetColormap(v8, pt.r, pt.g, pt.b);
                }

                pt.irIntensity = 0;
                if(irLeftData && irLeftW > 0 && irLeftH > 0) {
                    int iu = u * irLeftW / depthW;
                    int iv = v * irLeftH / depthH;
                    if(iu >= 0 && iu < irLeftW && iv >= 0 && iv < irLeftH) {
                        pt.irIntensity = irLeftData[iv * irLeftW + iu];
                    }
                }

                points_.push_back(pt);
            }
        }
    }

    void clearPoints() { points_.clear(); }

    uint8_t *renderFrame(float time, bool useIMU, const Mat3x3 &imuOrient) {
        memset(frameBuf_.data(), 0, outW_ * outH_ * 3);
        for(int i = 0; i < outW_ * outH_; i++) zBuf_[i] = 1e10f;

        Mat3x3 viewRot;
        if(cfg_.autoRotateSpeed > 0) {
            float angle = time * cfg_.autoRotateSpeed;
            viewRot = Mat3x3::rotationY(angle);
        }
        if(useIMU) {
            viewRot = imuOrient * viewRot;
        }

        camera_.pos = Vec3(0, cfg_.camHeight, -cfg_.camOrbitRadius);
        camera_.rot = viewRot;

        Vec3 camFwd = camera_.forward();
        Vec3 camUp = camera_.up();
        Vec3 camRight = camera_.right();

        float aspect = static_cast<float>(outW_) / static_cast<float>(outH_);
        float fovRad = camera_.fov * 3.14159265f / 180.0f;
        float halfH = std::tan(fovRad * 0.5f);
        float halfW = halfH * aspect;

        for(const auto &pt : points_) {
            Vec3 rel = pt.pos - camera_.pos;
            float zc = rel.dot(camFwd);
            if(zc < camera_.nearP || zc > camera_.farP) continue;

            float xc = rel.dot(camRight);
            float yc = -rel.dot(camUp);

            float px = xc / (zc * halfW);
            float py = yc / (zc * halfH);

            int screenX = static_cast<int>((px * 0.5f + 0.5f) * outW_);
            int screenY = static_cast<int>((-py * 0.5f + 0.5f) * outH_);

            if(screenX < 0 || screenX >= outW_ || screenY < 0 || screenY >= outH_) continue;

            int pixIdx = screenY * outW_ + screenX;
            if(zc < zBuf_[pixIdx]) {
                zBuf_[pixIdx] = zc;
                uint8_t r = pt.r, g = pt.g, b = pt.b;

                if(pt.irIntensity > 0 && cfg_.depthAlpha > 0) {
                    float ia = cfg_.depthAlpha * (pt.irIntensity / 255.0f);
                    float inv = 1.0f - ia;
                    float irVal = pt.irIntensity * 0.9f;
                    r = static_cast<uint8_t>(inv * r + ia * irVal * 0.2f + 0.5f);
                    g = static_cast<uint8_t>(inv * g + ia * irVal * 0.9f + 0.5f);
                    b = static_cast<uint8_t>(inv * b + ia * irVal * 0.7f + 0.5f);
                }

                float distFade = 1.0f - (zc - camera_.nearP) / (camera_.farP - camera_.nearP);
                distFade = std::max(0.15f, std::min(1.0f, distFade));
                r = static_cast<uint8_t>(r * distFade + 0.5f);
                g = static_cast<uint8_t>(g * distFade + 0.5f);
                b = static_cast<uint8_t>(b * distFade + 0.5f);

                frameBuf_[pixIdx * 3 + 0] = r;
                frameBuf_[pixIdx * 3 + 1] = g;
                frameBuf_[pixIdx * 3 + 2] = b;
            }
        }

        drawGrid(camFwd, camUp, camRight, halfW, halfH);
        drawOverlay(time, useIMU);

        return frameBuf_.data();
    }

    size_t getPointCount() const { return points_.size(); }
    Mat3x3 getIMUOrientation() const { return imuOrientation_; }

private:
    void drawGrid(const Vec3 &camFwd, const Vec3 &camUp, const Vec3 &camRight,
        float halfW, float halfH) {
        int gridLines = 10;
        float gridExtent = 5.0f;
        for(int i = -gridLines; i <= gridLines; i++) {
            float x = i * gridExtent / gridLines;
            drawWorldLine(Vec3(x, 0, -gridExtent), Vec3(x, 0, gridExtent),
                30, 30, 30, camFwd, camUp, camRight, halfW, halfH);
            drawWorldLine(Vec3(-gridExtent, 0, x), Vec3(gridExtent, 0, x),
                30, 30, 30, camFwd, camUp, camRight, halfW, halfH);
        }
    }

    void drawWorldLine(const Vec3 &p0, const Vec3 &p1,
        uint8_t r, uint8_t g, uint8_t b,
        const Vec3 &camFwd, const Vec3 &camUp, const Vec3 &camRight,
        float halfW, float halfH) {
        int steps = 60;
        for(int i = 0; i < steps; i++) {
            float t0 = (float)i / steps;
            Vec3 wp = p0 * (1.0f - t0) + p1 * t0;
            Vec3 rel = wp - camera_.pos;
            float zc = rel.dot(camFwd);
            if(zc < camera_.nearP || zc > camera_.farP) continue;
            float xc = rel.dot(camRight);
            float yc = -rel.dot(camUp);
            float px = xc / (zc * halfW);
            float py = yc / (zc * halfH);
            int sx = static_cast<int>((px * 0.5f + 0.5f) * outW_);
            int sy = static_cast<int>((-py * 0.5f + 0.5f) * outH_);
            if(sx >= 0 && sx < outW_ && sy >= 0 && sy < outH_) {
                int idx = sy * outW_ + sx;
                if(zc < zBuf_[idx]) {
                    frameBuf_[idx*3+0] = r; frameBuf_[idx*3+1] = g; frameBuf_[idx*3+2] = b;
                }
            }
        }
    }

    void drawOverlay(float time, bool useIMU) {
        char buf[128];
        snprintf(buf, sizeof(buf), "3D Render | t=%.1fs | pts=%zu | IMU:%s",
            time, points_.size(), useIMU ? "ON" : "OFF");
        for(int i = 0; buf[i] && i < 60; i++) {
            int x = 10 + i * 8;
            if(x + 7 < outW_) {
                drawChar5x7(x, 10, buf[i], 220, 220, 220);
            }
        }
    }

    void drawChar5x7(int x0, int y0, char c, uint8_t r, uint8_t g, uint8_t b) {
        static const uint8_t font5x7[][5] = {
            {0x00,0x00,0x00,0x00,0x00},
            {0x00,0x00,0x5F,0x00,0x00},
            {0x00,0x03,0x00,0x03,0x00},
            {0x14,0x3E,0x14,0x3E,0x14},
            {0x24,0x2A,0x7F,0x2A,0x12},
            {0x23,0x13,0x08,0x64,0x62},
            {0x36,0x49,0x55,0x22,0x50},
            {0x00,0x05,0x03,0x00,0x00},
            {0x00,0x1C,0x22,0x41,0x00},
            {0x00,0x41,0x22,0x1C,0x00},
            {0x08,0x2A,0x1C,0x2A,0x08},
            {0x08,0x08,0x3E,0x08,0x08},
            {0x00,0x50,0x30,0x00,0x00},
            {0x08,0x08,0x08,0x08,0x08},
            {0x00,0x60,0x60,0x00,0x00},
            {0x20,0x10,0x08,0x04,0x02},
            {0x3E,0x51,0x49,0x45,0x3E},
            {0x00,0x42,0x7F,0x40,0x00},
            {0x42,0x61,0x51,0x49,0x46},
            {0x21,0x41,0x45,0x4B,0x31},
            {0x18,0x14,0x12,0x7F,0x10},
            {0x27,0x45,0x45,0x45,0x39},
            {0x3C,0x4A,0x49,0x49,0x30},
            {0x01,0x71,0x09,0x05,0x03},
            {0x36,0x49,0x49,0x49,0x36},
            {0x06,0x49,0x49,0x29,0x1E},
            {0x00,0x36,0x36,0x00,0x00},
            {0x00,0x56,0x36,0x00,0x00},
            {0x00,0x08,0x14,0x22,0x41},
            {0x14,0x14,0x14,0x14,0x14},
            {0x41,0x22,0x14,0x08,0x00},
            {0x02,0x01,0x51,0x09,0x06},
            {0x32,0x49,0x79,0x41,0x3E},
            {0x7E,0x09,0x09,0x09,0x7E},
            {0x7F,0x49,0x49,0x49,0x36},
            {0x3E,0x41,0x41,0x41,0x22},
            {0x7F,0x41,0x41,0x22,0x1C},
            {0x7F,0x49,0x49,0x49,0x41},
            {0x7F,0x09,0x09,0x01,0x01},
            {0x3E,0x41,0x41,0x49,0x3A},
            {0x7F,0x08,0x08,0x08,0x7F},
            {0x00,0x41,0x7F,0x41,0x00},
            {0x20,0x40,0x41,0x3F,0x01},
            {0x7F,0x08,0x14,0x22,0x41},
            {0x7F,0x40,0x40,0x40,0x40},
            {0x7F,0x02,0x04,0x02,0x7F},
            {0x7F,0x04,0x08,0x10,0x7F},
            {0x3E,0x41,0x41,0x41,0x3E},
            {0x7F,0x09,0x09,0x09,0x06},
            {0x3E,0x41,0x51,0x21,0x5E},
            {0x7F,0x09,0x09,0x19,0x66},
            {0x26,0x49,0x49,0x49,0x32},
            {0x01,0x01,0x7F,0x01,0x01},
            {0x3F,0x40,0x40,0x40,0x3F},
            {0x1F,0x20,0x40,0x20,0x1F},
            {0x7F,0x20,0x10,0x20,0x7F},
            {0x63,0x14,0x08,0x14,0x63},
            {0x07,0x08,0x70,0x08,0x07},
            {0x61,0x51,0x49,0x45,0x43},
        };
        int idx = c - ' ';
        if(idx < 0 || idx >= (int)(sizeof(font5x7)/sizeof(font5x7[0]))) return;
        const uint8_t *glyph = font5x7[idx];
        for(int col = 0; col < 5; col++) {
            uint8_t colBits = glyph[col];
            for(int row = 0; row < 7; row++) {
                if(colBits & (1 << row)) {
                    int px = x0 + col;
                    int py = y0 + row;
                    if(px >= 0 && px < outW_ && py >= 0 && py < outH_) {
                        int i = (py * outW_ + px) * 3;
                        frameBuf_[i+0] = r; frameBuf_[i+1] = g; frameBuf_[i+2] = b;
                    }
                }
            }
        }
    }

    int outW_, outH_;
    RenderConfig cfg_;
    std::vector<uint8_t> frameBuf_;
    std::vector<float> zBuf_;
    Camera3D camera_;
    std::vector<ScenePoint> points_;
    Mat3x3 imuOrientation_;
};

static RenderConfig parseArgs(int argc, char **argv) {
    RenderConfig cfg;
    cfg.outW = 1280; cfg.outH = 720; cfg.outFps = 30;
    cfg.autoRotateSpeed = 0.3f;
    cfg.camOrbitRadius = 2.5f;
    cfg.camHeight = 0.5f;
    cfg.camFov = 70.0f;
    cfg.depthMinM = 0.1f; cfg.depthMaxM = 8.0f;
    cfg.depthAlpha = 0.6f;
    cfg.pointSkip = 2;
    for(int i=1; i<argc; i++) {
        std::string arg = argv[i];
        if(arg == "--out-w" && i+1<argc) cfg.outW = std::stoi(argv[++i]);
        else if(arg == "--out-h" && i+1<argc) cfg.outH = std::stoi(argv[++i]);
        else if(arg == "--out-fps" && i+1<argc) cfg.outFps = std::stoi(argv[++i]);
        else if(arg == "--rotate-speed" && i+1<argc) cfg.autoRotateSpeed = std::stof(argv[++i]);
        else if(arg == "--orbit-radius" && i+1<argc) cfg.camOrbitRadius = std::stof(argv[++i]);
        else if(arg == "--cam-height" && i+1<argc) cfg.camHeight = std::stof(argv[++i]);
        else if(arg == "--depth-min" && i+1<argc) cfg.depthMinM = std::stof(argv[++i]);
        else if(arg == "--depth-max" && i+1<argc) cfg.depthMaxM = std::stof(argv[++i]);
        else if(arg == "--depth-alpha" && i+1<argc) cfg.depthAlpha = std::stof(argv[++i]);
        else if(arg == "--point-skip" && i+1<argc) cfg.pointSkip = std::stoi(argv[++i]);
        else if(arg == "--no-rotate") cfg.autoRotateSpeed = 0.0f;
        else if(arg == "--help") {
            std::cout << "Usage: nio_3d_render [device_filter...] [options]\n"
                << "  --out-w W          Output width (default: 1280)\n"
                << "  --out-h H          Output height (default: 720)\n"
                << "  --out-fps FPS      Output FPS (default: 30)\n"
                << "  --rotate-speed SPD Auto-rotate speed rad/s (default: 0.3)\n"
                << "  --no-rotate        Disable auto-rotation\n"
                << "  --orbit-radius R   Camera orbit radius m (default: 2.5)\n"
                << "  --cam-height H     Camera height m (default: 0.5)\n"
                << "  --depth-min M      Min depth m (default: 0.1)\n"
                << "  --depth-max M      Max depth m (default: 8.0)\n"
                << "  --depth-alpha A    Depth color alpha 0-1 (default: 0.6)\n"
                << "  --point-skip N     Skip every N-th point (default: 2)\n";
            exit(0);
        }
        else if(arg.substr(0,2) != "--") cfg.deviceFilter.push_back(arg);
    }
    return cfg;
}

int main(int argc, char **argv) try {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    RenderConfig cfg = parseArgs(argc, argv);

    NIO_LOG_INIT("nio_3d_render", "3d_render_output");
    NIO_LOG_SET_LEVEL(nio::LogLevel::TRACE);
    NIO_LOG_INFO_S("Process started, outW=" << cfg.outW << " outH=" << cfg.outH << " outFps=" << cfg.outFps
        << " autoRotate=" << cfg.autoRotateSpeed << " orbitRadius=" << cfg.camOrbitRadius
        << " depthMin=" << cfg.depthMinM << " depthMax=" << cfg.depthMaxM
        << " depthAlpha=" << cfg.depthAlpha << " pointSkip=" << cfg.pointSkip);

    ob::Context context;
    auto deviceList = context.queryDeviceList();
    if(deviceList->getCount() < 1) {
        std::cerr << "No Orbbec device found!" << std::endl;
        NIO_LOG_FATAL("No Orbbec device found!");
        return -1;
    }

    std::string sessionTimestamp = getTimestampMs();
    std::string outputRootDir = "3d_render_output/" + sessionTimestamp;
    mkdirp(outputRootDir);
    NIO_LOG_INFO_S("Session timestamp=" << sessionTimestamp << " outputDir=" << outputRootDir);

    std::vector<std::shared_ptr<DeviceRender>> devices;

    for(uint32_t i = 0; i < deviceList->getCount(); i++) {
        auto device = deviceList->getDevice(i);
        auto devInfo = device->getDeviceInfo();
        std::string name = devInfo->getName();

        if(!deviceMatches(name, cfg.deviceFilter)) {
            std::cout << "Skipping device: " << name << std::endl;
            NIO_LOG_DEBUG_S("Skipping device: " << name);
            continue;
        }

        std::cout << "Found device: " << name
            << " (SN: " << devInfo->getSerialNumber()
            << ", PID: 0x" << std::hex << std::setw(4) << std::setfill('0')
            << devInfo->getPid() << std::dec << ")" << std::endl;
        NIO_LOG_INFO_S("Found device: " << name << " SN=" << devInfo->getSerialNumber()
            << " PID=0x" << std::hex << devInfo->getPid() << std::dec);

        auto safeName = name;
        std::replace(safeName.begin(), safeName.end(), ' ', '_');

        auto dr = std::make_shared<DeviceRender>();
        dr->deviceName = safeName;
        dr->depthScale = 0.001f;
        dr->hasIMU = false;
        dr->hasIntrinsics = false;
        dr->renderFrameCount = std::make_shared<std::atomic<uint64_t>>(0);
        dr->imuState = std::make_shared<IMUState>();

        try { device->timerSyncWithHost(); }
        catch(ob::Error &e) { std::cerr << " Timer sync warning: " << e.what() << std::endl;
            NIO_LOG_WARN_S("Timer sync failed for " << safeName << ": " << e.what()); }
        if(device->isGlobalTimestampSupported()) {
            try { device->enableGlobalTimestamp(true); } catch(...) {}
        }

        dr->videoPipeline = std::make_shared<ob::Pipeline>(device);
        auto config = std::make_shared<ob::Config>();
        config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

        auto sensorList = device->getSensorList();
        bool hasColor = false, hasDepth = false;
        bool hasIRLeft = false, hasIRRight = false;
        bool hasAccel = false, hasGyro = false;
        OBFormat colorFormat = OB_FORMAT_UNKNOWN, depthFormat = OB_FORMAT_UNKNOWN;
        OBFormat irLeftFormat = OB_FORMAT_UNKNOWN, irRightFormat = OB_FORMAT_UNKNOWN;
        int colorW=0, colorH=0, colorFps=30;
        int depthW=0, depthH=0, depthFps=30;
        int irLW=0, irLH=0, irLFps=30;
        int irRW=0, irRH=0, irRFps=30;
        std::shared_ptr<ob::VideoStreamProfile> colorProfile, depthProfile, irLeftProfile, irRightProfile;

        for(uint32_t s = 0; s < sensorList->getCount(); s++) {
            auto sensorType = sensorList->getSensorType(s);
            auto sensor = sensorList->getSensor(s);
            auto profileList = sensor->getStreamProfileList();

            switch(sensorType) {
            case OB_SENSOR_COLOR:
                hasColor = true;
                colorProfile = selectBestProfile(profileList, OB_FORMAT_MJPG);
                if(colorProfile) {
                    colorFormat = colorProfile->getFormat();
                    if(colorFormat == OB_FORMAT_UNKNOWN) {
                        for(uint32_t k=0; k<profileList->getCount(); k++) {
                            try { auto p = profileList->getProfile(k)->as<ob::VideoStreamProfile>();
                                if(p && p->getFormat() != OB_FORMAT_UNKNOWN) { colorProfile=p; colorFormat=p->getFormat(); break; }
                            } catch(...) {}
                        }
                    }
                    if(colorFormat != OB_FORMAT_UNKNOWN) {
                        config->enableStream(colorProfile);
                        colorW = colorProfile->getWidth(); colorH = colorProfile->getHeight(); colorFps = colorProfile->getFps();
                    } else { hasColor = false; }
                } else { hasColor = false; }
                if(hasColor) std::cout << "  Color: " << colorW << "x" << colorH << "@" << colorFps << " fmt=" << colorFormat << std::endl;
                if(hasColor) NIO_LOG_INFO_S("Color stream: " << colorW << "x" << colorH << "@" << colorFps << " fmt=" << colorFormat);
                break;
            case OB_SENSOR_DEPTH:
                hasDepth = true;
                depthProfile = selectBestProfile(profileList, OB_FORMAT_Y16);
                if(depthProfile) {
                    depthFormat = depthProfile->getFormat();
                    if(depthFormat == OB_FORMAT_UNKNOWN) {
                        for(uint32_t k=0; k<profileList->getCount(); k++) {
                            try { auto p = profileList->getProfile(k)->as<ob::VideoStreamProfile>();
                                if(p && p->getFormat() != OB_FORMAT_UNKNOWN) { depthProfile=p; depthFormat=p->getFormat(); break; }
                            } catch(...) {}
                        }
                    }
                    if(depthFormat != OB_FORMAT_UNKNOWN) {
                        config->enableStream(depthProfile);
                        depthW = depthProfile->getWidth(); depthH = depthProfile->getHeight(); depthFps = depthProfile->getFps();
                    } else { hasDepth = false; }
                } else { hasDepth = false; }
                if(hasDepth) std::cout << "  Depth: " << depthW << "x" << depthH << "@" << depthFps << " fmt=" << depthFormat << std::endl;
                if(hasDepth) NIO_LOG_INFO_S("Depth stream: " << depthW << "x" << depthH << "@" << depthFps << " fmt=" << depthFormat
                    << " scale=" << dr->depthScale);
                try {
                    int32_t pl = device->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
                    switch(pl) { case 0: dr->depthScale=0.001f; break; case 1: dr->depthScale=0.0005f; break;
                        case 2: dr->depthScale=0.00025f; break; case 3: dr->depthScale=0.0001f; break; default: dr->depthScale=0.001f; break; }
                    std::cout << "  Depth scale: " << dr->depthScale << std::endl;
                } catch(...) { dr->depthScale = 0.001f; }
                break;
            case OB_SENSOR_IR_LEFT:
                hasIRLeft = true;
                irLeftProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if(irLeftProfile) {
                    irLeftFormat = irLeftProfile->getFormat();
                    if(irLeftFormat == OB_FORMAT_UNKNOWN) irLeftFormat = OB_FORMAT_Y8;
                    config->enableStream(irLeftProfile);
                    irLW = irLeftProfile->getWidth(); irLH = irLeftProfile->getHeight(); irLFps = irLeftProfile->getFps();
                } else { hasIRLeft = false; }
                if(hasIRLeft) std::cout << "  IR Left: " << irLW << "x" << irLH << "@" << irLFps << " fmt=" << irLeftFormat << std::endl;
                if(hasIRLeft) NIO_LOG_INFO_S("IR Left stream: " << irLW << "x" << irLH << "@" << irLFps << " fmt=" << irLeftFormat);
                break;
            case OB_SENSOR_IR_RIGHT:
                hasIRRight = true;
                irRightProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if(irRightProfile) {
                    irRightFormat = irRightProfile->getFormat();
                    if(irRightFormat == OB_FORMAT_UNKNOWN) irRightFormat = OB_FORMAT_Y8;
                    config->enableStream(irRightProfile);
                    irRW = irRightProfile->getWidth(); irRH = irRightProfile->getHeight(); irRFps = irRightProfile->getFps();
                } else { hasIRRight = false; }
                if(hasIRRight) std::cout << "  IR Right: " << irRW << "x" << irRH << "@" << irRFps << " fmt=" << irRightFormat << std::endl;
                if(hasIRRight) NIO_LOG_INFO_S("IR Right stream: " << irRW << "x" << irRH << "@" << irRFps << " fmt=" << irRightFormat);
                break;
            case OB_SENSOR_ACCEL: hasAccel = true; break;
            case OB_SENSOR_GYRO: hasGyro = true; break;
            default: break;
            }
        }

        auto pid = devInfo->getPid(); auto vid = devInfo->getVid();
        if(ob_smpl::isGemini305gDevice(vid, pid, devInfo->getConnectionType())) {
            config->disableStream(OB_SENSOR_IR_LEFT);
            hasIRLeft = false;
            std::cout << "  Gemini 305g: disabled IR_LEFT" << std::endl;
            NIO_LOG_INFO("Gemini 305g detected, disabled IR_LEFT stream");
        }

        if(!hasDepth) {
            std::cerr << "  " << safeName << " has no depth sensor, skipping" << std::endl;
            NIO_LOG_WARN_S(safeName << " has no depth sensor, skipping 3D render");
            continue;
        }

        dr->colorFormat = colorFormat;
        dr->colorW = colorW; dr->colorH = colorH;
        dr->depthW = depthW; dr->depthH = depthH;

        dr->mjpgRes = std::make_shared<MjpgDecoderRes>();
        dr->mjpgRes->init(colorW, colorH, colorFormat);

        if(hasColor && hasDepth) {
            dr->alignFilter = std::make_shared<ob::Align>(OB_STREAM_COLOR);
        }

        try {
            OBCalibrationParam calibParam = {};
            bool gotCalib = false;
            try {
                calibParam = dr->videoPipeline->getCalibrationParam(config);
                gotCalib = true;
            } catch(...) {}

            if(gotCalib) {
                dr->depthIntrinsic = calibParam.intrinsics[OB_SENSOR_DEPTH];
                dr->depthDistortion = calibParam.distortion[OB_SENSOR_DEPTH];
                dr->depthToColorExtrinsic = calibParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR];
            } else {
                auto camParam = dr->videoPipeline->getCameraParam();
                dr->depthIntrinsic = camParam.depthIntrinsic;
                dr->depthDistortion = camParam.depthDistortion;
                dr->depthToColorExtrinsic = camParam.transform;
            }
            if(dr->depthIntrinsic.fx < 1.0f || dr->depthIntrinsic.fy < 1.0f) {
                std::cout << "  WARNING: Zero/invalid intrinsics from SDK, using computed defaults" << std::endl;
                NIO_LOG_WARN_S("Zero/invalid intrinsics from SDK for " << safeName << ", using computed defaults: fx=" << dr->depthIntrinsic.fx);
                dr->depthIntrinsic.fx = depthW * 0.5f / std::tan(70.0f * 3.14159265f / 360.0f);
                dr->depthIntrinsic.fy = dr->depthIntrinsic.fx;
                dr->depthIntrinsic.cx = depthW * 0.5f;
                dr->depthIntrinsic.cy = depthH * 0.5f;
                memset(dr->depthToColorExtrinsic.rot, 0, sizeof(dr->depthToColorExtrinsic.rot));
                dr->depthToColorExtrinsic.rot[0] = 1; dr->depthToColorExtrinsic.rot[4] = 1; dr->depthToColorExtrinsic.rot[8] = 1;
                memset(dr->depthToColorExtrinsic.trans, 0, sizeof(dr->depthToColorExtrinsic.trans));
            }
            dr->hasIntrinsics = true;
            std::cout << "  Intrinsics: depth fx=" << dr->depthIntrinsic.fx
                << " fy=" << dr->depthIntrinsic.fy
                << " cx=" << dr->depthIntrinsic.cx
                << " cy=" << dr->depthIntrinsic.cy << std::endl;
            NIO_LOG_INFO_S("Depth intrinsics: fx=" << dr->depthIntrinsic.fx
                << " fy=" << dr->depthIntrinsic.fy
                << " cx=" << dr->depthIntrinsic.cx
                << " cy=" << dr->depthIntrinsic.cy);
        } catch(...) {
            std::cout << "  WARNING: Could not get camera intrinsics, using defaults" << std::endl;
            NIO_LOG_WARN_S("Could not get camera intrinsics for " << safeName << ", using defaults");
            dr->depthIntrinsic.fx = depthW * 0.5f / std::tan(70.0f * 3.14159265f / 360.0f);
            dr->depthIntrinsic.fy = dr->depthIntrinsic.fx;
            dr->depthIntrinsic.cx = depthW * 0.5f;
            dr->depthIntrinsic.cy = depthH * 0.5f;
            memset(dr->depthToColorExtrinsic.rot, 0, sizeof(dr->depthToColorExtrinsic.rot));
            dr->depthToColorExtrinsic.rot[0] = 1; dr->depthToColorExtrinsic.rot[4] = 1; dr->depthToColorExtrinsic.rot[8] = 1;
            memset(dr->depthToColorExtrinsic.trans, 0, sizeof(dr->depthToColorExtrinsic.trans));
            dr->hasIntrinsics = true;
        }

        try { dr->videoPipeline->enableFrameSync(); } catch(...) {}

        auto drCapture = dr;
        try {
            dr->videoPipeline->start(config,
                [drCapture, hasColor, hasDepth, hasIRLeft, hasIRRight]
                (std::shared_ptr<ob::FrameSet> frameSet) {
                if(!frameSet) return;

                auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
                if(!depthFrame) return;

                auto alignedFS = frameSet;
                if(drCapture->alignFilter && hasColor) {
                    auto aligned = drCapture->alignFilter->process(frameSet);
                    if(aligned) {
                        auto fs = std::dynamic_pointer_cast<ob::FrameSet>(aligned);
                        if(fs) alignedFS = fs;
                    }
                }

                uint16_t *depthPtr = reinterpret_cast<uint16_t *>(depthFrame->getData());
                int dW = drCapture->depthW;
                int dH = drCapture->depthH;

                float scale = drCapture->depthScale;
                try { auto df = depthFrame->as<ob::DepthFrame>(); if(df) scale = df->getValueScale(); } catch(...) {}

                std::vector<uint8_t> colorRGB;
                int cW = 0, cH = 0;
                OBFormat cFmt = drCapture->colorFormat;
                if(hasColor) {
                    auto colorFrame = alignedFS->getFrame(OB_FRAME_COLOR);
                    if(colorFrame) {
                        cW = drCapture->colorW; cH = drCapture->colorH;
                        colorRGB.resize(cW * cH * 3, 0);
                        decodeColorToRGB(colorFrame->getData(), colorFrame->getDataSize(), cFmt,
                            cW, cH, colorRGB.data(), drCapture->mjpgRes);
                    }
                }

                const uint8_t *irLeftData = nullptr;
                int irLW = 0, irLH = 0;
                if(hasIRLeft) {
                    auto irLFrame = frameSet->getFrame(OB_FRAME_IR_LEFT);
                    if(irLFrame) {
                        irLeftData = irLFrame->getData();
                        try { auto vf = irLFrame->as<ob::VideoFrame>(); irLW = vf->getWidth(); irLH = vf->getHeight(); } catch(...) {}
                    }
                }

                const uint8_t *irRightData = nullptr;
                int irRW = 0, irRH = 0;
                if(hasIRRight) {
                    auto irRFrame = frameSet->getFrame(OB_FRAME_IR_RIGHT);
                    if(irRFrame) {
                        irRightData = irRFrame->getData();
                        try { auto vf = irRFrame->as<ob::VideoFrame>(); irRW = vf->getWidth(); irRH = vf->getHeight(); } catch(...) {}
                    }
                }

                drCapture->renderFrameCount->fetch_add(1);
                drCapture->latestDataMtx.lock();
                drCapture->latestDepth.assign(depthPtr, depthPtr + dW * dH);
                drCapture->latestDepthW = dW;
                drCapture->latestDepthH = dH;
                drCapture->latestScale = scale;
                drCapture->latestColorRGB = std::move(colorRGB);
                drCapture->latestColorW = cW;
                drCapture->latestColorH = cH;
                if(irLeftData && irLW > 0 && irLH > 0)
                    drCapture->latestIRLeftData.assign(irLeftData, irLeftData + (irLW * irLH));
                else
                    drCapture->latestIRLeftData.clear();
                drCapture->latestIRLeftW = irLW;
                drCapture->latestIRLeftH = irLH;
                if(irRightData && irRW > 0 && irRH > 0)
                    drCapture->latestIRRightData.assign(irRightData, irRightData + (irRW * irRH));
                else
                    drCapture->latestIRRightData.clear();
                drCapture->latestIRRightW = irRW;
                drCapture->latestIRRightH = irRH;
                drCapture->latestDataReady = true;
                drCapture->latestDataMtx.unlock();
                drCapture->latestDataCV.notify_one();
            });
        } catch(ob::Error &e) {
            std::cerr << "  Pipeline start failed for " << safeName << ": " << e.what() << std::endl;
            NIO_LOG_ERROR_S("Pipeline start failed for " << safeName << ": " << e.what());
            dr->videoPipeline.reset();
            continue;
        }

        dr->hasIMU = (hasAccel && hasGyro);
        if(dr->hasIMU) {
            auto imuDev = dr->videoPipeline->getDevice();
            dr->imuPipeline = std::make_shared<ob::Pipeline>(imuDev);
            auto imuConfig = std::make_shared<ob::Config>();
            imuConfig->enableAccelStream();
            imuConfig->enableGyroStream();
            imuConfig->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
            try {
                dr->imuPipeline->start(imuConfig, [dr](std::shared_ptr<ob::FrameSet> frameSet) {
                    if(!frameSet) return;
                    auto accelFrame = frameSet->getFrame(OB_FRAME_ACCEL);
                    auto gyroFrame = frameSet->getFrame(OB_FRAME_GYRO);
                    Vec3 a, g;
                    if(accelFrame) {
                        try { auto af = accelFrame->as<ob::AccelFrame>(); if(af) { auto v = af->getValue(); a = Vec3(v.x, v.y, v.z); } } catch(...) {}
                    }
                    if(gyroFrame) {
                        try { auto gf = gyroFrame->as<ob::GyroFrame>(); if(gf) { auto v = gf->getValue(); g = Vec3(v.x, v.y, v.z); } } catch(...) {}
                    }
                    dr->imuState->update(a, g);
                });
            } catch(ob::Error &e) {
                std::cerr << "  IMU pipeline start failed: " << e.what() << std::endl;
                NIO_LOG_WARN_S("IMU pipeline start failed for " << safeName << ": " << e.what());
                dr->hasIMU = false;
            }
        }

        devices.push_back(dr);
    }

    if(devices.empty()) {
        std::cerr << "No suitable devices found!" << std::endl;
        NIO_LOG_FATAL("No suitable devices found for 3D render!");
        return -1;
    }

    std::cout << "\n=== 3D Render recording started ===" << std::endl;
    std::cout << "Output: " << outputRootDir << "/" << std::endl;
    std::cout << "Rendering " << devices.size() << " device(s) to "
        << cfg.outW << "x" << cfg.outH << "@" << cfg.outFps << "fps" << std::endl;
    std::cout << "Depth range: " << cfg.depthMinM << "m - " << cfg.depthMaxM << "m" << std::endl;
    std::cout << "Auto-rotate: " << (cfg.autoRotateSpeed > 0 ? "ON" : "OFF")
        << " (speed=" << cfg.autoRotateSpeed << " rad/s)" << std::endl;
    std::cout << "Press Ctrl+C or 'q' to stop.\n" << std::endl;
    NIO_LOG_INFO_S("=== 3D Render recording started === devices=" << devices.size()
        << " outW=" << cfg.outW << " outH=" << cfg.outH << "@" << cfg.outFps << "fps"
        << " depthRange=" << cfg.depthMinM << "m-" << cfg.depthMaxM << "m"
        << " autoRotate=" << cfg.autoRotateSpeed);
    NIO_LOG_INFO_S("Log file: " << NIO_LOG_PATH());

    Scene3DRenderer renderer(cfg.outW, cfg.outH, cfg);

    std::string startTs = getTimestampMs();
    std::string fusedPath = outputRootDir + "/3d_render_fused_" + startTs + ".h264";
    auto fusedFile = std::make_shared<std::ofstream>(fusedPath, std::ios::binary);

    H264Encoder encoder;
    if(!encoder.initRGB(cfg.outW, cfg.outH, cfg.outFps, 6000000)) {
        std::cerr << "Failed to init H264 encoder!" << std::endl;
        NIO_LOG_FATAL_S("Failed to init H264 encoder " << cfg.outW << "x" << cfg.outH << "@" << cfg.outFps);
        return -1;
    }

    std::mutex encodeMtx;
    auto startTime = std::chrono::steady_clock::now();
    auto lastReportTime = ob_smpl::getNowTimesMs();
    uint64_t encodedFrames = 0;

    while(g_running) {
        auto key = ob_smpl::waitForKeyPressed(33);
        if(key == ESC_KEY || key == 'q' || key == 'Q') { g_running = false; break; }

        bool anyData = false;
        renderer.clearPoints();

        Mat3x3 imuOrient;
        bool useIMU = false;
        for(auto &dr : devices) {
            if(dr->hasIMU) {
                Vec3 a, g;
                if(dr->imuState->get(a, g)) {
                    renderer.updateIMUOrientation(a, g, 0.033f);
                    imuOrient = renderer.getIMUOrientation();
                    useIMU = true;
                }
            }
        }

        for(auto &dr : devices) {
            std::lock_guard<std::mutex> lock(dr->latestDataMtx);
            if(dr->latestDataReady && !dr->latestDepth.empty()) {
                anyData = true;

                renderer.addDepthPoints(
                    dr->latestDepth.data(), dr->latestDepthW, dr->latestDepthH,
                    dr->latestScale,
                    dr->depthIntrinsic, dr->depthToColorExtrinsic,
                    dr->latestColorRGB.empty() ? nullptr : dr->latestColorRGB.data(),
                    dr->latestColorW, dr->latestColorH, dr->colorFormat,
                    dr->latestIRLeftData.empty() ? nullptr : dr->latestIRLeftData.data(),
                    dr->latestIRLeftW, dr->latestIRLeftH,
                    dr->latestIRRightData.empty() ? nullptr : dr->latestIRRightData.data(),
                    dr->latestIRRightW, dr->latestIRRightH
                );
            }
        }

        if(!anyData) continue;

        auto elapsed = std::chrono::steady_clock::now() - startTime;
        float timeSec = std::chrono::duration<float>(elapsed).count();

        uint8_t *frameRGB = renderer.renderFrame(timeSec, useIMU, imuOrient);

        encoder.encodeRGB(frameRGB, *fusedFile, encodeMtx, getTimestampMsInt());
        encodedFrames++;

        auto currentTime = ob_smpl::getNowTimesMs();
        if(currentTime >= lastReportTime + 2000) {
            lastReportTime = currentTime;
            size_t totalPts = renderer.getPointCount();
            float avgFps = encodedFrames / timeSec;
            std::cout << "[3D Render] FPS: " << std::fixed << std::setprecision(1) << avgFps
                << " | Points: " << totalPts
                << " | Encoded: " << encodedFrames << std::endl;
            NIO_LOG_TRACE_S("[3D Render] FPS=" << std::fixed << std::setprecision(1) << avgFps
                << " points=" << totalPts << " encoded=" << encodedFrames);
        }
    }

    std::cout << "\n=== Stopping 3D render ===" << std::endl;
    NIO_LOG_INFO("=== Stopping 3D render ===");
    encoder.close();
    fusedFile->close();

    for(auto &dr : devices) {
        if(dr->videoPipeline) dr->videoPipeline->stop();
        if(dr->hasIMU && dr->imuPipeline) dr->imuPipeline->stop();
        std::cout << "Stopped: " << dr->deviceName << std::endl;
        NIO_LOG_INFO_S("Stopped device: " << dr->deviceName);
    }

    std::cout << "Total encoded frames: " << encodedFrames << std::endl;
    std::cout << "Output: " << fusedPath << std::endl;
    NIO_LOG_INFO_S("Total encoded frames: " << encodedFrames << " output: " << fusedPath);
    NIO_LOG_SHUTDOWN();
    return 0;
}
catch(ob::Error &e) {
    std::cerr << "OB Error: " << e.getFunction() << "\n  " << e.what()
        << "\n  status: " << e.getStatus() << std::endl;
    NIO_LOG_FATAL_S("OB Error: " << e.getFunction() << " " << e.what() << " status=" << e.getStatus());
    NIO_LOG_SHUTDOWN();
    return -1;
}
catch(std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    NIO_LOG_FATAL_S("Exception: " << e.what());
    NIO_LOG_SHUTDOWN();
    return -1;
}
