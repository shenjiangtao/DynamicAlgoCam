// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// stereo_uvc.cpp — Generic UVC stereo camera implementation.

#include "stereo_adapter.hpp"

#include "dynalgo_log.hpp"
#include "dynalgo_common.hpp"
#include "utils.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <memory>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <mutex>
#include <condition_variable>

namespace dynalgo {

class UvcStereoCamera : public IStereoCamera {
public:
    UvcStereoCamera() = default;
    ~UvcStereoCamera() override { close(); }

    bool open(const StereoConfig& cfg) override {
        if (isOpen_) {
            DYNALGO_LOG_WARN_S("UvcStereoCamera: already open");
            return true;
        }

        cfg_ = cfg;

        // Open left and right cameras
        // For generic UVC stereo, we assume two separate video devices
        // or a single device with multiple streams
        leftFd_ = openDevice((cfg.devicePath + "_left").c_str());
        rightFd_ = openDevice((cfg.devicePath + "_right").c_str());

        if (leftFd_ < 0 || rightFd_ < 0) {
            DYNALGO_LOG_ERROR_S("UvcStereoCamera: failed to open devices");
            close();
            return false;
        }

        // Configure format
        if (!configureFormat(leftFd_, cfg.width, cfg.height, cfg.fps) ||
            !configureFormat(rightFd_, cfg.width, cfg.height, cfg.fps)) {
            close();
            return false;
        }

        // Start streaming
        if (!startCapture(leftFd_) || !startCapture(rightFd_)) {
            close();
            return false;
        }

        // Initialize camera info
        info_.serialNumber = cfg.devicePath;
        info_.modelName = "Generic UVC Stereo";
        info_.leftIntrinsic = DynalgoIntrinsic{}; // TODO: load from calibration file
        info_.rightIntrinsic = DynalgoIntrinsic{};
        info_.leftToRight = DynalgoExtrinsic{};
        info_.baselineMeters = 0.12f;

        isOpen_ = true;
        DYNALGO_LOG_INFO_S("UvcStereoCamera opened: " << cfg.devicePath
                           << " " << cfg.width << "x" << cfg.height << "@" << cfg.fps);
        return true;
    }

    bool close() override {
        stopStreaming();
        if (leftFd_ >= 0) {
            stopCapture(leftFd_);
            ::close(leftFd_);
            leftFd_ = -1;
        }
        if (rightFd_ >= 0) {
            stopCapture(rightFd_);
            ::close(rightFd_);
            rightFd_ = -1;
        }
        isOpen_ = false;
        return true;
    }

    bool isOpen() const override { return isOpen_; }

    const StereoCameraInfo& getInfo() const override { return info_; }

    bool grab(StereoFrameSet& outFrame, int timeoutMs) override {
        if (!isOpen_) return false;

        // Grab from both cameras (simplified - real impl needs sync)
        DynalgoFrame leftRaw, rightRaw;
        if (!grabFrame(leftFd_, leftRaw, timeoutMs) ||
            !grabFrame(rightFd_, rightRaw, timeoutMs)) {
            return false;
        }

        // Rectify (placeholder - real impl uses calibration maps)
        outFrame.leftRect = leftRaw;
        outFrame.rightRect = rightRaw;
        outFrame.leftRaw = std::move(leftRaw);
        outFrame.rightRaw = std::move(rightRaw);
        outFrame.timestampUs = dynalgo::getTimestampMsInt() * 1000;
        outFrame.frameId = frameId_++;
        outFrame.leftIntrinsic = info_.leftIntrinsic;
        outFrame.rightIntrinsic = info_.rightIntrinsic;
        outFrame.leftToRight = info_.leftToRight;

        // Compute depth if enabled (placeholder)
        if (cfg_.computeDepth) {
            computeDepth(outFrame);
        }

        return true;
    }

    bool startStreaming(std::function<void(const StereoFrameSet&)> callback) override {
        if (streaming_) return true;
        callback_ = std::move(callback);
        streaming_ = true;
        streamThread_ = std::thread(&UvcStereoCamera::streamLoop, this);
        return true;
    }

    bool stopStreaming() override {
        if (!streaming_) return true;
        streaming_ = false;
        if (streamThread_.joinable()) streamThread_.join();
        callback_ = nullptr;
        return true;
    }

    bool triggerCapture() override {
        // Hardware trigger not supported on generic UVC
        return false;
    }

    bool setExposure(float exposureMs) override {
        return setControl(leftFd_, V4L2_CID_EXPOSURE_ABSOLUTE, static_cast<int>(exposureMs * 1000)) &&
               setControl(rightFd_, V4L2_CID_EXPOSURE_ABSOLUTE, static_cast<int>(exposureMs * 1000));
    }

    bool setGain(float gain) override {
        return setControl(leftFd_, V4L2_CID_GAIN, static_cast<int>(gain * 100)) &&
               setControl(rightFd_, V4L2_CID_GAIN, static_cast<int>(gain * 100));
    }

    float getExposure() const override {
        int val = 0;
        getControl(leftFd_, V4L2_CID_EXPOSURE_ABSOLUTE, val);
        return val / 1000.0f;
    }

    float getGain() const override {
        int val = 0;
        getControl(leftFd_, V4L2_CID_GAIN, val);
        return val / 100.0f;
    }

    bool getRectificationMaps(
        std::vector<float>& leftMapX, std::vector<float>& leftMapY,
        std::vector<float>& rightMapX, std::vector<float>& rightMapY) const override {
        // Placeholder - would load from calibration file
        return false;
    }

private:
    int openDevice(const char* path) {
        int fd = ::open(path, O_RDWR | O_NONBLOCK);
        if (fd < 0) {
            DYNALGO_LOG_ERROR_S("Failed to open " << path << ": " << strerror(errno));
        }
        return fd;
    }

    bool configureFormat(int fd, int width, int height, int fps) {
        struct v4l2_format fmt = {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24; // or YUYV
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) return false;

        struct v4l2_streamparm parm = {};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator = fps;
        ioctl(fd, VIDIOC_S_PARM, &parm);

        return true;
    }

    bool startCapture(int fd) {
        struct v4l2_requestbuffers req = {};
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) return false;

        buffers_[fd].resize(req.count);
        for (unsigned i = 0; i < req.count; ++i) {
            struct v4l2_buffer buf = {};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) return false;

            buffers_[fd][i].length = buf.length;
            buffers_[fd][i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                                        MAP_SHARED, fd, buf.m.offset);
            if (buffers_[fd][i].start == MAP_FAILED) return false;

            // Queue buffer
            if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) return false;
        }

        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        return ioctl(fd, VIDIOC_STREAMON, &type) == 0;
    }

    void stopCapture(int fd) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd, VIDIOC_STREAMOFF, &type);
        for (auto& buf : buffers_[fd]) {
            if (buf.start && buf.start != MAP_FAILED) {
                munmap(buf.start, buf.length);
            }
        }
        buffers_[fd].clear();
    }

    bool grabFrame(int fd, DynalgoFrame& outFrame, int timeoutMs) {
        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        struct timeval tv = {timeoutMs / 1000, (timeoutMs % 1000) * 1000};
        if (select(fd + 1, &fds, nullptr, nullptr, &tv) <= 0) return false;

        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) return false;

        // Copy frame data
        size_t frameSize = buffers_[fd][buf.index].length;
        outFrame.data.assign(
            static_cast<uint8_t*>(buffers_[fd][buf.index].start),
            static_cast<uint8_t*>(buffers_[fd][buf.index].start) + frameSize);
        outFrame.width = cfg_.width;
        outFrame.height = cfg_.height;
        outFrame.format = DynalgoFormat::BGR;
        outFrame.timestampUs = buf.timestamp.tv_sec * 1000000ULL + buf.timestamp.tv_usec;

        // Re-queue buffer
        return ioctl(fd, VIDIOC_QBUF, &buf) == 0;
    }

    bool setControl(int fd, int cid, int value) {
        struct v4l2_control ctrl = {cid, value};
        return ioctl(fd, VIDIOC_S_CTRL, &ctrl) == 0;
    }

    bool getControl(int fd, int cid, int& value) const {
        struct v4l2_control ctrl = {cid, 0};
        if (ioctl(fd, VIDIOC_G_CTRL, &ctrl) < 0) return false;
        value = ctrl.value;
        return true;
    }

    void streamLoop() {
        StereoFrameSet frame;
        while (streaming_) {
            if (grab(frame, 100)) {
                if (callback_) callback_(frame);
            }
        }
    }

    void computeDepth(StereoFrameSet& frame) {
        // Placeholder: stereo matching (SGBM, BM, etc.)
        // Real implementation would use OpenCV stereo matching
        frame.disparity = DynalgoFrame{};
        frame.disparity.width = frame.leftRect.width;
        frame.disparity.height = frame.leftRect.height;
        frame.disparity.format = DynalgoFormat::Y16;
        frame.disparity.data.resize(frame.disparity.width * frame.disparity.height * 2);
        std::fill(frame.disparity.data.begin(), frame.disparity.data.end(), 0);

        frame.depth = DynalgoFrame{};
        frame.depth.width = frame.leftRect.width;
        frame.depth.height = frame.leftRect.height;
        // Note: No DEPTH_FLOAT32 format in DynalgoFormat enum; using Y16 as placeholder
        // Real implementation would add float32 depth format or use custom handling
        frame.depth.format = DynalgoFormat::Y16;
        frame.depth.data.resize(frame.depth.width * frame.depth.height * 2);
        std::fill(frame.depth.data.begin(), frame.depth.data.end(), 0);
    }

    struct Buffer { void* start = nullptr; size_t length = 0; };
    std::map<int, std::vector<Buffer>> buffers_;

    int leftFd_ = -1;
    int rightFd_ = -1;
    StereoConfig cfg_;
    StereoCameraInfo info_;
    uint32_t frameId_ = 0;
    bool isOpen_ = false;
    bool streaming_ = false;
    std::function<void(const StereoFrameSet&)> callback_;
    std::thread streamThread_;
};

std::unique_ptr<IStereoCamera> StereoCameraFactory::create(Vendor vendor) {
    switch (vendor) {
        case Vendor::GENERIC_UVC:
            return std::make_unique<UvcStereoCamera>();
        default:
            DYNALGO_LOG_ERROR_S("StereoCameraFactory: vendor not implemented: " << static_cast<int>(vendor));
            return nullptr;
    }
}

std::vector<std::string> StereoCameraFactory::discoverDevices(Vendor vendor) {
    std::vector<std::string> devices;
    // Scan /dev/video* for stereo pairs
    // Simplified implementation
    return devices;
}

} // namespace dynalgo