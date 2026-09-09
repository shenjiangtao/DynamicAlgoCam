/*
 * V4L2 Camera HAL for NVIDIA platforms (fallback)
 */
#include <dynalgo/hal/camera_hal.hpp>
#include <dynalgo/hal/hal_types.hpp>

#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <chrono>
#include <queue>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

namespace dynalgo {

class V4L2CameraHAL : public hal::ICameraHAL {
public:
    V4L2CameraHAL() = default;
    ~V4L2CameraHAL() override { deinitialize(); }

    hal::Result<std::vector<hal::CameraDeviceInfo>> enumerateDevices() override {
        std::vector<hal::CameraDeviceInfo> devices;
        // Enumerate /dev/video* devices
        for (int i = 0; i < 16; ++i) {
            std::string dev_path = "/dev/video" + std::to_string(i);
            int fd = open(dev_path.c_str(), O_RDWR | O_NONBLOCK);
            if (fd >= 0) {
                struct v4l2_capability cap;
                if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
                    hal::CameraDeviceInfo info;
                    info.device_id = dev_path;
                    info.display_name = reinterpret_cast<const char*>(cap.card);
                    info.vendor = "generic";
                    info.bus_info = reinterpret_cast<const char*>(cap.bus_info);
                    info.is_available = true;
                    info.sensor_info.name = reinterpret_cast<const char*>(cap.card);
                    info.sensor_info.vendor = "generic";
                    info.sensor_info.max_width = 3840;
                    info.sensor_info.max_height = 2160;
                    info.sensor_info.supported_formats = {hal::PixelFormat::NV12, hal::PixelFormat::YUYV, hal::PixelFormat::MJPEG};
                    info.sensor_info.supported_resolutions = {{3840, 2160}, {1920, 1080}, {1280, 720}};
                    info.sensor_info.supported_fps = {30, 60};
                    info.sensor_info.supports_hdr = false;
                    info.sensor_info.supports_hw_sync = false;
                    devices.push_back(std::move(info));
                }
                close(fd);
            }
        }
        return hal::Result<std::vector<hal::CameraDeviceInfo>>::ok(std::move(devices));
    }

    hal::Result<std::vector<hal::StreamConfig>> getSupportedStreams(const std::string& device_id) override {
        std::vector<hal::StreamConfig> streams;
        int fd = open(device_id.c_str(), O_RDWR);
        if (fd < 0) return hal::Result<std::vector<hal::StreamConfig>>::err(hal::ErrorCode::DEVICE_ERROR);
        
        struct v4l2_fmtdesc fmt = {0};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        while (ioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0) {
            struct v4l2_frmsizeenum frmsize = {0};
            frmsize.pixel_format = fmt.pixelformat;
            while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
                if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
                    hal::StreamConfig sc;
                    sc.stream_id = streams.size();
                    sc.width = frmsize.discrete.width;
                    sc.height = frmsize.discrete.height;
                    sc.fps = 30;
                    sc.buffer_count = 4;
                    // Map V4L2 format to HAL format
                    switch (fmt.pixelformat) {
                        case V4L2_PIX_FMT_NV12: sc.format = hal::PixelFormat::NV12; break;
                        case V4L2_PIX_FMT_YUYV: sc.format = hal::PixelFormat::YUYV; break;
                        case V4L2_PIX_FMT_MJPEG: sc.format = hal::PixelFormat::MJPEG; break;
                        default: sc.format = hal::PixelFormat::NV12; break;
                    }
                    streams.push_back(sc);
                }
                frmsize.index++;
            }
            fmt.index++;
        }
        close(fd);
        return hal::Result<std::vector<hal::StreamConfig>>::ok(std::move(streams));
    }

    hal::Result<hal::SensorInfo> getSensorInfo(const std::string& device_id) override {
        hal::SensorInfo info;
        info.name = "V4L2 Camera";
        info.vendor = "generic";
        info.max_width = 3840;
        info.max_height = 2160;
        info.supported_formats = {hal::PixelFormat::NV12, hal::PixelFormat::YUYV, hal::PixelFormat::MJPEG};
        info.supported_resolutions = {{3840, 2160}, {1920, 1080}, {1280, 720}};
        info.supported_fps = {30, 60};
        info.supports_hdr = false;
        info.supports_hw_sync = false;
        return hal::Result<hal::SensorInfo>::ok(std::move(info));
    }

    hal::ResultVoid initialize(const hal::CameraConfig& config) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_initialized) return hal::ResultVoid::err(hal::ErrorCode::ALREADY_INITIALIZED);
        
        m_config = config;
        m_fd = open(config.device_id.c_str(), O_RDWR | O_NONBLOCK);
        if (m_fd < 0) return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
        
        struct v4l2_format fmt = {0};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = config.width;
        fmt.fmt.pix.height = config.height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        if (ioctl(m_fd, VIDIOC_S_FMT, &fmt) < 0) {
            close(m_fd);
            return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
        }
        
        struct v4l2_streamparm parm = {0};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator = config.fps;
        ioctl(m_fd, VIDIOC_S_PARM, &parm);
        
        struct v4l2_requestbuffers req = {0};
        req.count = config.buffer_count;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(m_fd, VIDIOC_REQBUFS, &req) < 0) {
            close(m_fd);
            return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
        }
        
        m_buffers.resize(req.count);
        for (unsigned int i = 0; i < req.count; ++i) {
            struct v4l2_buffer buf = {0};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (ioctl(m_fd, VIDIOC_QUERYBUF, &buf) < 0) return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
            m_buffers[i].length = buf.length;
            m_buffers[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, buf.m.offset);
            if (m_buffers[i].start == MAP_FAILED) return hal::ResultVoid::err(hal::ErrorCode::OUT_OF_MEMORY);
        }
        
        for (unsigned int i = 0; i < req.count; ++i) {
            struct v4l2_buffer buf = {0};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            ioctl(m_fd, VIDIOC_QBUF, &buf);
        }
        
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(m_fd, VIDIOC_STREAMON, &type);
        
        m_initialized = true;
        m_running = true;
        m_capture_thread = std::thread(&V4L2CameraHAL::captureLoop, this);
        
        return hal::ResultVoid::ok();
    }

    hal::ResultVoid initializeMulti(const hal::MultiCameraConfig& config) override {
        return hal::ResultVoid::err(hal::ErrorCode::NOT_SUPPORTED);
    }

    hal::ResultVoid start() override { return hal::ResultVoid::ok(); }
    hal::ResultVoid stop() override { return hal::ResultVoid::ok(); }

    hal::ResultVoid deinitialize() override {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_initialized) return hal::ResultVoid::ok();
        
        if (m_running) {
            m_running = false;
            if (m_capture_thread.joinable()) m_capture_thread.join();
        }
        
        if (m_fd >= 0) {
            enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(m_fd, VIDIOC_STREAMOFF, &type);
            for (auto& buf : m_buffers) {
                if (buf.start) munmap(buf.start, buf.length);
            }
            close(m_fd);
            m_fd = -1;
        }
        m_initialized = false;
        return hal::ResultVoid::ok();
    }

    hal::Result<hal::FrameMetadata> acquireFrame(uint32_t timeout_ms = 1000) override {
        std::unique_lock<std::mutex> lock(m_frame_mutex);
        if (m_frame_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return !m_frame_queue.empty(); })) {
            auto frame = std::move(m_frame_queue.front());
            m_frame_queue.pop();
            return hal::Result<hal::FrameMetadata>::ok(std::move(frame));
        }
        return hal::Result<hal::FrameMetadata>::err(hal::ErrorCode::TIMEOUT);
    }

    hal::Result<std::vector<hal::FrameMetadata>> acquireFrames(uint32_t timeout_ms = 1000) override {
        std::unique_lock<std::mutex> lock(m_frame_mutex);
        std::vector<hal::FrameMetadata> frames;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        
        while (frames.size() < m_config.buffer_count) {
            auto remaining = deadline - std::chrono::steady_clock::now();
            if (remaining.count() <= 0) break;
            if (m_frame_cv.wait_for(lock, remaining, [this] { return !m_frame_queue.empty(); })) {
                frames.push_back(std::move(m_frame_queue.front()));
                m_frame_queue.pop();
            } else break;
        }
        if (frames.empty()) return hal::Result<std::vector<hal::FrameMetadata>>::err(hal::ErrorCode::TIMEOUT);
        return hal::Result<std::vector<hal::FrameMetadata>>::ok(std::move(frames));
    }

    hal::ResultVoid registerCallback(hal::ICameraHAL::FrameCallback cb) override {
        m_user_callback = std::move(cb);
        return hal::ResultVoid::ok();
    }

    hal::ResultVoid unregisterCallback() override {
        m_user_callback = nullptr;
        return hal::ResultVoid::ok();
    }

    hal::Result<hal::BufferHandle> importBuffer(const hal::BufferHandle& external) override {
        return hal::Result<hal::BufferHandle>::err(hal::ErrorCode::NOT_SUPPORTED);
    }

    hal::ResultVoid releaseBuffer(const hal::BufferHandle& buffer) override {
        return hal::ResultVoid::ok();
    }

    hal::ResultVoid releaseFrames(const std::vector<hal::FrameMetadata>& frames) override {
        return hal::ResultVoid::ok();
    }

    hal::ResultVoid setControl(const std::string& device_id, hal::CameraControl control, int64_t value) override {
        if (m_fd < 0) return hal::ResultVoid::err(hal::ErrorCode::NOT_INITIALIZED);
        struct v4l2_control ctrl = {0};
        switch (control) {
            case hal::CameraControl::EXPOSURE_TIME: ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE; break;
            case hal::CameraControl::GAIN: ctrl.id = V4L2_CID_GAIN; break;
            default: return hal::ResultVoid::err(hal::ErrorCode::NOT_SUPPORTED);
        }
        ctrl.value = value;
        return ioctl(m_fd, VIDIOC_S_CTRL, &ctrl) == 0 ? hal::ResultVoid::ok() : hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
    }

    hal::Result<int64_t> getControl(const std::string& device_id, hal::CameraControl control) override {
        if (m_fd < 0) return hal::Result<int64_t>::err(hal::ErrorCode::NOT_INITIALIZED);
        struct v4l2_control ctrl = {0};
        switch (control) {
            case hal::CameraControl::EXPOSURE_TIME: ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE; break;
            case hal::CameraControl::GAIN: ctrl.id = V4L2_CID_GAIN; break;
            default: return hal::Result<int64_t>::err(hal::ErrorCode::NOT_SUPPORTED);
        }
        if (ioctl(m_fd, VIDIOC_G_CTRL, &ctrl) < 0) return hal::Result<int64_t>::err(hal::ErrorCode::DEVICE_ERROR);
        return hal::Result<int64_t>::ok(ctrl.value);
    }

    hal::ResultVoid triggerFrame(const std::string& device_id) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_SUPPORTED); }
    hal::ResultVoid triggerFrames(const std::vector<std::string>& device_ids) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_SUPPORTED); }
    hal::ResultVoid flush() override {
        std::lock_guard<std::mutex> lock(m_frame_mutex);
        std::queue<hal::FrameMetadata> empty;
        std::swap(m_frame_queue, empty);
        return hal::ResultVoid::ok();
    }

    hal::Result<std::string> getName() const override { return hal::Result<std::string>::ok("V4L2 Camera HAL"); }
    hal::Result<std::string> getVendor() const override { return hal::Result<std::string>::ok("generic"); }
    hal::Result<std::string> getVersion() const override { return hal::Result<std::string>::ok("1.0.0"); }
    hal::ResultVoid vendorCommand(uint32_t, const void*, size_t, void*, size_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_SUPPORTED); }
    bool isRunning() const override { return m_running; }
    uint32_t getDroppedFrameCount() const override { return 0; }
    void resetDroppedFrameCount() override {}

private:
    struct Buffer { void* start; size_t length; };
    
    void captureLoop() {
        while (m_running) {
            struct v4l2_buffer buf = {0};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            if (ioctl(m_fd, VIDIOC_DQBUF, &buf) < 0) continue;
            
            hal::FrameMetadata frame;
            frame.timestamp_ns = hal::now_ns();
            frame.frame_id = m_frame_counter++;
            frame.format = hal::PixelFormat::NV12;
            frame.width = m_config.width;
            frame.height = m_config.height;
            frame.buffer.handle = reinterpret_cast<uint64_t>(m_buffers[buf.index].start);
            frame.buffer.platform_id = 0;
            
            {
                std::lock_guard<std::mutex> lock(m_frame_mutex);
                m_frame_queue.push(std::move(frame));
                m_frame_cv.notify_one();
            }
            
            ioctl(m_fd, VIDIOC_QBUF, &buf);
        }
    }

    std::mutex m_mutex;
    hal::CameraConfig m_config;
    int m_fd = -1;
    std::vector<Buffer> m_buffers;
    std::thread m_capture_thread;
    std::atomic<bool> m_initialized{false};
    std::atomic<bool> m_running{false};
    std::atomic<uint32_t> m_frame_counter{0};
    
    std::mutex m_frame_mutex;
    std::condition_variable m_frame_cv;
    std::queue<hal::FrameMetadata> m_frame_queue;
    hal::ICameraHAL::FrameCallback m_user_callback;
    std::atomic<bool> m_initialized{false};
    std::atomic<bool> m_running{false};
    std::atomic<uint32_t> m_frame_counter{0};
};

} // namespace dynalgo

extern "C" {
dynalgo::hal::ICameraHAL* dynalgo_hal_camera_create() { return new dynalgo::V4L2CameraHAL(); }
void dynalgo_hal_camera_destroy(dynalgo::hal::ICameraHAL* ptr) { delete ptr; }
}