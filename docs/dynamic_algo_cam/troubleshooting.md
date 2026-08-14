# dynamic_algo_cam 故障诊断

## 1. 设备发现与连接

### 现象：程序提示 "No device found!"

**排查步骤**：
1. 确认设备已通过 USB 正确连接，`lsusb` 中能看到设备
   - Orbbec 设备 VID 为 `2bc5`
   - RoboSense RS-AC1 设备 VID 为 `3840`
   - 命令: `lsusb | grep -E '2bc5|3840'`
2. 检查 udev 规则是否已安装：
   ```bash
   # Orbbec
   ls /etc/udev/rules.d/ | grep orbbec
   # RS-AC1
   ls /etc/udev/rules.d/ | grep robosense
   ```
   若不存在，安装规则：
   ```bash
   # Orbbec
   sudo cp vendors/OrbbecSDK/scripts/99-obsensor-usb.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger

   # RS-AC1
   echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="3840", ATTR{idProduct}=="1010", MODE="0666"' | \
     sudo tee /etc/udev/rules.d/99-robosense-ac1.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
3. **RS-AC1 专属**：确认已 unbind 内核 `uvcvideo` 驱动
   ```bash
   lsusb | grep 3840:1010
   echo '1-5' | sudo tee /sys/bus/usb/drivers/uvcvideo/unbind 2>/dev/null || true
   ```
4. 拔插设备后重试
5. 非 root 用户需要 udev 规则授权才能访问 USB 设备
6. 检查编译选项：`ENABLE_ORBBEC`/`ENABLE_RS_AC1` 是否启用（查看 `CMakeCache.txt`）
7. 两个选项都 OFF 时 CMake 会报 `FATAL_ERROR`

### 现象：设备列表为空但 lsusb 可见

**原因**：权限不足或 SDK 版本与设备固件不兼容。

**修复**：
- 确认当前用户在 `plugdev` 组中：`groups $(whoami)`
- 更新 OrbbecSDK 到与设备匹配的版本

---

## 2. USB 带宽与多设备问题

### 现象：多设备时 `UVC_ERROR_NO_MEM` 或第二台设备启动失败

**原因**：Linux 默认 `usbfs_memory_mb=16`，不足以支撑多台 USB3 摄像头。

**修复**：
```bash
# 临时生效
echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb

# 永久生效
echo "options usbcore usbfs_memory_mb=256" | sudo tee /etc/modprobe.d/usbcore.conf
# 重启系统
```

程序启动时会自动检测并给出警告。

### 现象：多设备场景下某设备 FPS=0 或帧率极低

**原因**：USB 控制器带宽竞争。

**修复**：
1. 将设备分布到不同的 USB 控制器：`lsusb -t`
2. 降低分辨率或帧率
3. 使用 `--no-fusion` 减少处理开销
4. 使用 USB 3.0 接口，检查线缆质量

---

## 3. 硬件 D2C 对齐问题

### 现象：日志显示 "HW D2C: not supported, using software alignment"

**原因**：当前设备的流配置不支持硬件 D2C 对齐，程序自动回退到软件对齐。

**排查**：
1. 确认设备型号支持 HW D2C（如 Gemini 2、Femto Mega）
2. 确认彩色与深度流帧率一致（HW D2C 要求相同帧率）
3. 检查分辨率组合是否在设备的 D2C 支持列表中

**说明**：程序自动检测 HW D2C 支持情况。RS-AC1 始终为 HW D2C。
自本次改动起，`ObDevice::setupPipeline()` 在选深度 profile 时已优先在
`getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE)` 返回的 HW-D2C 支持集合内挑选
（详见技术参考 §7.1.1），进一步降低软件对齐发生率。若设备的 HW D2C 支持列表为空，
打分项等价于不加分，按原行为回退 SW 最佳 profile 选择。

### 现象：日志显示 "PCD fallback: self-computed back-projection fed pcdTask"

**原因**：本帧 SDK 未产出 `OB_FRAME_POINTS`（未启用 `setPointCloudEnabled(true)` 或
当帧 `pointCloudFilter_->process()` 抛出/返回空），`PointcloudFrameConsumer` 退路用针孔
反投影自算点云。详见技术参考 §7.5。

**排查**：
1. 若希望走 Driver 点云：确认 `ObPipeline::setPointCloudEnabled(true)` 被调用
   （`CaptureSession::createPcdTask()` 仅在 `pipeline_->isPcdEnabled()` 时启用 PCD 任务，
   Driver pipeline 必须已置 `pcdEnabled_=true`）。
2. Fallback 路径需要 `sensorInfo_.depthIntrinsic.fx != 0 && depthScale > 0`；缺失时该帧不写 PCD。
3. 双轨对比日志 `PCD dual-track: driverPts=... ownPts=...` 仅在 Driver 与自反投影同时存在时输出，
   用于校验，不替代主路径输出。

### 现象：RS-AC1 D2C 融合画面有明显块效应

**原因**：RS-AC1 深度分辨率仅 96×288，上采样到 1920×1080 后存在固有限制。

**缓解**：调低 `--alpha` 值，或调窄 `--depth-min` / `--depth-max` 范围。

---

## 4. H.264 录制与播放

### 现象：录制文件无法播放或损坏

**排查**：
1. 确认文件大小 > 0 字节
2. 使用 `ffplay -f h264 <file>.h264` 播放裸流
3. 若文件开头播放异常（花屏），可能是首帧非关键帧，需跳过前几秒
4. 封装为 MP4 后播放：
   ```bash
   ffmpeg -y -fflags +genpts -r 30 -i <file>.h264 -c copy output.mp4
   ```

### 现象：彩色流 H.264 文件为空（0 字节）

**原因**：MJPEG 解码或 H264 编码器初始化失败。

**排查**：
1. 检查 FFmpeg 库是否正确安装
2. 查看程序 stderr 输出的错误信息
3. 确认 `libx264` 编码器可用：`ffmpeg -encoders | grep libx264`

---

## 5. 深度数据问题

### 现象：深度值全为 0 或 65535

**原因**：
- 0 值：无效深度（超出量程或反射率过低）
- 65535 值：深度饱和（Gemini 305 常见，表示超出最大测量距离）

**修复**：
- 调整 `--depth-min` 和 `--depth-max` 过滤无效值
- 融合时，值为 0 的像素自动显示原始彩色

### 现象：深度 `.raw` 文件无法解析

**排查**：
1. 确认文件包含 44 字节头部（魔数 `NIO_DEPTH_RAW`，旧文件为 `ORBBEC_DEPTH_RAW`）
2. 使用解析工具：
   ```bash
   python3 app/tools/parse_depth_raw.py <file.raw> --stats
   ```
3. 确认头部的 width、height、scale 字段合理

---

## 6. IMU 数据问题

### 现象：IMU 数据文件为空

**原因**：设备不支持 IMU 或 IMU 管道启动失败。

**排查**：
1. Gemini 305 无独立 IMU — 属于正常行为
2. RS-AC1 固件/驱动当前不产出 IMU 数据 — 程序自动跳过 IMU 文件
3. Gemini 335L/336L 有 IMU，查看日志中 "Starting IMU pipeline" 是否成功

---

## 7. 性能与资源

### 现象：CPU 占用过高

**原因**：多设备 + D2C 融合需要大量 CPU 计算（MJPEG 解码 + 逐像素着色 + alpha 混合 + H264 编码）。

**优化建议**：
1. HW D2C 自动启用时可减少软件对齐开销
2. 使用 `--no-fusion` 禁用融合
3. 减少同时录制的设备数量
4. 降低帧率或分辨率

### 现象：内存持续增长

**排查**：
1. 确认程序版本包含 git commit 信息（日志中 "Git commit:" 行）
2. 检查是否有异常帧率导致编码队列积压
3. 使用 `valgrind --leak-check=full` 检测内存泄漏

---

## 8. RS-AC1 特定问题

### 现象：RS-AC1 设备未被发现

**排查**：
1. 确认 `lsusb` 中可见 RS-AC1 设备 (VID=**3840**, PID=1010)
   ```bash
   lsusb | grep 3840
   ```
2. 确认编译时 `ENABLE_RS_AC1=ON`（查看 `CMakeCache.txt`）
3. RS-AC1 需 USB 3.0 接口
4. 内核 `uvcvideo` 驱动必须 unbind

### 现象：RS-AC1 启动报 "Wrong Input Type 4"

**原因**：编译时未定义 `ENABLE_USB`。

**修复**：CMake `ENABLE_RS_AC1=ON` 时自动添加 `-DENABLE_USB`。确认 `app/driver/CMakeLists.txt` 中 RS-AC1 块的 `target_compile_definitions` 包含 `ENABLE_USB`。

### 现象：RS-AC1 所有输出文件 0 字节

**原因**：Pipeline 初始化失败（通常是权限或驱动绑定问题）。

**排查**：按上述 "设备未被发现" 步骤逐一检查。

### 现象：RS-AC1 IMU 温度字段为 0

**原因**：rs_driver 未暴露 IMU 温度数据，温度字段始终为 0.0。

**说明**：这是已知限制。IMU 数据本身（accel+gyro ~100Hz）正常工作。

---

## 9. 构建问题

### 现象：CMake 报 FATAL_ERROR "Both ENABLE_ORBBEC and ENABLE_RS_AC1 are OFF"

**原因**：两个选项都关闭，无厂商 SDK 被选中。

**修复**：至少启用一个：
```bash
cmake .. -DENABLE_ORBBEC=ON    # 或 -DENABLE_RS_AC1=ON
```

### 现象：链接错误 `undefined reference to dynalgo::discoverDevices()`

**原因**：`dynalgo_driver_factory.cpp` 未被编译。此文件在 `ENABLE_ORBBEC OR ENABLE_RS_AC1` 时应加入 `dynalgo_drivers` 目标。

**确认**：检查 `app/driver/CMakeLists.txt` 中存在 `if(ENABLE_ORBBEC OR ENABLE_RS_AC1)` 块包含 `dynalgo_driver_factory.cpp`。

### 现象：OrbbecSDK 链接失败

**排查**：
1. 确认 `ENABLE_ORBBEC=ON`
2. 确认 `vendors/OrbbecSDK` 子目录存在且完整
3. 检查 `OB_SDK_LIB_NAME` 缓存变量

---

## 10. 日志与调试

### 查看日志

程序日志默认写入与录制数据相同的目录下的 `dynalgo.log`。

### 关键日志行含义

| 日志关键字 | 含义 |
|-----------|------|
| `Git commit:` | 当前构建的 git commit hash |
| `HW D2C supported` | 设备支持硬件深度对齐 |
| `falling back to SW alignment` | Orbbec 设备不支持 HW D2C，使用软件对齐 |
| `usbfs_memory_mb=... too low` | USB 内存配置不足 |
| `Pipeline start failed` | 管道启动异常 |
| `Failed to init fused H264 encoder` | 融合编码器初始化失败 |
| `Found RS-AC1 device` | 发现 RoboSense RS-AC1 设备 |
| `Orbbec support disabled` | `ENABLE_ORBBEC=OFF`，跳过 OrbbecSDK |
| `RS-AC1 support disabled` | `ENABLE_RS_AC1=OFF`，跳过 RoboSense |

### 问题反馈

请附带以下信息：
1. 程序版本（git commit hash，日志中 "Git commit:" 行）
2. 设备型号与数量（Orbbec / RS-AC1）
3. 完整日志输出
4. 操作系统版本：`uname -a`
5. USB 拓扑：`lsusb -t`
6. 编译选项：检查 `CMakeCache.txt` 中 `ENABLE_ORBBEC` / `ENABLE_RS_AC1` 值
