# nio_multi_capture 故障诊断

## 1. 设备发现与连接

### 现象：程序提示 "No Orbbec device found!"

**排查步骤**：
1. 确认设备已通过 USB 正确连接，`lsusb` 中能看到 Orbbec 设备
2. 检查 udev 规则是否已安装：
   ```bash
   ls /etc/udev/rules.d/ | grep orbbec
   ```
   若不存在，安装规则：
   ```bash
   sudo cp /opt/orbbec/99-orbbec-usb.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
3. 拔插设备后重试
4. 非	root 用户需要 udev 规则授权才能访问 USB 设备

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
sudo modprobe -r usbcore && sudo modprobe usbcore
# 或重启系统
```

程序启动时会自动检测并给出警告。

### 现象：多设备场景下某设备 FPS=0 或帧率极低

**原因**：USB 控制器带宽竞争。

**修复**：
1. 将设备分布到不同的 USB 控制器（可用 `lsusb -t` 查看）
2. 降低分辨率或帧率
3. 使用 `--no-fusion` 减少处理开销
4. 检查 USB 线缆质量，优先使用 USB 3.0 接口

---

## 3. 硬件 D2C 对齐问题

### 现象：日志显示 "HW D2C: not supported, using software alignment"

**原因**：当前设备的流配置不支持硬件 D2C 对齐，程序自动回退到软件对齐。

**排查**：
1. 确认设备型号支持 HW D2C（如 Gemini 2、Femto Mega 等）
2. 确认彩色与深度流帧率一致（HW D2C 要求相同帧率）
3. 检查分辨率组合是否在设备的 D2C 支持列表中
4. 查看 SDK 日志中 `getD2CDepthProfileList` 返回的结果

**说明**：程序会自动检测 HW D2C 支持情况，无需手动配置。支持时使用硬件对齐，不支持时自动回退到软件对齐。

### 现象：HW D2C 模式下深度帧仍为原始分辨率

**原因**：硬件 D2C 应在设备端完成对齐，深度帧应已对齐到彩色分辨率。如果未生效：
1. 确认 `config->setAlignMode(ALIGN_D2C_HW_MODE)` 已调用
2. 检查设备固件是否为最新版本
3. 程序会自动回退到 SW D2C 模式

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
1. 确认文件包含 44 字节头部（魔数 `ORBBEC_DEPTH_RAW`）
2. 使用 `parse_depth_raw.py` 解析：
   ```bash
   python3 parse_depth_raw.py <file.raw> --stats
   ```
3. 确认头部的 width、height、scale 字段合理

---

## 6. IMU 数据问题

### 现象：IMU 数据文件为空

**原因**：设备不支持 IMU 或 IMU 管道启动失败。

**排查**：
1. 确认设备有加速度计和陀螺仪（如 Gemini 305 无独立 IMU）
2. 查看日志中 "Starting IMU pipeline" 是否成功
3. 某些设备需要单独启动 IMU 管道

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

## 8. 日志与调试

### 查看日志

程序日志默认写入与录制数据相同的目录，也可通过以下方式查看实时日志：
```bash
# 日志文件路径在启动时打印
# 日志级别: TRACE（最详细）
```

### 关键日志行含义

| 日志关键字 | 含义 |
|-----------|------|
| `Git commit:` | 当前构建的 git commit hash |
| `HW D2C supported` | 设备支持硬件深度对齐 |
| `falling back to SW alignment` | 设备不支持 HW D2C，使用软件对齐 |
| `usbfs_memory_mb=... too low` | USB 内存配置不足 |
| `Pipeline start failed` | 管道启动异常 |
| `Failed to init fused H264 encoder` | 融合编码器初始化失败 |

### 问题反馈

请附带以下信息：
1. 程序版本（git commit hash）
2. 设备型号与数量
3. 完整日志输出
4. 操作系统版本（`uname -a`）
5. USB 拓扑结构（`lsusb -t`）
