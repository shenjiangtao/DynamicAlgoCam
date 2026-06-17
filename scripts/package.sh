#!/usr/bin/env bash
# package.sh — 一键打包 NIO Orbbec 采集工具，目标机器解压即用
#
# Usage:
#   ./scripts/package.sh               # 打包 nio_multi_capture
#   ./scripts/package.sh --output /tmp/my_pkg  # 自定义输出路径
#   ./scripts/package.sh --skip-libs   # 不打包系统 .so（目标机已有）
#
# 输出: <output_dir>/nio_capture_tools_<arch>_<date>.tar.gz
# 解压后目录结构:
# nio_capture_tools/
# ├── bin/                # nio_multi_capture 可执行文件 + detect_orbbec_usb.sh
# ├── lib/                # libOrbbecSDK.so + extensions + 运行依赖 .so
# ├── extensions/         # SDK extensions (frameprocessor, filters 等)
# ├── OrbbecSDKConfig.xml # SDK 配置文件 (供 CWD 查找)
# ├── config/             # OrbbecSDKConfig.xml 副本
# ├── docs/               # use_guide.md + troubleshooting.md
# ├── rules/              # udev 规则 (99-obsensor-libusb.rules)
# └── README.txt          # 使用说明

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"
BIN_DIR="${BUILD_DIR}/linux_x86_64/bin"
LIB_DIR="${BUILD_DIR}/linux_x86_64/lib"

DEFAULT_OUTPUT_DIR="${PROJECT_ROOT}/dist"
SKIP_SYSTEM_LIBS=false
TARGET_BINARIES=("nio_multi_capture")

while [ $# -gt 0 ]; do
case "$1" in
--output|-o)
    shift; DEFAULT_OUTPUT_DIR="$1"; shift ;;
--skip-libs)
    SKIP_SYSTEM_LIBS=true; shift ;;
--help|-h)
    sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# //'
    exit 0 ;;
-*)
    echo "未知选项: $1" >&2; exit 1 ;;
*)
    TARGET_BINARIES+=("$1"); shift ;;
esac
done

ARCH="$(uname -m)"
DATE_TAG="$(date +%Y%m%d_%H%M%S)"
PKG_NAME="nio_capture_tools_${ARCH}_${DATE_TAG}"
STAGE_DIR="/tmp/${PKG_NAME}"

if [ ${#TARGET_BINARIES[@]} -eq 0 ]; then
    echo "错误: 未指定目标程序" >&2
    exit 1
fi

echo "========================================="
echo "  NIO Capture Tools 打包工具"
echo "========================================="
echo "项目根目录:  ${PROJECT_ROOT}"
echo "构建目录:    ${BUILD_DIR}"
echo "目标程序:    ${TARGET_BINARIES[*]}"
echo "输出目录:    ${DEFAULT_OUTPUT_DIR}"
echo "跳过系统库:  ${SKIP_SYSTEM_LIBS}"
echo "========================================="

for bin_name in "${TARGET_BINARIES[@]}"; do
    if [ ! -f "${BIN_DIR}/${bin_name}" ]; then
        echo "错误: 可执行文件不存在 — ${BIN_DIR}/${bin_name}" >&2
        echo "请先编译: cd build && cmake --build . --target ${bin_name}" >&2
        exit 1
    fi
done

if [ ! -f "${LIB_DIR}/libOrbbecSDK.so" ]; then
    echo "错误: libOrbbecSDK.so 未找到于 ${LIB_DIR}/" >&2
    exit 1
fi

rm -rf "${STAGE_DIR}"
mkdir -p "${STAGE_DIR}"/{bin,lib,config,docs,rules}

echo ""
echo "[1/6] 复制可执行文件..."
for bin_name in "${TARGET_BINARIES[@]}"; do
    cp -v "${BIN_DIR}/${bin_name}" "${STAGE_DIR}/bin/"
    chmod +x "${STAGE_DIR}/bin/${bin_name}"
done

DETECT_USB_SRC="${PROJECT_ROOT}/scripts/nio_multi_capture/detect_orbbec_usb.sh"
if [ -f "${DETECT_USB_SRC}" ]; then
    cp -v "${DETECT_USB_SRC}" "${STAGE_DIR}/bin/"
    chmod +x "${STAGE_DIR}/bin/detect_orbbec_usb.sh"
else
    echo "警告: USB 检测脚本未找到 — ${DETECT_USB_SRC}" >&2
fi

echo ""
echo "[2/6] 复制 libOrbbecSDK.so + extensions..."
cp -v "${LIB_DIR}/libOrbbecSDK.so"        "${STAGE_DIR}/lib/"
cp -v "${LIB_DIR}/libOrbbecSDK.so."*      "${STAGE_DIR}/lib/" 2>/dev/null || true
cp -v "${LIB_DIR}/OrbbecSDKConfig.xml"    "${STAGE_DIR}/"
cp -v "${LIB_DIR}/OrbbecSDKConfig.xml"    "${STAGE_DIR}/config/"

if [ -d "${LIB_DIR}/extensions" ]; then
    cp -rv "${LIB_DIR}/extensions" "${STAGE_DIR}/"
fi

UDEV_RULES_SRC="${PROJECT_ROOT}/scripts/env_setup/99-obsensor-libusb.rules"
if [ -f "${UDEV_RULES_SRC}" ]; then
    cp -v "${UDEV_RULES_SRC}" "${STAGE_DIR}/rules/"
else
    echo "警告: udev 规则文件未找到 — ${UDEV_RULES_SRC}" >&2
fi

echo ""
echo "[3/6] 收集共享库..."

# 核心系统库 — 这些由 glibc/libc 提供, 随 Linux 发行版自带, 无需打包
SKIP_LIB_PATTERNS=(
    linux-vdso
    ld-linux
    libc.so
    libc-2.
    libm.so
    libm-2.
    libpthread.so
    libpthread-2.
    libdl.so
    libdl-2.
    librt.so
    librt-2.
    libgcc_s.so
    libstdc++.so
)

should_skip_lib() {
    local lib_name="$1"
    for pat in "${SKIP_LIB_PATTERNS[@]}"; do
        if [[ "$lib_name" == *"$pat"* ]]; then
            return 0
        fi
    done
    return 1
}

collect_system_libs() {
    local collected=0

    if [ "$SKIP_SYSTEM_LIBS" = true ]; then
        echo "  (跳过系统库收集)"
        return
    fi

    for bin_name in "${TARGET_BINARIES[@]}"; do
        local bin_path="${STAGE_DIR}/bin/${bin_name}"
        local resolved_libs
        resolved_libs=$(ldd "$bin_path" 2>/dev/null | grep '=>' | grep -v 'not found' | awk '{print $3}' | sort -u)

        for lib_path in $resolved_libs; do
            [ -z "$lib_path" ] && continue
            local lib_name
            lib_name="$(basename "$lib_path")"

            if should_skip_lib "$lib_name"; then
                continue
            fi

            if [ ! -f "${STAGE_DIR}/lib/$lib_name" ]; then
                cp -vn "$lib_path" "${STAGE_DIR}/lib/" 2>/dev/null && {
                    cp -vn "$(readlink -f "$lib_path")" "${STAGE_DIR}/lib/" 2>/dev/null || true
                    collected=$((collected + 1))
                    echo "  + $lib_name (from $bin_name)"
                }
            fi
        done
    done

    # 同样收集 libOrbbecSDK.so 的运行依赖
    if [ -f "${STAGE_DIR}/lib/libOrbbecSDK.so" ]; then
        local sdk_libs
        sdk_libs=$(ldd "${STAGE_DIR}/lib/libOrbbecSDK.so" 2>/dev/null | grep '=>' | grep -v 'not found' | awk '{print $3}' | sort -u)
        for lib_path in $sdk_libs; do
            [ -z "$lib_path" ] && continue
            local lib_name
            lib_name="$(basename "$lib_path")"

            if should_skip_lib "$lib_name"; then
                continue
            fi

            if [ ! -f "${STAGE_DIR}/lib/$lib_name" ]; then
                cp -vn "$lib_path" "${STAGE_DIR}/lib/" 2>/dev/null && {
                    cp -vn "$(readlink -f "$lib_path")" "${STAGE_DIR}/lib/" 2>/dev/null || true
                    collected=$((collected + 1))
                    echo "  + $lib_name (from libOrbbecSDK.so)"
                }
            fi
        done
    fi

    # 收集 lib/ 下已打包 .so 的传递依赖
    local transitive_libs
    transitive_libs=$(ldd "${STAGE_DIR}"/lib/*.so* 2>/dev/null | grep '=>' | grep -v 'not found' | awk '{print $3}' | sort -u)
    for lib_path in $transitive_libs; do
        [ -z "$lib_path" ] && continue
        local lib_name
        lib_name="$(basename "$lib_path")"

        if should_skip_lib "$lib_name"; then
            continue
        fi

        if [ ! -f "${STAGE_DIR}/lib/$lib_name" ]; then
            cp -vn "$lib_path" "${STAGE_DIR}/lib/" 2>/dev/null && {
                cp -vn "$(readlink -f "$lib_path")" "${STAGE_DIR}/lib/" 2>/dev/null || true
                collected=$((collected + 1))
                echo "  + $lib_name (transitive)"
            }
        fi
    done

    echo "  收集了 ${collected} 个共享库"
}

collect_system_libs

echo ""
echo "[4/6] 复制文档..."

MULTI_CAPTURE_DIR="${PROJECT_ROOT}/docs/nio_multi_capture"

if [ -f "${MULTI_CAPTURE_DIR}/use_guide.md" ]; then
    cp -v "${MULTI_CAPTURE_DIR}/use_guide.md" "${STAGE_DIR}/docs/"
fi

if [ -f "${MULTI_CAPTURE_DIR}/troubleshooting.md" ]; then
    cp -v "${MULTI_CAPTURE_DIR}/troubleshooting.md" "${STAGE_DIR}/docs/"
fi

echo ""
echo "[5/6] 生成运行脚本和 README..."

for bin_name in "${TARGET_BINARIES[@]}"; do
RUN_SCRIPT="${STAGE_DIR}/run_${bin_name}"
cat > "$RUN_SCRIPT" <<'RUNEOF'
#!/usr/bin/env bash
# run_<BIN> — 一键运行脚本
# 用法:
#   ./run_<BIN> [程序参数...]
#   ./run_<BIN> -c "305" "336L" -s /HDD/nio_capture
#   ./run_<BIN> --alpha 0.6
#   ./run_<BIN> --help
#
# 环境变量:
#   NIO_USBFS_MB — 手动指定 usbfs_memory_mb (默认: 自动计算, 每设备 256MB)
#   NIO_USBFS_PER_DEV — 每设备所需 usbfs 内存 MB (默认: 256)
set -euo pipefail

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LD_LIBRARY_PATH="${SELF_DIR}/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# SDK 通过 CWD 查找 OrbbecSDKConfig.xml 和 extensions/ 目录
# 必须先 cd 到包目录, 再启动程序
cd "${SELF_DIR}"

if [ ! -f "${SELF_DIR}/lib/libOrbbecSDK.so" ]; then
    echo "错误: libOrbbecSDK.so 未找到于 ${SELF_DIR}/lib/" >&2
    exit 1
fi

# ---- 检查 udev 规则 ----
UDEV_RULES_TARGET="/etc/udev/rules.d/99-obsensor-libusb.rules"
UDEV_RULES_LOCAL="${SELF_DIR}/rules/99-obsensor-libusb.rules"

if [ ! -f "${UDEV_RULES_TARGET}" ]; then
    echo "========================================="
    echo " udev 规则检查"
    echo "========================================="
    echo "  ${UDEV_RULES_TARGET} 不存在"
    echo "  Orbbec USB 设备需要 udev 规则才能非 root 访问"

    if [ -f "${UDEV_RULES_LOCAL}" ]; then
        echo "  正在安装 udev 规则..."
        if sudo cp "${UDEV_RULES_LOCAL}" "${UDEV_RULES_TARGET}" 2>/dev/null; then
            sudo udevadm control --reload-rules 2>/dev/null && sudo udevadm trigger 2>/dev/null
            echo "成功: udev 规则已安装到 ${UDEV_RULES_TARGET}"
        else
            echo "错误: 无法安装 udev 规则 (需要 sudo 权限)" >&2
            echo "  请手动执行:" >&2
            echo "  sudo cp ${UDEV_RULES_LOCAL} ${UDEV_RULES_TARGET}" >&2
            echo "  sudo udevadm control --reload-rules && sudo udevadm trigger" >&2
            exit 1
        fi
    else
        echo "警告: 包内未找到 ${UDEV_RULES_LOCAL}, 跳过自动安装" >&2
    fi
    echo "========================================="
fi

# ---- 检查当前用户是否在 video 组 (udev 规则指定的 GROUP) ----
UDEV_GROUP="video"
if ! id -nG 2>/dev/null | grep -qw "${UDEV_GROUP}"; then
    echo "========================================="
    echo " 用户组检查"
    echo "========================================="
    echo "  当前用户 '$(whoami)' 不在 '${UDEV_GROUP}' 组中"
    echo "  Orbbec USB 设备的 udev 规则将访问权限授予 ${UDEV_GROUP} 组"
    echo "  正在将用户加入 ${UDEV_GROUP} 组..."
    if sudo usermod -aG "${UDEV_GROUP}" "$(whoami)" 2>/dev/null; then
        echo "成功: 用户已加入 ${UDEV_GROUP} 组"
        echo "  注意: 组成员变更需重新登录后生效"
        echo "  当前会话可通过 'newgrp ${UDEV_GROUP}' 临时生效"
        echo "  或拔插 USB 设备后重试"
    else
        echo "错误: 无法将用户加入 ${UDEV_GROUP} 组 (需要 sudo 权限)" >&2
        echo "  请手动执行: sudo usermod -aG ${UDEV_GROUP} $(whoami)" >&2
        exit 1
    fi
    echo "========================================="
fi

# ---- 自动检测并设置 usbfs_memory_mb ----
USBFS_PER_DEV="${NIO_USBFS_PER_DEV:-256}"

# 检测 Orbbec USB 设备数量 (VID=2bc5)
DEVICE_COUNT=$(lsusb 2>/dev/null | grep -i -E '2bc5|orbbec' | wc -l)
[ "${DEVICE_COUNT}" -eq 0 ] && DEVICE_COUNT=1

# 若用户通过 NIO_USBFS_MB 手动指定, 优先使用; 否则按 设备数 * 每设备需求 自动计算
if [ -n "${NIO_USBFS_MB:-}" ]; then
    USBFS_MB="${NIO_USBFS_MB}"
else
    USBFS_MB=$((DEVICE_COUNT * USBFS_PER_DEV))
fi

# 每个 USB3 设备最高可用bulk传输约 24MB/s，但usbfs缓冲区需要按流数分配
# 单设备至少 128MB, 多设备建议每设备 256MB
[ "${USBFS_MB}" -lt 128 ] && USBFS_MB=128

CURRENT_USBFS="$(cat /sys/module/usbcore/parameters/usbfs_memory_mb 2>/dev/null || echo 0)"

if [ "${CURRENT_USBFS}" -lt "${USBFS_MB}" ] 2>/dev/null; then
    echo "========================================="
    echo " usbfs_memory_mb 检查"
    echo "========================================="
    echo "  当前值:    ${CURRENT_USBFS} MB"
    echo "  需要值:    ${USBFS_MB} MB (设备数=${DEVICE_COUNT}, 每设备=${USBFS_PER_DEV}MB)"
    echo "  差额:      $((USBFS_MB - CURRENT_USBFS)) MB"
    echo "========================================="

    echo "正在临时修改 usbfs_memory_mb..."
    if echo "${USBFS_MB}" | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb >/dev/null 2>&1; then
        NEW_USBFS="$(cat /sys/module/usbcore/parameters/usbfs_memory_mb 2>/dev/null || echo 0)"
        if [ "${NEW_USBFS}" -ge "${USBFS_MB}" ] 2>/dev/null; then
            echo "成功: usbfs_memory_mb 已设置为 ${NEW_USBFS} MB"
        else
            echo "警告: 写入成功但读回值 ${NEW_USBFS}MB 不满足要求" >&2
        fi
    else
        echo "错误: 无法写入 usbfs_memory_mb (需要 sudo 权限)" >&2
        echo "  请手动执行: echo ${USBFS_MB} | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb" >&2
        echo "  或永久生效: echo 'options usbcore usbfs_memory_mb=${USBFS_MB}' | sudo tee /etc/modprobe.d/usbcore.conf" >&2
        exit 1
    fi

    # 提示永久配置
    if [ ! -f /etc/modprobe.d/usbcore.conf ] || ! grep -q "usbfs_memory_mb" /etc/modprobe.d/usbcore.conf 2>/dev/null; then
        echo ""
        echo "提示: 当前修改仅临时生效, 重启后将失效。永久生效请执行:"
        echo "  echo 'options usbcore usbfs_memory_mb=${USBFS_MB}' | sudo tee /etc/modprobe.d/usbcore.conf"
        echo ""
    fi
fi

echo "usbfs_memory_mb 已满足要求 (${USBFS_MB}MB), 启动应用..."
exec "${SELF_DIR}/bin/<BIN>" "$@"
RUNEOF
sed -i "s|<BIN>|${bin_name}|g" "$RUN_SCRIPT"
chmod +x "$RUN_SCRIPT"
echo "  生成 run_${bin_name}"
done

cat > "${STAGE_DIR}/README.txt" << 'README'
=======================================
NIO Orbbec 采集工具包
=======================================

1. 快速开始
-----------
./run_nio_multi_capture                     # 默认录制所有摄像头
./run_nio_multi_capture --help              # 查看参数
./run_nio_multi_capture -c "305"            # 仅录制 305 型号
./run_nio_multi_capture -s /data/cap        # 指定保存目录
./run_nio_multi_capture --no-fusion         # 仅录制原始流

2. USB 配置（多摄像头必读）
--------------------------
run_nio_multi_capture 启动时会自动检测 Orbbec USB 设备数量,
并自动计算和设置 usbfs_memory_mb (每设备 256MB):
  - 若当前值不足, 将自动通过 sudo 临时修改
  - 若自动修改失败(无 sudo 权限), 脚本将退出并提示手动命令

手动配置:
  echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb

自定义每设备内存 (默认 256MB):
  NIO_USBFS_PER_DEV=512 ./run_nio_multi_capture

手动指定总内存:
  NIO_USBFS_MB=1024 ./run_nio_multi_capture

永久生效: 在 /etc/modprobe.d/ 创建文件:
  echo "options usbcore usbfs_memory_mb=1024" | sudo tee /etc/modprobe.d/usbcore.conf

3. 依赖说明
-----------
- 本包已自带 libOrbbecSDK.so 及 extensions + libSDL2 + 所有运行依赖 .so
- 运行依赖 .so 已内置于 lib/ 目录，通过 RPATH + LD_LIBRARY_PATH 加载
- SDK 配置文件 OrbbecSDKConfig.xml 位于包根目录 (SDK 通过 CWD 查找)
- 运行脚本会自动 cd 到包目录, 确保路径正确
- 需要 Linux x86_64, glibc >= 2.31 (libc/libm/libpthread/libstdc++ 等核心库由系统提供)

4. USB 检测工具
--------------
bin/detect_orbbec_usb.sh — 检测已连接的 Orbbec 设备:
  ./bin/detect_orbbec_usb.sh
  输出: 设备型号、USB 总线/速率、/dev/bus/usb 路径、USB2/3 汇总

5. 文档
-------
docs/use_guide.md       — 详细使用说明与架构设计
docs/troubleshooting.md — 故障诊断与排查指南

6. 输出文件说明
---------------
*_color_*.h264         — 彩色流 H264
*_depth_*.h264         — 深度流 H264 (jet colormap 编码)
*_depth_raw_*.raw      — 原始深度 Y16 数据
*_ir_*.h264            — 红外流 H264
*_imu_*.txt            — IMU 数据 CSV (host_ts_ms,type,device_ts_us,x,y,z,temp)
*_d2c_fused_*.h264     — D2C 融合 (深度 jet + 彩色 alpha 混合) H264

H264 文件中每帧前有 SEI NAL 单元, dts= 字段为相机设备时间戳(微秒)

7. 权限
-------
run_nio_multi_capture 启动时会自动检查:
  a) /etc/udev/rules.d/99-obsensor-libusb.rules 是否已安装 (未安装则自动 sudo cp)
  b) 当前用户是否在 video 组 (udev 规则将 USB 设备权限授予 video 组)

手动安装 udev 规则:
  sudo cp rules/99-obsensor-libusb.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger

手动将用户加入 video 组:
  sudo usermod -aG video $USER
  (需重新登录生效, 或 newgrp video 临时生效)
README

echo ""
echo "[6/6] 打包为 tar.gz..."
mkdir -p "${DEFAULT_OUTPUT_DIR}"
OUTPUT_FILE="${DEFAULT_OUTPUT_DIR}/${PKG_NAME}.tar.gz"

tar -czf "${OUTPUT_FILE}" -C "$(dirname "${STAGE_DIR}")" "$(basename "${STAGE_DIR}")"

PKG_SIZE=$(du -sh "${OUTPUT_FILE}" | awk '{print $1}')
STAGE_SIZE=$(du -sh "${STAGE_DIR}" | awk '{print $1}')
BIN_COUNT=$(ls "${STAGE_DIR}/bin/" | wc -l)
LIB_COUNT=$(ls "${STAGE_DIR}/lib/"*.so* 2>/dev/null | wc -l)

rm -rf "${STAGE_DIR}"

echo ""
echo "========================================="
echo "  打包完成!"
echo "========================================="
echo "输出文件:     ${OUTPUT_FILE}"
echo "包大小:       ${PKG_SIZE}"
echo "解压后大小:   ${STAGE_SIZE}"
echo "程序数:       ${BIN_COUNT}"
echo "库文件数:     ${LIB_COUNT}"
echo ""
echo "在目标机器上使用:"
echo "  tar xzf ${PKG_NAME}.tar.gz"
echo "  cd ${PKG_NAME}"
echo "  ./run_nio_multi_capture --help"
echo "========================================="
