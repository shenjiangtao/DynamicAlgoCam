#!/usr/bin/env bash
# package.sh — 一键打包 NIO Orbbec 采集工具，目标机器解压即用
#
# Usage:
#   ./scripts/package.sh                              # 打包所有 nio 程序
#   ./scripts/package.sh nio_multi_capture            # 仅打包指定程序
#   ./scripts/package.sh nio_multi_capture nio_d2c_fusion  # 多个
#   ./scripts/package.sh --all                        # 打包所有 nio 程序（默认）
#   ./scripts/package.sh --output /tmp/my_pkg         # 自定义输出路径
#   ./scripts/package.sh --skip-libs                  # 不打包系统 .so（目标机已有）
#
# 输出: <output_dir>/nio_capture_tools_<arch>_<date>.tar.gz
#        解压后目录结构:
#          nio_capture_tools/
#          ├── bin/                    # 可执行文件
#          ├── lib/                    # libOrbbecSDK.so + extensions + 系统 .so
#          ├── config/                 # OrbbecSDKConfig.xml
#          ├── scripts/                # 辅助脚本 (h264→mp4, colorize, parse_depth_raw 等)
#          ├── docs/                   # use_guide.md
#          ├── run_nio_multi_capture   # 一键运行封装脚本
#          └── README.txt              # 使用说明

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"
BIN_DIR="${BUILD_DIR}/linux_x86_64/bin"
LIB_DIR="${BUILD_DIR}/linux_x86_64/lib"

DEFAULT_OUTPUT_DIR="${PROJECT_ROOT}/dist"
SKIP_SYSTEM_LIBS=false
TARGET_BINARIES=()

for arg in "$@"; do
    case "$arg" in
        --output|-o)
            shift; DEFAULT_OUTPUT_DIR="$1"; shift ;;
        --skip-libs)
            SKIP_SYSTEM_LIBS=true ;;
        --all)
            ;;
        --help|-h)
            sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# //'
            exit 0 ;;
        -*)
            echo "未知选项: $arg" >&2; exit 1 ;;
        *)
            TARGET_BINARIES+=("$arg") ;;
    esac
done

ARCH="$(uname -m)"
DATE_TAG="$(date +%Y%m%d_%H%M%S)"
PKG_NAME="nio_capture_tools_${ARCH}_${DATE_TAG}"
STAGE_DIR="/tmp/${PKG_NAME}"

if [ ${#TARGET_BINARIES[@]} -eq 0 ]; then
    shopt -s nullglob
    for bin_path in "${BIN_DIR}"/nio_*; do
        TARGET_BINARIES+=("$(basename "$bin_path")")
    done
    shopt -u nullglob
fi

if [ ${#TARGET_BINARIES[@]} -eq 0 ]; then
    echo "错误: 未找到任何 nio_* 可执行文件于 ${BIN_DIR}/" >&2
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
mkdir -p "${STAGE_DIR}"/{bin,lib,config,scripts,docs}

echo ""
echo "[1/6] 复制可执行文件..."
for bin_name in "${TARGET_BINARIES[@]}"; do
    cp -v "${BIN_DIR}/${bin_name}" "${STAGE_DIR}/bin/"
    chmod +x "${STAGE_DIR}/bin/${bin_name}"
done

echo ""
echo "[2/6] 复制 libOrbbecSDK.so + extensions..."
cp -v "${LIB_DIR}/libOrbbecSDK.so"        "${STAGE_DIR}/lib/"
cp -v "${LIB_DIR}/libOrbbecSDK.so."*      "${STAGE_DIR}/lib/" 2>/dev/null || true
cp -v "${LIB_DIR}/OrbbecSDKConfig.xml"    "${STAGE_DIR}/config/"

if [ -d "${LIB_DIR}/extensions" ]; then
    cp -rv "${LIB_DIR}/extensions" "${STAGE_DIR}/lib/"
fi

echo ""
echo "[3/6] 收集非系统共享库..."
NON_SYSTEM_LIBS=(
    libOrbbecSDK.so
    libavcodec.so
    libavutil.so
    libswscale.so
    libswresample.so
    libx264.so
    libx265.so
    libvpx.so
    libopus.so
    libvorbis.so
    libvorbisenc.so
    libtheoraenc.so
    libtheoradec.so
    libogg.so
    libaom.so
    libopenjp2.so
    libmp3lame.so
    libspeex.so
    libsnappy.so
    libnuma.so
    libva.so
    libva-drm.so
    libva-x11.so
    libvdpau.so
    libdrm.so
    libX11.so
    libXext.so
    libXfixes.so
    libXrender.so
    libxcb.so
    libxcb-shm.so
    libxcb-render.so
    libXau.so
    libXdmcp.so
    libOpenCL.so
    libsoxr.so
    libz.so
    liblzma.so
    libwebp.so
    libwebpmux.so
    libzvbi.so
    librsvg-2.so
    libcairo.so
    libcairo-gobject.so
    libgdk_pixbuf-2.0.so
    libgio-2.0.so
    libglib-2.0.so
    libgmodule-2.0.so
    libgobject-2.0.so
    libpango-1.0.so
    libpangocairo-1.0.so
    libpangoft2-1.0.so
    libharfbuzz.so
    libfribidi.so
    libfontconfig.so
    libfreetype.so
    libpixman-1.so
    libpng16.so
    libjpeg.so
    libxml2.so
    libicuuc.so
    libicudata.so
    libpcre.so
    libpcre2-8.so
    libffi.so
    libgomp.so
    libbsd.so
    libexpat.so
    libuuid.so
    libblkid.so
    libmount.so
    libselinux.so
    libresolv.so
    libcodec2.so
    libgsm.so
    libshine.so
    libtwolame.so
    libwavpack.so
    libxvidcore.so
)

collect_system_libs() {
    local main_bin="${STAGE_DIR}/bin/${TARGET_BINARIES[0]}"
    local collected=0

    if [ "$SKIP_SYSTEM_LIBS" = true ]; then
        echo "  (跳过系统库收集)"
        return
    fi

    local resolved_libs
    resolved_libs=$(ldd "$main_bin" 2>/dev/null | grep '=>' | grep -v 'not found' | grep -v 'linux-vdso' | grep -v '/lib64/ld-linux' | awk '{print $3}' | sort -u)

    for lib_path in $resolved_libs; do
        local lib_name
        lib_name="$(basename "$lib_path")"
        local base_name="${lib_name%%.so*}"
        local matched=false
        for known in "${NON_SYSTEM_LIBS[@]}"; do
            if [[ "$base_name" == "${known%%.so*}" ]]; then
                matched=true
                break
            fi
        done

        if [ "$matched" = true ]; then
            cp -vn "$lib_path" "${STAGE_DIR}/lib/" 2>/dev/null && {
                cp -vn "$(readlink -f "$lib_path")" "${STAGE_DIR}/lib/" 2>/dev/null || true
                collected=$((collected + 1))
                echo "  + $lib_name"
            }
        fi
    done

    for other_bin in "${TARGET_BINARIES[@]:1}"; do
        local extra_libs
        extra_libs=$(ldd "${STAGE_DIR}/bin/$other_bin" 2>/dev/null | grep '=>' | grep -v 'not found' | grep -v 'linux-vdso' | grep -v '/lib64/ld-linux' | awk '{print $3}' | sort -u)
        for lib_path in $extra_libs; do
            local lib_name
            lib_name="$(basename "$lib_path")"
            if [ ! -f "${STAGE_DIR}/lib/$lib_name" ]; then
                local base_name="${lib_name%%.so*}"
                for known in "${NON_SYSTEM_LIBS[@]}"; do
                    if [[ "$base_name" == "${known%%.so*}" ]]; then
                        cp -vn "$lib_path" "${STAGE_DIR}/lib/" 2>/dev/null && {
                            cp -vn "$(readlink -f "$lib_path")" "${STAGE_DIR}/lib/" 2>/dev/null || true
                            echo "  + $lib_name (from $other_bin)"
                        }
                        break
                    fi
                done
            fi
        done
    done

    echo "  收集了 ${collected} 个非系统共享库"
}

collect_system_libs

echo ""
echo "[4/6] 复制辅助脚本和文档..."

MULTI_CAPTURE_DIR="${PROJECT_ROOT}/examples/6.nio.multi_capture"
HELPER_SCRIPTS=(
    "batch_wrap_color_depth.sh"
    "colorize_from_video.sh"
    "colorize_from_video.py"
    "parse_depth_raw.py"
)

for script in "${HELPER_SCRIPTS[@]}"; do
    if [ -f "${MULTI_CAPTURE_DIR}/${script}" ]; then
        cp -v "${MULTI_CAPTURE_DIR}/${script}" "${STAGE_DIR}/scripts/"
        chmod +x "${STAGE_DIR}/scripts/${script}" 2>/dev/null || true
    fi
done

if [ -f "${MULTI_CAPTURE_DIR}/use_guide.md" ]; then
    cp -v "${MULTI_CAPTURE_DIR}/use_guide.md" "${STAGE_DIR}/docs/"
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
#   ./run_<BIN> --help
#
# 环境变量:
#   NIO_USBFS_MB  — 自动设置 usbfs_memory_mb (默认: 256)
set -euo pipefail

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LD_LIBRARY_PATH="${SELF_DIR}/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

if [ ! -f "${SELF_DIR}/lib/libOrbbecSDK.so" ]; then
    echo "错误: libOrbbecSDK.so 未找到于 ${SELF_DIR}/lib/" >&2
    exit 1
fi

USBFS_MB="${NIO_USBFS_MB:-256}"
CURRENT_USBFS="$(cat /sys/module/usbcore/parameters/usbfs_memory_mb 2>/dev/null || echo 0)"
if [ "${CURRENT_USBFS}" -lt "${USBFS_MB}" ] 2>/dev/null; then
    echo "提示: usbfs_memory_mb=${CURRENT_USBFS}MB, 建议设为 ${USBFS_MB}MB"
    echo "  临时: echo ${USBFS_MB} | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb"
    echo "  永久: sudo modprobe usbcore usbfs_memory_mb=${USBFS_MB}"
    echo "       或在 /etc/modprobe.d/usbcore.conf 中添加: options usbcore usbfs_memory_mb=${USBFS_MB}"
fi

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
   ./run_nio_multi_capture              # 默认录制所有摄像头
   ./run_nio_multi_capture --help       # 查看参数
   ./run_nio_multi_capture -c "305"     # 仅录制 305 型号
   ./run_nio_multi_capture -s /data/cap # 指定保存目录

2. USB 配置（多摄像头必读）
--------------------------
   多台摄像头同时使用时，需要增大 USB 内存限制:
   echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
   永久生效: 在 /etc/modprobe.d/ 创建文件:
     echo "options usbcore usbfs_memory_mb=256" | sudo tee /etc/modprobe.d/usbcore.conf

3. 依赖说明
-----------
   - 本包已自带 libOrbbecSDK.so 及其 extensions
   - 若使用 --skip-libs 打包，则目标机需安装: ffmpeg libx264
   - 系统 .so 已内置于 lib/ 目录，通过 LD_LIBRARY_PATH 加载
   - 需要 Linux x86_64, glibc >= 2.31

4. 辅助脚本
-----------
   scripts/batch_wrap_color_depth.sh  — 批量将 .h264 封装为 .mp4
   scripts/colorize_from_video.sh     — 深度视频伪彩色化
   scripts/parse_depth_raw.py         — 解析 .raw 深度文件

5. 输出文件说明
---------------
   *_color_*.h264     — 彩色流 H264
   *_depth_*.h264     — 深度流 H264 (jet colormap 编码)
   *_depth_raw_*.raw  — 原始深度 Y16 数据
   *_ir_*.h264        — 红外流 H264
   *_imu_*.txt        — IMU 数据 CSV (host_ts_ms,type,device_ts_us,x,y,z,temp)
   *_d2c_fused_*.h264 — D2C 融合 (深度 jet + 彩色 alpha 混合) H264

   H264 文件中每帧前有 SEI NAL 单元, dts= 字段为相机设备时间戳(微秒)

6. 权限
-------
   USB 设备可能需要 udev 规则才能非 root 访问:
   sudo cp /opt/orbbec/99-orbbec-usb.rules /etc/udev/rules.d/ 2>/dev/null
   sudo udevadm control --reload-rules && sudo udevadm trigger
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
