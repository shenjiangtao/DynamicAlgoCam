#!/usr/bin/env bash
# colorize_depth.sh
# 用法:
#   ./colorize_depth.sh input.h264 output_color.mp4 [--colormap jet|viridis|plasma|inferno|magma|turbo] [--min MIN] [--max MAX] [--fps FPS]
#
# 示例:
#   ./colorize_depth.sh /HDD/.../Orbbec_Gemini_336L_depth_1780033241752.h264 out.mp4 --colormap viridis --min 0 --max 4.0 --fps 30

set -e

if [ "$#" -lt 2 ]; then
  echo "Usage: $0 input.h264 output.mp4 [--colormap NAME] [--min MIN] [--max MAX] [--fps FPS]"
  exit 1
fi

INPUT_H264="$1"
OUTPUT_MP4="$2"
shift 2

# 默认参数
COLORMAP="viridis"
MIN_VAL=""
MAX_VAL=""
FPS="30"

# 解析可选参数
while [[ $# -gt 0 ]]; do
  case "$1" in
    --colormap)
      COLORMAP="$2"; shift 2;;
    --min)
      MIN_VAL="$2"; shift 2;;
    --max)
      MAX_VAL="$2"; shift 2;;
    --fps)
      FPS="$2"; shift 2;;
    *)
      echo "Unknown option: $1"; exit 1;;
  esac
done

# 检查依赖
command -v ffmpeg >/dev/null 2>&1 || { echo "需要 ffmpeg，请先安装"; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "需要 python3，请先安装"; exit 1; }

# 临时文件
TMP_DIR="$(mktemp -d)"
TMP_MP4="${TMP_DIR}/_tmp_input.mp4"
TMP_PY="${TMP_DIR}/_colorize.py"

echo "封装 H.264 为临时 MP4（生成 PTS）..."
ffmpeg -y -fflags +genpts -r "${FPS}" -i "${INPUT_H264}" -c copy "${TMP_MP4}" >/dev/null 2>&1 || {
  echo "警告: 直接封装失败，尝试转码封装（会重新编码）..."
  ffmpeg -y -r "${FPS}" -i "${INPUT_H264}" -c:v libx264 -preset veryfast -crf 18 "${TMP_MP4}"
}

# 写入 Python 脚本
cat > "${TMP_PY}" <<'PYCODE'
#!/usr/bin/env python3
# _colorize.py
# 逐帧读取视频，将灰度（Y 通道）归一化并应用伪彩色，写入输出视频。
import cv2, numpy as np, sys, argparse

parser = argparse.ArgumentParser()
parser.add_argument('input_mp4')
parser.add_argument('output_mp4')
parser.add_argument('--colormap', default='viridis', choices=['jet','viridis','plasma','inferno','magma','turbo'])
parser.add_argument('--min', type=float, default=None)
parser.add_argument('--max', type=float, default=None)
parser.add_argument('--fps', type=float, default=30.0)
args = parser.parse_args()

cv_colormaps = {
    'jet': cv2.COLORMAP_JET,
    'viridis': cv2.COLORMAP_VIRIDIS,
    'plasma': cv2.COLORMAP_PLASMA,
    'inferno': cv2.COLORMAP_INFERNO,
    'magma': cv2.COLORMAP_MAGMA,
    'turbo': cv2.COLORMAP_TURBO
}

cap = cv2.VideoCapture(args.input_mp4)
if not cap.isOpened():
    print("无法打开输入视频:", args.input_mp4, file=sys.stderr)
    sys.exit(2)

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = args.fps or cap.get(cv2.CAP_PROP_FPS) or 30.0

# 尝试使用常见的四字符编码，回退到 mp4v
fourcc = cv2.VideoWriter_fourcc(*'avc1')
out = cv2.VideoWriter(args.output_mp4, fourcc, fps, (w, h))
if not out.isOpened():
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(args.output_mp4, fourcc, fps, (w, h))
    if not out.isOpened():
        print("无法创建输出视频（VideoWriter），请检查 OpenCV 编译选项", file=sys.stderr)
        sys.exit(3)

global_min = args.min
global_max = args.max
colormap = cv_colormaps.get(args.colormap, cv2.COLORMAP_VIRIDIS)

frame_idx = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 如果是彩色帧，提取 Y 通道（YUV->BGR 时 OpenCV 已给 BGR，直接转灰度）
    if frame.ndim == 3 and frame.shape[2] == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        # 单通道直接使用
        gray = frame if frame.ndim == 2 else frame[:,:,0]

    # 将灰度视为深度可视化值（8-bit）
    # 如果输入是 16-bit 深度（rare for mp4），需要修改读取方式
    if global_min is None or global_max is None:
        mn = float(np.min(gray))
        mx = float(np.max(gray))
    else:
        mn = float(global_min)
        mx = float(global_max)

    if mx <= mn:
        mx = mn + 1.0

    norm = np.clip((gray.astype(np.float32) - mn) / (mx - mn), 0.0, 1.0)
    img8 = (norm * 255.0).astype(np.uint8)

    color = cv2.applyColorMap(img8, colormap)
    out.write(color)

    frame_idx += 1
    if frame_idx % 200 == 0:
        print(f"已处理帧: {frame_idx}", flush=True)

cap.release()
out.release()
print("处理完成，帧数:", frame_idx)
PYCODE

chmod +x "${TMP_PY}"

# 检查 Python 依赖
python3 - <<PYTEST
import sys
try:
    import cv2, numpy
except Exception as e:
    print("缺少 Python 依赖: 请运行 'pip3 install opencv-python numpy' 然后重试", file=sys.stderr)
    sys.exit(1)
print("python deps ok")
PYTEST

echo "开始彩色化处理..."
PY_ARGS=( "${TMP_MP4}" "${OUTPUT_MP4}" --colormap "${COLORMAP}" --fps "${FPS}" )
if [ -n "${MIN_VAL}" ]; then
  PY_ARGS+=( --min "${MIN_VAL}" )
fi
if [ -n "${MAX_VAL}" ]; then
  PY_ARGS+=( --max "${MAX_VAL}" )
fi

python3 "${TMP_PY}" "${PY_ARGS[@]}"

echo "彩色化完成，输出文件: ${OUTPUT_MP4}"

# 清理
rm -rf "${TMP_DIR}"
exit 0

