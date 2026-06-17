#!/usr/bin/env python3
# colorize_pipe_ffmpeg.py
# 用法:
#   python3 colorize_pipe_ffmpeg.py input.mp4 output.mp4 --colormap viridis --min 0 --max 4.0 --fps 30

import cv2, numpy as np, argparse, subprocess, shlex, sys, os

parser = argparse.ArgumentParser()
parser.add_argument('input')
parser.add_argument('output')
parser.add_argument('--colormap', default='viridis', choices=['jet','viridis','plasma','inferno','magma','turbo'])
parser.add_argument('--min', type=float, default=None)
parser.add_argument('--max', type=float, default=None)
parser.add_argument('--fps', type=float, default=None)
parser.add_argument('--crf', type=int, default=18)
parser.add_argument('--preset', default='veryfast')
args = parser.parse_args()

cv_colormaps = {
    'jet': cv2.COLORMAP_JET,
    'viridis': cv2.COLORMAP_VIRIDIS,
    'plasma': cv2.COLORMAP_PLASMA,
    'inferno': cv2.COLORMAP_INFERNO,
    'magma': cv2.COLORMAP_MAGMA,
    'turbo': cv2.COLORMAP_TURBO
}

cap = cv2.VideoCapture(args.input)
if not cap.isOpened():
    print("无法打开输入视频:", args.input, file=sys.stderr)
    sys.exit(2)

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = args.fps or cap.get(cv2.CAP_PROP_FPS) or 30.0
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if cap.get(cv2.CAP_PROP_FRAME_COUNT) > 0 else None

print(f"输入: {args.input}  分辨率: {w}x{h}  FPS: {fps}  目标: {args.output}")

ffmpeg_cmd = (
    f"ffmpeg -y -f rawvideo -pix_fmt bgr24 -s {w}x{h} -r {fps} -i - "
    f"-c:v libx264 -preset {args.preset} -crf {args.crf} -pix_fmt yuv420p {shlex.quote(args.output)}"
)
print("启动 ffmpeg:", ffmpeg_cmd)
proc = subprocess.Popen(shlex.split(ffmpeg_cmd), stdin=subprocess.PIPE)

colormap = cv_colormaps.get(args.colormap, cv2.COLORMAP_VIRIDIS)
frame_idx = 0

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 提取灰度（Y 通道）
        if frame.ndim == 3 and frame.shape[2] == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame if frame.ndim == 2 else frame[:,:,0]

        # 归一化
        if args.min is None or args.max is None:
            mn = float(np.min(gray))
            mx = float(np.max(gray))
        else:
            mn = float(args.min)
            mx = float(args.max)

        if mx <= mn:
            mx = mn + 1.0

        norm = np.clip((gray.astype(np.float32) - mn) / (mx - mn), 0.0, 1.0)
        img8 = (norm * 255.0).astype(np.uint8)

        color = cv2.applyColorMap(img8, colormap)  # 返回 BGR
        # 将 BGR 原始字节写入 ffmpeg stdin
        proc.stdin.write(color.tobytes())

        frame_idx += 1
        if frame_idx % 200 == 0:
            print(f"已处理帧: {frame_idx}", flush=True)
finally:
    cap.release()
    if proc.stdin:
        proc.stdin.close()
    proc.wait()

print("完成，处理帧数:", frame_idx)

