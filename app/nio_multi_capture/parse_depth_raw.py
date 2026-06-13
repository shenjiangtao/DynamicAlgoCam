#!/usr/bin/env python3
"""
Orbbec Depth Raw Data (.raw) Parser & Visualizer

Data structure of .raw file:
  Header (44 bytes):
    [0:16]  Magic string: "ORBBEC_DEPTH_RAW"
    [16:20] Width       (uint32_t)
    [20:24] Height      (uint32_t)
    [24:28] BPP         (uint32_t) - bytes per pixel (2 for Y16)
    [28:32] Scale       (float32)  - depth unit scale (e.g. 0.001 = 1mm)
    [32:36] FrameSize   (uint32_t) - bytes per frame (w*h*bpp)
    [36:44] StartTS     (uint64_t) - session start timestamp (ms)
  Frame data (repeating):
    [frameSize bytes each] - raw Y16 depth pixels (uint16_t per pixel)

If no header is found, the file is treated as raw Y16 frames
with dimensions specified via --width and --height.

Usage:
  python3 parse_depth_raw.py <file.raw> [options]

Options:
  --width W       Image width (default: 640, auto-detect from header)
  --height H      Image height (default: 480, auto-detect from header)
  --scale S       Depth scale (default: 0.001)
  --frame N       Frame index to visualize (default: 0)
  --all           Visualize all frames (saves to output directory)
  --output DIR    Output directory for images (default: depth_vis)
  --ascii         Print ASCII depth map to terminal
  --stats         Print depth statistics for each frame
  --histogram     Generate depth histogram plot
  --colormap CM   Matplotlib colormap name (default: jet)
  --min-depth D   Minimum depth for colormap (mm)
  --max-depth D   Maximum depth for colormap (mm)
  --no-header     Force no-header mode (raw Y16 from start)
"""

import sys
import os
import struct
import argparse
import numpy as np

def parse_header(data):
    header = {}
    magic = data[0:16]
    if isinstance(magic, bytes):
        magic_str = magic.split(b'\x00')[0].decode('ascii', errors='replace')
    else:
        magic_str = str(magic)

    if magic_str.startswith('ORBBEC_DEPTH_RAW'):
        header['has_header'] = True
        header['magic'] = magic_str
        header['width'] = struct.unpack_from('<I', data, 16)[0]
        header['height'] = struct.unpack_from('<I', data, 20)[0]
        header['bpp'] = struct.unpack_from('<I', data, 24)[0]
        header['scale'] = struct.unpack_from('<f', data, 28)[0]
        header['frame_size'] = struct.unpack_from('<I', data, 32)[0]
        header['start_ts'] = struct.unpack_from('<Q', data, 36)[0]
        return header
    else:
        header['has_header'] = False
        return header


def parse_raw_file(filepath, width=640, height=480, scale=0.001, force_no_header=False):
    with open(filepath, 'rb') as f:
        data = f.read()

    file_size = len(data)
    header = {}

    if not force_no_header:
        header = parse_header(data)

    if header.get('has_header'):
        w = header['width']
        h = header['height']
        bpp = header['bpp']
        sc = header['scale']
        frame_size = header['frame_size']
        header_size = 44
        print(f"=== Orbbec Depth RAW File Header ===")
        print(f"  Magic:     {header['magic']}")
        print(f"  Width:     {w}")
        print(f"  Height:    {h}")
        print(f"  BPP:       {bpp}")
        print(f"  Scale:     {sc} ({1.0/sc:.1f} units/meter)")
        print(f"  FrameSize: {frame_size} bytes")
        print(f"  StartTS:   {header['start_ts']} ms")
        print(f"  File size: {file_size} bytes")
    else:
        w = width
        h = height
        bpp = 2
        sc = scale
        frame_size = w * h * bpp
        header_size = 0
        print(f"=== No header detected, using defaults ===")
        print(f"  Width:     {w}")
        print(f"  Height:    {h}")
        print(f"  BPP:       {bpp}")
        print(f"  Scale:     {sc}")
        print(f"  FrameSize: {frame_size} bytes")
        print(f"  File size: {file_size} bytes")

    remaining = file_size - header_size
    if frame_size <= 0:
        print("ERROR: Invalid frame size")
        return None, None, None, None

    num_frames = remaining // frame_size
    print(f"  Num frames: {num_frames}")
    print(f"  Remaining bytes after last frame: {remaining % frame_size}")
    print()

    frames = []
    for i in range(num_frames):
        offset = header_size + i * frame_size
        frame_data = data[offset:offset + frame_size]
        depth_arr = np.frombuffer(frame_data, dtype=np.uint16).reshape((h, w))
        frames.append(depth_arr)

    return frames, w, h, sc


def print_ascii_depth(depth, w_out=80, h_out=24, scale=0.001):
    h, w = depth.shape
    depth_mm = depth.astype(np.float32) * scale * 1000.0
    depth_mm[depth == 0] = -1

    chars = " .:-=+*#%@"
    max_depth = 5000.0
    min_depth = 200.0

    step_w = max(1, w // w_out)
    step_h = max(1, h // h_out)

    for y in range(0, h, step_h):
        line = ""
        for x in range(0, w, step_w):
            d = depth_mm[y, x]
            if d < 0:
                line += " "
            else:
                norm = (d - min_depth) / (max_depth - min_depth)
                norm = max(0.0, min(1.0, norm))
                ci = int(norm * (len(chars) - 1))
                line += chars[ci]
        print(line)


def print_stats(depth, frame_idx, scale=0.001):
    depth_mm = depth.astype(np.float64) * scale * 1000.0
    valid = depth_mm[depth > 0]
    print(f"  Frame {frame_idx}:")
    print(f"    Total pixels:  {depth.size}")
    print(f"    Valid pixels:  {len(valid)} ({100.0*len(valid)/depth.size:.1f}%)")
    print(f"    Zero pixels:   {np.sum(depth == 0)} ({100.0*np.sum(depth==0)/depth.size:.1f}%)")
    if len(valid) > 0:
        print(f"    Min depth:     {valid.min():.1f} mm ({valid.min()/1000.0:.3f} m)")
        print(f"    Max depth:     {valid.max():.1f} mm ({valid.max()/1000.0:.3f} m)")
        print(f"    Mean depth:    {valid.mean():.1f} mm ({valid.mean()/1000.0:.3f} m)")
        print(f"    Median depth:  {np.median(valid):.1f} mm")
        print(f"    Std depth:     {valid.std():.1f} mm")
        percentiles = [10, 25, 50, 75, 90, 95, 99]
        vals = np.percentile(valid, percentiles)
        pstr = ", ".join(f"{p}%-ile={v:.0f}mm" for p, v in zip(percentiles, vals))
        print(f"    Percentiles:   {pstr}")
    print()


def save_colorized(depth, filepath, scale=0.001, colormap='jet',
                   min_depth=None, max_depth=None):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("ERROR: matplotlib required for image output. Install: pip3 install matplotlib")
        return

    depth_mm = depth.astype(np.float32) * scale * 1000.0
    depth_vis = depth_mm.copy()
    depth_vis[depth == 0] = np.nan

    if min_depth is None:
        valid = depth_mm[depth > 0]
        min_depth = np.percentile(valid, 2) if len(valid) > 0 else 0
    if max_depth is None:
        valid = depth_mm[depth > 0]
        max_depth = np.percentile(valid, 98) if len(valid) > 0 else 5000

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    ax = axes[0]
    im = ax.imshow(depth_vis, cmap=colormap, vmin=min_depth, vmax=max_depth)
    ax.set_title('Depth Color Map')
    ax.set_xlabel('X (pixels)')
    ax.set_ylabel('Y (pixels)')
    plt.colorbar(im, ax=ax, label='Depth (mm)')

    ax = axes[1]
    valid_mask = depth > 0
    valid_depths = depth_mm[valid_mask]
    if len(valid_depths) > 0:
        ax.hist(valid_depths, bins=100, color='steelblue', edgecolor='none', alpha=0.8)
        ax.axvline(np.mean(valid_depths), color='red', linestyle='--', label=f'Mean={np.mean(valid_depths):.0f}mm')
        ax.axvline(np.median(valid_depths), color='orange', linestyle='--', label=f'Median={np.median(valid_depths):.0f}mm')
        ax.legend(fontsize=8)
    ax.set_title('Depth Histogram')
    ax.set_xlabel('Depth (mm)')
    ax.set_ylabel('Pixel Count')

    ax = axes[2]
    center_y = depth.shape[0] // 2
    cross_section = depth_mm[center_y, :]
    ax.plot(range(depth.shape[1]), cross_section, 'b-', linewidth=0.5)
    ax.set_title(f'Cross-section at y={center_y}')
    ax.set_xlabel('X (pixels)')
    ax.set_ylabel('Depth (mm)')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(filepath, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {filepath}")


def save_histogram(depth, filepath, scale=0.001):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("ERROR: matplotlib required")
        return

    depth_mm = depth.astype(np.float32) * scale * 1000.0
    valid = depth_mm[depth > 0]

    fig, ax = plt.subplots(figsize=(10, 6))
    if len(valid) > 0:
        ax.hist(valid, bins=200, color='steelblue', edgecolor='none', alpha=0.8)
        ax.axvline(np.mean(valid), color='red', linestyle='--',
                   label=f'Mean={np.mean(valid):.0f}mm')
        ax.axvline(np.median(valid), color='orange', linestyle='--',
                   label=f'Median={np.median(valid):.0f}mm')
        ax.legend()
    ax.set_title('Depth Distribution')
    ax.set_xlabel('Depth (mm)')
    ax.set_ylabel('Pixel Count')
    ax.grid(True, alpha=0.3)
    plt.savefig(filepath, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {filepath}")


def main():
    parser = argparse.ArgumentParser(description='Orbbec Depth RAW Parser & Visualizer')
    parser.add_argument('file', help='Path to .raw file')
    parser.add_argument('--width', type=int, default=640, help='Image width')
    parser.add_argument('--height', type=int, default=480, help='Image height')
    parser.add_argument('--scale', type=float, default=0.001, help='Depth scale')
    parser.add_argument('--frame', type=int, default=0, help='Frame index to visualize')
    parser.add_argument('--all', action='store_true', help='Visualize all frames')
    parser.add_argument('--output', default='depth_vis', help='Output directory')
    parser.add_argument('--ascii', action='store_true', help='Print ASCII depth map')
    parser.add_argument('--stats', action='store_true', help='Print depth statistics')
    parser.add_argument('--histogram', action='store_true', help='Generate histogram')
    parser.add_argument('--colormap', default='jet', help='Matplotlib colormap')
    parser.add_argument('--min-depth', type=float, default=None, help='Min depth (mm)')
    parser.add_argument('--max-depth', type=float, default=None, help='Max depth (mm)')
    parser.add_argument('--no-header', action='store_true', help='Force no-header mode')
    args = parser.parse_args()

    if not os.path.exists(args.file):
        print(f"ERROR: File not found: {args.file}")
        sys.exit(1)

    result = parse_raw_file(args.file, args.width, args.height, args.scale, args.no_header)
    frames, w, h, scale = result

    if frames is None or len(frames) == 0:
        print("No frames found!")
        sys.exit(1)

    print(f"Loaded {len(frames)} frame(s) of {w}x{h}, scale={scale}\n")

    if args.ascii:
        idx = min(args.frame, len(frames) - 1)
        print(f"=== ASCII Depth Map (frame {idx}) ===")
        print_ascii_depth(frames[idx], scale=scale)
        print()

    if args.stats:
        print("=== Depth Statistics ===")
        if args.all:
            for i, frame in enumerate(frames):
                print_stats(frame, i, scale)
        else:
            idx = min(args.frame, len(frames) - 1)
            print_stats(frames[idx], idx, scale)

    if args.ascii or args.stats:
        if not args.all and not args.histogram:
            return

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("WARNING: matplotlib not installed, skipping image output")
        print("Install with: pip3 install matplotlib")
        return

    os.makedirs(args.output, exist_ok=True)

    if args.all:
        print(f"=== Generating visualizations for {len(frames)} frames ===")
        for i, frame in enumerate(frames):
            outpath = os.path.join(args.output, f"frame_{i:06d}.png")
            save_colorized(frame, outpath, scale, args.colormap,
                          args.min_depth, args.max_depth)
            if args.histogram:
                hpath = os.path.join(args.output, f"histogram_{i:06d}.png")
                save_histogram(frame, hpath, scale)
    else:
        idx = min(args.frame, len(frames) - 1)
        outpath = os.path.join(args.output, f"frame_{idx:06d}.png")
        print(f"=== Generating visualization for frame {idx} ===")
        save_colorized(frames[idx], outpath, scale, args.colormap,
                      args.min_depth, args.max_depth)
        if args.histogram:
            hpath = os.path.join(args.output, f"histogram_{idx:06d}.png")
            save_histogram(frames[idx], hpath, scale)

    print("\nDone!")


if __name__ == '__main__':
    main()
