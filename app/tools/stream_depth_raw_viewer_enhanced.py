#!/usr/bin/env python3
"""
Enhanced stream NIO .raw viewer

Features:
 - Stream frames from .raw without loading whole file
 - Exclude sentinel depth values (default 65535)
 - Color mapping using percentile clipping (default 2-98)
 - Optional voxel downsampling before rendering
 - Debug prints per frame with accurate counts of excluded pixels
 - Auto-suggest / auto-set max_depth_m based on first N frames
 - Spatial median filter (kernel 3 or 5)
 - Temporal median filter (window of N frames)
 - Mode switch near/far (keyboard M or --mode)
 - Controls: Space=play/pause, N=next, P=prev, M=toggle near/far, Q/ESC=quit
"""
import struct, argparse, os, time, collections
import numpy as np

try:
    import open3d as o3d
except Exception:
    raise SystemExit("Open3D is required. Install: pip3 install open3d")

try:
    import cv2
except Exception:
    raise SystemExit("OpenCV is required. Install: pip3 install opencv-python")

def parse_header_from_file(f):
    f.seek(0)
    data = f.read(44)
    if len(data) < 44:
        return {'has_header': False}
    magic = data[0:16].split(b'\x00')[0].decode('ascii', errors='ignore')
    if magic.startswith('NIO_DEPTH_RAW') or magic.startswith('ORBBEC_DEPTH_RAW'):
        width = struct.unpack_from('<I', data, 16)[0]
        height = struct.unpack_from('<I', data, 20)[0]
        bpp = struct.unpack_from('<I', data, 24)[0]
        scale = struct.unpack_from('<f', data, 28)[0]
        frame_size = struct.unpack_from('<I', data, 32)[0]
        start_ts = struct.unpack_from('<Q', data, 36)[0]
        return {'has_header': True, 'width': width, 'height': height,
                'bpp': bpp, 'scale': scale, 'frame_size': frame_size,
                'start_ts': start_ts, 'header_size': 44, 'magic': magic}
    else:
        return {'has_header': False}

def read_frame_at(f, offset, frame_size, w, h):
    f.seek(offset)
    data = f.read(frame_size)
    if len(data) < frame_size:
        return None
    depth = np.frombuffer(data, dtype=np.uint16).reshape((h, w))
    return depth

def apply_spatial_median(depth, ksize):
    if ksize is None or ksize <= 1:
        return depth
    # OpenCV medianBlur expects uint16 -> convert to uint16 image
    # medianBlur supports 3x3,5x5,... kernel sizes
    try:
        depth_u16 = depth.astype(np.uint16)
        filtered = cv2.medianBlur(depth_u16, ksize)
        return filtered
    except Exception:
        return depth

def apply_temporal_median(buffer_list):
    # buffer_list: list of uint16 arrays same shape
    if len(buffer_list) == 0:
        return None
    stacked = np.stack(buffer_list, axis=0)  # (T,H,W)
    med = np.median(stacked, axis=0).astype(np.uint16)
    return med

def depth_to_points(depth, scale, fx, fy, cx, cy, max_depth_m, exclude_sentinel):
    h, w = depth.shape
    z = depth.astype(np.float32) * scale
    if exclude_sentinel is None:
        valid = (depth > 0) & (z <= max_depth_m)
    else:
        valid = (depth > 0) & (depth < exclude_sentinel) & (z <= max_depth_m)
    if not np.any(valid):
        return np.zeros((0,3), dtype=np.float32)
    u = np.arange(w, dtype=np.float32)
    v = np.arange(h, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)
    zv = z[valid]
    xu = (uu[valid] - cx) * zv / fx
    yv = (vv[valid] - cy) * zv / fy
    pts = np.stack([xu, yv, zv], axis=-1)
    return pts

def color_from_depth_percentile(points, percentiles=(2,98), cmap_name='viridis'):
    if points.shape[0] == 0:
        return np.zeros((0,3), dtype=np.float32)
    z = points[:,2]
    lo = np.percentile(z, percentiles[0])
    hi = np.percentile(z, percentiles[1])
    if hi - lo < 1e-6:
        n = np.zeros_like(z)
    else:
        n = np.clip((z - lo) / (hi - lo), 0.0, 1.0)
    try:
        import matplotlib.pyplot as plt
        cm = plt.get_cmap(cmap_name)
        cols = (cm(n)[:, :3]).astype(np.float32)
    except Exception:
        r = np.clip(4*(n-0.75), 0, 1)
        g = np.clip(4*(np.abs(n-0.5)-0.25), 0, 1)
        b = np.clip(4*(0.25-n), 0, 1)
        cols = np.stack([r,g,b], axis=-1).astype(np.float32)
    return cols

def compute_auto_max_depth(path, header, frames_to_sample, exclude_sentinel, scale, percentile):
    file_size = os.path.getsize(path)
    header_size = header['header_size']
    frame_size = header['frame_size']
    num_frames = (file_size - header_size) // frame_size
    frames_to_sample = min(frames_to_sample, max(1, num_frames))
    z_values = []
    with open(path, 'rb') as f:
        for i in range(frames_to_sample):
            offset = header_size + i * frame_size
            depth = read_frame_at(f, offset, frame_size, header['width'], header['height'])
            if depth is None:
                continue
            if exclude_sentinel is None:
                mask = (depth > 0)
            else:
                mask = (depth > 0) & (depth < exclude_sentinel)
            if np.any(mask):
                z = depth.astype(np.float32)[mask] * scale
                z_values.append(z)
    if len(z_values) == 0:
        return None
    all_z = np.concatenate(z_values)
    suggested = float(np.percentile(all_z, percentile))
    return suggested

def run_viewer(path, fx, fy, cx, cy, max_depth_m, downsample, debug,
               exclude_sentinel, voxel_size, percentiles,
               auto_max_depth, auto_frames, auto_percentile, auto_factor,
               median_kernel, temporal_window, mode):
    with open(path, 'rb') as f:
        hdr = parse_header_from_file(f)
        if not hdr['has_header']:
            raise RuntimeError("No NIO_DEPTH_RAW header found. Use a file with NIO_DEPTH_RAW or ORBBEC_DEPTH_RAW header.")
        w = hdr['width']; h = hdr['height']; scale = hdr['scale']
        frame_size = hdr['frame_size']; header_size = hdr['header_size']
        magic = hdr.get('magic', '')
        start_ts = hdr.get('start_ts', 0)

        file_size = os.path.getsize(path)
        num_frames = (file_size - header_size) // frame_size

        if debug:
            print("=== HEADER ===")
            print(f" magic: {magic}")
            print(f" width: {w}, height: {h}, bpp: {hdr['bpp']}")
            print(f" scale: {scale} (depth_value * scale = meters)")
            print(f" frame_size: {frame_size}, start_ts: {start_ts}")
            print(f" file_size: {file_size}, header_size: {header_size}, num_frames: {num_frames}")
            print("================\n")

        # auto max depth suggestion
        if auto_max_depth:
            suggested = compute_auto_max_depth(path, hdr, auto_frames, exclude_sentinel, scale, auto_percentile)
            if suggested is not None:
                suggested_val = suggested * auto_factor
                if debug:
                    print(f"[AutoMaxDepth] suggested {auto_percentile}th percentile = {suggested:.3f} m, after factor {auto_factor} -> {suggested_val:.3f} m")
                if max_depth_m is None or max_depth_m <= 0:
                    max_depth_m = suggested_val
                    if debug:
                        print(f"[AutoMaxDepth] using auto max_depth_m = {max_depth_m:.3f} m")

        # default fallback
        if max_depth_m is None or max_depth_m <= 0:
            max_depth_m = 10.0

        # near/far presets
        presets = {
            'near': {'max_depth_m': 5.0, 'voxel_size': 0.01, 'percentiles': (2,95)},
            'far':  {'max_depth_m': max_depth_m, 'voxel_size': voxel_size if voxel_size else 0.02, 'percentiles': percentiles}
        }
        # apply mode
        if mode == 'near':
            max_depth_m = presets['near']['max_depth_m']
            voxel_size = presets['near']['voxel_size']
            percentiles = presets['near']['percentiles']
        elif mode == 'far':
            max_depth_m = presets['far']['max_depth_m']
            voxel_size = presets['far']['voxel_size']
            # percentiles already set

        # Open3D visualizer setup
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="Depth Stream Viewer (enhanced)", width=1280, height=720)
        pcd = o3d.geometry.PointCloud()
        vis.add_geometry(pcd)
        render_opt = vis.get_render_option()
        render_opt.point_size = 1.0

        # temporal buffer for median
        temporal_buf = collections.deque(maxlen=max(1, temporal_window))

        state = {'paused': False, 'frame_idx': 0, 'running': True, 'mode': mode}
        def toggle_pause(vis): state['paused'] = not state['paused']; print("Paused" if state['paused'] else "Playing"); return False
        def next_frame(vis): state['frame_idx'] = min(state['frame_idx'] + 1, num_frames-1); print("Jump to frame", state['frame_idx']); return False
        def prev_frame(vis): state['frame_idx'] = max(state['frame_idx'] - 1, 0); print("Jump to frame", state['frame_idx']); return False
        def stop(vis): state['running'] = False; return False
        def toggle_mode(vis):
            state['mode'] = 'far' if state['mode'] == 'near' else 'near'
            print("Switched mode to", state['mode'])
            return False

        vis.register_key_callback(ord(" "), lambda vis: toggle_pause(vis))
        vis.register_key_callback(ord("N"), lambda vis: next_frame(vis))
        vis.register_key_callback(ord("P"), lambda vis: prev_frame(vis))
        vis.register_key_callback(ord("Q"), lambda vis: stop(vis))
        vis.register_key_callback(256, lambda vis: stop(vis))  # ESC
        vis.register_key_callback(ord("M"), lambda vis: toggle_mode(vis))

        print("Controls: Space=play/pause  N=next  P=prev  M=toggle near/far  Q/ESC=quit")
        last_time = time.time()
        while state['running']:
            if not state['paused']:
                idx = state['frame_idx']
                offset = header_size + idx * frame_size
                depth = read_frame_at(f, offset, frame_size, w, h)
                if depth is None:
                    print("End of file or read error at frame", idx)
                    break

                # downsample
                if downsample > 1:
                    depth_ds = depth[::downsample, ::downsample]
                    cx_ds = cx / downsample
                    cy_ds = cy / downsample
                    fx_ds = fx / downsample
                    fy_ds = fy / downsample
                else:
                    depth_ds = depth
                    cx_ds, cy_ds, fx_ds, fy_ds = cx, cy, fx, fy

                # spatial median
                if median_kernel is not None and median_kernel > 1:
                    depth_ds = apply_spatial_median(depth_ds, median_kernel)

                # push to temporal buffer and compute temporal median if window>1
                if temporal_window is not None and temporal_window > 1:
                    temporal_buf.append(depth_ds)
                    if len(temporal_buf) == temporal_buf.maxlen:
                        depth_proc = apply_temporal_median(list(temporal_buf))
                    else:
                        # not enough frames yet, use current
                        depth_proc = depth_ds
                else:
                    depth_proc = depth_ds

                # compute masks and counts consistent with rendering
                total = depth_proc.size
                num_zero = int(np.sum(depth_proc == 0))
                num_sentinel = int(np.sum(depth_proc >= exclude_sentinel)) if exclude_sentinel is not None else 0
                z_m = depth_proc.astype(np.float32) * scale
                num_over_max = int(np.sum(z_m > max_depth_m))
                if exclude_sentinel is None:
                    mask_valid_all = (depth_proc > 0)
                else:
                    mask_valid_all = (depth_proc > 0) & (depth_proc < exclude_sentinel)
                num_valid_after_all = int(np.sum(mask_valid_all & (z_m <= max_depth_m)))

                if debug:
                    print(f"[Frame {idx}] total={total}, zero={num_zero}, sentinel={num_sentinel}, over_max={num_over_max}, valid_after_filter={num_valid_after_all}")

                # stats for rendering range (apply same mask)
                mask_for_stats = mask_valid_all & (z_m <= max_depth_m)
                if debug:
                    if np.any(mask_for_stats):
                        zmin = float(np.min(z_m[mask_for_stats]))
                        zmax = float(np.max(z_m[mask_for_stats]))
                        print(f"         z range used for rendering (m): min={zmin:.4f}, max={zmax:.4f}")
                    else:
                        print("         z range used for rendering: no valid pixels after filtering")

                pts = depth_to_points(depth_proc, scale, fx_ds, fy_ds, cx_ds, cy_ds, max_depth_m, exclude_sentinel)
                cols = color_from_depth_percentile(pts, percentiles)

                if debug:
                    print(f"         generated points: {len(pts)}")
                    if len(cols) > 0:
                        print(f"         color min/max per channel: {cols.min(axis=0)}, {cols.max(axis=0)}")
                    else:
                        print("         colors: none")

                # build pcd
                pcd.points = o3d.utility.Vector3dVector(pts)
                if len(cols) > 0:
                    pcd.colors = o3d.utility.Vector3dVector(cols)
                else:
                    pcd.colors = o3d.utility.Vector3dVector(np.zeros((len(pts),3), dtype=np.float32))

                # optional voxel downsample to reduce overplotting
                if voxel_size is not None and voxel_size > 0 and len(pts) > 0:
                    try:
                        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
                    except Exception:
                        pass

                vis.update_geometry(pcd)
                vis.poll_events()
                vis.update_renderer()

                state['frame_idx'] += 1
                if state['frame_idx'] >= num_frames:
                    state['frame_idx'] = 0  # loop

                # frame rate control (~15 FPS)
                now = time.time()
                dt = now - last_time
                target_dt = 1.0 / 15.0
                if dt < target_dt:
                    time.sleep(max(0, target_dt - dt))
                last_time = time.time()
            else:
                vis.poll_events()
                vis.update_renderer()
                time.sleep(0.05)

        vis.destroy_window()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Enhanced NIO .raw viewer')
    parser.add_argument('file', help='Path to .raw file')
    parser.add_argument('--fx', type=float, required=True)
    parser.add_argument('--fy', type=float, required=True)
    parser.add_argument('--cx', type=float, required=True)
    parser.add_argument('--cy', type=float, required=True)
    parser.add_argument('--max-depth-m', type=float, default=None, help='Max depth in meters for filtering (None to use default or auto)')
    parser.add_argument('--downsample', type=int, default=2, help='Spatial downsample factor for speed')
    parser.add_argument('--debug', action='store_true', help='Enable debug prints')
    parser.add_argument('--exclude-sentinel', type=int, default=65535, help='Depth sentinel to exclude (set 0 to disable)')
    parser.add_argument('--voxel-size', type=float, default=0.0, help='Voxel size (m) for downsampling before render; 0 disables')
    parser.add_argument('--color-percentiles', type=str, default='2,98', help='Percentiles for color mapping, e.g. "2,98"')
    parser.add_argument('--auto-max-depth', action='store_true', help='Auto-suggest and use max_depth_m based on first frames')
    parser.add_argument('--auto-frames', type=int, default=5, help='Number of frames to sample for auto max depth')
    parser.add_argument('--auto-percentile', type=float, default=95.0, help='Percentile to use when auto computing max depth')
    parser.add_argument('--auto-factor', type=float, default=1.2, help='Multiplier applied to auto percentile to get final max_depth_m')
    parser.add_argument('--median-kernel', type=int, default=3, help='Spatial median kernel size (3,5). Set 1 to disable')
    parser.add_argument('--temporal-window', type=int, default=1, help='Temporal median window (frames). Set 1 to disable')
    parser.add_argument('--mode', type=str, default='auto', choices=['auto','near','far'], help='Viewer mode: auto/near/far')
    args = parser.parse_args()

    # parse percentiles
    try:
        p0, p1 = [float(x) for x in args.color_percentiles.split(',')]
        percentiles = (max(0.0, min(100.0, p0)), max(0.0, min(100.0, p1)))
    except Exception:
        percentiles = (2.0, 98.0)

    exclude_sentinel = args.exclude_sentinel if args.exclude_sentinel != 0 else None
    voxel_size = args.voxel_size if args.voxel_size > 0 else None
    median_kernel = args.median_kernel if args.median_kernel and args.median_kernel > 1 else None
    temporal_window = args.temporal_window if args.temporal_window and args.temporal_window > 1 else 1

    run_viewer(args.file, args.fx, args.fy, args.cx, args.cy,
               args.max_depth_m, args.downsample, args.debug,
               exclude_sentinel, voxel_size, percentiles,
               args.auto_max_depth, args.auto_frames, args.auto_percentile, args.auto_factor,
               median_kernel, temporal_window, args.mode)

