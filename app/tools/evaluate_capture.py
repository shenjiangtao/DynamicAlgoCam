#!/usr/bin/env python3
"""
NIO Multi-Capture Post-Capture Evaluation Script

Analyzes captured data from Orbbec 335L/336L and RoboSense AC1 devices:
  - Depth raw (.raw): frame count, valid pixel ratio, depth stats, temporal stability
  - IMU (.txt): sample rate, noise (Allan variance approx), bias, temperature
  - Intrinsic (.json): camera/LiDAR parameters
  - PCD point cloud (AC1 only): point count, distance distribution, valid/invalid ratio
  - Capture log: FPS, D2C mode, stream start/stop timing
  - Multi-device temporal alignment: clock drift, frame/IMU timestamp consistency
  - Cross-device depth comparison: distribution differences from mounting positions
  - H.264 video streams: frame count, bitrate, resolution, per-stream quality metrics
  - Color stream: sharpness, entropy, brightness, exposure quality assessment
  - D2C fusion: SSIM/PSNR vs color, 8x8 block artifact analysis, SW vs HW comparison
  - Comparison against published specs with pass/fail criteria

Usage:
  # Auto-discover all devices under a unified session directory:
  python3 evaluate_capture.py /path/to/session_dir

  # Legacy mode: separate directories per device:
  python3 evaluate_capture.py /path/to/335L /path/to/ac1

  # With optional 336L directory:
  python3 evaluate_capture.py /path/to/session_dir --data-dir-336l /path/to/336L

Options:
  --output FILE    Output evaluation document path (default: evaluation_report.md)
  --full           Include per-frame depth stats (verbose)

Dependencies:
  - ffprobe (ffmpeg): required for H.264 stream probe
  - opencv-python (cv2): optional, for frame-level quality/D2C analysis
"""

import sys
import os
import struct
import json
import re
import math
import argparse
import subprocess
import numpy as np
from datetime import datetime, timezone

HEADER_SIZE = 44
NIO_DEPTH_RAW_MAGIC = b"NIO_DEPTH_RAW"
ORBBEC_DEPTH_RAW_MAGIC = b"ORBBEC_DEPTH_RAW"

DEVICE_PATTERNS = {
    "335L": ["335L", "Gemini_335L", "Orbbec_Gemini_335L"],
    "336L": ["336L", "Gemini_336L", "Orbbec_Gemini_336L"],
    "AC1": ["AC1", "RoboSense_AC1", "RS_AC1"],
}


def identify_device(dir_name):
    name = dir_name.replace(" ", "_")
    for dev_type, patterns in DEVICE_PATTERNS.items():
        for p in patterns:
            if p in name:
                safe = name
                return dev_type, safe
    return None, name


def discover_devices(data_root):
    devices = {}
    log_file = None
    for dirpath, dirnames, filenames in os.walk(data_root):
        for fn in filenames:
            if fn.startswith("nio_multi_capture_log") and fn.endswith(".log"):
                log_file = os.path.join(dirpath, fn)
        for dn in dirnames:
            dev_type, safe_name = identify_device(dn)
            if dev_type is None:
                continue
            full = os.path.join(dirpath, dn)
            if dev_type in devices:
                continue
            devices[dev_type] = {"path": full, "safe_name": safe_name}
    return devices, log_file


H264_STREAM_TYPES = ["color", "depth", "d2c_fused", "ir_left", "ir_right"]


def find_files(data_root):
    result = {"raw": None, "imu": None, "intrinsic": None, "log": None, "pcd_dir": None, "h264_files": [], "h264_by_type": {}, "imu_path": None, "pcd_dir_path": None}
    for dirpath, dirnames, filenames in os.walk(data_root):
        for fn in filenames:
            fp = os.path.join(dirpath, fn)
            if fn.endswith("_depth_raw_") and fn.endswith(".raw") or ("_depth_raw_" in fn and fn.endswith(".raw")):
                if result["raw"] is None:
                    result["raw"] = fp
            if "_imu_" in fn and fn.endswith(".txt"):
                if result["imu"] is None:
                    result["imu"] = fp
                    result["imu_path"] = fp
            if "_depth_intrinsic_" in fn and fn.endswith(".json"):
                if result["intrinsic"] is None:
                    result["intrinsic"] = fp
            if fn.startswith("nio_multi_capture_log") and fn.endswith(".log"):
                if result["log"] is None:
                    result["log"] = fp
            if fn.endswith(".h264"):
                result["h264_files"].append(fp)
                for st in H264_STREAM_TYPES:
                    if f"_{st}_" in fn:
                        if st not in result["h264_by_type"]:
                            result["h264_by_type"][st] = fp
                        break
        for dn in dirnames:
            dp = os.path.join(dirpath, dn)
            if "_pcd_" in dn and os.path.isdir(dp):
                if result["pcd_dir"] is None:
                    result["pcd_dir"] = dp
                    result["pcd_dir_path"] = dp

    if result["log"] is None:
        parent = os.path.dirname(data_root.rstrip("/"))
        if parent and os.path.isdir(parent):
            for fn in os.listdir(parent):
                if fn.startswith("nio_multi_capture_log") and fn.endswith(".log"):
                    result["log"] = os.path.join(parent, fn)
                    break
    return result


import mmap

def parse_depth_raw(filepath):
    file_size = os.path.getsize(filepath)
    with open(filepath, "rb") as f:
        with mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ) as mm:
            magic = mm[0:16]
            magic_str = magic.split(b'\x00')[0].decode('ascii', errors='replace') if isinstance(magic, bytes) else str(magic)
            if not (magic_str.startswith('NIO_DEPTH_RAW') or magic_str.startswith('ORBBEC_DEPTH_RAW')):
                return None
            w = struct.unpack_from('<I', mm, 16)[0]
            h = struct.unpack_from('<I', mm, 20)[0]
            bpp = struct.unpack_from('<I', mm, 24)[0]
            scale = struct.unpack_from('<f', mm, 28)[0]
            frame_size = struct.unpack_from('<I', mm, 32)[0]
            start_ts = struct.unpack_from('<Q', mm, 36)[0]
    if frame_size == 0:
        return None
    remaining = file_size - HEADER_SIZE
    num_frames = remaining // frame_size
    frames = []
    with open(filepath, "rb") as f:
        with mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ) as mm:
            for i in range(num_frames):
                offset = HEADER_SIZE + i * frame_size
                depth_arr = np.frombuffer(mm, dtype=np.uint16, count=w * h, offset=offset).reshape((h, w))
                frames.append(depth_arr)
    return {
        "magic": magic_str,
        "width": w,
        "height": h,
        "bpp": bpp,
        "scale": scale,
        "frame_size": frame_size,
        "start_ts": start_ts,
        "file_size": file_size,
        "num_frames": num_frames,
        "frames": frames,
    }


def analyze_depth(depth_info, device_name, max_sample_frames=30):
    if depth_info is None:
        return {}
    frames = depth_info["frames"]
    n = len(frames)
    scale = depth_info["scale"]
    w, h = depth_info["width"], depth_info["height"]
    scale_is_meters = scale < 1.0
    convert = scale * 1000.0 if scale_is_meters else scale

    step = max(1, n // max_sample_frames)
    sampled_indices = list(range(0, n, step))

    all_valid_ratios = np.empty(len(sampled_indices))
    all_means_mm = []
    all_stds_mm = []
    all_mins_mm = []
    all_maxs_mm = []
    per_frame_valid_mm = []

    for si, i in enumerate(sampled_indices):
        depth = frames[i]
        valid_mask = depth > 0
        valid_count = int(np.count_nonzero(valid_mask))
        all_valid_ratios[si] = valid_count / depth.size if depth.size > 0 else 0.0
        if valid_count > 0:
            depth_mm = depth[valid_mask].astype(np.float64) * convert
            all_means_mm.append(float(depth_mm.mean()))
            all_stds_mm.append(float(depth_mm.std()))
            all_mins_mm.append(float(depth_mm.min()))
            all_maxs_mm.append(float(depth_mm.max()))
            per_frame_valid_mm.append(depth_mm)

    result = {
        "device": device_name,
        "resolution": f"{w}x{h}",
        "num_frames": n,
        "depth_scale": scale,
        "file_size_mb": depth_info["file_size"] / (1024 * 1024),
        "bytes_per_frame_kb": depth_info["frame_size"] / 1024,
        "is_lidar": scale >= 1.0,
        "scale_is_meters": scale_is_meters,
        "start_ts_us": depth_info["start_ts"],
    }

    if len(all_valid_ratios) > 0:
        result["avg_valid_ratio"] = float(np.mean(all_valid_ratios))
        result["min_valid_ratio"] = float(np.min(all_valid_ratios))
        result["max_valid_ratio"] = float(np.max(all_valid_ratios))
    if all_means_mm:
        means_arr = np.array(all_means_mm)
        result["avg_mean_depth_mm"] = float(means_arr.mean())
        result["avg_std_depth_mm"] = float(np.mean(all_stds_mm))
        result["min_depth_mm"] = min(all_mins_mm)
        result["max_depth_mm"] = max(all_maxs_mm)
        result["depth_temporal_std_mm"] = float(means_arr.std())

    all_valid_flat = np.concatenate(per_frame_valid_mm) if per_frame_valid_mm else np.array([])
    if len(all_valid_flat) > 0:
        result["global_mean_mm"] = float(np.mean(all_valid_flat))
        result["global_median_mm"] = float(np.median(all_valid_flat))
        result["global_std_mm"] = float(np.std(all_valid_flat))
        result["global_mean_m"] = result["global_mean_mm"] / 1000.0
        result["global_median_m"] = result["global_median_mm"] / 1000.0
        result["global_std_m"] = result["global_std_mm"] / 1000.0
        pcts = [5, 10, 25, 50, 75, 90, 95, 99]
        vals = np.percentile(all_valid_flat, pcts)
        result["percentiles_mm"] = {f"p{p}": float(v) for p, v in zip(pcts, vals)}
        result["percentiles_m"] = {f"p{p}": float(v) / 1000.0 for p, v in zip(pcts, vals)}

    if n >= 2:
        first_mm = frames[0][frames[0] > 0].astype(np.float64) * convert if np.any(frames[0] > 0) else None
        last_mm = frames[-1][frames[-1] > 0].astype(np.float64) * convert if np.any(frames[-1] > 0) else None
        valid_both = (frames[0] > 0) & (frames[-1] > 0)
        if np.any(valid_both):
            first_full = frames[0].astype(np.float64) * convert
            last_full = frames[-1].astype(np.float64) * convert
            diff = np.abs(first_full[valid_both] - last_full[valid_both])
            result["temporal_drift_mm"] = float(np.mean(diff))
            result["temporal_drift_max_mm"] = float(np.max(diff))

    return result


def parse_imu(filepath):
    raw_lines = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            parts = line.split(',')
            if len(parts) < 7:
                continue
            try:
                rec = (int(parts[0]), parts[1].strip(), int(parts[2]),
                       float(parts[3]), float(parts[4]), float(parts[5]),
                       float(parts[6]))
                raw_lines.append(rec)
            except (ValueError, IndexError):
                continue
    records = [{"host_ts_ms": r[0], "type": r[1], "device_ts_us": r[2],
                "x": r[3], "y": r[4], "z": r[5], "temperature": r[6]} for r in raw_lines]
    return records


def analyze_imu(records, device_name):
    result = {"device": device_name}
    if not records:
        result["total_samples"] = 0
        return result

    accel = [r for r in records if r["type"] == "ACCEL"]
    gyro = [r for r in records if r["type"] == "GYRO"]

    result["total_samples"] = len(records)
    result["accel_samples"] = len(accel)
    result["gyro_samples"] = len(gyro)

    if accel:
        ax = np.array([r["x"] for r in accel])
        ay = np.array([r["y"] for r in accel])
        az = np.array([r["z"] for r in accel])
        a_host_ts = np.array([r["host_ts_ms"] for r in accel], dtype=np.int64)
        a_dev_ts = np.array([r["device_ts_us"] for r in accel], dtype=np.int64)
        accel_mag = np.sqrt(ax**2 + ay**2 + az**2)
        ts_first = int(a_host_ts[0])
        ts_last = int(a_host_ts[-1])
        duration_s = (ts_last - ts_first) / 1000.0 if ts_last > ts_first else 0.001
        result["accel_rate_hz"] = len(accel) / duration_s
        result["accel_duration_s"] = duration_s
        result["accel_mean"] = [float(ax.mean()), float(ay.mean()), float(az.mean())]
        result["accel_std"] = [float(ax.std()), float(ay.std()), float(az.std())]
        result["accel_mag_mean"] = float(accel_mag.mean())
        result["accel_mag_std"] = float(accel_mag.std())
        result["accel_host_ts_first"] = ts_first
        result["accel_host_ts_last"] = ts_last
        if len(accel) >= 2:
            intervals = np.diff(a_host_ts)
            result["accel_interval_mean_ms"] = float(np.mean(intervals))
            result["accel_interval_std_ms"] = float(np.std(intervals))
            result["accel_jitter_ms"] = float(np.std(intervals))

    if gyro:
        gx = np.array([r["x"] for r in gyro])
        gy = np.array([r["y"] for r in gyro])
        gz = np.array([r["z"] for r in gyro])
        g_host_ts = np.array([r["host_ts_ms"] for r in gyro], dtype=np.int64)
        ts_first = int(g_host_ts[0])
        ts_last = int(g_host_ts[-1])
        duration_s = (ts_last - ts_first) / 1000.0 if ts_last > ts_first else 0.001
        result["gyro_rate_hz"] = len(gyro) / duration_s
        result["gyro_duration_s"] = duration_s
        result["gyro_mean"] = [float(gx.mean()), float(gy.mean()), float(gz.mean())]
        result["gyro_std"] = [float(gx.std()), float(gy.std()), float(gz.std())]
        result["gyro_noise_rms"] = float(np.sqrt(gx**2 + gy**2 + gz**2).mean())
        result["gyro_host_ts_first"] = ts_first
        result["gyro_host_ts_last"] = ts_last
        if len(gyro) >= 2:
            intervals = np.diff(g_host_ts)
            result["gyro_interval_mean_ms"] = float(np.mean(intervals))
            result["gyro_interval_std_ms"] = float(np.std(intervals))
            result["gyro_jitter_ms"] = float(np.std(intervals))

    temps = [r["temperature"] for r in records if r["temperature"] != 0.0]
    if temps:
        t = np.array(temps)
        result["temp_mean"] = float(t.mean())
        result["temp_min"] = float(t.min())
        result["temp_max"] = float(t.max())
        result["temp_available"] = True
    else:
        result["temp_available"] = False

    if len(records) >= 2:
        host_ts_all = np.array([r["host_ts_ms"] for r in records], dtype=np.int64)
        dev_ts_all = np.array([r["device_ts_us"] for r in records], dtype=np.int64)
        host_durations = np.diff(host_ts_all) / 1000.0
        dev_durations = np.diff(dev_ts_all) / 1e6
        valid = (host_durations > 0) & (dev_durations > 0)
        if np.any(valid):
            ratios = host_durations[valid] / dev_durations[valid]
            result["clock_drift_ppm"] = float((np.mean(ratios) - 1.0) * 1e6)
            result["clock_drift_ratio_mean"] = float(np.mean(ratios))
            result["clock_drift_ratio_std"] = float(np.std(ratios))

    return result


def analyze_temporal_alignment(all_device_data):
    result = {}
    devices = list(all_device_data.keys())
    if len(devices) < 2:
        return result

    imu_data = {}
    depth_data = {}
    for dev in devices:
        d = all_device_data[dev]
        if d.get("imu_stats") and d["imu_stats"].get("total_samples", 0) > 0:
            imu_data[dev] = d["imu_stats"]
        if d.get("depth_stats") and d["depth_stats"].get("num_frames", 0) > 0:
            depth_data[dev] = d["depth_stats"]

    if len(imu_data) >= 2:
        dev_list = list(imu_data.keys())
        ts_starts = {}
        ts_ends = {}
        for dev in dev_list:
            imu = imu_data[dev]
            accel_start = imu.get("accel_host_ts_first")
            accel_end = imu.get("accel_host_ts_last")
            gyro_start = imu.get("gyro_host_ts_first")
            gyro_end = imu.get("gyro_host_ts_last")
            starts = [s for s in [accel_start, gyro_start] if s is not None]
            ends = [e for e in [accel_end, gyro_end] if e is not None]
            if starts:
                ts_starts[dev] = min(starts)
            if ends:
                ts_ends[dev] = max(ends)

        if len(ts_starts) >= 2:
            all_starts = list(ts_starts.values())
            all_ends = list(ts_ends.values())
            result["imu_start_offset_ms"] = float(max(all_starts) - min(all_starts))
            result["imu_end_offset_ms"] = float(max(all_ends) - min(all_ends))
            overlap_start = max(all_starts)
            overlap_end = min(all_ends)
            result["imu_overlap_s"] = float((overlap_end - overlap_start) / 1000.0) if overlap_end > overlap_start else 0.0

            drifts = {}
            for dev in dev_list:
                if "clock_drift_ppm" in imu_data[dev]:
                    drifts[dev] = imu_data[dev]["clock_drift_ppm"]
            if len(drifts) >= 2:
                drift_vals = list(drifts.values())
                result["clock_drift_diff_ppm"] = float(max(drift_vals) - min(drift_vals))
                result["clock_drift_per_device"] = drifts

    if len(depth_data) >= 2:
        dev_list = list(depth_data.keys())
        ts_starts = {}
        for dev in dev_list:
            d = depth_data[dev]
            if "start_ts_us" in d:
                ts_starts[dev] = d["start_ts_us"]

        if len(ts_starts) >= 2:
            all_starts = list(ts_starts.values())
            result["depth_start_offset_us"] = float(max(all_starts) - min(all_starts))
            result["depth_start_offset_ms"] = float((max(all_starts) - min(all_starts)) / 1000.0)

    return result


def analyze_cross_device_depth(all_device_data):
    result = {}
    devices = list(all_device_data.keys())
    if len(devices) < 2:
        return result

    depth_stats = {}
    for dev in devices:
        d = all_device_data[dev].get("depth_stats")
        if d and d.get("num_frames", 0) > 0:
            depth_stats[dev] = d

    if len(depth_stats) < 2:
        return result

    result["devices"] = list(depth_stats.keys())
    result["comparison"] = {}

    dev_pairs = []
    devs = list(depth_stats.keys())
    for i in range(len(devs)):
        for j in range(i + 1, len(devs)):
            dev_pairs.append((devs[i], devs[j]))

    for d1, d2 in dev_pairs:
        s1, s2 = depth_stats[d1], depth_stats[d2]
        comp = {}

        if "global_mean_mm" in s1 and "global_mean_mm" in s2:
            comp["mean_diff_mm"] = float(abs(s1["global_mean_mm"] - s2["global_mean_mm"]))
            comp["mean_diff_m"] = comp["mean_diff_mm"] / 1000.0
            comp["d1_global_mean_mm"] = float(s1["global_mean_mm"])
            comp["d2_global_mean_mm"] = float(s2["global_mean_mm"])

        if "avg_valid_ratio" in s1 and "avg_valid_ratio" in s2:
            comp["valid_ratio_diff"] = float(abs(s1["avg_valid_ratio"] - s2["avg_valid_ratio"]))

        if "resolution" in s1 and "resolution" in s2:
            comp["resolution_diff"] = f"{s1['resolution']} vs {s2['resolution']}"

        if "depth_temporal_std_mm" in s1 and "depth_temporal_std_mm" in s2:
            comp["d1_temporal_std_mm"] = float(s1["depth_temporal_std_mm"])
            comp["d2_temporal_std_mm"] = float(s2["depth_temporal_std_mm"])

        result["comparison"][f"{d1}_vs_{d2}"] = comp

    return result


def analyze_pcd_dir(pcd_dir, device_name, max_files=20):
    result = {"device": device_name}
    if pcd_dir is None or not os.path.isdir(pcd_dir):
        result["available"] = False
        return result

    result["available"] = True
    pcd_files = sorted([f for f in os.listdir(pcd_dir) if f.endswith('.pcd')])
    result["total_pcd_files"] = len(pcd_files)

    if not pcd_files:
        return result

    point_counts = []
    sample_files = pcd_files[:max_files] if len(pcd_files) > max_files else pcd_files

    for pcd_fn in sample_files:
        pcd_path = os.path.join(pcd_dir, pcd_fn)
        info = parse_pcd_header(pcd_path)
        if info and info.get("points", 0) > 0:
            point_counts.append(info["points"])

    if point_counts:
        result["avg_points"] = float(np.mean(point_counts))
        result["min_points"] = int(np.min(point_counts))
        result["max_points"] = int(np.max(point_counts))
        result["std_points"] = float(np.std(point_counts))

    if pcd_files:
        timestamps = []
        for fn in pcd_files:
            m = re.search(r'(\d{13,})', fn)
            if m:
                timestamps.append(int(m.group(1)))
        if len(timestamps) >= 2:
            ts_sorted = sorted(timestamps)
            duration_us = ts_sorted[-1] - ts_sorted[0]
            duration_s = duration_us / 1e6
            if duration_s > 0:
                result["pcd_fps"] = len(pcd_files) / duration_s
                result["pcd_duration_s"] = duration_s

    return result


def _scan_h264_nals(data):
    sep3 = np.frombuffer(b'\x00\x00\x01', dtype=np.uint8)
    sep4 = np.frombuffer(b'\x00\x00\x00\x01', dtype=np.uint8)
    arr = np.frombuffer(data, dtype=np.uint8)
    n = len(arr)
    if n < 4:
        return 0, 0, {}
    matches4 = np.ones(n - 3, dtype=bool)
    for k in range(4):
        matches4 &= (arr[k:n - 3 + k] == sep4[k])
    match4_idx = np.where(matches4)[0]
    encoding = np.zeros(n, dtype=np.uint8)
    for idx in match4_idx:
        encoding[idx] = 4
    sep3_m = np.ones(n - 2, dtype=bool)
    for k in range(3):
        sep3_m &= (arr[k:n - 2 + k] == sep3[k])
    for idx in np.where(sep3_m)[0]:
        if encoding[idx] == 0:
            encoding[idx] = 3
    nal_types = {}
    vcl_count = 0
    i = 0
    while i < n:
        if encoding[i] == 4 and i + 5 <= n:
            nt = data[i + 4] & 0x1F
            nal_types[nt] = nal_types.get(nt, 0) + 1
            if nt in (1, 5):
                vcl_count += 1
            i += 4
        elif encoding[i] == 3 and i + 4 <= n:
            nt = data[i + 3] & 0x1F
            nal_types[nt] = nal_types.get(nt, 0) + 1
            if nt in (1, 5):
                vcl_count += 1
            i += 3
        else:
            i += 1
    return vcl_count, len(match4_idx) + int(np.sum(sep3_m)) - len(match4_idx), nal_types


_h264_probe_cache = {}

def probe_h264_stream(path):
    if path in _h264_probe_cache:
        return _h264_probe_cache[path]
    result = {"path": path, "available": True}
    try:
        import subprocess
        r = subprocess.run(
            ["ffprobe", "-v", "quiet", "-print_format", "json",
             "-show_streams", "-show_format", path],
            capture_output=True, text=True, timeout=10
        )
        if r.returncode != 0:
            return {"path": path, "available": False}
        import json as _json
        d = _json.loads(r.stdout)
        s = d.get("streams", [{}])[0]
        fmt = d.get("format", {})
        result["width"] = s.get("width")
        result["height"] = s.get("height")
        rfr = s.get("r_frame_rate", "")
        if "/" in str(rfr):
            num, den = rfr.split("/")
            result["r_frame_rate"] = float(num) / float(den) if float(den) > 0 else 0
        else:
            result["r_frame_rate"] = float(rfr) if rfr else 0
        result["profile"] = s.get("profile", "")
        result["level"] = s.get("level")
        result["pix_fmt"] = s.get("pix_fmt", "")
        result["color_range"] = s.get("color_range", "")
        result["color_space"] = s.get("color_space", "")
        result["color_primaries"] = s.get("color_primaries", "")
        result["color_transfer"] = s.get("color_transfer", "")
        result["refs"] = s.get("refs")
        result["has_b_frames"] = s.get("has_b_frames")
        result["bit_rate"] = int(fmt.get("bit_rate", 0))
        result["duration_s"] = float(fmt.get("duration", 0))
        result["format_size"] = int(fmt.get("size", 0))
    except Exception:
        pass

    result["file_size"] = os.path.getsize(path) if os.path.exists(path) else 0
    with open(path, 'rb') as f:
        h264_data = f.read()
    vcl_count, _, nal_types = _scan_h264_nals(h264_data)
    result["nal_frame_count"] = vcl_count

    if vcl_count > 0 and result["file_size"] > 0:
        result["avg_frame_bytes"] = result["file_size"] / vcl_count
        result["avg_bitrate_kbps"] = result["avg_frame_bytes"] * result.get("r_frame_rate", 30) * 8 / 1000
    result["_nal_types"] = nal_types
    _h264_probe_cache[path] = result
    return result


def analyze_h264_streams(h264_by_type, device_name, device_type):
    result = {"device": device_name, "streams": {}, "available": bool(h264_by_type)}
    if not h264_by_type:
        return result

    for stream_type, path in h264_by_type.items():
        if not path or not os.path.exists(path):
            continue
        info = probe_h264_stream(path)
        result["streams"][stream_type] = info

    return result


NAL_TYPE_NAMES = {1: "non-IDR", 5: "IDR", 6: "SEI", 7: "SPS", 8: "PPS", 9: "AUD"}


def _nal_distribution_from_probe(probe):
    nal_types = probe.get("_nal_types", {})
    idr_count = nal_types.get(5, 0)
    non_idr_count = nal_types.get(1, 0)
    if idr_count > 0 and non_idr_count > 0:
        avg_gop_size = (non_idr_count / idr_count) + 1
    elif idr_count > 0:
        avg_gop_size = 1
    else:
        avg_gop_size = 0
    return {
        "nal_types": nal_types,
        "idr_count": idr_count,
        "non_idr_count": non_idr_count,
        "avg_gop_size": round(avg_gop_size, 1),
    }


def analyze_h264_encoding(h264_by_type, device_name):
    result = {"device": device_name, "streams": {}}
    for stream_type, path in h264_by_type.items():
        if not path or not os.path.exists(path):
            continue
        probe = probe_h264_stream(path)
        nal = _nal_distribution_from_probe(probe)
        diag = {
            "profile": probe.get("profile", ""),
            "level": probe.get("level"),
            "pix_fmt": probe.get("pix_fmt", ""),
            "color_range": probe.get("color_range", ""),
            "color_space": probe.get("color_space", ""),
            "color_primaries": probe.get("color_primaries", ""),
            "color_transfer": probe.get("color_transfer", ""),
            "refs": probe.get("refs"),
            "has_b_frames": probe.get("has_b_frames"),
            "resolution": f"{probe.get('width', '?')}x{probe.get('height', '?')}",
            "nal_frame_count": probe.get("nal_frame_count", 0),
            "file_size_kb": round(probe.get("file_size", 0) / 1024),
            "avg_bitrate_kbps": round(probe.get("avg_bitrate_kbps", 0)),
            "avg_frame_bytes": round(probe.get("avg_frame_bytes", 0)),
            "nal_types": nal["nal_types"],
            "idr_count": nal["idr_count"],
            "non_idr_count": nal["non_idr_count"],
            "avg_gop_size": nal["avg_gop_size"],
        }
        result["streams"][stream_type] = diag
    return result


def assess_h264_encoding(enc_info, device_type):
    checks = []
    for stream_type, diag in enc_info.get("streams", {}).items():
        if stream_type not in ("color", "d2c_fused"):
            continue
        prefix = f"H.264 {stream_type}"
        profile = diag.get("profile", "")
        if profile:
            checks.append((f"{prefix} profile", profile, "Constrained Baseline+",
                           profile in ("Constrained Baseline", "Baseline", "Main", "High")))
        level = diag.get("level")
        if level is not None:
            level_ok = level >= 30
            checks.append((f"{prefix} level", str(level), ">=30", level_ok))
        pix_fmt = diag.get("pix_fmt", "")
        if pix_fmt:
            checks.append((f"{prefix} pix_fmt", pix_fmt, "yuvj420p/yuv420p",
                           pix_fmt in ("yuvj420p", "yuv420p")))
        idr = diag.get("idr_count", 0)
        checks.append((f"{prefix} IDR keyframes", str(idr), ">0", idr > 0))
        gop = diag.get("avg_gop_size", 0)
        if gop > 0:
            checks.append((f"{prefix} avg GOP size", f"{gop:.1f}", "1-120", 1 <= gop <= 120))
        bframes = diag.get("has_b_frames")
        if bframes is not None:
            checks.append((f"{prefix} B-frames", str(bframes), "0 (realtime encode)", bframes == 0))
    return checks


def extract_h264_sample_frames(h264_path, frame_indices, tmp_dir):
    import subprocess
    frames = {}
    if not h264_path or not os.path.exists(h264_path):
        return frames
    os.makedirs(tmp_dir, exist_ok=True)
    probe = probe_h264_stream(h264_path)
    fps = probe.get("r_frame_rate", 30) or 30
    total = probe.get("nal_frame_count", 0)
    for idx in frame_indices:
        if total > 0 and idx >= total:
            continue
        out_path = os.path.join(tmp_dir, f"frame_{idx}.png")
        if os.path.exists(out_path) and os.path.getsize(out_path) > 0:
            try:
                import cv2
                img = cv2.imread(out_path)
                if img is not None:
                    frames[idx] = img
                    continue
            except ImportError:
                pass
        r = subprocess.run(
            ["ffmpeg", "-y", "-fflags", "+genpts", "-r", str(int(fps)),
             "-i", h264_path, "-vf", f"select=eq(n\\,{idx})", "-frames:v", "1",
             "-q:v", "2", out_path],
            capture_output=True, text=True, timeout=15
        )
        if os.path.exists(out_path) and os.path.getsize(out_path) > 0:
            try:
                import cv2
                img = cv2.imread(out_path)
                if img is not None:
                    frames[idx] = img
            except ImportError:
                pass
    return frames


def compute_image_quality(img):
    import cv2
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()
    hist = np.histogram(gray.flatten(), bins=256, range=(0, 256))[0].astype(float)
    hist = hist / hist.sum()
    hist = hist[hist > 0]
    entropy = -np.sum(hist * np.log2(hist))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hue_std = float(hsv[:, :, 0].std())
    sat_mean = float(hsv[:, :, 1].mean())
    val_mean = float(hsv[:, :, 2].mean())
    nonzero_ratio = float(np.count_nonzero(gray)) / gray.size
    return {
        "sharpness": round(sharpness, 2),
        "entropy": round(entropy, 2),
        "hue_std": round(hue_std, 1),
        "sat_mean": round(sat_mean, 1),
        "val_mean": round(val_mean, 1),
        "brightness_mean": round(float(gray.mean()), 1),
        "brightness_std": round(float(gray.std()), 1),
        "nonzero_ratio": round(nonzero_ratio, 4),
        "width": w,
        "height": h,
    }


def compute_blockiness(gray):
    h, w = gray.shape
    boundary_scores = []
    block_vars = []
    for y in range(0, h - 8, 8):
        for x in range(0, w - 8, 8):
            block = gray[y:y+8, x:x+8].astype(float)
            block_vars.append(float(block.var()))
            if x > 0 and x + 8 < w:
                left = gray[y:y+8, x-1].astype(float)
                right = gray[y:y+8, x].astype(float)
                boundary_scores.append(float(np.mean(np.abs(left - right))))
            if y > 0 and y + 8 < h:
                top = gray[y-1, x:x+8].astype(float)
                bot = gray[y, x:x+8].astype(float)
                boundary_scores.append(float(np.mean(np.abs(top - bot))))
    if not boundary_scores:
        return {}
    arr = np.array(boundary_scores)
    bv_arr = np.array(block_vars)
    return {
        "boundary_mean": round(float(arr.mean()), 2),
        "boundary_p95": round(float(np.percentile(arr, 95)), 2),
        "boundary_max": round(float(arr.max()), 0),
        "block_var_mean": round(float(bv_arr.mean()), 1),
        "block_var_median": round(float(np.median(bv_arr)), 1),
        "very_smooth_block_pct": round(100.0 * np.sum(bv_arr < 5) / len(bv_arr), 1) if len(bv_arr) > 0 else 0,
    }


def compute_ssim_psnr(img1, img2):
    import cv2
    if img1.shape[:2] != img2.shape[:2]:
        img2 = cv2.resize(img2, (img1.shape[1], img1.shape[0]))
    mse = np.mean((img1.astype(float) - img2.astype(float)) ** 2)
    psnr = 10 * np.log10(255**2 / mse) if mse > 0 else float('inf')
    C1, C2 = (0.01 * 255) ** 2, (0.03 * 255) ** 2
    ssim_per_ch = []
    for ch in range(3):
        c1 = img1[:, :, ch].astype(np.float64)
        c2 = img2[:, :, ch].astype(np.float64)
        mu1 = cv2.GaussianBlur(c1, (11, 11), 1.5)
        mu2 = cv2.GaussianBlur(c2, (11, 11), 1.5)
        s1 = np.sqrt(np.maximum(cv2.GaussianBlur(c1**2, (11, 11), 1.5) - mu1**2, 0))
        s2 = np.sqrt(np.maximum(cv2.GaussianBlur(c2**2, (11, 11), 1.5) - mu2**2, 0))
        s12 = cv2.GaussianBlur(c1 * c2, (11, 11), 1.5) - mu1 * mu2
        ssim_map = ((2 * mu1 * mu2 + C1) * (2 * s12 + C2)) / ((mu1**2 + mu2**2 + C1) * (s1**2 + s2**2 + C2))
        ssim_per_ch.append(float(ssim_map.mean()))
    return {
        "psnr_db": round(psnr, 2),
        "ssim": round(float(np.mean(ssim_per_ch)), 4),
        "ssim_per_channel": [round(s, 4) for s in ssim_per_ch],
    }


def analyze_color_sensor_quality(frames, color_source="unknown"):
    import cv2
    result = {"color_source": color_source}
    sorted_indices = sorted(frames.keys())
    n = len(sorted_indices)
    if n == 0:
        result["error"] = "no frames"
        return result

    first_frame = frames[sorted_indices[0]]
    gray_first = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)
    h, w = gray_first.shape

    if color_source.upper() in ("MJPG", "MJPEG") or "335" in color_source:
        result["source_type"] = "MJPEG"
        result["noise_attribution"] = "MJPEG解压块效应 + yuvj422p→yuv420p sws转换伪影，非原始传感器噪声"
    elif color_source.upper() in ("NV12", "RAW") or "AC1" in color_source.upper():
        result["source_type"] = "NV12"
        result["noise_attribution"] = "原始传感器噪声（NV12直出），无MJPEG中间解压"
    else:
        result["source_type"] = "unknown"
        result["noise_attribution"] = "无法确定噪声来源格式"

    ssim_adjacent = []
    ssim_spread = []
    psnr_adjacent = []
    psnr_spread = []
    for i in range(1, n):
        idx_prev = sorted_indices[i - 1]
        idx_cur = sorted_indices[i]
        if idx_prev in frames and idx_cur in frames:
            sp = compute_ssim_psnr(frames[idx_prev], frames[idx_cur])
            if idx_cur - idx_prev <= 2:
                ssim_adjacent.append(sp["ssim"])
                psnr_adjacent.append(sp["psnr_db"])
            else:
                ssim_spread.append(sp["ssim"])
                psnr_spread.append(sp["psnr_db"])
    all_ssim = ssim_adjacent + ssim_spread
    all_psnr = psnr_adjacent + psnr_spread
    if ssim_adjacent:
        result["inter_frame_ssim_adjacent_mean"] = round(float(np.mean(ssim_adjacent)), 4)
        result["inter_frame_ssim_adjacent_min"] = round(float(np.min(ssim_adjacent)), 4)
    if ssim_spread:
        result["inter_frame_ssim_spread_mean"] = round(float(np.mean(ssim_spread)), 4)
        result["inter_frame_ssim_spread_min"] = round(float(np.min(ssim_spread)), 4)
    if all_ssim:
        result["inter_frame_ssim_mean"] = round(float(np.mean(all_ssim)), 4)
        result["inter_frame_ssim_min"] = round(float(np.min(all_ssim)), 4)
        result["inter_frame_ssim_std"] = round(float(np.std(all_ssim)), 4)
    if all_psnr:
        result["inter_frame_psnr_mean"] = round(float(np.mean(all_psnr)), 2)
        result["inter_frame_psnr_min"] = round(float(np.min(all_psnr)), 2)

    lap = cv2.Laplacian(gray_first, cv2.CV_64F)
    smooth_mask = cv2.GaussianBlur(gray_first, (31, 31), 5)
    smooth_std = cv2.Laplacian(smooth_mask, cv2.CV_64F).var()
    result["laplacian_var_full"] = round(float(lap.var()), 2)
    result["laplacian_var_smooth_region"] = round(float(smooth_std), 4)

    kernel = np.array([[-1, -1, -1], [-1, 8, -1], [-1, -1, -1]])
    high_freq = cv2.filter2D(gray_first.astype(np.float32), -1, kernel.astype(np.float32))
    signal_power = float(np.mean(gray_first.astype(float) ** 2))
    noise_power = float(np.mean(high_freq ** 2))
    result["noise_laplacian_std"] = round(float(lap.std()), 2)
    result["snr_proxy_db"] = round(10 * np.log10(signal_power / noise_power), 2) if noise_power > 0 else 0

    dark_mask = gray_first < 30
    dark_pixel_count = int(np.sum(dark_mask))
    dark_pixel_pct = round(100.0 * dark_pixel_count / gray_first.size, 1)
    result["dark_pixel_pct"] = dark_pixel_pct
    if dark_pixel_count > 100:
        dark_region = gray_first[dark_mask].astype(float)
        result["dark_region_noise_std"] = round(float(dark_region.std()), 2)
        result["dark_region_mean"] = round(float(dark_region.mean()), 2)
    else:
        result["dark_region_noise_std"] = 0
        result["dark_region_mean"] = 0

    hsv = cv2.cvtColor(first_frame, cv2.COLOR_BGR2HSV)
    sat_channel = hsv[:, :, 1].astype(float)
    result["chroma_snr_db"] = round(10 * np.log10(sat_channel.mean() ** 2 / (sat_channel.std() ** 2 + 1e-10)), 2)

    hist = np.histogram(gray_first.flatten(), bins=256, range=(0, 256))[0].astype(float)
    total_px = gray_first.size
    clip_lo_pct = round(100.0 * hist[0] / total_px, 1)
    clip_hi_pct = round(100.0 * hist[255] / total_px, 1)
    result["hist_clip_lo_pct"] = clip_lo_pct
    result["hist_clip_hi_pct"] = clip_hi_pct

    grays = {}
    for idx in sorted_indices:
        g = gray_first if idx == sorted_indices[0] else cv2.cvtColor(frames[idx], cv2.COLOR_BGR2GRAY)
        grays[idx] = g
        sharpness_values.append(float(cv2.Laplacian(g, cv2.CV_64F).var()))
    result["sharpness_mean"] = round(float(np.mean(sharpness_values)), 2)
    result["sharpness_std"] = round(float(np.std(sharpness_values)), 2) if len(sharpness_values) > 1 else 0
    result["sharpness_temporal_stability"] = round(float(np.std(sharpness_values)) / (float(np.mean(sharpness_values)) + 1e-10), 4) if sharpness_values else 0

    gray_blur = cv2.GaussianBlur(gray_first.astype(float), (5, 5), 0)
    edge_resp = np.abs(gray_first.astype(float) - gray_blur)
    result["edge_preservation"] = round(float(np.mean(edge_resp)), 2)

    return result


def analyze_d2c_fusion(h264_by_type, device_name, device_type, log_info):
    result = {"device": device_name, "d2c_mode": log_info.get("d2c_mode", "unknown"), "alpha": log_info.get("alpha")}
    color_path = h264_by_type.get("color")
    fused_path = h264_by_type.get("d2c_fused")
    depth_path = h264_by_type.get("depth")

    if not fused_path or not os.path.exists(fused_path):
        result["fused_available"] = False
        return result
    result["fused_available"] = True

    color_probe = probe_h264_stream(color_path) if color_path and os.path.exists(color_path) else {}
    fused_probe = probe_h264_stream(fused_path)
    depth_probe = probe_h264_stream(depth_path) if depth_path and os.path.exists(depth_path) else {}

    result["fused_resolution"] = f"{fused_probe.get('width', '?')}x{fused_probe.get('height', '?')}"
    result["fused_frame_count"] = fused_probe.get("nal_frame_count", 0)
    result["fused_avg_bitrate_kbps"] = fused_probe.get("avg_bitrate_kbps", 0)
    result["fused_file_size_kb"] = fused_probe.get("file_size", 0) / 1024

    try:
        import cv2
        tmp_dir = os.path.join("/tmp", f"nio_eval_d2c_{device_name}")
        color_frames = extract_h264_sample_frames(color_path, [0, 50], tmp_dir + "_color") if color_path else {}
        fused_frames = extract_h264_sample_frames(fused_path, [0, 50], tmp_dir + "_fused")

        if fused_frames:
            sample_idx = list(fused_frames.keys())[0]
            fused_img = fused_frames[sample_idx]
            fused_gray = cv2.cvtColor(fused_img, cv2.COLOR_BGR2GRAY)
            fused_quality = compute_image_quality(fused_img)
            fused_block = compute_blockiness(fused_gray)
            result["fused_quality"] = fused_quality
            result["fused_blockiness"] = fused_block

        if color_frames and fused_frames:
            sample_idx = min(list(color_frames.keys())[0], list(fused_frames.keys())[0])
            if sample_idx in color_frames and sample_idx in fused_frames:
                sim = compute_ssim_psnr(color_frames[sample_idx], fused_frames[sample_idx])
                result["color_vs_fused_ssim"] = sim["ssim"]
                result["color_vs_fused_psnr_db"] = sim["psnr_db"]
                result["color_vs_fused_detail"] = sim

    except ImportError:
        result["image_quality_note"] = "cv2 not available, skipping frame-level analysis"

    d2c_mode = log_info.get("d2c_mode", "unknown")
    result["d2c_mode"] = d2c_mode
    result["fusion_method_note"] = (
        "SW D2C: depth 1280x800→color 1280x720 software reprojection"
        if d2c_mode == "SW" or device_type in ("335L", "336L")
        else "HW D2C: depth 96x288→color 1920x1080 hardware upsampling (block artifacts expected)"
        if d2c_mode == "HW" or device_type == "AC1"
        else f"D2C mode={d2c_mode}"
    )

    color_frame_count = color_probe.get("nal_frame_count", 0)
    fused_frame_count = fused_probe.get("nal_frame_count", 0)
    result["color_to_fused_frame_ratio"] = round(color_frame_count / fused_frame_count, 2) if fused_frame_count > 0 else 0
    result["color_frame_count"] = color_frame_count
    result["depth_frame_count"] = depth_probe.get("nal_frame_count", 0)

    return result


def analyze_color_streams(h264_by_type, device_name, device_type, depth_stats):
    result = {"device": device_name}
    color_path = h264_by_type.get("color")
    if not color_path or not os.path.exists(color_path):
        result["color_available"] = False
        return result
    result["color_available"] = True

    color_probe = probe_h264_stream(color_path)
    result["color_resolution"] = f"{color_probe.get('width', '?')}x{color_probe.get('height', '?')}"
    result["color_frame_count"] = color_probe.get("nal_frame_count", 0)
    result["color_avg_bitrate_kbps"] = color_probe.get("avg_bitrate_kbps", 0)
    result["color_file_size_kb"] = color_probe.get("file_size", 0) / 1024
    result["color_fps"] = color_probe.get("r_frame_rate", 0)

    try:
        import cv2
        tmp_dir = os.path.join("/tmp", f"nio_eval_color_{device_name}")
        total_frames = color_probe.get("nal_frame_count", 0)
        if total_frames > 10:
            sensor_indices = [0]
            for base in [10, 50, 100, 200, 500, 1000]:
                if base + 1 < total_frames:
                    sensor_indices.extend([base, base + 1])
            if total_frames > 20:
                step = total_frames // 10
                for k in range(1, 10):
                    idx = step * k
                    if idx < total_frames and idx not in sensor_indices:
                        sensor_indices.append(idx)
            sensor_indices = sorted(set(sensor_indices))
        else:
            sensor_indices = [0, min(1, total_frames - 1)] if total_frames > 1 else [0]
        frames = extract_h264_sample_frames(color_path, sensor_indices, tmp_dir)
        if frames:
            sample_idx = list(frames.keys())[0]
            quality = compute_image_quality(frames[sample_idx])
            result["color_quality"] = quality

            if quality["brightness_mean"] < 15 and quality["nonzero_ratio"] < 0.5:
                result["color_issue"] = "SEVERE_UNDEREXPOSURE"
                result["color_issue_note"] = f"Color frame near-black (mean={quality['brightness_mean']:.1f}, nonzero={quality['nonzero_ratio']*100:.1f}%), possible HW exposure/trigger issue"
            elif quality["brightness_mean"] < 30:
                result["color_issue"] = "DARK"
                result["color_issue_note"] = f"Color frame dark (mean={quality['brightness_mean']:.1f}), verify lighting/exposure settings"

            if len(frames) >= 2:
                color_source = device_type if device_type in ("335L", "336L", "AC1") else "unknown"
                result["sensor_quality"] = analyze_color_sensor_quality(frames, color_source)
    except ImportError:
        result["image_quality_note"] = "cv2 not available"

    depth_raw_frames = depth_stats.get("num_frames", 0) if depth_stats else 0
    if result["color_frame_count"] > 0 and depth_raw_frames > 0:
        result["color_depth_frame_ratio"] = round(result["color_frame_count"] / depth_raw_frames, 2)
    result["depth_raw_frames"] = depth_raw_frames

    ir_paths = []
    for st in ["ir_left", "ir_right"]:
        p = h264_by_type.get(st)
        if p and os.path.exists(p):
            ir_paths.append((st, p))
    result["ir_streams_available"] = [st for st, _ in ir_paths]
    for st, p in ir_paths:
        probe = probe_h264_stream(p)
        result[f"{st}_frame_count"] = probe.get("nal_frame_count", 0)
        result[f"{st}_resolution"] = f"{probe.get('width', '?')}x{probe.get('height', '?')}"

    return result


def assess_color(color_info, device_type):
    checks = []
    if not color_info.get("color_available", False):
        checks.append(("Color stream", "NOT AVAILABLE", "PRESENT", False))
        return checks
    checks.append(("Color stream present", "YES", "YES", True))
    q = color_info.get("color_quality", {})
    if q:
        if device_type == "335L":
            checks.append(("Color resolution", color_info.get("color_resolution", "?"), "1280x720", color_info.get("color_resolution") == "1280x720"))
        elif device_type == "AC1":
            checks.append(("Color resolution", color_info.get("color_resolution", "?"), "1920x1080", color_info.get("color_resolution") == "1920x1080"))
        if color_info.get("color_issue") == "SEVERE_UNDEREXPOSURE":
            checks.append(("Color brightness", f"mean={q.get('brightness_mean',0):.1f}", ">15", False))
        elif q.get("brightness_mean", 0) > 5:
            checks.append(("Color brightness", f"mean={q.get('brightness_mean',0):.1f}", ">5", True))
    checks.append(("Color frame count", str(color_info.get("color_frame_count", 0)), ">0", color_info.get("color_frame_count", 0) > 0))
    return checks


def assess_d2c_fusion(d2c_info):
    checks = []
    if not d2c_info.get("fused_available", False):
        checks.append(("D2C Fusion stream", "NOT AVAILABLE", "PRESENT", False))
        return checks
    checks.append(("D2C Fusion present", "YES", "YES", True))
    if d2c_info.get("d2c_mode"):
        checks.append(("D2C mode", d2c_info["d2c_mode"], "SW or HW", d2c_info["d2c_mode"] in ("SW", "HW")))
    if d2c_info.get("fused_frame_count", 0) > 0:
        checks.append(("D2C Fusion frame count", str(d2c_info["fused_frame_count"]), ">0", True))
    q = d2c_info.get("fused_quality", {})
    if q:
        checks.append(("Fused image sharpness", f"{q.get('sharpness', 0):.1f}", ">0", q.get("sharpness", 0) > 0))
        checks.append(("Fused image entropy", f"{q.get('entropy', 0):.2f}", ">2.0", q.get("entropy", 0) > 2.0))
    blk = d2c_info.get("fused_blockiness", {})
    if blk:
        checks.append(("8x8 block boundary score", f"p95={blk.get('boundary_p95', 0):.1f}", "<50 (H.264 normal)", blk.get("boundary_p95", 999) < 50))
    return checks


def assess_color_sensor(sensor_info, device_type):
    checks = []
    sq = sensor_info.get("sensor_quality", {})
    if not sq:
        return checks
    src = sq.get("source_type", "unknown")
    checks.append(("Color source format", src, "MJPEG or NV12", src in ("MJPEG", "NV12")))

    if "inter_frame_ssim_adjacent_mean" in sq:
        ssim_m = sq["inter_frame_ssim_adjacent_mean"]
        if src == "MJPEG":
            threshold = 0.84
        else:
            threshold = 0.92
        checks.append(("Inter-frame SSIM (adjacent)", f"{ssim_m:.4f}", f">={threshold} (source={src})", ssim_m >= threshold))

    if "inter_frame_ssim_adjacent_min" in sq:
        ssim_min = sq["inter_frame_ssim_adjacent_min"]
        lo = 0.75 if src == "MJPEG" else 0.85
        checks.append(("Inter-frame SSIM min (adjacent)", f"{ssim_min:.4f}", f">={lo}", ssim_min >= lo))

    if "inter_frame_ssim_spread_mean" in sq:
        ssim_sp = sq["inter_frame_ssim_spread_mean"]
        lo_sp = 0.60 if src == "MJPEG" else 0.70
        checks.append(("Inter-frame SSIM (spread)", f"{ssim_sp:.4f}", f">={lo_sp} (source={src})", ssim_sp >= lo_sp))

    if "noise_laplacian_std" in sq:
        nl = sq["noise_laplacian_std"]
        if src == "MJPEG":
            hi = 100
        else:
            hi = 60
        checks.append(("Noise level (Laplacian std)", f"{nl:.1f}", f"<{hi} (source={src})", nl < hi))

    if "dark_region_noise_std" in sq and sq["dark_region_noise_std"] > 0:
        dr = sq["dark_region_noise_std"]
        checks.append(("Dark region noise", f"{dr:.2f}", "<20.0", dr < 20.0))

    if "chroma_snr_db" in sq:
        csnr = sq["chroma_snr_db"]
        lo_db = -5.0
        checks.append(("Chroma SNR", f"{csnr:.2f} dB", f">={lo_db} dB", csnr >= lo_db))

    if "hist_clip_hi_pct" in sq:
        clip_hi = sq["hist_clip_hi_pct"]
        checks.append(("Highlight clipping", f"{clip_hi:.1f}%", "<5.0%", clip_hi < 5.0))

    if "hist_clip_lo_pct" in sq:
        clip_lo = sq["hist_clip_lo_pct"]
        checks.append(("Shadow clipping", f"{clip_lo:.1f}%", "<30.0%", clip_lo < 30.0))

    if "sharpness_temporal_stability" in sq:
        ts = sq["sharpness_temporal_stability"]
        checks.append(("Temporal sharpness stability (CV)", f"{ts:.4f}", "<0.5", ts < 0.5))

    return checks


def parse_pcd_header(filepath):
    try:
        with open(filepath, 'rb') as f:
            header_lines = []
            while True:
                line = f.readline()
                if not line:
                    break
                decoded = line.decode('ascii', errors='replace').strip()
                header_lines.append(decoded)
                if decoded.startswith('DATA'):
                    break
            points = 0
            fields = ""
            for hl in header_lines:
                if hl.startswith('POINTS'):
                    points = int(hl.split()[1])
                if hl.startswith('FIELDS'):
                    fields = hl.split(None, 1)[1] if len(hl.split()) > 1 else ""
            return {"points": points, "fields": fields}
    except Exception:
        return None


def parse_capture_log(log_path):
    result = {}
    if not log_path or not os.path.exists(log_path):
        return result

    with open(log_path, 'r') as f:
        content = f.read()

    start_match = re.search(r'Process started.*saveDir=(\S+)', content)
    if start_match:
        result["save_dir"] = start_match.group(1)

    filter_match = re.search(r'Camera filter\[(\d+)\]=(\S+)', content)
    if filter_match:
        result["camera_filter"] = filter_match.group(2)

    git_match = re.search(r'Git commit: (\S+)', content)
    if git_match:
        result["git_commit"] = git_match.group(1)

    fusion_match = re.search(r'D2C Fusion enabled:\s+(\S+).*?mode=(\w+)', content)
    if fusion_match:
        result["d2c_resolution"] = fusion_match.group(1)
        result["d2c_mode"] = fusion_match.group(2)

    alpha_match = re.search(r'alpha=([\d.]+)', content)
    if alpha_match:
        result["alpha"] = float(alpha_match.group(1))

    depth_range_match = re.search(r'depthRange=([\d.]+)m-([\d.]+)m', content)
    if depth_range_match:
        result["depth_min_m"] = float(depth_range_match.group(1))
        result["depth_max_m"] = float(depth_range_match.group(2))

    color_fmt_match = re.search(r'Color output:.*?fmt=(\S+)', content)
    if color_fmt_match:
        result["color_format"] = color_fmt_match.group(1)

    timestamps = []
    for m in re.finditer(r'^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)', content, re.MULTILINE):
        ts_str = m.group(1)
        try:
            dt = datetime.strptime(ts_str.split('.')[0], "%Y-%m-%d %H:%M:%S")
            us = int(ts_str.split('.')[1]) if '.' in ts_str else 0
            ts_s = dt.timestamp() + us / 1e6
            timestamps.append(ts_s)
        except ValueError:
            pass

    if len(timestamps) >= 2:
        result["session_duration_s"] = timestamps[-1] - timestamps[0]

    consumer_start = re.search(r'video consumer started', content)
    consumer_stop = re.search(r'video consumer stopped', content)
    if consumer_start and consumer_stop:
        start_idx = content.index(consumer_start.group())
        stop_idx = content.index(consumer_stop.group())
        start_ts_m = re.search(r'^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)', content[:start_idx], re.MULTILINE)
        stop_ts_m = re.search(r'^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)', content[:stop_idx], re.MULTILINE)
        if start_ts_m and stop_ts_m:
            try:
                s1 = datetime.strptime(start_ts_m.group(1).split('.')[0], "%Y-%m-%d %H:%M:%S").timestamp()
                s2 = datetime.strptime(stop_ts_m.group(1).split('.')[0], "%Y-%m-%d %H:%M:%S").timestamp()
                result["capture_active_s"] = s2 - s1
            except ValueError:
                pass

    return result


def load_intrinsic(filepath):
    if not filepath or not os.path.exists(filepath):
        return None
    with open(filepath, 'r') as f:
        return json.load(f)


def assess_depth(d, device_type):
    checks = []
    if device_type == "335L":
        checks.append(("Depth resolution", d.get("resolution", ""), "1280x800", d.get("resolution") == "1280x800"))
        checks.append(("Depth scale", f"{d.get('depth_scale', 0):.4f} (m/unit)", "~0.001", d.get("depth_scale") is not None and abs(d.get("depth_scale", 0) - 0.001) < 0.0001))
        checks.append(("Valid pixel ratio", f"{d.get('avg_valid_ratio', 0)*100:.1f}%", ">20%", d.get("avg_valid_ratio", 0) > 0.2))
        checks.append(("Temporal stability (std of mean)", f"{d.get('depth_temporal_std_mm', 999):.1f} mm", "<200 mm", d.get("depth_temporal_std_mm", 999) < 200))
    elif device_type == "336L":
        checks.append(("Depth resolution", d.get("resolution", ""), "1280x800", d.get("resolution") == "1280x800"))
        checks.append(("Depth scale", f"{d.get('depth_scale', 0):.4f} (m/unit)", "~0.001", d.get("depth_scale") is not None and abs(d.get("depth_scale", 0) - 0.001) < 0.0001))
        checks.append(("Valid pixel ratio", f"{d.get('avg_valid_ratio', 0)*100:.1f}%", ">20%", d.get("avg_valid_ratio", 0) > 0.2))
        checks.append(("Temporal stability (std of mean)", f"{d.get('depth_temporal_std_mm', 999):.1f} mm", "<200 mm", d.get("depth_temporal_std_mm", 999) < 200))
    elif device_type == "AC1":
        checks.append(("Depth resolution", d.get("resolution", ""), "96x288 (LiDAR grid)", d.get("resolution") in ["96x288", "288x96"]))
        checks.append(("Depth scale", str(d.get("depth_scale")), "5.0", d.get("depth_scale") is not None and abs(d.get("depth_scale", 0) - 5.0) < 0.1))
        checks.append(("Valid pixel ratio", f"{d.get('avg_valid_ratio', 0)*100:.1f}%", ">10%", d.get("avg_valid_ratio", 0) > 0.1))
    return checks


def assess_imu(imu, device_type):
    checks = []
    if imu.get("total_samples", 0) == 0:
        checks.append(("IMU available", "NO", "YES", False))
        return checks
    checks.append(("IMU available", "YES", "YES", True))
    if device_type in ("335L", "336L"):
        checks.append(("Accelerometer rate", f"{imu.get('accel_rate_hz', 0):.0f} Hz", "~200 Hz", imu.get("accel_rate_hz", 0) > 100))
        checks.append(("Gyroscope rate", f"{imu.get('gyro_rate_hz', 0):.0f} Hz", "~200 Hz", imu.get("gyro_rate_hz", 0) > 100))
        checks.append(("Gyro noise (RMS)", f"{imu.get('gyro_noise_rms', 999):.4f} rad/s", "<0.02 rad/s", imu.get("gyro_noise_rms", 999) < 0.02))
        checks.append(("Temperature available", "YES" if imu.get("temp_available") else "NO", "YES", imu.get("temp_available", False)))
    elif device_type == "AC1":
        checks.append(("Accelerometer rate", f"{imu.get('accel_rate_hz', 0):.0f} Hz", "~100 Hz", imu.get("accel_rate_hz", 0) > 50))
        checks.append(("Gyroscope rate", f"{imu.get('gyro_rate_hz', 0):.0f} Hz", "~100 Hz", imu.get("gyro_rate_hz", 0) > 50))
        checks.append(("Temperature available", "YES" if imu.get("temp_available") else "NO (driver=0.0)", "Expected NO", not imu.get("temp_available", False)))
        checks.append(("Gyro noise (RMS)", f"{imu.get('gyro_noise_rms', 999):.4f} rad/s", "<0.05 rad/s", imu.get("gyro_noise_rms", 999) < 0.05))
    return checks


def assess_pcd(pcd_info):
    checks = []
    if not pcd_info.get("available", False):
        checks.append(("PCD point cloud", "N/A", "N/A (335L only)", True))
        return checks
    checks.append(("PCD files present", str(pcd_info.get("total_pcd_files", 0)), ">0", pcd_info.get("total_pcd_files", 0) > 0))
    if pcd_info.get("pcd_fps"):
        checks.append(("PCD frame rate", f"{pcd_info['pcd_fps']:.1f} fps", "~10 fps", pcd_info["pcd_fps"] > 5))
    if pcd_info.get("avg_points"):
        checks.append(("Points per frame", f"{pcd_info['avg_points']:.0f}", ">0", pcd_info["avg_points"] > 0))
    return checks


def assess_temporal_alignment(ta):
    checks = []
    if not ta:
        return checks
    if "imu_start_offset_ms" in ta:
        checks.append(("IMU start offset", f"{ta['imu_start_offset_ms']:.1f} ms", "<1000 ms", ta["imu_start_offset_ms"] < 1000))
    if "imu_overlap_s" in ta:
        checks.append(("IMU time overlap", f"{ta['imu_overlap_s']:.1f} s", ">5 s", ta["imu_overlap_s"] > 5))
    if "depth_start_offset_ms" in ta:
        checks.append(("Depth start offset", f"{ta['depth_start_offset_ms']:.1f} ms", "<1000 ms", ta["depth_start_offset_ms"] < 1000))
    if "clock_drift_diff_ppm" in ta:
        checks.append(("Clock drift diff", f"{ta['clock_drift_diff_ppm']:.1f} ppm", "<500 ppm", ta["clock_drift_diff_ppm"] < 500))
    return checks


def _fmt_val(v, fmt=""):
    if v is None:
        return "N/A"
    if isinstance(v, float):
        if fmt:
            return fmt.format(v)
        return f"{v:.2f}"
    return str(v)


def generate_report(devices_data, alignment, cross_depth, output_path):
    now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    dev_types = sorted(devices_data.keys())
    dev_labels = {"335L": "Orbbec Gemini 335L", "336L": "Orbbec Gemini 336L", "AC1": "RoboSense AC1"}

    lines = []
    lines.append("# NIO Multi-Capture 采集数据评估报告")
    lines.append("")
    lines.append(f"> 生成时间：{now}")
    data_dirs = [f"{dt}=`{devices_data[dt]['path']}`" for dt in dev_types]
    lines.append(f"> 测试数据目录：{', '.join(data_dirs)}")
    if any(devices_data[dt].get("log_info") for dt in dev_types):
        git_commits = []
        for dt in dev_types:
            li = devices_data[dt].get("log_info", {})
            gc = li.get("git_commit", "N/A")
            git_commits.append(f"{dt}=`{gc}`")
        lines.append(f"> Git commit: {', '.join(git_commits)}")
    lines.append("")

    lines.append("## 目录")
    lines.append("")
    lines.append("1. 采集会话概览")
    lines.append("2. 深度数据 (Depth Raw)")
    lines.append("3. IMU 数据分析")
    lines.append("4. 点云 (PCD) 分析")
    lines.append("5. 内参 (Intrinsic) 分析")
    lines.append("6. 数据量对比")
    lines.append("7. 多设备时间同步对齐分析")
    lines.append("8. 跨设备深度差异分析")
    lines.append("9. 通过/不通过判定")
    lines.append("10. 336L 评估 (待补充)")
    lines.append("11. 不同距离精度评估 (待补充)")
    lines.append("12. 关键发现与建议")
    lines.append("13. 附录：原始数据路径")
    lines.append("14. 彩色与红外视频流分析")
    lines.append("15. D2C 深度-彩色融合分析")
    lines.append("16. 跨设备融合方法对比")
    lines.append("")

    lines.append("## 1. 采集会话概览")
    lines.append("")
    header = "| 项目 |" + "|".join([f" {dev_labels.get(dt, dt)} " for dt in dev_types]) + "|"
    sep = "|------|" + "|".join(["------" for _ in dev_types]) + "|"
    lines.append(header)
    lines.append(sep)
    log_keys = ["git_commit", "d2c_mode", "d2c_resolution", "alpha", "depth_min_m", "depth_max_m", "color_format", "capture_active_s"]
    log_labels = ["Git commit", "D2C 模式", "D2C 分辨率", "Alpha 融合系数", "深度范围 min", "深度范围 max", "彩色格式", "采集活跃时间"]
    for lk, ll in zip(log_keys, log_labels):
        vals = []
        for dt in dev_types:
            li = devices_data[dt].get("log_info", {})
            v = li.get(lk, "N/A")
            if lk == "capture_active_s" and isinstance(v, (int, float)):
                vals.append(f"{v:.1f}s")
            elif lk in ("depth_min_m", "depth_max_m") and v != "N/A":
                vals.append(f"{v}m")
            else:
                vals.append(str(v))
        row = f"| {ll} |" + "|".join([f" {v} " for v in vals]) + "|"
        lines.append(row)
    lines.append("")

    lines.append("## 2. 深度数据 (Depth Raw)")
    lines.append("")
    depth_metrics = [
        ("分辨率", "resolution", None),
        ("帧数", "num_frames", None),
        ("depthScale", "depth_scale", None),
        ("文件大小 (MB)", "file_size_mb", ".1f"),
        ("每帧大小 (KB)", "bytes_per_frame_kb", ".0f"),
        ("有效像素比", "avg_valid_ratio", None),
        ("全局均值 (mm)", "global_mean_mm", ".1f"),
        ("全局均值 (m)", "global_mean_m", ".3f"),
        ("全局中位数 (mm)", "global_median_mm", ".1f"),
        ("全局标准差 (mm)", "global_std_mm", ".1f"),
        ("时间稳定性-跨帧std (mm)", "depth_temporal_std_mm", ".1f"),
        ("有效像素比-最小", "min_valid_ratio", None),
        ("有效像素比-最大", "max_valid_ratio", None),
    ]

    has_depth = any(devices_data[dt].get("depth_stats") for dt in dev_types)
    if has_depth:
        header = "| 指标 |" + "|".join([f" {dev_labels.get(dt, dt)} " for dt in dev_types]) + "| 说明 |"
        sep = "|------|" + "|".join(["------" for _ in dev_types]) + "------|"
        lines.append(header)
        lines.append(sep)

        for label, key, fmt in depth_metrics:
            vals = []
            for dt in dev_types:
                ds = devices_data[dt].get("depth_stats", {})
                v = ds.get(key)
                if v is None:
                    vals.append("N/A")
                elif key in ("avg_valid_ratio", "min_valid_ratio", "max_valid_ratio"):
                    vals.append(f"{v*100:.1f}%")
                elif fmt:
                    vals.append(format(v, fmt))
                elif key == "depth_scale":
                    if ds.get("scale_is_meters"):
                        vals.append(f"{v:.4f} (m/unit)")
                    else:
                        vals.append(f"{v:.1f} (mm/unit)")
                else:
                    vals.append(str(v))
            notes = {
                "resolution": "335L=结构光像素级深度, AC1=LiDAR点云网格",
                "depth_scale": "335L/336L=m/unit, AC1=mm/unit",
                "avg_valid_ratio": "0值=无效深度",
                "depth_temporal_std_mm": "<100mm=稳定",
            }
            note = notes.get(key, "")
            row = f"| {label} |" + "|".join([f" {v} " for v in vals]) + f"| {note} |"
            lines.append(row)
        lines.append("")

    for dt in dev_types:
        ds = devices_data[dt].get("depth_stats", {})
        if ds and "percentiles_mm" in ds:
            lines.append(f"### 2.{dev_types.index(dt)+1} {dev_labels.get(dt, dt)} 深度分布百分位")
            lines.append("")
            lines.append("| 百分位 | 深度 (mm) | 深度 (m) |")
            lines.append("|--------|----------|----------|")
            for k, v in ds["percentiles_mm"].items():
                lines.append(f"| {k[1:]}% | {v:.0f} | {v/1000:.2f} |")
            lines.append("")

    lines.append("## 3. IMU 数据分析")
    lines.append("")
    has_imu = any(devices_data[dt].get("imu_stats", {}).get("total_samples", 0) > 0 for dt in dev_types)
    if has_imu:
        header = "| 指标 |" + "|".join([f" {dev_labels.get(dt, dt)} " for dt in dev_types]) + "|"
        sep = "|------|" + "|".join(["------" for _ in dev_types]) + "|"
        lines.append(header)
        lines.append(sep)

        imu_rows = [
            ("总采样数", "total_samples", None),
            ("加速度计采样数", "accel_samples", None),
            ("陀螺仪采样数", "gyro_samples", None),
            ("加速度计频率 (Hz)", "accel_rate_hz", ".0f"),
            ("陀螺仪频率 (Hz)", "gyro_rate_hz", ".0f"),
            ("Gyro 噪声 RMS (rad/s)", "gyro_noise_rms", ".5f"),
            ("温度", None, None),
            ("Accel 间隔抖动 std (ms)", "accel_jitter_ms", ".2f"),
            ("Gyro 间隔抖动 std (ms)", "gyro_jitter_ms", ".2f"),
        ]
        for label, key, fmt in imu_rows:
            vals = []
            for dt in dev_types:
                imu = devices_data[dt].get("imu_stats", {})
                if key == "total_samples" and imu.get("total_samples", 0) == 0:
                    vals.append("N/A")
                elif label == "温度":
                    if imu.get("temp_available"):
                        vals.append(f"{imu.get('temp_mean', 0):.1f}°C")
                    elif dt == "AC1":
                        vals.append("0.0 (driver)")
                    else:
                        vals.append("N/A")
                elif key and imu.get(key) is not None:
                    v = imu[key]
                    if isinstance(v, list):
                        vals.append(f"[{v[0]:.4f}, {v[1]:.4f}, {v[2]:.4f}]")
                    elif fmt:
                        vals.append(format(v, fmt))
                    else:
                        vals.append(str(v))
                else:
                    vals.append("N/A")
            row = f"| {label} |" + "|".join([f" {v} " for v in vals]) + "|"
            lines.append(row)
        lines.append("")

        accel_mean_present = any(isinstance(devices_data[dt].get("imu_stats", {}).get("accel_mean"), list) for dt in dev_types)
        if accel_mean_present:
            lines.append("### 3.1 加速度计/陀螺仪详细统计")
            lines.append("")
            lines.append("| 参数 |" + "|".join([f" {dev_labels.get(dt, dt)} Accel | {dev_labels.get(dt, dt)} Gyro " for dt in dev_types]) + "|")
            lines.append("|------|" + "|".join(["------|------" for _ in dev_types]) + "|")
            for stat_key, stat_label in [("mean", "均值"), ("std", "标准差")]:
                row = f"| {stat_label} |"
                for dt in dev_types:
                    imu = devices_data[dt].get("imu_stats", {})
                    am = imu.get(f"accel_{stat_key}", ["-", "-", "-"])
                    gm = imu.get(f"gyro_{stat_key}", ["-", "-", "-"])
                    if isinstance(am, list):
                        row += f" [{am[0]:.4f},{am[1]:.4f},{am[2]:.4f}] |"
                    else:
                        row += " - |"
                    if isinstance(gm, list):
                        row += f" [{gm[0]:.5f},{gm[1]:.5f},{gm[2]:.5f}] |"
                    else:
                        row += " - |"
                lines.append(row)
            lines.append("")
    else:
        lines.append("_无IMU数据_")
        lines.append("")

    pcd_devs = [dt for dt in dev_types if devices_data[dt].get("pcd_stats", {}).get("available")]
    lines.append("## 4. 点云 (PCD) 分析")
    lines.append("")
    if pcd_devs:
        for dt in pcd_devs:
            pcd = devices_data[dt].get("pcd_stats", {})
            lines.append(f"### {dev_labels.get(dt, dt)}")
            lines.append("")
            lines.append("| 指标 | 值 |")
            lines.append("|------|-----|")
            lines.append(f"| PCD 文件数 | {pcd.get('total_pcd_files', 0)} |")
            if pcd.get("avg_points"):
                lines.append(f"| 平均点数/帧 | {pcd['avg_points']:.0f} |")
                lines.append(f"| 最小点数 | {pcd.get('min_points', 'N/A')} |")
                lines.append(f"| 最大点数 | {pcd.get('max_points', 'N/A')} |")
                lines.append(f"| 点数标准差 | {pcd.get('std_points', 0):.0f} |")
            if pcd.get("pcd_fps"):
                lines.append(f"| PCD 帧率 | {pcd['pcd_fps']:.1f} fps |")
                lines.append(f"| PCD 持续时间 | {pcd.get('pcd_duration_s', 0):.1f} s |")
            lines.append("")
    else:
        lines.append("_无PCD数据（335L/336L 不输出独立 PCD 文件，点云通过 depth_raw 反投影生成）_")
        lines.append("")

    lines.append("## 5. 内参 (Intrinsic) 分析")
    lines.append("")
    for dt in dev_types:
        intr = devices_data[dt].get("intrinsic")
        if not intr:
            continue
        lines.append(f"### 5.{dev_types.index(dt)+1} {dev_labels.get(dt, dt)}")
        lines.append("")
        d_intr = intr.get("depth", {})
        c_intr = intr.get("color", {})
        lidar = intr.get("lidar", {})
        if dt in ("335L", "336L"):
            lines.append("| 参数 | Depth | Color |")
            lines.append("|------|-------|-------|")
            lines.append(f"| 分辨率 | {d_intr.get('width','?')}x{d_intr.get('height','?')} | {c_intr.get('width','?')}x{c_intr.get('height','?')} |")
            for p in ["fx", "fy", "cx", "cy"]:
                dv = f"{d_intr.get(p, 'N/A'):.3f}" if isinstance(d_intr.get(p), (int, float)) else d_intr.get(p, 'N/A')
                cv = f"{c_intr.get(p, 'N/A'):.3f}" if isinstance(c_intr.get(p), (int, float)) else c_intr.get(p, 'N/A')
                lines.append(f"| {p} | {dv} | {cv} |")
            lines.append(f"| depth_scale | {intr.get('depth_scale', 'N/A')} | - |")
        elif dt == "AC1":
            lines.append("| 参数 | 值 | 备注 |")
            lines.append("|------|-----|------|")
            lines.append(f"| depth fx/fy | {d_intr.get('fx','N/A')}/{d_intr.get('fy','N/A')} | 无传统内参(LiDAR) |")
            lines.append(f"| color 分辨率 | {c_intr.get('width','?')}x{c_intr.get('height','?')} | |")
            lines.append(f"| depth_scale | {intr.get('depth_scale','N/A')} | 5.0 mm/unit |")
            if lidar:
                for lk, ll in [("type", "LiDAR 类型"), ("point_grid_width", "点云网格宽"), ("point_grid_height", "点云网格高"),
                                ("distance_min_m", "测距最小"), ("distance_max_m", "测距最大"), ("distance_resolution_m", "测距分辨率"),
                                ("vector_base", "vector_base"), ("point_fields", "点字段")]:
                    if lk in lidar:
                        lines.append(f"| {ll} | {lidar[lk]} | |")
        lines.append("")

    lines.append("## 6. 数据量对比")
    lines.append("")
    header = "| 数据类型 |" + "|".join([f" {dev_labels.get(dt, dt)} " for dt in dev_types]) + "|"
    sep = "|----------|" + "|".join(["------" for _ in dev_types]) + "|"
    if len(dev_types) >= 2:
        header += " 对比 |"
        sep += "------|"
    lines.append(header)
    lines.append(sep)

    raw_sizes = {}
    for dt in dev_types:
        ds = devices_data[dt].get("depth_stats", {})
        raw_sizes[dt] = ds.get("file_size_mb", 0)
    if any(raw_sizes.values()):
        row = "| depth_raw |"
        for dt in dev_types:
            row += f" {raw_sizes[dt]:.1f} MB |"
        if len(dev_types) >= 2:
            vals = list(raw_sizes.values())
            non_zero = [v for v in vals if v > 0]
            if len(non_zero) >= 2:
                ratio = max(non_zero) / min(non_zero)
                row += f" {ratio:.0f}x |"
            else:
                row += " - |"
        lines.append(row)

    imu_sizes = {}
    for dt in dev_types:
        imu_path = devices_data[dt].get("_files", {}).get("imu_path")
        imu_sizes[dt] = os.path.getsize(imu_path) / (1024*1024) if imu_path and os.path.exists(imu_path) else 0
    row = "| IMU txt |"
    for dt in dev_types:
        row += f" {imu_sizes[dt]:.3f} MB |"
    if len(dev_types) >= 2:
        non_zero = [v for v in imu_sizes.values() if v > 0]
        if len(non_zero) >= 2:
            row += f" {max(non_zero)/min(non_zero):.1f}x |"
        else:
            row += " - |"
    lines.append(row)

    pcd_devs_with_data = [dt for dt in dev_types if devices_data[dt].get("pcd_stats", {}).get("available") and devices_data[dt]["pcd_stats"].get("total_pcd_files", 0) > 0]
    if pcd_devs_with_data:
        row = "| PCD 点云 |"
        for dt in dev_types:
            if dt in pcd_devs_with_data:
                pcd_dir = devices_data[dt].get("_files", {}).get("pcd_dir_path")
                pcd_size = _dir_size_mb(pcd_dir) if pcd_dir else 0
                row += f" {pcd_size:.1f} MB |"
            else:
                row += " N/A |"
        if len(dev_types) >= 2:
            row += " - |"
        lines.append(row)

    h264_types_for_table = ["color", "depth", "d2c_fused", "ir_left", "ir_right"]
    h264_labels = {"color": "Color H.264", "depth": "Depth H.264", "d2c_fused": "D2C Fused H.264", "ir_left": "IR-Left H.264", "ir_right": "IR-Right H.264"}
    for ht in h264_types_for_table:
        sizes = {}
        for dt in dev_types:
            h264 = devices_data[dt].get("h264_stats", {}).get("streams", {}).get(ht, {})
            sizes[dt] = h264.get("file_size", 0) / (1024 * 1024) if h264.get("file_size") else 0
        if any(sizes.values()):
            row = f"| {h264_labels.get(ht, ht)} |"
            for dt in dev_types:
                row += f" {sizes[dt]:.2f} MB |" if sizes[dt] > 0 else " N/A |"
            if len(dev_types) >= 2:
                non_zero = [v for v in sizes.values() if v > 0]
                if len(non_zero) >= 2:
                    row += f" {max(non_zero)/min(non_zero):.1f}x |"
                else:
                    row += " - |"
            lines.append(row)
    lines.append("")

    lines.append("## 7. 多设备时间同步对齐分析")
    lines.append("")
    if alignment:
        lines.append("本节分析同时采集时多设备间的时间同步质量。")
        lines.append("")
        if "imu_start_offset_ms" in alignment:
            lines.append(f"- IMU 采集起始时间偏移：**{alignment['imu_start_offset_ms']:.1f} ms**")
        if "imu_end_offset_ms" in alignment:
            lines.append(f"- IMU 采集结束时间偏移：**{alignment['imu_end_offset_ms']:.1f} ms**")
        if "imu_overlap_s" in alignment:
            lines.append(f"- IMU 时间重叠区间：**{alignment['imu_overlap_s']:.1f} s**")
        if "depth_start_offset_ms" in alignment:
            lines.append(f"- Depth 采集起始时间偏移：**{alignment['depth_start_offset_ms']:.1f} ms**")
        if "clock_drift_diff_ppm" in alignment:
            lines.append(f"- 设备间时钟漂移差：**{alignment['clock_drift_diff_ppm']:.1f} ppm**")
        if "clock_drift_per_device" in alignment:
            lines.append("")
            lines.append("| 设备 | 时钟漂移 (ppm) |")
            lines.append("|------|---------------|")
            for dev, ppm in alignment["clock_drift_per_device"].items():
                lines.append(f"| {dev_labels.get(dev, dev)} | {ppm:.1f} |")
        lines.append("")
        lines.append("_注：host_ts 为采集主机时间戳（同一时钟域），device_ts 为设备内部时间戳。")
        lines.append("时钟漂移 (ppm) 由 host 间隔 / device 间隔的比值偏差估算。_")
    else:
        lines.append("_数据不足或仅有一台设备，无法进行时间同步分析_")
    lines.append("")

    lines.append("## 8. 跨设备深度差异分析")
    lines.append("")
    if cross_depth and cross_depth.get("comparison"):
        lines.append("因安装位置差异，各设备观测视角不同，深度分布不可直接等同对比。")
        lines.append("以下分析展示各设备深度分布差异，用于确认设备间是否存在系统性偏差。")
        lines.append("")
        for pair_key, comp in cross_depth["comparison"].items():
            d1, d2 = pair_key.split("_vs_")
            lines.append(f"### {dev_labels.get(d1, d1)} vs {dev_labels.get(d2, d2)}")
            lines.append("")
            if "resolution_diff" in comp:
                lines.append(f"- 分辨率差异：{comp['resolution_diff']}")
            if "mean_diff_mm" in comp:
                lines.append(f"- 全局均值差：**{comp['mean_diff_mm']:.1f} mm ({comp['mean_diff_m']:.3f} m)**")
                lines.append(f"  - {d1}: {comp['d1_global_mean_mm']:.1f} mm")
                lines.append(f"  - {d2}: {comp['d2_global_mean_mm']:.1f} mm")
            if "valid_ratio_diff" in comp:
                lines.append(f"- 有效像素比差异：{comp['valid_ratio_diff']*100:.1f}%")
            if "d1_temporal_std_mm" in comp:
                lines.append(f"- 时间稳定性对比：{d1}={comp['d1_temporal_std_mm']:.1f} mm, {d2}={comp['d2_temporal_std_mm']:.1f} mm")
            lines.append("")
            lines.append("_注：均值差异主要来源于安装位置和视角差异，不直接代表测距精度差异。_")
            lines.append("")
    else:
        lines.append("_仅有一台设备或深度数据不足，无法进行跨设备对比_")
    lines.append("")

    lines.append("## 9. 通过/不通过判定")
    lines.append("")

    all_checks = []
    for dt in dev_types:
        ds = devices_data[dt].get("depth_stats")
        imu = devices_data[dt].get("imu_stats", {})
        pcd = devices_data[dt].get("pcd_stats", {})
        color = devices_data[dt].get("color_stats", {})
        d2c = devices_data[dt].get("d2c_fusion", {})
        if ds:
            all_checks.append((f"{dev_labels.get(dt, dt)} 深度", assess_depth(ds, dt)))
        all_checks.append((f"{dev_labels.get(dt, dt)} IMU", assess_imu(imu, dt)))
        if dt == "AC1" and pcd:
            all_checks.append((f"{dev_labels.get(dt, dt)} 点云", assess_pcd(pcd)))
        if color:
            all_checks.append((f"{dev_labels.get(dt, dt)} 彩色流", assess_color(color, dt)))
        if d2c:
            all_checks.append((f"{dev_labels.get(dt, dt)} D2C融合", assess_d2c_fusion(d2c)))
        enc = devices_data[dt].get("h264_encoding", {})
        if enc.get("streams"):
            all_checks.append((f"{dev_labels.get(dt, dt)} H.264编码", assess_h264_encoding(enc, dt)))
        sensor = devices_data[dt].get("color_stats", {})
        if sensor.get("sensor_quality"):
            all_checks.append((f"{dev_labels.get(dt, dt)} 传感器成像", assess_color_sensor(sensor, dt)))

    if alignment:
        ta_checks = assess_temporal_alignment(alignment)
        if ta_checks:
            all_checks.append(("多设备时间同步", ta_checks))

    for section_name, checks in all_checks:
        if not checks:
            continue
        lines.append(f"### {section_name}")
        lines.append("")
        lines.append("| 检查项 | 实际值 | 期望值 | 结果 |")
        lines.append("|--------|--------|--------|------|")
        for name, actual, expected, passed in checks:
            status = "PASS" if passed else "FAIL"
            lines.append(f"| {name} | {actual} | {expected} | {status} |")
        lines.append("")

    total = sum(len(c) for _, c in all_checks)
    passed = sum(1 for _, checks in all_checks for _, _, _, p in checks if p)
    failed = total - passed
    lines.append(f"**总计：{total} 项检查，{passed} 通过，{failed} 不通过**")
    if failed > 0:
        lines.append("")
        lines.append("**不通过项目：**")
        for section_name, checks in all_checks:
            for name, actual, expected, p in checks:
                if not p:
                    lines.append(f"- {section_name} / {name}：实际={actual}，期望={expected}")
    lines.append("")

    lines.append("## 10. 336L 评估 (待补充)")
    lines.append("")
    ds_336l = devices_data.get("336L", {}).get("depth_stats")
    imu_336l = devices_data.get("336L", {}).get("imu_stats")
    if ds_336l:
        lines.append("_336L 数据已采集，评估结果如下_")
        lines.append("")
        lines.append("| 指标 | 336L |")
        lines.append("|------|------|")
        lines.append(f"| 分辨率 | {ds_336l.get('resolution', 'N/A')} |")
        lines.append(f"| 帧数 | {ds_336l.get('num_frames', 0)} |")
        lines.append(f"| depthScale | {ds_336l.get('depth_scale', 'N/A')} |")
        lines.append(f"| 文件大小 | {ds_336l.get('file_size_mb', 0):.1f} MB |")
        if "avg_valid_ratio" in ds_336l:
            lines.append(f"| 有效像素比 | {ds_336l['avg_valid_ratio']*100:.1f}% |")
        if "global_mean_mm" in ds_336l:
            lines.append(f"| 全局均值 | {ds_336l.get('global_mean_mm', 0):.1f} mm ({ds_336l.get('global_mean_m', 0):.3f} m) |")
    else:
        lines.append("_336L 暂无采集数据。336L（Gemini 336L, VID:PID 2BC5:0807）为335L的长距版本，")
        lines.append("官方标称测距范围 0.3–10m（vs 335L 0.3–5m），深度分辨率与335L相同（1280x800）。_")
        lines.append("")
        lines.append("| 对比项 | 335L | 336L (spec) |")
        lines.append("|--------|------|-------------|")
        lines.append("| 测距范围 | 0.3–5m | 0.3–10m |")
        lines.append("| 深度分辨率 | 1280x800 | 1280x800 |")
        lines.append("| depthScale | 0.001 | 0.001 (预估) |")
        lines.append("| D2C | SW | 待测 |")
    lines.append("")

    lines.append("## 11. 不同距离精度评估 (待补充)")
    lines.append("")
    lines.append("当有多距离采集数据后，此节将对比各设备在不同目标距离下的深度误差。")
    lines.append("")
    dist_devices = [dt for dt in dev_types if dt != "AC1"] + ["AC1"]
    header = "| 目标距离 (m) |" + "|".join([f" {dt} 误差 (mm) " for dt in dist_devices]) + "| 测试方法 |"
    sep = "|-------------|" + "|".join(["----------------"] * len(dist_devices)) + "|----------|"
    lines.append(header)
    lines.append(sep)
    for d in [0.5, 1.0, 2.0, 3.0, 5.0, 10.0, 20.0, 50.0]:
        row = f"| {d} |" + "|".join([" - " for _ in dist_devices]) + "|"
        method = "平面标定板" if d <= 5 else ("大型平面/墙面" if d <= 20 else "远距目标")
        row += f" {method} |"
        lines.append(row)
    lines.append("")
    lines.append("_测试方法：在已知距离放置标定板/平面，采集30帧以上，取深度均值与已知距离之差为系统误差，标准差为随机误差。_")
    lines.append("")

    lines.append("## 12. 关键发现与建议")
    lines.append("")
    findings = []

    depth_data = {dt: devices_data[dt].get("depth_stats") for dt in dev_types}

    if "335L" in depth_data and "AC1" in depth_data and depth_data["335L"] and depth_data["AC1"]:
        ds3 = depth_data["335L"]
        dsa = depth_data["AC1"]
        size_ratio = ds3.get("file_size_mb", 1) / max(dsa.get("file_size_mb", 0.01), 0.01)
        findings.append(f"335L depth_raw 数据量约为 AC1 的 **{size_ratio:.0f}倍**（{ds3.get('file_size_mb',0):.0f}MB vs {dsa.get('file_size_mb',0):.1f}MB），长时间采集需考虑存储压力")

    imu_data = {dt: devices_data[dt].get("imu_stats") for dt in dev_types}
    orb_z = [dt for dt in ("335L", "336L") if imu_data.get(dt) and imu_data[dt].get("gyro_noise_rms")]
    if "AC1" in imu_data and imu_data["AC1"].get("gyro_noise_rms") and orb_z:
        ref = orb_z[0]
        noise_ratio = imu_data["AC1"]["gyro_noise_rms"] / imu_data[ref]["gyro_noise_rms"] if imu_data[ref]["gyro_noise_rms"] > 0 else 0
        findings.append(f"AC1 陀螺仪噪声 RMS 约为 {ref} 的 **{noise_ratio:.1f}倍**（{imu_data['AC1']['gyro_noise_rms']:.5f} vs {imu_data[ref]['gyro_noise_rms']:.5f} rad/s），高精度姿态估计建议选用 {ref}")

    if "AC1" in imu_data and not imu_data["AC1"].get("temp_available", True):
        findings.append("AC1 IMU 温度始终为 0.0，这是 rs_driver 的已知限制（HID接口不返回温度），不支持温补校准")

    scale_issues = []
    for dt in dev_types:
        ds = depth_data.get(dt)
        if ds and ds.get("scale_is_meters") is not None:
            if ds["scale_is_meters"]:
                scale_issues.append(f"{dt}={ds['depth_scale']:.4f}(m/unit)")
            else:
                scale_issues.append(f"{dt}={ds['depth_scale']:.1f}(mm/unit)")
    if len(scale_issues) >= 2 and any("m/unit" in s for s in scale_issues) and any("mm/unit" in s for s in scale_issues):
        findings.append(f"depth_raw header scale 字段单位不一致：{', '.join(scale_issues)}，解析时需判断 scale<1.0 则乘1000转mm")

    if "335L" in depth_data and "AC1" in depth_data and depth_data["335L"] and depth_data["AC1"]:
        v335l = depth_data["335L"].get("avg_valid_ratio", 0)
        vac1 = depth_data["AC1"].get("avg_valid_ratio", 0)
        findings.append(f"335L 有效像素比 {v335l*100:.1f}% 低于 AC1 {vac1*100:.1f}%，主要因为335L超量程范围(depthMax=5m)的像素为0")

    findings.append("335L 使用 SW D2C（深度→彩色对齐），1280x800→1280x720 需软件重投影；AC1 使用 HW D2C，但 96x288→1920x1080 上采样产生块状伪影")

    if alignment and alignment.get("imu_start_offset_ms") is not None:
        offset = alignment["imu_start_offset_ms"]
        if offset > 100:
            findings.append(f"设备间 IMU 起始时间偏移 {offset:.1f} ms 较大，可能影响多设备帧同步融合的精度")
        else:
            findings.append(f"设备间 IMU 起始时间偏移仅 {offset:.1f} ms，时间同步质量良好")

    if "336L" not in dev_types or not depth_data.get("336L"):
        findings.append("336L（长距版 335L）暂无物理设备，评估需等待设备接入后补充测试（含不同距离精度评估）")

    for dt in dev_types:
        cs = devices_data[dt].get("color_stats", {})
        if cs.get("color_issue") == "SEVERE_UNDEREXPOSURE":
            findings.append(f"{dt} 彩色流严重欠曝光（亮度均值={cs.get('color_quality',{}).get('brightness_mean',0):.1f}），60%以上像素为零，疑似硬件曝光控制或触发时序问题")

    for dt in dev_types:
        d2c = devices_data[dt].get("d2c_fusion", {})
        blk = d2c.get("fused_blockiness", {})
        if blk and blk.get("very_smooth_block_pct", 0) > 30:
            findings.append(f"{dt} D2C 融合有 {blk['very_smooth_block_pct']:.1f}% 极平滑8x8块（var<5），可能源于低分辨率深度上采样")

    d2c_devs = [dt for dt in dev_types if devices_data[dt].get("d2c_fusion", {}).get("fused_available")]
    sw_devs = [dt for dt in d2c_devs if devices_data[dt].get("d2c_fusion", {}).get("d2c_mode") == "SW"]
    hw_devs = [dt for dt in d2c_devs if devices_data[dt].get("d2c_fusion", {}).get("d2c_mode") == "HW"]
    if sw_devs and hw_devs:
        findings.append(f"设备间 D2C 模式不同：{'/'.join(sw_devs)}=SW(软件重投影), {'/'.join(hw_devs)}=HW(硬件上采样)，融合视觉效果不可直接比较")

    color_335l = devices_data.get("335L", {}).get("color_stats", {})
    color_ac1 = devices_data.get("AC1", {}).get("color_stats", {})
    if color_335l.get("color_quality") and color_ac1.get("color_quality"):
        q3 = color_335l["color_quality"]
        qa = color_ac1["color_quality"]
        findings.append(f"彩色流质量：335L 锐度={q3.get('sharpness',0):.1f}、亮度={q3.get('brightness_mean',0):.1f}；AC1 锐度={qa.get('sharpness',0):.1f}、亮度={qa.get('brightness_mean',0):.1f}")

    for i, f in enumerate(findings, 1):
        lines.append(f"{i}. {f}")
    lines.append("")

    lines.append("## 13. 附录：原始数据路径")
    lines.append("")
    for dt in dev_types:
        lines.append(f"- {dev_labels.get(dt, dt)}: `{devices_data[dt]['path']}`")
    lines.append("")

    lines.append("## 14. 彩色与红外视频流分析")
    lines.append("")
    for dt in dev_types:
        cs = devices_data[dt].get("color_stats", {})
        if not cs:
            continue
        lines.append(f"### 14.{dev_types.index(dt)+1} {dev_labels.get(dt, dt)}")
        lines.append("")
        lines.append("| 指标 | 值 |")
        lines.append("|------|-----|")
        lines.append(f"| 彩色流可用 | {'是' if cs.get('color_available') else '否'} |")
        if cs.get("color_available"):
            lines.append(f"| 彩色分辨率 | {cs.get('color_resolution', 'N/A')} |")
            lines.append(f"| 彩色帧数 | {cs.get('color_frame_count', 0)} |")
            lines.append(f"| 彩色帧率 (H.264) | {cs.get('color_fps', 0):.0f} fps |")
            lines.append(f"| 彩色文件大小 | {cs.get('color_file_size_kb', 0):.0f} KB |")
            lines.append(f"| 彩色平均码率 | {cs.get('color_avg_bitrate_kbps', 0):.0f} kbps |")
            q = cs.get("color_quality", {})
            if q:
                lines.append(f"| 锐度 (Laplacian var) | {q.get('sharpness', 0):.1f} |")
                lines.append(f"| 信息熵 (bits) | {q.get('entropy', 0):.2f} |")
                lines.append(f"| 平均亮度 | {q.get('brightness_mean', 0):.1f} |")
                lines.append(f"| 亮度标准差 | {q.get('brightness_std', 0):.1f} |")
                lines.append(f"| 色相标准差 | {q.get('hue_std', 0):.1f} |")
                lines.append(f"| 饱和度均值 | {q.get('sat_mean', 0):.1f} |")
                lines.append(f"| 亮度均值 | {q.get('val_mean', 0):.1f} |")
                lines.append(f"| 非零像素比 | {q.get('nonzero_ratio', 0)*100:.1f}% |")
            if cs.get("color_issue"):
                lines.append(f"| **问题** | **{cs['color_issue']}**: {cs.get('color_issue_note', '')} |")
            if cs.get("color_depth_frame_ratio"):
                lines.append(f"| 彩色/深度_raw 帧数比 | {cs['color_depth_frame_ratio']:.2f} |")
            ir_list = cs.get("ir_streams_available", [])
            if ir_list:
                lines.append(f"| 红外流 | {', '.join(ir_list)} |")
                for ist in ir_list:
                    fc = cs.get(f"{ist}_frame_count", 0)
                    res = cs.get(f"{ist}_resolution", "N/A")
                    lines.append(f"| {ist} 帧数/分辨率 | {fc} / {res} |")
        lines.append("")

    enc_sub_idx = sum(1 for dt in dev_types if devices_data[dt].get("color_stats"))
    lines.append(f"### 14.{enc_sub_idx + 1} H.264 编码参数诊断")
    lines.append("")
    lines.append("本节从 H.264 Annex-B 码流层面分析编码配置与 NAL 单元分布，诊断编码质量相关问题。")
    lines.append("")
    for dt in dev_types:
        enc = devices_data[dt].get("h264_encoding", {})
        if not enc or not enc.get("streams"):
            continue
        lines.append(f"#### {dev_labels.get(dt, dt)}")
        lines.append("")
        for st, diag in enc["streams"].items():
            lines.append(f"**{st}**")
            lines.append("")
            lines.append("| 编码参数 | 值 | 诊断说明 |")
            lines.append("|----------|-----|----------|")
            lines.append(f"| Profile | {diag.get('profile', 'N/A')} | {'实时采集常用，无B帧，低延迟' if 'Baseline' in str(diag.get('profile','')) else '支持B帧/CABAC'} |")
            level = diag.get("level")
            level_note = ""
            if level is not None:
                if level <= 10:
                    level_note = "仅支持QVGA"
                elif level <= 21:
                    level_note = "支持SD"
                elif level <= 31:
                    level_note = "支持720p/1080p@30"
                elif level <= 40:
                    level_note = "支持1080p@30/60"
                else:
                    level_note = "支持高分辨率/帧率"
            lines.append(f"| Level | {level if level is not None else 'N/A'} | {level_note} |")
            lines.append(f"| 分辨率 | {diag.get('resolution', 'N/A')} | |")
            lines.append(f"| 帧数 (VCL NAL) | {diag.get('nal_frame_count', 0)} | IDR+non-IDR 帧数 |")
            lines.append(f"| pix_fmt | {diag.get('pix_fmt', 'N/A')} | {'JPEG色彩范围(full)，H.264编码器默认' if 'j420' in str(diag.get('pix_fmt','')) else '标准范围'} |")
            lines.append(f"| color_range | {diag.get('color_range', 'N/A')} | {'pc=full range (0-255)' if diag.get('color_range') == 'pc' else 'tv=limited range (16-235)'} |")
            lines.append(f"| color_space | {diag.get('color_space', 'N/A')} | {'BT.709 (HD标准)' if diag.get('color_space') == 'bt709' else ''} |")
            lines.append(f"| color_primaries | {diag.get('color_primaries', 'N/A')} | |")
            lines.append(f"| color_transfer | {diag.get('color_transfer', 'N/A')} | |")
            refs = diag.get("refs")
            lines.append(f"| 参考帧数 | {refs if refs is not None else 'N/A'} | {'1=仅前向参考(Baseline)' if refs == 1 else ''} |")
            bframes = diag.get("has_b_frames")
            lines.append(f"| B-frames | {bframes if bframes is not None else 'N/A'} | {'0=无B帧(实时采集)' if bframes == 0 else '含B帧(非实时，增加延迟)'} |")
            lines.append(f"| 平均码率 | {diag.get('avg_bitrate_kbps', 0)} kbps | 按文件大小/帧数估算 |")
            lines.append(f"| 平均帧大小 | {diag.get('avg_frame_bytes', 0)} bytes |")
            lines.append(f"| 文件大小 | {diag.get('file_size_kb', 0)} KB | |")

            nal = diag.get("nal_types", {})
            nal_desc = ", ".join([f"{NAL_TYPE_NAMES.get(k, f'type{k}')}={v}" for k, v in sorted(nal.items())])
            lines.append(f"| NAL 单元分布 | {nal_desc} | SPS/PPS/SEI 为控制信息，IDR/non-IDR 为视频帧 |")

            idr = diag.get("idr_count", 0)
            non_idr = diag.get("non_idr_count", 0)
            gop = diag.get("avg_gop_size", 0)
            gop_note = ""
            if gop > 0:
                if gop <= 2:
                    gop_note = "全帧/近全帧IDR，码率较高但随机访问最佳"
                elif gop <= 30:
                    gop_note = f"约每{gop:.0f}帧一个关键帧，平衡码率与随机访问"
                elif gop <= 120:
                    gop_note = f"GOP较大({gop:.0f}帧)，码率低但seek/跳帧恢复慢"
                else:
                    gop_note = f"GOP过大({gop:.0f}帧)，seek延迟高，可能影响播放体验"
            lines.append(f"| IDR 关键帧数 / GOP大小 | {idr} / {gop:.1f} 帧 | {gop_note} |")
            lines.append("")

    sq_sub_idx = enc_sub_idx + 2
    lines.append(f"### 14.{sq_sub_idx} 彩色传感器成像质量分析")
    lines.append("")
    lines.append("本节从传感器物理成像层面分析彩色流图像质量，区分 MJPEG 解压伪影与原始传感器噪声。")
    lines.append("335L 彩色源为 MJPEG（经 yuvj422p→yuv420p sws 转换），AC1 为 NV12 直出，噪声来源不同。")
    lines.append("")
    for dt in dev_types:
        cs = devices_data[dt].get("color_stats", {})
        sq = cs.get("sensor_quality", {})
        if not sq:
            continue
        lines.append(f"#### {dev_labels.get(dt, dt)}")
        lines.append("")
        lines.append(f"- **彩色源类型**: {sq.get('source_type', 'unknown')}")
        lines.append(f"- **噪声归因**: {sq.get('noise_attribution', '')}")
        lines.append("")
        lines.append("| 成像质量指标 | 值 | 说明 |")
        lines.append("|--------------|-----|------|")
        if "inter_frame_ssim_adjacent_mean" in sq:
            lines.append(f"| 帧间 SSIM 均值 (相邻帧) | {sq['inter_frame_ssim_adjacent_mean']:.4f} | 1帧间隔，反映传感器+编码稳定性 |")
        if "inter_frame_ssim_adjacent_min" in sq:
            lines.append(f"| 帧间 SSIM 最低 (相邻帧) | {sq['inter_frame_ssim_adjacent_min']:.4f} | 最差相邻帧对 |")
        if "inter_frame_ssim_spread_mean" in sq:
            lines.append(f"| 帧间 SSIM 均值 (间隔帧) | {sq['inter_frame_ssim_spread_mean']:.4f} | 远间隔帧，反映场景变化+长期稳定性 |")
        if "inter_frame_ssim_mean" in sq:
            lines.append(f"| 帧间 SSIM 均值 (全部) | {sq['inter_frame_ssim_mean']:.4f} | 所有帧对综合 |")
        if "inter_frame_ssim_std" in sq:
            lines.append(f"| 帧间 SSIM 标准差 | {sq['inter_frame_ssim_std']:.4f} | 波动越小越稳定 |")
        if "inter_frame_psnr_mean" in sq:
            lines.append(f"| 帧间 PSNR 均值 | {sq['inter_frame_psnr_mean']:.2f} dB | 时间域信噪比 |")
        if "noise_laplacian_std" in sq:
            lines.append(f"| Laplacian 噪声估计 | {sq['noise_laplacian_std']:.1f} | 高频纹理/噪声强度 |")
        if "snr_proxy_db" in sq:
            lines.append(f"| SNR 代理 (dB) | {sq['snr_proxy_db']:.2f} | 信号/噪声功率比 (高频估计) |")
        if "dark_pixel_pct" in sq:
            lines.append(f"| 暗区像素占比 | {sq['dark_pixel_pct']:.1f}% | 亮度<30 像素 |")
        if "dark_region_noise_std" in sq and sq["dark_region_noise_std"] > 0:
            lines.append(f"| 暗区噪声标准差 | {sq['dark_region_noise_std']:.2f} | 暗区 (gray<30) 内噪声 |")
            lines.append(f"| 暗区均值 | {sq['dark_region_mean']:.2f} | 暗区平均亮度 |")
        if "chroma_snr_db" in sq:
            lines.append(f"| 色度信噪比 | {sq['chroma_snr_db']:.2f} dB | HSV S通道 SNR |")
        if "hist_clip_lo_pct" in sq:
            lines.append(f"| 阴影裁切 (gray=0) | {sq['hist_clip_lo_pct']:.1f}% | 灰度直方图低端裁切 |")
        if "hist_clip_hi_pct" in sq:
            lines.append(f"| 高光裁切 (gray=255) | {sq['hist_clip_hi_pct']:.1f}% | 灰度直方图高端裁切 |")
        if "sharpness_mean" in sq:
            lines.append(f"| 锐度均值 (Laplacian var) | {sq['sharpness_mean']:.2f} | 多帧锐度平均 |")
        if "sharpness_std" in sq:
            lines.append(f"| 锐度标准差 | {sq['sharpness_std']:.2f} | 帧间锐度波动 |")
        if "sharpness_temporal_stability" in sq:
            lines.append(f"| 锐度时间稳定性 (CV) | {sq['sharpness_temporal_stability']:.4f} | 变异系数，<0.5为稳定 |")
        if "edge_preservation" in sq:
            lines.append(f"| 边缘保留强度 | {sq['edge_preservation']:.2f} | 原始-模糊 差分均值 |")
        lines.append("")

    lines.append("## 15. D2C 深度-彩色融合分析")
    lines.append("")
    for dt in dev_types:
        d2c = devices_data[dt].get("d2c_fusion", {})
        if not d2c:
            continue
        lines.append(f"### 15.{dev_types.index(dt)+1} {dev_labels.get(dt, dt)}")
        lines.append("")
        lines.append("| 指标 | 值 |")
        lines.append("|------|-----|")
        lines.append(f"| 融合流可用 | {'是' if d2c.get('fused_available') else '否'} |")
        if d2c.get("fused_available"):
            lines.append(f"| D2C 模式 | {d2c.get('d2c_mode', 'N/A')} |")
            lines.append(f"| Alpha 融合系数 | {d2c.get('alpha', 'N/A')} |")
            lines.append(f"| 融合方法说明 | {d2c.get('fusion_method_note', '')} |")
            lines.append(f"| 融合输出分辨率 | {d2c.get('fused_resolution', 'N/A')} |")
            lines.append(f"| 融合帧数 | {d2c.get('fused_frame_count', 0)} |")
            lines.append(f"| 融合文件大小 | {d2c.get('fused_file_size_kb', 0):.0f} KB |")
            lines.append(f"| 融合平均码率 | {d2c.get('fused_avg_bitrate_kbps', 0):.0f} kbps |")
            lines.append(f"| 彩色帧数 | {d2c.get('color_frame_count', 0)} |")
            lines.append(f"| 深度帧数 | {d2c.get('depth_frame_count', 0)} |")
            lines.append(f"| 彩色/融合 帧数比 | {d2c.get('color_to_fused_frame_ratio', 0):.2f} |")
            fq = d2c.get("fused_quality", {})
            if fq:
                lines.append(f"| 融合锐度 | {fq.get('sharpness', 0):.1f} |")
                lines.append(f"| 融合信息熵 | {fq.get('entropy', 0):.2f} |")
                lines.append(f"| 融合亮度均值 | {fq.get('brightness_mean', 0):.1f} |")
                lines.append(f"| 融合色相标准差 | {fq.get('hue_std', 0):.1f} |")
            blk = d2c.get("fused_blockiness", {})
            if blk:
                lines.append(f"| 8x8块边界均值 | {blk.get('boundary_mean', 0):.2f} |")
                lines.append(f"| 8x8块边界 P95 | {blk.get('boundary_p95', 0):.2f} |")
                lines.append(f"| 8x8块边界最大值 | {blk.get('boundary_max', 0):.0f} |")
                lines.append(f"| 块内方差均值 | {blk.get('block_var_mean', 0):.1f} |")
                lines.append(f"| 极平滑块占比 (var<5) | {blk.get('very_smooth_block_pct', 0):.1f}% |")
            if "color_vs_fused_ssim" in d2c:
                lines.append(f"| Color vs Fused SSIM | {d2c['color_vs_fused_ssim']:.4f} |")
                lines.append(f"| Color vs Fused PSNR | {d2c['color_vs_fused_psnr_db']:.2f} dB |")
                detail = d2c.get("color_vs_fused_detail", {})
                if detail.get("ssim_per_channel"):
                    lines.append(f"| SSIM 逐通道 (B/G/R) | {detail['ssim_per_channel']} |")
        lines.append("")

    lines.append("## 16. 跨设备融合方法对比")
    lines.append("")
    d2c_devs = [dt for dt in dev_types if devices_data[dt].get("d2c_fusion", {}).get("fused_available")]
    if len(d2c_devs) >= 2:
        lines.append("以下对比不同设备 D2C 融合方式的差异：")
        lines.append("")
        header = "| 对比项 |" + "|".join([f" {dev_labels.get(dt, dt)} " for dt in d2c_devs]) + "|"
        sep = "|--------|" + "|".join(["------" for _ in d2c_devs]) + "|"
        lines.append(header)
        lines.append(sep)
        for row_key, row_label in [
            ("d2c_mode", "D2C 模式"),
            ("fused_resolution", "融合分辨率"),
            ("fused_frame_count", "融合帧数"),
            ("fusion_method_note", "融合方法"),
        ]:
            row = f"| {row_label} |"
            for dt in d2c_devs:
                v = devices_data[dt]["d2c_fusion"].get(row_key, "N/A")
                row += f" {v} |"
            lines.append(row)
        for row_key, row_label in [
            ("sharpness", "锐度"),
            ("entropy", "信息熵"),
            ("brightness_mean", "亮度均值"),
            ("hue_std", "色相标准差"),
        ]:
            row = f"| {row_label} |"
            for dt in d2c_devs:
                q = devices_data[dt]["d2c_fusion"].get("fused_quality", {})
                v = f"{q.get(row_key, 'N/A')}" if row_key in q else "N/A"
                row += f" {v} |"
            lines.append(row)
        for row_key, row_label in [
            ("boundary_mean", "块边界均值"),
            ("boundary_p95", "块边界P95"),
            ("very_smooth_block_pct", "极平滑块占比%"),
        ]:
            row = f"| {row_label} |"
            for dt in d2c_devs:
                blk = devices_data[dt]["d2c_fusion"].get("fused_blockiness", {})
                v = f"{blk.get(row_key, 'N/A')}" if row_key in blk else "N/A"
                row += f" {v} |"
            lines.append(row)
        lines.append("")

        lines.append("### 16.1 SW D2C vs HW D2C 分析")
        lines.append("")
        lines.append("- **SW D2C（335L/336L）**：深度帧 1280x800 通过软件重投影对齐到彩色 1280x720，")
        lines.append("  深度像素被插值到彩色平面，融合图保持高空间分辨率，边缘可能出现轻微对齐偏差。")
        lines.append("  Alpha-blend 融合公式：`fused = (1-alpha)*color + alpha*jetmap(depth)`。")
        lines.append("- **HW D2C（AC1）**：深度 96x288 通过硬件上采样到彩色 1920x1080 分辨率，")
        lines.append("  上采样比例极高（~20x），导致明显的块状伪影。深度信息在 8x8 块内均匀填充，")
        lines.append("  块边界处出现颜色/亮度跳变。D2C 融合帧率受制于深度帧率（~10fps vs 彩色 ~30fps）。")
        lines.append("")
        lines.append("_注：AC1 的 HW D2C 块状伪影是 96x288→1920x1080 上采样的固有特征，不代表深度原始数据质量问题。")
        lines.append("使用原始 depth_raw 或 PCD 数据可避免此伪影。_")
        lines.append("")
    elif len(d2c_devs) == 1:
        lines.append(f"_仅 {dev_labels.get(d2c_devs[0], d2c_devs[0])} 有 D2C 融合数据，无法进行跨设备对比_")
        lines.append("")
    else:
        lines.append("_无 D2C 融合数据可用_")
        lines.append("")

    report = "\n".join(lines)
    with open(output_path, 'w') as f:
        f.write(report)
    print(f"Evaluation report written to: {output_path}")
    return report


def _find_imu_path(data_root):
    return None


def _find_pcd_dir(data_root):
    return None


def _dir_size_mb(path):
    total = 0
    for dirpath, dirnames, filenames in os.walk(path):
        for fn in filenames:
            fp = os.path.join(dirpath, fn)
            if os.path.isfile(fp):
                total += os.path.getsize(fp)
    return total / (1024 * 1024)


def evaluate_device(dev_path, dev_type, session_log=None):
    result = {"path": dev_path, "depth_stats": {}, "imu_stats": {}, "pcd_stats": {}, "intrinsic": None, "log_info": {}, "h264_stats": {}, "d2c_fusion": {}, "color_stats": {}, "h264_encoding": {}}
    files = find_files(dev_path)
    result["_files"] = files
    files = find_files(dev_path)

    if files["raw"]:
        print(f"  Parsing depth_raw: {files['raw']}")
        depth_info = parse_depth_raw(files["raw"])
        if depth_info:
            print(f"    {depth_info['num_frames']} frames, {depth_info['width']}x{depth_info['height']}, scale={depth_info['scale']}")
            result["depth_stats"] = analyze_depth(depth_info, dev_type)
            ds = result["depth_stats"]
            print(f"    Valid ratio: {ds.get('avg_valid_ratio', 0)*100:.1f}%, Global mean: {ds.get('global_mean_mm', 0):.1f} mm")

    if files["imu"]:
        print(f"  Parsing IMU: {files['imu']}")
        imu_records = parse_imu(files["imu"])
        result["imu_stats"] = analyze_imu(imu_records, dev_type)
        imu = result["imu_stats"]
        print(f"    {imu.get('total_samples', 0)} samples, accel={imu.get('accel_rate_hz',0):.0f}Hz, gyro={imu.get('gyro_rate_hz',0):.0f}Hz")

    if files["pcd_dir"]:
        print(f"  Analyzing PCD: {files['pcd_dir']}")
        result["pcd_stats"] = analyze_pcd_dir(files["pcd_dir"], dev_type)
        pcd = result["pcd_stats"]
        print(f"    {pcd.get('total_pcd_files', 0)} PCD files, avg points={pcd.get('avg_points', 0):.0f}")

    result["intrinsic"] = load_intrinsic(files["intrinsic"])
    log_path = files["log"] or session_log
    result["log_info"] = parse_capture_log(log_path)

    if files["h264_by_type"]:
        print(f"  Analyzing H.264 streams: {list(files['h264_by_type'].keys())}")
        result["h264_stats"] = analyze_h264_streams(files["h264_by_type"], dev_type, dev_type)
        print(f"  Analyzing H.264 encoding diagnostics...")
        result["h264_encoding"] = analyze_h264_encoding(files["h264_by_type"], dev_type)
        print(f"  Analyzing color stream quality...")
        result["color_stats"] = analyze_color_streams(files["h264_by_type"], dev_type, dev_type, result["depth_stats"])
        cs = result["color_stats"]
        if cs.get("color_issue"):
            print(f"    WARNING: Color issue={cs['color_issue']} ({cs.get('color_issue_note', '')})")
        print(f"  Analyzing D2C fusion...")
        result["d2c_fusion"] = analyze_d2c_fusion(files["h264_by_type"], dev_type, dev_type, result["log_info"])

    return result


def main():
    parser = argparse.ArgumentParser(description="NIO Multi-Capture Post-Capture Evaluation")
    parser.add_argument("data_dirs", nargs="+", help="Session data directory (auto-discover) or per-device dirs (335L AC1)")
    parser.add_argument("--data-dir-336l", default=None, help="336L capture data root directory (optional)")
    parser.add_argument("--output", default="evaluation_report.md", help="Output report path")
    parser.add_argument("--full", action="store_true", help="Include verbose per-frame stats")
    args = parser.parse_args()

    devices_data = {}

    if len(args.data_dirs) == 1 and os.path.isdir(args.data_dirs[0]):
        root = args.data_dirs[0]
        discovered, log_file = discover_devices(root)
        if discovered:
            print(f"=== Auto-discovered {len(discovered)} device(s) in {root} ===")
            for dt, info in discovered.items():
                print(f"  {dt}: {info['path']}")
            for dt, info in discovered.items():
                print(f"\n=== Evaluating {dt} ===")
                devices_data[dt] = evaluate_device(info["path"], dt, session_log=log_file)
            if log_file:
                print(f"\nSession log: {log_file}")
        else:
            print(f"Warning: No devices auto-discovered in {root}, treating as single device directory")
            devices_data["unknown_0"] = evaluate_device(root, "unknown_0")
    elif len(args.data_dirs) >= 2:
        for i, d in enumerate(args.data_dirs):
            if not os.path.isdir(d):
                print(f"Warning: {d} is not a directory, skipping")
                continue
            discovered, session_log = discover_devices(d)
            if discovered:
                for dt, info in discovered.items():
                    print(f"\n=== Evaluating {dt} (from {d}) ===")
                    devices_data[dt] = evaluate_device(info["path"], dt, session_log=session_log)
            else:
                dt = "335L" if i == 0 else ("AC1" if i == 1 else f"unknown_{i}")
                print(f"\n=== Evaluating {dt} (legacy mode: {d}) ===")
                devices_data[dt] = evaluate_device(d, dt)

    if args.data_dir_336l and "336L" not in devices_data and os.path.isdir(args.data_dir_336l):
        print(f"\n=== Evaluating 336L (explicit) ===")
        devices_data["336L"] = evaluate_device(args.data_dir_336l, "336L")

    if not devices_data:
        print("ERROR: No device data found!")
        sys.exit(1)

    print(f"\n=== Temporal alignment analysis ===")
    alignment = analyze_temporal_alignment(devices_data)
    if alignment:
        for k, v in alignment.items():
            if k != "clock_drift_per_device":
                print(f"  {k}: {v}")

    print(f"\n=== Cross-device depth analysis ===")
    cross_depth = analyze_cross_device_depth(devices_data)
    if cross_depth and cross_depth.get("comparison"):
        for pair, comp in cross_depth["comparison"].items():
            print(f"  {pair}: mean_diff={comp.get('mean_diff_mm', 'N/A')}mm")

    print(f"\n=== Generating evaluation report ===")
    report = generate_report(devices_data, alignment, cross_depth, args.output)

    pass_count = report.count("PASS")
    fail_count = report.count("FAIL")
    print(f"\n=== Done: {pass_count} PASS, {fail_count} FAIL ===")


if __name__ == "__main__":
    main()
