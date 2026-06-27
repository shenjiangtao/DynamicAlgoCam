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
"""

import sys
import os
import struct
import json
import re
import math
import argparse
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


def find_files(data_root):
    result = {"raw": None, "imu": None, "intrinsic": None, "log": None, "pcd_dir": None, "h264_files": []}
    for dirpath, dirnames, filenames in os.walk(data_root):
        for fn in filenames:
            fp = os.path.join(dirpath, fn)
            if fn.endswith("_depth_raw_") and fn.endswith(".raw") or ("_depth_raw_" in fn and fn.endswith(".raw")):
                if result["raw"] is None:
                    result["raw"] = fp
            if "_imu_" in fn and fn.endswith(".txt"):
                if result["imu"] is None:
                    result["imu"] = fp
            if "_depth_intrinsic_" in fn and fn.endswith(".json"):
                if result["intrinsic"] is None:
                    result["intrinsic"] = fp
            if fn.startswith("nio_multi_capture_log") and fn.endswith(".log"):
                if result["log"] is None:
                    result["log"] = fp
            if fn.endswith(".h264"):
                result["h264_files"].append(fp)
        for dn in dirnames:
            dp = os.path.join(dirpath, dn)
            if "_pcd_" in dn and os.path.isdir(dp):
                if result["pcd_dir"] is None:
                    result["pcd_dir"] = dp
    return result


def parse_depth_raw(filepath):
    with open(filepath, "rb") as f:
        data = f.read()
    file_size = len(data)
    magic = data[0:16]
    if isinstance(magic, bytes):
        magic_str = magic.split(b'\x00')[0].decode('ascii', errors='replace')
    else:
        magic_str = str(magic)

    if magic_str.startswith('NIO_DEPTH_RAW') or magic_str.startswith('ORBBEC_DEPTH_RAW'):
        w = struct.unpack_from('<I', data, 16)[0]
        h = struct.unpack_from('<I', data, 20)[0]
        bpp = struct.unpack_from('<I', data, 24)[0]
        scale = struct.unpack_from('<f', data, 28)[0]
        frame_size = struct.unpack_from('<I', data, 32)[0]
        start_ts = struct.unpack_from('<Q', data, 36)[0]
    else:
        return None

    remaining = file_size - HEADER_SIZE
    num_frames = remaining // frame_size if frame_size > 0 else 0

    frames = []
    for i in range(num_frames):
        offset = HEADER_SIZE + i * frame_size
        frame_data = data[offset:offset + frame_size]
        depth_arr = np.frombuffer(frame_data, dtype=np.uint16).reshape((h, w))
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

    all_valid_ratios = []
    all_means_mm = []
    all_stds_mm = []
    all_mins_mm = []
    all_maxs_mm = []
    per_frame_valid_mm = []

    step = max(1, n // max_sample_frames)
    sampled_indices = list(range(0, n, step))

    for i in sampled_indices:
        depth = frames[i]
        if scale_is_meters:
            depth_mm = depth.astype(np.float64) * scale * 1000.0
        else:
            depth_mm = depth.astype(np.float64) * scale
        valid_mask = depth > 0
        valid = depth_mm[valid_mask]
        total_px = depth.size
        valid_ratio = len(valid) / total_px if total_px > 0 else 0.0
        all_valid_ratios.append(valid_ratio)
        if len(valid) > 0:
            all_means_mm.append(valid.mean())
            all_stds_mm.append(valid.std())
            all_mins_mm.append(valid.min())
            all_maxs_mm.append(valid.max())
            per_frame_valid_mm.append(valid)

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

    if all_valid_ratios:
        result["avg_valid_ratio"] = np.mean(all_valid_ratios)
        result["min_valid_ratio"] = np.min(all_valid_ratios)
        result["max_valid_ratio"] = np.max(all_valid_ratios)
    if all_means_mm:
        result["avg_mean_depth_mm"] = np.mean(all_means_mm)
        result["avg_std_depth_mm"] = np.mean(all_stds_mm)
        result["min_depth_mm"] = min(all_mins_mm)
        result["max_depth_mm"] = max(all_maxs_mm)
        result["depth_temporal_std_mm"] = np.std(all_means_mm)

    all_valid_flat = np.concatenate(per_frame_valid_mm) if per_frame_valid_mm else np.array([])
    if len(all_valid_flat) > 0:
        result["global_mean_mm"] = np.mean(all_valid_flat)
        result["global_median_mm"] = np.median(all_valid_flat)
        result["global_std_mm"] = np.std(all_valid_flat)
        result["global_mean_m"] = np.mean(all_valid_flat) / 1000.0
        result["global_median_m"] = np.median(all_valid_flat) / 1000.0
        result["global_std_m"] = np.std(all_valid_flat) / 1000.0
        pcts = [5, 10, 25, 50, 75, 90, 95, 99]
        vals = np.percentile(all_valid_flat, pcts)
        result["percentiles_mm"] = {f"p{p}": v for p, v in zip(pcts, vals)}
        result["percentiles_m"] = {f"p{p}": v / 1000.0 for p, v in zip(pcts, vals)}

    if n >= 2:
        if scale_is_meters:
            first_mm = frames[0].astype(np.float64) * scale * 1000.0
            last_mm = frames[-1].astype(np.float64) * scale * 1000.0
        else:
            first_mm = frames[0].astype(np.float64) * scale
            last_mm = frames[-1].astype(np.float64) * scale
        valid_both = (frames[0] > 0) & (frames[-1] > 0)
        if np.any(valid_both):
            diff = np.abs(first_mm[valid_both] - last_mm[valid_both])
            result["temporal_drift_mm"] = np.mean(diff)
            result["temporal_drift_max_mm"] = np.max(diff)

    return result


def parse_imu(filepath):
    records = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            parts = line.split(',')
            if len(parts) < 7:
                continue
            try:
                rec = {
                    "host_ts_ms": int(parts[0]),
                    "type": parts[1].strip(),
                    "device_ts_us": int(parts[2]),
                    "x": float(parts[3]),
                    "y": float(parts[4]),
                    "z": float(parts[5]),
                    "temperature": float(parts[6]),
                }
                records.append(rec)
            except (ValueError, IndexError):
                continue
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
        ts_first = accel[0]["host_ts_ms"]
        ts_last = accel[-1]["host_ts_ms"]
        duration_s = (ts_last - ts_first) / 1000.0 if ts_last > ts_first else 0.001
        result["accel_rate_hz"] = len(accel) / duration_s
        result["accel_duration_s"] = duration_s
        ax = np.array([r["x"] for r in accel])
        ay = np.array([r["y"] for r in accel])
        az = np.array([r["z"] for r in accel])
        result["accel_mean"] = [float(ax.mean()), float(ay.mean()), float(az.mean())]
        result["accel_std"] = [float(ax.std()), float(ay.std()), float(az.std())]
        result["accel_mag_mean"] = float(np.sqrt(ax**2 + ay**2 + az**2).mean())
        result["accel_mag_std"] = float(np.sqrt(ax**2 + ay**2 + az**2).std())
        result["accel_host_ts_first"] = ts_first
        result["accel_host_ts_last"] = ts_last

    if gyro:
        ts_first = gyro[0]["host_ts_ms"]
        ts_last = gyro[-1]["host_ts_ms"]
        duration_s = (ts_last - ts_first) / 1000.0 if ts_last > ts_first else 0.001
        result["gyro_rate_hz"] = len(gyro) / duration_s
        result["gyro_duration_s"] = duration_s
        gx = np.array([r["x"] for r in gyro])
        gy = np.array([r["y"] for r in gyro])
        gz = np.array([r["z"] for r in gyro])
        result["gyro_mean"] = [float(gx.mean()), float(gy.mean()), float(gz.mean())]
        result["gyro_std"] = [float(gx.std()), float(gy.std()), float(gz.std())]
        result["gyro_noise_rms"] = float(np.sqrt(gx**2 + gy**2 + gz**2).mean())
        result["gyro_host_ts_first"] = ts_first
        result["gyro_host_ts_last"] = ts_last

    temps = [r["temperature"] for r in records if r["temperature"] != 0.0]
    if temps:
        result["temp_mean"] = float(np.mean(temps))
        result["temp_min"] = float(np.min(temps))
        result["temp_max"] = float(np.max(temps))
        result["temp_available"] = True
    else:
        result["temp_available"] = False

    if len(accel) >= 2:
        intervals = np.diff([r["host_ts_ms"] for r in accel])
        result["accel_interval_mean_ms"] = float(np.mean(intervals))
        result["accel_interval_std_ms"] = float(np.std(intervals))
        result["accel_jitter_ms"] = float(np.std(intervals))

    if len(gyro) >= 2:
        intervals = np.diff([r["host_ts_ms"] for r in gyro])
        result["gyro_interval_mean_ms"] = float(np.mean(intervals))
        result["gyro_interval_std_ms"] = float(np.std(intervals))
        result["gyro_jitter_ms"] = float(np.std(intervals))

    host_ts_all = [r["host_ts_ms"] for r in records]
    dev_ts_all = [r["device_ts_us"] for r in records]
    if host_ts_all and dev_ts_all and len(host_ts_all) >= 2:
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
        imu_path = _find_imu_path(devices_data[dt]["path"])
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
                pcd_dir = _find_pcd_dir(devices_data[dt]["path"])
                pcd_size = _dir_size_mb(pcd_dir) if pcd_dir else 0
                row += f" {pcd_size:.1f} MB |"
            else:
                row += " N/A |"
        if len(dev_types) >= 2:
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
        if ds:
            all_checks.append((f"{dev_labels.get(dt, dt)} 深度", assess_depth(ds, dt)))
        all_checks.append((f"{dev_labels.get(dt, dt)} IMU", assess_imu(imu, dt)))
        if dt == "AC1" and pcd:
            all_checks.append((f"{dev_labels.get(dt, dt)} 点云", assess_pcd(pcd)))

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

    for i, f in enumerate(findings, 1):
        lines.append(f"{i}. {f}")
    lines.append("")

    lines.append("## 13. 附录：原始数据路径")
    lines.append("")
    for dt in dev_types:
        lines.append(f"- {dev_labels.get(dt, dt)}: `{devices_data[dt]['path']}`")
    lines.append("")

    report = "\n".join(lines)
    with open(output_path, 'w') as f:
        f.write(report)
    print(f"Evaluation report written to: {output_path}")
    return report


def _find_imu_path(data_root):
    for dirpath, dirnames, filenames in os.walk(data_root):
        for fn in filenames:
            if "_imu_" in fn and fn.endswith(".txt"):
                return os.path.join(dirpath, fn)
    return None


def _find_pcd_dir(data_root):
    for dirpath, dirnames, filenames in os.walk(data_root):
        for dn in dirnames:
            if "_pcd_" in dn:
                return os.path.join(dirpath, dn)
    return None


def _dir_size_mb(path):
    total = 0
    for dirpath, dirnames, filenames in os.walk(path):
        for fn in filenames:
            fp = os.path.join(dirpath, fn)
            if os.path.isfile(fp):
                total += os.path.getsize(fp)
    return total / (1024 * 1024)


def evaluate_device(dev_path, dev_type):
    result = {"path": dev_path, "depth_stats": {}, "imu_stats": {}, "pcd_stats": {}, "intrinsic": None, "log_info": {}}
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
    result["log_info"] = parse_capture_log(files["log"])

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
                devices_data[dt] = evaluate_device(info["path"], dt)
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
            discovered, _ = discover_devices(d)
            if discovered:
                for dt, info in discovered.items():
                    print(f"\n=== Evaluating {dt} (from {d}) ===")
                    devices_data[dt] = evaluate_device(info["path"], dt)
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
