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

# Optional plotting dependencies — if missing, the report degrades gracefully
# to text-only (no embedded images). matplotlib + cv2 are only needed by the
# "raw data figures" feature (depth heatmap, IMU time series/spectrum, color
# vs D2C frame diff overlays). Per the report's "test description / method /
# significance" text sections do NOT depend on these.
try:
    import matplotlib
    matplotlib.use("Agg")  # non-interactive backend safe for headless CI
    import matplotlib.pyplot as plt
    # Figure labels use CJK; default DejaVu Sans lacks the glyphs and emits
    # "missing glyph" warnings + tofu boxes. Probe for a CJK font and prefer
    # it globally when present (Noto CJK / WenQuanYi / SimHei). Best-effort:
    # if no CJK font is found, Chinese labels become tofu — still visible
    # structurally, no crash.
    try:
        import matplotlib.font_manager as _fm
        # Probe order — accept regional variants (SC/TC/JP share most glyphs
        # with minor visible stylistic differences; the report has no JP text
        # so SC/TC/JP are all acceptable for our CJK labels).
        _cjk_candidates = [
            "Noto Sans CJK SC", "Noto Sans CJK TC", "Noto Sans CJK JP",
            "Noto Serif CJK SC", "Noto Serif CJK TC", "Noto Serif CJK JP",
            "WenQuanYi Zen Hei", "WenQuanYi Micro Hei",
            "SimHei", "Microsoft YaHei",
            "LXGW WenKai",
        ]
        _available = set(f.name for f in _fm.fontManager.ttflist)
        for _cand in _cjk_candidates:
            if _cand in _available:
                plt.rcParams["font.family"] = [_cand, "DejaVu Sans"]
                break
    except Exception:
        pass
    _MPL_AVAILABLE = True
except ImportError:
    _MPL_AVAILABLE = False
try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False

HEADER_SIZE = 44
NIO_DEPTH_RAW_MAGIC = b"NIO_DEPTH_RAW"
ORBBEC_DEPTH_RAW_MAGIC = b"ORBBEC_DEPTH_RAW"

DEVICE_PATTERNS = {
    "335L": ["335L", "Gemini_335L", "Orbbec_Gemini_335L"],
    "336L": ["336L", "Gemini_336L", "Orbbec_Gemini_336L"],
    "AC1": ["AC1", "RoboSense_AC1", "RS_AC1"],
}

DEVICE_SPECS = {
    "335L": {
        "vendor": "Orbbec", "model": "Gemini 335L",
        "depth_tech": "Active Stereo IR (Structured Light Speckle)",
        "depth_res": "1280x800", "depth_fps": 30,
        "depth_range_m": (0.3, 5.0),
        "depth_fov": "90°x65°",
        "color_res": "1280x800", "color_fps": 30, "color_fov": "86°x55°",
        "baseline_mm": 50, "depth_accuracy_pct": None,
        "imu_rate_hz": 200, "ir_streams": 2,
        "d2c_mode": "SW", "ir_filter": False,
    },
    "336L": {
        "vendor": "Orbbec", "model": "Gemini 336L",
        "depth_tech": "Active Stereo IR (Structured Light Speckle + IR Pass Filter)",
        "depth_res": "1280x800", "depth_fps": 30,
        "depth_range_m": (0.1, 20.0),
        "depth_fov": "90°x65°",
        "spatial_accuracy_pct": 1.5,
        "color_res": "1920x1080", "color_fps": 30, "color_fov": "86°x55°",
        "baseline_mm": 50, "depth_accuracy_pct": 1.5,
        "imu_rate_hz": 200, "ir_streams": 2,
        "d2c_mode": "SW", "ir_filter": True,
    },
    "AC1": {
        "vendor": "RoboSense", "model": "Active Camera 1",
        "depth_tech": "VCSEL+SPAD+CMOS TOF LiDAR (All-Solid-State)",
        "depth_res": "point_grid", "depth_fps": 10,
        "depth_range_m": (0.1, 70.0), "depth_range_10pct_m": 40.0,
        "depth_fov": "120°x90°",
        "depth_accuracy_cm": (1.0, 5.0), "depth_accuracy_cm_far": (3.0, 5.0),
        "color_res": "1920x1080", "color_fps": 30, "color_fov": "144°x78°",
        "pts_per_sec": 173333,
        "imu_rate_hz": 200, "ir_streams": 0,
        "d2c_mode": "HW", "laser_wavelength_nm": 940,
        "sunlight_resistance_klux": 100,
    },
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
            if fn.startswith("dynamic_algo_cam_log") and fn.endswith(".log"):
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
            if fn.startswith("dynamic_algo_cam_log") and fn.endswith(".log"):
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
        for _dir in [os.path.dirname(data_root.rstrip("/")),
                     os.path.dirname(os.path.dirname(data_root.rstrip("/")))]:
            if _dir and os.path.isdir(_dir):
                for fn in os.listdir(_dir):
                    if fn.startswith("dynamic_algo_cam_log") and fn.endswith(".log"):
                        result["log"] = os.path.join(_dir, fn)
                        break
                if result["log"]:
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
                depth_arr = np.frombuffer(mm, dtype=np.uint16, count=w * h, offset=offset).reshape((h, w)).copy()
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
        "depth_format": f"Y16 (scale={scale:.4f}, {'m/unit' if scale_is_meters else 'mm/unit'})",
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
        # Clock drift = how fast host wall-clock advances vs device hw-timestamp.
        # The previous implementation averaged *per-sample* ratios
        # (np.diff(host)/np.diff(dev)) — but IMU records are written in
        # batches where every record in one batch shares the same host_ts_ms
        # (the C++ writer stamps the whole batch with one wall-clock value,
        # see onImuSamples in nio_capture_session.cpp). Per-sample diffs are
        # therefore mostly 0 (filtered out) plus occasional scheduler/jitter
        # bursts that inflate the mean ratio far above real crystal drift —
        # observed values up to ~870000 ppm are physically impossible for
        # an oscillator. Use the end-to-end ratio instead: total wall-clock
        # duration divided by total device-timestamp duration. This is the
        # standard estimator for relative clock rate, robust to batch
        # stamping, and returns crystal-scale ppm (tens to a few hundred).
        host_dur_ms = float(host_ts_all[-1] - host_ts_all[0])
        dev_dur_ms = float(dev_ts_all[-1] - dev_ts_all[0]) / 1000.0
        if host_dur_ms > 0.0 and dev_dur_ms > 0.0:
            ratio = host_dur_ms / dev_dur_ms
            result["clock_drift_ppm"] = float((ratio - 1.0) * 1e6)
            result["clock_drift_ratio_mean"] = float(ratio)
            result["clock_drift_ratio_std"] = 0.0

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

        if "percentiles_mm" in s1 and "percentiles_mm" in s2:
            p1, p2 = s1["percentiles_mm"], s2["percentiles_mm"]
            pct_keys = sorted(set(p1.keys()) & set(p2.keys()))
            pct_comp = {}
            for k in pct_keys:
                pct_comp[k] = {
                    "d1_mm": float(p1[k]),
                    "d2_mm": float(p2[k]),
                    "diff_mm": float(abs(p1[k] - p2[k])),
                }
            comp["percentile_diff"] = pct_comp

        if "global_median_mm" in s1 and "global_median_mm" in s2:
            comp["median_diff_mm"] = float(abs(s1["global_median_mm"] - s2["global_median_mm"]))
            comp["d1_global_median_mm"] = float(s1["global_median_mm"])
            comp["d2_global_median_mm"] = float(s2["global_median_mm"])

        if "global_std_mm" in s1 and "global_std_mm" in s2:
            comp["d1_global_std_mm"] = float(s1["global_std_mm"])
            comp["d2_global_std_mm"] = float(s2["global_std_mm"])

        result["comparison"][f"{d1}_vs_{d2}"] = comp

    result["per_device_summary"] = {}
    for dev, s in depth_stats.items():
        li = all_device_data[dev].get("log_info", {})
        imu = all_device_data[dev].get("imu_stats", {})
        color = all_device_data[dev].get("color_stats", {})
        d2c = all_device_data[dev].get("d2c_fusion", {})
        summary = {"depth_resolution": s.get("resolution", "N/A"),
                    "depth_format": s.get("depth_format", "N/A"),
                    "color_resolution": color.get("color_resolution", li.get("d2c_resolution", "N/A")),
                    "color_format": li.get("color_format", "N/A"),
                    "d2c_mode": li.get("d2c_mode", "N/A"),
                    "depth_scale": s.get("depth_scale", "N/A"),
                    "depth_mean_mm": s.get("global_mean_mm", "N/A"),
                    "depth_median_mm": s.get("global_median_mm", "N/A"),
                    "depth_std_mm": s.get("global_std_mm", "N/A"),
                    "valid_ratio": s.get("avg_valid_ratio", "N/A"),
                    "temporal_std_mm": s.get("depth_temporal_std_mm", "N/A"),
                    "num_frames": s.get("num_frames", "N/A"),
                    "imu_accel_hz": imu.get("accel_rate_hz", "N/A"),
                    "imu_gyro_hz": imu.get("gyro_rate_hz", "N/A"),
                    "imu_total_samples": imu.get("total_samples", "N/A")}
        if isinstance(summary["valid_ratio"], float):
            summary["valid_ratio"] = f"{summary['valid_ratio']*100:.1f}%"
        if isinstance(summary["depth_scale"], float):
            summary["depth_scale"] = f"{summary['depth_scale']:.4f}"
        result["per_device_summary"][dev] = summary

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
    elif "YUYV" in color_source.upper():
        result["source_type"] = "YUYV"
        result["noise_attribution"] = "原始传感器噪声（YUYV直出），无MJPEG中间解压"
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
    sharpness_values = []
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
    if d2c_mode == "unknown" and device_type in ("335L", "336L"):
        d2c_mode = "SW"
    result["d2c_mode"] = d2c_mode
    result["fusion_method_note"] = (
        "SW D2C: depth 1280x800→color 1280x800 software reprojection"
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


def analyze_color_streams(h264_by_type, device_name, device_type, depth_stats, color_format="unknown"):
    result = {"device": device_name}
    color_path = h264_by_type.get("color")
    if not color_path or not os.path.exists(color_path):
        result["color_available"] = False
        return result
    result["color_available"] = True
    # Preserve the real upstream color format (parsed from the capture log by
    # parse_capture_log). Used downstream by analyze_color_sensor_quality to
    # attribute noise sources; previously this function hard-coded "335L/336L
    # -> MJPEG" which was wrong — 335L/336L prefer YUYV (see
    # COLOR_FMT_GEMINI335L336L in nio_ob_spec.hpp).
    result["color_format"] = color_format

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
                # Prefer the real color format parsed from the capture log
                # (YUYV / MJPG / NV12 / ...) over the legacy device_type
                # shortcut. Fall back to device_type only when the log value
                # is unavailable — this keeps AC1 (NV12) detection working
                # even when the log was not found.
                src = color_format if color_format and color_format != "unknown" else (
                    device_type if device_type in ("335L", "336L", "AC1") else "unknown"
                )
                result["sensor_quality"] = analyze_color_sensor_quality(frames, src)
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


def analyze_ir_stream_quality(h264_by_type, device_name, device_type):
    result = {"device": device_name, "ir_available": False, "ir_streams": {}}
    ir_paths = []
    for st in ["ir_left", "ir_right"]:
        p = h264_by_type.get(st)
        if p and os.path.exists(p):
            ir_paths.append((st, p))
    if not ir_paths:
        return result
    result["ir_available"] = True
    result["ir_tech_note"] = (
        "Orbbec active stereo IR: VCSEL speckle projector illuminates scene, "
        "left/right global-shutter IR cameras capture speckle pattern for stereo matching. "
        "336L adds IR pass filter to reject visible light."
        if device_type in ("335L", "336L")
        else "AC1 has no separate IR streams (LiDAR uses 940nm VCSEL+SPAD directly)"
    )
    if device_type == "336L":
        result["ir_filter_note"] = (
            "336L IR pass filter blocks visible light (<750nm), enhancing speckle SNR "
            "in outdoor/high-ambient-light conditions. Expected: higher IR contrast "
            "and fewer ambient-light artifacts vs 335L"
        )

    try:
        import cv2
        for st, p in ir_paths:
            probe = probe_h264_stream(p)
            tmp_dir = os.path.join("/tmp", f"nio_eval_ir_{device_name}_{st}")
            indices = [0, min(30, probe.get("nal_frame_count", 1) - 1)]
            frames = extract_h264_sample_frames(p, indices, tmp_dir)
            ir_info = {
                "resolution": f"{probe.get('width', '?')}x{probe.get('height', '?')}",
                "frame_count": probe.get("nal_frame_count", 0),
                "avg_bitrate_kbps": probe.get("avg_bitrate_kbps", 0),
            }
            if frames:
                sample_idx = list(frames.keys())[0]
                ir_img = frames[sample_idx]
                ir_gray = cv2.cvtColor(ir_img, cv2.COLOR_BGR2GRAY)
                h, w = ir_gray.shape
                lap = cv2.Laplacian(ir_gray, cv2.CV_64F)
                ir_info["sharpness"] = round(float(lap.var()), 2)
                ir_info["brightness_mean"] = round(float(ir_gray.mean()), 1)
                ir_info["brightness_std"] = round(float(ir_gray.std()), 1)
                ir_info["brightness_max"] = int(ir_gray.max())

                bright_mask = ir_gray > 200
                bright_pct = round(100.0 * np.count_nonzero(bright_mask) / ir_gray.size, 1)
                ir_info["saturated_pixel_pct"] = bright_pct

                center_roi = ir_gray[h//4:3*h//4, w//4:3*w//4]
                edge_roi_lst = [
                    ir_gray[:h//8, :], ir_gray[7*h//8:, :],
                    ir_gray[:, :w//8], ir_gray[:, 7*w//8:]
                ]
                edge_roi = np.concatenate([r.flatten() for r in edge_roi_lst if r.size > 0])
                if center_roi.size > 0 and edge_roi.size > 0:
                    center_mean = float(center_roi.mean())
                    edge_mean = float(edge_roi.mean())
                    ir_info["center_brightness"] = round(center_mean, 1)
                    ir_info["edge_brightness"] = round(edge_mean, 1)
                    ir_info["illumination_uniformity"] = round(edge_mean / (center_mean + 1e-6), 3)

                ksize = min(h, w) // 8
                if ksize >= 3 and ksize % 2 == 0:
                    ksize -= 1
                if ksize >= 3:
                    blurred = cv2.GaussianBlur(ir_gray, (ksize, ksize), 0)
                    high_freq = ir_gray.astype(float) - blurred.astype(float)
                    hf_energy = float(np.mean(high_freq ** 2))
                    ir_info["speckle_contrast"] = round(float(high_freq.std()), 2)
                    ir_info["speckle_energy"] = round(hf_energy, 1)
                    signal_power = float(np.mean(ir_gray.astype(float) ** 2))
                    ir_info["speckle_snr_db"] = round(10 * np.log10(signal_power / (hf_energy + 1e-10)), 2)

            result["ir_streams"][st] = ir_info

        both_keys = [k for k in result["ir_streams"] if k in ("ir_left", "ir_right")]
        if len(both_keys) == 2:
            il = result["ir_streams"]["ir_left"]
            ir = result["ir_streams"]["ir_right"]
            if "brightness_mean" in il and "brightness_mean" in ir:
                result["stereo_ir_brightness_diff"] = round(abs(il["brightness_mean"] - ir["brightness_mean"]), 1)
                result["stereo_ir_brightness_ratio"] = round(
                    min(il["brightness_mean"], ir["brightness_mean"]) /
                    (max(il["brightness_mean"], ir["brightness_mean"]) + 1e-6), 3
                )
            if "speckle_contrast" in il and "speckle_contrast" in ir:
                result["stereo_speckle_contrast_diff"] = round(
                    abs(il["speckle_contrast"] - ir["speckle_contrast"]), 2
                )
    except ImportError:
        result["note"] = "cv2 not available, IR quality analysis skipped"

    return result


def analyze_pcd_cloud(pcd_dir, device_name, device_type, max_files=30):
    result = {"device": device_name, "available": False}
    if pcd_dir is None or not os.path.isdir(pcd_dir):
        return result
    result["available"] = True
    pcd_files = sorted([f for f in os.listdir(pcd_dir) if f.endswith('.pcd')])
    result["total_pcd_files"] = len(pcd_files)
    if not pcd_files:
        return result

    spec = DEVICE_SPECS.get(device_type, {})
    sample_files = pcd_files[:max_files] if len(pcd_files) > max_files else pcd_files
    point_counts = []
    all_distances_m = []
    field_sets = set()

    for pcd_fn in sample_files:
        pcd_path = os.path.join(pcd_dir, pcd_fn)
        info = parse_pcd_header(pcd_path)
        if not info or info.get("points", 0) == 0:
            continue
        pts = info["points"]
        point_counts.append(pts)
        fields = info.get("fields", "")
        field_sets.add(fields)

        if "x" in fields:
            try:
                with open(pcd_path, 'rb') as f:
                    header_size = 0
                    for _ in range(20):
                        line = f.readline()
                        header_size += len(line)
                        if line.decode('ascii', errors='replace').strip().startswith('DATA'):
                            break
                    data_start = f.tell()
                    n_pts = pts
                    sizes = info.get("sizes", [])
                    stride = sum(sizes) if sizes else (4 * len(fields.split()))
                    stride = max(stride, 12)
                    f.seek(data_start)
                    sample_n = min(n_pts, 5000)
                    step = max(1, n_pts // sample_n)
                    dists = []
                    for i in range(0, n_pts, step):
                        f.seek(data_start + i * stride)
                        xyz_bytes = f.read(12)
                        if len(xyz_bytes) < 12:
                            break
                        x, y, z = struct.unpack('<fff', xyz_bytes)
                        d = math.sqrt(x*x + y*y + z*z)
                        if not math.isfinite(d) or d < 0.01 or d > 200.0:
                            continue
                        dists.append(d)
                    if dists:
                        all_distances_m.extend(dists)
            except Exception:
                pass

    if point_counts:
        result["avg_points"] = float(np.mean(point_counts))
        result["min_points"] = int(np.min(point_counts))
        result["max_points"] = int(np.max(point_counts))
        result["std_points"] = float(np.std(point_counts))
        result["point_count_stability"] = round(
            float(np.std(point_counts)) / (float(np.mean(point_counts)) + 1e-6), 4
        )

    if field_sets:
        result["pcd_field_layouts"] = list(field_sets)
        for fs in field_sets:
            if "intensity" in fs:
                result["has_intensity"] = True
            if "ring" in fs:
                result["has_ring"] = True
            if "timestamp" in fs:
                result["has_timestamp"] = True

    if all_distances_m:
        dist_arr = np.array(all_distances_m)
        dist_arr = dist_arr[np.isfinite(dist_arr)]
        if dist_arr.size > 0:
            result["distance_mean_m"] = round(float(np.mean(dist_arr)), 2)
            result["distance_median_m"] = round(float(np.median(dist_arr)), 2)
            result["distance_std_m"] = round(float(np.std(dist_arr)), 2)
            result["distance_min_m"] = round(float(np.min(dist_arr)), 2)
            result["distance_max_m"] = round(float(np.max(dist_arr)), 2)
            pcts = [5, 10, 25, 50, 75, 90, 95]
            vals = np.percentile(dist_arr, pcts)
            result["distance_percentiles_m"] = {f"p{p}": round(float(v), 2) for p, v in zip(pcts, vals)}

            range_min, range_max = spec.get("depth_range_m", (0.1, 70.0))
            in_range = np.sum((dist_arr >= range_min) & (dist_arr <= range_max))
            result["points_in_spec_range_pct"] = round(100.0 * in_range / len(dist_arr), 1)

            short_range = np.sum(dist_arr < 1.0)
            mid_range = np.sum((dist_arr >= 1.0) & (dist_arr < 5.0))
            far_range = np.sum(dist_arr >= 5.0)
            total = len(dist_arr)
            result["distance_bins"] = {
                "short_lt1m_pct": round(100.0 * short_range / total, 1),
                "mid_1to5m_pct": round(100.0 * mid_range / total, 1),
                "far_gt5m_pct": round(100.0 * far_range / total, 1),
            }

    if spec.get("pts_per_sec") and result.get("avg_points"):
        expected = spec["pts_per_sec"] / spec.get("depth_fps", 10)
        result["expected_pts_per_frame"] = int(expected)
        result["pts_per_frame_vs_spec_pct"] = round(
            100.0 * result["avg_points"] / (expected + 1e-6), 1
        )

    if pcd_files:
        timestamps = []
        for fn in pcd_files:
            m = re.search(r'(\d{13,})', fn)
            if m:
                timestamps.append(int(m.group(1)))
        if len(timestamps) >= 2:
            ts_sorted = sorted(timestamps)
            duration_s = (ts_sorted[-1] - ts_sorted[0]) / 1e6
            if duration_s > 0:
                result["pcd_fps"] = len(pcd_files) / duration_s
                result["pcd_duration_s"] = round(duration_s, 1)

    return result


def analyze_depth_accuracy_by_distance(depth_info, device_name, num_bins=8):
    result = {"device": device_name, "available": False}
    if depth_info is None or depth_info.get("num_frames", 0) < 3:
        return result
    result["available"] = True
    frames = depth_info["frames"]
    n = len(frames)
    scale = depth_info["scale"]
    w, h = depth_info["width"], depth_info["height"]
    scale_is_meters = scale < 1.0
    convert = scale * 1000.0 if scale_is_meters else scale

    sample_frames = frames[:min(n, 30)]

    valid_all = [f[f > 0].astype(np.float64).flatten() * convert for f in sample_frames if np.any(f > 0)]
    if not valid_all:
        result["available"] = False
        return result

    all_depth_mm = np.concatenate(valid_all)
    all_depth_m = all_depth_mm / 1000.0

    if len(all_depth_m) < 100:
        result["available"] = False
        return result

    min_d = max(float(np.percentile(all_depth_m, 1)), 0.1)
    max_d = min(float(np.percentile(all_depth_m, 99)), 20.0)
    if max_d <= min_d + 0.1:
        result["available"] = False
        return result

    bin_edges = np.linspace(min_d, max_d, num_bins + 1)
    bin_labels = []
    for i in range(num_bins):
        lo = bin_edges[i]
        hi = bin_edges[i + 1]
        mask = (all_depth_m >= lo) & (all_depth_m < hi)
        count = int(np.sum(mask))
        if count < 10:
            bin_labels.append(None)
            continue
        bin_vals_mm = all_depth_mm[mask]
        bin_mean_m = round(lo + (hi - lo) / 2, 2)
        depth_std_mm = float(np.std(bin_vals_mm))
        depth_mean_mm = float(np.mean(bin_vals_mm))
        if bin_mean_m > 0:
            relative_pct = (depth_std_mm / (bin_mean_m * 1000)) * 100.0
        else:
            relative_pct = 0

        spec = DEVICE_SPECS.get(device_name, {})
        if device_name in ("335L", "336L"):
            spec_pct = spec.get("depth_accuracy_pct")
        elif device_name == "AC1":
            spec_cm = spec.get("depth_accuracy_cm", (1, 5))
            expected_sigma_mm = (spec_cm[0] * 10) * bin_mean_m / 5.0 if bin_mean_m <= 5 else (spec_cm[1] * 10)
            relative_pct_spec = (expected_sigma_mm / (bin_mean_m * 1000)) * 100.0
            spec_pct = round(relative_pct_spec, 2)
        else:
            spec_pct = None

        entry = {
            "distance_m": bin_mean_m,
            "num_points": count,
            "depth_std_mm": round(depth_std_mm, 1),
            "relative_precision_pct": round(relative_pct, 2),
            "spec_accuracy_pct": spec_pct,
            "passes_spec": relative_pct <= spec_pct if spec_pct is not None else None,
        }
        bin_labels.append(entry)

    result["distance_bins"] = [b for b in bin_labels if b is not None]

    if n >= 10:
        f0 = frames[0].astype(np.float64) * convert
        f1 = frames[min(5, n - 1)].astype(np.float64) * convert
        valid = (frames[0] > 0) & (frames[min(5, n - 1)] > 0)
        if np.any(valid):
            diff = np.abs(f0[valid] - f1[valid])
            depth_vals = f0[valid]
            permille_bins = {}
            for lo_m, hi_m in [(0.3, 1.0), (1.0, 3.0), (3.0, 5.0), (5.0, 10.0), (10.0, 20.0)]:
                mask = (depth_vals >= lo_m * 1000) & (depth_vals < hi_m * 1000)
                if np.sum(mask) > 100:
                    d = diff[mask]
                    permille_bins[f"{lo_m}-{hi_m}m"] = {
                        "temporal_noise_median_mm": round(float(np.median(d)), 2),
                        "temporal_noise_p95_mm": round(float(np.percentile(d, 95)), 2),
                    }
            if permille_bins:
                result["temporal_noise_by_distance"] = permille_bins

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
            checks.append(("Color resolution", color_info.get("color_resolution", "?"), "1280x800", color_info.get("color_resolution") == "1280x800"))
        elif device_type == "AC1":
            checks.append(("Color resolution", color_info.get("color_resolution", "?"), "1920x1080", color_info.get("color_resolution") == "1920x1080"))
        if color_info.get("color_issue") == "SEVERE_UNDEREXPOSURE":
            checks.append(("Color brightness", f"mean={q.get('brightness_mean',0):.1f}", ">15", False))
        elif q.get("brightness_mean", 0) > 5:
            checks.append(("Color brightness", f"mean={q.get('brightness_mean',0):.1f}", ">5", True))
    checks.append(("Color frame count", str(color_info.get("color_frame_count", 0)), ">0", color_info.get("color_frame_count", 0) > 0))
    return checks


def assess_d2c_fusion(d2c_info, device_type=""):
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
        # 8x8 block-boundary p95 measures H.264 macroblock edge artifacts.
        # AC1 HW D2C upsamples the 96x288 LiDAR depth to 1920x1080 (~20x
        # nearest-neighbor), so the fused frame already carries 8x8-aligned
        # block steps before H.264 — that is an intrinsic upsampling trait,
        # not a compression defect (see tech comparison note). Apply a
        # relaxed threshold for AC1; keep "<50" for SW D2C devices whose
        # depth arrives at color resolution (no upsampling).
        p95 = blk.get("boundary_p95", 999)
        if device_type == "AC1":
            checks.append(("8x8 block boundary score", f"p95={p95:.1f}", "<200 (AC1 ~20x upsampling)", p95 < 200))
        else:
            checks.append(("8x8 block boundary score", f"p95={p95:.1f}", "<50 (H.264 normal)", p95 < 50))
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
            sizes = []
            types = []
            for hl in header_lines:
                if hl.startswith('POINTS'):
                    points = int(hl.split()[1])
                if hl.startswith('FIELDS'):
                    fields = hl.split(None, 1)[1] if len(hl.split()) > 1 else ""
                if hl.startswith('SIZE'):
                    sizes = [int(x) for x in hl.split()[1:]]
                if hl.startswith('TYPE'):
                    types = hl.split()[1:]
            return {"points": points, "fields": fields, "sizes": sizes, "types": types}
    except Exception:
        return None


def parse_capture_log(log_path, device_path=None):
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

    if device_path:
        dev_name = os.path.basename(device_path.rstrip('/'))
    else:
        dev_name = None

    if dev_name:
        pat_fusion = re.compile(r'D2C Fusion enabled:\s+(\S+).*?mode=(\w+).*?output=\S*' + re.escape(dev_name))
        m = pat_fusion.search(content)
        if m:
            result["d2c_resolution"] = m.group(1)
            result["d2c_mode"] = m.group(2)
        pat_color = re.compile(r'Color output:.*?' + re.escape(dev_name) + r'.*?fmt=(\S+)')
        m = pat_color.search(content)
        if m:
            result["color_format"] = m.group(1)
    if "d2c_mode" not in result:
        fusion_match = re.search(r'D2C Fusion enabled:\s+(\S+).*?mode=(\w+)', content)
        if fusion_match:
            result["d2c_resolution"] = fusion_match.group(1)
            result["d2c_mode"] = fusion_match.group(2)
    if "color_format" not in result:
        color_fmt_match = re.search(r'Color output:.*?fmt=(\S+)', content)
        if color_fmt_match:
            result["color_format"] = color_fmt_match.group(1)

    alpha_match = re.search(r'alpha=([\d.]+)', content)
    if alpha_match:
        result["alpha"] = float(alpha_match.group(1))

    depth_range_match = re.search(r'depthRange=([\d.]+)m-([\d.]+)m', content)
    if depth_range_match:
        result["depth_min_m"] = float(depth_range_match.group(1))
        result["depth_max_m"] = float(depth_range_match.group(2))

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


def assess_temporal_alignment(ta, dev_types=None):
    # Thresholds below were originally written for the 335L/336L multi-device
    # setup (USB frames arriving near-simultaneously, low single-digit ppm
    # drift). They are too strict for some vendor/topology combinations:
    #
    #  - Depth start offset: comparing the first-frame device-timestamp of two
    #    *independent* USB streams — the start moment is governed by USB
    #    enumeration order and per-device depth-stream ramp-up, which can
    #    differ by several seconds for two RoboSense AC1 units (the depth
    #    stream only starts after LiDAR init completes). 1 s covers a single
    #    Orbbec; a multi-AC1 array needs ~5 s.
    #  - Clock drift diff: the per-device ppm is computed end-to-end on the
    #    IMU timestamp ratio (see analyze_imu). RoboSense's imu->timestamp is a
    #    host-clock-derived scalar from the decoder, so the "drift" between two
    #    AC1s measures decoder/host scheduling jitter, not a hardware crystal —
    #    the original <500 ppm (real-crystal-class) does not apply.
    if dev_types is None:
        dev_types = []
    is_ac1_only = bool(dev_types) and all(dt == "AC1" for dt in dev_types)
    depth_start_max_ms = 5000 if is_ac1_only else 1000
    drift_diff_max_ppm = 2000 if is_ac1_only else 500

    checks = []
    if not ta:
        return checks
    if "imu_start_offset_ms" in ta:
        checks.append(("IMU start offset", f"{ta['imu_start_offset_ms']:.1f} ms", "<1000 ms", ta["imu_start_offset_ms"] < 1000))
    if "imu_overlap_s" in ta:
        checks.append(("IMU time overlap", f"{ta['imu_overlap_s']:.1f} s", ">5 s", ta["imu_overlap_s"] > 5))
    if "depth_start_offset_ms" in ta:
        checks.append(("Depth start offset", f"{ta['depth_start_offset_ms']:.1f} ms", f"<{depth_start_max_ms} ms", ta["depth_start_offset_ms"] < depth_start_max_ms))
    if "clock_drift_diff_ppm" in ta:
        checks.append(("Clock drift diff", f"{ta['clock_drift_diff_ppm']:.1f} ppm", f"<{drift_diff_max_ppm} ppm", ta["clock_drift_diff_ppm"] < drift_diff_max_ppm))
    return checks


def _fmt_val(v, fmt=""):
    if v is None:
        return "N/A"
    if isinstance(v, float):
        if fmt:
            return fmt.format(v)
        return f"{v:.2f}"
    return str(v)


# ---------------------------------------------------------------------------
# Per-section test documentation (说明 / 方法 / 意义)
# ---------------------------------------------------------------------------
# Each entry has three short paragraphs (purpose / method / significance)
# rendered as a small blockquote right after the section heading. Keeping
# them in one place decouples the "what is this test" prose from the metric-
# emitting code below, and makes the docs easy to audit/diff.
SECTION_TEST_NOTES = {
    1: (
        "汇总本次采集会话的基础元数据：Git 提交、D2C 模式、融合分辨率、α 系数、深度区间、彩色格式与采集活跃时长。",
        "解析 dynamic_algo_cam_log*.log 中各设备写入的 d2c_mode / alpha / depth_min_m / depth_max_m / color_format / capture_active_s 等字段；缺失字段以 N/A 标注。",
        "回答“这次采集是在哪个代码版本、什么参数下做的”。若回归，可一眼定位采集上下文是否发生了变化。",
    ),
    2: (
        "对每个设备的 depth_raw (.raw) 文件做基本质量度量：分辨率、帧数、有效像素比、深度统计分布与时间稳定性。",
        "按 NIO_DEPTH_RAW 头部 (magic/width/height/bpp/scale/frame_size/start_ts) 解析整个文件，逐帧以 uint16>0 判定有效像素，并计算全局均值 / 中位数 / 标准差 / 百分位以及首末帧逐像素差。",
        "深度是下游感知的核心信号；有效像素比过低或分布异常意味着深度流本身不可用，不应进入融合或下游算法。",
    ),
    3: (
        "对 IMU .txt 文件度量采样率、噪声与时钟漂移：三轴加速度 / 角速度、加速度幅值、陀螺噪声 RMS、温度可用性、批处理频率。",
        "逐行解析 host_ts_ms,type,device_ts_us,x,y,z,temperature；按 ACCEL / GYRO 分组统计采样率与抖动；以首末两个端点的 host_ts 与 dev_ts 比率估算 clock_drift_ppm（端点法而非逐批次比率，详见 analyze_imu 内部注释）。",
        "回答“IMU 是否按预期频率到达、是否存在静止噪声异常、时钟是否有显著漂移”。IMU 是 SLAM、VIO、标定的间接输入。",
    ),
    4: (
        "概览 PCD 点云文件数、平均点数与可用率。",
        "枚举 *_pcd_* 目录下的 PCD 文件，按 ASCII PCD header 读取点计数；对 AC1 输出的 PointXYZIRT 列点云写入有效 / 无效比例判断。",
        "AC1 深度是由 LiDAR 点云合成的；点云是插值基坑前最不可逆的信息载体，PCD 帧率 / 点数直接决定后续 3D 处理可用性。",
    ),
    5: (
        "解析 intrinsic .json，验证 fx / fy / cx / cy / width / height 与设备预期是否一致，并曝光内参字段供后续校正使用。",
        "json.load 后提取深度与彩色两套内参；缺失或异常者会在表中以 N/A 标注。",
        "D2C / SLAM / 标定都依赖内参；内参崩溃、缺失或与期望不一致是 D2C 错位与深度-彩色不对齐最常见的根因。",
    ),
    6: (
        "对比各设备的总数据量以及彩色 / 深度 / 融合 / IMU / PCD 各分项的文件大小。",
        "对每个分项目录递归统计文件大小 (_dir_size_mb / sum)，并按 MB 列表呈现。",
        "验证采集确实写入文件（未缓存丢失），同时检验写盘速度是否能跟上数据率；任何一项为 0 通常意味着该流未启动或异常退出。",
    ),
    7: (
        "跨多设备度量启动偏移 (start_offset)、重叠时长 (overlap) 与时钟漂移差 (clock_drift_diff)。",
        "对 IMU / depth 两种源分别取所有设备的首帧 ts 求极差；IMU 的 clock_drift_ppm 取端点法；阈值对同构 AC1 阵列与 335L / 336L 等其它部署类型区分对待（详见 assess_temporal_alignment 内注释）。",
        "多设备融合、多传感器交叉、SLAM 都要求时钟在同一域；偏移过大则跨设备对齐误差不可接受，会污染下游融合结果。",
    ),
    8: (
        "在深度全局中位数 / 有效像素比 / 分布百分位等维度上对两台设备做 1:1 差异对比。",
        "对每个深度帧同名字段作差；用对比表 / 串行化文本呈现；对比不修原始数据，仅呈现差异。",
        "同一场景两台设备理论上应一致；意外差异（如 95 分位差远大于中位差）通常意味着标定出错、视角不同或某台设备故障。",
    ),
    9: (
        "把 9.x 各节点的指标对所有设备做 PASS / FAIL 汇总，作为总体采集质量门控。",
        "把前述各 assess_* 函数返回的 (检查项、实际值、期望值、PASS/FAIL) 拉平集中成一张表；阈值差异逻辑已在各 assess 函数内部按设备类型判断。",
        "作为采集门控；任意一项 FAIL 都需关注；多台设备多项 FAIL 通常代表采集本身未成功。",
    ),
    10: (
        "结构光 (335L / 336L) 与 LiDAR TOF (AC1) 两条技术路线的硬参数 / 软参数结构化对比表。",
        "表中各行直接引用既有代码常量与文档结论，不依赖本次采集刚出来的数据，仅作为观众背景资料展示。",
        "为读者提供背景，解释“为什么 AC1 上采样会导致块状伪影”、“为什么 AC1 的 depthScale=5.0 而 OB 的为 0.001”等技术路线带来的固有差异。",
    ),
    11: (
        "按距离分桶 (0~1m、1~2m、…) 统计深度帧中有效像素的均值与稳定性，评估远 / 近场精度。",
        "在 analyze_depth_accuracy_by_distance 中保存每帧有效像素并按 |坐标| 落桶；统计每桶的均值与标准差。",
        "实际感知的关键在近场（0~3m）；远场衰减与零星毛刺仅在分桶后才能暴露“远场测距逐渐退化”这一典型 LiDAR 现象。",
    ),
    12: (
        "对结构光 IR (335L / 336L) 散斑 IR-Left / IR-Right 视频流做锐度 / 亮度 / 帧间一致性 / 散斑可见性评估。",
        "从 IR H.264 抽帧 → 灰度 → 计算 Laplacian var 锐度、静止均方差、帧间 SSIM。",
        "IR 散斑是结构光深度的源；散斑不可见、IR 视频黑场或锐度骤降通常对应深度流也异常，可作为深度异常的先验指示。",
    ),
    13: (
        "分析 PCD 点云的密度 / 极角-方位角分布 / 距离直方图与有效点占比，把点云 3D 分布特征化。",
        "逐 PCD 文件读取点云，对 xyz 计算 ACE 分布、距离频率直方图与 ring 分布。",
        "点云分布直接反映 LiDAR 出点状况与盲区；某一方位角密度骤降通常意味着点云在该方向被遮挡或本帧被剔除。",
    ),
    14: (
        "由脚本自动汇总“关键发现”列表与优先级建议，作为采集现场快速反馈。",
        "rule-based 复盘前述各项 PASS/FAIL 与异常阈值，写入建议列表；不修改采集数据本身。",
        "把 9 节的 PASS/FAIL 转成“可直接落地”的条目化行动列表，便于研究者快速决定工单是否通过。",
    ),
    15: (
        "附录：列出本会话每台设备生成的关键原始数据文件路径。",
        "由 _files 字段读取并展示 raw / imu / pcd_dir / h264_by_type / log / intrinsic 的具体文件路径。",
        "供读者手动追本溯源，或后续用其它工具重读原始数据。",
    ),
    16: (
        "对彩色与红外 H.264 视频流做编码层面与传感器成像层面的双重质量评估：参数 (Profile/Level/码率/GOP)、锐度、熵、亮度、色彩、帧间 SSIM、暗区噪声、阴影 / 高光裁切、噪声归因。",
        "用 ffprobe 解析 NAL 单元分布；用 ffmpeg 抽帧 → cv2/NumPy 计算 Laplacian var 锐度、灰度 / HSV 分布、帧间 SSIM / PSNR；区分 MJPEG 解压伪影与原始传感器噪声。",
        "彩色是 D2C 输出 / 后续感知输入；H.264 编码 / 传感器双重缺陷都会影响彩色质量，故需双维度评估避免误判编码或传感器的责任。",
    ),
    17: (
        "对 D2C 深度-彩色融合流做编码参数、融合分辨率、帧数、SSIM / PSNR 与彩色帧的对比、8x8 块状伪影评估。",
        "用 ffprobe / ffmpeg 抽 fused 与 color 同索引帧 → 计算 SSIM / PSNR / blockiness；按 device_type 区分阈值（详见 assess_d2c_fusion 内注释）。",
        "D2C 融合是最终感知的输出之一；融合错位、模糊或块状伪影会直接传递到下游，是采集门控的关键项。",
    ),
    18: (
        "对多台具备 D2C 流的设备做跨设备融合方式对比，比较 D2C 模式、融合分辨率、融合方法与帧数。",
        "对有 fused_available 的每台设备构造一行，并按 d2c_mode / fused_resolution / fusion_method_note 列拉表。",
        "揭示结构光 (SW D2C) 与 LiDAR TOF (HW D2C + 上采样) 两种机制在采集行为上的差异，给读者跨设备的直觉参照。",
    ),
}


def _emit_section_notes(lines, section_id):
    """Append the section's (test purpose / method / significance) notes."""
    notes = SECTION_TEST_NOTES.get(section_id)
    if not notes:
        return
    purpose, method_, significance = notes
    lines.append("")
    lines.append("> **测试说明**：" + purpose)
    lines.append(">")
    lines.append("> **测试方法**：" + method_)
    lines.append(">")
    lines.append("> **测试意义**：" + significance)
    lines.append("")


def _report_asset_dir(output_path):
    """Return (assets_abs_dir, assets_rel_dir) for storing PNG figures next
    to the Markdown report. Asset dir name is `<report_stem>_assets/`."""
    report_dir = os.path.dirname(os.path.abspath(output_path)) or "."
    stem = os.path.splitext(os.path.basename(output_path))[0] or "evaluation_report"
    assets_abs = os.path.join(report_dir, stem + "_assets")
    os.makedirs(assets_abs, exist_ok=True)
    # Relative path from the report file (report and assets share the parent).
    return assets_abs, stem + "_assets"


def _save_fig_to_report(fig, assets_abs_dir, assets_rel_dir, fname, lines, caption):
    """Save ``fig`` to <assets_abs_dir>/<fname>.png, close it, and append a
    Markdown image reference (relative path) + caption to ``lines``.

    Returns True if the image was saved, False if matplotlib is unavailable.
    """
    if not _MPL_AVAILABLE or fig is None:
        return False
    out_path = os.path.join(assets_abs_dir, fname + ".png")
    fig.savefig(out_path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    rel = os.path.join(assets_rel_dir, fname + ".png")
    lines.append("")
    lines.append(f"![{caption}]({rel})")
    lines.append("")
    lines.append(f"*图：{caption}*")
    lines.append("")
    return True


def _make_depth_heatmap_figure(dev_type, depth_stats):
    """Return a matplotlib Figure of the first depth frame as a jet heatmap,
    or None if matplotlib is unavailable or no frame is available."""
    if not _MPL_AVAILABLE:
        return None
    frame0 = depth_stats.get("_raw_frame0")
    scale = depth_stats.get("_raw_scale")
    w = depth_stats.get("_raw_w")
    h = depth_stats.get("_raw_h")
    if frame0 is None or w is None or h is None:
        return None
    arr = np.array(frame0, dtype=np.float64)
    # Convert raw -> mm using the same convention as analyze_depth.
    if scale is not None and scale < 1.0:
        arr = arr * scale * 1000.0  # m/unit -> mm
    elif scale is not None:
        arr = arr * scale  # already mm/unit
    # Mask invalid (raw==0) pixels.
    masked = np.ma.masked_where(arr <= 0, arr)
    fig, ax = plt.subplots(figsize=(6, 5))
    im = ax.imshow(masked, cmap="jet", aspect="auto")
    ax.set_title(f"{dev_type} 首帧深度 (raw→mm, 0 视为无效)")
    ax.set_xlabel("x (列)")
    ax.set_ylabel("y (行)")
    cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("深度 (mm)")
    fig.tight_layout()
    return fig


def _has_imu_records(imu_stats):
    return bool(imu_stats and imu_stats.get("_records"))


def _make_imu_figure(dev_type, imu_stats):
    """Return a matplotlib Figure with two stacked subplots:
    1) Accel/gyro 3-axis time series.
    2) Power spectral density of accel magnitude (and gyro magnitude).
    Returns None if matplotlib is unavailable or no records exist.
    """
    if not _MPL_AVAILABLE or not _has_imu_records(imu_stats):
        return None
    records = imu_stats["_records"]
    if len(records) < 4:
        return None

    accel = [r for r in records if r["type"] == "ACCEL"]
    gyro = [r for r in records if r["type"] == "GYRO"]

    if not accel and not gyro:
        return None
    n_subplots = 2
    fig, axes = plt.subplots(n_subplots, 1, figsize=(10, 7))

    def _extract(rs):
        host = np.array([r["host_ts_ms"] for r in rs], dtype=np.int64)
        host = host - host[0]
        ax = np.array([r["x"] for r in rs], dtype=np.float64)
        ay = np.array([r["y"] for r in rs], dtype=np.float64)
        az = np.array([r["z"] for r in rs], dtype=np.float64)
        mag = np.sqrt(ax * ax + ay * ay + az * az)
        return host, ax, ay, az, mag

    if accel:
        t_ms, ax_, ay_, az_, amag = _extract(accel)
        ax_top = axes[0]
        ax_top.plot(t_ms / 1000.0, ax_, label="ax", linewidth=0.8)
        ax_top.plot(t_ms / 1000.0, ay_, label="ay", linewidth=0.8)
        ax_top.plot(t_ms / 1000.0, az_, label="az", linewidth=0.8)
        ax_top.set_title(f"{dev_type} ACCEL 三轴时间序列")
        ax_top.set_xlabel("时间 (s)")
        ax_top.set_ylabel("加速度 (g)")
        ax_top.grid(True, alpha=0.3)
        ax_top.legend(fontsize=8, loc="upper right")
    else:
        axes[0].text(0.5, 0.5, "无 ACCEL 数据", ha="center", va="center", transform=axes[0].transAxes)
        axes[0].set_title(f"{dev_type} 无 ACCEL 数据")

    # Spectrum: PSD of accel magnitude via np.fft
    ax_spec = axes[1]
    if accel and len(accel) >= 4:
        t_ms, _, _, _, amag = _extract(accel)
        # Estimate sampling rate from median inter-sample interval.
        if len(t_ms) >= 2:
            dt_ms = float(np.median(np.diff(t_ms))) if len(t_ms) > 1 else 10.0
            fs = 1e3 / dt_ms if dt_ms > 0 else 100.0
            # Detrend + zero-mean
            x = amag - np.mean(amag)
            n = len(x)
            xf = np.fft.rfftfreq(n, d=1.0 / fs)
            spectrum = np.abs(np.fft.rfft(x)) ** 2
            if len(xf) > 1:
                ax_spec.semilogy(xf[1:], spectrum[1:], label="|ACCEL|² PSD", linewidth=0.8)
            ax_spec.set_title(f"{dev_type} ACCEL 幅值 PSD (估计 fs={fs:.0f} Hz)")
            ax_spec.set_xlabel("频率 (Hz)")
            ax_spec.set_ylabel("功率")
            ax_spec.grid(True, alpha=0.3)
            ax_spec.legend(fontsize=8)
        else:
            ax_spec.text(0.5, 0.5, "样本不足", ha="center", va="center", transform=ax_spec.transAxes)
    else:
        ax_spec.text(0.5, 0.5, "无 ACCEL PSD 数据", ha="center", va="center", transform=ax_spec.transAxes)
        ax_spec.set_title("无 ACCEL PSD 数据")

    fig.tight_layout()
    return fig


def _diff_ssim_str():
    return ""


def _make_color_d2c_pair_figure(dev_type, devices_data, dt):
    """Return a matplotlib Figure comparing color and D2C fused frames side by
    side, with a difference heatmap and 8x8 block-boundary overlays on the
    fused frame. Returns None if matplotlib/cv2 unavailable or no frames."""
    if not _MPL_AVAILABLE or not _CV2_AVAILABLE:
        return None
    d2c = devices_data.get(dt, {}).get("d2c_fusion", {}) or {}
    if not d2c or not d2c.get("fused_available"):
        return None
    files = devices_data.get(dt, {}).get("_files", {}) or {}
    h264_by_type = files.get("h264_by_type", {}) or {}
    color_path = h264_by_type.get("color")
    fused_path = h264_by_type.get("d2c_fused")
    if not color_path or not fused_path:
        return None
    if not os.path.exists(color_path) or not os.path.exists(fused_path):
        return None
    tmp_dir = os.path.join("/tmp", f"nio_eval_fig_{dt}")
    try:
        color_frames = extract_h264_sample_frames(color_path, [0], tmp_dir + "_c_fig")
        fused_frames = extract_h264_sample_frames(fused_path, [0], tmp_dir + "_f_fig")
    except Exception:
        return None
    if not color_frames or not fused_frames:
        return None
    sample_idx = list(color_frames.keys())[0]
    if sample_idx not in fused_frames:
        sample_idx = list(fused_frames.keys())[0]
    if sample_idx not in color_frames or sample_idx not in fused_frames:
        return None
    color_img = color_frames[sample_idx]
    fused_img = fused_frames[sample_idx]
    # Convert to grayscale for absolute-diff heatmap.
    color_gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    fused_gray = cv2.cvtColor(fused_img, cv2.COLOR_BGR2GRAY)
    # Match shapes — fused resolution should equal color resolution since
    # the fusion task blends depth over a color-resolution buffer.
    if color_gray.shape != fused_gray.shape:
        return None
    diff = cv2.absdiff(color_gray, fused_gray)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    axes[0].imshow(cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB))
    axes[0].set_title(f"{dev_type} 彩色 (帧 {sample_idx})")
    axes[0].axis("off")

    # Fused frame + 8x8 block-boundary overlays (edges only, block boundary score highlighted).
    axes[1].imshow(cv2.cvtColor(fused_img, cv2.COLOR_BGR2RGB))
    # Draw 8x8 grid lines on fused image.
    h, w = fused_gray.shape
    for y in range(0, h, 8):
        axes[1].axhline(y, color="black", linewidth=0.3, alpha=0.4)
    for x in range(0, w, 8):
        axes[1].axvline(x, color="black", linewidth=0.3, alpha=0.4)
    axes[1].set_title(f"{dev_type} D2C 融合帧（叠加 8x8 块网格）")
    axes[1].axis("off")

    im = axes[2].imshow(diff, cmap="hot", aspect="auto")
    axes[2].set_title("彩色-融合 差异热图(|Δ|)")
    axes[2].axis("off")
    fig.colorbar(im, ax=axes[2], fraction=0.046, pad=0.04)

    # Caption under figure with quantitative diff stats to make 'block artifacts'
    # visible to a human reader without p95 alone.
    diff_flat = diff.astype(np.float64).flatten()
    diff_mean = float(np.mean(diff_flat))
    diff_p95 = float(np.percentile(diff_flat, 95))
    fig.suptitle(
        f"彩色 vs 融合 (帧 {sample_idx})——差异均值={diff_mean:.1f}, p95={diff_p95:.1f}；"
        f"块网格叠加显示 8x8 块边界跳变点",
        fontsize=10,
    )
    fig.tight_layout()
    return fig


def generate_report(devices_data, alignment, cross_depth, output_path):
    now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    dev_types = sorted(devices_data.keys())
    dev_labels = {"335L": "Orbbec Gemini 335L", "336L": "Orbbec Gemini 336L", "AC1": "RoboSense AC1"}

    # Asset directory for figures — created once, shared across sections.
    assets_abs, assets_rel = _report_asset_dir(output_path)
    if not _MPL_AVAILABLE:
        # Note: the report itself is fully useful without figures. Mention the
        # limitation once at the top so readers know which images are skipped.
        pass

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
    lines.append("10. 设备技术规格对比 (Structured Light vs LiDAR TOF)")
    lines.append("11. 不同距离精度评估")
    lines.append("12. IR 红外散斑流质量分析")
    lines.append("13. PCD 点云密度与分布分析")
    lines.append("14. 关键发现与建议")
    lines.append("15. 附录：原始数据路径")
    lines.append("16. 彩色与红外视频流分析")
    lines.append("17. D2C 深度-彩色融合分析")
    lines.append("18. 跨设备融合方法对比")
    lines.append("")

    lines.append("## 1. 采集会话概览")
    lines.append("")
    _emit_section_notes(lines, 1)
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
    _emit_section_notes(lines, 2)
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

    # 深度首帧热图（per 设备）
    if _MPL_AVAILABLE:
        lines.append("### 2.N 深度首帧热图（原始数据可视化）")
        lines.append("")
        lines.append("下面为每台设备 depth_raw 中的首帧，按 raw→mm 解码后用 jet colormap 渲染；有效像素 (raw>0) 着色，无效像素 (raw=0) 标为透明。")
        lines.append("")
        any_depth_fig = False
        for dt in dev_types:
            ds = devices_data[dt].get("depth_stats", {})
            if ds and ds.get("_raw_frame0") is not None:
                fig = _make_depth_heatmap_figure(dt, ds)
                ok = _save_fig_to_report(fig, assets_abs, assets_rel,
                                         f"depth_heatmap_{dt}", lines,
                                         f"{dt} 深度首帧热图")
                any_depth_fig = any_depth_fig or ok
        if not any_depth_fig:
            lines.append("_（无可绘制的深度帧数据）_")
            lines.append("")
    else:
        lines.append("> *（matplotlib 不可用——跳过深度首帧热图生成。）*")
        lines.append("")

    lines.append("## 3. IMU 数据分析")
    lines.append("")
    _emit_section_notes(lines, 3)
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

    # IMU 时间序列 + 幅值 PSD（per 设备）
    if _MPL_AVAILABLE:
        any_imu = any(_has_imu_records(devices_data[dt].get("imu_stats", {})) for dt in dev_types)
        if any_imu:
            lines.append("### 3.N IMU 时间序列与频谱（原始数据可视化）")
            lines.append("")
            lines.append("上子图：ACCEL 三轴时间序列 (host_ts 相对值)。下子图：ACCEL 幅值的功率谱密度 (np.fft 估计，采样率由样本中位间隔倒推)。")
            lines.append("")
            any_imu_fig = False
            for dt in dev_types:
                imu_stats = devices_data[dt].get("imu_stats", {})
                if _has_imu_records(imu_stats):
                    fig = _make_imu_figure(dt, imu_stats)
                    ok = _save_fig_to_report(fig, assets_abs, assets_rel,
                                             f"imu_{dt}", lines,
                                             f"{dt} IMU ACCEL 时间序列与频谱")
                    any_imu_fig = any_imu_fig or ok
            if not any_imu_fig:
                lines.append("_（无足够 ACCEL 样本生成图像）_")
                lines.append("")
        else:
            lines.append("")
    else:
        lines.append("> *（matplotlib 不可用——跳过 IMU 时间序列与频谱图生成。）*")
        lines.append("")

    pcd_devs = [dt for dt in dev_types if devices_data[dt].get("pcd_stats", {}).get("available")]
    lines.append("## 4. 点云 (PCD) 分析")
    lines.append("")
    _emit_section_notes(lines, 4)
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
    _emit_section_notes(lines, 5)
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
    _emit_section_notes(lines, 6)
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
    _emit_section_notes(lines, 7)
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
    _emit_section_notes(lines, 8)
    if cross_depth and cross_depth.get("comparison"):
        lines.append("因安装位置差异，各设备观测视角不同，深度分布不可直接等同对比。")
        lines.append("以下分析展示各设备深度分布差异，用于确认设备间是否存在系统性偏差。")
        lines.append("")

        if cross_depth.get("per_device_summary"):
            lines.append("### 8.1 设备关键参数对比总表")
            lines.append("")
            summary_devs = list(cross_depth["per_device_summary"].keys())
            header = "| 参数 |" + "|".join([f" {dev_labels.get(d, d)} " for d in summary_devs]) + "|"
            sep = "|------|" + "|".join(["------" for _ in summary_devs]) + "|"
            lines.append(header)
            lines.append(sep)
            row_keys = [
                ("depth_resolution", "深度分辨率"),
                ("depth_format", "深度格式"),
                ("depth_scale", "depthScale"),
                ("depth_mean_mm", "深度均值 (mm)"),
                ("depth_median_mm", "深度中位数 (mm)"),
                ("depth_std_mm", "深度标准差 (mm)"),
                ("valid_ratio", "有效像素比"),
                ("temporal_std_mm", "时间稳定性-std (mm)"),
                ("num_frames", "总帧数"),
                ("color_resolution", "彩色分辨率"),
                ("color_format", "彩色格式"),
                ("d2c_mode", "D2C 模式"),
                ("imu_accel_hz", "IMU 加速度计 (Hz)"),
                ("imu_gyro_hz", "IMU 陀螺仪 (Hz)"),
                ("imu_total_samples", "IMU 总采样数"),
            ]
            for rk, rl in row_keys:
                row = f"| {rl} |"
                for d in summary_devs:
                    v = cross_depth["per_device_summary"][d].get(rk, "N/A")
                    if isinstance(v, float):
                        v = f"{v:.1f}" if rk in ("depth_mean_mm", "depth_median_mm", "depth_std_mm", "temporal_std_mm", "imu_accel_hz", "imu_gyro_hz") else str(v)
                    row += f" {v} |"
                lines.append(row)
            lines.append("")

        for pair_key, comp in cross_depth["comparison"].items():
            d1, d2 = pair_key.split("_vs_")
            lines.append(f"### 8.{2+list(cross_depth['comparison'].keys()).index(pair_key)} {dev_labels.get(d1, d1)} vs {dev_labels.get(d2, d2)}")
            lines.append("")
            if "resolution_diff" in comp:
                lines.append(f"- 分辨率差异：{comp['resolution_diff']}")
            if "mean_diff_mm" in comp:
                lines.append(f"- 全局均值差：**{comp['mean_diff_mm']:.1f} mm ({comp['mean_diff_m']:.3f} m)**")
                lines.append(f"  - {d1}: {comp['d1_global_mean_mm']:.1f} mm")
                lines.append(f"  - {d2}: {comp['d2_global_mean_mm']:.1f} mm")
            if "median_diff_mm" in comp:
                lines.append(f"- 全局中位数差：**{comp['median_diff_mm']:.1f} mm**")
                lines.append(f"  - {d1}: {comp['d1_global_median_mm']:.1f} mm")
                lines.append(f"  - {d2}: {comp['d2_global_median_mm']:.1f} mm")
            if "valid_ratio_diff" in comp:
                lines.append(f"- 有效像素比差异：{comp['valid_ratio_diff']*100:.1f}%")
            if "d1_temporal_std_mm" in comp:
                lines.append(f"- 时间稳定性对比：{d1}={comp['d1_temporal_std_mm']:.1f} mm, {d2}={comp['d2_temporal_std_mm']:.1f} mm")
            lines.append("")

            if comp.get("percentile_diff"):
                lines.append(f"#### 深度百分位对比")
                lines.append("")
                pct = comp["percentile_diff"]
                header = "| 百分位 |" + f" {dev_labels.get(d1, d1)} (mm) | {dev_labels.get(d2, d2)} (mm) | 差值 (mm) |"
                sep = "|--------|" + "------|------|------|"
                lines.append(header)
                lines.append(sep)
                for k in sorted(pct.keys()):
                    v = pct[k]
                    pct_label = k[1:] if k.startswith("p") else k
                    lines.append(f"| {pct_label}% | {v['d1_mm']:.0f} | {v['d2_mm']:.0f} | {v['diff_mm']:.0f} |")
                lines.append("")

            lines.append("_注：均值差异主要来源于安装位置和视角差异，不直接代表测距精度差异。_")
            lines.append("")
    else:
        lines.append("_仅有一台设备或深度数据不足，无法进行跨设备对比_")
    lines.append("")

    lines.append("## 9. 通过/不通过判定")
    lines.append("")
    _emit_section_notes(lines, 9)

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
            all_checks.append((f"{dev_labels.get(dt, dt)} D2C融合", assess_d2c_fusion(d2c, dt)))
        enc = devices_data[dt].get("h264_encoding", {})
        if enc.get("streams"):
            all_checks.append((f"{dev_labels.get(dt, dt)} H.264编码", assess_h264_encoding(enc, dt)))
        sensor = devices_data[dt].get("color_stats", {})
        if sensor.get("sensor_quality"):
            all_checks.append((f"{dev_labels.get(dt, dt)} 传感器成像", assess_color_sensor(sensor, dt)))

    if alignment:
        ta_checks = assess_temporal_alignment(alignment, dev_types)
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

    lines.append("## 10. 设备技术规格对比 (Structured Light vs LiDAR TOF)")
    lines.append("")
    _emit_section_notes(lines, 10)
    lines.append("本节从测距原理层面对比 Orbbec（主动立体视觉+IR散斑结构光）与 RoboSense AC1（VCSEL+SPAD TOF LiDAR）")
    lines.append("的根本差异，这些差异决定了深度数据特性而非简单的数值高低。")
    lines.append("")

    lines.append("### 10.1 测距原理对比")
    lines.append("")
    lines.append("| 特性 | Structured Light (335L/336L) | TOF LiDAR (AC1) |")
    lines.append("|------|---------------------------|-----------------|")
    lines.append("| 测距原理 | VCSEL投射IR散斑→双IR相机立体匹配→视差→深度 | VCSEL发射脉冲→SPAD计时往返→深度 |")
    lines.append("| 深度密度 | 每像素一个深度值（1280x800=1M pts） | 稀疏点云（~17k pts/frame） |")
    lines.append("| 深度噪声特征 | 与散斑匹配置信度相关，表面反射率影响大 | 与计时精度相关，距离影响大 |")
    lines.append("| 深度精度范围 | 335L: 0.3-5m; 336L: 0.1-20m | AC1: 0.1-70m (@10%反射率40m) |")
    lines.append("| 户外性能 | 受环境IR干扰（阳光含IR）；336L IR滤光片缓解 | TOF抗干扰强，支持100klux阳光 |")
    lines.append("| 暗面/黑体 | 散斑反射率低→匹配失败→空洞 | SPAD灵敏度低→回波弱→无效点 |")
    lines.append("| 透明/镜面 | 散斑穿透/反射→错误深度 | 脉冲穿透→无效或穿透后反射 |")
    lines.append("| 近距精度 | 极高（视差范围大，亚像素精度高） | 一般（计时分辨率固定） |")
    lines.append("| 远距精度 | 335L: 5m截止; 336L: 精度随距离²下降 | 远距仍有效，精度±3cm@>5m |")
    lines.append("| 帧率 | 30fps（全局快门IR，无运动模糊） | 10fps（累积时间影响帧率） |")
    lines.append("| D2C方式 | SW重投影（深度→彩色坐标变换） | HW上采样（96x288→1920x1080） |")
    lines.append("")

    lines.append("### 10.2 已发布规格对比表")
    lines.append("")
    spec_devs = [dt for dt in dev_types if dt in DEVICE_SPECS]
    if spec_devs:
        spec_header = "| 规格项 |" + "|".join([f" {dev_labels.get(dt, dt)} " for dt in spec_devs]) + "|"
        spec_sep = "|--------|" + "|".join(["------" for _ in spec_devs]) + "|"
        lines.append(spec_header)
        lines.append(spec_sep)
        spec_rows = [
            ("测距原理", "depth_tech"),
            ("深度分辨率", "depth_res"),
            ("深度帧率", None),
            ("深度范围", None),
            ("深度精度", None),
            ("基线/cm", None),
            ("彩色分辨率", "color_res"),
            ("彩色FOV", "color_fov"),
            ("深度FOV", "depth_fov"),
            ("IMU频率", None),
            ("IR流数", "ir_streams"),
            ("D2C模式", "d2c_mode"),
        ]
        for label, key in spec_rows:
            row = f"| {label} |"
            for dt in spec_devs:
                s = DEVICE_SPECS[dt]
                if label == "深度帧率":
                    row += f" {s.get('depth_fps', 'N/A')} fps |"
                elif label == "深度范围":
                    r = s.get("depth_range_m", ())
                    row += f" {r[0]}-{r[1]}m |" if r else " N/A |"
                elif label == "深度精度":
                    if s.get("depth_accuracy_pct"):
                        row += f" ≤{s['depth_accuracy_pct']}% @2m |"
                    elif s.get("depth_accuracy_cm"):
                        a = s["depth_accuracy_cm"]
                        row += f" ±{a[0]}cm(0-5m), ±{a[1]}cm(>5m) |"
                    else:
                        row += " 335L: N/A |"
                elif label == "基线/cm":
                    b = s.get("baseline_mm")
                    row += f" {b/10:.1f}cm |" if b else " N/A(固态) |"
                elif label == "IMU频率":
                    row += f" {s.get('imu_rate_hz', 'N/A')}Hz |"
                elif label == "IR滤光片":
                    row += f" {'是' if s.get('ir_filter') else '否'} |"
                else:
                    row += f" {s.get(key, 'N/A')} |"
            lines.append(row)
        lines.append("")

        lines.append("### 10.3 336L IR 滤光片特性")
        lines.append("")
        lines.append("336L 在 335L 基础上增加了 **IR带通滤光片**（透过>750nm，阻挡可见光<750nm）。")
        lines.append("效果：")
        lines.append("- 增强户外/强光下的散斑信噪比（滤除环境可见光噪声）")
        lines.append("- 改善 IR 图像对比度（仅 IR 散斑投影与场景反射 IR 通过）")
        lines.append("- 测距范围从 5m 扩展到 20m（散斑匹配在更长基线下仍可靠）")
        lines.append("- 代价：IR 图像失去可见光信息，不适合需要可见光+IR 的融合应用")
        lines.append("")
        ir_336l = devices_data.get("336L", {}).get("ir_stats", {})
        if ir_336l.get("ir_available"):
            lines.append("_336L IR 流数据已采集，详见第12节 IR 散斑质量分析_")
        else:
            lines.append("_336L IR 流暂无采集数据，待设备接入后补充_")
        lines.append("")
    else:
        lines.append("_无已采集设备匹配 DEVICE_SPECS 配置_")
        lines.append("")

    lines.append("## 11. 不同距离精度评估")
    lines.append("")
    _emit_section_notes(lines, 11)
    lines.append("本节基于采集的 depth_raw 数据，按距离区间统计深度精度（标准差/相对精度），")
    lines.append("并与各设备已发布规格进行对比。实测精度受场景内容影响，仅供参考。")
    lines.append("")

    any_acc_data = False
    for dt in dev_types:
        acc = devices_data[dt].get("depth_accuracy_stats", {})
        if not acc.get("available"):
            continue
        any_acc_data = True
        lines.append(f"### 11.{dev_types.index(dt)+1} {dev_labels.get(dt, dt)}")
        lines.append("")
        bins = acc.get("distance_bins", [])
        if not bins:
            lines.append("_距离区间统计数据不足（有效深度点<100），跳过_")
            lines.append("")
            continue

        lines.append("| 目标距离 (m) | 采样点数 | 深度std (mm) | 相对精度 (%) | 规格精度 (%) | 是否达标 |")
        lines.append("|-------------|---------|-------------|-------------|-------------|---------|")
        for b in bins:
            spec_str = f"{b['spec_accuracy_pct']:.2f}%" if b.get("spec_accuracy_pct") is not None else "N/A"
            pass_str = "✅" if b.get("passes_spec") is True else ("❌" if b.get("passes_spec") is False else "—")
            lines.append(f"| {b['distance_m']:.2f} | {b['num_points']} | {b['depth_std_mm']:.1f} | {b['relative_precision_pct']:.2f} | {spec_str} | {pass_str} |")
        lines.append("")

        temporal = acc.get("temporal_noise_by_distance", {})
        if temporal:
            lines.append("**时间域噪声（帧间深度差）按距离区间：**")
            lines.append("")
            lines.append("| 距离区间 | 中位数噪声 (mm) | P95噪声 (mm) |")
            lines.append("|---------|-----------------|-------------|")
            for rng, vals in sorted(temporal.items()):
                lines.append(f"| {rng} | {vals.get('temporal_noise_median_mm', 'N/A')} | {vals.get('temporal_noise_p95_mm', 'N/A')} |")
            lines.append("")

    if not any_acc_data:
        lines.append("_暂无足量深度帧数据（需≥3帧）进行距离精度分析，待采集后补充_")
        lines.append("")
        lines.append("| 目标距离 (m) | 335L 误差 (mm) | 336L 误差 (mm) | AC1 误差 (mm) | 测试方法 |")
        lines.append("|-------------|---------------|---------------|--------------|----------|")
        for d in [0.5, 1.0, 2.0, 3.0, 5.0, 10.0, 20.0, 50.0]:
            method = "平面标定板" if d <= 5 else ("大型平面/墙面" if d <= 20 else "远距目标")
            lines.append(f"| {d} | - | - | - | {method} |")
        lines.append("")
        lines.append("_测试方法：在已知距离放置标定板/平面，采集30帧以上，取深度均值与已知距离之差为系统误差，标准差为随机误差。_")
        lines.append("")

    lines.append("## 12. IR 红外散斑流质量分析")
    lines.append("")
    _emit_section_notes(lines, 12)
    lines.append("本节分析 Orbbec 设备的 IR 流质量，散斑投影器（VCSEL）投射随机散斑到场景，")
    lines.append("左右 IR 全局快门相机捕捉散斑图用于立体匹配，深度质量直接取决于 IR 散斑质量。")
    lines.append("")

    any_ir = False
    for dt in dev_types:
        ir = devices_data[dt].get("ir_stats", {})
        if not ir.get("ir_available"):
            continue
        any_ir = True
        lines.append(f"### 12.{dev_types.index(dt)+1} {dev_labels.get(dt, dt)}")
        lines.append("")
        if ir.get("ir_tech_note"):
            lines.append(f"_技术说明：{ir['ir_tech_note']}_")
            lines.append("")
        if ir.get("ir_filter_note"):
            lines.append(f"**IR 滤光片效果**：{ir['ir_filter_note']}")
            lines.append("")

        for st, info in ir.get("ir_streams", {}).items():
            label = "左IR" if "left" in st else "右IR"
            lines.append(f"**{label} ({st})**")
            lines.append("")
            lines.append("| 指标 | 值 | 说明 |")
            lines.append("|------|-----|------|")
            lines.append(f"| 分辨率 | {info.get('resolution', 'N/A')} | |")
            lines.append(f"| 帧数 | {info.get('frame_count', 0)} | |")
            lines.append(f"| 平均码率 | {info.get('avg_bitrate_kbps', 0)} kbps | |")
            if "sharpness" in info:
                lines.append(f"| 锐度 (Laplacian var) | {info['sharpness']:.2f} | IR 纹理清晰度 |")
            if "brightness_mean" in info:
                lines.append(f"| 平均亮度 | {info['brightness_mean']:.1f} | IR 灰度均值 |")
            if "brightness_std" in info:
                lines.append(f"| 亮度标准差 | {info['brightness_std']:.1f} | 场景对比度 |")
            if "saturated_pixel_pct" in info:
                sat_status = "⚠️ 过曝" if info['saturated_pixel_pct'] > 10 else "正常"
                lines.append(f"| 饱和像素占比 | {info['saturated_pixel_pct']:.1f}% | {sat_status} |")
            if "center_brightness" in info:
                lines.append(f"| 中心亮度 | {info['center_brightness']:.1f} | 投影器中心照度 |")
            if "edge_brightness" in info:
                lines.append(f"| 边缘亮度 | {info['edge_brightness']:.1f} | 投影器边缘照度 |")
            if "illumination_uniformity" in info:
                u = info['illumination_uniformity']
                u_note = "均匀" if u > 0.7 else ("尚可" if u > 0.4 else "⚠️ 不均匀")
                lines.append(f"| 照明均匀度 (边缘/中心) | {u:.3f} | {u_note} |")
            if "speckle_contrast" in info:
                lines.append(f"| 散斑对比度 | {info['speckle_contrast']:.2f} | 高频标准差，越高散斑越明显 |")
            if "speckle_energy" in info:
                lines.append(f"| 散斑能量 | {info['speckle_energy']:.1f} | 高频MSE |")
            if "speckle_snr_db" in info:
                snr = info['speckle_snr_db']
                snr_note = "优秀" if snr > 20 else ("良好" if snr > 10 else "⚠️ 偏低")
                lines.append(f"| 散斑信噪比 (dB) | {snr:.2f} | {snr_note} |")
            lines.append("")

        if ir.get("stereo_ir_brightness_diff") is not None:
            lines.append(f"**立体IR一致性**：左右IR亮度差 = {ir['stereo_ir_brightness_diff']:.1f}，"
                         f"亮度比 = {ir.get('stereo_ir_brightness_ratio', 'N/A')}")
            if ir.get("stereo_speckle_contrast_diff") is not None:
                lines.append(f"左右散斑对比度差 = {ir['stereo_speckle_contrast_diff']:.2f}")
            lines.append("")

    if not any_ir:
        lines.append("_无 Orbbec 设备 IR 流数据，或 cv2 不可用。IR 散斑流分析仅适用于 335L/336L。_")
        lines.append("")

    lines.append("## 13. PCD 点云密度与分布分析")
    lines.append("")
    _emit_section_notes(lines, 13)
    lines.append("本节分析 PCD 点云数据的密度、距离分布、有效比例等空间特性，")
    lines.append("区分 Structured Light（密集深度图转点云）与 LiDAR TOF（稀疏直接点云）的密度差异。")
    lines.append("")

    any_pcd_cloud = False
    for dt in dev_types:
        pc = devices_data[dt].get("pcd_cloud_stats", {})
        if not pc.get("available"):
            continue
        any_pcd_cloud = True
        lines.append(f"### 13.{dev_types.index(dt)+1} {dev_labels.get(dt, dt)}")
        lines.append("")
        lines.append("| 指标 | 值 | 说明 |")
        lines.append("|------|-----|------|")
        lines.append(f"| PCD文件数 | {pc.get('total_pcd_files', 0)} | |")
        if "avg_points" in pc:
            lines.append(f"| 平均点数/帧 | {pc['avg_points']:.0f} | |")
            lines.append(f"| 最小点数 | {pc['min_points']} | |")
            lines.append(f"| 最大点数 | {pc['max_points']} | |")
            lines.append(f"| 点数稳定性 (CV) | {pc.get('point_count_stability', 'N/A')} | <0.1稳定 |")
        if "expected_pts_per_frame" in pc:
            lines.append(f"| 规格预期点数/帧 | {pc['expected_pts_per_frame']} | pts_per_sec/fps |")
            lines.append(f"| 实际/规格比 | {pc.get('pts_per_frame_vs_spec_pct', 'N/A')}% | |")
        if "distance_mean_m" in pc:
            lines.append(f"| 平均距离 | {pc['distance_mean_m']} m | |")
            lines.append(f"| 距离中位数 | {pc['distance_median_m']} m | |")
            lines.append(f"| 距离标准差 | {pc['distance_std_m']} m | |")
            lines.append(f"| 距离范围 | {pc['distance_min_m']}-{pc['distance_max_m']} m | |")
        if "points_in_spec_range_pct" in pc:
            lines.append(f"| 规格范围内点占比 | {pc['points_in_spec_range_pct']:.1f}% | depth_range_m |")
        bins = pc.get("distance_bins", {})
        if bins:
            lines.append(f"| 近距(<1m) | {bins.get('short_lt1m_pct', 'N/A')}% | |")
            lines.append(f"| 中距(1-5m) | {bins.get('mid_1to5m_pct', 'N/A')}% | |")
            lines.append(f"| 远距(>5m) | {bins.get('far_gt5m_pct', 'N/A')}% | |")
        if "pcd_fps" in pc:
            lines.append(f"| PCD 帧率 | {pc['pcd_fps']:.1f} fps | |")
            lines.append(f"| PCD 持续时间 | {pc.get('pcd_duration_s', 'N/A')} s | |")
        lines.append("")

        dist_pcts = pc.get("distance_percentiles_m", {})
        if dist_pcts:
            lines.append("**距离分布百分位：**")
            pstr = " / ".join([f"P{p.replace('p','')}={v}m" for p, v in dist_pcts.items()])
            lines.append(f"{pstr}")
            lines.append("")

    if not any_pcd_cloud:
        lines.append("_无 PCD 目录数据，点云密度分析不可用_")
        lines.append("")

    lines.append("## 14. 关键发现与建议")
    lines.append("")
    _emit_section_notes(lines, 14)
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

    findings.append("335L 使用 SW D2C（深度→彩色对齐），1280x800→1280x800 软件重投影；AC1 使用 HW D2C，但 96x288→1920x1080 上采样产生块状伪影")

    if alignment and alignment.get("imu_start_offset_ms") is not None:
        offset = alignment["imu_start_offset_ms"]
        if offset > 100:
            findings.append(f"设备间 IMU 起始时间偏移 {offset:.1f} ms 较大，可能影响多设备帧同步融合的精度")
        else:
            findings.append(f"设备间 IMU 起始时间偏移仅 {offset:.1f} ms，时间同步质量良好")

    if "336L" not in dev_types or not depth_data.get("336L"):
        findings.append("336L（Gemini 336L, IR滤光片+长距版335L）暂无物理设备，评估需等待设备接入。336L标称测距0.1-20m，精度≤1.5%@2m，IR带通滤光片增强户外散斑SNR")

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

    for dt in dev_types:
        ir = devices_data[dt].get("ir_stats", {})
        for st, info in ir.get("ir_streams", {}).items():
            if info.get("saturated_pixel_pct", 0) > 10:
                findings.append(f"{dt} {st} 红外流过曝（饱和像素{info['saturated_pixel_pct']:.1f}%），散斑投影器功率可能与近距离场景不匹配，考虑降低曝光或调整距离")
            u = info.get("illumination_uniformity", 0)
            if u and u < 0.4:
                findings.append(f"{dt} {st} IR 照明均匀度={u:.3f}（边缘/中心比），偏低，影响边缘深度质量")

    for dt in dev_types:
        pc = devices_data[dt].get("pcd_cloud_stats", {})
        if pc.get("available") and pc.get("pts_per_frame_vs_spec_pct"):
            pct = pc["pts_per_frame_vs_spec_pct"]
            if pct < 50:
                findings.append(f"{dt} PCD 实际点数仅为规格的 {pct:.1f}%，可能存在点云过滤或数据丢失")
            elif pct > 120:
                findings.append(f"{dt} PCD 实际点数超过规格 {pct:.1f}%，检查点云格式是否含重复/额外字段")

    orb_devs = [dt for dt in dev_types if dt in ("335L", "336L")]
    ac1_devs = [dt for dt in dev_types if dt == "AC1"]
    if orb_devs and ac1_devs:
        for odt in orb_devs:
            pc_orb = devices_data[odt].get("pcd_cloud_stats", {})
            pc_ac1 = devices_data.get("AC1", {}).get("pcd_cloud_stats", {})
            if pc_orb.get("avg_points") and pc_ac1.get("avg_points"):
                ratio = pc_orb["avg_points"] / max(pc_ac1["avg_points"], 1)
                tech_note = "（结构光每像素一个深度点 vs LiDAR稀疏离散点）"
                findings.append(f"{odt} PCD 点数约 {pc_orb['avg_points']:.0f}，AC1 约 {pc_ac1['avg_points']:.0f}，"
                               f"比值 {ratio:.0f}x {tech_note}")

    for dt in dev_types:
        acc = devices_data[dt].get("depth_accuracy_stats", {})
        if acc.get("available"):
            bins = acc.get("distance_bins", [])
            failed_bins = [b for b in bins if b.get("passes_spec") is False]
            if failed_bins:
                dists = ", ".join([f"{b['distance_m']:.1f}m({b['relative_precision_pct']:.2f}%>spec{b['spec_accuracy_pct']:.2f}%)" for b in failed_bins])
                findings.append(f"{dt} 部分距离区间深度精度未达规格：{dists}")
            temporal = acc.get("temporal_noise_by_distance", {})
            for rng, vals in temporal.items():
                if vals.get("temporal_noise_p95_mm", 0) > 50:
                    findings.append(f"{dt} 距离区间{rng}时间噪声P95={vals['temporal_noise_p95_mm']:.1f}mm偏高，可能影响深度滤波收敛")

    for i, f in enumerate(findings, 1):
        lines.append(f"{i}. {f}")
    lines.append("")

    lines.append("## 15. 附录：原始数据路径")
    lines.append("")
    _emit_section_notes(lines, 15)
    for dt in dev_types:
        lines.append(f"- {dev_labels.get(dt, dt)}: `{devices_data[dt]['path']}`")
    lines.append("")

    lines.append("## 16. 彩色与红外视频流分析")
    lines.append("")
    _emit_section_notes(lines, 16)
    for dt in dev_types:
        cs = devices_data[dt].get("color_stats", {})
        if not cs:
            continue
        lines.append(f"### 16.{dev_types.index(dt)+1} {dev_labels.get(dt, dt)}")
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
    lines.append(f"### 16.{enc_sub_idx + 1} H.264 编码参数诊断")
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
    lines.append(f"### 16.{sq_sub_idx} 彩色传感器成像质量分析")
    lines.append("")
    lines.append("本节从传感器物理成像层面分析彩色流图像质量，区分 MJPEG 解压伪影与原始传感器噪声。")
    lines.append("以下列出每个设备的彩色源类型与噪声归因，以了解 MJPEG 解压伪影或原始传感器噪声。")
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

    # 彩色流首帧对比（构成“对比视频截图”）
    if _MPL_AVAILABLE and _CV2_AVAILABLE:
        any_color_frame = False
        for dt in dev_types:
            files = devices_data.get(dt, {}).get("_files", {}) or {}
            h264_by_type = files.get("h264_by_type", {}) or {}
            color_path = h264_by_type.get("color")
            if not color_path or not os.path.exists(color_path):
                continue
            tmp_dir = os.path.join("/tmp", f"nio_eval_colorfig_{dt}")
            try:
                frames = extract_h264_sample_frames(color_path, [0], tmp_dir)
            except Exception:
                frames = {}
            if not frames:
                continue
            sample_idx = list(frames.keys())[0]
            img_bgr = frames[sample_idx]
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            fig, ax = plt.subplots(figsize=(8, 5))
            ax.imshow(img_rgb)
            ax.set_title(f"{dt} 彩色流首帧 (帧 {sample_idx})")
            ax.axis("off")
            ok = _save_fig_to_report(fig, assets_abs, assets_rel,
                                     f"color_frame0_{dt}", lines,
                                     f"{dt} 彩色首帧")
            any_color_frame = any_color_frame or ok
        if any_color_frame:
            # Insert a section header before the figures were appended (re-write
            # would require buffering; use the inline notes header instead).
            pass
        else:
            lines.append("> *（未找到可抽帧的彩色 H.264 流——跳过彩色截图对比。）*")
            lines.append("")
    elif _MPL_AVAILABLE and not _CV2_AVAILABLE:
        lines.append("> *（cv2 不可用——跳过彩色首帧对比图生成。）*")
        lines.append("")
    else:
        lines.append("> *（matplotlib 不可用——跳过彩色首帧对比图生成。）*")
        lines.append("")

    lines.append("## 17. D2C 深度-彩色融合分析")
    lines.append("")
    _emit_section_notes(lines, 17)
    for dt in dev_types:
        d2c = devices_data[dt].get("d2c_fusion", {})
        if not d2c:
            continue
        lines.append(f"### 17.{dev_types.index(dt)+1} {dev_labels.get(dt, dt)}")
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

    # D2C 合成帧与对配彩色对比图（含 8x8 块网格标注与差异热图）
    if _MPL_AVAILABLE and _CV2_AVAILABLE:
        header_emitted = False
        any_d2c_fig = False
        for dt in dev_types:
            d2c = devices_data.get(dt, {}).get("d2c_fusion", {}) or {}
            if not d2c.get("fused_available"):
                continue
            if not header_emitted:
                lines.append("### 17.N 融合帧与原始彩色帧的差异可视化")
                lines.append("")
                lines.append("每张图含 3 列：彩色帧、融合帧 (叠加 8x8 块网格)、以及彩色-融合绝对差异热图。8x8 网格叠加用以帮助读者人工核对 \"8x8 block boundary score\" 项；黑色热区表示融合未改变彩色像素，亮色热区表示融合叠加了 jet 深度色或对齐错位带来的差异。")
                lines.append("")
                header_emitted = True
            fig = _make_color_d2c_pair_figure(dt, devices_data, dt)
            ok = _save_fig_to_report(fig, assets_abs, assets_rel,
                                     f"d2c_pair_{dt}", lines,
                                     f"{dt} 彩色 vs D2C 融合帧（含 8x8 块网格标注 + 差异热图）")
            any_d2c_fig = any_d2c_fig or ok
        if not any_d2c_fig:
            lines.append("_（无可用的 fused + paired color 帧——可能 fusion 未生成或无对应彩色流。）_")
            lines.append("")
    elif _MPL_AVAILABLE and not _CV2_AVAILABLE:
        lines.append("> *（cv2 不可用——跳过融合/彩色对比图生成。）*")
        lines.append("")
    else:
        lines.append("> *（matplotlib 不可用——跳过融合/彩色对比图生成。）*")
        lines.append("")

    lines.append("## 18. 跨设备融合方法对比")
    lines.append("")
    _emit_section_notes(lines, 18)
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

        lines.append("### 18.1 SW D2C vs HW D2C 分析")
        lines.append("")
        lines.append("- **SW D2C（335L/336L）**：深度帧 1280x800 通过软件重投影对齐到彩色 1280x800，")
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
    result = {"path": dev_path, "depth_stats": {}, "imu_stats": {}, "pcd_stats": {}, "intrinsic": None, "log_info": {}, "h264_stats": {}, "d2c_fusion": {}, "color_stats": {}, "h264_encoding": {}, "ir_stats": {}, "pcd_cloud_stats": {}, "depth_accuracy_stats": {}}
    files = find_files(dev_path)
    result["_files"] = files
    files = find_files(dev_path)

    depth_info = None
    if files["raw"]:
        print(f"  Parsing depth_raw: {files['raw']}")
        depth_info = parse_depth_raw(files["raw"])
        if depth_info:
            print(f"    {depth_info['num_frames']} frames, {depth_info['width']}x{depth_info['height']}, scale={depth_info['scale']}")
            result["depth_stats"] = analyze_depth(depth_info, dev_type)
            # Keep a (deep) copy of the parsed depth info so generate_report
            # can render raw frame heatmaps. Only the first frame is needed
            # for the figure, so avoid dragging all frames into the report
            # path — store just frame[0] under depth_stats.
            if depth_info.get("frames"):
                result["depth_stats"]["_raw_frame0"] = depth_info["frames"][0]
                result["depth_stats"]["_raw_scale"] = depth_info["scale"]
                result["depth_stats"]["_raw_w"] = depth_info["width"]
                result["depth_stats"]["_raw_h"] = depth_info["height"]
            ds = result["depth_stats"]
            print(f"    Valid ratio: {ds.get('avg_valid_ratio', 0)*100:.1f}%, Global mean: {ds.get('global_mean_mm', 0):.1f} mm")

    if files["imu"]:
        print(f"  Parsing IMU: {files['imu']}")
        imu_records = parse_imu(files["imu"])
        result["imu_stats"] = analyze_imu(imu_records, dev_type)
        # Keep the parsed IMU records so generate_report can render time
        # series / spectrum plots. The file may be large but it is already
        # fully parsed above; embedding a reference costs nothing extra.
        result["imu_stats"]["_records"] = imu_records
        imu = result["imu_stats"]
        print(f"    {imu.get('total_samples', 0)} samples, accel={imu.get('accel_rate_hz',0):.0f}Hz, gyro={imu.get('gyro_rate_hz',0):.0f}Hz")

    if files["pcd_dir"]:
        print(f"  Analyzing PCD: {files['pcd_dir']}")
        result["pcd_stats"] = analyze_pcd_dir(files["pcd_dir"], dev_type)
        pcd = result["pcd_stats"]
        print(f"    {pcd.get('total_pcd_files', 0)} PCD files, avg points={pcd.get('avg_points', 0):.0f}")

    result["intrinsic"] = load_intrinsic(files["intrinsic"])
    log_path = files["log"] or session_log
    result["log_info"] = parse_capture_log(log_path, device_path=dev_path)

    if files["h264_by_type"]:
        print(f"  Analyzing H.264 streams: {list(files['h264_by_type'].keys())}")
        result["h264_stats"] = analyze_h264_streams(files["h264_by_type"], dev_type, dev_type)
        print(f"  Analyzing H.264 encoding diagnostics...")
        result["h264_encoding"] = analyze_h264_encoding(files["h264_by_type"], dev_type)
        print(f"  Analyzing color stream quality...")
        result["color_stats"] = analyze_color_streams(files["h264_by_type"], dev_type, dev_type, result["depth_stats"], result["log_info"].get("color_format", "unknown"))
        cs = result["color_stats"]
        if cs.get("color_issue"):
            print(f"    WARNING: Color issue={cs['color_issue']} ({cs.get('color_issue_note', '')})")
        print(f"  Analyzing D2C fusion...")
        result["d2c_fusion"] = analyze_d2c_fusion(files["h264_by_type"], dev_type, dev_type, result["log_info"])

    if dev_type in ("335L", "336L") and files.get("h264_by_type"):
        print(f"  Analyzing IR stream quality...")
        result["ir_stats"] = analyze_ir_stream_quality(files["h264_by_type"], dev_type, dev_type)

    if files["pcd_dir"]:
        print(f"  Analyzing PCD cloud distribution...")
        result["pcd_cloud_stats"] = analyze_pcd_cloud(files["pcd_dir"], dev_type, dev_type)

    if depth_info and depth_info.get("num_frames", 0) >= 3:
        print(f"  Analyzing depth accuracy by distance...")
        result["depth_accuracy_stats"] = analyze_depth_accuracy_by_distance(
            depth_info, dev_type
        )

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
        if not log_file:
            parent = os.path.dirname(root.rstrip("/"))
            if parent and os.path.isdir(parent):
                for fn in os.listdir(parent):
                    if fn.startswith("dynamic_algo_cam_log") and fn.endswith(".log"):
                        log_file = os.path.join(parent, fn)
                        break
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
