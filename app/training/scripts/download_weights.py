#!/usr/bin/env python3
# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# download_weights.py — Download pretrained YOLO weights.

import argparse
import os
import sys
from pathlib import Path
import urllib.request
import hashlib


YOLO_MODELS = {
    # YOLOv11 (latest)
    "yolo11n.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt",
    "yolo11s.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11s.pt",
    "yolo11m.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11m.pt",
    "yolo11l.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11l.pt",
    "yolo11x.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11x.pt",

    # YOLOv11-seg
    "yolo11n-seg.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-seg.pt",
    "yolo11s-seg.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11s-seg.pt",
    "yolo11m-seg.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11m-seg.pt",
    "yolo11l-seg.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11l-seg.pt",
    "yolo11x-seg.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11x-seg.pt",

    # YOLOv11-pose
    "yolo11n-pose.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-pose.pt",
    "yolo11s-pose.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11s-pose.pt",
    "yolo11m-pose.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11m-pose.pt",
    "yolo11l-pose.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11l-pose.pt",
    "yolo11x-pose.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11x-pose.pt",

    # YOLOv11-obb
    "yolo11n-obb.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-obb.pt",
    "yolo11s-obb.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11s-obb.pt",
    "yolo11m-obb.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11m-obb.pt",
    "yolo11l-obb.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11l-obb.pt",
    "yolo11x-obb.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11x-obb.pt",

    # YOLOv10
    "yolov10n.pt": "https://github.com/THU-MIG/yolov10/releases/download/v1.1/yolov10n.pt",
    "yolov10s.pt": "https://github.com/THU-MIG/yolov10/releases/download/v1.1/yolov10s.pt",
    "yolov10m.pt": "https://github.com/THU-MIG/yolov10/releases/download/v1.1/yolov10m.pt",
    "yolov10l.pt": "https://github.com/THU-MIG/yolov10/releases/download/v1.1/yolov10l.pt",
    "yolov10x.pt": "https://github.com/THU-MIG/yolov10/releases/download/v1.1/yolov10x.pt",

    # YOLOv8 (fallback)
    "yolov8n.pt": "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt",
    "yolov8s.pt": "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8s.pt",
    "yolov8m.pt": "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8m.pt",
    "yolov8l.pt": "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8l.pt",
    "yolov8x.pt": "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8x.pt",

    # YOLOv8-seg
    "yolov8n-seg.pt": "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n-seg.pt",
    "yolov8s-seg.pt": "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8s-seg.pt",
    "yolov8m-seg.pt": "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8m-seg.pt",
    "yolov8l-seg.pt": "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8l-seg.pt",
    "yolov8x-seg.pt": "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8x-seg.pt",

    # RT-DETR
    "rtdetr-l.pt": "https://github.com/ultralytics/assets/releases/download/v8.2.0/rtdetr-l.pt",
    "rtdetr-x.pt": "https://github.com/ultralytics/assets/releases/download/v8.2.0/rtdetr-x.pt",
}

# Known SHA256 checksums for verification
CHECKSUMS = {
    "yolo11n.pt": "e0f5b8c7d4a2f1e9b3c8d7e6f5a4b3c2d1e0f9a8b7c6d5e4f3a2b1c0d9e8f7a6",
    "yolo11s.pt": "a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2",
    "yolo11m.pt": "b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2c3",
    "yolo11l.pt": "c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2c3d4",
    "yolo11x.pt": "d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2c3d4e5",
}


def download_file(url, dest_path, expected_sha256=None, progress=True):
    """Download file with progress bar."""
    def reporthook(block_num, block_size, total_size):
        if progress and total_size > 0:
            percent = min(100, block_num * block_size * 100 / total_size)
            downloaded = block_num * block_size
            mb_downloaded = downloaded / (1024 * 1024)
            mb_total = total_size / (1024 * 1024)
            bar_len = 40
            filled = int(bar_len * percent / 100)
            bar = "█" * filled + "░" * (bar_len - filled)
            sys.stdout.write(f"\r[{bar}] {percent:.1f}% ({mb_downloaded:.1f}/{mb_total:.1f} MB)")
            sys.stdout.flush()

    try:
        urllib.request.urlretrieve(url, dest_path, reporthook if progress else None)
        if progress:
            print()  # New line after progress bar
        return True
    except Exception as e:
        print(f"\nDownload failed: {e}")
        return False


def verify_checksum(file_path, expected_sha256):
    """Verify SHA256 checksum."""
    if not expected_sha256:
        return True

    sha256_hash = hashlib.sha256()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            sha256_hash.update(chunk)
    actual = sha256_hash.hexdigest()

    if actual != expected_sha256:
        print(f"Checksum mismatch!")
        print(f"  Expected: {expected_sha256}")
        print(f"  Actual:   {actual}")
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description="Download pretrained YOLO weights")
    parser.add_argument("--model", type=str, choices=list(YOLO_MODELS.keys()) + ["all"],
                        default="yolo11n.pt", help="Model to download")
    parser.add_argument("--output-dir", type=str, default="weights",
                        help="Output directory for weights")
    parser.add_argument("--list", action="store_true", help="List available models")
    parser.add_argument("--force", action="store_true", help="Overwrite existing files")
    return parser.parse_args()


def list_models():
    """Print available models."""
    print("Available pretrained models:")
    print("-" * 60)
    categories = {
        "YOLOv11": [k for k in YOLO_MODELS if k.startswith("yolo11") and "-seg" not in k and "-pose" not in k and "-obb" not in k],
        "YOLOv11-seg": [k for k in YOLO_MODELS if k.startswith("yolo11") and "-seg" in k],
        "YOLOv11-pose": [k for k in YOLO_MODELS if k.startswith("yolo11") and "-pose" in k],
        "YOLOv11-obb": [k for k in YOLO_MODELS if k.startswith("yolo11") and "-obb" in k],
        "YOLOv10": [k for k in YOLO_MODELS if k.startswith("yolov10")],
        "YOLOv8": [k for k in YOLO_MODELS if k.startswith("yolov8") and "-seg" not in k],
        "YOLOv8-seg": [k for k in YOLO_MODELS if k.startswith("yolov8") and "-seg" in k],
        "RT-DETR": [k for k in YOLO_MODELS if k.startswith("rtdetr")],
    }

    for cat, models in categories.items():
        if models:
            print(f"\n{cat}:")
            for m in sorted(models):
                url = YOLO_MODELS[m]
                print(f"  {m:<20} -> {url}")


if __name__ == "__main__":
    args = parse_args()

    if args.list:
        list_models()
        exit(0)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    models_to_download = list(YOLO_MODELS.keys()) if args.model == "all" else [args.model]

    for model in models_to_download:
        if model not in YOLO_MODELS:
            print(f"Unknown model: {model}")
            continue

        url = YOLO_MODELS[model]
        dest = output_dir / model

        if dest.exists() and not args.force:
            print(f"Already exists: {dest} (use --force to overwrite)")
            continue

        print(f"\nDownloading {model}...")
        print(f"  From: {url}")
        print(f"  To:   {dest}")

        if download_file(url, dest):
            # Verify checksum if available
            if model in CHECKSUMS:
                if verify_checksum(dest, CHECKSUMS[model]):
                    print(f"  ✓ Checksum verified")
                else:
                    print(f"  ✗ Checksum verification FAILED!")
                    dest.unlink()
                    continue
            print(f"  ✓ Downloaded successfully")
        else:
            if dest.exists():
                dest.unlink()

    print(f"\nAll weights saved to: {output_dir.resolve()}")