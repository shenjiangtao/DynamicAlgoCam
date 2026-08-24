#!/usr/bin/env python3
# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# quick_train.py — Quick start training for common scenarios.

import argparse
import os
import sys
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Quick start training for mosquito/weed detection")
    parser.add_argument("task", type=str, choices=["mosquito", "weed", "custom"],
                        help="Training task preset")
    parser.add_argument("--data", type=str, required=True, help="Dataset root directory")
    parser.add_argument("--epochs", type=int, default=100, help="Training epochs")
    parser.add_argument("--batch", type=int, default=16, help="Batch size")
    parser.add_argument("--imgsz", type=int, default=640, help="Image size")
    parser.add_argument("--device", type=str, default="0", help="CUDA device (0, 1, cpu)")
    parser.add_argument("--model-size", type=str, default="n", choices=["n", "s", "m", "l", "x"],
                        help="YOLOv11 model size (n/s/m/l/x)")
    parser.add_argument("--export-onnx", action="store_true", help="Export ONNX after training")
    parser.add_argument("--convert-rknn", action="store_true", help="Convert to RKNN")
    parser.add_argument("--dry-run", action="store_true", help="Print commands only")
    return parser.parse_args()


PRESETS = {
    "mosquito": {
        "model": "yolo11n.pt",
        "config": "configs/yolo11_mosquito.yaml",
        "classes": ["mosquito", "fly", "other_insect"],
        "imgsz": 800,  # Higher resolution for small objects
    },
    "weed": {
        "model": "yolo11s-seg.pt",
        "config": "configs/yolo11_weed.yaml",
        "classes": ["weed", "crop"],
        "imgsz": 640,
    },
    "custom": {
        "model": "yolo11n.pt",
        "config": "",
        "classes": [],
        "imgsz": 640,
    }
}


def main():
    args = parse_args()
    preset = PRESETS[args.task]

    # Build dataset.yaml path
    data_yaml = Path(args.data) / "dataset.yaml"
    if not data_yaml.exists():
        # Try to create from class names
        class_names = preset["classes"]
        if not class_names:
            print("Error: For custom task, dataset.yaml must exist or provide --classes")
            exit(1)

        # Check if dataset structure exists
        for split in ["train", "val", "test"]:
            img_dir = Path(args.data) / "images" / split
            lbl_dir = Path(args.data) / "labels" / split
            if not img_dir.exists():
                print(f"Warning: {img_dir} not found")

        # Create dataset.yaml
        import yaml
        cfg = {
            "path": str(Path(args.data).resolve()),
            "train": "images/train",
            "val": "images/val",
            "test": "images/test",
            "nc": len(class_names),
            "names": {i: name for i, name in enumerate(class_names)}
        }
        data_yaml.parent.mkdir(parents=True, exist_ok=True)
        with open(data_yaml, "w") as f:
            yaml.dump(cfg, f, default_flow_style=False)
        print(f"Created dataset.yaml: {data_yaml}")

    if not data_yaml.exists():
        print(f"Error: dataset.yaml not found at {data_yaml}")
        print("Run: python prepare_dataset.py first, or ensure dataset.yaml exists")
        exit(1)

    # Determine model
    model = preset["model"]
    if args.model_size != "n" and "n" in model:
        model = model.replace("n", args.model_size)

    # Determine config
    config = preset["config"]
    if args.task == "custom" and not config:
        config = ""

    # Build command
    cmd = [
        sys.executable, "train.py",
        "--data", str(data_yaml),
        "--model", model,
        "--epochs", str(args.epochs),
        "--batch", str(args.batch),
        "--imgsz", str(preset["imgsz"] if args.task != "custom" else args.imgsz),
        "--device", args.device,
        "--project", "runs/train",
        "--name", f"{args.task}_{args.model_size}",
    ]

    if config:
        cmd.extend(["--config", config])

    if args.export_onnx:
        cmd.append("--export-onnx")

    if args.convert_rknn:
        cmd.extend(["--convert-rknn", "--export-onnx"])

    if args.dry_run:
        print("Would run:")
        print(" ".join(cmd))
        return

    print(f"Starting {args.task} training with {model}")
    print(f"Command: {' '.join(cmd)}")

    # Execute
    os.chdir(Path(__file__).parent)
    result = os.system(" ".join(cmd))
    sys.exit(result)


if __name__ == "__main__":
    main()