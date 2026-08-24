#!/usr/bin/env python3
# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# train.py — Complete training pipeline orchestrator.

import argparse
import os
import subprocess
import sys
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Complete training pipeline")
    parser.add_argument("--data", type=str, required=True, help="Dataset YAML config")
    parser.add_argument("--model", type=str, default="yolo11n.pt", help="Base model")
    parser.add_argument("--epochs", type=int, default=100, help="Training epochs")
    parser.add_argument("--batch", type=int, default=16, help="Batch size")
    parser.add_argument("--imgsz", type=int, default=640, help="Image size")
    parser.add_argument("--device", type=str, default="0", help="CUDA device (0, 1, cpu)")
    parser.add_argument("--project", type=str, default="runs/train", help="Project directory")
    parser.add_argument("--name", type=str, default="exp", help="Experiment name")
    parser.add_argument("--config", type=str, default="", help="Hyperparameter config YAML")
    parser.add_argument("--pretrained", action="store_true", help="Use pretrained weights")
    parser.add_argument("--resume", type=str, default="", help="Resume from checkpoint")
    parser.add_argument("--workers", type=int, default=8, help="Data loader workers")
    parser.add_argument("--amp", action="store_true", default=True, help="Automatic mixed precision")
    parser.add_argument("--cache", action="store_true", help="Cache images in RAM")
    parser.add_argument("--export-onnx", action="store_true", help="Export to ONNX after training")
    parser.add_argument("--onnx-opset", type=int, default=12, help="ONNX opset version")
    parser.add_argument("--onnx-simplify", action="store_true", default=True, help="Simplify ONNX")
    parser.add_argument("--onnx-half", action="store_true", help="FP16 export")
    parser.add_argument("--convert-rknn", action="store_true", help="Convert to RKNN after ONNX")
    parser.add_argument("--rknn-platform", type=str, default="rk3588", help="RKNN target platform")
    parser.add_argument("--validate", action="store_true", help="Run validation after training")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without executing")
    return parser.parse_args()


def run_cmd(cmd, dry_run=False):
    """Run command and return success status."""
    print(f"\n>>> {' '.join(cmd)}")
    if dry_run:
        return True
    result = subprocess.run(cmd, capture_output=False)
    return result.returncode == 0


def main():
    args = parse_args()

    # Check dataset config
    if not os.path.exists(args.data):
        print(f"Dataset config not found: {args.data}")
        exit(1)

    # Load dataset config to get class names
    import yaml
    with open(args.data) as f:
        data_cfg = yaml.safe_load(f)
    class_names = data_cfg.get("names", {})
    nc = data_cfg.get("nc", len(class_names))

    print("=" * 60)
    print("DYNAMICALGOCAM TRAINING PIPELINE")
    print("=" * 60)
    print(f"Dataset: {args.data}")
    print(f"Classes ({nc}): {list(class_names.values())}")
    print(f"Model: {args.model}")
    print(f"Epochs: {args.epochs}, Batch: {args.batch}, Img size: {args.imgsz}")
    print(f"Device: {args.device}")
    print(f"Project: {args.project}/{args.name}")
    if args.config:
        print(f"Config: {args.config}")

    # Step 1: Train
    train_cmd = [
        sys.executable, "train_yolo.py",
        "--data", args.data,
        "--model", args.model,
        "--epochs", str(args.epochs),
        "--batch", str(args.batch),
        "--imgsz", str(args.imgsz),
        "--device", args.device,
        "--project", args.project,
        "--name", args.name,
        "--workers", str(args.workers),
        "--amp" if args.amp else "--no-amp",
    ]
    if args.config:
        train_cmd.extend(["--config", args.config])
    if args.pretrained:
        train_cmd.append("--pretrained")
    if args.resume:
        train_cmd.extend(["--resume", args.resume])
    if args.cache:
        train_cmd.append("--cache")

    if not run_cmd(train_cmd, args.dry_run):
        print("Training failed!")
        exit(1)

    # Find best model
    best_model = Path(args.project) / args.name / "weights" / "best.pt"
    if not best_model.exists():
        # Try last.pt
        best_model = Path(args.project) / args.name / "weights" / "last.pt"
    if not best_model.exists():
        print(f"Error: No trained model found in {args.project}/{args.name}/weights/")
        exit(1)

    print(f"\nBest model: {best_model}")

    # Step 2: Validate (if requested)
    if args.validate:
        val_cmd = [
            sys.executable, "validate_model.py",
            "--model", str(best_model.with_suffix(".onnx") if args.export_onnx else best_model),
            "--data", args.data,
            "--imgsz", str(args.imgsz), str(args.imgsz),
        ]
        if not run_cmd(val_cmd, args.dry_run):
            print("Validation failed!")
            exit(1)

    # Step 3: Export ONNX (if requested)
    if args.export_onnx:
        onnx_path = best_model.with_suffix(".onnx")
        export_cmd = [
            sys.executable, "export_onnx.py",
            "--weights", str(best_model),
            "--imgsz", str(args.imgsz), str(args.imgsz),
            "--opset", str(args.onnx_opset),
            "--simplify" if args.onnx_simplify else "--no-simplify",
            "--output", str(onnx_path),
            "--nms",  # Disable NMS for C++ inference
        ]
        if args.onnx_half:
            export_cmd.append("--half")
        if args.dynamic:
            export_cmd.append("--dynamic")

        if not run_cmd(export_cmd, args.dry_run):
            print("ONNX export failed!")
            exit(1)

        print(f"ONNX model: {onnx_path}")

        # Step 4: Convert to RKNN (if requested)
        if args.convert_rknn:
            rknn_path = onnx_path.with_suffix(".rknn")
            convert_cmd = [
                sys.executable, "convert_rknn.py",
                "--model", str(onnx_path),
                "--platform", args.rknn_platform,
                "--output", str(rknn_path),
            ]
            if not run_cmd(convert_cmd, args.dry_run):
                print("RKNN conversion failed!")
                exit(1)
            print(f"RKNN model: {rknn_path}")

    print("\n" + "=" * 60)
    print("PIPELINE COMPLETED SUCCESSFULLY")
    print("=" * 60)
    print(f"Best PyTorch model: {best_model}")
    if args.export_onnx:
        print(f"ONNX model: {best_model.with_suffix('.onnx')}")
    if args.convert_rknn:
        print(f"RKNN model: {best_model.with_suffix('.rknn')}")


if __name__ == "__main__":
    main()