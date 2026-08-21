#!/usr/bin/env python3
# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# train_yolo.py — Train YOLO model for mosquito/weed detection.

import argparse
import os
import yaml
from pathlib import Path

try:
    from ultralytics import YOLO
except ImportError:
    print("Error: ultralytics not installed. Run: pip install ultralytics")
    exit(1)


def parse_args():
    parser = argparse.ArgumentParser(description="Train YOLO for mosquito/weed detection")
    parser.add_argument("--data", type=str, required=True, help="Dataset YAML config")
    parser.add_argument("--model", type=str, default="yolo11n.pt", help="Base model (yolo11n/s/m/l/x.pt)")
    parser.add_argument("--epochs", type=int, default=100, help="Training epochs")
    parser.add_argument("--batch", type=int, default=16, help="Batch size")
    parser.add_argument("--imgsz", type=int, default=640, help="Image size")
    parser.add_argument("--device", type=str, default="0", help="CUDA device (0, 1, cpu)")
    parser.add_argument("--project", type=str, default="runs/train", help="Project directory")
    parser.add_argument("--name", type=str, default="exp", help="Experiment name")
    parser.add_argument("--patience", type=int, default=20, help="Early stopping patience")
    parser.add_argument("--pretrained", action="store_true", help="Use pretrained weights")
    parser.add_argument("--resume", type=str, default="", help="Resume from checkpoint")
    parser.add_argument("--workers", type=int, default=8, help="Data loader workers")
    parser.add_argument("--amp", action="store_true", default=True, help="Automatic mixed precision")
    parser.add_argument("--cache", action="store_true", help="Cache images in RAM")
    return parser.parse_args()


def create_dataset_yaml(data_dir: str, class_names: list, output_path: str):
    """Create dataset.yaml for YOLO training."""
    data = {
        "path": data_dir,
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "nc": len(class_names),
        "names": {i: name for i, name in enumerate(class_names)}
    }
    with open(output_path, 'w') as f:
        yaml.dump(data, f)
    print(f"Created dataset config: {output_path}")


def main():
    args = parse_args()

    # Validate dataset
    if not os.path.exists(args.data):
        print(f"Dataset config not found: {args.data}")
        exit(1)

    # Load model
    model = YOLO(args.model)

    # Train
    print(f"Starting training: {args.name}")
    print(f"Model: {args.model}, Epochs: {args.epochs}, Batch: {args.batch}, Image size: {args.imgsz}")

    results = model.train(
        data=args.data,
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=args.device,
        project=args.project,
        name=args.name,
        patience=args.patience,
        pretrained=args.pretrained,
        resume=args.resume if args.resume else False,
        workers=args.workers,
        amp=args.amp,
        cache=args.cache,
        save=True,
        save_period=10,
        val=True,
        plots=True,
    )

    print(f"Training completed. Best model: {results.best}")

    # Validate
    print("Running validation...")
    metrics = model.val()
    print(f"mAP50: {metrics.box.map50:.4f}, mAP50-95: {metrics.box.map:.4f}")


if __name__ == "__main__":
    main()