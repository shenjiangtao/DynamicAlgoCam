#!/usr/bin/env python3
# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# export_onnx.py — Export trained YOLO model to ONNX format.

import argparse
import os
import sys

try:
    from ultralytics import YOLO
except ImportError:
    print("Error: ultralytics not installed. Run: pip install ultralytics")
    exit(1)


def parse_args():
    parser = argparse.ArgumentParser(description="Export YOLO model to ONNX")
    parser.add_argument("--weights", type=str, required=True, help="Path to trained .pt weights")
    parser.add_argument("--imgsz", type=int, nargs=2, default=[640, 640], help="Image size H W")
    parser.add_argument("--batch", type=int, default=1, help="Batch size")
    parser.add_argument("--dynamic", action="store_true", help="Dynamic batch/axes")
    parser.add_argument("--simplify", action="store_true", default=True, help="Simplify ONNX model")
    parser.add_argument("--opset", type=int, default=12, help="ONNX opset version")
    parser.add_argument("--half", action="store_true", help="FP16 export")
    parser.add_argument("--nms", action="store_false", dest="nms", help="Disable NMS in export (do NMS in C++)")
    parser.add_argument("--output", type=str, default="", help="Output ONNX path")
    return parser.parse_args()


def main():
    args = parse_args()

    if not os.path.exists(args.weights):
        print(f"Weights not found: {args.weights}")
        exit(1)

    model = YOLO(args.weights)

    # Export
    output_path = args.output or args.weights.replace(".pt", ".onnx")

    print(f"Exporting {args.weights} -> {output_path}")
    print(f"Image size: {args.imgsz}, Batch: {args.batch}, Dynamic: {args.dynamic}")
    print(f"Simplify: {args.simplify}, Opset: {args.opset}, Half: {args.half}")
    print(f"NMS included: {args.nms} (recommend False for C++ TensorRT)")

    try:
        exported = model.export(
            format="onnx",
            imgsz=args.imgsz,
            batch=args.batch,
            dynamic=args.dynamic,
            simplify=args.simplify,
            opset=args.opset,
            half=args.half,
            nms=args.nms,
        )
        print(f"Export successful: {exported}")
    except Exception as e:
        print(f"Export failed: {e}")
        exit(1)


if __name__ == "__main__":
    main()