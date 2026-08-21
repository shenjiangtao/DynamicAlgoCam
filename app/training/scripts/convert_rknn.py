#!/usr/bin/env python3
# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# convert_rknn.py — Convert ONNX model to RKNN format for RK3588.

import argparse
import os
import sys

try:
    from rknn.api import RKNN
except ImportError:
    print("Error: rknn-toolkit2 not installed. Install from Rockchip SDK.")
    exit(1)


def parse_args():
    parser = argparse.ArgumentParser(description="Convert ONNX to RKNN")
    parser.add_argument("--model", type=str, required=True, help="ONNX model path")
    parser.add_argument("--platform", type=str, default="rk3588",
                        choices=["rk3588", "rk3568", "rk3566", "rv1126"],
                        help="Target platform")
    parser.add_argument("--dataset", type=str, default="", help="Quantization dataset (txt file with image paths)")
    parser.add_argument("--do-quant", action="store_true", help="Enable INT8 quantization")
    parser.add_argument("--imgsz", type=int, nargs=2, default=[640, 640], help="Input image size")
    parser.add_argument("--mean", type=float, nargs=3, default=[0, 0, 0], help="Input mean")
    parser.add_argument("--std", type=float, nargs=3, default=[255, 255, 255], help="Input std")
    parser.add_argument("--output", type=str, default="", help="Output RKNN path")
    return parser.parse_args()


def main():
    args = parse_args()

    if not os.path.exists(args.model):
        print(f"Model not found: {args.model}")
        exit(1)

    if args.do_quant and not args.dataset:
        print("Quantization requires --dataset (txt file with image paths)")
        exit(1)

    output_path = args.output or args.model.replace(".onnx", ".rknn")

    rknn = RKNN(verbose=True)

    # Configure
    print("Configuring RKNN...")
    rknn.config(
        mean_values=[args.mean],
        std_values=[args.std],
        target_platform=args.platform,
        optimization_level=3,
        quantized_algorithm="normal",
        quantized_method="channel",
        float_dtype="float16" if not args.do_quant else "float32",
    )

    # Load ONNX
    print(f"Loading ONNX: {args.model}")
    ret = rknn.load_onnx(model=args.model)
    if ret != 0:
        print(f"Load ONNX failed: {ret}")
        exit(1)

    # Build
    print("Building RKNN model...")
    if args.do_quant:
        print(f"Quantizing with dataset: {args.dataset}")
        ret = rknn.build(do_quantization=True, dataset=args.dataset)
    else:
        ret = rknn.build(do_quantization=False)

    if ret != 0:
        print(f"Build failed: {ret}")
        exit(1)

    # Export
    print(f"Exporting RKNN: {output_path}")
    ret = rknn.export_rknn(output_path)
    if ret != 0:
        print(f"Export failed: {ret}")
        exit(1)

    print(f"Success! RKNN model saved to: {output_path}")

    # Optional: accuracy analysis
    if args.do_quant and args.dataset:
        print("Running accuracy analysis...")
        rknn.accuracy_analysis(inputs=[args.dataset])

    rknn.release()


if __name__ == "__main__":
    main()