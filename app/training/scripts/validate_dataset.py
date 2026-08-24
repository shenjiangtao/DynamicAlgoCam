#!/usr/bin/env python3
# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# validate_dataset.py — Validate YOLO dataset structure and annotations.

import argparse
import os
import sys
from pathlib import Path
from collections import Counter
import cv2
import yaml
from tqdm import tqdm


def parse_args():
    parser = argparse.ArgumentParser(description="Validate YOLO dataset")
    parser.add_argument("--data", type=str, required=True, help="Dataset YAML config")
    parser.add_argument("--check-images", action="store_true", help="Verify image files exist and are readable")
    parser.add_argument("--check-labels", action="store_true", help="Verify label files and format")
    parser.add_argument("--check-classes", action="store_true", help="Verify class IDs match dataset.yaml")
    parser.add_argument("--stats", action="store_true", help="Print dataset statistics")
    parser.add_argument("--fix", action="store_true", help="Attempt to fix common issues")
    return parser.parse_args()


def load_dataset_yaml(data_path):
    with open(data_path, 'r') as f:
        return yaml.safe_load(f)


def validate_structure(data_cfg):
    """Check dataset directory structure."""
    errors = []
    warnings = []

    base_path = Path(data_cfg['path'])
    splits = ['train', 'val', 'test']

    for split in splits:
        if split not in data_cfg or not data_cfg[split]:
            warnings.append(f"Split '{split}' not defined in dataset.yaml")
            continue

        img_dir = base_path / data_cfg[split]
        lbl_dir = base_path / data_cfg[split].replace('images', 'labels')

        if not img_dir.exists():
            errors.append(f"Images directory not found: {img_dir}")
        if not lbl_dir.exists():
            warnings.append(f"Labels directory not found: {lbl_dir}")

    return errors, warnings


def validate_labels(data_cfg, class_names, check_images=False, fix=False):
    """Validate label files."""
    errors = []
    warnings = []
    stats = Counter()
    class_counts = Counter()

    base_path = Path(data_cfg['path'])
    splits = ['train', 'val', 'test']
    valid_classes = set(range(len(class_names)))

    for split in splits:
        if split not in data_cfg or not data_cfg[split]:
            continue

        lbl_dir = base_path / data_cfg[split].replace('images', 'labels')
        img_dir = base_path / data_cfg[split]

        if not lbl_dir.exists():
            continue

        label_files = list(lbl_dir.glob("*.txt"))
        for label_file in tqdm(label_files, desc=f"Checking {split} labels"):
            try:
                with open(label_file) as f:
                    lines = f.readlines()

                if not lines:
                    warnings.append(f"Empty label file: {label_file.relative_to(base_path)}")
                    if fix:
                        label_file.unlink()
                    continue

                for line_num, line in enumerate(lines, 1):
                    parts = line.strip().split()
                    if len(parts) < 5:
                        errors.append(f"{label_file}:{line_num} - Invalid format (expected 5 values, got {len(parts)})")
                        continue

                    try:
                        cls_id = int(parts[0])
                        x, y, w, h = map(float, parts[1:5])

                        # Check class ID
                        if cls_id not in valid_classes:
                            errors.append(f"{label_file}:{line_num} - Invalid class ID: {cls_id} (valid: 0-{len(class_names)-1})")
                        else:
                            class_counts[cls_id] += 1

                        # Check bbox values (should be normalized 0-1)
                        for val, name in [(x, 'x'), (y, 'y'), (w, 'width'), (h, 'height')]:
                            if val < 0 or val > 1:
                                errors.append(f"{label_file}:{line_num} - {name} out of range [0,1]: {val}")

                        # Check for zero width/height
                        if w <= 0 or h <= 0:
                            errors.append(f"{label_file}:{line_num} - Zero or negative width/height")

                    except ValueError as e:
                        errors.append(f"{label_file}:{line_num} - Parse error: {e}")

                    stats['total_boxes'] += 1

                # Check corresponding image exists
                if check_images:
                    img_file = img_dir / (label_file.stem + ".jpg")
                    if not img_file.exists():
                        # Try other extensions
                        found = False
                        for ext in [".png", ".jpeg", ".bmp"]:
                            if (img_dir / (label_file.stem + ext)).exists():
                                found = True
                                break
                        if not found:
                            warnings.append(f"Image not found for label: {label_file.name}")

            except Exception as e:
                errors.append(f"Failed to read {label_file}: {e}")

    return errors, warnings, stats, class_counts


def validate_images(data_cfg):
    """Verify image files are readable."""
    errors = []
    warnings = []
    base_path = Path(data_cfg['path'])

    splits = ['train', 'val', 'test']
    for split in splits:
        if split not in data_cfg or not data_cfg[split]:
            continue

        img_dir = base_path / data_cfg[split]
        if not img_dir.exists():
            continue

        img_files = list(img_dir.glob("*.jpg")) + list(img_dir.glob("*.png")) + \
                    list(img_dir.glob("*.jpeg")) + list(img_dir.glob("*.bmp"))

        for img_file in tqdm(img_files, desc=f"Checking {split} images"):
            try:
                img = cv2.imread(str(img_file))
                if img is None:
                    errors.append(f"Cannot read image: {img_file.relative_to(base_path)}")
                elif img.size == 0:
                    errors.append(f"Empty image: {img_file.relative_to(base_path)}")
            except Exception as e:
                errors.append(f"Error reading {img_file}: {e}")

    return errors, warnings


def print_stats(data_cfg, stats, class_counts, class_names):
    """Print dataset statistics."""
    print("\n" + "=" * 50)
    print("DATASET STATISTICS")
    print("=" * 50)
    print(f"Total bounding boxes: {stats['total_boxes']}")

    if class_counts:
        print("\nClass distribution:")
        for cls_id, count in sorted(class_counts.items()):
            name = class_names[cls_id] if cls_id < len(class_names) else f"class_{cls_id}"
            print(f"  {cls_id} ({name}): {count}")

    base_path = Path(data_cfg['path'])
    splits = ['train', 'val', 'test']
    print("\nSplit sizes:")
    for split in splits:
        if split not in data_cfg or not data_cfg[split]:
            continue
        img_dir = base_path / data_cfg[split]
        lbl_dir = base_path / data_cfg[split].replace('images', 'labels')
        n_img = len(list(img_dir.glob("*.jpg")) + list(img_dir.glob("*.png"))) if img_dir.exists() else 0
        n_lbl = len(list(lbl_dir.glob("*.txt"))) if lbl_dir.exists() else 0
        print(f"  {split}: {n_img} images, {n_lbl} labels")


def main():
    args = parse_args()

    if not os.path.exists(args.data):
        print(f"Dataset config not found: {args.data}")
        exit(1)

    data_cfg = load_dataset_yaml(args.data)
    class_names = [data_cfg['names'][i] for i in range(data_cfg['nc'])]

    print(f"Validating dataset: {args.data}")
    print(f"Classes ({data_cfg['nc']}): {class_names}")
    print()

    all_errors = []
    all_warnings = []

    # Structure validation
    errors, warnings = validate_structure(data_cfg)
    all_errors.extend(errors)
    all_warnings.extend(warnings)

    # Label validation
    if args.check_labels or args.check_classes:
        errors, warnings, stats, class_counts = validate_labels(
            data_cfg, class_names, args.check_images, args.fix
        )
        all_errors.extend(errors)
        all_warnings.extend(warnings)

        if args.stats:
            print_stats(data_cfg, stats, class_counts, class_names)

    # Image validation
    if args.check_images:
        errors, warnings = validate_images(data_cfg)
        all_errors.extend(errors)
        all_warnings.extend(warnings)

    # Summary
    print("\n" + "=" * 50)
    print("VALIDATION SUMMARY")
    print("=" * 50)

    if all_errors:
        print(f"\nERRORS ({len(all_errors)}):")
        for err in all_errors[:20]:  # Limit output
            print(f"  ✗ {err}")
        if len(all_errors) > 20:
            print(f"  ... and {len(all_errors) - 20} more errors")
    else:
        print("\nNo errors found ✓")

    if all_warnings:
        print(f"\nWARNINGS ({len(all_warnings)}):")
        for warn in all_warnings[:20]:
            print(f"  ⚠ {warn}")
        if len(all_warnings) > 20:
            print(f"  ... and {len(all_warnings) - 20} more warnings")

    if not all_errors and not all_warnings:
        print("\nAll checks passed! ✓")

    if args.fix and all_errors:
        print("\nNote: --fix only removes empty label files. Other errors need manual fixing.")

    sys.exit(1 if all_errors else 0)


if __name__ == "__main__":
    main()