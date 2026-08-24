#!/usr/bin/env python3
# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# prepare_dataset.py — Convert various annotation formats to YOLO format.

import argparse
import os
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from tqdm import tqdm
import yaml


def parse_args():
    parser = argparse.ArgumentParser(description="Convert dataset annotations to YOLO format")
    parser.add_argument("--input", type=str, required=True, help="Input dataset directory")
    parser.add_argument("--output", type=str, required=True, help="Output YOLO dataset directory")
    parser.add_argument("--format", type=str, required=True,
                        choices=["coco", "voc", "labelme", "yolo"],
                        help="Input annotation format")
    parser.add_argument("--split", type=str, default="train,val,test",
                        help="Comma-separated list of splits to process")
    parser.add_argument("--classes", type=str, default="",
                        help="Comma-separated class names (optional, auto-detected if empty)")
    parser.add_argument("--copy-images", action="store_true",
                        help="Copy images to output directory (default: symlink)")
    parser.add_argument("--val-split", type=float, default=0.2,
                        help="Validation split ratio (if input has no splits)")
    parser.add_argument("--test-split", type=float, default=0.1,
                        help="Test split ratio (if input has no splits)")
    return parser.parse_args()


def create_yolo_dirs(output_dir, splits):
    """Create YOLO directory structure."""
    for split in splits:
        (Path(output_dir) / "images" / split).mkdir(parents=True, exist_ok=True)
        (Path(output_dir) / "labels" / split).mkdir(parents=True, exist_ok=True)


def get_class_names(classes_arg, annotations, fmt):
    """Get class names from args or annotations."""
    if classes_arg:
        return [c.strip() for c in classes_arg.split(",")]

    class_names = set()
    if fmt == "coco":
        for ann in annotations.get("categories", []):
            class_names.add(ann["name"])
    elif fmt == "voc":
        for ann in annotations:
            for obj in ann.findall(".//object"):
                class_names.add(obj.find("name").text)
    elif fmt == "labelme":
        for ann in annotations:
            for shape in ann.get("shapes", []):
                class_names.add(shape["label"])

    return sorted(list(class_names))


def convert_coco(input_dir, output_dir, splits, class_names, copy_images):
    """Convert COCO format to YOLO."""
    for split in splits:
        ann_file = Path(input_dir) / "annotations" / f"instances_{split}.json"
        if not ann_file.exists():
            print(f"Warning: {ann_file} not found, skipping {split}")
            continue

        with open(ann_file) as f:
            coco = json.load(f)

        # Build category mapping
        cat_id_to_idx = {cat["id"]: idx for idx, cat in enumerate(sorted(coco["categories"], key=lambda x: x["id"]))}

        # Group annotations by image
        img_to_anns = {}
        for ann in coco["annotations"]:
            img_id = ann["image_id"]
            if img_id not in img_to_anns:
                img_to_anns[img_id] = []
            img_to_anns[img_id].append(ann)

        # Process images
        for img_info in tqdm(coco["images"], desc=f"Converting {split}"):
            img_id = img_info["id"]
            img_name = img_info["file_name"]
            img_w = img_info["width"]
            img_h = img_info["height"]

            # Copy or symlink image
            src_img = Path(input_dir) / split / img_name
            if not src_img.exists():
                # Try alternative locations
                for alt in [Path(input_dir) / "images" / split / img_name,
                           Path(input_dir) / img_name]:
                    if alt.exists():
                        src_img = alt
                        break

            if not src_img.exists():
                print(f"Warning: Image not found: {img_name}")
                continue

            dst_img = Path(output_dir) / "images" / split / img_name
            if copy_images:
                shutil.copy2(src_img, dst_img)
            else:
                try:
                    dst_img.symlink_to(src_img.resolve())
                except FileExistsError:
                    pass

            # Write YOLO label
            label_path = Path(output_dir) / "labels" / split / (Path(img_name).stem + ".txt")
            with open(label_path, "w") as f:
                for ann in img_to_anns.get(img_id, []):
                    cat_idx = cat_id_to_idx[ann["category_id"]]
                    bbox = ann["bbox"]  # [x, y, w, h] in pixels
                    # Convert to YOLO format (normalized center x, y, w, h)
                    x_center = (bbox[0] + bbox[2] / 2) / img_w
                    y_center = (bbox[1] + bbox[3] / 2) / img_h
                    w_norm = bbox[2] / img_w
                    h_norm = bbox[3] / img_h
                    f.write(f"{cat_idx} {x_center:.6f} {y_center:.6f} {w_norm:.6f} {h_norm:.6f}\n")


def convert_voc(input_dir, output_dir, splits, class_names, copy_images):
    """Convert VOC format to YOLO."""
    class_to_idx = {name: idx for idx, name in enumerate(class_names)}

    for split in splits:
        split_file = Path(input_dir) / "ImageSets" / "Main" / f"{split}.txt"
        if not split_file.exists():
            print(f"Warning: {split_file} not found, skipping {split}")
            continue

        with open(split_file) as f:
            img_ids = [line.strip() for line in f if line.strip()]

        for img_id in tqdm(img_ids, desc=f"Converting {split}"):
            # Find image
            img_exts = [".jpg", ".jpeg", ".png", ".bmp"]
            src_img = None
            for ext in img_exts:
                p = Path(input_dir) / "JPEGImages" / f"{img_id}{ext}"
                if p.exists():
                    src_img = p
                    break

            if not src_img:
                print(f"Warning: Image not found for {img_id}")
                continue

            # Parse XML
            xml_file = Path(input_dir) / "Annotations" / f"{img_id}.xml"
            if not xml_file.exists():
                print(f"Warning: Annotation not found for {img_id}")
                continue

            tree = ET.parse(xml_file)
            root = tree.getroot()
            size = root.find("size")
            img_w = int(size.find("width").text)
            img_h = int(size.find("height").text)

            # Copy/symlink image
            dst_img = Path(output_dir) / "images" / split / src_img.name
            if copy_images:
                shutil.copy2(src_img, dst_img)
            else:
                try:
                    dst_img.symlink_to(src_img.resolve())
                except FileExistsError:
                    pass

            # Write YOLO label
            label_path = Path(output_dir) / "labels" / split / f"{img_id}.txt"
            with open(label_path, "w") as f:
                for obj in root.findall("object"):
                    cls_name = obj.find("name").text
                    if cls_name not in class_to_idx:
                        continue
                    cls_idx = class_to_idx[cls_name]

                    bbox = obj.find("bndbox")
                    xmin = float(bbox.find("xmin").text)
                    ymin = float(bbox.find("ymin").text)
                    xmax = float(bbox.find("xmax").text)
                    ymax = float(bbox.find("ymax").text)

                    # Convert to YOLO format
                    x_center = (xmin + xmax) / 2 / img_w
                    y_center = (ymin + ymax) / 2 / img_h
                    w_norm = (xmax - xmin) / img_w
                    h_norm = (ymax - ymin) / img_h
                    f.write(f"{cls_idx} {x_center:.6f} {y_center:.6f} {w_norm:.6f} {h_norm:.6f}\n")


def convert_labelme(input_dir, output_dir, splits, class_names, copy_images):
    """Convert LabelMe format to YOLO."""
    class_to_idx = {name: idx for idx, name in enumerate(class_names)}

    # LabelMe typically has all JSON files in one directory
    json_files = list(Path(input_dir).glob("*.json"))
    if not json_files:
        print(f"No JSON files found in {input_dir}")
        return

    # Split files
    import random
    random.shuffle(json_files)
    n_total = len(json_files)
    n_val = int(n_total * 0.2)
    n_test = int(n_total * 0.1)
    n_train = n_total - n_val - n_test

    split_files = {
        "train": json_files[:n_train],
        "val": json_files[n_train:n_train + n_val],
        "test": json_files[n_train + n_val:]
    }

    for split in splits:
        for json_file in tqdm(split_files.get(split, []), desc=f"Converting {split}"):
            with open(json_file) as f:
                data = json.load(f)

            img_name = data.get("imagePath", json_file.stem + ".jpg")
            img_h = data.get("imageHeight", 0)
            img_w = data.get("imageWidth", 0)

            # Find image
            src_img = Path(input_dir) / img_name
            if not src_img.exists():
                print(f"Warning: Image not found: {img_name}")
                continue

            dst_img = Path(output_dir) / "images" / split / img_name
            if copy_images:
                shutil.copy2(src_img, dst_img)
            else:
                try:
                    dst_img.symlink_to(src_img.resolve())
                except FileExistsError:
                    pass

            # Write YOLO label
            label_path = Path(output_dir) / "labels" / split / (Path(img_name).stem + ".txt")
            with open(label_path, "w") as f:
                for shape in data.get("shapes", []):
                    if shape["shape_type"] != "rectangle":
                        continue
                    cls_name = shape["label"]
                    if cls_name not in class_to_idx:
                        continue
                    cls_idx = class_to_idx[cls_name]

                    points = shape["points"]
                    xmin = min(p[0] for p in points)
                    ymin = min(p[1] for p in points)
                    xmax = max(p[0] for p in points)
                    ymax = max(p[1] for p in points)

                    x_center = (xmin + xmax) / 2 / img_w
                    y_center = (ymin + ymax) / 2 / img_h
                    w_norm = (xmax - xmin) / img_w
                    h_norm = (ymax - ymin) / img_h
                    f.write(f"{cls_idx} {x_center:.6f} {y_center:.6f} {w_norm:.6f} {h_norm:.6f}\n")


def create_dataset_yaml(output_dir, class_names, splits):
    """Create dataset.yaml for YOLO training."""
    data = {
        "path": str(Path(output_dir).resolve()),
        "train": "images/train" if "train" in splits else "",
        "val": "images/val" if "val" in splits else "",
        "test": "images/test" if "test" in splits else "",
        "nc": len(class_names),
        "names": {i: name for i, name in enumerate(class_names)}
    }
    yaml_path = Path(output_dir) / "dataset.yaml"
    with open(yaml_path, "w") as f:
        yaml.dump(data, f, default_flow_style=False)
    print(f"Created dataset config: {yaml_path}")


def main():
    args = parse_args()
    splits = [s.strip() for s in args.split.split(",")]

    if not os.path.exists(args.input):
        print(f"Input directory not found: {args.input}")
        exit(1)

    Path(args.output).mkdir(parents=True, exist_ok=True)
    create_yolo_dirs(args.output, splits)

    # Determine class names
    class_names = []
    if args.classes:
        class_names = [c.strip() for c in args.classes.split(",")]

    # Convert based on format
    if args.format == "coco":
        # Need to load annotations first to get class names if not provided
        if not class_names:
            # Try to load from first split
            for split in splits:
                ann_file = Path(args.input) / "annotations" / f"instances_{split}.json"
                if ann_file.exists():
                    with open(ann_file) as f:
                        coco = json.load(f)
                    class_names = sorted([cat["name"] for cat in coco["categories"]])
                    break

        if not class_names:
            print("Error: Could not determine class names. Use --classes argument.")
            exit(1)

        convert_coco(args.input, args.output, splits, class_names, args.copy_images)

    elif args.format == "voc":
        if not class_names:
            # Scan all XML files to get class names
            class_names = set()
            for xml_file in Path(args.input).glob("Annotations/*.xml"):
                tree = ET.parse(xml_file)
                for obj in tree.getroot().findall("object"):
                    class_names.add(obj.find("name").text)
            class_names = sorted(class_names)

        if not class_names:
            print("Error: No classes found in VOC annotations.")
            exit(1)

        convert_voc(args.input, args.output, splits, class_names, args.copy_images)

    elif args.format == "labelme":
        if not class_names:
            # Scan all JSON files to get class names
            class_names = set()
            for json_file in Path(args.input).glob("*.json"):
                with open(json_file) as f:
                    data = json.load(f)
                for shape in data.get("shapes", []):
                    class_names.add(shape["label"])
            class_names = sorted(class_names)

        if not class_names:
            print("Error: No classes found in LabelMe annotations.")
            exit(1)

        convert_labelme(args.input, args.output, splits, class_names, args.copy_images)

    elif args.format == "yolo":
        # Already in YOLO format, just copy/symlink and create yaml
        print("Input already in YOLO format, creating dataset.yaml...")
        # Find class names from existing labels
        class_names = set()
        for split in splits:
            label_dir = Path(args.input) / "labels" / split
            if label_dir.exists():
                for label_file in label_dir.glob("*.txt"):
                    with open(label_file) as f:
                        for line in f:
                            parts = line.strip().split()
                            if parts:
                                class_names.add(int(parts[0]))
        # This would need a class mapping file - skip for now
        print("Note: For YOLO to YOLO, please provide --classes argument")

    # Create dataset.yaml
    if class_names:
        create_dataset_yaml(args.output, class_names, splits)

    print(f"\nDone! YOLO dataset created at: {args.output}")
    print(f"Dataset config: {Path(args.output) / 'dataset.yaml'}")


if __name__ == "__main__":
    main()