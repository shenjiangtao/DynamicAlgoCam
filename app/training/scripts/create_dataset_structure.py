#!/usr/bin/env python3
# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# create_dataset_structure.py — Create YOLO dataset directory structure.

import argparse
import os
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Create YOLO dataset directory structure")
    parser.add_argument("--output", type=str, required=True, help="Output dataset root directory")
    parser.add_argument("--splits", type=str, default="train,val,test",
                        help="Comma-separated list of splits")
    parser.add_argument("--classes", type=str, required=True,
                        help="Comma-separated class names")
    parser.add_argument("--name", type=str, default="dataset",
                        help="Dataset name for dataset.yaml")
    return parser.parse_args()


def main():
    args = parse_args()
    splits = [s.strip() for s in args.splits.split(",")]
    class_names = [c.strip() for c in args.classes.split(",")]

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Create directory structure
    for split in splits:
        (output_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (output_dir / "labels" / split).mkdir(parents=True, exist_ok=True)

    # Create dataset.yaml
    import yaml
    data = {
        "path": str(output_dir.resolve()),
        "train": "images/train" if "train" in splits else "",
        "val": "images/val" if "val" in splits else "",
        "test": "images/test" if "test" in splits else "",
        "nc": len(class_names),
        "names": {i: name for i, name in enumerate(class_names)}
    }
    yaml_path = output_dir / "dataset.yaml"
    with open(yaml_path, "w") as f:
        yaml.dump(data, f, default_flow_style=False)

    # Create README
    readme_path = output_dir / "README.md"
    with open(readme_path, "w") as f:
        f.write(f"# {args.name}\n\n")
        f.write(f"YOLO format dataset for {', '.join(class_names)} detection.\n\n")
        f.write("## Structure\n\n")
        f.write("```\n")
        f.write(f"{args.name}/\n")
        for split in splits:
            f.write(f"├── images/{split}/\n")
            f.write(f"│   └── *.jpg, *.png\n")
            f.write(f"├── labels/{split}/\n")
            f.write(f"│   └── *.txt (YOLO format: class_id x_center y_center width height)\n")
        f.write("├── dataset.yaml\n")
        f.write("└── README.md\n")
        f.write("```\n\n")
        f.write("## Classes\n\n")
        for i, name in enumerate(class_names):
            f.write(f"{i}: {name}\n")

    print(f"Created dataset structure at: {output_dir}")
    print(f"Splits: {', '.join(splits)}")
    print(f"Classes ({len(class_names)}): {', '.join(class_names)}")
    print(f"Config: {yaml_path}")


if __name__ == "__main__":
    main()