#!/usr/bin/env python3
# Copyright (c) shenjiangtao. All Rights Reserved.
# Licensed under the MIT License.
#
# validate_model.py — Validate ONNX/TensorRT model against test dataset.

import argparse
import os
import cv2
import numpy as np
from pathlib import Path
from tqdm import tqdm

try:
    import onnxruntime as ort
except ImportError:
    print("Error: onnxruntime not installed. Run: pip install onnxruntime")
    exit(1)

try:
    import yaml
except ImportError:
    print("Error: pyyaml not installed. Run: pip install pyyaml")
    exit(1)


def parse_args():
    parser = argparse.ArgumentParser(description="Validate ONNX model on test dataset")
    parser.add_argument("--model", type=str, required=True, help="ONNX model path")
    parser.add_argument("--data", type=str, required=True, help="Dataset YAML config")
    parser.add_argument("--imgsz", type=int, nargs=2, default=[640, 640], help="Input image size")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
    parser.add_argument("--iou", type=float, default=0.45, help="NMS IoU threshold")
    parser.add_argument("--provider", type=str, default="CUDAExecutionProvider",
                        choices=["CPUExecutionProvider", "CUDAExecutionProvider"],
                        help="ONNX Runtime provider")
    parser.add_argument("--batch", type=int, default=1, help="Batch size")
    return parser.parse_args()


def load_dataset_yaml(data_path):
    with open(data_path, 'r') as f:
        return yaml.safe_load(f)


def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=False, scaleup=True):
    """Resize and pad image while meeting stride-multiple constraints."""
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:
        r = min(r, 1.0)

    # Compute padding
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]

    if auto:
        dw, dh = np.mod(dw, 32), np.mod(dh, 32)

    dw /= 2
    dh /= 2

    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)

    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)

    return img, r, (dw, dh)


def nms(boxes, scores, iou_thresh):
    """Non-maximum suppression."""
    if len(boxes) == 0:
        return []

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)

        inds = np.where(iou <= iou_thresh)[0]
        order = order[inds + 1]

    return keep


def preprocess(img, input_shape):
    """Preprocess image for ONNX inference."""
    img, ratio, (dw, dh) = letterbox(img, new_shape=input_shape)
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, HWC to CHW
    img = np.ascontiguousarray(img, dtype=np.float32) / 255.0
    img = np.expand_dims(img, 0)  # add batch dim
    return img, ratio, (dw, dh)


def postprocess(output, ratio, pad, conf_thresh, iou_thresh, num_classes):
    """Postprocess ONNX output to detections."""
    # output shape: [batch, num_dets, 5 + num_classes] or similar
    pred = output[0]  # assume batch=1

    # Filter by confidence
    conf = pred[:, 4]
    mask = conf > conf_thresh
    pred = pred[mask]

    if len(pred) == 0:
        return []

    # Get class scores
    cls_scores = pred[:, 5:]
    cls_ids = np.argmax(cls_scores, axis=1)
    cls_conf = cls_scores[np.arange(len(pred)), cls_ids]
    scores = conf[mask] * cls_conf

    # Filter again
    mask2 = scores > conf_thresh
    pred = pred[mask2]
    scores = scores[mask2]
    cls_ids = cls_ids[mask2]

    if len(pred) == 0:
        return []

    # Convert xywh to xyxy
    boxes = pred[:, :4].copy()
    boxes[:, 0] = pred[:, 0] - pred[:, 2] / 2  # x1
    boxes[:, 1] = pred[:, 1] - pred[:, 3] / 2  # y1
    boxes[:, 2] = pred[:, 0] + pred[:, 2] / 2  # x2
    boxes[:, 3] = pred[:, 1] + pred[:, 3] / 2  # y2

    # Remove padding and scale back
    boxes[:, [0, 2]] -= pad[0]
    boxes[:, [1, 3]] -= pad[1]
    boxes /= ratio

    # NMS
    keep = nms(boxes, scores, iou_thresh)

    detections = []
    for i in keep:
        detections.append({
            'bbox': boxes[i].tolist(),
            'score': float(scores[i]),
            'class_id': int(cls_ids[i])
        })

    return detections


def load_ground_truth(label_path, img_shape):
    """Load YOLO format ground truth labels."""
    if not os.path.exists(label_path):
        return []

    h, w = img_shape[:2]
    gts = []
    with open(label_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 5:
                cls_id = int(parts[0])
                xc, yc, bw, bh = map(float, parts[1:5])
                x1 = (xc - bw/2) * w
                y1 = (yc - bh/2) * h
                x2 = (xc + bw/2) * w
                y2 = (yc + bh/2) * h
                gts.append({'bbox': [x1, y1, x2, y2], 'class_id': cls_id})
    return gts


def compute_iou(box1, box2):
    """Compute IoU between two boxes."""
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])

    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    return inter / (area1 + area2 - inter + 1e-6)


def evaluate(detections, gts, iou_thresh=0.5):
    """Evaluate detections against ground truth."""
    if not gts:
        return {'tp': 0, 'fp': len(detections), 'fn': 0}

    matched = [False] * len(gts)
    tp = 0
    fp = 0

    for det in detections:
        best_iou = 0
        best_idx = -1
        for i, gt in enumerate(gts):
            if not matched[i] and det['class_id'] == gt['class_id']:
                iou = compute_iou(det['bbox'], gt['bbox'])
                if iou > best_iou:
                    best_iou = iou
                    best_idx = i

        if best_iou >= iou_thresh:
            tp += 1
            matched[best_idx] = True
        else:
            fp += 1

    fn = sum(1 for m in matched if not m)
    return {'tp': tp, 'fp': fp, 'fn': fn}


def main():
    args = parse_args()

    # Load dataset config
    data_cfg = load_dataset_yaml(args.data)
    test_img_dir = os.path.join(data_cfg['path'], data_cfg.get('test', 'images/test'))
    test_label_dir = os.path.join(data_cfg['path'], data_cfg.get('test', 'images/test').replace('images', 'labels'))

    if not os.path.exists(test_img_dir):
        print(f"Test images not found: {test_img_dir}")
        exit(1)

    # Create ONNX Runtime session
    providers = [args.provider]
    session = ort.InferenceSession(args.model, providers=providers)
    input_name = session.get_inputs()[0].name
    output_names = [o.name for o in session.get_outputs()]

    print(f"Model: {args.model}")
    print(f"Input: {input_name}, Output: {output_names}")
    print(f"Provider: {args.provider}")

    # Get image list
    img_exts = {'.jpg', '.jpeg', '.png', '.bmp'}
    img_files = [f for f in os.listdir(test_img_dir)
                 if Path(f).suffix.lower() in img_exts]

    print(f"Found {len(img_files)} test images")

    # Statistics
    total_tp = total_fp = total_fn = 0
    class_stats = {}

    for img_file in tqdm(img_files, desc="Validating"):
        img_path = os.path.join(test_img_dir, img_file)
        label_file = Path(img_file).stem + ".txt"
        label_path = os.path.join(test_label_dir, label_file)

        img = cv2.imread(img_path)
        if img is None:
            continue

        # Preprocess
        input_tensor, ratio, pad = preprocess(img, args.imgsz)

        # Inference
        outputs = session.run(output_names, {input_name: input_tensor})

        # Postprocess
        detections = postprocess(outputs, ratio, pad, args.conf, args.iou,
                                data_cfg.get('nc', 80))

        # Load ground truth
        gts = load_ground_truth(label_path, img.shape)

        # Evaluate
        stats = evaluate(detections, gts, 0.5)
        total_tp += stats['tp']
        total_fp += stats['fp']
        total_fn += stats['fn']

        # Per-class stats
        for det in detections:
            cls = det['class_id']
            if cls not in class_stats:
                class_stats[cls] = {'tp': 0, 'fp': 0, 'fn': 0}
            # Simplified: just count detections per class

    # Summary
    precision = total_tp / (total_tp + total_fp + 1e-6)
    recall = total_tp / (total_tp + total_fn + 1e-6)
    f1 = 2 * precision * recall / (precision + recall + 1e-6)

    print("\n=== Validation Results ===")
    print(f"Total TP: {total_tp}, FP: {total_fp}, FN: {total_fn}")
    print(f"Precision: {precision:.4f}")
    print(f"Recall: {recall:.4f}")
    print(f"F1-Score: {f1:.4f}")


if __name__ == "__main__":
    main()