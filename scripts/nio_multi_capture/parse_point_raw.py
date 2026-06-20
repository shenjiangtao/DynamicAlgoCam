#!/usr/bin/env python3
"""
NIO Point Cloud Raw Data (.raw) Parser & PCD Converter

Data structure of .point_raw file:
  File header (64 bytes, frame 0 only):
    [0:16]   Magic string: "NIO_POINT_CLOUD_R"  (NIO_POINT_CLOUD_RAW, 16 bytes)
    [16:20]  Version      (uint32) = 1
    [20:24]  Field count  (uint32) = 6
    [24:72]  Field names  (48 bytes, null-separated: "x\\0y\\0z\\0intensity\\0ring\\0timestamp")
  Per-frame header (32 bytes, every frame):
    [0:8]    Frame index  (uint64)
    [8:16]   TimestampUs  (uint64)
    [16:20]  Point count  (uint32)
    [20:24]  Data bytes   (uint32) = pointCount * 26
    [24:32]  Reserved     (uint64, zero)
  Per-frame point data (pointCount * 26 bytes each):
    float x(4) + float y(4) + float z(4) + float intensity(4) +
    uint16 ring(2) + double timestamp(8) = 26 bytes per point

Usage:
  python3 parse_point_raw.py <file.raw> [options]

Options:
  --frame N       Frame index to extract (default: 0)
  --all           Extract all frames as individual PCD files
  --output DIR    Output directory for PCD files (default: point_pcd)
  --info          Print file info and frame summary only
"""

import argparse
import os
import struct
import sys

MAGIC = b"NIO_POINT_CLOUD_R"
FILE_HEADER_SIZE = 64
FRAME_HEADER_SIZE = 32
POINT_SIZE = 26

POINT_FIELDS = ["x", "y", "z", "intensity", "ring", "timestamp"]
POINT_SIZES = [4, 4, 4, 4, 2, 8]
POINT_TYPES = ["F", "F", "F", "F", "U", "F"]


def read_file_header(f):
    data = f.read(FILE_HEADER_SIZE)
    if len(data) < FILE_HEADER_SIZE:
        return None
    magic = data[0:16]
    if magic != MAGIC:
        return None
    version, field_count = struct.unpack_from("<II", data, 16)
    return {"version": version, "field_count": field_count}


def read_frame_header(f):
    data = f.read(FRAME_HEADER_SIZE)
    if len(data) < FRAME_HEADER_SIZE:
        return None
    frame_index, timestamp_us, point_count, data_bytes, reserved = struct.unpack_from("<QQIII", data)
    return {
        "frame_index": frame_index,
        "timestamp_us": timestamp_us,
        "point_count": point_count,
        "data_bytes": data_bytes,
        "reserved": reserved,
    }


def write_pcd(filepath, point_count, points_data):
    with open(filepath, "wb") as pcd:
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z intensity ring timestamp\n"
            "SIZE 4 4 4 4 2 8\n"
            "TYPE F F F F U F\n"
            "COUNT 1 1 1 1 1 1\n"
            "WIDTH {0}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            "POINTS {0}\n"
            "DATA binary\n".format(point_count)
        )
        pcd.write(header.encode("ascii"))
        pcd.write(points_data)


def parse_file(filepath):
    frames = []
    with open(filepath, "rb") as f:
        hdr = read_file_header(f)
        if hdr is None:
            print("Error: not a NIO_POINT_CLOUD_RAW file (bad magic)", file=sys.stderr)
            return None, None
        print(f"Version: {hdr['version']}, Fields: {hdr['field_count']}")
        while True:
            fh = read_frame_header(f)
            if fh is None:
                break
            expected = fh["point_count"] * POINT_SIZE
            if fh["data_bytes"] != expected:
                print(f"Warning: frame {fh['frame_index']} data_bytes={fh['data_bytes']} != expected={expected}", file=sys.stderr)
            data = f.read(fh["data_bytes"])
            if len(data) < fh["data_bytes"]:
                print(f"Warning: truncated data for frame {fh['frame_index']}", file=sys.stderr)
                break
            fh["data_offset"] = f.tell() - fh["data_bytes"]
            fh["points_data"] = data
            frames.append(fh)
    return hdr, frames


def main():
    parser = argparse.ArgumentParser(description="NIO Point Cloud Raw (.raw) Parser & PCD Converter")
    parser.add_argument("file", help="Input .point_raw file")
    parser.add_argument("--frame", type=int, default=0, help="Frame index to extract (default: 0)")
    parser.add_argument("--all", action="store_true", help="Extract all frames as individual PCD files")
    parser.add_argument("--output", default="point_pcd", help="Output directory for PCD files (default: point_pcd)")
    parser.add_argument("--info", action="store_true", help="Print file info and frame summary only")
    args = parser.parse_args()

    hdr, frames = parse_file(args.file)
    if hdr is None or frames is None:
        sys.exit(1)

    print(f"Total frames: {len(frames)}")
    if len(frames) > 0:
        print(f"First frame ts: {frames[0]['timestamp_us']} us")
        print(f"Last  frame ts: {frames[-1]['timestamp_us']} us")
        print(f"Points per frame (first): {frames[0]['point_count']}")

    if args.info:
        for i, fh in enumerate(frames):
            print(f"  Frame {i}: idx={fh['frame_index']} pts={fh['point_count']} ts={fh['timestamp_us']}")
        sys.exit(0)

    os.makedirs(args.output, exist_ok=True)
    basename = os.path.splitext(os.path.basename(args.file))[0]

    if args.all:
        for i, fh in enumerate(frames):
            pcd_path = os.path.join(args.output, f"{basename}_{i:06d}.pcd")
            write_pcd(pcd_path, fh["point_count"], fh["points_data"])
        print(f"Extracted {len(frames)} frames to {args.output}/")
    else:
        if args.frame < 0 or args.frame >= len(frames):
            print(f"Error: frame index {args.frame} out of range [0, {len(frames)-1}]", file=sys.stderr)
            sys.exit(1)
        fh = frames[args.frame]
        pcd_path = os.path.join(args.output, f"{basename}_{args.frame:06d}.pcd")
        write_pcd(pcd_path, fh["point_count"], fh["points_data"])
        print(f"Wrote frame {args.frame} ({fh['point_count']} points) to {pcd_path}")


if __name__ == "__main__":
    main()
