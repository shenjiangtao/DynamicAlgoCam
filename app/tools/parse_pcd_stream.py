#!/usr/bin/env python3
"""
NIO PCD Stream (.pcs) Parser & Point Cloud Extractor

File format (NIO_PCD_STREAM):
  [Header]
    16B  magic = "NIO_PCD_STREAM\0"
    4B   numFields    (uint32, little-endian)
    4B   srcPointSize (uint32)
    4B   pcdPointSize (uint32)
    numFields * 24B   PcdFieldDesc entries
      16B name[16]  (null-terminated)
       1B srcSize
       1B srcOffset
       1B pcdSize
       1B pcdType    ('F'=float, 'U'=unsigned, 'I'=signed)
       4B pad[4]
  [Frame 0]
    8B   timestampUs  (uint64)
    4B   pointCount   (uint32)
    pointCount * pcdPointSize bytes (PCD binary data, already converted)
  [Frame 1]
    ...
  [Trailing Index]
    8B   dataStartOffset (uint64) — byte offset of first frame
    4B   numFrames       (uint32)
    numFrames * 16B entries:
      8B  timestampUs  (uint64)
      8B  frameOffset  (uint64) — byte offset from file start

Usage:
  python3 parse_pcd_stream.py <file.pcs> [options]

Output options:
  --frame N       Extract frame N as standalone .pcd file (default: 0)
  --all           Extract all frames as individual .pcd files
  --index         Print the frame index table
  --info          Print header info only
  --output DIR    Output directory (default: pcd_output)
  --view N        Open interactive 3D viewer for frame N (requires open3d)

Examples:
  # Print header and index:
  python3 parse_pcd_stream.py capture.pcs --info --index

  # Extract frame 0 as standalone PCD:
  python3 parse_pcd_stream.py capture.pcs --frame 0

  # Extract all frames:
  python3 parse_pcd_stream.py capture.pcs --all

  # View frame 5 interactively:
  python3 parse_pcd_stream.py capture.pcs --view 5
"""

import sys
import os
import struct
import argparse
import numpy as np


PCD_FIELD_DESC_SIZE = 24
PCD_FIELD_DESC_FMT = '<16s BBB B 4s'  # name[16], srcSize, srcOffset, pcdSize, pcdType, pad[4]
HEADER_MAGIC = b'NIO_PCD_STREAM'
DYNALOGO_PCD_STREAM_MAGIC = b'DYNALOGO_PCD_STREAM'


def parse_field_desc(data, offset):
    name_raw = data[offset:offset + 16]
    name = name_raw.split(b'\x00')[0].decode('ascii', errors='replace')
    src_size = data[offset + 16]
    src_offset = data[offset + 17]
    pcd_size = data[offset + 18]
    pcd_type = chr(data[offset + 19])
    return {
        'name': name,
        'srcSize': src_size,
        'srcOffset': src_offset,
        'pcdSize': pcd_size,
        'pcdType': pcd_type,
    }


def parse_header(data):
    if len(data) < 28:
        return None

    magic = data[0:16]
    magic_str = magic.split(b'\x00')[0].decode('ascii', errors='replace')
    if magic_str != 'NIO_PCD_STREAM' and magic_str != 'DYNALOGO_PCD_STREAM':
        print(f"ERROR: Invalid magic: '{magic_str}' (expected 'NIO_PCD_STREAM' or 'DYNALOGO_PCD_STREAM')")
        return None

    num_fields = struct.unpack_from('<I', data, 16)[0]
    src_point_size = struct.unpack_from('<I', data, 20)[0]
    pcd_point_size = struct.unpack_from('<I', data, 24)[0]

    fields = []
    for i in range(num_fields):
        off = 28 + i * PCD_FIELD_DESC_SIZE
        if off + PCD_FIELD_DESC_SIZE > len(data):
            print(f"ERROR: Truncated field descriptor at index {i}")
            return None
        fields.append(parse_field_desc(data, off))

    header_size = 28 + num_fields * PCD_FIELD_DESC_SIZE

    return {
        'magic': magic_str,
        'numFields': num_fields,
        'srcPointSize': src_point_size,
        'pcdPointSize': pcd_point_size,
        'fields': fields,
        'headerSize': header_size,
    }


def find_index_table(data):
    header = parse_header(data)
    if not header:
        return None, None

    expected_data_start = header['headerSize']
    target = struct.pack('<Q', expected_data_start)
    pos = data.rfind(target)
    while pos != -1:
        if pos + 12 > len(data):
            pos = data.rfind(target, 0, pos)
            continue
        ds, nf = struct.unpack_from('<QI', data, pos)
        if ds == expected_data_start and 0 < nf < 1000000:
            expected_size = 12 + nf * 16
            if pos + expected_size <= len(data):
                entries = []
                for i in range(nf):
                    ts, off = struct.unpack_from('<QQ', data, pos + 12 + i * 16)
                    entries.append({'timestampUs': ts, 'offset': off})
                return entries, pos
        pos = data.rfind(target, 0, pos)

    return None, None


def parse_frame(data, offset, pcd_point_size):
    if offset + 12 > len(data):
        return None
    ts = struct.unpack_from('<Q', data, offset)[0]
    point_count = struct.unpack_from('<I', data, offset + 8)[0]
    frame_data_size = point_count * pcd_point_size
    if offset + 12 + frame_data_size > len(data):
        return None
    point_data = data[offset + 12:offset + 12 + frame_data_size]
    return {
        'timestampUs': ts,
        'pointCount': point_count,
        'offset': offset,
        'pointData': point_data,
    }


def frame_to_pcd_bytes(frame, header):
    fields = header['fields']
    fields_line = ' '.join(f['name'] for f in fields)
    size_line = ' '.join(str(f['pcdSize']) for f in fields)
    type_line = ' '.join(f['pcdType'] for f in fields)
    count_line = ' '.join('1' for _ in fields)

    pcd_header = (
        f"# .PCD v0.7 - Point Cloud Data file format\n"
        f"VERSION 0.7\n"
        f"FIELDS {fields_line}\n"
        f"SIZE {size_line}\n"
        f"TYPE {type_line}\n"
        f"COUNT {count_line}\n"
        f"WIDTH {frame['pointCount']}\n"
        f"HEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {frame['pointCount']}\n"
        f"DATA binary\n"
    )
    return pcd_header.encode('utf-8') + frame['pointData']


def extract_point_arrays(frame, header):
    fields = header['fields']
    pcd_pt_sz = header['pcdPointSize']
    n = frame['pointCount']
    raw = frame['pointData']

    result = {}
    offset = 0
    for f in fields:
        name = f['name']
        sz = f['pcdSize']
        typ = f['pcdType']
        dtype = {'F': np.float32, 'U': np.uint32, 'I': np.int32}.get(typ, np.uint8)
        if sz == 1:
            dtype = np.uint8
        elif sz == 2 and typ == 'U':
            dtype = np.uint16
        elif sz == 2 and typ == 'I':
            dtype = np.int16
        arr = np.frombuffer(raw, dtype=dtype, count=n, offset=offset)
        result[name] = arr.copy()
        offset += sz
    return result


def view_frame(frame, header, frame_idx):
    try:
        import open3d as o3d
    except ImportError:
        print("ERROR: open3d required for interactive viewer. Install: pip3 install open3d")
        return

    pts = extract_point_arrays(frame, header)
    if 'x' not in pts or 'y' not in pts or 'z' not in pts:
        print("ERROR: Frame does not contain xyz fields")
        return

    xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1).astype(np.float64)
    valid = ~np.isnan(xyz).any(axis=1) & (np.abs(xyz) < 1e6).all(axis=1)
    xyz = xyz[valid]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    if 'intensity' in pts:
        intensity = pts['intensity'][valid].astype(np.float64)
        if intensity.max() > 1.0:
            intensity = intensity / max(intensity.max(), 1e-6)
        colors = np.stack([intensity, intensity, intensity], axis=-1)
        pcd.colors = o3d.utility.Vector3dVector(colors)

    print(f"Viewing frame {frame_idx}: {len(xyz)} points, ts={frame['timestampUs']}us")
    o3d.visualization.draw_geometries([pcd], window_name=f"PCD Stream Frame {frame_idx}",
                                      width=1280, height=720)


def main():
    parser = argparse.ArgumentParser(description='NIO PCD Stream (.pcs) Parser & Point Cloud Extractor')
    parser.add_argument('file', help='Path to .pcs file')
    parser.add_argument('--frame', type=int, default=None, help='Extract frame N as standalone PCD')
    parser.add_argument('--all', action='store_true', help='Extract all frames as individual PCD files')
    parser.add_argument('--index', action='store_true', help='Print frame index table')
    parser.add_argument('--info', action='store_true', help='Print header info only')
    parser.add_argument('--output', default='pcd_output', help='Output directory (default: pcd_output)')
    parser.add_argument('--view', type=int, default=None, metavar='N',
                        help='Open interactive 3D viewer for frame N (requires open3d)')
    args = parser.parse_args()

    if not os.path.exists(args.file):
        print(f"ERROR: File not found: {args.file}")
        sys.exit(1)

    with open(args.file, 'rb') as f:
        data = f.read()

    header = parse_header(data)
    if not header:
        print("ERROR: Failed to parse .pcs header")
        sys.exit(1)

    print(f"=== NIO PCD Stream Header ===")
    print(f"  Magic:         {header['magic']}")
    print(f"  Num fields:    {header['numFields']}")
    print(f"  SrcPointSize:  {header['srcPointSize']} bytes")
    print(f"  PcdPointSize:  {header['pcdPointSize']} bytes")
    print(f"  Fields:")
    for f in header['fields']:
        print(f"    {f['name']:12s}  srcSize={f['srcSize']} srcOffset={f['srcOffset']} "
              f"pcdSize={f['pcdSize']} pcdType={f['pcdType']}")
    print(f"  Header size:   {header['headerSize']} bytes")
    print(f"  File size:     {len(data)} bytes")
    print()

    index_entries, index_pos = find_index_table(data)

    if index_entries is not None:
        num_frames = len(index_entries)
        print(f"=== Frame Index ({num_frames} frames) ===")
        if args.index:
            for i, entry in enumerate(index_entries):
                ts_us = entry['timestampUs']
                ts_s = ts_us / 1e6
                print(f"  Frame {i:6d}: ts={ts_us:20d}us ({ts_s:.6f}s) offset={entry['offset']}")
        else:
            if num_frames > 0:
                t0 = index_entries[0]['timestampUs']
                tN = index_entries[-1]['timestampUs']
                dur = (tN - t0) / 1e6 if num_frames > 1 else 0
                print(f"  First frame ts: {t0}us")
                print(f"  Last  frame ts: {tN}us")
                if dur > 0:
                    print(f"  Duration:       {dur:.3f}s ({num_frames} frames, avg {num_frames/dur:.1f} fps)")
    else:
        print("WARNING: No trailing index table found. Scanning frames sequentially...")
        index_entries = []
        offset = header['headerSize']
        pcd_pt_sz = header['pcdPointSize']
        while offset + 12 <= len(data):
            ts = struct.unpack_from('<Q', data, offset)[0]
            pc = struct.unpack_from('<I', data, offset + 8)[0]
            frame_bytes = 12 + pc * pcd_pt_sz
            if offset + frame_bytes > len(data):
                break
            index_entries.append({'timestampUs': ts, 'offset': offset})
            offset += frame_bytes
        print(f"  Found {len(index_entries)} frames by sequential scan")

    num_frames = len(index_entries)
    print()

    do_extract = args.all or args.frame is not None or args.view is not None
    if not do_extract and not args.index and not args.info:
        print("Tip: use --frame N to extract a frame, --all for all, --index for index table, --view N for 3D viewer")
        return

    if not do_extract:
        return

    os.makedirs(args.output, exist_ok=True)

    pcd_pt_sz = header['pcdPointSize']

    frames_to_extract = []
    if args.view is not None:
        idx = min(args.view, num_frames - 1) if num_frames > 0 else 0
        frames_to_extract.append(idx)
    elif args.frame is not None:
        idx = min(args.frame, max(0, num_frames - 1)) if num_frames > 0 else 0
        frames_to_extract.append(idx)
    elif args.all:
        frames_to_extract = list(range(num_frames))

    for idx in frames_to_extract:
        if idx >= num_frames:
            print(f"ERROR: Frame {idx} out of range (0..{num_frames-1})")
            continue
        entry = index_entries[idx]
        frame = parse_frame(data, entry['offset'], pcd_pt_sz)
        if frame is None:
            print(f"ERROR: Failed to parse frame {idx}")
            continue

        ts_us = frame['timestampUs']
        out_name = f"frame_{idx:06d}_{ts_us}.pcd"
        out_path = os.path.join(args.output, out_name)
        pcd_bytes = frame_to_pcd_bytes(frame, header)

        with open(out_path, 'wb') as f:
            f.write(pcd_bytes)

        print(f"  Frame {idx}: {frame['pointCount']} points, ts={ts_us}us -> {out_path}")

        if args.view is not None and idx == args.view:
            view_frame(frame, header, idx)

    print(f"\nExtracted {len(frames_to_extract)} frame(s) to {args.output}/")
    print("Done!")


if __name__ == '__main__':
    main()
