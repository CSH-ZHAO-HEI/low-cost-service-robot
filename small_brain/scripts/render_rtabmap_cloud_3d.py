#!/usr/bin/env python3
"""
Render a simple paper-friendly 3D view from an RTAB-Map exported binary PLY.
"""

from __future__ import annotations

import argparse
import math
import os
import struct
from typing import List, Tuple

from PIL import Image, ImageDraw, ImageFont


Point = Tuple[float, float, float, int, int, int]


def expand(path: str) -> str:
    return os.path.abspath(os.path.expanduser(path))


def load_font(size: int) -> ImageFont.ImageFont:
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
    ):
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def read_ply_xyzrgb(path: str, stride: int) -> List[Point]:
    with open(expand(path), "rb") as f:
        header = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError("unexpected end of PLY header")
            text = line.decode("ascii", errors="replace").strip()
            header.append(text)
            if text == "end_header":
                break

        fmt = None
        vertex_count = None
        props = []
        in_vertex = False
        for line in header:
            if line.startswith("format "):
                fmt = line.split()[1]
            elif line.startswith("element vertex "):
                vertex_count = int(line.split()[2])
                in_vertex = True
            elif line.startswith("element ") and not line.startswith("element vertex "):
                in_vertex = False
            elif in_vertex and line.startswith("property "):
                _, typ, name = line.split()[:3]
                props.append((typ, name))

        if fmt != "binary_little_endian":
            raise ValueError(f"unsupported PLY format: {fmt}")
        if vertex_count is None:
            raise ValueError("PLY has no vertex count")

        codes = {
            "float": "f", "float32": "f", "double": "d",
            "uchar": "B", "uint8": "B", "char": "b", "int8": "b",
            "ushort": "H", "uint16": "H", "short": "h", "int16": "h",
            "uint": "I", "uint32": "I", "int": "i", "int32": "i",
        }
        names = [name for _, name in props]
        row = struct.Struct("<" + "".join(codes[t] for t, _ in props))
        ix, iy, iz = names.index("x"), names.index("y"), names.index("z")
        ir = names.index("red") if "red" in names else None
        ig = names.index("green") if "green" in names else None
        ib = names.index("blue") if "blue" in names else None

        points = []
        for i in range(vertex_count):
            values = row.unpack(f.read(row.size))
            if i % stride == 0:
                if ir is not None and ig is not None and ib is not None:
                    color = (int(values[ir]), int(values[ig]), int(values[ib]))
                else:
                    color = (80, 80, 80)
                points.append((float(values[ix]), float(values[iy]), float(values[iz]), *color))
        return points


def rotate_point(p: Point, yaw: float, pitch: float) -> Point:
    x, y, z, r, g, b = p
    cy, sy = math.cos(yaw), math.sin(yaw)
    x1 = cy * x - sy * y
    y1 = sy * x + cy * y
    z1 = z

    cp, sp = math.cos(pitch), math.sin(pitch)
    y2 = cp * y1 - sp * z1
    z2 = sp * y1 + cp * z1
    return x1, y2, z2, r, g, b


def render(
    cloud: str,
    output: str,
    width: int,
    height: int,
    stride: int,
    yaw_deg: float,
    pitch_deg: float,
    z_min: float,
    z_max: float,
) -> None:
    raw = [p for p in read_ply_xyzrgb(cloud, stride=stride) if z_min <= p[2] <= z_max]
    if not raw:
        raise ValueError("no points after z filter")

    cx = sum(p[0] for p in raw) / len(raw)
    cy = sum(p[1] for p in raw) / len(raw)
    cz = sum(p[2] for p in raw) / len(raw)
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    pts = [rotate_point((x - cx, y - cy, z - cz, r, g, b), yaw, pitch) for x, y, z, r, g, b in raw]

    min_x, max_x = min(p[0] for p in pts), max(p[0] for p in pts)
    min_y, max_y = min(p[1] for p in pts), max(p[1] for p in pts)
    min_depth, max_depth = min(p[2] for p in pts), max(p[2] for p in pts)
    pad = 45
    scale = min((width - 2 * pad) / max(1e-6, max_x - min_x),
                (height - 2 * pad) / max(1e-6, max_y - min_y))

    img = Image.new("RGB", (width, height), (226, 226, 226))
    draw = ImageDraw.Draw(img)

    # Draw far points first.
    for x, y, d, r, g, b in sorted(pts, key=lambda v: v[2]):
        px = int((x - min_x) * scale + pad)
        py = int(height - ((y - min_y) * scale + pad))
        depth_t = (d - min_depth) / max(1e-6, max_depth - min_depth)
        light = 0.55 + 0.45 * depth_t
        color = (
            max(0, min(255, int(r * light))),
            max(0, min(255, int(g * light))),
            max(0, min(255, int(b * light))),
        )
        if 0 <= px < width and 0 <= py < height:
            draw.rectangle((px, py, px + 1, py + 1), fill=color)

    font = load_font(18)
    small = load_font(13)
    draw.text((18, 16), "RTAB-Map 3D point cloud", font=font, fill=(20, 20, 20))
    caption = f"{len(raw)} rendered points from exported RTAB-Map cloud"
    draw.rectangle((12, height - 32, 430, height - 8), fill=(255, 255, 255), outline=(170, 170, 170))
    draw.text((18, height - 28), caption, font=small, fill=(30, 30, 30))

    os.makedirs(os.path.dirname(expand(output)) or ".", exist_ok=True)
    img.save(expand(output))
    print(f"[ok] wrote {expand(output)}")
    print(f"[info] source_points={len(raw)}, stride={stride}, size={width}x{height}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Render RTAB-Map exported PLY as a 3D-looking image")
    parser.add_argument("--cloud", required=True)
    parser.add_argument("--output", default="rtabmap_3d_cloud.png")
    parser.add_argument("--width", type=int, default=1200)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--stride", type=int, default=2)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=58.0)
    parser.add_argument("--z-min", type=float, default=-0.2)
    parser.add_argument("--z-max", type=float, default=2.2)
    args = parser.parse_args()
    render(args.cloud, args.output, args.width, args.height, args.stride, args.yaw, args.pitch, args.z_min, args.z_max)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
