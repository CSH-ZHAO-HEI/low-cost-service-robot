#!/usr/bin/env python3
"""
Render a top-down semantic overlay from an RTAB-Map exported PLY cloud.

Typical use:
  rtabmap-export --cloud --output_dir thesis_figures/rtabmap_export \
    --output rtabmap_cloud --decimation 4 --voxel 0.05 --max_range 8 ~/.ros/rtabmap.db

  python3 render_rtabmap_cloud_overlay.py \
    --cloud thesis_figures/rtabmap_export/rtabmap_cloud_cloud.ply \
    --semantic gazebo_scene.yaml \
    --output thesis_figures/rtabmap_cloud_semantic_overlay.png
"""

from __future__ import annotations

import argparse
import os
import struct
from typing import Dict, Iterable, List, Tuple

import yaml
from PIL import Image, ImageDraw, ImageFont


Point = Tuple[float, float, float]


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


def read_ply_xyz(path: str) -> List[Point]:
    path = expand(path)
    with open(path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PLY header ended unexpectedly")
            text = line.decode("ascii", errors="replace").strip()
            header_lines.append(text)
            if text == "end_header":
                break

        fmt = None
        vertex_count = None
        props = []
        in_vertex = False
        for line in header_lines:
            if line.startswith("format "):
                fmt = line.split()[1]
            elif line.startswith("element vertex "):
                vertex_count = int(line.split()[2])
                in_vertex = True
            elif line.startswith("element ") and not line.startswith("element vertex "):
                in_vertex = False
            elif in_vertex and line.startswith("property "):
                parts = line.split()
                props.append((parts[1], parts[2]))

        if fmt != "binary_little_endian":
            raise ValueError(f"Only binary_little_endian PLY is supported, got {fmt!r}")
        if vertex_count is None:
            raise ValueError("PLY has no vertex count")

        struct_codes = {
            "float": "f",
            "float32": "f",
            "double": "d",
            "uchar": "B",
            "uint8": "B",
            "char": "b",
            "int8": "b",
            "ushort": "H",
            "uint16": "H",
            "short": "h",
            "int16": "h",
            "uint": "I",
            "uint32": "I",
            "int": "i",
            "int32": "i",
        }
        names = [name for _, name in props]
        fmt_codes = "<" + "".join(struct_codes[t] for t, _ in props)
        row = struct.Struct(fmt_codes)
        ix, iy, iz = names.index("x"), names.index("y"), names.index("z")

        points = []
        for _ in range(vertex_count):
            values = row.unpack(f.read(row.size))
            points.append((float(values[ix]), float(values[iy]), float(values[iz])))
        return points


def load_semantic(path: str) -> Dict[str, dict]:
    with open(expand(path), "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data if isinstance(data, dict) else {}


def world_to_pixel(x: float, y: float, min_x: float, max_y: float, resolution: float, pad: int) -> Tuple[int, int]:
    px = int(round((x - min_x) / resolution)) + pad
    py = int(round((max_y - y) / resolution)) + pad
    return px, py


def draw_circle(draw: ImageDraw.ImageDraw, p: Tuple[int, int], radius: int, fill, outline) -> None:
    x, y = p
    draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=fill, outline=outline, width=2)


def render(
    cloud_path: str,
    semantic_path: str,
    output_path: str,
    resolution: float,
    z_min: float,
    z_max: float,
    label_limit: int,
) -> None:
    points = [p for p in read_ply_xyz(cloud_path) if z_min <= p[2] <= z_max]
    semantic = load_semantic(semantic_path)
    semantic_points = [
        (name, float(v["object_x"]), float(v["object_y"]))
        for name, v in semantic.items()
        if isinstance(v, dict) and "object_x" in v and "object_y" in v
    ]

    xs = [p[0] for p in points] + [p[1] for p in []]
    ys = [p[1] for p in points]
    if semantic_points:
        xs += [x for _, x, _ in semantic_points]
        ys += [y for _, _, y in semantic_points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    pad = 40
    width = int(round((max_x - min_x) / resolution)) + pad * 2 + 1
    height = int(round((max_y - min_y) / resolution)) + pad * 2 + 1

    img = Image.new("RGB", (width, height), (210, 210, 210))
    pix = img.load()
    for x, y, _ in points:
        px, py = world_to_pixel(x, y, min_x, max_y, resolution, pad)
        if 0 <= px < width and 0 <= py < height:
            pix[px, py] = (40, 40, 40)

    # Light dilation makes sparse clouds readable in a paper figure.
    img = img.resize((width * 2, height * 2), Image.Resampling.NEAREST)
    draw = ImageDraw.Draw(img)
    font = load_font(16)
    small_font = load_font(12)
    scale = 2

    drawn = 0
    for name, x, y in sorted(semantic_points):
        px, py = world_to_pixel(x, y, min_x, max_y, resolution, pad)
        px *= scale
        py *= scale
        if not (0 <= px < img.size[0] and 0 <= py < img.size[1]):
            continue
        draw_circle(draw, (px, py), 7, fill=(0, 215, 55), outline=(0, 90, 20))
        if drawn < label_limit:
            tx, ty = px + 10, py - 18
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                draw.text((tx + dx, ty + dy), name, font=font, fill=(245, 245, 245))
            draw.text((tx, ty), name, font=font, fill=(0, 75, 255))
        drawn += 1

    caption = f"RTAB-Map DB cloud overlay: {len(points)} points, {drawn} semantic points"
    draw.rectangle((8, img.size[1] - 30, 460, img.size[1] - 6), fill=(255, 255, 255), outline=(170, 170, 170))
    draw.text((14, img.size[1] - 26), caption, font=small_font, fill=(30, 30, 30))

    os.makedirs(os.path.dirname(expand(output_path)) or ".", exist_ok=True)
    img.save(expand(output_path))
    print(f"[ok] wrote {expand(output_path)}")
    print(f"[info] cloud_points={len(points)}, semantic_points={drawn}, size={img.size}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Render semantic labels on an RTAB-Map exported cloud")
    parser.add_argument("--cloud", required=True, help="RTAB-Map exported binary PLY cloud")
    parser.add_argument("--semantic", default="gazebo_scene.yaml", help="semantic YAML")
    parser.add_argument("--output", default="rtabmap_cloud_semantic_overlay.png", help="output PNG")
    parser.add_argument("--resolution", type=float, default=0.05, help="top-down render resolution in meters")
    parser.add_argument("--z-min", type=float, default=-0.2, help="minimum z to render")
    parser.add_argument("--z-max", type=float, default=2.2, help="maximum z to render")
    parser.add_argument("--label-limit", type=int, default=40, help="maximum labels to draw")
    args = parser.parse_args()
    render(args.cloud, args.semantic, args.output, args.resolution, args.z_min, args.z_max, args.label_limit)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
