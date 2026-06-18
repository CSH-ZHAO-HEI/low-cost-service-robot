#!/usr/bin/env python3
"""
Render a paper-friendly semantic map overlay.

Inputs:
  - ROS map_server YAML (image/resolution/origin)
  - semantic_map.yaml with object_x/object_y and optional approach_x/y/yaw

This script only reads files and writes a PNG. It does not modify RTAB-Map data.
"""

from __future__ import annotations

import argparse
import math
import os
import re
from typing import Dict, Tuple

import yaml
from PIL import Image, ImageDraw, ImageFont


Point2 = Tuple[float, float]


def expand(path: str) -> str:
    return os.path.abspath(os.path.expanduser(path))


def load_yaml(path: str) -> dict:
    with open(expand(path), "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def resolve_map_image(map_yaml_path: str, image_value: str) -> str:
    image_value = os.path.expanduser(image_value)
    if os.path.isabs(image_value):
        return image_value
    # map_server's map_saver may write either "map.pgm" or a path that is
    # already relative to the current project directory, e.g.
    # "thesis_figures/rtabmap_grid_map.pgm". Support both forms.
    yaml_relative = os.path.join(os.path.dirname(expand(map_yaml_path)), image_value)
    if os.path.exists(yaml_relative):
        return yaml_relative
    cwd_relative = expand(image_value)
    if os.path.exists(cwd_relative):
        return cwd_relative
    return yaml_relative


def world_to_pixel(x: float, y: float, origin: Tuple[float, float], resolution: float, height: int) -> Point2:
    px = (x - origin[0]) / resolution
    py = height - 1 - ((y - origin[1]) / resolution)
    return px, py


def load_font(size: int) -> ImageFont.ImageFont:
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
    ]
    for path in candidates:
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def display_name(name: str, simplify: bool) -> str:
    if not simplify:
        return name
    short_prefixes = {
        "RoomWindow": "W",
        "FoldingDoor": "Door",
        "CoffeeTable": "Coffee",
        "KitchenTable": "KTable",
        "BalconyTable": "BTable",
        "NightStand": "NightStand",
        "ChairA": "Chair",
        "ChairD": "Chair",
    }
    parts = name.split("_")
    if len(parts) >= 3 and parts[-2].isdigit() and parts[-1].isdigit():
        prefix = short_prefixes.get(parts[0], parts[0])
        return f"{prefix}{int(parts[-1]):02d}"
    if name.endswith("_inside_cam"):
        return name.replace("_inside_cam", "")
    return name.replace("_", "")


def draw_circle(draw: ImageDraw.ImageDraw, p: Point2, radius: int, fill, outline=None) -> None:
    x, y = p
    draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=fill, outline=outline, width=2)


def draw_arrow(draw: ImageDraw.ImageDraw, start: Point2, yaw: float, length: int, color, width: int) -> None:
    sx, sy = start
    ex = sx + math.cos(yaw) * length
    ey = sy - math.sin(yaw) * length
    draw.line((sx, sy, ex, ey), fill=color, width=width)

    head = 10
    for delta in (math.pi * 0.78, -math.pi * 0.78):
        hx = ex + math.cos(yaw + delta) * head
        hy = ey - math.sin(yaw + delta) * head
        draw.line((ex, ey, hx, hy), fill=color, width=width)


def in_bounds(p: Point2, width: int, height: int, margin: int = 20) -> bool:
    return -margin <= p[0] <= width + margin and -margin <= p[1] <= height + margin


def render(
    map_yaml_path: str,
    semantic_path: str,
    output_path: str,
    show_approach: bool,
    include_regex: str,
    simplify_labels: bool,
    label_size: int,
    min_distance_px: float,
) -> None:
    map_cfg = load_yaml(map_yaml_path)
    semantic = load_yaml(semantic_path)
    include_pattern = re.compile(include_regex, re.IGNORECASE) if include_regex else None

    image_path = resolve_map_image(map_yaml_path, map_cfg["image"])
    resolution = float(map_cfg["resolution"])
    origin_raw = map_cfg.get("origin", [0.0, 0.0, 0.0])
    origin = (float(origin_raw[0]), float(origin_raw[1]))

    base = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(base)
    width, height = base.size

    label_font = load_font(label_size)
    small_font = load_font(12)

    object_fill = (0, 210, 55)
    object_outline = (0, 95, 20)
    approach_fill = (255, 185, 35)
    approach_outline = (160, 95, 0)
    label_color = (0, 75, 255)
    arrow_color = (255, 140, 0)

    drawn = 0
    skipped = []
    drawn_points = []
    for name, item in sorted(semantic.items()):
        if include_pattern and not include_pattern.search(name):
            continue
        if not isinstance(item, dict) or "object_x" not in item or "object_y" not in item:
            continue
        ox = float(item["object_x"])
        oy = float(item["object_y"])
        obj_px = world_to_pixel(ox, oy, origin, resolution, height)
        if not in_bounds(obj_px, width, height):
            skipped.append(name)
            continue
        if min_distance_px > 0.0:
            too_close = any(
                ((obj_px[0] - p[0]) ** 2 + (obj_px[1] - p[1]) ** 2) ** 0.5 < min_distance_px
                for p in drawn_points
            )
            if too_close:
                continue
        drawn_points.append(obj_px)

        draw_circle(draw, obj_px, radius=7, fill=object_fill, outline=object_outline)

        if show_approach and "approach_x" in item and "approach_y" in item:
            ax = float(item["approach_x"])
            ay = float(item["approach_y"])
            app_px = world_to_pixel(ax, ay, origin, resolution, height)
            yaw = float(item.get("approach_yaw", math.atan2(oy - ay, ox - ax)))
            if in_bounds(app_px, width, height):
                draw_circle(draw, app_px, radius=5, fill=approach_fill, outline=approach_outline)
                draw_arrow(draw, app_px, yaw, length=28, color=arrow_color, width=3)

        label = display_name(name, simplify_labels)
        tx = obj_px[0] + 8
        ty = obj_px[1] - label_size - 3
        bbox = draw.textbbox((tx, ty), label, font=label_font)
        label_w = bbox[2] - bbox[0]
        if tx + label_w > width - 2:
            tx = obj_px[0] - label_w - 8
        if ty < 2:
            ty = obj_px[1] + 8
        # Thin white shadow keeps blue text readable on gray maps.
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            draw.text((tx + dx, ty + dy), label, font=label_font, fill=(245, 245, 245))
        draw.text((tx, ty), label, font=label_font, fill=label_color)
        drawn += 1

    caption = f"semantic overlay: {drawn} labels"
    draw.rectangle((8, height - 28, 210, height - 6), fill=(255, 255, 255), outline=(180, 180, 180))
    draw.text((14, height - 25), caption, font=small_font, fill=(30, 30, 30))

    os.makedirs(os.path.dirname(expand(output_path)) or ".", exist_ok=True)
    base.save(expand(output_path))
    print(f"[ok] wrote {expand(output_path)}")
    print(f"[info] drawn={drawn}, skipped_out_of_map={len(skipped)}")
    if skipped:
        print("[warn] skipped:", ", ".join(skipped))


def main() -> int:
    parser = argparse.ArgumentParser(description="Render semantic labels on a ROS occupancy map image")
    parser.add_argument("--map", required=True, help="path to map.yaml")
    parser.add_argument("--semantic", default="~/.ros/semantic_map.yaml", help="path to semantic YAML")
    parser.add_argument("--output", default="semantic_map_overlay.png", help="output PNG path")
    parser.add_argument("--no-approach", action="store_true", help="hide approach points/arrows")
    parser.add_argument(
        "--include-regex",
        default="",
        help="only render semantic names matching this regex, e.g. 'Trash|Table|red_block'",
    )
    parser.add_argument("--simplify-labels", action="store_true", help="shorten labels for paper figures")
    parser.add_argument("--label-size", type=int, default=16, help="label font size")
    parser.add_argument(
        "--min-distance-px",
        type=float,
        default=0.0,
        help="skip matched semantic points closer than this many pixels to an already drawn point",
    )
    args = parser.parse_args()

    render(
        map_yaml_path=args.map,
        semantic_path=args.semantic,
        output_path=args.output,
        show_approach=not args.no_approach,
        include_regex=args.include_regex,
        simplify_labels=args.simplify_labels,
        label_size=args.label_size,
        min_distance_px=args.min_distance_px,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
