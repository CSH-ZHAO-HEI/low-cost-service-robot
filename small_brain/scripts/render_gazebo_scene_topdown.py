#!/usr/bin/env python3
"""
Render a top-down scene overview from gazebo_scene.yaml.

This is a paper-friendly scene layout figure. It does not modify Gazebo or
RTAB-Map; it only reads the exported scene YAML.
"""

from __future__ import annotations

import argparse
import os
import re
from typing import Dict, Tuple

import yaml
from PIL import Image, ImageDraw, ImageFont


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


def category(name: str) -> str:
    low = name.lower()
    if "wall" in low or "window" in low or "door" in low or "curtain" in low:
        return "structure"
    if "table" in low or "desk" in low or "bench" in low or "nightstand" in low:
        return "surface"
    if "chair" in low or "sofa" in low or "bed" in low:
        return "furniture"
    if "trash" in low or "bin" in low:
        return "target"
    if "block" in low or "coke" in low or "cup" in low:
        return "task"
    return "other"


def color_for(cat: str):
    return {
        "structure": ((210, 210, 210), (80, 80, 80)),
        "surface": ((255, 220, 125), (150, 105, 20)),
        "furniture": ((160, 205, 255), (40, 95, 150)),
        "target": ((255, 150, 150), (160, 35, 35)),
        "task": ((120, 230, 130), (20, 130, 35)),
        "other": ((220, 220, 240), (100, 100, 130)),
    }.get(cat, ((220, 220, 240), (100, 100, 130)))


def render(scene_path: str, output: str, include_regex: str, label_regex: str) -> None:
    with open(expand(scene_path), "r", encoding="utf-8") as f:
        scene = yaml.safe_load(f) or {}

    include = re.compile(include_regex, re.IGNORECASE) if include_regex else None
    label_pat = re.compile(label_regex, re.IGNORECASE) if label_regex else None

    items = []
    for name, info in scene.items():
        if not isinstance(info, dict) or "object_x" not in info or "object_y" not in info:
            continue
        if include and not include.search(name):
            continue
        x = float(info["object_x"])
        y = float(info["object_y"])
        hx = float(info.get("bbox_half_x", 0.18))
        hy = float(info.get("bbox_half_y", 0.18))
        items.append((name, x, y, max(hx, 0.08), max(hy, 0.08), category(name)))

    if not items:
        raise ValueError("no scene items to render")

    min_x = min(x - hx for _, x, _, hx, _, _ in items)
    max_x = max(x + hx for _, x, _, hx, _, _ in items)
    min_y = min(y - hy for _, _, y, _, hy, _ in items)
    max_y = max(y + hy for _, _, y, _, hy, _ in items)

    width, height = 1400, 900
    pad = 70
    scale = min((width - 2 * pad) / max(1e-6, max_x - min_x),
                (height - 2 * pad) / max(1e-6, max_y - min_y))

    def wp(x: float, y: float) -> Tuple[int, int]:
        return int((x - min_x) * scale + pad), int(height - ((y - min_y) * scale + pad))

    img = Image.new("RGB", (width, height), (244, 244, 244))
    draw = ImageDraw.Draw(img)
    title_font = load_font(24)
    label_font = load_font(15)
    small_font = load_font(13)

    draw.text((24, 20), "Gazebo scene top-down overview", font=title_font, fill=(25, 25, 25))

    # Draw larger structural/furniture objects first, task objects last.
    order = {"structure": 0, "surface": 1, "furniture": 2, "other": 3, "target": 4, "task": 5}
    for name, x, y, hx, hy, cat in sorted(items, key=lambda it: (order[it[5]], -(it[3] * it[4]))):
        fill, outline = color_for(cat)
        x0, y0 = wp(x - hx, y + hy)
        x1, y1 = wp(x + hx, y - hy)
        if cat in {"task", "target"}:
            cx, cy = wp(x, y)
            r = 8 if cat == "task" else 10
            draw.ellipse((cx - r, cy - r, cx + r, cy + r), fill=fill, outline=outline, width=2)
        else:
            draw.rectangle((x0, y0, x1, y1), fill=fill, outline=outline, width=2)

        if label_pat and label_pat.search(name):
            lx, ly = wp(x, y)
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                draw.text((lx + 8 + dx, ly - 16 + dy), name, font=label_font, fill=(255, 255, 255))
            draw.text((lx + 8, ly - 16), name, font=label_font, fill=(0, 70, 220))

    legend = [
        ("structure", "walls/doors/windows"),
        ("surface", "tables/desks"),
        ("furniture", "chairs/sofa/bed"),
        ("target", "trash/bin"),
        ("task", "task objects"),
    ]
    lx, ly = 24, height - 150
    for i, (cat, text) in enumerate(legend):
        fill, outline = color_for(cat)
        y = ly + i * 24
        draw.rectangle((lx, y, lx + 18, y + 18), fill=fill, outline=outline)
        draw.text((lx + 26, y - 1), text, font=small_font, fill=(35, 35, 35))

    caption = f"{len(items)} objects from gazebo_scene.yaml"
    draw.rectangle((24, height - 34, 300, height - 10), fill=(255, 255, 255), outline=(180, 180, 180))
    draw.text((32, height - 30), caption, font=small_font, fill=(30, 30, 30))

    os.makedirs(os.path.dirname(expand(output)) or ".", exist_ok=True)
    img.save(expand(output))
    print(f"[ok] wrote {expand(output)}")
    print(f"[info] rendered_objects={len(items)}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Render top-down Gazebo scene overview from gazebo_scene.yaml")
    parser.add_argument("--scene", default="gazebo_scene.yaml")
    parser.add_argument("--output", default="gazebo_scene_topdown.png")
    parser.add_argument("--include-regex", default="")
    parser.add_argument(
        "--label-regex",
        default="Trash|Table|NightStand|Sofa|red_block|blue_block|yellow_block|Coke",
        help="only names matching this regex get text labels",
    )
    args = parser.parse_args()
    render(args.scene, args.output, args.include_regex, args.label_regex)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
