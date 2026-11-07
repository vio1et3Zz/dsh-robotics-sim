"""Render SO-101 gripper point clouds (body-local) into a PNG contact sheet."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

VIEWS = [
    ("xy", 0, 1, "x", "y"),
    ("xz", 0, 2, "x", "z"),
    ("yz", 1, 2, "y", "z"),
]


def render(points: list[list[float]], plane: tuple[int, int], scale: float, size: int) -> Image.Image:
    image = Image.new("RGB", (size, size), (255, 255, 255))
    draw = ImageDraw.Draw(image)
    a_index, b_index = plane
    projected = [(point[a_index], point[b_index]) for point in points]
    a_values = [p[0] for p in projected]
    b_values = [p[1] for p in projected]
    a_mid = 0.5 * (min(a_values) + max(a_values))
    b_mid = 0.5 * (min(b_values) + max(b_values))
    span = max(max(a_values) - min(a_values), max(b_values) - min(b_values))
    step = 1 if len(points) < 60000 else 2
    for a, b in projected[::step]:
        px = int(size / 2 + (a - a_mid) / span * (size * 0.9))
        py = int(size / 2 - (b - b_mid) / span * (size * 0.9))
        draw.point((px, py), fill=(30, 80, 200))
    # body origin
    ox = int(size / 2 + (0.0 - a_mid) / span * (size * 0.9))
    oy = int(size / 2 - (0.0 - b_mid) / span * (size * 0.9))
    draw.line([(ox - 6, oy), (ox + 6, oy)], fill=(200, 30, 30), width=2)
    draw.line([(ox, oy - 6), (ox, oy + 6)], fill=(200, 30, 30), width=2)
    draw.text((6, 6), f"{plane} span={span*1000:.1f}mm step={step}", fill=(0, 0, 0))
    return image


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--size", type=int, default=420)
    args = parser.parse_args()
    data = json.loads(args.input.read_text(encoding="utf-8"))
    bodies = [name for name in ("gripper", "moving_jaw_so101_v1") if name in data]
    cell = args.size
    sheet = Image.new("RGB", (cell * len(VIEWS), cell * len(bodies)), (255, 255, 255))
    for row, body in enumerate(bodies):
        points = data[body]["points_local_body"]
        for column, (label, a_index, b_index, _, _) in enumerate(VIEWS):
            image = render(points, (a_index, b_index), 1.0, args.size)
            sheet.paste(image, (column * cell, row * cell))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(args.output)
    print(f"wrote {args.output} ({sheet.width}x{sheet.height})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
