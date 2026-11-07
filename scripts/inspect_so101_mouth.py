"""Inspect the SO-101 gripper mouth cross-sections from the local point clouds."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
POINTS = ROOT / "artifacts" / "t06_2026-11-07_1130" / "gripper_points.json"

data = json.loads(POINTS.read_text(encoding="utf-8"))
palm = np.asarray(data["gripper"]["points_local_body"], dtype=np.float64)
jaw = np.asarray(data["moving_jaw_so101_v1"]["points_local_body"], dtype=np.float64)

# jaw local -> palm local (closed pose identity assumed for the raw stage):
#   palm = jaw + hinge; jaw_x -> palm_x, jaw_y -> palm_z, jaw_z -> -palm_y
HINGE = np.array([0.0202, 0.0188, -0.0234])
jaw_in_palm = np.stack(
    [jaw[:, 0] + HINGE[0], -jaw[:, 2] + HINGE[1], jaw[:, 1] + HINGE[2]], axis=1
)

for label, points in (("palm", palm), ("jaw_closed", jaw_in_palm)):
    print(f"== {label} local extent: min={points.min(axis=0)} max={points.max(axis=0)}")
    for z_lo, z_hi in ((-0.105, -0.095), (-0.095, -0.085), (-0.085, -0.075), (-0.075, -0.055), (-0.055, -0.035)):
        slab = points[(points[:, 2] >= z_lo) & (points[:, 2] < z_hi)]
        if len(slab) == 0:
            print(f"  z[{z_lo:+.3f},{z_hi:+.3f}] empty")
            continue
        print(
            f"  z[{z_lo:+.3f},{z_hi:+.3f}] n={len(slab):6d} "
            f"x[{slab[:,0].min():+.4f},{slab[:,0].max():+.4f}] "
            f"y[{slab[:,1].min():+.4f},{slab[:,1].max():+.4f}]"
        )

# ASCII mouth views: x (up) vs y (side) at several z depths.
for label, points in (("palm", palm), ("jaw", jaw_in_palm)):
    print(f"\n== {label} x/y occupancy at front z in [-0.105,-0.075]")
    slab = points[(points[:, 2] >= -0.105) & (points[:, 2] <= -0.075)]
    if len(slab):
        xs = slab[:, 0]
        ys = slab[:, 1]
        rows = 18
        cols = 44
        x_lo, x_hi = -0.040, 0.035
        y_lo, y_hi = -0.050, 0.030
        grid = [[" "] * cols for _ in range(rows)]
        for x, y in zip(xs, ys):
            col = int((y - y_lo) / (y_hi - y_lo) * (cols - 1))
            row = int((x_hi - x) / (x_hi - x_lo) * (rows - 1))
            if 0 <= row < rows and 0 <= col < cols:
                grid[row][col] = "#"
        for row_index, row in enumerate(grid):
            x_value = x_hi - (x_hi - x_lo) * row_index / (rows - 1)
            print(f"{x_value:+.3f} |" + "".join(row) + "|")
        print("       " + "".join(f"{y:+.3f}" for y in np.linspace(y_lo, y_hi, 6)))
