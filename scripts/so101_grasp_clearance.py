"""Offline clearance check: SO-101 jaw/palm vs the red block at the grasp pose."""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
POINTS = ROOT / "artifacts" / "t06_2026-11-07_1130" / "gripper_points.json"
WAYPOINTS = json.loads((ROOT / "artifacts" / "t06_2026-11-07_1130" / "pick_waypoints2.json").read_text())
spec = importlib.util.spec_from_file_location("so101_kinematics", ROOT / "scripts" / "so101_kinematics.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

from pxr import Usd

stage = Usd.Stage.Open(str(module.USD_PATH))
frames = module.joint_frames(stage)
data = json.loads(POINTS.read_text())
palm = np.asarray(data["gripper"]["points_local_body"])
jaw = np.asarray(data["moving_jaw_so101_v1"]["points_local_body"])
HINGE = np.array([0.0202, 0.0188, -0.0234])
# jaw body -> palm at rest: palm = (jaw_x + 0.0202, -jaw_z + 0.0188, jaw_y - 0.0234)
jaw_palm_rest = np.stack([jaw[:, 0] + HINGE[0], -jaw[:, 2] + HINGE[1], jaw[:, 1] + HINGE[2]], axis=1)
AXIS = np.array([0.0, -1.0, 0.0])


def rotate_about(points: np.ndarray, axis: np.ndarray, anchor: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    c, s = math.cos(angle), math.sin(angle)
    rel = points - anchor
    rotated = (
        rel * c
        + np.cross(axis, rel) * s
        + np.outer(rel @ axis, axis) * (1 - c)
    )
    return rotated + anchor


BLOCK_MIN = np.array([0.165, -0.015, 0.060])
BLOCK_MAX = np.array([0.195, 0.015, 0.090])


def clearance(points_world: np.ndarray) -> float:
    # signed distance outside the box: negative means inside
    dx = np.maximum(BLOCK_MIN[0] - points_world[:, 0], points_world[:, 0] - BLOCK_MAX[0])
    dy = np.maximum(BLOCK_MIN[1] - points_world[:, 1], points_world[:, 1] - BLOCK_MAX[1])
    dz = np.maximum(BLOCK_MIN[2] - points_world[:, 2], points_world[:, 2] - BLOCK_MAX[2])
    outside = np.sqrt(np.maximum(dx, 0) ** 2 + np.maximum(dy, 0) ** 2 + np.maximum(dz, 0) ** 2)
    inside = (dx <= 0) & (dy <= 0) & (dz <= 0)
    outside[inside] = -np.minimum.reduce(
        [BLOCK_MAX[0] - points_world[:, 0], points_world[:, 0] - BLOCK_MIN[0],
         BLOCK_MAX[1] - points_world[:, 1], points_world[:, 1] - BLOCK_MIN[1],
         BLOCK_MAX[2] - points_world[:, 2], points_world[:, 2] - BLOCK_MIN[2]]
    )[inside]
    return float(outside.min())


for name in ("approach", "grasp"):
    entry = next(w for w in WAYPOINTS["waypoints"] if w["name"] == name)
    angles = np.radians(entry["angles_deg"])
    palm_transform = module.forward(frames, angles)
    palm_world = (palm_transform[:3, :3] @ palm.T).T + palm_transform[:3, 3]
    print(f"== {name}: mouth={entry['mouth']} palm_pos={palm_transform[:3,3]}")
    print(f"   palm clearance {clearance(palm_world)*1000:.1f} mm")
    for phi in (0.0, 0.2, 0.3, 0.4, 0.6, 1.0):
        jaw_rotated = rotate_about(jaw_palm_rest, AXIS, HINGE, phi)
        jaw_world = (palm_transform[:3, :3] @ jaw_rotated.T).T + palm_transform[:3, 3]
        d = clearance(jaw_world)
        # where is the closest point (useful for debugging)
        print(f"   jaw phi={phi:.1f} rad clearance {d*1000:7.1f} mm")

