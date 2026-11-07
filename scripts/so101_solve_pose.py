"""Solve one SO-101 mouth pose offline and print the joint angles.

Used to seed the physics pose-IK for T06: the offline model (USD chain,
scipy L-BFGS with random starts) finds a joint configuration that reaches the
mouth target; the physics IK then refines it under the real controllers.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
from pxr import Usd

spec = importlib.util.spec_from_file_location(
    "so101_kinematics", Path(__file__).resolve().parent / "so101_kinematics.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

DIRECTIONS = {
    "plus_x": (1.0, 0.0, 0.0),
    "minus_y": (0.0, -1.0, 0.0),
    "plus_y": (0.0, 1.0, 0.0),
    "minus_x": (-1.0, 0.0, 0.0),
    "diag_px_my": (0.7071, -0.7071, 0.0),
    "top": (0.0, 0.0, -1.0),
    "top_px": (0.5, 0.0, -0.866),
}
BOUNDS_DEG = [
    (-109.99, 109.99),
    (-100.0, 100.0),
    (-96.83, 96.83),
    (-95.0, 95.0),
    (-157.21, 162.79),
]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--x", type=float, default=0.22)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.075)
    parser.add_argument("--direction", default="plus_x")
    parser.add_argument("--offset", type=float, default=0.0, help="retreat distance along the mouth axis")
    parser.add_argument("--starts", type=int, default=120)
    args = parser.parse_args()

    direction = DIRECTIONS[args.direction]
    rotation = module.rotation_for(direction)
    mouth_target = np.array([args.x, args.y, args.z]) - args.offset * np.array(direction)
    palm_target = mouth_target - rotation @ module.GRASP_LOCAL
    bounds = [(math.radians(low), math.radians(high)) for low, high in BOUNDS_DEG]
    stage = Usd.Stage.Open(str(module.USD_PATH))
    frames = module.joint_frames(stage)
    result = module.solve(frames, mouth_target, np.array(direction), rotation[:, 0], bounds, starts=args.starts)
    result.update(
        {
            "x": args.x,
            "y": args.y,
            "z": args.z,
            "direction": args.direction,
            "offset": args.offset,
            "mouth_target": [float(v) for v in mouth_target],
            "palm_target": [float(v) for v in palm_target],
            "angles_deg": [float(math.degrees(v)) for v in result["angles"]],
        }
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
