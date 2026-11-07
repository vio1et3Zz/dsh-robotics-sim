"""Solve the joint waypoints for the SO-101 top-grasp pick/place."""

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

BOUNDS = [
    (math.radians(-109.99), math.radians(109.99)),
    (math.radians(-100.0), math.radians(100.0)),
    (math.radians(-96.83), math.radians(96.83)),
    (math.radians(-95.0), math.radians(95.0)),
    (math.radians(-157.21), math.radians(162.79)),
]
GRASP_Y = -0.005
PLACE = (0.22, 0.05)


def build_waypoints(grasp_x: float, grasp_y: float) -> list[tuple[str, float, float, float]]:
    """Grasp waypoints at the requested x/y; transfer/place/retreat stay fixed."""
    return [
        ("approach", grasp_x, grasp_y, 0.115),
        ("grasp", grasp_x, grasp_y, 0.085),
        ("lift1", grasp_x, grasp_y, 0.105),
        ("lift2", grasp_x, grasp_y, 0.130),
        ("transfer", PLACE[0], PLACE[1], 0.130),
        ("place", PLACE[0], PLACE[1], 0.075),
        ("retreat", PLACE[0], PLACE[1], 0.160),
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--starts", type=int, default=100)
    parser.add_argument("--grasp-x", type=float, default=0.18)
    parser.add_argument("--grasp-y", type=float, default=GRASP_Y)
    parser.add_argument(
        "--preapproach-z",
        type=float,
        default=None,
        help="also solve a high entry point at this z (for variants whose direct approach sweeps the block)",
    )
    args = parser.parse_args()
    stage = Usd.Stage.Open(str(module.USD_PATH))
    frames = module.joint_frames(stage)
    rotation = module.rotation_for((0.0, 0.0, -1.0))
    direction = np.array([0.0, 0.0, -1.0])
    results = []
    waypoints = build_waypoints(args.grasp_x, args.grasp_y)
    if args.preapproach_z is not None:
        waypoints.insert(0, ("preapproach", args.grasp_x, args.grasp_y, args.preapproach_z))
    for name, x, y, z in waypoints:
        result = module.solve(frames, np.array([x, y, z]), direction, rotation[:, 0], BOUNDS, starts=args.starts)
        result.update({"name": name, "x": x, "y": y, "z": z})
        result["angles_deg"] = [float(math.degrees(v)) for v in result["angles"]]
        results.append(result)
        print(
            f"{name:9s} ({x:.2f},{y:+.2f},{z:.3f}) pos={result['position_error_m']*1000:6.2f}mm "
            f"dir={result['direction_error_deg']:5.2f}deg angles={[round(v,2) for v in result['angles_deg']]}",
            flush=True,
        )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({"waypoints": results}, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

