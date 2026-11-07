"""Solve top-grasp poses at several candidate place targets."""

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
TARGETS = [
    (0.20, 0.10, 0.075),
    (0.20, 0.10, 0.085),
    (0.22, 0.00, 0.085),
    (0.22, 0.05, 0.085),
    (0.24, 0.05, 0.085),
    (0.20, -0.05, 0.085),
    (0.18, -0.06, 0.085),
    (0.24, -0.02, 0.085),
]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--starts", type=int, default=100)
    args = parser.parse_args()
    stage = Usd.Stage.Open(str(module.USD_PATH))
    frames = module.joint_frames(stage)
    rotation = module.rotation_for((0.0, 0.0, -1.0))
    direction = np.array([0.0, 0.0, -1.0])
    results = []
    for x, y, z in TARGETS:
        mouth = np.array([x, y, z])
        result = module.solve(frames, mouth, direction, rotation[:, 0], BOUNDS, starts=args.starts)
        result.update({"x": x, "y": y, "z": z})
        result["angles_deg"] = [float(math.degrees(v)) for v in result["angles"]]
        results.append(result)
        print(
            f"({x:.2f},{y:+.2f},{z:.3f}) pos={result['position_error_m']*1000:6.2f}mm "
            f"dir={result['direction_error_deg']:5.2f}deg angles={[round(v,1) for v in result['angles_deg']]}",
            flush=True,
        )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({"results": results}, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
