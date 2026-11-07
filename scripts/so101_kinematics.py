"""SO-101 kinematic model from the USD + reachability sweep (no Kit).

Builds the serial chain base -> shoulder -> upper_arm -> lower_arm -> wrist ->
gripper from the USD joint rest transforms and optimises the five arm joint
angles for a mouth pose target.  Used to choose the T06 workspace.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from pxr import Usd, UsdPhysics
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
USD_PATH = PROJECT_ROOT / "assets" / "isaac6" / "RobotStudio" / "so101_new_calib" / "so101_new_calib.usd"

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRASP_LOCAL = np.array([0.0071, 0.0, -0.085])
MOUTH_AXIS = np.array([0.0, 0.0, -1.0])
GAP_AXIS = np.array([1.0, 0.0, 0.0])


def gf_quat_to_matrix(quat) -> np.ndarray:
    real = float(quat.GetReal())
    imag = quat.GetImaginary()
    return Rotation.from_quat([float(imag[0]), float(imag[1]), float(imag[2]), real]).as_matrix()


def joint_frames(stage) -> dict[str, np.ndarray]:
    frames: dict[str, np.ndarray] = {}
    for joint_name in JOINTS:
        prim = stage.GetPrimAtPath(f"/so101_new_calib/joints/{joint_name}")
        joint = UsdPhysics.Joint(prim)
        t0 = np.eye(4)
        t0[:3, :3] = gf_quat_to_matrix(joint.GetLocalRot0Attr().Get())
        t0[:3, 3] = np.array([float(v) for v in joint.GetLocalPos0Attr().Get()])
        t1 = np.eye(4)
        t1[:3, :3] = gf_quat_to_matrix(joint.GetLocalRot1Attr().Get())
        t1[:3, 3] = np.array([float(v) for v in joint.GetLocalPos1Attr().Get()])
        frames[joint_name] = (t0, np.linalg.inv(t1))
    return frames


def rotation_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    matrix = np.eye(4)
    matrix[0, 0] = c
    matrix[0, 1] = -s
    matrix[1, 0] = s
    matrix[1, 1] = c
    return matrix


def forward(frames: dict[str, np.ndarray], angles: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    for joint_name, angle in zip(JOINTS, angles):
        t0, t1_inv = frames[joint_name]
        transform = transform @ t0 @ rotation_z(float(angle)) @ t1_inv
    return transform


def palm_axes(transform: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    position = transform[:3, 3] + transform[:3, :3] @ GRASP_LOCAL
    direction = transform[:3, :3] @ MOUTH_AXIS
    gap = transform[:3, :3] @ GAP_AXIS
    return position, direction, gap


def solve(
    frames: dict[str, np.ndarray],
    target_position: np.ndarray,
    target_direction: np.ndarray,
    target_gap: np.ndarray,
    bounds: list[tuple[float, float]],
    starts: int = 80,
    seed: int = 7,
) -> dict:
    rng = np.random.default_rng(seed)

    def cost(angles: np.ndarray) -> float:
        transform = forward(frames, angles)
        position, direction, gap = palm_axes(transform)
        alignment = max(0.0, float(np.dot(direction, target_direction)))
        gap_alignment = max(0.0, float(np.dot(gap, target_gap)))
        return float(
            np.sum((position - target_position) ** 2)
            + 0.0015 * (1.0 - alignment)
            + 0.0005 * (1.0 - gap_alignment)
        )

    best: dict | None = None
    for _ in range(starts):
        guess = np.array([rng.uniform(low, high) for low, high in bounds])
        result = minimize(
            cost,
            guess,
            method="L-BFGS-B",
            bounds=bounds,
            options={"maxiter": 400, "ftol": 1e-14, "gtol": 1e-10},
        )
        if best is None or result.fun < best["cost"]:
            best = {"cost": float(result.fun), "angles": [float(v) for v in result.x]}
    assert best is not None
    transform = forward(frames, np.array(best["angles"]))
    position, direction, gap = palm_axes(transform)
    best.update(
        {
            "mouth": [float(v) for v in position],
            "position_error_m": float(np.linalg.norm(position - target_position)),
            "direction_error_deg": float(np.degrees(np.arccos(np.clip(np.dot(direction, target_direction), -1.0, 1.0)))),
            "gap_error_deg": float(np.degrees(np.arccos(np.clip(np.dot(gap, target_gap), -1.0, 1.0)))),
        }
    )
    return best


def rotation_for(direction: tuple[float, float, float]) -> np.ndarray:
    d = np.array(direction, dtype=float)
    d /= np.linalg.norm(d)
    gap = np.array([0.0, 1.0, 0.0])
    if abs(d[1]) > 0.9:
        gap = np.array([1.0, 0.0, 0.0])
    gap = gap - np.dot(gap, d) * d
    gap /= np.linalg.norm(gap)
    z_axis = -d
    y_axis = np.cross(z_axis, gap)
    return np.column_stack([gap, y_axis, z_axis])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--starts", type=int, default=60)
    args = parser.parse_args()
    stage = Usd.Stage.Open(str(USD_PATH))
    frames = joint_frames(stage)

    # verify the rest pose against the known palm transform
    rest = forward(frames, np.zeros(5))
    rest_position = rest[:3, 3]
    expected = np.array([0.0205999, -0.2774733, 0.2668521])
    report: dict = {
        "rest_palm": [float(v) for v in rest_position],
        "rest_palm_expected": [float(v) for v in expected],
        "rest_error_m": float(np.linalg.norm(rest_position - expected)),
        "joint_limits_deg": {
            "shoulder_pan": (-109.99987030029297, 109.99987030029297),
            "shoulder_lift": (-100.00003814697266, 100.00003814697266),
            "elbow_flex": (-96.82986450195312, 96.82986450195312),
            "wrist_flex": (-94.99983215332031, 94.99983215332031),
            "wrist_roll": (-157.2110137939453, 162.78932189941406),
        },
        "candidates": [],
    }
    bounds = [(math.radians(low), math.radians(high)) for low, high in report["joint_limits_deg"].values()]

    directions = {
        "plus_x": (1.0, 0.0, 0.0),
        "minus_y": (0.0, -1.0, 0.0),
        "plus_y": (0.0, 1.0, 0.0),
        "minus_x": (-1.0, 0.0, 0.0),
        "diag_px_my": (0.7071, -0.7071, 0.0),
        "diag_px_py": (0.7071, 0.7071, 0.0),
        "diag_mx_my": (-0.7071, -0.7071, 0.0),
        "diag_mx_py": (-0.7071, 0.7071, 0.0),
        "top": (0.0, 0.0, -1.0),
        "top_px": (0.5, 0.0, -0.866),
        "top_mx": (-0.5, 0.0, -0.866),
    }
    xs = [0.16, 0.18, 0.20, 0.22, 0.24, 0.26, 0.28, 0.30, 0.32]
    for x in xs:
        for label, direction in directions.items():
            rotation = rotation_for(direction)
            target_direction = -rotation[:, 2]
            target_gap = rotation[:, 0]
            mouth_target = np.array([x, 0.0, 0.075])
            entry = solve(
                frames,
                mouth_target,
                target_direction,
                target_gap,
                bounds,
                starts=args.starts,
            )
            entry.update({"x": x, "direction": label})
            report["candidates"].append(entry)
            print(
                f"x={x:.2f} {label:12s} pos={entry['position_error_m']*1000:6.1f}mm "
                f"dir={entry['direction_error_deg']:6.1f} gap={entry['gap_error_deg']:6.1f}",
                flush=True,
            )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"rest_error={report['rest_error_m']:.6f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
