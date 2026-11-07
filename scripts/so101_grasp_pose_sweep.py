"""Sweep reachable (block_x, grasp direction) pairs for the SO-101.

Targets an approach pose 4 cm above the block, never touching the scene.  The
arm is seeded near the requested pan so the DLS solve does not wander through
the joint limits.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from isaaclab.app import AppLauncher

GRASP_LOCAL = (0.0071, 0.0, -0.085)
BLOCK_Z = 0.075
APPROACH_LIFT = 0.04
MAX_STEP_RAD = 0.12

# (label, direction in world for the mouth axis)
DIRECTIONS = [
    ("plus_x", (1.0, 0.0, 0.0)),
    ("minus_y", (0.0, -1.0, 0.0)),
    ("plus_y", (0.0, 1.0, 0.0)),
    ("minus_x", (-1.0, 0.0, 0.0)),
    ("diag_px_my", (0.7071, -0.7071, 0.0)),
    ("diag_px_py", (0.7071, 0.7071, 0.0)),
    ("diag_mx_my", (-0.7071, -0.7071, 0.0)),
    ("top", (0.0, 0.0, -1.0)),
    ("top_px", (0.5, 0.0, -0.866)),
]
BLOCK_XS = (0.18, 0.22, 0.26, 0.30)


def quat_from_matrix(rotation: list[list[float]]) -> tuple[float, float, float, float]:
    m00, m01, m02 = rotation[0]
    m10, m11, m12 = rotation[1]
    m20, m21, m22 = rotation[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    return (x, y, z, w)


def rotation_for(direction: tuple[float, float, float]) -> list[list[float]]:
    d = direction
    # gap axis: horizontal, perpendicular to the mouth axis when possible.
    gap = (0.0, 1.0, 0.0)
    if abs(d[1]) > 0.9:
        gap = (1.0, 0.0, 0.0)
    # orthonormalise gap against d
    dot = gap[0] * d[0] + gap[1] * d[1] + gap[2] * d[2]
    g = (gap[0] - dot * d[0], gap[1] - dot * d[1], gap[2] - dot * d[2])
    norm = math.sqrt(sum(v * v for v in g)) or 1.0
    g = (g[0] / norm, g[1] / norm, g[2] / norm)
    z_axis = (-d[0], -d[1], -d[2])
    y_axis = (
        z_axis[1] * g[2] - z_axis[2] * g[1],
        z_axis[2] * g[0] - z_axis[0] * g[2],
        z_axis[0] * g[1] - z_axis[1] * g[0],
    )
    return [[g[0], y_axis[0], z_axis[0]], [g[1], y_axis[1], z_axis[1]], [g[2], y_axis[2], z_axis[2]]]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--iterations", type=int, default=260)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"output exists: {args.output}")
    app = AppLauncher(args).app
    report: dict = {"candidates": [], "phase": "startup"}
    try:
        import torch
        from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
        from robotics_backends.isaaclab_physx.so101_objects_scene import build_so101_object_scene

        def save() -> None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")

        def marker(message: str) -> None:
            print(f"[pose-sweep] {message}", flush=True)
            report["phase"] = message
            save()

        marker("launch")
        dt = 0.005
        sim, scene, robot, block = build_so101_object_scene(device=getattr(app, "device", "cuda:0"), dt=dt)
        marker("scene-ready")
        body_names = list(robot.body_names)
        joint_names = list(robot.joint_names)
        palm_index = body_names.index("gripper")
        gripper_index = joint_names.index("gripper")
        pan_index = joint_names.index("shoulder_pan")
        lift_index = joint_names.index("shoulder_lift")
        elbow_index = joint_names.index("elbow_flex")
        wrist_flex_index = joint_names.index("wrist_flex")
        limits = robot.data.joint_pos_limits.torch[0].detach()
        low, high = limits[:, 0], limits[:, 1]
        default = robot.data.default_joint_pos.torch.clone()
        ik = DifferentialIKController(
            DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
            num_envs=1,
            device=robot.device,
        )

        def quat_R(quat: torch.Tensor) -> torch.Tensor:
            x, y, z, w = [float(v) for v in quat.detach().to("cpu", torch.float64)]
            return torch.tensor(
                [
                    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
                ],
                dtype=torch.float32,
                device=robot.device,
            )

        grasp_local = torch.tensor(GRASP_LOCAL, dtype=torch.float32, device=robot.device)

        def step(count: int) -> None:
            for _ in range(count):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(dt)
                if not bool(torch.isfinite(robot.data.joint_pos.torch).all()):
                    raise RuntimeError("non-finite joint state")

        def seed(pan_rad: float) -> None:
            q = default.clone()
            q[0, pan_index] = max(min(pan_rad, float(high[pan_index])), float(low[pan_index]))
            q[0, lift_index] = -0.55
            q[0, elbow_index] = 1.15
            q[0, wrist_flex_index] = 0.2
            q[0, gripper_index] = 1.7
            robot.write_joint_state_to_sim(q, torch.zeros_like(q))
            robot.set_joint_position_target_index(target=q, full_data=True)
            step(40)

        def execute(block_x: float, label: str, direction: tuple[float, float, float], iterations: int) -> dict:
            rotation = rotation_for(direction)
            quat_xyzw = quat_from_matrix(rotation)
            target_R = torch.tensor(rotation, dtype=torch.float32, device=robot.device)
            target_quat = torch.tensor([quat_xyzw], dtype=torch.float32, device=robot.device)
            mouth_target = torch.tensor(
                [[block_x, 0.0, BLOCK_Z + APPROACH_LIFT]], dtype=torch.float32, device=robot.device
            )
            palm_target = mouth_target[0] - target_R @ grasp_local
            command = torch.cat((palm_target.unsqueeze(0), target_quat), dim=1)
            pan = math.atan2(direction[1], direction[0]) if abs(direction[2]) < 0.9 else 0.8
            seed(pan + math.pi / 2.0)
            set_gripper = robot.data.joint_pos.torch.clone()
            set_gripper[0, gripper_index] = 1.7
            robot.set_joint_position_target_index(target=set_gripper, full_data=True)
            step(20)
            for _ in range(iterations):
                q = robot.data.joint_pos.torch.clone()
                pos = robot.data.body_pos_w.torch[0, palm_index].unsqueeze(0)
                quat = robot.data.body_quat_w.torch[0, palm_index].unsqueeze(0)
                ik.set_command(command, pos, quat)
                jacobian = robot.data.body_link_jacobian_w.torch[0, palm_index - 1].unsqueeze(0)
                q_des = ik.compute(pos, quat, jacobian, q)
                if not bool(torch.isfinite(q_des).all()):
                    raise RuntimeError("non-finite IK output")
                q_des = torch.max(torch.min(q_des, high.unsqueeze(0)), low.unsqueeze(0))
                q_des = q + (q_des - q).clamp(-MAX_STEP_RAD, MAX_STEP_RAD)
                q_des[0, gripper_index] = 1.7
                robot.set_joint_position_target_index(target=q_des, full_data=True)
                step(1)
            pos = robot.data.body_pos_w.torch[0, palm_index]
            quat = robot.data.body_quat_w.torch[0, palm_index]
            rotation_actual = quat_R(quat)
            mouth = pos + rotation_actual @ grasp_local
            direction_actual = rotation_actual @ torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32, device=robot.device)
            target_direction = torch.tensor(direction, dtype=torch.float32, device=robot.device)
            target_direction = target_direction / torch.linalg.norm(target_direction)
            joints = robot.data.joint_pos.torch[0]
            return {
                "block_x": block_x,
                "direction": label,
                "position_error_m": float(torch.linalg.norm(mouth - mouth_target[0])),
                "direction_error_deg": float(
                    torch.rad2deg(torch.acos(torch.clamp(torch.dot(direction_actual, target_direction), -1.0, 1.0)))
                ),
                "palm_target": [float(v) for v in palm_target.detach().to("cpu", torch.float64)],
                "palm_world": [float(v) for v in pos.detach().to("cpu", torch.float64)],
                "mouth_world": [float(v) for v in mouth.detach().to("cpu", torch.float64)],
                "joints": [float(v) for v in joints.detach().to("cpu", torch.float64)],
                "at_limit": [
                    name
                    for name, index in (
                        ("pan", pan_index),
                        ("lift", lift_index),
                        ("elbow", elbow_index),
                        ("wrist_flex", wrist_flex_index),
                    )
                    if float(joints[index]) <= float(low[index]) + 1e-3
                    or float(joints[index]) >= float(high[index]) - 1e-3
                ],
            }

        for block_x in BLOCK_XS:
            for label, direction in DIRECTIONS:
                try:
                    entry = execute(block_x, label, direction, args.iterations)
                except Exception as error:  # noqa: BLE001
                    entry = {"block_x": block_x, "direction": label, "error": f"{type(error).__name__}: {error}"}
                report["candidates"].append(entry)
                save()
                print(
                    f"[pose-sweep] x={block_x:.2f} {label:12s} pos={entry.get('position_error_m', float('nan')) * 1000:6.1f}mm "
                    f"dir={entry.get('direction_error_deg', float('nan')):6.1f}deg limits={entry.get('at_limit')}",
                    flush=True,
                )
        marker("done")
    except Exception as exc:  # noqa: BLE001
        import traceback

        report["exception"] = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        raise
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
