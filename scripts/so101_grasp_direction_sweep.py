"""Sweep SO-101 grasp directions to find a reachable pose-IK grasp.

For each candidate mouth direction in the XY plane the probe resets the arm,
opens the jaw, runs the pose IK to the approach pose (5 cm above the block)
and records the residual mouth position error plus the finger/gap direction
errors.  No contact is attempted here.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from isaaclab.app import AppLauncher

GRASP_LOCAL = (0.0071, 0.0, -0.085)
BLOCK_CENTER = (0.18, 0.0, 0.075)
CANDIDATES = [0, -20, -30, -40, -50, -60, -70, -80, -90, 20, 30, 40, 50, 60, 70, 80, 90, 120, 150, 180, -120, -150]
MAX_STEP_RAD = 0.12


def quat_from_matrix(rotation: list[list[float]]) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to an xyzw quaternion."""
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


def target_rotation(degrees: float) -> list[list[float]]:
    theta = math.radians(degrees)
    direction = [math.cos(theta), math.sin(theta), 0.0]
    gap = [-math.sin(theta), math.cos(theta), 0.0]
    z_axis = [-direction[0], -direction[1], -direction[2]]
    y_axis = [
        z_axis[1] * gap[2] - z_axis[2] * gap[1],
        z_axis[2] * gap[0] - z_axis[0] * gap[2],
        z_axis[0] * gap[1] - z_axis[1] * gap[0],
    ]
    return [[gap[0], y_axis[0], z_axis[0]], [gap[1], y_axis[1], z_axis[1]], [gap[2], y_axis[2], z_axis[2]]]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--iterations", type=int, default=300)
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
            print(f"[sweep] {message}", flush=True)
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

        def reset_robot() -> None:
            robot.write_joint_state_to_sim(default, torch.zeros_like(default))
            robot.set_joint_position_target_index(target=default, full_data=True)
            step(30)

        def set_gripper(value: float, count: int = 120) -> None:
            q = robot.data.joint_pos.torch.clone()
            q[0, gripper_index] = value
            robot.set_joint_position_target_index(target=q, full_data=True)
            step(count)

        def execute(degrees: float, iterations: int) -> dict:
            rotation = target_rotation(degrees)
            quat_xyzw = quat_from_matrix(rotation)
            target_R = torch.tensor(rotation, dtype=torch.float32, device=robot.device)
            target_quat = torch.tensor([quat_xyzw], dtype=torch.float32, device=robot.device)
            mouth_target = torch.tensor(
                [[BLOCK_CENTER[0], BLOCK_CENTER[1], BLOCK_CENTER[2] + 0.05]], dtype=torch.float32, device=robot.device
            )
            palm_target = mouth_target[0] - target_R @ grasp_local
            command = torch.cat((palm_target.unsqueeze(0), target_quat), dim=1)
            reset_robot()
            set_gripper(1.7, 120)
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
            direction = rotation_actual @ torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32, device=robot.device)
            gap = rotation_actual @ torch.tensor([1.0, 0.0, 0.0], dtype=torch.float32, device=robot.device)
            target_direction = torch.tensor(
                [math.cos(math.radians(degrees)), math.sin(math.radians(degrees)), 0.0],
                dtype=torch.float32,
                device=robot.device,
            )
            target_gap = torch.tensor(
                [-math.sin(math.radians(degrees)), math.cos(math.radians(degrees)), 0.0],
                dtype=torch.float32,
                device=robot.device,
            )
            return {
                "degrees": degrees,
                "quat_xyzw": [float(v) for v in quat_xyzw],
                "position_error_m": float(torch.linalg.norm(mouth - mouth_target[0])),
                "direction_error_deg": float(
                    torch.rad2deg(torch.acos(torch.clamp(torch.dot(direction, target_direction), -1.0, 1.0)))
                ),
                "gap_error_deg": float(
                    torch.rad2deg(torch.acos(torch.clamp(torch.dot(gap, target_gap), -1.0, 1.0)))
                ),
                "mouth_world": [float(v) for v in mouth.detach().to("cpu", torch.float64)],
                "palm_world": [float(v) for v in pos.detach().to("cpu", torch.float64)],
                "palm_quat_xyzw": [float(v) for v in quat.detach().to("cpu", torch.float64)],
                "joints": [float(v) for v in robot.data.joint_pos.torch[0].detach().to("cpu", torch.float64)],
            }

        for degrees in CANDIDATES:
            try:
                entry = execute(float(degrees), args.iterations)
            except Exception as error:  # noqa: BLE001 - one bad candidate must not stop the sweep
                entry = {"degrees": degrees, "error": f"{type(error).__name__}: {error}"}
            report["candidates"].append(entry)
            save()
            print(
                f"[sweep] {degrees:+.0f} deg -> pos {entry.get('position_error_m', float('nan')) * 1000 if 'position_error_m' in entry else float('nan'):.1f} mm, "
                f"dir {entry.get('direction_error_deg', float('nan')):.1f} deg, gap {entry.get('gap_error_deg', float('nan')):.1f} deg",
                flush=True,
            )
        marker("done")
    except Exception as exc:  # noqa: BLE001 - keep evidence
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
