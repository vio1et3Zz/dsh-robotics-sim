"""SO-101 side grasp: straddle the block with the gripper mouth, then lift.

Measured mouth geometry (T06 evidence, body-local):
  fixed bar inner face x=-0.0079, moving bar inner face x=+0.0099 at rest
  mouth centre (x=0.0071, y=0, z=-0.085), mouth axis local -Z, gap local X

Desired grasp orientation: mouth axis -> world +X, gap -> world +Y, so the
palm frame columns are X=(0,1,0), Y=(0,0,-1), Z=(-1,0,0); quat xyzw
(-0.5,-0.5,0.5,0.5).  The block is then clamped between the bars in Y.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from isaaclab.app import AppLauncher

GRASP_LOCAL = (0.0071, 0.0, -0.085)
TARGET_QUAT = (-0.5, -0.5, 0.5, 0.5)
TARGET_COLUMNS = ((0.0, 1.0, 0.0), (0.0, 0.0, -1.0), (-1.0, 0.0, 0.0))
BLOCK_CENTER = (0.18, 0.0, 0.075)
TARGET_BOX_CENTER = (0.20, 0.10, 0.055)
MAX_STEP_RAD = 0.12


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--iterations", type=int, default=700)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"output exists: {args.output}")
    app = AppLauncher(args).app
    report: dict = {"phase": "startup"}
    try:
        import torch
        from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
        from robotics_backends.isaaclab_physx.so101_objects_scene import build_so101_object_scene

        def save() -> None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")

        def marker(message: str) -> None:
            print(f"[side-pick] {message}", flush=True)
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

        target_R = torch.tensor(TARGET_COLUMNS, dtype=torch.float32, device=robot.device)
        target_quat = torch.tensor([TARGET_QUAT], dtype=torch.float32, device=robot.device)
        grasp_local = torch.tensor(GRASP_LOCAL, dtype=torch.float32, device=robot.device)
        grip = {"value": 1.7}

        def step(count: int) -> None:
            for _ in range(count):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(dt)
                joint_pos = robot.data.joint_pos.torch
                if not bool(torch.isfinite(joint_pos).all()):
                    raise RuntimeError("non-finite joint state")

        def set_gripper(value: float, count: int = 140) -> None:
            grip["value"] = value
            q = robot.data.joint_pos.torch.clone()
            q[0, gripper_index] = value
            robot.set_joint_position_target_index(target=q, full_data=True)
            step(count)

        def mouth_state() -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
            pos = robot.data.body_pos_w.torch[0, palm_index]
            quat = robot.data.body_quat_w.torch[0, palm_index]
            rotation = quat_R(quat)
            mouth = pos + rotation @ grasp_local
            direction = rotation @ torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32, device=robot.device)
            gap = rotation @ torch.tensor([1.0, 0.0, 0.0], dtype=torch.float32, device=robot.device)
            return mouth, direction, gap

        def mouth_errors(target: torch.Tensor) -> dict:
            mouth, direction, gap = mouth_state()
            return {
                "mouth_target": [float(v) for v in target.detach().to("cpu", torch.float64)],
                "mouth_world": [float(v) for v in mouth.detach().to("cpu", torch.float64)],
                "position_error_m": float(torch.linalg.norm(mouth - target)),
                "direction": [float(v) for v in direction.detach().to("cpu", torch.float64)],
                "gap_direction": [float(v) for v in gap.detach().to("cpu", torch.float64)],
                "direction_error_deg": float(
                    torch.rad2deg(torch.acos(torch.clamp(torch.dot(direction, -target_R[:, 2]), -1.0, 1.0)))
                ),
                "gap_error_deg": float(
                    torch.rad2deg(torch.acos(torch.clamp(torch.dot(gap, target_R[:, 0]), -1.0, 1.0)))
                ),
            }

        def block_pos() -> list[float]:
            return [float(v) for v in block.data.root_pos_w.torch[0].detach().to("cpu", torch.float64)]

        def solve_to(mouth_target: list[float], iterations: int) -> float:
            target = torch.tensor([mouth_target], dtype=torch.float32, device=robot.device)
            palm_target = target[0] - target_R @ grasp_local
            command = torch.cat((palm_target.unsqueeze(0), target_quat), dim=1)
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
                delta = (q_des - q).clamp(-MAX_STEP_RAD, MAX_STEP_RAD)
                q_des = q + delta
                q_des[0, gripper_index] = grip["value"]
                robot.set_joint_position_target_index(target=q_des, full_data=True)
                step(1)
            mouth, _, _ = mouth_state()
            return float(torch.linalg.norm(mouth - target[0]))

        report.update(
            {
                "grasp_local": list(GRASP_LOCAL),
                "target_quat_xyzw": list(TARGET_QUAT),
                "block_before": block_pos(),
            }
        )
        save()
        set_gripper(1.7, 160)
        marker("gripper-open")

        report["errors_approach"] = solve_to([BLOCK_CENTER[0], BLOCK_CENTER[1], BLOCK_CENTER[2] + 0.05], 400)
        report["state_approach"] = mouth_errors(torch.tensor([BLOCK_CENTER], device=robot.device))
        save()
        marker("approach-done")
        report["errors_grasp"] = solve_to(list(BLOCK_CENTER), args.iterations)
        report["state_grasp"] = mouth_errors(torch.tensor([BLOCK_CENTER], device=robot.device))
        report["block_at_grasp"] = block_pos()
        save()
        marker("grasp-done")

        close_samples = []
        for value in (1.4, 1.2, 1.0, 0.8, 0.6, 0.4, 0.2, 0.0):
            set_gripper(value, 40)
            close_samples.append({"command": value, "block": block_pos()})
            report["close_samples"] = close_samples
            save()
        report["block_after_close"] = block_pos()
        marker("close-done")

        lift_samples = []
        for height in (0.09, 0.11, 0.13, 0.16, 0.18):
            solve_to([BLOCK_CENTER[0], BLOCK_CENTER[1], height], 260)
            mouth = mouth_state()[0]
            lift_samples.append(
                {
                    "mouth_z_target": height,
                    "block": block_pos(),
                    "mouth": [float(v) for v in mouth.detach().to("cpu", torch.float64)],
                }
            )
            report["lift_samples"] = lift_samples
            save()
        report["block_after_lift"] = block_pos()
        marker("lift-done")

        solve_to([TARGET_BOX_CENTER[0], TARGET_BOX_CENTER[1], 0.17], 300)
        report["block_transferred"] = block_pos()
        save()
        solve_to([TARGET_BOX_CENTER[0], TARGET_BOX_CENTER[1], TARGET_BOX_CENTER[2]], 300)
        report["block_lowered"] = block_pos()
        save()
        set_gripper(1.7, 150)
        step(120)
        report["block_after_release"] = block_pos()
        solve_to([TARGET_BOX_CENTER[0], TARGET_BOX_CENTER[1], 0.20], 260)
        report["block_after_retreat"] = block_pos()
        marker("place-done")

        before = report["block_before"]
        after = report["block_after_retreat"]
        lifted = any(sample["block"][2] > 0.11 for sample in lift_samples)
        dropped = report["block_after_lift"][2] < 0.08
        in_target = 0.15 <= after[0] <= 0.25 and 0.05 <= after[1] <= 0.15 and after[2] <= 0.08
        report["checks"] = {
            "grasp_reached": report["errors_grasp"] < 0.01 and report["state_grasp"]["direction_error_deg"] < 15.0,
            "lifted": lifted,
            "not_dropped_during_transfer": not dropped,
            "placed_in_target": in_target,
            "orientation_ok": report["state_grasp"]["gap_error_deg"] < 15.0,
        }
        report["status"] = "passed" if all(report["checks"].values()) else "failed"
        marker("done")
        print(json.dumps({"status": report["status"], "checks": report["checks"], "after": after}))
    except Exception as exc:  # noqa: BLE001 - keep evidence even on failure
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
