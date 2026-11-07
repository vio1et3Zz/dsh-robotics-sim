"""SO-101 top grasp pick/place in the real PhysX object scene.

Seed from ``so101_solve_pose.py --direction top`` (USD chain, exact), jaws are
opened before the seed pose is applied, then the physics pose-IK refines the
mouth to the measured mouth frame:
  mouth centre (0.0071, 0, -0.085) in the palm frame,
  fixed bar inner face -0.0079, moving bar inner face +0.0099.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from isaaclab.app import AppLauncher

SEED_ANGLES_DEG = [-95.79286962430396, -16.604288581127793, 20.44962262480745, 86.15488348257483, 84.20670898037434]
GRASP_LOCAL = (0.0071, 0.0, -0.085)
TOP_QUAT = (0.0, 0.0, 0.7071067811865476, 0.7071067811865476)
TOP_R = ((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
MAX_STEP_RAD = 0.08
OPEN_JAW = 0.4


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--place-x", type=float, default=0.22)
    parser.add_argument("--place-y", type=float, default=0.05)
    parser.add_argument("--place-z", type=float, default=0.075)
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
            print(f"[top-pick] {message}", flush=True)
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

        target_R = torch.tensor(TOP_R, dtype=torch.float32, device=robot.device)
        target_quat = torch.tensor([TOP_QUAT], dtype=torch.float32, device=robot.device)
        grasp_local = torch.tensor(GRASP_LOCAL, dtype=torch.float32, device=robot.device)
        grip = {"value": OPEN_JAW}

        def step(count: int) -> None:
            for _ in range(count):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(dt)
                if not bool(torch.isfinite(robot.data.joint_pos.torch).all()):
                    raise RuntimeError("non-finite joint state")

        def set_gripper(value: float, count: int = 60) -> None:
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

        def block_pos() -> list[float]:
            return [float(v) for v in block.data.root_pos_w.torch[0].detach().to("cpu", torch.float64)]

        def state() -> dict:
            mouth, direction, gap = mouth_state()
            palm_pos = robot.data.body_pos_w.torch[0, palm_index]
            palm_quat = robot.data.body_quat_w.torch[0, palm_index]
            return {
                "block": block_pos(),
                "mouth": [float(v) for v in mouth.detach().to("cpu", torch.float64)],
                "direction": [float(v) for v in direction.detach().to("cpu", torch.float64)],
                "gap": [float(v) for v in gap.detach().to("cpu", torch.float64)],
                "palm": [float(v) for v in palm_pos.detach().to("cpu", torch.float64)],
                "palm_quat_xyzw": [float(v) for v in palm_quat.detach().to("cpu", torch.float64)],
                "joints": [float(v) for v in robot.data.joint_pos.torch[0].detach().to("cpu", torch.float64)],
            }

        def solve_to(mouth_target: tuple[float, float, float], iterations: int) -> dict:
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
                q_des = q + (q_des - q).clamp(-MAX_STEP_RAD, MAX_STEP_RAD)
                q_des[0, gripper_index] = grip["value"]
                robot.set_joint_position_target_index(target=q_des, full_data=True)
                step(1)
            mouth, direction, _ = mouth_state()
            return {
                "target": [float(v) for v in mouth_target],
                "error_m": float(torch.linalg.norm(mouth - target[0])),
                "direction": [float(v) for v in direction.detach().to("cpu", torch.float64)],
                "state": state(),
            }

        report.update(
            {
                "seed_angles_deg": SEED_ANGLES_DEG,
                "block_before": block_pos(),
                "place_target": [args.place_x, args.place_y, args.place_z],
            }
        )
        # Open the jaw in the same write as the seed so the descending bars
        # never close on the block.
        seed = torch.tensor([SEED_ANGLES_DEG], dtype=torch.float32, device=robot.device) * math.pi / 180.0
        full = robot.data.joint_pos.torch.clone()
        full[0, :5] = seed[0]
        full[0, gripper_index] = OPEN_JAW
        robot.write_joint_state_to_sim(full, torch.zeros_like(full))
        robot.set_joint_position_target_index(target=full, full_data=True)
        step(100)
        report["seeded_state"] = state()
        marker("seeded")

        report["approach"] = solve_to((0.18, 0.0, 0.115), 180)
        report["block_after_approach"] = block_pos()
        marker("approach")
        report["grasp"] = solve_to((0.18, 0.0, 0.085), 200)
        report["block_at_grasp"] = block_pos()
        marker("grasp")

        close_samples = []
        for value in (0.35, 0.3, 0.25, 0.2, 0.15, 0.1, 0.05, 0.0):
            set_gripper(value, 45)
            close_samples.append({"command": value, "block": block_pos()})
            report["close_samples"] = close_samples
            save()
        report["block_after_close"] = block_pos()
        marker("closed")

        lift = []
        for height in (0.095, 0.11, 0.125):
            result = solve_to((0.18, 0.0, height), 160)
            lift.append({"target_z": height, "error_m": result["error_m"], "block": block_pos()})
            report["lift"] = lift
            save()
        report["block_after_lift"] = block_pos()
        marker("lifted")

        report["transfer"] = solve_to((args.place_x, args.place_y, 0.13), 220)
        report["block_transferred"] = block_pos()
        save()
        report["lower"] = solve_to((args.place_x, args.place_y, args.place_z), 220)
        report["block_lowered"] = block_pos()
        save()
        set_gripper(0.6, 140)
        step(100)
        report["block_after_release"] = block_pos()
        report["retreat"] = solve_to((args.place_x, args.place_y, 0.16), 180)
        report["block_after_retreat"] = block_pos()
        marker("placed")

        before = report["block_before"]
        after = report["block_after_retreat"]
        lifted = any(entry["block"][2] > before[2] + 0.03 for entry in lift)
        final_ok = (
            abs(after[0] - args.place_x) <= 0.05
            and abs(after[1] - args.place_y) <= 0.05
            and after[2] <= 0.09
        )
        report["checks"] = {
            "grasp_reached": report["grasp"]["error_m"] < 0.012,
            "lifted": lifted,
            "not_dropped": report["block_after_lift"][2] > before[2] + 0.02,
            "placed_in_target_region": final_ok,
            "released": abs(report["block_after_retreat"][2] - report["block_after_lift"][2]) < 0.05,
        }
        report["status"] = "passed" if all(report["checks"].values()) else "failed"
        marker("done")
        print(json.dumps({"status": report["status"], "checks": report["checks"], "block_after": after}))
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

