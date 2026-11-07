"""SO-101 pick/place driven by offline-solved joint waypoints (real PhysX).

Waypoints come from ``so101_pick_sequence_solve.py`` (USD chain + scipy, exact
mouth poses with the measured mouth frame).  Joint-space linear interpolation
keeps the motion smooth; the measured mouth/block at each waypoint is saved.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from isaaclab.app import AppLauncher

WAYPOINTS = {
    "approach": [-94.01, -16.81, 20.65, 86.16, 85.99],
    "grasp": [-94.01, -14.69, 31.48, 73.22, 85.99],
    "lift1": [-94.01, -16.49, 24.59, 81.90, 85.99],
    "lift2": [-94.01, -16.61, 14.09, 92.53, 85.99],
    "transfer": [-108.37, 7.93, -13.92, 95.00, 71.58],
    "place": [-108.37, 10.81, 5.67, 73.51, 71.63],
    "retreat": [-108.37, -1.66, -13.47, 95.00, 71.44],
}
GRASP_LOCAL = (0.0071, 0.0, -0.085)
BLOCK_BEFORE = (0.18, 0.0, 0.075)
PLACE_TARGET = (0.22, 0.05, 0.055)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"output exists: {args.output}")
    app = AppLauncher(args).app
    report: dict = {"phase": "startup", "waypoints_deg": WAYPOINTS}
    try:
        import torch
        from robotics_backends.isaaclab_physx.so101_objects_scene import build_so101_object_scene

        def save() -> None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")

        def marker(message: str) -> None:
            print(f"[waypoint] {message}", flush=True)
            report["phase"] = message
            save()

        marker("launch")
        dt = 0.005
        sim, scene, robot, block = build_so101_object_scene(device=getattr(app, "device", "cuda:0"), dt=dt)
        marker("scene-ready")
        joint_names = list(robot.joint_names)
        body_names = list(robot.body_names)
        palm_index = body_names.index("gripper")
        gripper_index = joint_names.index("gripper")
        limits = robot.data.joint_pos_limits.torch[0].detach()
        low, high = limits[:, 0], limits[:, 1]

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

        def block_pos() -> list[float]:
            return [float(v) for v in block.data.root_pos_w.torch[0].detach().to("cpu", torch.float64)]

        def mouth() -> list[float]:
            pos = robot.data.body_pos_w.torch[0, palm_index]
            quat = robot.data.body_quat_w.torch[0, palm_index]
            return [float(v) for v in (pos + quat_R(quat) @ grasp_local).detach().to("cpu", torch.float64)]

        def state() -> dict:
            return {
                "block": block_pos(),
                "mouth": mouth(),
                "joints": [float(v) for v in robot.data.joint_pos.torch[0].detach().to("cpu", torch.float64)],
                "joint_targets": [
                    float(v) for v in robot.data.joint_pos_target.torch[0].detach().to("cpu", torch.float64)
                ]
                if hasattr(robot.data, "joint_pos_target")
                else None,
            }

        def command(angles_deg: list[float], gripper: float, steps: int) -> None:
            start = robot.data.joint_pos.torch.clone()
            goal = start.clone()
            goal[0, :5] = torch.tensor([angles_deg], dtype=torch.float32, device=robot.device) * math.pi / 180.0
            goal[0, 5] = gripper
            # clamp the arm goals to the calibrated limits
            goal = torch.max(torch.min(goal, high.unsqueeze(0)), low.unsqueeze(0))
            for index in range(1, steps + 1):
                alpha = index / float(steps)
                target = start + (goal - start) * alpha
                robot.set_joint_position_target_index(target=target, full_data=True)
                step(1)

        def hold(steps: int) -> None:
            # keep the last commanded arm target; only let the PD settle
            step(steps)

        report["block_before"] = block_pos()
        save()
        # home -> approach with the jaw open
        command(WAYPOINTS["approach"], 0.4, 260)
        hold(60)
        report["approach"] = state()
        report["block_after_approach"] = block_pos()
        marker("approach")

        command(WAYPOINTS["grasp"], 0.4, 140)
        hold(60)
        report["grasp"] = state()
        report["block_at_grasp"] = block_pos()
        marker("grasp")

        # close slowly so the block is squeezed, not hit; keep the commanded
        # grasp target so the arm does not sag back while the jaw closes.
        base = robot.data.joint_pos_target.torch.clone()
        for value in (0.35, 0.30, 0.25, 0.20, 0.15, 0.10, 0.05, 0.0):
            target = base.clone()
            target[0, gripper_index] = value
            robot.set_joint_position_target_index(target=target, full_data=True)
            step(35)
            report.setdefault("close_samples", []).append({"command": value, "block": block_pos()})
            save()
        report["block_after_close"] = block_pos()
        marker("closed")

        command(WAYPOINTS["lift1"], 0.0, 130)
        hold(50)
        report["lift1"] = state()
        report["block_lift1"] = block_pos()
        save()
        command(WAYPOINTS["lift2"], 0.0, 130)
        hold(50)
        report["lift2"] = state()
        report["block_lift2"] = block_pos()
        marker("lifted")

        command(WAYPOINTS["transfer"], 0.0, 180)
        hold(50)
        report["transfer"] = state()
        report["block_transfer"] = block_pos()
        save()
        command(WAYPOINTS["place"], 0.0, 160)
        hold(50)
        report["place"] = state()
        report["block_place"] = block_pos()
        save()
        hold(150)
        step(100)
        report["block_released"] = block_pos()
        command(WAYPOINTS["retreat"], 0.6, 160)
        hold(50)
        report["block_after_retreat"] = block_pos()
        marker("placed")

        before = report["block_before"]
        grasp_mouth = report["grasp"]["mouth"]
        lifted_z = report["block_lift2"][2]
        checks = {
            "block_untouched_on_approach": math.dist(report["block_after_approach"], before) < 0.01,
            "grasp_centered": math.dist(grasp_mouth[:2], [before[0], before[1]]) < 0.008,
            "lifted": lifted_z > before[2] + 0.03,
            "carried": math.dist(report["block_transfer"][:2], [0.22, 0.05]) < 0.05,
            "placed_in_target": math.dist(report["block_after_retreat"][:2], [0.22, 0.05]) < 0.04
            and report["block_after_retreat"][2] <= 0.09,
            "released": report["block_after_retreat"][2] < 0.09,
        }
        report["checks"] = checks
        report["status"] = "passed" if all(checks.values()) else "failed"
        marker("done")
        print(json.dumps({"status": report["status"], "checks": checks, "after": report["block_after_retreat"]}))
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


