"""T05 SO-ARM101 calibration smoke: joints, limits, gripper, stability."""

from __future__ import annotations

import argparse
import json
import math
import time
import traceback
import uuid
from datetime import datetime, timezone
from pathlib import Path

from isaaclab.app import AppLauncher

ROOT = Path(__file__).resolve().parents[3]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="T05 SO-ARM101 calibration")
    parser.add_argument("--output_dir", type=Path, required=True)
    parser.add_argument("--dt", type=float, default=0.005)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error(f"output directory already exists: {args.output_dir}")
    return args


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True)
    started = time.time()
    result: dict = {"status": "failed", "stage": "startup"}
    app = None
    try:
        app = AppLauncher(args).app
        device = getattr(app, "device", None) or args.device or "cuda:0"
        from robotics_backends.isaaclab_physx.so101_scene import build_so101_scene

        import torch

        sim, scene, robot = build_so101_scene(device=device, dt=args.dt)
        joint_names = list(robot.joint_names)
        limits = robot.data.joint_pos_limits.torch[0].detach().to("cpu", torch.float64)
        defaults = robot.data.default_joint_pos.torch[0].detach().to("cpu", torch.float64)
        limits_list = [[float(low), float(high)] for low, high in limits.tolist()]
        default_list = [float(value) for value in defaults.tolist()]
        body_names = list(robot.body_names)
        jaw_index = body_names.index("moving_jaw_so101_v1") if "moving_jaw_so101_v1" in body_names else len(body_names) - 1

        def read() -> list[float]:
            return [float(value) for value in robot.data.joint_pos.torch[0].detach().to("cpu", torch.float64)]

        def command(values: list[float]) -> None:
            target = torch.tensor([values], dtype=torch.float32, device=robot.device)
            robot.set_joint_position_target_index(target=target, full_data=True)
            scene.write_data_to_sim()

        def step(count: int) -> None:
            for _ in range(count):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(args.dt)

        home_target = default_list[:]
        command(home_target)
        step(200)
        home_measured = read()

        move_results = []
        for index, name in enumerate(joint_names):
            before = read()
            target = before[:]
            low, high = limits_list[index]
            margin = min(0.05, (high - low) * 0.1)
            target[index] = min(max(before[index] + 0.05, low + margin), high - margin)
            command(target)
            step(100)
            after = read()
            command(before)
            step(100)
            move_results.append(
                {
                    "joint": name,
                    "index": index,
                    "target": target[index],
                    "before": before[index],
                    "after": after[index],
                    "delta": after[index] - before[index],
                }
            )

        gripper_index = joint_names.index("gripper")
        command([0.0 if i != gripper_index else 0.0 for i in range(len(joint_names))])
        step(150)
        gripper_closed = read()[gripper_index]
        open_target = [0.0 for _ in range(len(joint_names))]
        open_target[gripper_index] = 1.0
        command(open_target)
        step(150)
        gripper_open = read()[gripper_index]
        command([0.0 for _ in range(len(joint_names))])
        step(100)

        command(home_target)
        step(300)
        stability = read()
        drift = max(abs(a - b) for a, b in zip(stability, home_target))
        jaw_pos = robot.data.body_pos_w.torch[0, jaw_index].detach().to("cpu", torch.float64)

        checks = {
            "joint_count_is_six": len(joint_names) == 6,
            "limits_are_radians": max(abs(v) for pair in limits_list for v in pair) <= 3.2,
            "home_error_small": max(abs(a - b) for a, b in zip(home_measured, home_target)) <= 0.1,
            "each_joint_moved": all(item["delta"] > 0.005 for item in move_results),
            "gripper_moved": abs(gripper_open - gripper_closed) >= 0.2,
            "stable_hold": drift <= 0.05,
            "state_finite": all(math.isfinite(v) for v in stability),
        }
        result = {
            "status": "passed" if all(checks.values()) else "failed",
            "stage": "complete",
            "runtime_id": f"runtime_so101_{uuid.uuid4().hex[:12]}",
            "checks": checks,
            "joint_names": joint_names,
            "joint_limits_rad": limits_list,
            "default_joint_pos": default_list,
            "home_measured": home_measured,
            "moves": move_results,
            "gripper": {
                "closed_command_0": gripper_closed,
                "open_command_1": gripper_open,
                "delta": gripper_open - gripper_closed,
                "limits_rad": limits_list[gripper_index],
            },
            "stability": {"drift_rad": drift, "positions": stability},
            "ee_body": body_names[jaw_index],
            "ee_position_world": [float(v) for v in jaw_pos],
            "timings_s": {"total": time.time() - started},
            "at": datetime.now(timezone.utc).isoformat(),
        }
        (args.output_dir / "so101_calibration.json").write_text(
            json.dumps(result, indent=2), encoding="utf-8"
        )
        print(json.dumps({"status": result["status"], "checks": checks}))
    except Exception as error:
        result = {
            "status": "failed",
            "stage": "exception",
            "error": f"{type(error).__name__}: {error}",
            "traceback": traceback.format_exc(),
        }
        (args.output_dir / "so101_calibration.json").write_text(
            json.dumps(result, indent=2), encoding="utf-8"
        )
        print(traceback.format_exc())
    finally:
        if app is not None:
            app.close()
    return 0 if result.get("status") == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
