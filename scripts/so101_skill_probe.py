"""Run the SO101PickPlace skill class in success or injected-failure modes."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from isaaclab.app import AppLauncher


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["success", "empty", "grasp_missed", "unreachable"], required=True)
    parser.add_argument("--output", type=Path, required=True)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"output exists: {args.output}")
    app = AppLauncher(args).app
    result: dict = {"mode": args.mode}
    try:
        import torch
        from robotics_backends.isaaclab_physx.so101_objects_scene import build_so101_object_scene
        from robotics_backends.isaaclab_physx.so101_pick_place import SO101PickPlace, SkillCanceled

        dt = 0.005
        sim, scene, robot, block = build_so101_object_scene(device=getattr(app, "device", "cuda:0"), dt=dt)
        # The target box is a kinematic AssetBaseCfg; fetching it from the scene
        # is not needed for the physical verification and stresses the frame
        # view on this build, so the skill uses the declared place target.
        target_box = None

        def step(count: int) -> None:
            for _ in range(count):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(dt)

        def move_block(position: tuple[float, float, float], count: int = 30) -> None:
            pose = torch.tensor([[position[0], position[1], position[2], 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32, device=robot.device)
            block.write_root_pose_to_sim(pose)
            block.write_root_velocity_to_sim(torch.zeros((1, 6), dtype=torch.float32, device=robot.device))
            step(count)

        if args.mode == "grasp_missed":
            move_block((0.20, 0.0, 0.075))
            result["block_moved_to"] = [0.20, 0.0, 0.075]

        skill = SO101PickPlace(sim, scene, robot, block, target_box, dt=dt)
        fault = None
        if args.mode == "empty":
            def fault() -> None:
                move_block((0.28, 0.0, 0.15))
                result["fault"] = "block moved away after grasp verification"

        place_xy = (0.30, 0.20) if args.mode == "unreachable" else (0.22, 0.05)
        result["place_xy"] = list(place_xy)
        before = skill.observe()
        result["before"] = before
        try:
            outcome = skill.execute(place_xy, fault_after_grasp=fault)
            result["outcome"] = outcome.to_dict()
        except SkillCanceled as canceled:
            result["outcome"] = {"passed": False, "failure": "CANCELED", "message": str(canceled)}
        result["after"] = skill.observe()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps({"mode": args.mode, "outcome": result["outcome"]}))
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
