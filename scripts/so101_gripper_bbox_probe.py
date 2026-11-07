"""Print SO-101 jaw/palm world AABBs at open and closed gripper commands."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from isaaclab.app import AppLauncher


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    app = AppLauncher(args).app
    try:
        import torch
        import omni.usd
        from pxr import Usd, UsdGeom, UsdPhysics
        from robotics_backends.isaaclab_physx.so101_objects_scene import build_so101_object_scene

        dt = 0.005
        sim, scene, robot, block = build_so101_object_scene(device=getattr(app, "device", "cuda:0"), dt=dt)
        stage = omni.usd.get_context().get_stage()
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        gripper_index = list(robot.joint_names).index("gripper")

        def prim(name: str):
            for candidate in stage.Traverse():
                if (
                    candidate.GetName() == name
                    and "/Robot/" in str(candidate.GetPath())
                    and candidate.HasAPI(UsdPhysics.RigidBodyAPI)
                ):
                    return candidate
            raise RuntimeError(name)

        def box(name: str) -> dict:
            aligned = cache.ComputeWorldBound(prim(name)).ComputeAlignedRange()
            return {
                "min": [float(v) for v in aligned.GetMin()],
                "max": [float(v) for v in aligned.GetMax()],
            }

        def step(count: int) -> None:
            for _ in range(count):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(dt)

        def command(value: float) -> None:
            q = robot.data.joint_pos.torch.clone()
            q[0, gripper_index] = value
            robot.set_joint_position_target_index(target=q, full_data=True)
            step(150)

        report = {}
        for label, value in (("open_1p7", 1.7), ("open_1p0", 1.0), ("closed_0", 0.0)):
            command(value)
            report[label] = {
                "jaw": box("moving_jaw_so101_v1"),
                "palm": box("gripper"),
                "joint": float(robot.data.joint_pos.torch[0, gripper_index]),
            }
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2))
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
