"""Measure SO-101 jaw/palm collision geometry and its motion at runtime.

The USD BBoxCache does not follow the PhysX state, so this probe extracts the
collision mesh vertices in each link's local frame once (stage starts at the
spawn pose) and then rebuilds the world AABB from the live body poses.
"""

from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path

from isaaclab.app import AppLauncher


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"output exists: {args.output}")
    app = AppLauncher(args).app
    try:
        import numpy as np
        import torch
        import omni.usd
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        from robotics_backends.isaaclab_physx.so101_objects_scene import build_so101_object_scene

        dt = 0.005
        sim, scene, robot, block = build_so101_object_scene(device=getattr(app, "device", "cuda:0"), dt=dt)
        stage = omni.usd.get_context().get_stage()
        body_names = list(robot.body_names)
        joint_names = list(robot.joint_names)
        palm_index = body_names.index("gripper")
        jaw_index = body_names.index("moving_jaw_so101_v1")
        gripper_index = joint_names.index("gripper")

        def prim_named(name: str):
            for candidate in stage.Traverse():
                if (
                    candidate.GetName() == name
                    and "/Robot/" in str(candidate.GetPath())
                    and candidate.HasAPI(UsdPhysics.RigidBodyAPI)
                ):
                    return candidate
            raise RuntimeError(name)

        def collision_points_local(body_prim) -> np.ndarray:
            body_xf = UsdGeom.Xformable(body_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            inv_body = body_xf.GetInverse()
            points: list[tuple[float, float, float]] = []
            for prim in Usd.PrimRange(body_prim):
                if not prim.HasAPI(UsdPhysics.CollisionAPI):
                    continue
                xf = UsdGeom.Xformable(prim)
                world = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default()) if xf else Gf.Matrix4d(1.0)
                matrix = inv_body * world
                if prim.IsA(UsdGeom.Mesh):
                    mesh = UsdGeom.Mesh(prim)
                    mesh_points = mesh.GetPointsAttr().Get() or []
                    for value in mesh_points:
                        world_point = matrix.Transform(Gf.Vec3d(value))
                        points.append((world_point[0], world_point[1], world_point[2]))
                elif prim.IsA(UsdGeom.Cube):
                    half = 0.5 * float(UsdGeom.Cube(prim).GetSizeAttr().Get() or 1.0)
                    for corner in itertools.product((-half, half), repeat=3):
                        world_point = matrix.Transform(Gf.Vec3d(*corner))
                        points.append((world_point[0], world_point[1], world_point[2]))
                elif prim.IsA(UsdGeom.Sphere):
                    center = matrix.Transform(Gf.Vec3d(0.0, 0.0, 0.0))
                    radius = float(UsdGeom.Sphere(prim).GetRadiusAttr().Get() or 0.0)
                    points.append((center[0] + radius, center[1], center[2]))
                    points.append((center[0] - radius, center[1], center[2]))
                    points.append((center[0], center[1] + radius, center[2]))
                    points.append((center[0], center[1] - radius, center[2]))
                    points.append((center[0], center[1], center[2] + radius))
                    points.append((center[0], center[1], center[2] - radius))
            if not points:
                raise RuntimeError(f"no collision points under {body_prim.GetPath()}")
            return np.asarray(points, dtype=np.float64)

        palm_prim = prim_named("gripper")
        jaw_prim = prim_named("moving_jaw_so101_v1")
        palm_local = collision_points_local(palm_prim)
        jaw_local = collision_points_local(jaw_prim)

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

        def dynamic_box(local: np.ndarray, index: int) -> dict:
            points = torch.tensor(local, dtype=torch.float32, device=robot.device)
            pos = robot.data.body_pos_w.torch[0, index]
            quat = robot.data.body_quat_w.torch[0, index]
            world = points @ quat_R(quat).T + pos
            low = world.min(dim=0).values
            high = world.max(dim=0).values
            return {
                "min": [float(v) for v in low.detach().to("cpu", torch.float64)],
                "max": [float(v) for v in high.detach().to("cpu", torch.float64)],
                "center": [float(v) for v in (0.5 * (low + high)).detach().to("cpu", torch.float64)],
            }

        def pose(index: int) -> dict:
            return {
                "pos": [float(v) for v in robot.data.body_pos_w.torch[0, index].detach().to("cpu", torch.float64)],
                "quat_xyzw": [
                    float(v) for v in robot.data.body_quat_w.torch[0, index].detach().to("cpu", torch.float64)
                ],
            }

        def relative(index: int, reference: int) -> dict:
            world_pos = robot.data.body_pos_w.torch[0, index] - robot.data.body_pos_w.torch[0, reference]
            rotation = quat_R(robot.data.body_quat_w.torch[0, reference])
            local_pos = rotation.T @ world_pos
            relative_rotation = rotation.T @ quat_R(robot.data.body_quat_w.torch[0, index])
            return {
                "pos": [float(v) for v in local_pos.detach().to("cpu", torch.float64)],
                "rotation": [[float(v) for v in row] for row in relative_rotation.detach().to("cpu", torch.float64)],
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
            step(160)

        report: dict = {
            "body_names": body_names,
            "joint_names": joint_names,
            "limits_rad": {
                "gripper": [
                    float(robot.data.joint_pos_limits.torch[0, gripper_index, 0]),
                    float(robot.data.joint_pos_limits.torch[0, gripper_index, 1]),
                ]
            },
            "collision_points": {
                "palm": int(len(palm_local)),
                "jaw": int(len(jaw_local)),
            },
            "local_aabb": {
                "palm": {
                    "min": [float(v) for v in palm_local.min(axis=0)],
                    "max": [float(v) for v in palm_local.max(axis=0)],
                },
                "jaw": {
                    "min": [float(v) for v in jaw_local.min(axis=0)],
                    "max": [float(v) for v in jaw_local.max(axis=0)],
                },
            },
            "samples": {},
        }

        for label, value in (("open_1p7", 1.7), ("open_1p0", 1.0), ("closed_0", 0.0), ("closed_limit", -0.1745)):
            command(value)
            report["samples"][label] = {
                "command": value,
                "joint_rad": float(robot.data.joint_pos.torch[0, gripper_index]),
                "palm": pose(palm_index),
                "jaw": pose(jaw_index),
                "jaw_in_palm": relative(jaw_index, palm_index),
                "palm_world_aabb": dynamic_box(palm_local, palm_index),
                "jaw_world_aabb": dynamic_box(jaw_local, jaw_index),
            }

        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(
            json.dumps(
                {
                    "body_names": body_names,
                    "gripper_limits": report["limits_rad"]["gripper"],
                    "local_aabb": report["local_aabb"],
                    "closed_limit_jaw_in_palm": report["samples"]["closed_limit"]["jaw_in_palm"],
                }
            )
        )
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
