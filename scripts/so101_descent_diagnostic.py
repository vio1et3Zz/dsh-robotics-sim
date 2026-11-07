"""Diagnose which SO-101 gripper points touch the block during the grasp descent.

Loads the measured palm/jaw point clouds once, then steps the real PhysX scene
from the approach waypoint to the grasp waypoint in small increments and logs
the signed clearance to the current block AABB plus the closest point.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from isaaclab.app import AppLauncher

WAYPOINTS = {
    "approach": [-94.01, -16.81, 20.65, 86.16, 85.99],
    "grasp": [-94.01, -14.69, 31.48, 73.22, 85.99],
}
GRASP_LOCAL = (0.0071, 0.0, -0.085)
POINTS = Path(__file__).resolve().parents[1] / "artifacts" / "t06_2026-11-07_1130" / "gripper_points.json"


def rotate_about(points: np.ndarray, axis: np.ndarray, anchor: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    c, s = math.cos(angle), math.sin(angle)
    rel = points - anchor
    return (
        rel * c
        + np.cross(axis, rel) * s
        + np.outer(rel @ axis, axis) * (1.0 - c)
        + anchor
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    app = AppLauncher(args).app
    log: list[dict] = []
    try:
        import torch
        from robotics_backends.isaaclab_physx.so101_objects_scene import build_so101_object_scene

        data = json.loads(POINTS.read_text(encoding="utf-8"))
        palm_points = np.asarray(data["gripper"]["points_local_body"], dtype=np.float64)[::5]
        jaw_points = np.asarray(data["moving_jaw_so101_v1"]["points_local_body"], dtype=np.float64)[::5]
        hinge = np.array([0.0202, 0.0188, -0.0234])
        jaw_palm_rest = np.stack(
            [jaw_points[:, 0] + hinge[0], -jaw_points[:, 2] + hinge[1], jaw_points[:, 1] + hinge[2]], axis=1
        )

        dt = 0.005
        sim, scene, robot, block = build_so101_object_scene(device=getattr(app, "device", "cuda:0"), dt=dt)
        joint_names = list(robot.joint_names)
        body_names = list(robot.body_names)
        palm_index = body_names.index("gripper")
        gripper_index = joint_names.index("gripper")
        limits = robot.data.joint_pos_limits.torch[0].detach()
        low, high = limits[:, 0], limits[:, 1]

        def step(count: int) -> None:
            for _ in range(count):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(dt)

        def command(angles_deg: list[float], gripper: float, steps: int) -> None:
            start = robot.data.joint_pos.torch.clone()
            goal = start.clone()
            goal[0, :5] = torch.tensor([angles_deg], dtype=torch.float32, device=robot.device) * math.pi / 180.0
            goal[0, 5] = gripper
            goal = torch.max(torch.min(goal, high.unsqueeze(0)), low.unsqueeze(0))
            for index in range(1, steps + 1):
                alpha = index / float(steps)
                robot.set_joint_position_target_index(target=start + (goal - start) * alpha, full_data=True)
                step(1)

        def quat_R(quat: torch.Tensor) -> np.ndarray:
            x, y, z, w = [float(v) for v in quat.detach().to("cpu", torch.float64)]
            return np.array(
                [
                    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
                ],
                dtype=np.float64,
            )

        def gripper_frames() -> tuple[np.ndarray, np.ndarray, float]:
            palm_pos = robot.data.body_pos_w.torch[0, palm_index].detach().to("cpu", torch.float64).numpy()
            palm_quat = robot.data.body_quat_w.torch[0, palm_index]
            rotation = quat_R(palm_quat)
            jaw_angle = float(robot.data.joint_pos.torch[0, gripper_index])
            return palm_pos, rotation, jaw_angle

        def clearance(points_world: np.ndarray, block_pos: list[float]) -> tuple[float, list[float]]:
            center = np.array(block_pos)
            lo = center - 0.015
            hi = center + 0.015
            d = np.maximum.reduce(
                [
                    lo[0] - points_world[:, 0],
                    points_world[:, 0] - hi[0],
                    lo[1] - points_world[:, 1],
                    points_world[:, 1] - hi[1],
                    lo[2] - points_world[:, 2],
                    points_world[:, 2] - hi[2],
                ]
            )
            outside = np.maximum(d, 0.0)
            inside = d < 0.0
            signed = outside.copy()
            if inside.any():
                depth = np.minimum.reduce(
                    [
                        hi[0] - points_world[:, 0],
                        points_world[:, 0] - lo[0],
                        hi[1] - points_world[:, 1],
                        points_world[:, 1] - lo[1],
                        hi[2] - points_world[:, 2],
                        points_world[:, 2] - lo[2],
                    ]
                )
                signed[inside] = -depth[inside]
            index = int(np.argmin(signed))
            return float(signed[index]), [float(v) for v in points_world[index]]

        command(WAYPOINTS["approach"], 0.4, 260)
        step(60)
        block_before = [float(v) for v in block.data.root_pos_w.torch[0].detach().to("cpu", torch.float64)]
        start = robot.data.joint_pos.torch.clone()
        goal = start.clone()
        goal[0, :5] = torch.tensor([WAYPOINTS["grasp"]], dtype=torch.float32, device=robot.device) * math.pi / 180.0
        goal[0, 5] = 0.4
        goal = torch.max(torch.min(goal, high.unsqueeze(0)), low.unsqueeze(0))
        for index in range(1, 121):
            robot.set_joint_position_target_index(target=start + (goal - start) * (index / 120.0), full_data=True)
            step(1)
            if index % 5:
                continue
            block_pos = [float(v) for v in block.data.root_pos_w.torch[0].detach().to("cpu", torch.float64)]
            palm_pos, rotation, jaw_angle = gripper_frames()
            palm_world = palm_points @ rotation.T + palm_pos
            jaw_local = rotate_about(jaw_palm_rest, np.array([0.0, -1.0, 0.0]), hinge, jaw_angle)
            jaw_world = jaw_local @ rotation.T + palm_pos
            palm_clear, palm_point = clearance(palm_world, block_pos)
            jaw_clear, jaw_point = clearance(jaw_world, block_pos)
            mouth = palm_pos + rotation @ np.array(GRASP_LOCAL)
            log.append(
                {
                    "step": index,
                    "joints": [float(v) for v in robot.data.joint_pos.torch[0].detach().to("cpu", torch.float64)],
                    "mouth": [float(v) for v in mouth],
                    "block": block_pos,
                    "delta_block": [float(a - b) for a, b in zip(block_pos, block_before)],
                    "palm_clearance_mm": palm_clear * 1000.0,
                    "palm_closest": palm_point,
                    "jaw_clearance_mm": jaw_clear * 1000.0,
                    "jaw_closest": jaw_point,
                    "jaw_angle": jaw_angle,
                }
            )
        step(60)
        log.append(
            {
                "step": "final",
                "block": [float(v) for v in block.data.root_pos_w.torch[0].detach().to("cpu", torch.float64)],
            }
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({"block_before": block_before, "log": log}, indent=2), encoding="utf-8")
        print(json.dumps({"entries": len(log), "final": log[-1]}))
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
