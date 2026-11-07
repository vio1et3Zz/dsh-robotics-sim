"""Probe SO-ARM101 pick/place with differential IK and the revolute gripper."""

from __future__ import annotations

import argparse
import json
import math
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
        import torch
        from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
        from robotics_backends.isaaclab_physx.so101_objects_scene import build_so101_object_scene

        dt = 0.005
        sim, scene, robot, block = build_so101_object_scene(device=getattr(app, "device", "cuda:0"), dt=dt)
        body_names = list(robot.body_names)
        jaw_index = body_names.index("moving_jaw_so101_v1")
        jacobi_index = jaw_index - 1
        gripper_index = list(robot.joint_names).index("gripper")
        limits = robot.data.joint_pos_limits.torch[0].detach()
        low, high = limits[:, 0], limits[:, 1]
        ik = DifferentialIKController(
            DifferentialIKControllerCfg(command_type="position", use_relative_mode=False, ik_method="dls"),
            num_envs=1,
            device=robot.device,
        )

        def step(count: int) -> None:
            for _ in range(count):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(dt)

        def set_gripper(value: float, count: int = 60) -> None:
            q = robot.data.joint_pos.torch.clone()
            q[0, gripper_index] = value
            robot.set_joint_position_target_index(target=q, full_data=True)
            step(count)

        def ik_to(target: list[float], gripper_value: float, iterations: int = 250) -> float:
            target_t = torch.tensor([target], dtype=torch.float32, device=robot.device)
            for _ in range(iterations):
                q = robot.data.joint_pos.torch.clone()
                pos = robot.data.body_pos_w.torch[0, jaw_index].unsqueeze(0)
                quat = robot.data.body_quat_w.torch[0, jaw_index].unsqueeze(0)
                ik.set_command(target_t, pos, quat)
                jacobian = robot.data.body_link_jacobian_w.torch[0, jacobi_index].unsqueeze(0)
                q_des = ik.compute(pos, quat, jacobian, q)
                q_des = torch.max(torch.min(q_des, high.unsqueeze(0)), low.unsqueeze(0))
                q_des[0, gripper_index] = gripper_value
                robot.set_joint_position_target_index(target=q_des, full_data=True)
                step(2)
            final = robot.data.body_pos_w.torch[0, jaw_index]
            return float(torch.linalg.norm(final - target_t[0]))

        def block_pos() -> list[float]:
            return [float(v) for v in block.data.root_pos_w.torch[0].detach().to("cpu", torch.float64)]

        # Home and open the gripper; build a grasp frame from the jaw/palm USD
        # bounding boxes instead of the jaw body origin.
        import omni.usd
        from pxr import Usd, UsdGeom, UsdPhysics

        set_gripper(1.0, 80)
        stage = omni.usd.get_context().get_stage()
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

        def world_center(name: str):
            for prim in stage.Traverse():
                if (
                    prim.GetName() == name
                    and "/Robot/" in str(prim.GetPath())
                    and prim.HasAPI(UsdPhysics.RigidBodyAPI)
                ):
                    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                    return (box.GetMin() + box.GetMax()) * 0.5
            raise RuntimeError(f"robot rigid body not found: {name}")

        jaw_center = torch.tensor([float(v) for v in world_center("moving_jaw_so101_v1")], device=robot.device)
        palm_center = torch.tensor([float(v) for v in world_center("gripper")], device=robot.device)
        grasp_world = 0.5 * (jaw_center + palm_center)
        jaw_pos = robot.data.body_pos_w.torch[0, jaw_index]
        quat = robot.data.body_quat_w.torch[0, jaw_index]
        x, y, z, w = [float(v) for v in quat.detach().to("cpu", torch.float64)]
        rotation = torch.tensor(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=torch.float32,
            device=robot.device,
        )
        local_offset = rotation.T @ (grasp_world - jaw_pos)
        block_before = block_pos()
        def quat_to_R(quat: torch.Tensor) -> torch.Tensor:
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

        def ik_to_grasp(grasp_point: list[float], gripper_value: float, iterations: int = 400) -> float:
            point = torch.tensor([grasp_point], dtype=torch.float32, device=robot.device)
            for _ in range(iterations):
                q = robot.data.joint_pos.torch.clone()
                pos = robot.data.body_pos_w.torch[0, jaw_index].unsqueeze(0)
                quat = robot.data.body_quat_w.torch[0, jaw_index].unsqueeze(0)
                jaw_target = point - quat_to_R(quat[0]) @ local_offset
                ik.set_command(jaw_target.unsqueeze(0), pos, quat)
                jacobian = robot.data.body_link_jacobian_w.torch[0, jacobi_index].unsqueeze(0)
                q_des = ik.compute(pos, quat, jacobian, q)
                q_des = torch.max(torch.min(q_des, high.unsqueeze(0)), low.unsqueeze(0))
                q_des[0, gripper_index] = gripper_value
                robot.set_joint_position_target_index(target=q_des, full_data=True)
                step(1)
            final_jaw = robot.data.body_pos_w.torch[0, jaw_index]
            final_point = final_jaw + quat_to_R(robot.data.body_quat_w.torch[0, jaw_index]) @ local_offset
            return float(torch.linalg.norm(final_point - point[0]))

        block_center = [0.18, 0.0, 0.075]
        grasp_target = block_center
        errors = {}
        errors["grasp"] = ik_to_grasp(block_center, 1.7, 450)
        set_gripper(0.3, 200)
        block_closed = block_pos()
        errors["lift"] = ik_to_grasp([0.18, 0.0, 0.15], 0.3, 260)
        block_lifted = block_pos()
        errors["over_box"] = ik_to_grasp([0.20, 0.10, 0.14], 0.3, 250)
        block_transferred = block_pos()
        errors["place"] = ik_to_grasp([0.20, 0.10, 0.07], 0.3, 200)
        set_gripper(1.0, 120)
        step(80)
        block_placed = block_pos()
        errors["retreat"] = ik_to_grasp([0.20, 0.10, 0.18], 1.0, 150)
        block_after = block_pos()

        delta = math.sqrt(sum((a - b) ** 2 for a, b in zip(block_after, block_before)))
        in_region = (
            0.15 <= block_after[0] <= 0.25
            and 0.05 <= block_after[1] <= 0.15
            and block_after[2] <= 0.08
        )
        checks = {
            "lifted": block_lifted[2] > block_before[2] + 0.03,
            "moved": delta > 0.05,
            "in_target_region": in_region,
            "ik_errors_below_3cm": all(value < 0.03 for value in errors.values()),
        }
        result = {
            "status": "passed" if all(checks.values()) else "failed",
            "checks": checks,
            "grasp_world_open": [float(v) for v in grasp_world.detach().to("cpu", torch.float64)],
            "local_offset": [float(v) for v in local_offset.detach().to("cpu", torch.float64)],
            "grasp_target": grasp_target,
            "ik_errors_m": errors,
            "block_before": block_before,
            "block_after_close": block_closed,
            "block_lifted": block_lifted,
            "block_transferred": block_transferred,
            "block_placed": block_placed,
            "block_after": block_after,
            "block_delta_m": delta,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps({"status": result["status"], "checks": checks, "block_after": block_after}))
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
