"""SO-101 palm-frame pick/place probe on a thin pedestal."""

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
        import omni.usd
        from pxr import Usd, UsdGeom, UsdPhysics
        from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
        from robotics_backends.isaaclab_physx.so101_objects_scene import build_so101_object_scene

        dt = 0.005
        sim, scene, robot, block = build_so101_object_scene(device=getattr(app, "device", "cuda:0"), dt=dt)
        body_names = list(robot.body_names)
        palm_index = body_names.index("gripper")
        jacobi_index = palm_index - 1
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

        def set_gripper(value: float, count: int = 120) -> None:
            q = robot.data.joint_pos.torch.clone()
            q[0, gripper_index] = value
            robot.set_joint_position_target_index(target=q, full_data=True)
            step(count)

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

        def block_pos() -> list[float]:
            return [float(v) for v in block.data.root_pos_w.torch[0].detach().to("cpu", torch.float64)]

        # Home, open gripper, then derive the palm-local grasp point from the
        # rest-stage palm AABB and the current palm body pose.
        set_gripper(1.7, 150)
        stage = omni.usd.get_context().get_stage()
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

        def palm_prim():
            for candidate in stage.Traverse():
                if (
                    candidate.GetName() == "gripper"
                    and "/Robot/" in str(candidate.GetPath())
                    and candidate.HasAPI(UsdPhysics.RigidBodyAPI)
                ):
                    return candidate
            raise RuntimeError("palm link not found")

        aligned = cache.ComputeWorldBound(palm_prim()).ComputeAlignedRange()
        palm_min = torch.tensor([float(v) for v in aligned.GetMin()], device=robot.device)
        palm_max = torch.tensor([float(v) for v in aligned.GetMax()], device=robot.device)
        palm_pos = robot.data.body_pos_w.torch[0, palm_index]
        palm_quat = robot.data.body_quat_w.torch[0, palm_index]
        grasp_rest = torch.stack(
            (
                0.5 * (palm_min[0] + palm_max[0]),
                0.5 * (palm_min[1] + palm_max[1]),
                palm_max[2] + 0.015,
            )
        )
        local_grasp = quat_R(palm_quat).T @ (grasp_rest - palm_pos)

        def ik_to_grasp(grasp_point: list[float], gripper_value: float, iterations: int = 450) -> float:
            point = torch.tensor([grasp_point], dtype=torch.float32, device=robot.device)
            for _ in range(iterations):
                q = robot.data.joint_pos.torch.clone()
                pos = robot.data.body_pos_w.torch[0, palm_index].unsqueeze(0)
                quat = robot.data.body_quat_w.torch[0, palm_index].unsqueeze(0)
                target = point - quat_R(quat[0]) @ local_grasp
                ik.set_command(target.unsqueeze(0), pos, quat)
                jacobian = robot.data.body_link_jacobian_w.torch[0, jacobi_index].unsqueeze(0)
                q_des = ik.compute(pos, quat, jacobian, q)
                q_des = torch.max(torch.min(q_des, high.unsqueeze(0)), low.unsqueeze(0))
                q_des[0, gripper_index] = gripper_value
                robot.set_joint_position_target_index(target=q_des, full_data=True)
                step(1)
            final = robot.data.body_pos_w.torch[0, palm_index]
            final_point = final + quat_R(robot.data.body_quat_w.torch[0, palm_index]) @ local_grasp
            return float(torch.linalg.norm(final_point - point[0]))

        before = block_pos()
        errors = {}
        errors["grasp"] = ik_to_grasp([0.18, 0.0, 0.075], 1.7)
        set_gripper(0.0, 220)
        after_close = block_pos()
        errors["lift"] = ik_to_grasp([0.18, 0.0, 0.20], 0.0, 220)
        after_lift = block_pos()
        errors["over_box"] = ik_to_grasp([0.20, 0.10, 0.16], 0.0, 250)
        errors["place"] = ik_to_grasp([0.20, 0.10, 0.06], 0.0, 220)
        set_gripper(1.7, 150)
        step(100)
        after_release = block_pos()
        errors["retreat"] = ik_to_grasp([0.20, 0.10, 0.16], 1.7, 150)
        after = block_pos()

        delta = math.sqrt(sum((a - b) ** 2 for a, b in zip(after, before)))
        in_region = 0.15 <= after[0] <= 0.25 and 0.05 <= after[1] <= 0.15 and after[2] <= 0.09
        checks = {
            "lifted": after_lift[2] > before[2] + 0.03,
            "moved": delta > 0.05,
            "in_target_region": in_region,
            "ik_errors_below_3cm": all(value < 0.03 for value in errors.values()),
        }
        result = {
            "status": "passed" if all(checks.values()) else "failed",
            "checks": checks,
            "local_grasp": [float(v) for v in local_grasp.detach().to("cpu", torch.float64)],
            "ik_errors_m": errors,
            "block_before": before,
            "block_after_close": after_close,
            "block_after_lift": after_lift,
            "block_after_release": after_release,
            "block_after": after,
            "block_delta_m": delta,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps({"status": result["status"], "checks": checks, "after": after}))
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
