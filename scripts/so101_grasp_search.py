"""Search a small grasp-pose grid for SO-ARM101 on the pedestal block."""

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
        default_block_state = block.data.default_root_state.torch.clone()
        default_joint_pos = robot.data.default_joint_pos.torch.clone()
        default_joint_vel = torch.zeros_like(default_joint_pos)

        def step(count: int) -> None:
            for _ in range(count):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(dt)

        def reset() -> None:
            robot.write_joint_state_to_sim(default_joint_pos, default_joint_vel)
            block.write_root_pose_to_sim(default_block_state[:, :7])
            block.write_root_velocity_to_sim(torch.zeros((1, 6), device=robot.device))
            step(5)

        def set_gripper(value: float, count: int = 80) -> None:
            q = robot.data.joint_pos.torch.clone()
            q[0, gripper_index] = value
            robot.set_joint_position_target_index(target=q, full_data=True)
            step(count)

        def ik_to(target: list[float], gripper_value: float, iterations: int = 320) -> float:
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
                step(1)
            final = robot.data.body_pos_w.torch[0, jaw_index]
            return float(torch.linalg.norm(final - target_t[0]))

        def block_z() -> float:
            return float(block.data.root_pos_w.torch[0, 2])

        candidates = [
            (-0.03, 0.0, 0.0, 0.2),
            (-0.03, 0.0, 0.0, 0.5),
            (-0.05, 0.0, 0.0, 0.3),
            (-0.03, 0.0, 0.02, 0.3),
            (-0.05, 0.0, 0.02, 0.3),
            (-0.03, 0.02, 0.0, 0.3),
            (-0.03, -0.02, 0.0, 0.3),
            (-0.02, 0.0, -0.01, 0.3),
            (0.0, 0.0, 0.0, 1.0),
        ]
        results = []
        for dx, dy, dz, close_angle in candidates:
            reset()
            set_gripper(1.0, 60)
            target = [0.18 + dx, dy, 0.075 + dz]
            error = ik_to(target, 1.0)
            set_gripper(close_angle, 100)
            lift_error = ik_to([0.18, 0.0, 0.20], close_angle, 220)
            lifted_z = block_z()
            results.append(
                {
                    "offset": [dx, dy, dz],
                    "close_angle": close_angle,
                    "target": target,
                    "ik_error": error,
                    "lift_ik_error": lift_error,
                    "block_z_after_lift": lifted_z,
                    "lifted": lifted_z > 0.11,
                }
            )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({"results": results}, indent=2), encoding="utf-8")
        print(json.dumps([{"offset": r["offset"], "close": r["close_angle"], "z": round(r["block_z_after_lift"], 4), "lifted": r["lifted"]} for r in results]))
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
