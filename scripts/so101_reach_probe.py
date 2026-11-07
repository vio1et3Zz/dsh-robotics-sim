"""Probe reachable jaw targets for SO-ARM101 before choosing the pick pose."""

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

        def step(count: int) -> None:
            for _ in range(count):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(dt)

        def reset_home() -> None:
            q = torch.zeros((1, 6), device=robot.device)
            q[0, gripper_index] = 1.0
            robot.set_joint_position_target_index(target=q, full_data=True)
            step(120)

        def ik_to(target: list[float], iterations: int = 400) -> dict:
            target_t = torch.tensor([target], dtype=torch.float32, device=robot.device)
            for _ in range(iterations):
                q = robot.data.joint_pos.torch.clone()
                pos = robot.data.body_pos_w.torch[0, jaw_index].unsqueeze(0)
                quat = robot.data.body_quat_w.torch[0, jaw_index].unsqueeze(0)
                ik.set_command(target_t, pos, quat)
                jacobian = robot.data.body_link_jacobian_w.torch[0, jacobi_index].unsqueeze(0)
                q_des = ik.compute(pos, quat, jacobian, q)
                q_des = torch.max(torch.min(q_des, high.unsqueeze(0)), low.unsqueeze(0))
                q_des[0, gripper_index] = 1.0
                robot.set_joint_position_target_index(target=q_des, full_data=True)
                step(1)
            final = robot.data.body_pos_w.torch[0, jaw_index]
            return {
                "target": target,
                "final": [float(v) for v in final.detach().to("cpu", torch.float64)],
                "error_m": float(torch.linalg.norm(final - target_t[0])),
                "joint_positions": [float(v) for v in robot.data.joint_pos.torch[0].detach().to("cpu", torch.float64)],
            }

        targets = [
            [0.18, 0.0, 0.10],
            [0.18, 0.0, 0.08],
            [0.18, 0.0, 0.06],
            [0.14, -0.08, 0.06],
            [0.10, -0.12, 0.05],
            [0.08, -0.18, 0.06],
            [0.12, 0.0, 0.05],
        ]
        results = []
        for target in targets:
            reset_home()
            results.append(ik_to(target))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({"results": results}, indent=2), encoding="utf-8")
        print(json.dumps([{ "target": r["target"], "error": round(r["error_m"], 4)} for r in results]))
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
