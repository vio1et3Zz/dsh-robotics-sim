"""Probe SO-ARM101 differential IK reachability for candidate pick targets."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from isaaclab.app import AppLauncher

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=300)
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
        jacobi_index = jaw_index - 1  # fixed base: jacobi row excludes the root link
        limits = robot.data.joint_pos_limits.torch[0].detach()
        low = limits[:, 0]
        high = limits[:, 1]
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

        def home() -> None:
            target = torch.zeros((1, len(body_names)), device=robot.device)  # unused, replaced below
            q = torch.zeros((1, 6), device=robot.device)
            robot.set_joint_position_target_index(target=q, full_data=True)
            step(120)

        def body_pose() -> tuple[torch.Tensor, torch.Tensor]:
            pos = robot.data.body_pos_w.torch[0, jaw_index].unsqueeze(0)
            quat = robot.data.body_quat_w.torch[0, jaw_index].unsqueeze(0)
            return pos, quat

        results = []
        for target in ([0.18, 0.0, 0.10], [0.10, -0.20, 0.10], [0.05, -0.30, 0.10]):
            home()
            target_t = torch.tensor([target], dtype=torch.float32, device=robot.device)
            start_pos, _ = body_pose()
            for iteration in range(args.steps):
                q = robot.data.joint_pos.torch.clone()
                pos, quat = body_pose()
                ik.set_command(target_t, pos, quat)
                jacobian = robot.data.body_link_jacobian_w.torch[0, jacobi_index].unsqueeze(0)
                q_des = ik.compute(pos, quat, jacobian, q)
                q_des = torch.max(torch.min(q_des, high.unsqueeze(0)), low.unsqueeze(0))
                robot.set_joint_position_target_index(target=q_des, full_data=True)
                step(2)
            final_pos, _ = body_pose()
            error = float(torch.linalg.norm(final_pos - target_t))
            results.append(
                {
                    "target": target,
                    "start": [float(v) for v in start_pos[0].detach().to("cpu", torch.float64)],
                    "final": [float(v) for v in final_pos[0].detach().to("cpu", torch.float64)],
                    "error_m": error,
                    "joint_positions": [float(v) for v in robot.data.joint_pos.torch[0].detach().to("cpu", torch.float64)],
                }
            )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({"results": results}, indent=2), encoding="utf-8")
        print(json.dumps({"errors": [round(item["error_m"], 4) for item in results]}))
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
