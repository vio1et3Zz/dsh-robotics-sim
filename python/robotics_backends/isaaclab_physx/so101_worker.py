"""SO-ARM101 Worker: world state, robot metadata and the pick/place skill.

This worker owns one object scene and exposes the measured block/target-box
state plus the T06 pick/place skill.  ``move_object`` is a developer-only fault
injection used by the T07 recovery tests; it is listed in the capabilities so
the Agent can see it, and it reports the before/after pose.
"""

from __future__ import annotations

import json
import math
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from robotics_protocol import (
    ControlMode,
    EndEffectorSpec,
    Frame,
    GripperSpec,
    GripperState,
    JointSpec,
    JointType,
    ObjectState,
    ObservationSource,
    Pose,
    RobotAsset,
    RobotSpec,
    RobotState,
    SkillPhase,
    SkillStatus,
    WorldState,
)

from .so101_pick_place import GRASP_LOCAL, SO101PickPlace, SkillCanceled as PickSkillCanceled
from .so101_scene import SO101_USD
from .worker import SkillCanceled, SnapshotStore, WorkerMonitor

TARGET_BOX_CENTER = (0.22, 0.05, 0.02)
TARGET_BOX_SIZE = (0.10, 0.10, 0.04)
BLOCK_SIZE = (0.03, 0.03, 0.03)
ROBOT_ID = "so101"
SCENE_ID = "table_pick_place"
SKILLS = ["observe_scene", "pick", "move_object", "hold_pose", "vla_stub"]


class SO101Worker:
    """One SO-ARM101 object scene; all methods run on the simulation thread."""

    def __init__(
        self,
        sim: Any,
        scene: Any,
        robot: Any,
        block: Any,
        *,
        dt: float = 0.005,
        camera: Any = None,
        output_dir: Any = None,
    ) -> None:
        import torch

        self._torch = torch
        self._sim = sim
        self._scene = scene
        self._robot = robot
        self._block = block
        self._dt = float(dt)
        self._camera = camera
        self._frame_dir = None if output_dir is None else Path(output_dir) / "frames"
        self._closed = False
        self._simulation_id = f"sim_{uuid.uuid4().hex}"
        self._episode_id = f"ep_{uuid.uuid4().hex}"
        self._state_version = 0
        self._event_seq = 0
        self._sim_step = 0
        self._sim_time_s = 0.0
        self._last_skill: SkillStatus | None = None
        self._joint_names = list(robot.joint_names)
        self._body_names = list(robot.body_names)
        self._palm_index = self._body_names.index("gripper")
        self._gripper_index = self._joint_names.index("gripper")
        limits = robot.data.joint_pos_limits.torch[0].detach().to("cpu", torch.float64).numpy()
        self._limits = [(float(pair[0]), float(pair[1])) for pair in limits]
        velocities = robot.data.joint_vel_limits.torch[0].detach().to("cpu", torch.float64).numpy()
        self._velocity_limits = [float(v) for v in velocities]
        self._skill = SO101PickPlace(sim, scene, robot, block, None, dt=dt)
        self._scene.update(self._dt)
        # Explicit initial state: Isaac Lab's RigidObject.reset only clears
        # wrenches, so the worker restores the measured spawn pose itself.
        self._initial_block_pose = torch.cat(
            [block.data.root_pos_w.torch.clone(), block.data.root_quat_w.torch.clone()], dim=1
        ).clone()
        self._initial_joint_pos = robot.data.default_joint_pos.torch.clone()
        self._snapshots = SnapshotStore()
        self._monitor = WorkerMonitor(self)

    # -- lifecycle -----------------------------------------------------

    @property
    def monitor(self) -> WorkerMonitor:
        return self._monitor

    @property
    def snapshots(self) -> SnapshotStore:
        return self._snapshots

    def publish_snapshot(self) -> None:
        """Build a fresh ``WorldState`` and publish it for the command loop."""
        self._snapshots.publish_state(self.observe())

    def observe_snapshot(self) -> WorldState | None:
        """Latest published state; safe to call while the skill thread steps."""
        return self._snapshots.state()

    @property
    def simulation_id(self) -> str:
        return self._simulation_id

    @property
    def episode_id(self) -> str:
        return self._episode_id

    def observe(self) -> WorldState:
        self._require_open()
        joints = self._joints()
        velocities = [
            float(v) for v in self._robot.data.joint_vel.torch[0].detach().to("cpu", self._torch.float64)
        ]
        mouth = self._mouth()
        palm_quat = [
            float(v)
            for v in self._robot.data.body_quat_w.torch[0, self._palm_index]
            .detach()
            .to("cpu", self._torch.float64)
        ]
        lower, upper = self._limits[self._gripper_index]
        jaw = joints[self._gripper_index]
        width = (jaw - lower) / (upper - lower) if upper > lower else 0.0
        robot_state = RobotState(
            robot_id=ROBOT_ID,
            dof=len(joints),
            joint_positions=joints,
            joint_velocities=velocities,
            ee_pose=Pose(frame=Frame.WORLD, position=tuple(mouth), orientation_xyzw=tuple(palm_quat)),
            gripper=GripperState(width_m=min(max(width, 0.0), 1.0), closed=jaw < 0.05),
        )
        block_pos = self._block_pos()
        block_quat = [
            float(v)
            for v in self._block.data.root_quat_w.torch[0].detach().to("cpu", self._torch.float64)
        ]
        objects = [
            ObjectState(
                object_id="red_block",
                pose=Pose(frame=Frame.WORLD, position=tuple(block_pos), orientation_xyzw=tuple(block_quat)),
                size_m=BLOCK_SIZE,
            ),
            ObjectState(
                object_id="target_box",
                pose=Pose(
                    frame=Frame.WORLD,
                    position=TARGET_BOX_CENTER,
                    orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
                ),
                size_m=TARGET_BOX_SIZE,
            ),
        ]
        return WorldState(
            simulation_id=self._simulation_id,
            episode_id=self._episode_id,
            env_id=0,
            state_version=self._state_version,
            event_seq=self._event_seq,
            sim_step=self._sim_step,
            sim_time_s=self._sim_time_s,
            wall_time=datetime.now(timezone.utc),
            source=ObservationSource.SIM_GROUND_TRUTH,
            frame=Frame.WORLD,
            robots=[robot_state],
            objects=objects,
            skills=[self._last_skill] if self._last_skill is not None else [],
        )

    def capture_frame(self, camera_id: str = "front", *, fresh: bool = True) -> dict[str, Any]:
        """Return one frame's metadata and persist its PNG/NPY files.

        ``fresh=False`` serves the newest monitor-buffered frame (rendered on
        the simulation thread at 1 Hz while a skill runs); it can only be used
        while the skill thread owns the simulation.  ``fresh=True`` renders
        here, which is only safe from the command loop when no skill runs.
        """
        self._require_open()
        buffered = False
        slot = None if fresh else self._snapshots.latest_frame()
        if slot is not None:
            buffered = True
        else:
            if not fresh:
                raise RuntimeError("FRAME_NOT_READY: no buffered frame yet")
            slot = self._render_frame(camera_id)
            self._snapshots.add_frame(slot)
        metadata = self._persist_frame(slot)
        metadata["buffered"] = buffered
        return metadata

    def publish_frame(self) -> None:
        """Monitor hook: render one frame on the simulation thread into the buffer."""
        if self._camera is None or self._frame_dir is None:
            return
        self._snapshots.add_frame(self._render_frame())

    def _render_frame(self, camera_id: str = "front") -> dict[str, Any]:
        self._require_open()
        if self._camera is None:
            raise RuntimeError("CAMERA_DISABLED: this worker was built without a camera")
        if self._frame_dir is None:
            raise RuntimeError("FRAME_STORAGE_MISSING: worker has no output directory")
        import numpy as np

        start = time.perf_counter()
        self._camera.update(self._dt)
        self._torch.cuda.synchronize()
        capture_s = time.perf_counter() - start
        output = self._camera.data.output or {}
        if "rgb" not in output or "distance_to_image_plane" not in output:
            raise RuntimeError(f"camera output missing rgb/depth: {sorted(output)}")
        rgb = output["rgb"].torch[0]
        depth = output["distance_to_image_plane"].torch[0]
        intrinsic = self._camera.data.intrinsic_matrices.torch[0].detach().to("cpu").to(self._torch.float64).numpy()
        position = self._camera.data.pos_w.torch[0].detach().to("cpu").to(self._torch.float64).numpy()
        quat = self._camera.data.quat_w_world.torch[0].detach().to("cpu").to(self._torch.float64).numpy()

        if rgb.dtype.is_floating_point:
            rgb_u8 = (rgb.detach().to("cpu").clamp(0.0, 1.0) * 255.0).to(self._torch.uint8).numpy()
        else:
            rgb_u8 = rgb.detach().to("cpu").to(self._torch.uint8).numpy()
        depth_np = depth.detach().to("cpu").to(self._torch.float32).numpy()
        self._event_seq += 1
        height, width = int(depth_np.shape[0]), int(depth_np.shape[1])
        finite_depth = depth_np[np.isfinite(depth_np)]
        invalid_fraction = float(1.0 - finite_depth.size / max(depth_np.size, 1))
        frame_id = f"frame_{uuid.uuid4().hex}"
        metadata = {
            "frame_id": frame_id,
            "camera_id": camera_id,
            "simulation_id": self._simulation_id,
            "episode_id": self._episode_id,
            "state_version": self._state_version,
            "sim_step": self._sim_step,
            "sim_time_s": self._sim_time_s,
            "captured_at": datetime.now(timezone.utc).isoformat(),
            "width": width,
            "height": height,
            "rgb_format": "png",
            "depth_format": "npy",
            "depth_units": "m",
            "depth_type": "distance_to_image_plane",
            "intrinsics": {
                "fx": float(intrinsic[0, 0]),
                "fy": float(intrinsic[1, 1]),
                "cx": float(intrinsic[0, 2]),
                "cy": float(intrinsic[1, 2]),
                "matrix": [[float(v) for v in row] for row in intrinsic],
            },
            "extrinsics": {
                "frame": "world",
                "convention": "world",
                "position": [float(v) for v in position],
                "orientation_xyzw": [float(v) for v in quat],
            },
            "rgb_range": [float(rgb_u8.min()), float(rgb_u8.max())],
            "depth_range_m": [
                float(finite_depth.min()) if finite_depth.size else 0.0,
                float(finite_depth.max()) if finite_depth.size else 0.0,
            ],
            "depth_invalid_fraction": invalid_fraction,
            "capture_ms": capture_s * 1000.0,
            "buffered": False,
            "ground_truth_scene": True,
        }
        return {"frame_id": frame_id, "metadata": metadata, "rgb_u8": rgb_u8, "depth": depth_np, "persisted": False}

    def _persist_frame(self, slot: dict[str, Any]) -> dict[str, Any]:
        """Write PNG/NPY once per rendered frame, then return a metadata copy."""
        if not slot["persisted"]:
            import numpy as np
            from PIL import Image

            self._frame_dir.mkdir(parents=True, exist_ok=True)
            rgb_path = self._frame_dir / f"{slot['frame_id']}.png"
            depth_path = self._frame_dir / f"{slot['frame_id']}.npy"
            Image.fromarray(slot["rgb_u8"]).save(rgb_path)
            np.save(depth_path, slot["depth"])
            slot["metadata"]["rgb_path"] = str(rgb_path)
            slot["metadata"]["depth_path"] = str(depth_path)
            slot["persisted"] = True
        return dict(slot["metadata"])

    def reset(self) -> WorldState:
        self._require_open()
        # ``RigidObject.reset`` does not restore root poses and ``sim.reset()``
        # alone did not restore the object in this build; restore the measured
        # spawn state explicitly, then settle.
        self._sim.reset()
        self._scene.reset()
        self._robot.write_joint_state_to_sim(
            self._initial_joint_pos, self._torch.zeros_like(self._initial_joint_pos)
        )
        self._robot.set_joint_position_target_index(target=self._initial_joint_pos, full_data=True)
        self._block.write_root_pose_to_sim(self._initial_block_pose)
        self._block.write_root_velocity_to_sim(
            self._torch.zeros((1, 6), dtype=self._torch.float32, device=self._robot.device)
        )
        for _ in range(10):
            self._step_once()
        self._scene.update(self._dt)
        self._episode_id = f"ep_{uuid.uuid4().hex}"
        self._state_version += 1
        self._event_seq += 1
        self._sim_step = 0
        self._sim_time_s = 0.0
        self._last_skill = None
        self._snapshots.clear_frames()
        state = self.observe()
        self._snapshots.publish_state(state)
        return state

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._sim.stop()
        except Exception:
            pass

    # -- metadata ------------------------------------------------------

    def robot_spec(self) -> RobotSpec:
        joints = []
        for index, name in enumerate(self._joint_names):
            lower, upper = self._limits[index]
            velocity = self._velocity_limits[index]
            joints.append(
                JointSpec(
                    name=name,
                    logical_name=name,
                    type=JointType.REVOLUTE,
                    unit="rad",
                    lower=lower,
                    upper=upper,
                    home=min(max(0.0, lower), upper),
                    velocity_limit=velocity if math.isfinite(velocity) and velocity > 0.0 else 1.0,
                )
            )
        lower, upper = self._limits[self._gripper_index]
        return RobotSpec(
            robot_id=ROBOT_ID,
            model="SO-ARM101 (RobotStudio so101_new_calib)",
            dof=len(self._joint_names),
            joints=joints,
            end_effector=EndEffectorSpec(name="gripper", parent_joint="wrist_roll"),
            gripper=GripperSpec(
                kind="single",
                joint_names=["gripper"],
                open_value=upper,
                close_value=lower,
                max_width_m=0.04,
            ),
            control_modes=[ControlMode.POSITION],
            skills=SKILLS,
            asset=RobotAsset(
                usd_path=str(SO101_USD),
                license="Apache-2.0 (RobotStudio SO-ARM101)",
                source="assets/isaac6/RobotStudio/so101_new_calib",
            ),
        )

    # -- skills --------------------------------------------------------

    def run_skill(
        self,
        skill: str,
        parameters: dict[str, Any],
        *,
        cancel_check: Any = None,
        progress_cb: Any = None,
        step_cb: Any = None,
        phase_cb: Any = None,
    ) -> dict[str, Any]:
        self._require_open()
        if skill == "observe_scene":
            return {"passed": True, "state": self.observe().model_dump(mode="json")}
        if skill == "hold_pose":
            return self._hold_pose(parameters, cancel_check, progress_cb, step_cb)
        if skill == "move_object":
            return self._move_object(parameters, step_cb)
        if skill == "pick":
            return self._pick(parameters, cancel_check, progress_cb, step_cb)
        if skill == "vla_stub":
            return self._policy(parameters, cancel_check, progress_cb, step_cb, phase_cb)
        raise ValueError(f"worker does not implement {skill}")

    def _pick(
        self, parameters: dict[str, Any], cancel_check: Any, progress_cb: Any, step_cb: Any = None
    ) -> dict[str, Any]:
        place_x = float(parameters.get("place_x", TARGET_BOX_CENTER[0]))
        place_y = float(parameters.get("place_y", TARGET_BOX_CENTER[1]))
        grasp_x = parameters.get("grasp_x")
        self._skill._steps = 0
        base_step = self._sim_step
        base_time = self._sim_time_s

        def track_steps() -> None:
            # Pick keeps its own step counter; mirror it into the worker so the
            # snapshots and frames carry a rising sim_step during the skill.
            steps = self._skill.step_count
            self._sim_step = base_step + steps
            self._sim_time_s = base_time + steps * self._dt
            if step_cb is not None:
                step_cb()

        self._last_skill = SkillStatus(skill="pick", phase=SkillPhase.RUNNING, progress=0.0)
        self._event_seq += 1
        try:
            result = self._skill.execute(
                (place_x, place_y),
                grasp_x=float(grasp_x) if grasp_x is not None else None,
                cancel_check=cancel_check,
                progress_cb=progress_cb,
                step_cb=track_steps,
            )
        except PickSkillCanceled:
            self._last_skill = SkillStatus(
                skill="pick", phase=SkillPhase.CANCELED, progress=0.0, message="cancel acknowledged"
            )
            self._state_version += 1
            self._event_seq += 1
            raise SkillCanceled("cancel acknowledged")
        finally:
            steps = self._skill.step_count
            self._sim_step = base_step + steps
            self._sim_time_s = base_time + steps * self._dt
        data = result.to_dict()
        data["sim_steps"] = self._skill.step_count
        data["sim_time_s"] = self._skill.step_count * self._dt
        self._state_version += 1
        self._event_seq += 1
        self._last_skill = SkillStatus(
            skill="pick",
            phase=SkillPhase.SUCCEEDED if result.passed else SkillPhase.FAILED,
            progress=1.0,
            message=result.failure,
        )
        return data

    # -- policy stub (C1) ----------------------------------------------

    def _policy_waypoints(self, goal: str) -> list[list[float]]:
        from .so101_pick_place import GRASP_VARIANTS

        arm = GRASP_VARIANTS[0.18]

        def waypoint(name: str, jaw: float) -> list[float]:
            return [math.radians(value) for value in arm[name]] + [jaw]

        # Durations mirror the tuned pick skill (260/140/280/130/130 steps) and
        # add the 60-step settle holds it uses before grasping and closing.
        approach = waypoint("approach", 0.4)
        grasp = waypoint("grasp", 0.4)
        grasp_closed = waypoint("grasp", 0.0)
        if goal == "approach":
            return [
                {"joints": approach, "steps": 260},
                {"joints": approach, "steps": 60},
                {"joints": grasp, "steps": 140},
                {"joints": grasp, "steps": 60},
            ]
        if goal == "lift":
            return [
                {"joints": approach, "steps": 260},
                {"joints": approach, "steps": 60},
                {"joints": grasp, "steps": 140},
                {"joints": grasp, "steps": 60},
                {"joints": grasp_closed, "steps": 280},
                {"joints": waypoint("lift1", 0.0), "steps": 130},
                {"joints": waypoint("lift2", 0.0), "steps": 130},
            ]
        raise ValueError(f"unknown policy goal {goal!r} (use 'approach' or 'lift')")

    def _policy(
        self, parameters: dict[str, Any], cancel_check: Any, progress_cb: Any, step_cb: Any, phase_cb: Any
    ) -> dict[str, Any]:
        from .stub_policy import PolicyFailure, StubDeltaPolicy, StubWaypointPolicy

        mode = str(parameters.get("mode", "joint"))
        goal = str(parameters.get("goal", "lift"))
        fail = parameters.get("fail")
        chunk_steps = max(1, min(int(parameters.get("chunk_steps", 10)), 50))
        steps_per_waypoint = max(chunk_steps, int(parameters.get("steps_per_waypoint", 60)))
        max_seconds = float(parameters.get("max_seconds", 30.0))
        patience_chunks = max(1, int(parameters.get("patience_chunks", 20)))
        min_progress_m = float(parameters.get("min_progress_m", 0.002))
        min_joint_delta = float(parameters.get("min_joint_delta", 0.01))
        if mode == "ee_delta":
            deltas = [[0.0, 0.0, -0.004]] * 5 + [[0.0, 0.0, 0.004]] * 5
            policy: Any = StubDeltaPolicy(deltas, repeat=int(parameters.get("repeat", 2)), fail=str(fail) if fail else None)
        else:
            policy = StubWaypointPolicy(
                self._policy_waypoints(goal),
                chunk_steps=chunk_steps,
                steps_per_waypoint=steps_per_waypoint,
                fail=str(fail) if fail else None,
            )
        block_start = self._block_pos()
        policy.reset({"block": block_start})
        base_step = self._sim_step
        self._sim_step = base_step
        self._last_skill = SkillStatus(skill="vla_stub", phase=SkillPhase.RUNNING, progress=0.0)
        self._event_seq += 1
        started = time.time()
        chunks = 0
        stagnant = 0
        last_joints = self._joints()
        last_block = self._block_pos()
        log = self._open_policy_log()
        if phase_cb is not None:
            phase_cb("policy", chunk=0, step=self._sim_step, progress=0.0)
        try:
            while True:
                if cancel_check is not None and cancel_check():
                    raise SkillCanceled("cancel acknowledged")
                if time.time() - started > max_seconds:
                    raise PolicyFailure(
                        "POLICY_TIMEOUT",
                        f"policy exceeded {max_seconds:.1f}s",
                        {"chunks": chunks, "sim_step": self._sim_step},
                    )
                act_started = time.perf_counter()
                observation = self._policy_observation()
                action = policy.act(observation)
                act_ms = (time.perf_counter() - act_started) * 1000.0
                if action.get("done"):
                    break
                action_mode = str(action.get("mode", "joint"))
                if action_mode == "ee_delta":
                    targets = self._ee_delta_targets(action.get("deltas") or [])
                else:
                    targets = action.get("targets") or []
                exec_started = time.perf_counter()
                for target in targets:
                    self._apply_joint_target(target)
                    self._step_once()
                    if step_cb is not None:
                        step_cb()
                exec_ms = (time.perf_counter() - exec_started) * 1000.0
                if phase_cb is not None:
                    phase_cb("execute", chunk=chunks, step=self._sim_step, progress=policy.progress())
                self._write_policy_log(
                    log,
                    {
                        "chunk": chunks,
                        "sim_step": self._sim_step,
                        "state_version": self._state_version,
                        "mode": action_mode,
                        "targets": targets,
                        "frame_id": observation.get("frame_id"),
                        "latency_ms": {"act": round(act_ms, 2), "exec": round(exec_ms, 2)},
                        "progress": round(policy.progress(), 4),
                    },
                )
                chunks += 1
                if progress_cb is not None:
                    progress_cb(policy.progress())
                joints_now = self._joints()
                block_now = self._block_pos()
                joint_moved = max(abs(a - b) for a, b in zip(joints_now, last_joints))
                block_moved = math.dist(block_now, last_block)
                # Jaw closing moves no EE, so progress is "any joint or the
                # block changed", not just the end-effector.
                if joint_moved < min_joint_delta and block_moved < min_progress_m:
                    stagnant += 1
                else:
                    stagnant = 0
                last_joints = joints_now
                last_block = block_now
                if stagnant >= patience_chunks:
                    raise PolicyFailure(
                        "NO_PROGRESS",
                        f"EE moved < {min_progress_m} m for {stagnant} chunks",
                        {"chunks": chunks, "sim_step": self._sim_step},
                    )
            if phase_cb is not None:
                phase_cb("verify", step=self._sim_step, progress=1.0)
            final_block = self._block_pos()
            lifted = bool(final_block[2] >= block_start[2] + 0.03)
            self._state_version += 1
            self._event_seq += 1
            self._last_skill = SkillStatus(
                skill="vla_stub",
                phase=SkillPhase.SUCCEEDED if lifted else SkillPhase.FAILED,
                progress=1.0,
                message=None if lifted else "EMPTY_GRASP",
            )
            return {
                "passed": lifted,
                "failure": None if lifted else "EMPTY_GRASP",
                "policy": policy.name,
                "chunks": chunks,
                "sim_steps": self._sim_step - base_step,
                "elapsed_s": round(time.time() - started, 2),
                "block_start": block_start,
                "block_final": final_block,
                "policy_log": None if log is None else log["path"],
            }
        except (PolicyFailure, SkillCanceled):
            self._state_version += 1
            self._event_seq += 1
            raise
        finally:
            self._close_policy_log(log)

    def _policy_observation(self) -> dict[str, Any]:
        frame = self._snapshots.latest_frame()
        joints = self._joints()
        lower, upper = self._limits[self._gripper_index]
        jaw = joints[self._gripper_index]
        width = (jaw - lower) / (upper - lower) if upper > lower else 0.0
        palm_quat = [
            float(value)
            for value in self._robot.data.body_quat_w.torch[0, self._palm_index]
            .detach()
            .to("cpu", self._torch.float64)
        ]
        return {
            "joints": joints,
            "ee": self._mouth(),
            "ee_quat": palm_quat,
            "gripper": {"joint": jaw, "width_m": max(0.0, min(1.0, width)), "closed": jaw < 0.05},
            "objects": {"red_block": self._block_pos(), "target_box": list(TARGET_BOX_CENTER)},
            "sim_step": self._sim_step,
            "state_version": self._state_version,
            "episode_id": self._episode_id,
            "rgb": frame["rgb_u8"] if frame else None,
            "frame_id": frame["frame_id"] if frame else None,
        }

    @staticmethod
    def _clamp_joint(value: float, lower: float, upper: float) -> float:
        margin = 0.01
        if upper - lower <= 2 * margin:
            return min(max(value, lower), upper)
        return min(max(value, lower + margin), upper - margin)

    def _apply_joint_target(self, target: list[float]) -> None:
        tensor = self._robot.data.joint_pos.torch.clone()
        for index, value in enumerate(list(target)[: tensor.shape[1]]):
            lower, upper = self._limits[index]
            tensor[0, index] = self._clamp_joint(float(value), lower, upper)
        self._robot.set_joint_position_target_index(target=tensor, full_data=True)

    def _ee_delta_targets(self, deltas: list[list[float]]) -> list[list[float]]:
        """Damped least squares: EE delta (m) -> joint targets for each delta."""
        from .stub_policy import PolicyFailure

        torch = self._torch
        try:
            jacobians = self._robot.root_physx_view.get_jacobians()
        except Exception as error:  # noqa: BLE001 - surfaced as a policy error code
            raise PolicyFailure("EE_DELTA_UNAVAILABLE", f"jacobian unavailable: {error}") from error
        if hasattr(jacobians, "detach"):
            jacobians = jacobians.detach().to("cpu", torch.float64)
        else:
            jacobians = torch.as_tensor(jacobians, dtype=torch.float64)
        if jacobians.dim() != 4:
            raise PolicyFailure(
                "EE_DELTA_UNAVAILABLE", f"unexpected jacobian rank {jacobians.dim()} shape {tuple(jacobians.shape)}"
            )
        current = self._joints()
        body = min(self._palm_index, jacobians.shape[1] - 1)
        block = jacobians[0, body]
        if block.shape[0] != 6 and block.shape[1] == 6:
            block = block.transpose(0, 1)
        linear = block[0:3, : len(current)]
        if linear.shape[0] != 3 or linear.shape[1] != len(current):
            raise PolicyFailure("EE_DELTA_UNAVAILABLE", f"unexpected linear jacobian shape {tuple(linear.shape)}")
        targets: list[list[float]] = []
        damping = 0.02
        for delta in deltas:
            # Isaac Lab may set the default torch device to cuda:0, so pin the
            # small DLS matrices to the (CPU) jacobian's device explicitly.
            goal = torch.tensor([float(value) for value in delta], dtype=linear.dtype, device=linear.device)
            jj = linear @ linear.transpose(0, 1) + torch.eye(
                3, dtype=linear.dtype, device=linear.device
            ) * (damping**2)
            dq = linear.transpose(0, 1) @ torch.linalg.solve(jj, goal)
            current = [
                value + max(-0.15, min(0.15, float(dq[index]))) for index, value in enumerate(current)
            ]
            targets.append(list(current))
        return targets

    def _open_policy_log(self) -> dict[str, Any] | None:
        if self._frame_dir is None:
            return None
        log_dir = self._frame_dir.parent / "policy_logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        path = log_dir / f"policy_{self._episode_id}.jsonl"
        return {
            "handle": path.open("w", encoding="utf-8"),
            "path": str(path),
            "base": path,
            "index": 0,
            "lines": 0,
            "bytes": 0,
        }

    def _write_policy_log(self, log: dict[str, Any] | None, record: dict[str, Any]) -> None:
        if log is None or log["lines"] >= 5000:
            return
        line = json.dumps(record, ensure_ascii=False) + "\n"
        size = len(line.encode("utf-8"))
        if log["bytes"] + size > 2 * 1024 * 1024:
            # Rotate once per 2 MB so a long policy run cannot fill the disk.
            log["handle"].close()
            log["index"] += 1
            path = log["base"].with_name(f"{log['base'].stem}_{log['index']}.jsonl")
            log["handle"] = path.open("w", encoding="utf-8")
            log["path"] = str(path)
            log["lines"] = 0
            log["bytes"] = 0
        log["handle"].write(line)
        log["lines"] += 1
        log["bytes"] += size

    @staticmethod
    def _close_policy_log(log: dict[str, Any] | None) -> None:
        if log is not None:
            try:
                log["handle"].close()
            except Exception:
                pass

    def _move_object(self, parameters: dict[str, Any], step_cb: Any = None) -> dict[str, Any]:
        object_id = str(parameters.get("object_id", "red_block"))
        if object_id != "red_block":
            raise ValueError(f"move_object only supports red_block, not {object_id}")
        position = parameters.get("position")
        if not isinstance(position, (list, tuple)) or len(position) != 3:
            raise ValueError("position must be a 3-vector")
        before = self._block_pos()
        pose = self._torch.tensor(
            [[float(position[0]), float(position[1]), float(position[2]), 0.0, 0.0, 0.0, 1.0]],
            dtype=self._torch.float32,
            device=self._robot.device,
        )
        self._block.write_root_pose_to_sim(pose)
        self._block.write_root_velocity_to_sim(
            self._torch.zeros((1, 6), dtype=self._torch.float32, device=self._robot.device)
        )
        for _ in range(30):
            self._step_once()
            if step_cb is not None:
                step_cb()
        after = self._block_pos()
        self._state_version += 1
        self._event_seq += 1
        self._last_skill = SkillStatus(skill="move_object", phase=SkillPhase.SUCCEEDED, progress=1.0)
        return {"passed": True, "object_id": object_id, "before": before, "after": after}

    def _hold_pose(
        self, parameters: dict[str, Any], cancel_check: Any, progress_cb: Any, step_cb: Any = None
    ) -> dict[str, Any]:
        duration_s = float(parameters.get("duration_s", 2.0))
        if not math.isfinite(duration_s) or not 0.0 < duration_s <= 30.0:
            raise ValueError("duration_s must be in (0, 30]")
        start = self._joints()
        steps = max(1, int(round(duration_s / self._dt)))
        for index in range(steps):
            if cancel_check is not None and cancel_check():
                target = self._robot.data.joint_pos.torch.clone()
                self._robot.set_joint_position_target_index(target=target, full_data=True)
                for _ in range(max(1, int(round(0.05 / self._dt)))):
                    self._step_once()
                self._last_skill = SkillStatus(
                    skill="hold_pose", phase=SkillPhase.CANCELED, progress=0.0, message="cancel acknowledged"
                )
                self._state_version += 1
                self._event_seq += 1
                raise SkillCanceled("cancel acknowledged")
            self._step_once()
            if step_cb is not None:
                step_cb()
            if progress_cb is not None:
                progress_cb((index + 1) / float(steps))
        measured = self._joints()
        drift = max(abs(a - b) for a, b in zip(measured, start))
        self._state_version += 1
        self._event_seq += 1
        self._last_skill = SkillStatus(skill="hold_pose", phase=SkillPhase.SUCCEEDED, progress=1.0)
        return {"passed": drift <= 0.05, "max_drift_rad": drift}

    # -- internals ------------------------------------------------------

    def _step_once(self) -> None:
        self._scene.write_data_to_sim()
        self._sim.step(render=False)
        self._scene.update(self._dt)
        self._sim_step += 1
        self._sim_time_s += self._dt

    def _joints(self) -> list[float]:
        return [
            float(v) for v in self._robot.data.joint_pos.torch[0].detach().to("cpu", self._torch.float64)
        ]

    def _mouth(self) -> list[float]:
        position = self._robot.data.body_pos_w.torch[0, self._palm_index]
        quat = self._robot.data.body_quat_w.torch[0, self._palm_index]
        x, y, z, w = [float(v) for v in quat.detach().to("cpu", self._torch.float64)]
        rotation = self._torch.tensor(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=self._torch.float32,
            device=self._robot.device,
        )
        point = position + rotation @ self._torch.tensor(
            GRASP_LOCAL, dtype=self._torch.float32, device=self._robot.device
        )
        return [float(v) for v in point.detach().to("cpu", self._torch.float64)]

    def _block_pos(self) -> list[float]:
        return [
            float(v)
            for v in self._block.data.root_pos_w.torch[0].detach().to("cpu", self._torch.float64)
        ]

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("worker is closed")


__all__ = ["SO101Worker", "TARGET_BOX_CENTER", "TARGET_BOX_SIZE", "SKILLS"]
