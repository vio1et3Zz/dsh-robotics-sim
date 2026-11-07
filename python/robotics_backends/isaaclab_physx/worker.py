"""Franka PhysX worker: lifecycle, joint calibration and state snapshots.

T02 scope only.  Skills are executed synchronously on the caller's thread; the
Gateway/Job layer (T03) will put a command queue and cancellation in front of
this class.  No network code lives here.
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
import traceback
import uuid
from collections import deque
from datetime import datetime, timezone
from typing import Any

import torch

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



MAX_DELTA_RAD = 0.20
MAX_HOLD_S = 2.0
LIMIT_MARGIN_RAD = 0.05

# A1: the simulation thread publishes a state snapshot at 2 Hz and (workers
# with a camera) one rendered frame per second while a skill runs.  The command
# loop answers observe/capture from these copies, never from the simulation.
SNAPSHOT_PERIOD_S = 0.5
FRAME_PERIOD_S = 1.0
FRAME_CAPACITY = 8

_DEBUG = os.environ.get("DSH_ROBOTICS_DEBUG") == "1"


def _debug(message: str) -> None:
    if _DEBUG:
        print(f"[monitor] {message}", file=sys.stderr, flush=True)


class SkillCanceled(Exception):
    """Raised when the command loop observed a cancel request mid-skill."""


class SnapshotStore:
    """Thread-safe latest ``WorldState`` plus a ring buffer of rendered frames.

    The simulation thread publishes; the command loop reads.  Only plain dicts
    leave this object, so no torch tensor or Isaac handle crosses a thread.
    """

    def __init__(self, *, frame_capacity: int = FRAME_CAPACITY) -> None:
        self._lock = threading.Lock()
        self._state: dict[str, Any] | None = None
        self._frames: deque[dict[str, Any]] = deque(maxlen=max(1, frame_capacity))

    def publish_state(self, state: WorldState) -> None:
        payload = state.model_dump(mode="json")
        with self._lock:
            self._state = payload

    def state(self) -> WorldState | None:
        with self._lock:
            payload = self._state
        return WorldState.model_validate(payload) if payload is not None else None

    def add_frame(self, frame: dict[str, Any]) -> None:
        with self._lock:
            self._frames.append(frame)

    def latest_frame(self) -> dict[str, Any] | None:
        with self._lock:
            return self._frames[-1] if self._frames else None

    def clear_frames(self) -> None:
        with self._lock:
            self._frames.clear()


class WorkerMonitor:
    """Rate-limited snapshot/frame publishing driven by the simulation thread.

    ``tick`` is called after every control step (from the skill thread) and is
    cheap: it only does work when a period elapsed.  Workers implement
    ``publish_snapshot`` and (optionally) ``publish_frame``.
    """

    def __init__(
        self,
        worker: Any,
        *,
        state_period_s: float = SNAPSHOT_PERIOD_S,
        frame_period_s: float = FRAME_PERIOD_S,
    ) -> None:
        self._worker = worker
        self._state_period_s = max(0.05, float(state_period_s))
        self._frame_period_s = max(0.1, float(frame_period_s))
        self._publish_frame = getattr(worker, "publish_frame", None)
        self._running = threading.Event()
        self._last_state = 0.0
        self._last_frame = 0.0
        self.state_publishes = 0
        self.frame_publishes = 0
        self.errors = 0

    def start(self) -> None:
        now = time.perf_counter()
        self._last_state = now
        self._last_frame = now
        self.state_publishes = 0
        self.frame_publishes = 0
        self.errors = 0
        self._running.set()

    def stop(self) -> None:
        self._running.clear()

    def tick(self) -> None:
        if not self._running.is_set():
            return
        now = time.perf_counter()
        if now - self._last_state >= self._state_period_s:
            self._last_state = now
            started = time.perf_counter()
            try:
                self._worker.publish_snapshot()
                self.state_publishes += 1
                if self.state_publishes <= 3:
                    _debug(f"snapshot #{self.state_publishes} {1000 * (time.perf_counter() - started):.1f} ms")
            except Exception:
                self.errors += 1
                if self.errors <= 3:
                    _debug("snapshot failed")
                    traceback.print_exc(file=sys.stderr)
        if self._publish_frame is not None and now - self._last_frame >= self._frame_period_s:
            self._last_frame = now
            started = time.perf_counter()
            try:
                self._publish_frame()
                self.frame_publishes += 1
                if self.frame_publishes <= 3:
                    _debug(f"frame #{self.frame_publishes} {1000 * (time.perf_counter() - started):.1f} ms")
            except Exception:
                self.errors += 1
                if self.errors <= 3:
                    _debug("frame failed")
                    traceback.print_exc(file=sys.stderr)

    def stats(self) -> dict[str, Any]:
        return {
            "state_publishes": self.state_publishes,
            "frame_publishes": self.frame_publishes,
            "errors": self.errors,
        }


class IsaacLabWorker:
    """One Franka scene; all methods must run on the simulation thread."""

    def __init__(self, sim: Any, scene: Any, robot: Any, *, device: str, dt: float) -> None:
        self._sim = sim
        self._scene = scene
        self._robot = robot
        self._device = device
        self._dt = float(dt)
        self._closed = False
        self._simulation_id = f"sim_{uuid.uuid4().hex}"
        self._episode_id = f"ep_{uuid.uuid4().hex}"
        self._state_version = 0
        self._event_seq = 0
        self._sim_step = 0
        self._sim_time_s = 0.0
        self._last_skill: SkillStatus | None = None

        self._joint_names: list[str] = list(robot.joint_names)
        self._num_joints = len(self._joint_names)
        limits = robot.data.joint_pos_limits.torch[0].detach().to("cpu", torch.float64).numpy()
        velocities = robot.data.joint_vel_limits.torch[0].detach().to("cpu", torch.float64).numpy()
        defaults = robot.data.default_joint_pos.torch[0].detach().to("cpu", torch.float64).numpy()
        self._limits = [(float(pair[0]), float(pair[1])) for pair in limits]
        self._velocity_limits = [float(value) for value in velocities]
        self._default_joint_pos = [float(value) for value in defaults]

        body_names = list(robot.body_names)
        self._ee_body_index = body_names.index("panda_hand") if "panda_hand" in body_names else len(body_names) - 1

        self._snapshots = SnapshotStore()
        self._monitor = WorkerMonitor(self)
        self._scene.update(self._dt)

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
        joint_positions = self._joint_positions()
        joint_velocities = [
            float(value) for value in self._robot.data.joint_vel.torch[0].detach().to("cpu", torch.float64)
        ]
        ee_position, ee_quat = self._ee_pose()
        finger_width = sum(
            position
            for name, position in zip(self._joint_names, joint_positions)
            if "finger" in name
        )
        robot_state = RobotState(
            robot_id="franka",
            dof=self._num_joints,
            joint_positions=joint_positions,
            joint_velocities=joint_velocities,
            ee_pose=Pose(
                frame=Frame.WORLD,
                position=ee_position,
                orientation_xyzw=ee_quat,
            ),
            gripper=GripperState(width_m=min(max(finger_width, 0.0), 1.0), closed=finger_width < 0.005),
        )
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
            objects=[],
            skills=[self._last_skill] if self._last_skill is not None else [],
        )

    def reset(self) -> WorldState:
        self._require_open()
        self._sim.reset()
        self._scene.update(self._dt)
        self._episode_id = f"ep_{uuid.uuid4().hex}"
        self._state_version += 1
        self._event_seq += 1
        self._sim_step = 0
        self._sim_time_s = 0.0
        self._last_skill = None
        return self.observe()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._sim.stop()
        except Exception:
            pass

    # -- calibration ---------------------------------------------------

    def calibrate_joint(
        self,
        *,
        joint_name: str = "panda_joint1",
        delta_rad: float = 0.05,
        hold_s: float = 0.25,
        settle_s: float = 0.20,
        cancel_check: Any = None,
        progress_cb: Any = None,
        step_cb: Any = None,
    ) -> dict[str, Any]:
        """Move one joint by a bounded offset, measure the physical response."""

        self._require_open()
        if joint_name not in self._joint_names:
            raise ValueError(f"unknown joint: {joint_name}")
        if not math.isfinite(delta_rad) or abs(delta_rad) > MAX_DELTA_RAD:
            raise ValueError(f"delta_rad must be finite and within +/-{MAX_DELTA_RAD}")
        if not math.isfinite(hold_s) or not 0.0 < hold_s <= MAX_HOLD_S:
            raise ValueError(f"hold_s must be in (0, {MAX_HOLD_S}]")
        if not math.isfinite(settle_s) or not 0.0 < settle_s <= MAX_HOLD_S:
            raise ValueError(f"settle_s must be in (0, {MAX_HOLD_S}]")

        index = self._joint_names.index(joint_name)
        start = self._joint_positions()
        lower, upper = self._limits[index]
        target = start[index] + delta_rad
        target = min(max(target, lower + LIMIT_MARGIN_RAD), upper - LIMIT_MARGIN_RAD)

        self._last_skill = SkillStatus(
            skill="calibrate_joint", phase=SkillPhase.RUNNING, progress=0.0, message=f"{joint_name} -> {target:.4f}"
        )
        self._event_seq += 1

        full_target = self._joint_position_tensor().clone()
        full_target[0, index] = target
        self._robot.set_joint_position_target_index(target=full_target, full_data=True)
        self._scene.write_data_to_sim()

        hold_steps = max(1, int(round(hold_s / self._dt)))
        settle_steps = max(1, int(round(settle_s / self._dt)))

        def check_cancel() -> None:
            if cancel_check is None or not cancel_check():
                return
            # Stop generating new targets and actively hold the measured pose.
            hold_target = self._joint_position_tensor().clone()
            self._robot.set_joint_position_target_index(target=hold_target, full_data=True)
            for _ in range(max(1, int(round(0.05 / self._dt)))):
                self._step()
            self._last_skill = SkillStatus(
                skill="calibrate_joint",
                phase=SkillPhase.CANCELED,
                progress=0.0,
                message="cancel acknowledged",
            )
            self._state_version += 1
            self._event_seq += 1
            raise SkillCanceled("cancel acknowledged")

        hold_samples: list[float] = []
        for step in range(hold_steps + settle_steps):
            check_cancel()
            self._step()
            if step_cb is not None:
                step_cb()
            value = self._joint_positions()[index]
            if not math.isfinite(value):
                raise RuntimeError(f"non-finite joint position during calibration: {value}")
            if step >= settle_steps:
                hold_samples.append(value)
            if progress_cb is not None:
                progress_cb(min(1.0, (step + 1) / float(hold_steps + settle_steps)))

        measured_hold = self._joint_positions()
        hold_error = abs(measured_hold[index] - target)
        tail = hold_samples[-10:] if len(hold_samples) >= 10 else hold_samples
        max_tail_error = max(abs(value - target) for value in tail) if tail else hold_error

        # Return to the pre-calibration pose and let the controller settle.
        return_target = self._joint_position_tensor().clone()
        return_target[0, index] = start[index]
        self._robot.set_joint_position_target_index(target=return_target, full_data=True)
        for _ in range(settle_steps):
            check_cancel()
            self._step()
            if step_cb is not None:
                step_cb()
        settled = self._joint_positions()

        self._state_version += 1
        self._event_seq += 1
        self._last_skill = SkillStatus(
            skill="calibrate_joint",
            phase=SkillPhase.SUCCEEDED if hold_error <= 0.05 else SkillPhase.FAILED,
            progress=1.0,
            message=f"hold_error={hold_error:.4f} rad",
        )
        return {
            "joint": joint_name,
            "index": index,
            "delta_requested_rad": float(delta_rad),
            "target_rad": float(target),
            "lower_rad": float(lower),
            "upper_rad": float(upper),
            "margin_rad": LIMIT_MARGIN_RAD,
            "hold_s": float(hold_s),
            "settle_s": float(settle_s),
            "hold_steps": hold_steps,
            "start_rad": float(start[index]),
            "measured_hold_rad": float(measured_hold[index]),
            "hold_error_rad": float(hold_error),
            "max_tail_error_rad": float(max_tail_error),
            "settled_rad": float(settled[index]),
            "return_error_rad": float(abs(settled[index] - start[index])),
            "all_joint_positions_after_hold": measured_hold,
            "passed": bool(hold_error <= 0.05 and abs(settled[index] - start[index]) <= 0.05),
        }

    # -- hold ----------------------------------------------------------

    def hold_pose(
        self,
        *,
        duration_s: float = 10.0,
        cancel_check: Any = None,
        progress_cb: Any = None,
        step_cb: Any = None,
    ) -> dict[str, Any]:
        """Hold the measured pose for a bounded time; used for cancel tests."""

        self._require_open()
        if not math.isfinite(duration_s) or not 0.0 < duration_s <= 30.0:
            raise ValueError("duration_s must be in (0, 30]")
        start = self._joint_positions()
        self._last_skill = SkillStatus(skill="hold_pose", phase=SkillPhase.RUNNING, progress=0.0)
        self._event_seq += 1
        target = self._joint_position_tensor().clone()
        self._robot.set_joint_position_target_index(target=target, full_data=True)
        steps = max(1, int(round(duration_s / self._dt)))
        for step in range(steps):
            if cancel_check is not None and cancel_check():
                hold_target = self._joint_position_tensor().clone()
                self._robot.set_joint_position_target_index(target=hold_target, full_data=True)
                for _ in range(max(1, int(round(0.05 / self._dt)))):
                    self._step()
                self._last_skill = SkillStatus(
                    skill="hold_pose",
                    phase=SkillPhase.CANCELED,
                    progress=0.0,
                    message="cancel acknowledged",
                )
                self._state_version += 1
                self._event_seq += 1
                raise SkillCanceled("cancel acknowledged")
            self._step()
            if step_cb is not None:
                step_cb()
            if progress_cb is not None:
                progress_cb((step + 1) / float(steps))
        measured = self._joint_positions()
        drift = max(abs(a - b) for a, b in zip(measured, start))
        self._state_version += 1
        self._event_seq += 1
        self._last_skill = SkillStatus(skill="hold_pose", phase=SkillPhase.SUCCEEDED, progress=1.0)
        return {
            "duration_s": float(duration_s),
            "start_joint_positions": start,
            "measured_joint_positions": measured,
            "max_drift_rad": float(drift),
            "passed": bool(drift <= 0.05),
        }

    # -- robot metadata ------------------------------------------------

    def robot_spec(self) -> RobotSpec:
        joints = []
        for index, name in enumerate(self._joint_names):
            is_finger = "finger" in name
            lower, upper = self._limits[index]
            if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
                lower, upper = (0.0, 0.08) if is_finger else (-math.pi, math.pi)
            home = min(max(self._default_joint_pos[index], lower), upper)
            velocity = self._velocity_limits[index]
            if not math.isfinite(velocity) or velocity <= 0.0:
                velocity = 1.0
            joints.append(
                JointSpec(
                    name=name,
                    logical_name=name,
                    type=JointType.PRISMATIC if is_finger else JointType.REVOLUTE,
                    unit="m" if is_finger else "rad",
                    lower=lower,
                    upper=upper,
                    home=home,
                    velocity_limit=velocity,
                )
            )
        finger_joints = [name for name in self._joint_names if "finger" in name]
        parent_joint = "panda_joint7" if "panda_joint7" in self._joint_names else self._joint_names[-1]
        # Lazy import: keeps this module (and the command loop tests) importable
        # without launching Kit.
        from .scene import FRANKA_CFG

        return RobotSpec(
            robot_id="franka",
            model="Franka Emika Panda",
            dof=self._num_joints,
            joints=joints,
            end_effector=EndEffectorSpec(name="panda_hand", parent_joint=parent_joint),
            gripper=GripperSpec(
                kind="parallel" if finger_joints else "none",
                joint_names=finger_joints,
                open_value=0.04,
                close_value=0.0,
                max_width_m=0.08 if finger_joints else 0.0,
            ),
            control_modes=[ControlMode.POSITION],
            skills=["observe_scene", "calibrate_joint", "hold_pose"],
            asset=RobotAsset(
                usd_path=FRANKA_CFG.spawn.usd_path,
                license="Isaac Lab / NVIDIA Isaac Sim asset license",
                source="isaaclab_assets.robots.franka.FRANKA_PANDA_CFG (franka.usd)",
            ),
        )

    # -- internals -----------------------------------------------------

    def _step(self) -> None:
        self._scene.write_data_to_sim()
        self._sim.step(render=False)
        self._scene.update(self._dt)
        self._sim_step += 1
        self._sim_time_s += self._dt

    def _joint_position_tensor(self) -> torch.Tensor:
        return self._robot.data.joint_pos.torch.clone()

    def _joint_positions(self) -> list[float]:
        return [
            float(value)
            for value in self._robot.data.joint_pos.torch[0].detach().to("cpu", torch.float64)
        ]

    def _ee_pose(self) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        position = self._robot.data.body_pos_w.torch[0, self._ee_body_index].detach().to("cpu", torch.float64)
        quat = self._robot.data.body_quat_w.torch[0, self._ee_body_index].detach().to("cpu", torch.float64)
        norm = float(torch.linalg.norm(quat))
        if norm <= 1e-8:
            quat = torch.tensor([0.0, 0.0, 0.0, 1.0], dtype=torch.float64)
        else:
            quat = quat / norm
        return (
            (float(position[0]), float(position[1]), float(position[2])),
            (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
        )

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("worker is closed")


__all__ = [
    "IsaacLabWorker",
    "SkillCanceled",
    "SnapshotStore",
    "WorkerMonitor",
    "FRAME_CAPACITY",
    "FRAME_PERIOD_S",
    "SNAPSHOT_PERIOD_S",
]
