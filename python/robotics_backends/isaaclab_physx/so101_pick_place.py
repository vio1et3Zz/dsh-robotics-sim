"""SO-ARM101 top-grasp pick/place skill with phase-wise physical verification.

The trajectory is the fixed set of joint waypoints solved offline from the USD
chain (``scripts/so101_pick_sequence_solve.py``) for the T06 scene:

    block  (0.18, 0, 0.075) on the 6 cm platform
    target box (0.22, 0.05, 0.02), 10 cm x 10 cm x 4 cm

The mouth frame was measured from the collision meshes (identical to the
visual meshes): fixed bar inner face at palm-local x=-0.0079, moving bar inner
face at +0.0099, mouth centre (0.0071, 0, -0.085); the jaw opens upward.

A skill run is only reported as passed when the measured block poses satisfy
the fixed predicates below -- the jaw closing is never treated as a grasp.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable

GRASP_LOCAL = (0.0071, 0.0, -0.085)
OPEN_JAW_RAD = 0.4
RELEASE_JAW_RAD = 0.6

# joint waypoints in degrees for the five arm joints; the last value is the jaw
# Solved offline with scripts/so101_pick_sequence_solve.py --grasp-x <x>
# (position error 0.00 mm for every waypoint of the 0.16/0.17/0.19/0.21 runs).
GRASP_VARIANTS: dict[float, dict[str, list[float]]] = {
    0.16: {
        "approach": [-94.58, -26.80, 29.46, 87.33, 85.42],
        "grasp": [-94.58, -24.87, 40.74, 74.13, 85.42],
        "lift1": [-94.58, -26.56, 33.52, 83.04, 85.42],
        "lift2": [-94.58, -26.44, 22.80, 93.64, 85.42],
    },
    0.17: {
        "approach": [-94.27, -21.77, 25.22, 86.56, 85.73],
        "grasp": [-94.27, -19.70, 36.23, 73.47, 85.73],
        "lift1": [-94.27, -21.48, 29.21, 82.27, 85.73],
        "lift2": [-94.27, -21.52, 18.62, 92.90, 85.73],
    },
    0.18: {
        "approach": [-94.01, -16.81, 20.65, 86.16, 85.99],
        "grasp": [-94.01, -14.69, 31.48, 73.22, 85.99],
        "lift1": [-94.01, -16.49, 24.59, 81.90, 85.99],
        "lift2": [-94.01, -16.61, 14.09, 92.53, 85.99],
    },
    0.19: {
        "approach": [-93.77, -11.88, 15.75, 86.14, 86.23],
        "grasp": [-93.77, -9.80, 26.45, 73.35, 86.23],
        "lift1": [-93.77, -11.56, 19.66, 81.89, 86.23],
        "lift2": [-93.77, -11.68, 9.17, 92.51, 86.23],
    },
    0.20: {
        "approach": [-93.56, -6.95, 10.47, 86.47, 86.44],
        "grasp": [-93.56, -4.98, 21.14, 73.84, 86.44],
        "lift1": [-93.56, -6.65, 14.40, 82.25, 86.44],
        "lift2": [-93.56, -6.70, 3.83, 92.87, 86.44],
    },
    0.21: {
        # The direct approach sweeps the block at this extension, so enter from
        # a high point first (solved with --preapproach-z 0.175).
        "preapproach": [-93.38, -15.44, -4.13, 95.00, 86.30],
        "approach": [-93.37, -1.96, 4.78, 87.18, 86.63],
        "grasp": [-93.37, -0.18, 15.50, 74.68, 86.63],
        "lift1": [-93.37, -1.72, 8.75, 82.97, 86.63],
        "lift2": [-93.37, -1.61, -2.01, 93.62, 86.63],
    },
}
SHARED_WAYPOINTS: dict[str, list[float]] = {
    "transfer": [-108.37, 7.93, -13.92, 95.00, 71.58],
    "place": [-108.37, 10.81, 5.67, 73.51, 71.63],
    "retreat": [-108.37, -1.66, -13.47, 95.00, 71.44],
}
WAYPOINTS: dict[str, list[float]] = {**GRASP_VARIANTS[0.18], **SHARED_WAYPOINTS}
GRASP_VARIANT_XS = sorted(GRASP_VARIANTS)
GRASP_VARIANT_TOLERANCE_M = 0.015
PLACE_TARGET = (0.22, 0.05)
PLACE_TARGET_TOLERANCE_M = 0.01
GRIPPER_OPEN = OPEN_JAW_RAD
GRIPPER_CLOSED = 0.0

# fixed acceptance thresholds (metres / seconds)
APPROACH_DISPLACEMENT_M = 0.008
GRASP_XY_TOLERANCE_M = 0.010
LIFT_MIN_M = 0.030
CARRY_XY_TOLERANCE_M = 0.060
PLACE_TOLERANCE_M = 0.040
PLACE_MAX_Z_M = 0.090

# reachable region for the top grasp, from the offline reachability sweep
REACHABLE_X = (0.15, 0.27)
REACHABLE_Y = (-0.08, 0.08)
REACHABLE_MOUTH_Z = (0.070, 0.160)


class SkillCanceled(Exception):
    """Raised when a cancel request is observed between control steps."""


@dataclass
class SkillResult:
    passed: bool
    checks: dict[str, bool]
    failure: str | None
    measurements: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "passed": self.passed,
            "checks": self.checks,
            "failure": self.failure,
            "measurements": self.measurements,
        }


class SO101PickPlace:
    """Drives one scene; all methods run on the simulation thread."""

    def __init__(self, sim: Any, scene: Any, robot: Any, block: Any, target_box: Any = None, *, dt: float = 0.005) -> None:
        import torch  # local import keeps the module importable without Isaac

        self._torch = torch
        self._sim = sim
        self._scene = scene
        self._robot = robot
        self._block = block
        self._target_box = target_box
        self._dt = float(dt)
        self._steps = 0
        self._step_cb: Callable[[], None] | None = None
        self._body_names = list(robot.body_names)
        self._joint_names = list(robot.joint_names)
        self._palm_index = self._body_names.index("gripper")
        self._gripper_index = self._joint_names.index("gripper")
        limits = robot.data.joint_pos_limits.torch[0].detach()
        self._low = limits[:, 0]
        self._high = limits[:, 1]

    # -- public ---------------------------------------------------------

    def observe(self) -> dict[str, Any]:
        return {
            "joints": self._joints(),
            "mouth": self._mouth(),
            "block_pose": self._block_pose(),
            "target_box_pose": self._target_box_pose(),
        }

    def precheck(self, place_xy: tuple[float, float]) -> tuple[bool, str | None]:
        x, y = float(place_xy[0]), float(place_xy[1])
        if not (REACHABLE_X[0] <= x <= REACHABLE_X[1] and REACHABLE_Y[0] <= y <= REACHABLE_Y[1]):
            return False, "UNREACHABLE_TARGET"
        if abs(x - PLACE_TARGET[0]) > PLACE_TARGET_TOLERANCE_M or abs(y - PLACE_TARGET[1]) > PLACE_TARGET_TOLERANCE_M:
            return False, "UNREACHABLE_TARGET"
        block = self._block_pose()
        if block is None:
            return False, "OBJECT_NOT_FOUND"
        if not (0.14 <= block["position"][0] <= 0.22 and abs(block["position"][1]) <= 0.02):
            return False, "OBJECT_OUT_OF_WORKSPACE"
        return True, None

    def execute(
        self,
        place_xy: tuple[float, float],
        *,
        grasp_x: float | None = None,
        cancel_check: Callable[[], bool] | None = None,
        progress_cb: Callable[[float], None] | None = None,
        fault_after_grasp: Callable[[], None] | None = None,
        step_cb: Callable[[], None] | None = None,
    ) -> SkillResult:
        self._step_cb = step_cb
        checks: dict[str, bool] = {}
        measurements: dict[str, Any] = {"start": self.observe()}
        ok, failure = self.precheck(place_xy)
        if not ok:
            return SkillResult(False, {"precheck": False}, failure, measurements)

        observed_x = self._block_pos()[0]
        requested_x = observed_x if grasp_x is None else float(grasp_x)
        variant = min(GRASP_VARIANT_XS, key=lambda value: abs(value - requested_x))
        if abs(variant - requested_x) > GRASP_VARIANT_TOLERANCE_M:
            return SkillResult(False, {"precheck": False}, "GRASP_POSE_UNAVAILABLE", measurements)
        self._waypoints = {**GRASP_VARIANTS[variant], **SHARED_WAYPOINTS}
        measurements["observed_block_x"] = observed_x
        measurements["grasp_variant_x"] = variant

        block_before = self._block_pos()
        measurements["block_before"] = block_before

        # Some variants need a high entry point: the direct approach from rest
        # sweeps through the block at full extension.
        if "preapproach" in self._waypoints:
            self._command(self._waypoints["preapproach"], GRIPPER_OPEN, 220, cancel_check, progress_cb, 0.0, 0.10)
            self._step(40, cancel_check)

        # approach with the jaw open; block must not be disturbed
        approach_from = 0.10 if "preapproach" in self._waypoints else 0.0
        self._command(self._waypoints["approach"], GRIPPER_OPEN, 260, cancel_check, progress_cb, approach_from, 0.25)
        self._step(60, cancel_check)
        approach_block = self._block_pos()
        measurements["block_after_approach"] = approach_block
        checks["block_untouched_on_approach"] = math.dist(approach_block, block_before) <= APPROACH_DISPLACEMENT_M
        if not checks["block_untouched_on_approach"]:
            return SkillResult(False, checks, "BLOCK_MOVED_ON_APPROACH", measurements)

        # descend to the grasp pose and verify the mouth frames the block
        self._command(self._waypoints["grasp"], GRIPPER_OPEN, 140, cancel_check, progress_cb, 0.25, 0.40)
        self._step(60, cancel_check)
        grasp_mouth = self._mouth()
        grasp_block = self._block_pos()
        measurements["grasp_mouth"] = grasp_mouth
        measurements["block_at_grasp"] = grasp_block
        checks["grasp_centered"] = (
            math.dist(grasp_mouth[:2], grasp_block[:2]) <= GRASP_XY_TOLERANCE_M
            and math.dist(grasp_block, block_before) <= 0.015
        )
        if not checks["grasp_centered"]:
            return SkillResult(False, checks, "GRASP_MISSED", measurements)
        if fault_after_grasp is not None:
            # deterministic fault injection for the empty-grasp test
            fault_after_grasp()
            measurements["fault_injected"] = "after_grasp"

        # close slowly, then lift and check the block follows the mouth
        for value in (0.35, 0.30, 0.25, 0.20, 0.15, 0.10, 0.05, GRIPPER_CLOSED):
            self._set_gripper(value, 35, cancel_check)
        measurements["block_after_close"] = self._block_pos()
        self._command(self._waypoints["lift1"], GRIPPER_CLOSED, 130, cancel_check, progress_cb, 0.40, 0.55)
        self._step(50, cancel_check)
        lift1_mouth = self._mouth()
        lift1_block = self._block_pos()
        measurements["lift1_mouth"] = lift1_mouth
        measurements["block_lift1"] = lift1_block
        self._command(self._waypoints["lift2"], GRIPPER_CLOSED, 130, cancel_check, progress_cb, 0.55, 0.70)
        self._step(50, cancel_check)
        lift2_block = self._block_pos()
        measurements["block_lift2"] = lift2_block
        checks["lifted"] = lift2_block[2] >= block_before[2] + LIFT_MIN_M
        checks["block_follows_mouth"] = math.dist(lift2_block[:2], self._mouth()[:2]) <= 0.025
        if not (checks["lifted"] and checks["block_follows_mouth"]):
            return SkillResult(False, checks, "EMPTY_GRASP", measurements)

        # transfer and place on the target box
        self._command(self._waypoints["transfer"], GRIPPER_CLOSED, 180, cancel_check, progress_cb, 0.70, 0.82)
        self._step(50, cancel_check)
        transfer_block = self._block_pos()
        measurements["block_transfer"] = transfer_block
        checks["carried_to_place"] = math.dist(transfer_block[:2], place_xy) <= CARRY_XY_TOLERANCE_M
        if not checks["carried_to_place"]:
            return SkillResult(False, checks, "DROPPED_IN_TRANSFER", measurements)

        self._command(self._waypoints["place"], GRIPPER_CLOSED, 160, cancel_check, progress_cb, 0.82, 0.90)
        self._step(50, cancel_check)
        measurements["block_placed"] = self._block_pos()
        self._set_gripper(RELEASE_JAW_RAD, 150, cancel_check)
        self._step(100, cancel_check)
        released = self._block_pos()
        measurements["block_released"] = released
        self._command(self._waypoints["retreat"], RELEASE_JAW_RAD, 160, cancel_check, progress_cb, 0.90, 0.98)
        self._step(50, cancel_check)
        final = self._block_pos()
        measurements["block_final"] = final
        checks["placed_in_target_region"] = (
            math.dist(final[:2], place_xy) <= PLACE_TOLERANCE_M and final[2] <= PLACE_MAX_Z_M
        )
        checks["released"] = math.dist(final, released) <= 0.02
        failure = None
        if not checks["placed_in_target_region"]:
            failure = "PLACE_OUTSIDE_REGION"
        elif not checks["released"]:
            failure = "OBJECT_NOT_RELEASED"
        self._progress(progress_cb, 1.0)
        return SkillResult(all(checks.values()), checks, failure, measurements)

    @property
    def step_count(self) -> int:
        return self._steps

    # -- internals ------------------------------------------------------

    def _advance(self) -> None:
        """One physics step and the monitor hook; the only place ``sim.step`` runs."""
        self._scene.write_data_to_sim()
        self._sim.step(render=False)
        self._steps += 1
        self._scene.update(self._dt)
        if not bool(self._torch.isfinite(self._robot.data.joint_pos.torch).all()):
            raise RuntimeError("non-finite joint state")
        if self._step_cb is not None:
            self._step_cb()

    def _step(self, count: int, cancel_check: Callable[[], bool] | None) -> None:
        for _ in range(count):
            if cancel_check is not None and cancel_check():
                self._hold_current()
                raise SkillCanceled("cancel acknowledged")
            self._advance()

    def _hold_current(self) -> None:
        target = self._robot.data.joint_pos.torch.clone()
        self._robot.set_joint_position_target_index(target=target, full_data=True)
        for _ in range(max(1, int(round(0.05 / self._dt)))):
            self._advance()

    def _command(
        self,
        angles_deg: list[float],
        gripper: float,
        steps: int,
        cancel_check: Callable[[], bool] | None,
        progress_cb: Callable[[float], None] | None,
        progress_from: float,
        progress_to: float,
    ) -> None:
        start = self._robot.data.joint_pos.torch.clone()
        goal = start.clone()
        goal[0, :5] = self._torch.tensor([angles_deg], dtype=self._torch.float32, device=self._robot.device) * math.pi / 180.0
        goal[0, 5] = gripper
        goal = self._torch.max(self._torch.min(goal, self._high.unsqueeze(0)), self._low.unsqueeze(0))
        for index in range(1, steps + 1):
            if cancel_check is not None and cancel_check():
                self._hold_current()
                raise SkillCanceled("cancel acknowledged")
            alpha = index / float(steps)
            target = start + (goal - start) * alpha
            self._robot.set_joint_position_target_index(target=target, full_data=True)
            self._advance()
            if index % 10 == 0:
                self._progress(progress_cb, progress_from + (progress_to - progress_from) * alpha)

    def _set_gripper(self, value: float, steps: int, cancel_check: Callable[[], bool] | None) -> None:
        target = self._robot.data.joint_pos_target.torch.clone()
        target[0, self._gripper_index] = value
        self._robot.set_joint_position_target_index(target=target, full_data=True)
        self._step(steps, cancel_check)

    def _progress(self, callback: Callable[[float], None] | None, value: float) -> None:
        if callback is not None:
            callback(min(max(float(value), 0.0), 1.0))

    def _joints(self) -> list[float]:
        return [float(v) for v in self._robot.data.joint_pos.torch[0].detach().to("cpu", self._torch.float64)]

    def _mouth(self) -> list[float]:
        pos = self._robot.data.body_pos_w.torch[0, self._palm_index]
        quat = self._robot.data.body_quat_w.torch[0, self._palm_index]
        rotation = self._quat_matrix(quat)
        point = pos + rotation @ self._torch.tensor(GRASP_LOCAL, dtype=self._torch.float32, device=self._robot.device)
        return [float(v) for v in point.detach().to("cpu", self._torch.float64)]

    def _quat_matrix(self, quat: Any) -> Any:
        x, y, z, w = [float(v) for v in quat.detach().to("cpu", self._torch.float64)]
        return self._torch.tensor(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=self._torch.float32,
            device=self._robot.device,
        )

    def _block_pos(self) -> list[float]:
        return [float(v) for v in self._block.data.root_pos_w.torch[0].detach().to("cpu", self._torch.float64)]

    def _block_pose(self) -> dict[str, Any] | None:
        if self._block is None:
            return None
        return {
            "position": self._block_pos(),
            "quat_xyzw": [
                float(v) for v in self._block.data.root_quat_w.torch[0].detach().to("cpu", self._torch.float64)
            ],
        }

    def _target_box_pose(self) -> dict[str, Any] | None:
        if self._target_box is None:
            return None
        return {
            "position": [
                float(v)
                for v in self._target_box.data.root_pos_w.torch[0].detach().to("cpu", self._torch.float64)
            ]
        }


__all__ = [
    "GRASP_LOCAL",
    "WAYPOINTS",
    "SO101PickPlace",
    "SkillCanceled",
    "SkillResult",
]


