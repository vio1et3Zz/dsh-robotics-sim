"""A1 tests: snapshots/frame buffer and concurrent skill command loop.

No Isaac here: ``WorkerServer`` drives a fake worker over a real socket pair,
so the concurrency contract (observe/capture/busy/cancel while a skill runs) is
tested without a GPU.
"""

from __future__ import annotations

import json
import socket
import sys
import threading
import time
import unittest
import uuid
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

from robotics_backends.isaaclab_physx.server import WorkerServer  # noqa: E402
from robotics_backends.isaaclab_physx.worker import (  # noqa: E402
    SkillCanceled,
    SnapshotStore,
    WorkerMonitor,
)
from robotics_protocol import (  # noqa: E402
    ControlMode,
    EndEffectorSpec,
    Frame,
    GripperSpec,
    GripperState,
    JointSpec,
    JointType,
    ObservationSource,
    Pose,
    RobotAsset,
    RobotSpec,
    RobotState,
    WorldState,
)


def robot_spec() -> RobotSpec:
    return RobotSpec(
        robot_id="franka",
        model="fake",
        dof=1,
        joints=[
            JointSpec(
                name="j1",
                logical_name="j1",
                type=JointType.REVOLUTE,
                unit="rad",
                lower=-1.0,
                upper=1.0,
                home=0.0,
                velocity_limit=1.0,
            )
        ],
        end_effector=EndEffectorSpec(name="ee", parent_joint="j1"),
        gripper=GripperSpec(kind="none"),
        control_modes=[ControlMode.POSITION],
        skills=["hold_pose"],
        asset=RobotAsset(usd_path="fake.usd", license="test", source="test"),
    )


def world_state(sim_step: int) -> WorldState:
    return WorldState(
        simulation_id="sim_" + "a" * 32,
        episode_id="ep_" + "b" * 32,
        env_id=0,
        state_version=0,
        event_seq=0,
        sim_step=sim_step,
        sim_time_s=sim_step * 0.005,
        wall_time=datetime.now(timezone.utc),
        source=ObservationSource.SIM_GROUND_TRUTH,
        frame=Frame.WORLD,
        robots=[
            RobotState(
                robot_id="franka",
                dof=1,
                joint_positions=[0.0],
                joint_velocities=[0.0],
                ee_pose=Pose(frame=Frame.WORLD, position=(0.0, 0.0, 0.0), orientation_xyzw=(0.0, 0.0, 0.0, 1.0)),
                gripper=GripperState(width_m=0.5, closed=False),
            )
        ],
    )


class FakeWorker:
    """Same duck-typed surface as ``SO101Worker`` for the command loop."""

    def __init__(self, *, skill_duration_s: float = 2.0) -> None:
        self._snapshots = SnapshotStore()
        self.monitor = WorkerMonitor(self, state_period_s=0.05, frame_period_s=10.0)
        self.skill_duration_s = skill_duration_s
        self.steps = 0
        self.fresh_captures = 0

    def robot_spec(self) -> RobotSpec:
        return robot_spec()

    def observe(self) -> WorldState:
        return world_state(self.steps)

    def observe_snapshot(self) -> WorldState | None:
        return self._snapshots.state()

    def publish_snapshot(self) -> None:
        self._snapshots.publish_state(self.observe())

    def publish_frame(self) -> None:
        frame_id = f"frame_{uuid.uuid4().hex}"
        self._snapshots.add_frame(
            {
                "frame_id": frame_id,
                "metadata": {"frame_id": frame_id, "sim_step": self.steps, "buffered": True},
                "persisted": True,
            }
        )

    def capture_frame(self, camera_id: str = "front", *, fresh: bool = True) -> dict:
        if fresh:
            self.fresh_captures += 1
            return {"frame_id": f"frame_{uuid.uuid4().hex}", "sim_step": self.steps, "buffered": False}
        slot = self._snapshots.latest_frame()
        if slot is None:
            raise RuntimeError("FRAME_NOT_READY: no buffered frame yet")
        return dict(slot["metadata"])

    def run_skill(self, skill, parameters, *, cancel_check=None, progress_cb=None, step_cb=None, phase_cb=None):
        deadline = time.time() + self.skill_duration_s
        while time.time() < deadline:
            if cancel_check is not None and cancel_check():
                raise SkillCanceled("cancel acknowledged")
            self.steps += 1
            if step_cb is not None:
                step_cb()
            if progress_cb is not None:
                progress_cb(min(1.0, (time.time() - (deadline - self.skill_duration_s)) / self.skill_duration_s))
            time.sleep(0.01)
        return {"passed": True, "steps": self.steps}


class SnapshotStoreTests(unittest.TestCase):
    def test_state_round_trip(self) -> None:
        store = SnapshotStore()
        self.assertIsNone(store.state())
        store.publish_state(world_state(12))
        state = store.state()
        self.assertEqual(state.sim_step, 12)
        self.assertEqual(state.robots[0].dof, 1)

    def test_frames_keep_only_the_newest_capacity(self) -> None:
        store = SnapshotStore(frame_capacity=2)
        for step in range(5):
            store.add_frame({"frame_id": f"frame_{step}", "metadata": {"sim_step": step}, "persisted": True})
        self.assertEqual(store.latest_frame()["frame_id"], "frame_4")
        store.clear_frames()
        self.assertIsNone(store.latest_frame())

    def test_monitor_is_rate_limited_and_survives_worker_errors(self) -> None:
        class Counter:
            def __init__(self) -> None:
                self.states = 0
                self.frames = 0

            def publish_snapshot(self) -> None:
                self.states += 1

            def publish_frame(self) -> None:
                self.frames += 1
                raise RuntimeError("camera not ready")

        worker = Counter()
        monitor = WorkerMonitor(worker, state_period_s=0.05, frame_period_s=0.1)
        monitor.tick()
        self.assertEqual(worker.states, 0, "monitor must be idle before start")
        monitor.start()
        monitor.tick()
        self.assertEqual(worker.states, 0, "start seeds the timer; no immediate publish")
        time.sleep(0.06)
        monitor.tick()
        self.assertEqual(worker.states, 1)
        monitor.tick()
        self.assertEqual(worker.states, 1, "a second tick inside the period is a no-op")
        time.sleep(0.06)
        monitor.tick()
        self.assertEqual(worker.states, 2)
        self.assertGreaterEqual(worker.frames, 1)
        self.assertGreaterEqual(monitor.errors, 1, "frame errors are counted, not raised")
        monitor.stop()
        monitor.tick()
        self.assertEqual(worker.states, 2)


class WorkerServerConcurrencyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.client, server_sock = socket.socketpair()
        self.client.settimeout(5.0)
        self.worker = FakeWorker(skill_duration_s=2.0)
        self.server = WorkerServer(self.worker, server_sock)
        self.thread = threading.Thread(target=self.server.run, daemon=True)
        self.thread.start()
        self.reader = self.client.makefile("r", encoding="utf-8")
        self.pending: list[dict] = []
        ready = self.read_until(lambda message: message.get("type") == "ready")
        self.assertEqual(ready["robot_spec"]["robot_id"], "franka")

    def tearDown(self) -> None:
        try:
            self.closed = False
            self.send({"id": "close", "op": "close"})
            self.read_until(lambda message: message.get("id") == "close")
        except Exception:
            pass
        try:
            self.client.close()
        except Exception:
            pass
        self.thread.join(timeout=5.0)
        self.server.close()

    # -- helpers -------------------------------------------------------

    def send(self, payload: dict) -> None:
        self.client.sendall((json.dumps(payload) + "\n").encode("utf-8"))

    def read_until(self, predicate, timeout_s: float = 5.0) -> dict:
        for index, message in enumerate(self.pending):
            if predicate(message):
                return self.pending.pop(index)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            line = self.reader.readline()
            if not line:
                raise AssertionError("worker closed the connection")
            message = json.loads(line)
            if predicate(message):
                return message
            self.pending.append(message)
        raise AssertionError("timed out waiting for a worker message")

    def request(self, op: str, **fields) -> dict:
        request_id = f"req_{uuid.uuid4().hex}"
        self.send({"id": request_id, "op": op, **fields})
        return self.read_until(lambda message: message.get("id") == request_id)

    def submit(self) -> dict:
        return self.request(
            "submit",
            invocation={
                "skill": "hold_pose",
                "parameters": {"duration_s": 2.0},
                "frame": "world",
                "timeout_s": 30.0,
                "expected_duration_s": 2.0,
            },
        )

    # -- tests ---------------------------------------------------------

    def test_state_and_capture_are_answered_while_the_skill_runs(self) -> None:
        submitted = self.submit()
        self.assertTrue(submitted["ok"], submitted)
        self.assertTrue(submitted["job_id"].startswith("job_"))

        started = time.perf_counter()
        observed = self.request("observe")
        state_latency_ms = (time.perf_counter() - started) * 1000.0
        self.assertTrue(observed["ok"], observed)
        self.assertEqual(observed["state"]["robots"][0]["robot_id"], "franka")
        self.assertLess(state_latency_ms, 500.0, f"observe blocked for {state_latency_ms:.0f} ms")

        captured = self.request("capture")
        self.assertTrue(captured["ok"], captured)
        self.assertIn("frame", captured)
        self.assertEqual(captured["frame"]["buffered"], True, "capture during a skill must serve the buffer")
        self.assertEqual(self.worker.fresh_captures, 0, "no render is allowed from the command loop")

        # A second submit is rejected instead of silently serializing.
        duplicate = self.submit()
        self.assertFalse(duplicate["ok"])
        self.assertEqual(duplicate["error"]["code"], "WORKER_BUSY")

        reset = self.request("reset")
        self.assertFalse(reset["ok"])
        self.assertEqual(reset["error"]["code"], "WORKER_BUSY")

    def test_cancel_is_acknowledged_within_a_second(self) -> None:
        submitted = self.submit()
        self.assertTrue(submitted["ok"])
        time.sleep(0.2)
        started = time.perf_counter()
        canceled = self.request("cancel", job_id=submitted["job_id"])
        latency_ms = (time.perf_counter() - started) * 1000.0
        self.assertTrue(canceled["ok"], canceled)
        self.assertTrue(canceled["canceled"])
        self.assertLess(latency_ms, 1000.0, f"cancel ACK took {latency_ms:.0f} ms")
        event = self.read_until(lambda message: message.get("event", {}).get("type") == "canceled")
        self.assertEqual(event["event"]["status"], "canceled")

    def test_skill_success_emits_terminal_event_and_idle_capture_is_fresh(self) -> None:
        submitted = self.submit()
        self.assertTrue(submitted["ok"])
        succeeded = self.read_until(
            lambda message: message.get("event", {}).get("type") in {"succeeded", "failed", "canceled"},
            timeout_s=10.0,
        )
        self.assertEqual(succeeded["event"]["type"], "succeeded")
        time.sleep(0.05)
        captured = self.request("capture")
        self.assertTrue(captured["ok"], captured)
        self.assertEqual(self.worker.fresh_captures, 1, "idle capture renders a fresh frame")


if __name__ == "__main__":
    unittest.main()
