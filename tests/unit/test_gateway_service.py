"""Gateway job/cancel/idempotency tests against the loopback fake worker."""

from __future__ import annotations

import shutil
import sys
import unittest
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

from robotics_gateway import GatewayError, GatewayService, LoopbackEndpoint, Store  # noqa: E402
from robotics_protocol import JobStatus, SimulationSpec, SkillInvocation  # noqa: E402


def mock_spec() -> SimulationSpec:
    return SimulationSpec.model_validate(
        {
            "profile_id": "profile_mock_franka",
            "backend_kind": "mock",
            "robot_id": "franka",
            "scene_id": "table_top",
            "physics": {"engine": "physx", "dt": 0.005, "decimation": 4, "solver_iterations": 8},
            "renderer": {"enabled": False, "headless": True},
            "seed": 3,
            "num_envs": 1,
            "budget": {"max_sim_time_s": 60.0, "max_wall_time_s": 120.0},
        }
    )


def pick() -> SkillInvocation:
    return SkillInvocation.model_validate(
        {
            "skill": "pick",
            "parameters": {},
            "target_object_id": "red_block",
            "frame": "world",
            "timeout_s": 30.0,
            "expected_duration_s": 10.0,
        }
    )


class GatewayServiceTests(unittest.TestCase):
    def setUp(self) -> None:
        self._dir = ROOT / "artifacts" / "_tests" / f"gw_{uuid.uuid4().hex}"
        self._dir.mkdir(parents=True)
        self.store = Store(self._dir / "gateway.db")
        self.endpoint = LoopbackEndpoint(tick_s=0.02)
        self.service = GatewayService(self.store, self.endpoint)
        start = self.service.start_simulation(mock_spec(), None)
        start = self.service.wait_job(start["job_id"], timeout_s=5.0)
        self.assertEqual(start["status"], JobStatus.SUCCEEDED.value)
        self.simulation_id = self.endpoint.observe().simulation_id

    def tearDown(self) -> None:
        self.service.close()
        shutil.rmtree(self._dir, ignore_errors=True)

    def test_start_job_succeeds_and_state_is_readable(self) -> None:
        state = self.service.get_state(self.simulation_id)
        self.assertEqual(state.robots[0].dof, len(state.robots[0].joint_positions))
        self.assertEqual(state.episode_id[:3], "ep_")

    def test_duplicate_execute_returns_same_job(self) -> None:
        first = self.service.submit_skill(pick(), simulation_id=self.simulation_id, idempotency_key="dup-1")
        second = self.service.submit_skill(pick(), simulation_id=self.simulation_id, idempotency_key="dup-1")
        self.assertEqual(first["job_id"], second["job_id"])
        self.assertEqual(len([job for job in self.store.list_jobs() if job["kind"] == "skill"]), 1)

    def test_cancel_only_settles_after_worker_event(self) -> None:
        job = self.service.submit_skill(pick(), simulation_id=self.simulation_id)
        canceled = self.service.cancel_job(job["job_id"])
        self.assertIn(canceled["status"], {JobStatus.CANCELING.value, JobStatus.CANCELED.value})
        settled = self.service.wait_job(job["job_id"], timeout_s=5.0)
        self.assertEqual(settled["status"], JobStatus.CANCELED.value)
        events = self.store.events_after(simulation_id=self.simulation_id, after_seq=0)
        self.assertTrue(any(record["event"]["type"] == "canceled" for record in events))

    def test_reset_is_rejected_while_a_job_is_live(self) -> None:
        self.service.submit_skill(pick(), simulation_id=self.simulation_id)
        with self.assertRaises(GatewayError) as caught:
            self.service.reset(self.simulation_id)
        self.assertEqual(caught.exception.code, "CONTROL_BUSY")
        self.service.wait_job(self.store.list_jobs()[-1]["job_id"], timeout_s=5.0)

    def test_stale_episode_is_rejected(self) -> None:
        state = self.service.get_state(self.simulation_id)
        self.service.reset(self.simulation_id)
        with self.assertRaises(GatewayError) as caught:
            self.service.submit_skill(
                pick(), simulation_id=self.simulation_id, expected_episode=state.episode_id
            )
        self.assertEqual(caught.exception.code, "STALE_WORLD_STATE")

    def test_worker_loss_fails_live_job(self) -> None:
        job = self.service.submit_skill(pick(), simulation_id=self.simulation_id)
        self.endpoint.alive = lambda: False
        settled = self.service.wait_job(job["job_id"], timeout_s=5.0)
        self.assertEqual(settled["status"], JobStatus.FAILED.value)
        self.assertEqual(settled["error"]["code"], "WORKER_LOST")

    def test_restart_marks_live_jobs_failed(self) -> None:
        job = self.store.create_job(job_id="job_00000000000000000000000000000000", kind="skill", simulation_id=self.simulation_id)
        self.store.update_job(job["job_id"], status=JobStatus.RUNNING.value)
        self.assertGreaterEqual(self.store.mark_live_jobs_lost(), 1)
        self.assertEqual(self.store.get_job(job["job_id"])["status"], JobStatus.FAILED.value)


if __name__ == "__main__":
    unittest.main()
