"""Fake-backend tests: protocol behavior only, never simulation acceptance."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

from robotics_backends import FakeBackend, PROFILES  # noqa: E402
from robotics_backends.profiles import violation, validate_spec  # noqa: E402
from robotics_protocol import (  # noqa: E402
    BackendKind,
    ErrorCode,
    JobEventType,
    JobStatus,
    ProtocolViolation,
    SimulationSpec,
    SkillInvocation,
)


def mock_spec(**overrides) -> SimulationSpec:
    payload = {
        "profile_id": "profile_mock_franka",
        "backend_kind": "mock",
        "robot_id": "franka",
        "scene_id": "table_top",
        "physics": {"engine": "physx", "dt": 0.005, "decimation": 4, "solver_iterations": 8},
        "renderer": {"enabled": False, "headless": True},
        "seed": 1,
        "num_envs": 1,
        "budget": {"max_sim_time_s": 60.0, "max_wall_time_s": 120.0},
    }
    payload.update(overrides)
    return SimulationSpec.model_validate(payload)


def invocation(skill: str = "pick", target: str | None = "red_block") -> SkillInvocation:
    return SkillInvocation.model_validate(
        {
            "skill": skill,
            "parameters": {},
            "target_object_id": target,
            "frame": "world",
            "timeout_s": 30.0,
            "expected_duration_s": 10.0,
        }
    )


class FakeBackendTests(unittest.TestCase):
    def test_capabilities_are_mock_and_unverified(self) -> None:
        capabilities = FakeBackend().describe_capabilities()
        self.assertIs(capabilities.backend_kind, BackendKind.MOCK)
        self.assertTrue(capabilities.installed)
        self.assertFalse(capabilities.verified)
        self.assertEqual(capabilities.combinations[0].status.value, "installed")

    def test_start_returns_real_length_joint_arrays(self) -> None:
        backend = FakeBackend()
        state = backend.start(mock_spec())
        self.assertEqual(len(state.robots[0].joint_positions), state.robots[0].dof)
        self.assertEqual(state.episode_id[:3], "ep_")

    def test_profile_is_fixed_to_robot_and_scene(self) -> None:
        with self.assertRaises(ProtocolViolation) as caught:
            validate_spec(mock_spec(robot_id="so101"))
        self.assertIs(caught.exception.info.code, ErrorCode.UNSUPPORTED_CAPABILITY)

    def test_profile_limits_environment_count(self) -> None:
        with self.assertRaises(ProtocolViolation):
            validate_spec(mock_spec(num_envs=2))

    def test_fake_refuses_a_physx_profile(self) -> None:
        backend = FakeBackend()
        spec = mock_spec(profile_id="profile_dev_franka_physx", backend_kind="isaaclab_physx")
        with self.assertRaises(ProtocolViolation) as caught:
            backend.start(spec)
        self.assertIs(caught.exception.info.code, ErrorCode.UNSUPPORTED_BACKEND)

    def test_unknown_skill_is_rejected(self) -> None:
        backend = FakeBackend()
        backend.start(mock_spec())
        with self.assertRaises(ProtocolViolation) as caught:
            backend.submit_skill(invocation(skill="fly"))
        self.assertIs(caught.exception.info.code, ErrorCode.UNSUPPORTED_CAPABILITY)

    def test_unknown_target_object_is_rejected(self) -> None:
        backend = FakeBackend()
        backend.start(mock_spec())
        with self.assertRaises(ProtocolViolation) as caught:
            backend.submit_skill(invocation(target="ghost_block"))
        self.assertIs(caught.exception.info.code, ErrorCode.INVALID_REQUEST)

    def test_control_busy_while_a_skill_runs(self) -> None:
        backend = FakeBackend()
        backend.start(mock_spec())
        backend.submit_skill(invocation())
        with self.assertRaises(ProtocolViolation) as caught:
            backend.submit_skill(invocation(skill="place", target="target_box"))
        self.assertIs(caught.exception.info.code, ErrorCode.CONTROL_BUSY)

    def test_cancel_requires_worker_acknowledgement(self) -> None:
        backend = FakeBackend()
        backend.start(mock_spec())
        job_id = backend.submit_skill(invocation())
        backend.drain_events()
        self.assertTrue(backend.cancel_skill(job_id))
        events = backend.drain_events()
        self.assertEqual([event.type for event in events], [JobEventType.CANCEL_REQUESTED, JobEventType.CANCELED])
        self.assertIs(events[-1].status, JobStatus.CANCELED)
        self.assertFalse(backend.cancel_skill(job_id))

    def test_completed_skill_updates_physics_facing_state(self) -> None:
        backend = FakeBackend()
        backend.start(mock_spec())
        job_id = backend.submit_skill(invocation())
        backend.drain_events()
        for _ in range(3):
            backend.tick()
        events = backend.drain_events()
        self.assertIs(events[-1].type, JobEventType.SUCCEEDED)
        self.assertEqual(events[-1].job_id, job_id)
        state = backend.observe()
        grasped = {obj.object_id: obj.grasped_by for obj in state.objects}
        self.assertEqual(grasped["red_block"], "franka")

    def test_closed_backend_fails_fast(self) -> None:
        backend = FakeBackend()
        backend.start(mock_spec())
        backend.close()
        with self.assertRaises(RuntimeError):
            backend.observe()

    def test_profiles_are_an_allowlist(self) -> None:
        self.assertIn("profile_mock_franka", PROFILES)
        with self.assertRaises(ProtocolViolation):
            backend_spec = mock_spec(profile_id="profile_does_not_exist")
            validate_spec(backend_spec)


if __name__ == "__main__":
    unittest.main()
