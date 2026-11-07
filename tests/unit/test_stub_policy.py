"""C1 tests: scripted policies behind the Policy protocol (no Isaac needed)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

from robotics_backends.isaaclab_physx.stub_policy import (  # noqa: E402
    Policy,
    PolicyFailure,
    StubDeltaPolicy,
    StubWaypointPolicy,
)

START = [0.0] * 6
GOAL = [0.5] * 6


def obs(joints: list[float]) -> dict:
    return {"joints": joints, "ee": [0.0, 0.0, 0.0], "sim_step": 0}


class StubWaypointPolicyTests(unittest.TestCase):
    def test_chunks_interpolate_and_finish(self) -> None:
        policy = StubWaypointPolicy([START, GOAL], chunk_steps=5, steps_per_waypoint=100)
        self.assertIsInstance(policy, Policy)
        action = policy.act(obs(START))
        self.assertEqual(action["mode"], "joint")
        self.assertFalse(action["done"])
        self.assertEqual(len(action["targets"]), 5)
        self.assertLess(action["targets"][0][0], action["targets"][-1][0])
        self.assertGreater(action["targets"][-1][0], 0.0)

        joints = START
        done = False
        for _ in range(20):
            action = policy.act(obs(joints))
            if action["done"]:
                done = True
                break
            joints = action["targets"][-1]
        self.assertTrue(done, "policy never reached its final waypoint")
        self.assertAlmostEqual(policy.progress(), 1.0)

    def test_reset_restarts_the_sequence(self) -> None:
        policy = StubWaypointPolicy([START, GOAL], chunk_steps=5, steps_per_waypoint=10)
        policy.act(obs(START))
        policy.reset()
        self.assertEqual(policy.progress(), 0.0)
        action = policy.act(obs(START))
        self.assertFalse(action["done"])

    def test_no_progress_mode_holds_still(self) -> None:
        policy = StubWaypointPolicy([START, GOAL], chunk_steps=3, steps_per_waypoint=100, fail="no_progress")
        action = policy.act(obs([0.1] * 6))
        self.assertFalse(action["done"])
        for target in action["targets"]:
            self.assertEqual(target, [0.1] * 6)

    def test_timeout_mode_never_finishes(self) -> None:
        policy = StubWaypointPolicy([START, GOAL], chunk_steps=3, steps_per_waypoint=100, fail="timeout")
        action = policy.act(obs(START))
        self.assertFalse(action["done"])
        first = action["targets"][0][0]
        action = policy.act(obs(START))
        self.assertFalse(action["done"])
        self.assertNotEqual(first, action["targets"][0][0], "timeout stub should keep moving")


class StubDeltaPolicyTests(unittest.TestCase):
    def test_repeats_then_finishes(self) -> None:
        policy = StubDeltaPolicy([[0.0, 0.0, -0.01]], repeat=2)
        self.assertFalse(policy.act(obs(START))["done"])
        self.assertFalse(policy.act(obs(START))["done"])
        done = policy.act(obs(START))
        self.assertTrue(done["done"])
        self.assertEqual(policy.progress(), 1.0)

    def test_reset(self) -> None:
        policy = StubDeltaPolicy([[0.0, 0.0, -0.01]], repeat=1)
        policy.act(obs(START))
        policy.reset()
        self.assertEqual(policy.progress(), 0.0)


class PolicyFailureTests(unittest.TestCase):
    def test_code_and_message_are_preserved(self) -> None:
        failure = PolicyFailure("POLICY_TIMEOUT", "exceeded 5s", {"chunks": 3})
        self.assertEqual(failure.code, "POLICY_TIMEOUT")
        self.assertEqual(failure.message, "exceeded 5s")
        self.assertEqual(failure.detail["chunks"], 3)
        self.assertIn("POLICY_TIMEOUT", str(failure))


if __name__ == "__main__":
    unittest.main()
