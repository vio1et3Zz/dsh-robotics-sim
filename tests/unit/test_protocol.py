"""Shared-fixture validation and plan-hash integrity tests."""

from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

from pydantic import ValidationError  # noqa: E402

from robotics_protocol import MODELS, compute_plan_hash  # noqa: E402
from robotics_protocol.tasks import PlanStep  # noqa: E402

CASES_PATH = ROOT / "tests" / "fixtures" / "protocol" / "cases.json"


class ProtocolFixtureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.document = json.loads(CASES_PATH.read_text(encoding="utf-8"))
        cls.cases = cls.document["cases"]

    def test_cases_cover_every_model(self) -> None:
        covered = {case["model"] for case in self.cases if case["valid"]}
        self.assertEqual(covered, set(MODELS))

    def test_shared_fixtures_validate_as_declared(self) -> None:
        for case in self.cases:
            with self.subTest(case=case["name"]):
                model = MODELS[case["model"]]
                if case["valid"]:
                    model.model_validate(case["payload"])
                else:
                    with self.assertRaises(ValidationError):
                        model.model_validate(case["payload"])

    def test_plan_hash_is_content_addressed(self) -> None:
        payload = next(case["payload"] for case in self.cases if case["name"] == "valid_taskplan")
        steps = [PlanStep.model_validate(step) for step in payload["steps"]]
        expected = compute_plan_hash(
            task_id=payload["task_id"],
            episode_id=payload["episode_id"],
            base_state_version=payload["base_state_version"],
            steps=steps,
        )
        self.assertEqual(payload["plan_hash"], expected)

        changed = copy.deepcopy(payload)
        changed["steps"][1]["expected_duration_s"] += 1.0
        changed_steps = [PlanStep.model_validate(step) for step in changed["steps"]]
        self.assertNotEqual(
            expected,
            compute_plan_hash(
                task_id=changed["task_id"],
                episode_id=changed["episode_id"],
                base_state_version=changed["base_state_version"],
                steps=changed_steps,
            ),
        )

    def test_plan_hash_mismatch_is_rejected(self) -> None:
        payload = next(case["payload"] for case in self.cases if case["name"] == "valid_taskplan")
        changed = copy.deepcopy(payload)
        changed["plan_hash"] = "0" * 64
        with self.assertRaises(ValidationError):
            MODELS["TaskPlan"].model_validate(changed)

    def test_fixture_count(self) -> None:
        self.assertGreaterEqual(len(self.cases), 20)


if __name__ == "__main__":
    unittest.main()
