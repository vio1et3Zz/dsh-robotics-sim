"""HTTP/WS Gateway integration tests against the loopback fake worker."""

from __future__ import annotations

import asyncio
import json
import shutil
import socket
import sys
import threading
import time
import unittest
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

import requests  # noqa: E402
import uvicorn  # noqa: E402
import websockets  # noqa: E402

from robotics_gateway import GatewayService, LoopbackEndpoint, Store, create_app  # noqa: E402
from robotics_protocol import SimulationSpec, SkillInvocation  # noqa: E402


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def mock_spec() -> dict:
    return {
        "profile_id": "profile_mock_franka",
        "backend_kind": "mock",
        "robot_id": "franka",
        "scene_id": "table_top",
        "physics": {"engine": "physx", "dt": 0.005, "decimation": 4, "solver_iterations": 8},
        "renderer": {"enabled": False, "headless": True},
        "seed": 5,
        "num_envs": 1,
        "budget": {"max_sim_time_s": 60.0, "max_wall_time_s": 120.0},
    }


def pick() -> dict:
    return {
        "skill": "pick",
        "parameters": {},
        "target_object_id": "red_block",
        "frame": "world",
        "timeout_s": 30.0,
        "expected_duration_s": 10.0,
    }


class GatewayHTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.directory = ROOT / "artifacts" / "_tests" / f"http_{uuid.uuid4().hex}"
        cls.directory.mkdir(parents=True)
        cls.store = Store(cls.directory / "gateway.db")
        cls.endpoint = LoopbackEndpoint(tick_s=0.05)
        cls.service = GatewayService(cls.store, cls.endpoint)
        cls.port = free_port()
        cls.server = uvicorn.Server(
            uvicorn.Config(create_app(cls.service), host="127.0.0.1", port=cls.port, log_level="error")
        )
        cls.thread = threading.Thread(target=cls.server.run, daemon=True)
        cls.thread.start()
        deadline = time.time() + 10.0
        while not cls.server.started and time.time() < deadline:
            time.sleep(0.02)
        if not cls.server.started:
            raise RuntimeError("uvicorn did not start")
        cls.base = f"http://127.0.0.1:{cls.port}"

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.should_exit = True
        cls.thread.join(timeout=5.0)
        cls.service.close()
        shutil.rmtree(cls.directory, ignore_errors=True)

    def _wait(self, job_id: str, timeout: float = 10.0) -> dict:
        deadline = time.time() + timeout
        while time.time() < deadline:
            response = requests.get(f"{self.base}/api/v1/jobs/{job_id}", timeout=5)
            self.assertEqual(response.status_code, 200, response.text)
            job = response.json()
            if job["status"] in {"succeeded", "failed", "canceled"}:
                return job
            time.sleep(0.03)
        self.fail(f"job {job_id} did not settle")

    def test_health(self) -> None:
        response = requests.get(f"{self.base}/api/v1/health", timeout=5)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["gateway"], "ok")

    def test_full_start_skill_cancel_flow(self) -> None:
        response = requests.post(
            f"{self.base}/api/v1/simulations",
            json={"spec": mock_spec(), "idempotency_key": "http-start-1"},
            timeout=5,
        )
        self.assertEqual(response.status_code, 202, response.text)
        start_job = response.json()
        self.assertEqual(self._wait(start_job["job_id"])["status"], "succeeded")

        state_response = requests.get(f"{self.base}/api/v1/simulations/{self.endpoint.observe().simulation_id}/state", timeout=5)
        self.assertEqual(state_response.status_code, 200, state_response.text)
        simulation_id = state_response.json()["simulation_id"]

        submitted = requests.post(
            f"{self.base}/api/v1/simulations/{simulation_id}/skills",
            json={"invocation": pick(), "idempotency_key": "http-pick-1"},
            timeout=5,
        )
        self.assertEqual(submitted.status_code, 202, submitted.text)
        job = submitted.json()
        duplicate = requests.post(
            f"{self.base}/api/v1/simulations/{simulation_id}/skills",
            json={"invocation": pick(), "idempotency_key": "http-pick-1"},
            timeout=5,
        )
        self.assertEqual(duplicate.status_code, 202)
        self.assertEqual(duplicate.json()["job_id"], job["job_id"])

        canceled = requests.post(f"{self.base}/api/v1/jobs/{job['job_id']}/cancel", timeout=5)
        self.assertEqual(canceled.status_code, 202, canceled.text)
        settled = self._wait(job["job_id"])
        self.assertEqual(settled["status"], "canceled")

    def test_websocket_events_are_replayable(self) -> None:
        simulation_id = self.endpoint.observe().simulation_id
        uri = f"ws://127.0.0.1:{self.port}/api/v1/events?simulation_id={simulation_id}&after_seq=0"

        async def first_event() -> dict:
            async with websockets.connect(uri) as websocket:
                return json.loads(await asyncio.wait_for(websocket.recv(), timeout=5.0))

        event = asyncio.run(first_event())
        self.assertIn("seq", event)
        self.assertIn("event", event)
        self.assertEqual(event["simulation_id"], simulation_id)

    def test_console_assets_are_served(self) -> None:
        page = requests.get(f"{self.base}/ui", timeout=5)
        self.assertEqual(page.status_code, 200)
        self.assertIn("DSH Robotics", page.text)
        self.assertIn("/ui/assets/app.js", page.text)

        script = requests.get(f"{self.base}/ui/assets/app.js", timeout=5)
        self.assertEqual(script.status_code, 200)
        self.assertIn("function refreshState", script.text)

        styles = requests.get(f"{self.base}/ui/assets/style.css", timeout=5)
        self.assertEqual(styles.status_code, 200)
        self.assertIn("--accent", styles.text)

        redirect = requests.get(f"{self.base}/", timeout=5, allow_redirects=False)
        self.assertEqual(redirect.status_code, 307)
        self.assertEqual(redirect.headers["location"], "/ui")

    def test_job_events_endpoint_paginates(self) -> None:
        simulation_id = self.endpoint.observe().simulation_id
        response = requests.post(
            f"{self.base}/api/v1/simulations/{simulation_id}/skills",
            json={"invocation": pick()},
            timeout=5,
        )
        self.assertEqual(response.status_code, 202, response.text)
        job_id = response.json()["job_id"]
        self._wait(job_id)

        first = requests.get(f"{self.base}/api/v1/jobs/{job_id}/events?after_seq=0&limit=3", timeout=5)
        self.assertEqual(first.status_code, 200, first.text)
        page = first.json()
        self.assertEqual(page["job_id"], job_id)
        self.assertTrue(page["events"])
        seqs = [record["seq"] for record in page["events"]]
        self.assertEqual(seqs, sorted(seqs))
        self.assertEqual(page["next_seq"], seqs[-1])

        second = requests.get(
            f"{self.base}/api/v1/jobs/{job_id}/events?after_seq={page['next_seq']}&limit=50", timeout=5
        )
        self.assertEqual(second.status_code, 200)
        tail = [record["seq"] for record in second.json()["events"]]
        self.assertTrue(all(seq > page["next_seq"] for seq in tail))
        self.assertFalse(set(tail) & set(seqs), "after_seq must not repeat records")

        missing = requests.get(f"{self.base}/api/v1/jobs/job_00000000/events", timeout=5)
        self.assertEqual(missing.status_code, 404)
        self.assertEqual(missing.json()["error"]["code"], "UNKNOWN_JOB")

    def test_latest_frame_endpoint(self) -> None:
        simulation_id = self.endpoint.observe().simulation_id
        response = requests.get(f"{self.base}/api/v1/simulations/{simulation_id}/frames/latest", timeout=5)
        self.assertEqual(response.status_code, 404)
        self.assertEqual(response.json()["error"]["code"], "UNKNOWN_FRAME")

        unknown = requests.get(
            f"{self.base}/api/v1/simulations/sim_00000000/frames/latest", timeout=5
        )
        self.assertEqual(unknown.status_code, 404)
        self.assertEqual(unknown.json()["error"]["code"], "UNKNOWN_SIMULATION")

    def test_list_endpoints(self) -> None:
        simulations = requests.get(f"{self.base}/api/v1/simulations", timeout=5)
        self.assertEqual(simulations.status_code, 200)
        items = simulations.json()["simulations"]
        if not items:
            response = requests.post(
                f"{self.base}/api/v1/simulations",
                json={"spec": mock_spec(), "idempotency_key": "http-list-1"},
                timeout=5,
            )
            self.assertEqual(response.status_code, 202, response.text)
            self.assertEqual(self._wait(response.json()["job_id"])["status"], "succeeded")
            items = requests.get(f"{self.base}/api/v1/simulations", timeout=5).json()["simulations"]
        self.assertTrue(items)
        for item in items:
            self.assertIn("simulation_id", item)
            self.assertIn("closed", item)
        simulation_id = items[0]["simulation_id"]

        jobs = requests.get(f"{self.base}/api/v1/jobs?limit=10", timeout=5)
        self.assertEqual(jobs.status_code, 200)
        self.assertTrue(jobs.json()["jobs"])
        self.assertIn("status", jobs.json()["jobs"][0])

        frames = requests.get(f"{self.base}/api/v1/simulations/{simulation_id}/frames", timeout=5)
        self.assertEqual(frames.status_code, 200, frames.text)
        self.assertEqual(frames.json()["frames"], [])

        missing = requests.get(
            f"{self.base}/api/v1/simulations/sim_00000000/frames", timeout=5
        )
        self.assertEqual(missing.status_code, 404)
        self.assertEqual(missing.json()["error"]["code"], "UNKNOWN_SIMULATION")

        unsupported = requests.post(
            f"{self.base}/api/v1/simulations/{simulation_id}/frames", timeout=5
        )
        self.assertEqual(unsupported.status_code, 400)
        self.assertEqual(unsupported.json()["error"]["code"], "UNSUPPORTED_CAPABILITY")


if __name__ == "__main__":
    unittest.main()
