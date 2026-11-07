"""SQLite persistence for simulations, jobs, events, tasks and idempotency."""

from __future__ import annotations

import json
import sqlite3
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class Store:
    def __init__(self, path: Path | str) -> None:
        self._lock = threading.RLock()
        self._conn = sqlite3.connect(str(path), check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        with self._lock:
            self._conn.execute("PRAGMA journal_mode=WAL")
            self._conn.execute("PRAGMA synchronous=NORMAL")
            self._init_schema()

    def _init_schema(self) -> None:
        self._conn.executescript(
            """
            CREATE TABLE IF NOT EXISTS jobs (
                job_id TEXT PRIMARY KEY,
                kind TEXT NOT NULL,
                status TEXT NOT NULL,
                simulation_id TEXT,
                task_id TEXT,
                skill TEXT,
                worker_job_id TEXT,
                parent_job_id TEXT,
                progress REAL,
                error_json TEXT,
                cancel_requested INTEGER NOT NULL DEFAULT 0,
                created_at TEXT NOT NULL,
                updated_at TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS events (
                seq INTEGER PRIMARY KEY AUTOINCREMENT,
                simulation_id TEXT,
                job_id TEXT NOT NULL,
                payload TEXT NOT NULL,
                created_at TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS idempotency (
                key TEXT PRIMARY KEY,
                job_id TEXT NOT NULL,
                created_at TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS simulations (
                simulation_id TEXT PRIMARY KEY,
                episode_id TEXT,
                state_json TEXT,
                created_at TEXT NOT NULL,
                closed INTEGER NOT NULL DEFAULT 0
            );
            CREATE TABLE IF NOT EXISTS tasks (
                task_id TEXT PRIMARY KEY,
                spec_json TEXT NOT NULL,
                plan_json TEXT,
                plan_hash TEXT,
                snapshot_json TEXT,
                created_at TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS frames (
                frame_id TEXT PRIMARY KEY,
                simulation_id TEXT NOT NULL,
                metadata_json TEXT NOT NULL,
                created_at TEXT NOT NULL
            );
            CREATE INDEX IF NOT EXISTS events_sim_seq ON events(simulation_id, seq);
            CREATE INDEX IF NOT EXISTS jobs_worker ON jobs(worker_job_id);
            """
        )
        # T07 migration for databases created before the snapshot column existed.
        try:
            self._conn.execute("ALTER TABLE tasks ADD COLUMN snapshot_json TEXT")
        except sqlite3.OperationalError:
            pass
        self._conn.commit()

    # -- jobs ----------------------------------------------------------

    def create_job(
        self,
        *,
        job_id: str,
        kind: str,
        simulation_id: str | None = None,
        task_id: str | None = None,
        skill: str | None = None,
        parent_job_id: str | None = None,
    ) -> dict[str, Any]:
        now = utc_now()
        with self._lock:
            self._conn.execute(
                "INSERT INTO jobs(job_id, kind, status, simulation_id, task_id, skill, parent_job_id, created_at, updated_at)"
                " VALUES(?,?,?,?,?,?,?,?,?)",
                (job_id, kind, "pending", simulation_id, task_id, skill, parent_job_id, now, now),
            )
            self._conn.commit()
        return self.get_job(job_id)  # type: ignore[return-value]

    def update_job(self, job_id: str, **fields: Any) -> None:
        if not fields:
            return
        fields["updated_at"] = utc_now()
        assignments = ", ".join(f"{name}=?" for name in fields)
        values = list(fields.values())
        with self._lock:
            self._conn.execute(f"UPDATE jobs SET {assignments} WHERE job_id=?", (*values, job_id))
            self._conn.commit()

    def get_job(self, job_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self._conn.execute("SELECT * FROM jobs WHERE job_id=?", (job_id,)).fetchone()
        return self._job_row(row) if row else None

    def get_job_by_worker_id(self, worker_job_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self._conn.execute(
                "SELECT * FROM jobs WHERE worker_job_id=? ORDER BY created_at DESC LIMIT 1", (worker_job_id,)
            ).fetchone()
        return self._job_row(row) if row else None

    def list_jobs(self, simulation_id: str | None = None) -> list[dict[str, Any]]:
        with self._lock:
            if simulation_id:
                rows = self._conn.execute(
                    "SELECT * FROM jobs WHERE simulation_id=? ORDER BY created_at", (simulation_id,)
                ).fetchall()
            else:
                rows = self._conn.execute("SELECT * FROM jobs ORDER BY created_at").fetchall()
        return [self._job_row(row) for row in rows]

    def _job_row(self, row: sqlite3.Row) -> dict[str, Any]:
        data = dict(row)
        if data.get("error_json"):
            data["error"] = json.loads(data["error_json"])
        data.pop("error_json", None)
        data["cancel_requested"] = bool(data.get("cancel_requested"))
        return data

    def mark_live_jobs_lost(self) -> int:
        """A Gateway restart invalidates every live job; never replay motion."""

        now = utc_now()
        error = json.dumps({"code": "WORKER_LOST", "message": "gateway restarted; job not resumed"})
        with self._lock:
            cursor = self._conn.execute(
                "UPDATE jobs SET status='failed', error_json=?, updated_at=? WHERE status IN ('pending','running','canceling')",
                (error, now),
            )
            self._conn.commit()
        return cursor.rowcount

    # -- events --------------------------------------------------------

    def append_event(self, *, simulation_id: str | None, job_id: str, payload: dict[str, Any]) -> int:
        with self._lock:
            cursor = self._conn.execute(
                "INSERT INTO events(simulation_id, job_id, payload, created_at) VALUES(?,?,?,?)",
                (simulation_id, job_id, json.dumps(payload, ensure_ascii=False), utc_now()),
            )
            self._conn.commit()
        return int(cursor.lastrowid)

    def events_after(self, *, simulation_id: str | None, after_seq: int, limit: int = 500) -> list[dict[str, Any]]:
        with self._lock:
            if simulation_id:
                rows = self._conn.execute(
                    "SELECT seq, simulation_id, job_id, payload, created_at FROM events"
                    " WHERE simulation_id=? AND seq>? ORDER BY seq LIMIT ?",
                    (simulation_id, after_seq, limit),
                ).fetchall()
            else:
                rows = self._conn.execute(
                    "SELECT seq, simulation_id, job_id, payload, created_at FROM events"
                    " WHERE seq>? ORDER BY seq LIMIT ?",
                    (after_seq, limit),
                ).fetchall()
        return [
            {
                "seq": row["seq"],
                "simulation_id": row["simulation_id"],
                "job_id": row["job_id"],
                "event": json.loads(row["payload"]),
                "created_at": row["created_at"],
            }
            for row in rows
        ]

    def events_for_job(self, *, job_id: str, after_seq: int = 0, limit: int = 200) -> list[dict[str, Any]]:
        """One job's event timeline; ``after_seq`` paginates without gaps or repeats."""
        with self._lock:
            rows = self._conn.execute(
                "SELECT seq, simulation_id, job_id, payload, created_at FROM events"
                " WHERE job_id=? AND seq>? ORDER BY seq LIMIT ?",
                (job_id, after_seq, limit),
            ).fetchall()
        return [
            {
                "seq": row["seq"],
                "simulation_id": row["simulation_id"],
                "job_id": row["job_id"],
                "event": json.loads(row["payload"]),
                "created_at": row["created_at"],
            }
            for row in rows
        ]

    # -- idempotency ---------------------------------------------------

    def get_idempotent(self, key: str) -> str | None:
        with self._lock:
            row = self._conn.execute("SELECT job_id FROM idempotency WHERE key=?", (key,)).fetchone()
        return row["job_id"] if row else None

    def put_idempotent(self, key: str, job_id: str) -> None:
        with self._lock:
            self._conn.execute(
                "INSERT OR IGNORE INTO idempotency(key, job_id, created_at) VALUES(?,?,?)",
                (key, job_id, utc_now()),
            )
            self._conn.commit()

    # -- simulations ---------------------------------------------------

    def save_simulation(self, *, simulation_id: str, episode_id: str | None, state: dict[str, Any] | None) -> None:
        with self._lock:
            self._conn.execute(
                "INSERT INTO simulations(simulation_id, episode_id, state_json, created_at) VALUES(?,?,?,?)"
                " ON CONFLICT(simulation_id) DO UPDATE SET episode_id=excluded.episode_id, state_json=excluded.state_json, closed=0",
                (simulation_id, episode_id, json.dumps(state, ensure_ascii=False) if state else None, utc_now()),
            )
            self._conn.commit()

    def get_simulation(self, simulation_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self._conn.execute("SELECT * FROM simulations WHERE simulation_id=?", (simulation_id,)).fetchone()
        if not row:
            return None
        return {
            "simulation_id": row["simulation_id"],
            "episode_id": row["episode_id"],
            "state": json.loads(row["state_json"]) if row["state_json"] else None,
            "closed": bool(row["closed"]),
            "created_at": row["created_at"],
        }

    def close_simulation(self, simulation_id: str) -> None:
        with self._lock:
            self._conn.execute("UPDATE simulations SET closed=1 WHERE simulation_id=?", (simulation_id,))
            self._conn.commit()

    def list_simulations(self, limit: int = 20) -> list[dict[str, Any]]:
        """Most recent simulations first, for the operator console."""
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM simulations ORDER BY created_at DESC, simulation_id DESC LIMIT ?",
                (limit,),
            ).fetchall()
        return [
            {
                "simulation_id": row["simulation_id"],
                "episode_id": row["episode_id"],
                "closed": bool(row["closed"]),
                "created_at": row["created_at"],
            }
            for row in rows
        ]

    # -- tasks ---------------------------------------------------------

    def save_task(self, *, task_id: str, spec: dict[str, Any]) -> None:
        with self._lock:
            self._conn.execute(
                "INSERT OR REPLACE INTO tasks(task_id, spec_json, created_at) VALUES(?,?,?)",
                (task_id, json.dumps(spec, ensure_ascii=False), utc_now()),
            )
            self._conn.commit()

    def get_task(self, task_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self._conn.execute("SELECT * FROM tasks WHERE task_id=?", (task_id,)).fetchone()
        if not row:
            return None
        return {
            "task_id": row["task_id"],
            "spec": json.loads(row["spec_json"]),
            "plan": json.loads(row["plan_json"]) if row["plan_json"] else None,
            "plan_hash": row["plan_hash"],
            "snapshot": json.loads(row["snapshot_json"]) if row["snapshot_json"] else None,
            "created_at": row["created_at"],
        }

    def save_plan(
        self, *, task_id: str, plan: dict[str, Any], plan_hash: str, snapshot: dict[str, Any] | None = None
    ) -> None:
        with self._lock:
            self._conn.execute(
                "UPDATE tasks SET plan_json=?, plan_hash=?, snapshot_json=? WHERE task_id=?",
                (
                    json.dumps(plan, ensure_ascii=False),
                    plan_hash,
                    json.dumps(snapshot, ensure_ascii=False) if snapshot is not None else None,
                    task_id,
                ),
            )
            self._conn.commit()

    # -- frames --------------------------------------------------------

    def save_frame(self, *, frame_id: str, simulation_id: str, metadata: dict[str, Any]) -> None:
        with self._lock:
            self._conn.execute(
                "INSERT OR REPLACE INTO frames(frame_id, simulation_id, metadata_json, created_at) VALUES(?,?,?,?)",
                (frame_id, simulation_id, json.dumps(metadata, ensure_ascii=False), utc_now()),
            )
            self._conn.commit()

    def get_frame(self, frame_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self._conn.execute("SELECT * FROM frames WHERE frame_id=?", (frame_id,)).fetchone()
        if not row:
            return None
        return {
            "frame_id": row["frame_id"],
            "simulation_id": row["simulation_id"],
            "metadata": json.loads(row["metadata_json"]),
            "created_at": row["created_at"],
        }

    def list_frames(self, simulation_id: str, limit: int = 20) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT frame_id, created_at FROM frames WHERE simulation_id=? ORDER BY created_at DESC LIMIT ?",
                (simulation_id, limit),
            ).fetchall()
        return [{"frame_id": row["frame_id"], "created_at": row["created_at"]} for row in rows]

    def close(self) -> None:
        with self._lock:
            self._conn.close()

