"""Export one Gateway run's jobs, events and simulation record for evidence."""

from __future__ import annotations

import argparse
import json
import sqlite3
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--db", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    connection = sqlite3.connect(str(args.db))
    connection.row_factory = sqlite3.Row
    jobs = [dict(row) for row in connection.execute("SELECT * FROM jobs ORDER BY created_at")]
    events = []
    for row in connection.execute("SELECT * FROM events ORDER BY seq"):
        record = dict(row)
        record["event"] = json.loads(record.pop("payload"))
        events.append(record)
    simulations = [
        {
            "simulation_id": row["simulation_id"],
            "episode_id": row["episode_id"],
            "closed": bool(row["closed"]),
            "created_at": row["created_at"],
        }
        for row in connection.execute("SELECT * FROM simulations")
    ]
    connection.close()

    summary = {
        "db": str(args.db),
        "jobs": jobs,
        "events": events,
        "simulations": simulations,
        "counts": {"jobs": len(jobs), "events": len(events), "simulations": len(simulations)},
    }
    args.out.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(summary["counts"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
