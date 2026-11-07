"""Run the protocol schema exporter with the repository's python/ on sys.path."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from robotics_protocol.export_schema import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
