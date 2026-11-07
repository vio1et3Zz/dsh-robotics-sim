"""Print installed Isaac/Python package metadata as one JSON object.

Called by doctor.ps1.  Kept as a file instead of a ``python -c`` string because
Windows PowerShell 5.1 strips embedded double quotes when building a native
command line, which corrupts inline JSON/Python snippets.
"""

from __future__ import annotations

import importlib.metadata as metadata
import json
import platform

NAMES = [
    "isaaclab",
    "isaacsim",
    "isaaclab-physx",
    "isaaclab-newton",
    "newton",
    "torch",
    "gymnasium",
    "fastapi",
    "pydantic",
    "websockets",
]


def main() -> None:
    packages = {}
    for name in NAMES:
        try:
            packages[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            packages[name] = None
    print(
        json.dumps(
            {
                "python": platform.python_version(),
                "platform": platform.platform(),
                "packages": packages,
            }
        )
    )


if __name__ == "__main__":
    main()
