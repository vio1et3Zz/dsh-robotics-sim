"""Read one member file out of an Electron asar archive.

The DSH desktop build ships its runtime inside ``app.asar``.  Windows tools
cannot treat that path as a directory, so the doctor script uses this helper to
read package metadata (versions, dependency pins) from the *installed* archive
instead of trusting a hand-written table.

Usage:
    python read_asar_package.py <app.asar> <member/path.json>

The member path uses forward slashes relative to the archive root, e.g.
``dsh/package.json``.
"""

from __future__ import annotations

import json
import struct
import sys


def read_member(archive: str, member: str) -> bytes:
    with open(archive, "rb") as handle:
        prefix = handle.read(16)
        header_size = struct.unpack("<I", prefix[4:8])[0]
        json_size = struct.unpack("<I", prefix[12:16])[0]
        raw = handle.read(json_size)

    # Layout: [4][header_size][header_string_size][json_size][JSON][padding],
    # with file offsets relative to 8 + header_size.
    header = json.loads(raw.decode("utf-8"))
    base = 8 + header_size

    node = header
    for part in member.split("/"):
        node = node["files"][part]
    if "files" in node:
        raise SystemExit(f"member is a directory: {member}")

    offset = int(node["offset"])
    size = int(node["size"])
    with open(archive, "rb") as handle:
        handle.seek(base + offset)
        return handle.read(size)


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(__doc__.strip().splitlines()[0], file=sys.stderr)
        return 2
    sys.stdout.buffer.write(read_member(argv[1], argv[2]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
