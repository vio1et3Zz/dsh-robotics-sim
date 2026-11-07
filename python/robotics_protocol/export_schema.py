"""Export the Pydantic protocol to a versioned JSON Schema bundle and TS types.

The Pydantic models are authoritative.  The exporter is deterministic: rerun it
after any model change and fail a review if the generated files drift.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from pydantic.json_schema import models_json_schema

from . import MODELS, PROTOCOL_VERSION

PROTOCOL_DIR = Path(__file__).resolve().parents[2]
DEFAULT_SCHEMA_DIR = PROTOCOL_DIR / "schemas" / f"v{PROTOCOL_VERSION}"
DEFAULT_TS_FILE = PROTOCOL_DIR / "packages" / "robotics-protocol" / "types.ts"

BUNDLE_ID = f"https://dsh-robotics.local/protocol/{PROTOCOL_VERSION}/protocol.bundle.json"


def _collect_defs() -> dict[str, Any]:
    bundle, _ = models_json_schema(
        [(model, "validation") for model in MODELS.values()],
        ref_template="#/$defs/{model}",
    )
    defs = dict(bundle.get("$defs", {}))
    if not defs:
        for model in MODELS.values():
            schema = model.model_json_schema(ref_template="#/$defs/{model}")
            defs.update(schema.pop("$defs", {}))
            defs.setdefault(model.__name__, schema)
    return defs


def build_bundle() -> dict[str, Any]:
    defs = _collect_defs()
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": BUNDLE_ID,
        "title": "DSH Robotics protocol",
        "description": (
            "Generated from python/robotics_protocol. Lengths are metres, angles radians, "
            "time seconds, quaternions xyzw and normalized. Unknown fields are rejected."
        ),
        "x-protocol-version": PROTOCOL_VERSION,
        "$defs": {name: defs[name] for name in sorted(defs)},
    }


def _ref_name(ref: str) -> str:
    return ref.rsplit("/", 1)[-1]


def _literal(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False)


def _doc(text: str) -> str:
    return " ".join(str(text).split())


def render_type(schema: dict[str, Any], level: int = 0) -> str:
    pad = "  " * level
    if "$ref" in schema:
        return _ref_name(schema["$ref"])
    if "const" in schema:
        return _literal(schema["const"])
    if "enum" in schema:
        return " | ".join(_literal(value) for value in schema["enum"])
    if "anyOf" in schema:
        return " | ".join(render_type(option, level) for option in schema["anyOf"])
    if "oneOf" in schema:
        return " | ".join(render_type(option, level) for option in schema["oneOf"])
    if "allOf" in schema:
        return " & ".join(render_type(option, level) for option in schema["allOf"])

    schema_type = schema.get("type")
    if isinstance(schema_type, list):
        return " | ".join(render_type({**schema, "type": item}, level) for item in schema_type)

    if schema_type == "array":
        prefix = schema.get("prefixItems")
        if prefix:
            return "[" + ", ".join(render_type(item, level + 1) for item in prefix) + "]"
        items = schema.get("items")
        if isinstance(items, dict):
            inner = render_type(items, level + 1)
            if "\n" in inner:
                return "Array<\n" + inner + "\n" + pad + ">"
            if " | " in inner or " & " in inner:
                return "(" + inner + ")[]"
            return inner + "[]"
        return "unknown[]"

    if schema_type == "object":
        properties = schema.get("properties")
        if properties:
            lines = ["{"]
            required = set(schema.get("required", []))
            for name, prop in properties.items():
                optional = "" if name in required else "?"
                description = prop.get("description")
                if description:
                    lines.append(f"{pad}  /** {_doc(description)} */")
                lines.append(f"{pad}  {_literal(name)}{optional}: {render_type(prop, level + 1)};")
            lines.append(pad + "}")
            return "\n".join(lines)
        extra = schema.get("additionalProperties")
        if isinstance(extra, dict):
            return f"Record<string, {render_type(extra, level)}>"
        return "Record<string, unknown>"

    return {
        "string": "string",
        "integer": "number",
        "number": "number",
        "boolean": "boolean",
        "null": "null",
    }.get(schema_type, "unknown")


def render_definition(name: str, schema: dict[str, Any]) -> str:
    description = schema.get("description")
    lines: list[str] = []
    if description:
        lines.append(f"/** {_doc(description)} */")
    properties = schema.get("properties")
    if schema.get("type") == "object" and properties:
        lines.append(f"export interface {name} {{")
        required = set(schema.get("required", []))
        for prop_name, prop in properties.items():
            optional = "" if prop_name in required else "?"
            prop_description = prop.get("description")
            if prop_description:
                lines.append(f"  /** {_doc(prop_description)} */")
            rendered = render_type(prop, 1)
            lines.append(f"  {_literal(prop_name)}{optional}: {rendered};")
        lines.append("}")
    else:
        lines.append(f"export type {name} = {render_type(schema)};")
    return "\n".join(lines)


def render_types(bundle: dict[str, Any]) -> str:
    header = (
        "/**\n"
        " * Generated by python/robotics_protocol/export_schema.py - do not edit.\n"
        f" * DSH Robotics protocol v{PROTOCOL_VERSION}. Lengths in m, angles in rad, time in s,\n"
        " * quaternions xyzw and normalized. Unknown fields are rejected at runtime.\n"
        " */\n"
    )
    body = "\n\n".join(
        render_definition(name, schema) for name, schema in bundle["$defs"].items()
    )
    return header + "\n" + body + "\n"


def write_outputs(schema_dir: Path = DEFAULT_SCHEMA_DIR, ts_file: Path = DEFAULT_TS_FILE) -> tuple[Path, Path]:
    bundle = build_bundle()
    schema_dir.mkdir(parents=True, exist_ok=True)
    bundle_path = schema_dir / "protocol.bundle.json"
    bundle_path.write_text(json.dumps(bundle, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    index = {
        "protocolVersion": PROTOCOL_VERSION,
        "schema": "protocol.bundle.json",
        "models": sorted(bundle["$defs"]),
    }
    (schema_dir / "models.json").write_text(
        json.dumps(index, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    ts_file.parent.mkdir(parents=True, exist_ok=True)
    ts_file.write_text(render_types(bundle), encoding="utf-8")
    return bundle_path, ts_file


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--schema-dir", type=Path, default=DEFAULT_SCHEMA_DIR)
    parser.add_argument("--ts-file", type=Path, default=DEFAULT_TS_FILE)
    args = parser.parse_args()
    bundle_path, ts_file = write_outputs(args.schema_dir, args.ts_file)
    print(json.dumps({"schema": str(bundle_path), "types": str(ts_file)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
