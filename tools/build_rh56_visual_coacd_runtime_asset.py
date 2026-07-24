#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from rh56_collision_model import (  # noqa: E402
    REVIEWED_INTERNAL_EXCLUDED_BODY_PAIRS,
    VISUAL_COACD_ASSET_DIR,
    VISUAL_COACD_SOURCE_STEMS,
    patch_rh56_visual_coacd_collision_model,
)

ASSET_ROOT = PROJECT_ROOT / "data" / "sim_assets"
DEFAULT_SOURCE = ASSET_ROOT / "jaka_rh56.xml"
DEFAULT_OUTPUT = ASSET_ROOT / "jaka_rh56_visual_coacd.xml"
DEFAULT_MANIFEST = ASSET_ROOT / "jaka_rh56_visual_coacd.manifest.json"


def _sha256_bytes(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def _sha256_file(path: Path) -> str:
    return _sha256_bytes(path.read_bytes())


def _relative(path: Path) -> str:
    return path.resolve().relative_to(PROJECT_ROOT).as_posix()


def build_runtime_xml(source: Path) -> bytes:
    root = ET.parse(source).getroot()
    patch_rh56_visual_coacd_collision_model(root, asset_root=ASSET_ROOT)
    root.set("model", "jaka_minicobo_rh56_visual_coacd")
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n"


def build_manifest(source: Path, output: Path, xml_bytes: bytes) -> dict[str, object]:
    hull_paths = sorted((ASSET_ROOT / VISUAL_COACD_ASSET_DIR).glob("*_part*.stl"))
    visual_paths = sorted((ASSET_ROOT / "meshes" / "rh56").glob("*.STL"))
    checksums = {
        _relative(path): _sha256_file(path)
        for path in [source, *visual_paths, *hull_paths]
    }
    checksums[_relative(output)] = _sha256_bytes(xml_bytes)
    return {
        "baseline": "rh56_visual_coacd_148_v1",
        "status": "selected_default_collision_model",
        "runtime_xml": _relative(output),
        "derivation_source_xml": _relative(source),
        "vendor_visual_mesh_directory": "data/sim_assets/meshes/rh56",
        "coacd_mesh_directory": _relative(ASSET_ROOT / VISUAL_COACD_ASSET_DIR),
        "coacd_generation_manifest": _relative(ASSET_ROOT / VISUAL_COACD_ASSET_DIR / "manifest.json"),
        "vendor_visual_geom_count": len(VISUAL_COACD_SOURCE_STEMS),
        "coacd_hull_count": len(hull_paths),
        "reviewed_exclusions": [list(pair) for pair in REVIEWED_INTERNAL_EXCLUDED_BODY_PAIRS],
        "geometry_policy": {
            "vendor_visual_geoms": "rendering_and_diagnostics_only",
            "active_rh56_collision_geoms": "visual_coacd_only",
            "legacy_proxy_geoms": "absent",
            "correll_collision_geoms": "absent",
        },
        "sha256": checksums,
    }


def _json_bytes(value: dict[str, object]) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build or verify the fixed RH56 visual_coacd runtime MJCF without regenerating geometry."
    )
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--check", action="store_true", help="Fail if committed outputs differ from derivation.")
    args = parser.parse_args()

    source = args.source.resolve()
    output = args.output.resolve()
    manifest_path = args.manifest.resolve()
    xml_bytes = build_runtime_xml(source)
    manifest_bytes = _json_bytes(build_manifest(source, output, xml_bytes))

    if args.check:
        stale = [
            str(path)
            for path, expected in ((output, xml_bytes), (manifest_path, manifest_bytes))
            if not path.exists() or path.read_bytes() != expected
        ]
        if stale:
            raise SystemExit(f"stale visual_coacd runtime assets: {', '.join(stale)}")
        print(f"verified {_relative(output)} and {_relative(manifest_path)}")
        return

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(xml_bytes)
    manifest_path.write_bytes(manifest_bytes)
    print(f"wrote {_relative(output)}")
    print(f"wrote {_relative(manifest_path)}")


if __name__ == "__main__":
    main()
