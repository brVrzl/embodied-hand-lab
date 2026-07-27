#!/usr/bin/env python3
"""Run an explicit test allow-list and fail if a JAKA SDK image is loaded."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = ROOT / "tests/no_sdk_test_manifest.json"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def loaded_images() -> set[str]:
    maps = Path("/proc/self/maps")
    if not maps.is_file():
        raise RuntimeError("/proc/self/maps is required for the no-SDK load gate")
    images: set[str] = set()
    for line in maps.read_text(encoding="utf-8").splitlines():
        path = line.split()[-1]
        if path.startswith("/"):
            images.add(path)
    return images


def matching(values: set[str] | list[str], patterns: list[str]) -> list[str]:
    return sorted(value for value in values if any(pattern in value.lower() for pattern in patterns))


def inspect_native(path: Path, library_patterns: list[str], symbol_patterns: list[str]) -> dict[str, object]:
    dynamic = subprocess.run(
        ["readelf", "-d", str(path)], check=True, text=True, capture_output=True
    ).stdout
    symbols = subprocess.run(
        ["nm", "-D", str(path)], check=True, text=True, capture_output=True
    ).stdout
    forbidden_dependencies = matching(dynamic.lower().splitlines(), library_patterns)
    forbidden_symbols = matching(symbols.lower().splitlines(), symbol_patterns)
    if forbidden_dependencies or forbidden_symbols:
        raise RuntimeError(
            f"SDK dependency/symbol in {path}: deps={forbidden_dependencies} symbols={forbidden_symbols}"
        )
    needed = [line.strip() for line in dynamic.splitlines() if "(NEEDED)" in line]
    return {"path": str(path), "needed": needed, "forbidden_symbols": []}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--skip-ctest", action="store_true")
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    patterns = [value.lower() for value in manifest["forbidden_library_patterns"]]
    symbol_patterns = [value.lower() for value in manifest["forbidden_symbol_patterns"]]
    test_paths = [str(ROOT / value) for value in manifest["python_test_paths"]]
    forbidden_tests = set(manifest["forbidden_test_paths"])
    if forbidden_tests.intersection(manifest["python_test_paths"]):
        raise RuntimeError("forbidden SDK-linked test appears in no-SDK allow-list")

    before = loaded_images()
    if matches := matching(before, patterns):
        raise RuntimeError(f"JAKA SDK already loaded before tests: {matches}")
    native_reports = [
        inspect_native(ROOT / path, patterns, symbol_patterns)
        for path in manifest.get(
            "native_artifacts",
            [manifest["native_library"], manifest["native_test_binary"]],
        )
    ]
    if not args.skip_ctest:
        subprocess.run(
            ["ctest", "--test-dir", str(ROOT / manifest["native_build_directory"]),
             "--output-on-failure"], check=True
        )
    pytest_code = pytest.main(["-q", "-p", "no:cacheprovider", *test_paths])
    after = loaded_images()
    loaded_during_tests = after - before
    if matches := matching(after, patterns):
        raise RuntimeError(f"JAKA SDK loaded by no-SDK suite: {matches}")
    report = {
        "schema_version": manifest["schema_version"],
        "pytest_exit_code": int(pytest_code),
        "test_file_count": len(test_paths),
        "native": native_reports,
        "loaded_image_count_before": len(before),
        "loaded_image_count_after": len(after),
        "new_loaded_images": sorted(loaded_during_tests),
        "forbidden_loaded_images": [],
    }
    print("NO_SDK_GATE=" + json.dumps(report, sort_keys=True))
    return int(pytest_code)


if __name__ == "__main__":
    sys.exit(main())
