from __future__ import annotations

import json
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "tests/no_sdk_test_manifest.json"


def test_no_sdk_manifest_is_explicit_complete_and_excludes_linked_worker() -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    paths = manifest["python_test_paths"]
    assert manifest["schema_version"] == "teleop_no_sdk_test_manifest.v1"
    assert paths == sorted(paths)
    assert len(paths) == len(set(paths))
    assert "tests/test_native_jaka_servo_worker.py" in manifest["forbidden_test_paths"]
    assert "tests/test_native_jaka_servo_worker.py" not in paths
    assert all((ROOT / path).is_file() for path in paths)
    assert "tests/test_no_sdk_manifest.py" in paths
    assert "tests/test_teleop_engagement_recovery.py" in paths
    assert "tests/test_residual_acceleration_braking.py" in paths
    assert "build/teleop_shaping/thin_jaka_transport_tests" in manifest["native_artifacts"]


def test_reference_native_targets_have_no_jaka_dependency_or_symbol() -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    patterns = tuple(value.lower() for value in manifest["forbidden_library_patterns"])
    symbol_patterns = tuple(value.lower() for value in manifest["forbidden_symbol_patterns"])
    for relative_path in manifest["native_artifacts"]:
        path = ROOT / relative_path
        dynamic = subprocess.run(
            ["readelf", "-d", str(path)], check=True, text=True, capture_output=True
        ).stdout.lower()
        symbols = subprocess.run(
            ["nm", "-D", str(path)], check=True, text=True, capture_output=True
        ).stdout.lower()
        assert not any(pattern in dynamic for pattern in patterns)
        assert not any(pattern in symbols for pattern in symbol_patterns)
