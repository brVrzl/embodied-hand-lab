from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

from embodiment_core.doctor import collect_doctor_report


ROOT = Path(__file__).resolve().parents[1]


def test_doctor_is_read_only_and_parses_repository_configs() -> None:
    report = collect_doctor_report(ROOT)
    assert report["status"] == "ready_offline"
    assert report["safety"]["device_connections_attempted"] is False
    assert report["safety"]["robot_commands_sent"] is False
    assert report["repository"]["configurations"]["errors"] == []
    assert report["repository"]["project_git_metadata_present"] is (
        ROOT / ".git"
    ).exists()


def test_unified_cli_help_and_dataset_help() -> None:
    expected_usage = {
        ("--help",): "usage: embodied-lab",
        ("dataset", "--help"): "usage: embodied-lab dataset",
        ("distributed-smoke", "--help"): (
            "usage: embodied-lab distributed-smoke"
        ),
        ("benchmark", "--help"): "usage: embodied-lab benchmark",
    }
    for arguments, usage in expected_usage.items():
        result = subprocess.run(
            [sys.executable, "-m", "embodiment_core.cli", *arguments],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        assert result.returncode == 0, result.stderr
        assert usage in result.stdout


def test_headless_simulation_smoke() -> None:
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "embodiment_core.cli",
            "sim",
            "smoke",
            "--duration-sec",
            "0.004",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout)
    assert payload["status"] == "passed"
    assert payload["validation_level"] == "offline_simulation"
    assert payload["initial_contact_count"] == 0
    assert payload["hardware_connections_attempted"] is False
