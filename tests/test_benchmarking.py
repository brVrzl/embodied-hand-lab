from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys

import pytest

from benchmarking import (
    BENCHMARK_CONFIG_SCHEMA,
    BENCHMARK_RESULT_SCHEMA,
    BenchmarkConfig,
    run_mujoco_joint_reach_preshape,
    write_benchmark_result,
)
from benchmarking.cli import build_parser


ROOT = Path(__file__).resolve().parents[1]


def test_cli_has_repository_local_default_output() -> None:
    args = build_parser().parse_args([])
    assert args.output == ROOT / "build" / "validation" / "benchmark.json"


CONFIG_PATH = ROOT / "configs" / "benchmark" / "smoke.yaml"
TOOL_PATH = ROOT / "tools" / "run_benchmark.py"


def test_smoke_config_is_strict_and_snapshots_effective_values() -> None:
    config = BenchmarkConfig.load(CONFIG_PATH, repository_root=ROOT)
    snapshot = config.snapshot(repository_root=ROOT)
    assert snapshot["schema_version"] == BENCHMARK_CONFIG_SCHEMA
    assert snapshot["simulation"]["replay_config_path"] == (
        "configs/sim/quest_hts_jaka_mini2_offline.yaml"
    )
    assert snapshot["task"]["step_count"] == 250
    assert list(snapshot["task"]["hand"]["target_rad"]) == [
        "thumb_lateral",
        "thumb_close",
        "index",
        "middle",
        "ring",
        "pinky",
    ]


def test_fresh_simulations_are_deterministic_for_the_same_seed() -> None:
    config = BenchmarkConfig.load(CONFIG_PATH, repository_root=ROOT)
    first = run_mujoco_joint_reach_preshape(config, repository_root=ROOT)
    second = run_mujoco_joint_reach_preshape(config, repository_root=ROOT)

    assert first["schema_version"] == BENCHMARK_RESULT_SCHEMA
    assert first["status"] == "passed"
    assert first["failure_reason"] is None
    assert first["sampled_action"] == second["sampled_action"]
    assert first["metrics"] == second["metrics"]
    assert first["metrics"]["arm_reached"]
    assert first["metrics"]["hand_preshape_reached"]
    assert first["metrics"]["final_arm_max_absolute_error_rad"] <= 0.001
    assert first["metrics"]["final_hand_max_absolute_error_rad"] <= 0.002
    assert any("does not measure grasp" in text for text in first["limitations"])


def test_model_action_boundary_rejects_unbounded_target_without_steps() -> None:
    config = BenchmarkConfig.load(CONFIG_PATH, repository_root=ROOT)
    invalid = replace(
        config,
        arm_target_offset_rad=(100.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        arm_target_jitter_rad=(0.0,) * 6,
    )
    result = run_mujoco_joint_reach_preshape(invalid, repository_root=ROOT)
    assert result["status"] == "invalid"
    assert result["failure_reason"] == "arm_target_out_of_model_joint_range"
    assert result["steps_executed"] == 0
    assert result["metrics"] == {}


def test_result_writer_is_atomic_and_failed_serialization_preserves_old_result(
    tmp_path: Path,
) -> None:
    output = tmp_path / "results" / "smoke.json"
    assert write_benchmark_result(output, {"status": "passed"}) == output
    with pytest.raises(TypeError):
        write_benchmark_result(
            output, {"status": "passed", "invalid": object()}
        )
    assert json.loads(output.read_text(encoding="utf-8")) == {
        "status": "passed"
    }
    assert not list(output.parent.glob(f".{output.name}.*.tmp"))


def test_cli_help_and_default_smoke_run(tmp_path: Path) -> None:
    help_result = subprocess.run(
        [sys.executable, str(TOOL_PATH), "--help"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert help_result.returncode == 0
    assert "offline-only" in help_result.stdout
    assert "--output" in help_result.stdout

    output = tmp_path / "benchmark.json"
    run_result = subprocess.run(
        [sys.executable, str(TOOL_PATH), "--output", str(output)],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    assert run_result.returncode == 0, run_result.stderr
    result = json.loads(output.read_text(encoding="utf-8"))
    assert result["status"] == "passed"
    assert result["simulation"]["backend"] == "mujoco"
    assert result["steps_executed"] == 250
