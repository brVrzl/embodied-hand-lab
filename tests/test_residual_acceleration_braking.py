from __future__ import annotations

import json
from pathlib import Path

from teleop_rearchitecture.cpp_shaping import default_cpp_library
from teleop_rearchitecture.residual_braking import (
    build_residual_acceleration_cases,
    run_residual_acceleration_sweep,
)


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "data/sim_assets/jaka_rh56.xml"
CHECKED = ROOT / (
    "docs/research/teleop_rearchitecture/results/"
    "residual_acceleration_stop_sweep.json"
)


def test_residual_acceleration_matrix_has_required_boundaries() -> None:
    cases = build_residual_acceleration_cases()
    assert len(cases) == 115
    j2 = [case for case in cases if case["case_id"].startswith("j2_")]
    assert len(j2) == 99
    assert {case["velocity_rad_s"][1] for case in j2} == {
        0.0, 1e-6, -1e-6, 1e-4, -1e-4, 1e-3, -1e-3, 1e-2, -1e-2
    }
    assert {case["acceleration_rad_s2"][1] for case in j2} >= {
        0.0, 0.1, -0.1, 0.5, -0.5, 1.0, -1.0, 4.0, -4.0, 12.0, -12.0
    }
    assert any(case["case_id"] == "mixed_six_axis" for case in cases)
    assert sum(case["expected_failure"] is not None for case in cases) == 2


def test_residual_acceleration_sweep_matches_checked_artifact() -> None:
    actual = run_residual_acceleration_sweep(default_cpp_library(ROOT), MODEL)
    assert actual["summary"]["unexpected_failure_count"] == 0
    assert actual["summary"]["completed_count"] == 113
    assert actual["summary"]["expected_unplannable_count"] == 2
    assert actual["summary"]["direction_consistent_count"] == 113
    assert all(
        max(abs(value) for value in row["final_velocity_rad_s"]) <= 1e-12
        and max(abs(value) for value in row["final_acceleration_rad_s2"]) <= 1e-12
        for row in actual["cases"] if row["completion"]
    )
    checked = json.loads(CHECKED.read_text(encoding="utf-8"))
    assert actual == checked
