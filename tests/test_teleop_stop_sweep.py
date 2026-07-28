from __future__ import annotations

import json
from pathlib import Path

from teleop_rearchitecture.stop_sweep import (
    StopLimits,
    build_release_state_matrix,
    run_controlled_stop_sweep,
    simulate_controlled_stop,
)


MODEL = Path("data/sim_assets/jaka_rh56.xml")


def test_release_matrix_covers_required_velocities_and_state_classes() -> None:
    states = build_release_state_matrix()
    speeds = {state.nominal_peak_velocity_rad_s for state in states}
    assert {0.02, 0.05, 0.10, 0.25, 0.50, 1.00} <= speeds
    classes = {state.state_class for state in states}
    assert {
        "zero_acceleration", "positive_acceleration", "negative_acceleration",
        "jerk_ramp_up", "jerk_ramp_down", "after_target_replacement",
        "after_direction_reversal", "mixed_six_axis", "dominant_wrist",
        "dominant_shoulder",
    } <= classes


def test_all_stop_policies_preserve_position_continuity_and_limits() -> None:
    limits = StopLimits()
    state = next(
        state for state in build_release_state_matrix()
        if state.nominal_peak_velocity_rad_s == 0.25 and state.state_class == "mixed_six_axis"
    )
    for policy in (
        "stopping_point_tracking",
        "explicit_jerk_limited_zero_velocity",
        "adaptive_critically_damped",
    ):
        result = simulate_controlled_stop(state, policy=policy, limits=limits)
        assert result["position_jump_rad"] == 0.0
        assert result["limit_violations"] == {"velocity": 0, "acceleration": 0, "jerk": 0}
        assert result["strict_completion"]["completed"] is True
        assert result["post_completion_drift_rad"] <= 1e-4


def test_explicit_braking_improves_low_speed_strict_stop() -> None:
    state = next(
        state for state in build_release_state_matrix()
        if state.nominal_peak_velocity_rad_s == 0.02 and state.state_class == "zero_acceleration"
    )
    baseline = simulate_controlled_stop(state, policy="stopping_point_tracking")
    explicit = simulate_controlled_stop(state, policy="explicit_jerk_limited_zero_velocity")
    assert explicit["strict_completion"]["time_ms"] < baseline["strict_completion"]["time_ms"]


def test_stop_sweep_schema_is_finite_and_reports_all_policies() -> None:
    report = run_controlled_stop_sweep(
        model_path=MODEL,
        repository_commit="test-commit",
        working_tree_dirty=True,
    )
    assert report["schema_version"] == "teleop_controlled_stop_sweep.v1"
    assert set(report["policy_summary"]) == {
        "stopping_point_tracking",
        "explicit_jerk_limited_zero_velocity",
        "adaptive_critically_damped",
    }
    json.dumps(report, allow_nan=False)
