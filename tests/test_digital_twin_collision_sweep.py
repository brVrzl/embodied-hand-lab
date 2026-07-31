from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
import pytest

from digital_twin.collision_sweep import (
    bounded_duration,
    classify_contact_pair,
    contact_rows,
    deterministic_arm_samples,
    early_termination_reason,
    enforce_noncolliding_layers,
    environment_depth_status,
    hand_qpos_from_ctrl,
    is_baseline_pair,
    persistence_status,
    set_static_state,
    smoothstep_interpolation,
    update_consecutive_contact_durations,
    verify_operational_scene,
    write_csv,
)
from digital_twin.io import load_structured


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "models/digital_twin/workspace_scene.xml"
CONFIG = ROOT / "digital_twin/configs/collision_classification.yaml"
OPERATIONAL = ROOT / "digital_twin/configs/robot_operational_placement.yaml"


@pytest.fixture(scope="module")
def model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(SCENE))


@pytest.fixture(scope="module")
def config() -> dict:
    return load_structured(CONFIG)


def test_deterministic_sample_generation() -> None:
    limits = np.asarray([[-1.0, 1.0]] * 6)
    first = deterministic_arm_samples(limits, halton_samples=7, halton_skip=3)
    second = deterministic_arm_samples(limits, halton_samples=7, halton_skip=3)
    assert [name for name, _, _ in first] == [name for name, _, _ in second]
    assert np.allclose(np.stack([q for _, q, _ in first]), np.stack([q for _, q, _ in second]))
    assert any(name.startswith("oat_j1") for name, _, _ in first)
    assert any(name.startswith("pair_j1_j2") for name, _, _ in first)
    assert sum(name.startswith("halton_") for name, _, _ in first) == 7


def test_invalid_sample_bounds_are_rejected() -> None:
    with pytest.raises(ValueError, match="six valid"):
        deterministic_arm_samples([[0, 1]] * 5)


def test_baseline_matching_is_order_independent(config: dict) -> None:
    assert is_baseline_pair("jaka_Link_0_geom_0", "jaka_Link_1_geom_0", config)
    assert is_baseline_pair("jaka_Link_1_geom_0", "jaka_Link_0_geom_0", config)
    assert not is_baseline_pair("jaka_Link_1_geom_0", "jaka_Link_2_geom_0", config)


@pytest.mark.parametrize(
    ("depth", "expected"),
    [(0.0005, "REVIEW"), (0.001, "REVIEW"), (0.002, "WARN"), (0.003, "WARN"), (0.0031, "FAIL")],
)
def test_depth_thresholds(config: dict, depth: float, expected: str) -> None:
    assert environment_depth_status(depth, config) == expected


def test_persistence_thresholds(config: dict) -> None:
    assert persistence_status(0.1, config) is None
    assert persistence_status(0.1001, config) == "WARN"
    assert persistence_status(0.5, config) == "WARN"
    assert persistence_status(0.5001, config) == "FAIL"


def test_dynamic_interpolation_is_bounded_and_preserves_endpoints() -> None:
    start = np.zeros(3); target = np.asarray([1.0, -0.5, 0.25])
    duration = bounded_duration(start, target, base_duration_s=0.2, max_velocity=[0.5] * 3, max_acceleration=[1.0] * 3)
    trajectory = smoothstep_interpolation(start, target, 101)
    assert duration >= 3.0
    assert np.allclose(trajectory[0], start)
    assert np.allclose(trajectory[-1], target)
    assert np.all(np.diff(trajectory[:, 0]) >= 0)


def test_early_contact_duration_resets_for_inactive_pairs() -> None:
    pair = ("a", "b")
    first = update_consecutive_contact_durations([pair], {}, 0.002)
    second = update_consecutive_contact_durations([pair], first, 0.002)
    third = update_consecutive_contact_durations([], second, 0.002)
    assert second[pair] == pytest.approx(0.004)
    assert third == {}


def test_early_termination_policy() -> None:
    assert early_termination_reason([0.0], [0.0], [], 0.02) is None
    assert early_termination_reason([np.nan], [0.0], [], 0.02) == "nonfinite_state"
    assert early_termination_reason([0.0], [0.0], [{"baseline": False, "penetration_depth_m": 0.021}], 0.02) == "catastrophic_penetration"
    assert early_termination_reason([0.0], [0.0], [{"baseline": True, "penetration_depth_m": 0.2}], 0.02) is None


def test_hand_qpos_uses_existing_mimic_equalities(model: mujoco.MjModel) -> None:
    qpos = hand_qpos_from_ctrl(model, [0.5, 0.4, 0.7, 0.8, 0.9, 1.0])
    assert qpos.shape == (12,)
    assert qpos[2] == pytest.approx(0.24)
    assert qpos[3] == pytest.approx(0.32)
    assert qpos[5] == pytest.approx(qpos[4])
    assert qpos[11] == pytest.approx(qpos[10])


def test_operational_transform_and_noncollision_enforcement(model: mujoco.MjModel, config: dict) -> None:
    result = verify_operational_scene(model, load_structured(OPERATIONAL), config)
    assert result["yaw_deg"] == 180.0
    assert result["qpos0_all_zero"] is True
    assert result["palm_error_deg"] < 3.0
    layers = enforce_noncolliding_layers(model, config)
    assert len(layers["camera_sites_verified"]) == 3
    assert layers["forbidden_geoms_present"] == []


def test_operational_transform_regression_aborts(model: mujoco.MjModel, config: dict) -> None:
    operational = dict(load_structured(OPERATIONAL)); operational["yaw_deg"] = 0.0
    with pytest.raises(RuntimeError, match="yaw"):
        verify_operational_scene(model, operational, config)


def test_zero_pose_actual_scene_has_only_baseline_contacts(model: mujoco.MjModel, config: dict) -> None:
    data = mujoco.MjData(model); set_static_state(model, data, np.zeros(6), np.zeros(6))
    rows = contact_rows(model, data, config, context={"phase": "integration", "step": 0})
    assert len(rows) == 4
    assert {row["status"] for row in rows} == {"BASELINE"}
    assert {row["category"] for row in rows} == {"canonical_baseline_self_contact"}
    assert all(np.isfinite(row["normal_force_n"]) for row in rows)


def test_contact_classification_detects_arm_table(model: mujoco.MjModel, config: dict) -> None:
    arm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "jaka_Link_6_geom_0")
    table = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "workspace_tabletop")
    result = classify_contact_pair(model, arm, table, 0.002, config)
    assert result == {"category": "arm_table_contact", "status": "WARN", "baseline": False}


def test_pose_source_loading_and_report_aggregation(model: mujoco.MjModel) -> None:
    from tools.digital_twin.run_joint_space_collision_sweep import _aggregate_events, _load_repository_poses

    poses = _load_repository_poses(
        model, ROOT / "configs/sim/jaka_collision_sweep_poses.yaml"
    )
    names = {pose.name for pose in poses}
    assert {"upright", "teleop_ready"} <= names
    rows = [
        {"phase": "dynamic", "trajectory_id": 2, "sample_id": "", "status": "WARN", "geom_a": "a", "geom_b": "b", "body_a": "A", "body_b": "B", "category": "arm_table_contact", "simulation_time_s": time, "step": index, "penetration_depth_m": depth, "normal_force_n": 3.0, "contact_duration_s": time, "qpos": [index], "contact_position_m": [0, 0, 0], "trajectory_name": "test"}
        for index, (time, depth) in enumerate(((0.1, 0.001), (0.2, 0.002)))
    ]
    events = _aggregate_events(rows)
    assert len(events) == 1
    assert events[0]["contact_sample_count"] == 2
    assert events[0]["peak_penetration_m"] == pytest.approx(0.002)
    assert events[0]["first_contact_qpos"] == [0]


def test_contact_timeline_csv_serialization(tmp_path: Path) -> None:
    target = tmp_path / "timeline.csv"
    write_csv(target, [{"step": 3, "qpos": [0.0, 1.0], "status": "WARN"}], ["step", "qpos", "status"])
    text = target.read_text(encoding="utf-8")
    assert text.splitlines()[0] == "step,qpos,status"
    assert '"[0.0,1.0]"' in text
    assert text.rstrip().endswith(",WARN")
