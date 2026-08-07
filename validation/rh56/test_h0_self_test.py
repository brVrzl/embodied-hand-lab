from __future__ import annotations

import ast
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

from rh56_sim import CANONICAL_CHANNEL_ORDER, RH56_CHANNELS, Rh56H0SelfTest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = PROJECT_ROOT / "assets/jaka_rh56_visual_coacd.xml"
TOOL_PATH = PROJECT_ROOT / "tools/rh56_h0_self_test.py"
MODULE_PATH = PROJECT_ROOT / "src/rh56_sim/h0_self_test.py"


def _name(model: mujoco.MjModel, object_type: mujoco.mjtObj, object_id: int) -> str:
    value = mujoco.mj_id2name(model, object_type, object_id)
    assert value is not None
    return value


def _runner(tmp_path: Path) -> Rh56H0SelfTest:
    return Rh56H0SelfTest(
        model_path=MODEL_PATH,
        log_path=tmp_path / "h0.jsonl",
        cycle_seconds=0.05,
        amplitude_scale=0.10,
        initial_arm_joints_rad=(-1.57, -0.61, -1.57, 0.17, 1.13, -0.26),
    )


def test_h0_combined_mjcf_loads_with_six_arm_and_six_hand_actuators() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    actuator_names = tuple(
        _name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index) for index in range(model.nu)
    )
    hand_joint_names = tuple(
        _name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(model.njnt)
        if _name(model, mujoco.mjtObj.mjOBJ_JOINT, index).startswith("rh56_")
    )

    assert model.nu == 12
    assert len(actuator_names) == len(set(actuator_names))
    assert len([name for name in actuator_names if name.startswith("jaka_")]) == 6
    assert len([name for name in actuator_names if name.startswith("rh56_")]) == 6
    assert len(hand_joint_names) == 12


def test_h0_actuator_to_joint_mapping_and_ranges_are_explicit(tmp_path: Path) -> None:
    runner = _runner(tmp_path)

    for row, channel in zip(runner.mapping_rows(), RH56_CHANNELS, strict=True):
        actuator_id = mujoco.mj_name2id(
            runner.model, mujoco.mjtObj.mjOBJ_ACTUATOR, channel.actuator
        )
        joint_id = mujoco.mj_name2id(
            runner.model, mujoco.mjtObj.mjOBJ_JOINT, channel.joint
        )
        assert int(runner.model.actuator_trnid[actuator_id, 0]) == joint_id
        assert bool(runner.model.actuator_ctrllimited[actuator_id])
        assert row["joint_range"] == row["ctrl_range"]


def test_h0_canonical_protocol_raw_and_mujoco_orders_are_not_conflated(
    tmp_path: Path,
) -> None:
    runner = _runner(tmp_path)
    rows = runner.mapping_rows()

    assert tuple(row["canonical"] for row in rows) == CANONICAL_CHANNEL_ORDER
    assert tuple(row["protocol_index"] for row in rows) == (3, 2, 1, 0, 4, 5)
    assert tuple(row["raw_index"] for row in rows) == (2, 3, 4, 5, 0, 1)
    assert tuple(channel.spec.canonical for channel in runner.channels) != (
        "thumb_lateral",
        "thumb_close",
        "index",
        "middle",
        "ring",
        "pinky",
    )


def test_h0_thumb_close_and_lateral_commands_are_independent(tmp_path: Path) -> None:
    runner = _runner(tmp_path)
    neutral = runner.initial_hand_target
    close = runner.command_vector("thumb_close", 0.05)
    lateral = runner.command_vector("thumb_lateral", 0.11)
    close_index = CANONICAL_CHANNEL_ORDER.index("thumb_close")
    lateral_index = CANONICAL_CHANNEL_ORDER.index("thumb_lateral")

    assert close[close_index] == 0.05
    assert close[lateral_index] == neutral[lateral_index]
    assert lateral[lateral_index] == 0.11
    assert lateral[close_index] == neutral[close_index]
    assert np.count_nonzero(close != neutral) == 1
    assert np.count_nonzero(lateral != neutral) == 1


def test_h0_clips_to_joint_ctrl_intersection(tmp_path: Path) -> None:
    runner = _runner(tmp_path)

    assert runner.clipped_target("thumb_close", -1.0) == (0.0, True)
    clipped, saturated = runner.clipped_target("thumb_close", 2.0)
    assert clipped == pytest.approx(math.radians(40.0))
    assert saturated
    assert runner.clipped_target("thumb_close", 0.25) == (0.25, False)


def test_h0_headless_run_has_no_nan_and_keeps_arm_target(tmp_path: Path) -> None:
    runner = _runner(tmp_path)
    arm_before = runner.data.ctrl[runner.arm_actuator_ids].copy()

    result = runner.run(viewer=False)
    records = [json.loads(line) for line in result.log_path.read_text().splitlines()]

    assert result.completed
    assert result.invalid_count == 0
    assert result.arm_target_unchanged
    assert np.array_equal(runner.data.ctrl[runner.arm_actuator_ids], arm_before)
    assert result.completed_channels == CANONICAL_CHANNEL_ORDER
    assert records
    assert all(not record["invalid_nan"] for record in records)
    assert all(np.isfinite(record["requested_ctrl"]) for record in records)
    assert all(np.isfinite(record["clipped_ctrl"]) for record in records)
    assert all(np.isfinite(record["actual_qpos"]) for record in records)
    assert all(0.0 <= record["phase_progress"] <= 1.0 for record in records)


def test_h0_entry_has_no_hardware_or_network_imports() -> None:
    imports: set[str] = set()
    for path in (TOOL_PATH, MODULE_PATH):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module is not None:
                imports.add(node.module)
    forbidden = (
        "jaka_driver_adapter",
        "rh56_driver",
        "serial",
        "socket",
        "can",
        "modbus",
        "rs485",
    )

    assert all(
        not any(module == name or module.startswith(f"{name}.") for module in imports)
        for name in forbidden
    )
    assert "quest_jaka_sim" not in imports
    assert "motion_input" not in imports


def test_h0_initial_hand_has_no_penetrating_contact(tmp_path: Path) -> None:
    runner = _runner(tmp_path)

    assert runner.initial_penetrating_contacts == ()
