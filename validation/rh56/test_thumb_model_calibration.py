from __future__ import annotations

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from quest_jaka_sim import JakaMujocoSimulation, ReplayConfig
from quest_jaka_sim.hand_retarget import (
    RH56_THUMB_CLOSE_RANGE_RAD,
    RH56_THUMB_DIP_POLYCOEF,
    RH56_THUMB_DIP_RANGE_RAD,
    RH56_THUMB_LATERAL_RANGE_RAD,
    RH56_THUMB_PIP_POLYCOEF,
    RH56_THUMB_PIP_RANGE_RAD,
    thumb_close_coupled_joint_positions,
)


ROOT = Path(__file__).resolve().parents[2]
SOURCE_MODEL = ROOT / "assets/jaka_rh56.xml"
RUNTIME_MODEL = ROOT / "assets/jaka_rh56_visual_coacd.xml"
AUDIT_PATH = ROOT / "assets/rh56_thumb_table_calibration.json"
HAND_ACTUATOR_ORDER = (
    "thumb_lateral",
    "thumb_close",
    "index",
    "middle",
    "ring",
    "pinky",
)


def _polyval(coefficients: tuple[float, ...] | list[float], value: np.ndarray) -> np.ndarray:
    return sum(coefficient * value**power for power, coefficient in enumerate(coefficients))


def _named_element(root: ET.Element, tag: str, name: str) -> ET.Element:
    element = root.find(f".//{tag}[@name='{name}']")
    assert element is not None
    return element


def test_full_1001_row_table_audit_is_monotonic_and_cubic_fit_is_exact() -> None:
    audit = json.loads(AUDIT_PATH.read_text(encoding="utf-8"))
    source = audit["source"]
    assert source["sha256"] == "881d9693a01ee51086c928435df5ab66e12c8e886d100ebaf3dba8f950b305cc"
    assert source["data_row_count"] == 1001
    assert source["command_contiguous"]

    command = np.arange(1001, dtype=float)
    fraction = command / 1000.0
    bend_absolute_deg = 170.0 - 40.0 * fraction
    lateral_absolute_deg = 85.0 + 80.0 * fraction
    palm_absolute_deg = (
        144.01
        + 36.372 * fraction
        + 10.8048 * fraction**2
        - 2.18176 * fraction**3
    )
    tip_absolute_deg = (
        134.657
        + 53.5644 * fraction
        - 17.41424 * fraction**2
        - 0.535232 * fraction**3
    )

    assert np.all(np.diff(bend_absolute_deg) < 0.0)
    assert np.all(np.diff(lateral_absolute_deg) > 0.0)
    assert np.all(np.diff(palm_absolute_deg) > 0.0)
    assert np.all(np.diff(tip_absolute_deg) > 0.0)

    bend_line = np.linspace(bend_absolute_deg[0], bend_absolute_deg[-1], 1001)
    lateral_line = np.linspace(lateral_absolute_deg[0], lateral_absolute_deg[-1], 1001)
    palm_line = np.linspace(palm_absolute_deg[0], palm_absolute_deg[-1], 1001)
    tip_line = np.linspace(tip_absolute_deg[0], tip_absolute_deg[-1], 1001)
    assert np.max(np.abs(bend_absolute_deg - bend_line)) < 1e-12
    assert np.max(np.abs(lateral_absolute_deg - lateral_line)) < 1e-12
    assert np.max(np.abs(palm_absolute_deg - palm_line)) > 1.89
    assert np.max(np.abs(tip_absolute_deg - tip_line)) > 4.55

    close_qpos = np.deg2rad(170.0 - bend_absolute_deg)
    expected_pip = np.deg2rad(palm_absolute_deg - 144.01)
    expected_dip = np.deg2rad(tip_absolute_deg - 134.657)
    np.testing.assert_allclose(
        _polyval(RH56_THUMB_PIP_POLYCOEF, close_qpos),
        expected_pip,
        atol=2e-14,
        rtol=0.0,
    )
    np.testing.assert_allclose(
        _polyval(RH56_THUMB_DIP_POLYCOEF, close_qpos),
        expected_dip,
        atol=2e-14,
        rtol=0.0,
    )


@pytest.mark.parametrize("model_path", (SOURCE_MODEL, RUNTIME_MODEL))
def test_model_uses_relative_table_ranges_and_polynomial_equalities(model_path: Path) -> None:
    root = ET.parse(model_path).getroot()
    lateral = _named_element(root, "joint", "rh56_R_thumb_MCP_joint1")
    close = _named_element(root, "joint", "rh56_R_thumb_MCP_joint2")
    pip = _named_element(root, "joint", "rh56_R_thumb_PIP_joint")
    dip = _named_element(root, "joint", "rh56_R_thumb_DIP_joint")
    assert tuple(float(value) for value in lateral.get("range", "").split()) == pytest.approx(
        (0.0, RH56_THUMB_LATERAL_RANGE_RAD)
    )
    assert tuple(float(value) for value in close.get("range", "").split()) == pytest.approx(
        (0.0, RH56_THUMB_CLOSE_RANGE_RAD)
    )
    assert tuple(float(value) for value in pip.get("range", "").split()) == pytest.approx(
        (0.0, RH56_THUMB_PIP_RANGE_RAD)
    )
    assert tuple(float(value) for value in dip.get("range", "").split()) == pytest.approx(
        (0.0, RH56_THUMB_DIP_RANGE_RAD)
    )
    assert lateral.get("axis") == "0 -1 0"
    assert close.get("axis") == "1 0 0"

    for actuator_name, expected in (
        ("rh56_R_thumb_MCP_joint1_act", RH56_THUMB_LATERAL_RANGE_RAD),
        ("rh56_R_thumb_MCP_joint2_act", RH56_THUMB_CLOSE_RANGE_RAD),
    ):
        actuator = _named_element(root, "position", actuator_name)
        assert tuple(float(value) for value in actuator.get("ctrlrange", "").split()) == pytest.approx(
            (0.0, expected)
        )
        assert actuator.get("ctrllimited") == "true"

    equality = {
        element.get("joint1"): tuple(float(value) for value in element.get("polycoef", "").split())
        for element in root.findall("./equality/joint")
    }
    assert equality["rh56_R_thumb_PIP_joint"] == pytest.approx(RH56_THUMB_PIP_POLYCOEF)
    assert equality["rh56_R_thumb_DIP_joint"] == pytest.approx(RH56_THUMB_DIP_POLYCOEF)


def test_coupling_is_monotonic_and_hits_table_relative_endpoints() -> None:
    values = np.linspace(0.0, RH56_THUMB_CLOSE_RANGE_RAD, 1001)
    coupled = np.asarray([thumb_close_coupled_joint_positions(float(value)) for value in values])
    assert np.all(np.isfinite(coupled))
    assert np.all(np.diff(coupled[:, 0]) > 0.0)
    assert np.all(np.diff(coupled[:, 1]) > 0.0)
    assert coupled[0] == pytest.approx((0.0, 0.0))
    assert coupled[-1] == pytest.approx(
        (RH56_THUMB_PIP_RANGE_RAD, RH56_THUMB_DIP_RANGE_RAD)
    )


def test_positive_lateral_qpos_moves_current_geometry_from_open_toward_opposition() -> None:
    model = mujoco.MjModel.from_xml_path(str(RUNTIME_MODEL))
    data = mujoco.MjData(model)
    base_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link"
    )
    lateral_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "rh56_R_thumb_MCP_joint1"
    )
    lateral_qpos = int(model.jnt_qposadr[lateral_id])

    def body_point(body_name: str, offset: tuple[float, float, float]) -> np.ndarray:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        return data.xpos[body_id] + data.xmat[body_id].reshape(3, 3) @ np.asarray(offset)

    def hand_local(point: np.ndarray) -> np.ndarray:
        return data.xmat[base_id].reshape(3, 3).T @ (point - data.xpos[base_id])

    data.qpos[lateral_qpos] = 0.0
    mujoco.mj_forward(model, data)
    open_thumb = hand_local(body_point("rh56_R_thumb_distal", (0.0, 0.024, -0.001)))
    index_tip = hand_local(
        body_point("rh56_R_index_distal", (0.0083, 0.043, 0.0015))
    )
    pinky_tip = hand_local(body_point("rh56_R_pinky_distal", (0.0, 0.043, 0.0)))
    across_to_pinky = pinky_tip - index_tip
    across_to_pinky /= np.linalg.norm(across_to_pinky)

    data.qpos[lateral_qpos] = RH56_THUMB_LATERAL_RANGE_RAD
    mujoco.mj_forward(model, data)
    opposed_thumb = hand_local(
        body_point("rh56_R_thumb_distal", (0.0, 0.024, -0.001))
    )

    assert np.dot(opposed_thumb - open_thumb, across_to_pinky) > 0.08
    assert np.linalg.norm(opposed_thumb - pinky_tip) < np.linalg.norm(open_thumb - pinky_tip)


def test_full_static_fk_sweep_is_finite_and_detects_only_thumb_index_overlap() -> None:
    model = mujoco.MjModel.from_xml_path(str(RUNTIME_MODEL))
    data = mujoco.MjData(model)
    joint_names = (
        "rh56_R_thumb_MCP_joint1",
        "rh56_R_thumb_MCP_joint2",
        "rh56_R_thumb_PIP_joint",
        "rh56_R_thumb_DIP_joint",
    )
    qpos_addresses = [
        int(
            model.jnt_qposadr[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ]
        )
        for name in joint_names
    ]
    deepest = 0.0
    contact_body_pairs: set[tuple[str, str]] = set()

    for lateral in np.linspace(0.0, RH56_THUMB_LATERAL_RANGE_RAD, 21):
        for close in np.linspace(0.0, RH56_THUMB_CLOSE_RANGE_RAD, 21):
            pip, dip = thumb_close_coupled_joint_positions(float(close))
            data.qpos[qpos_addresses] = (lateral, close, pip, dip)
            mujoco.mj_forward(model, data)
            assert np.all(np.isfinite(data.qpos))
            assert np.all(np.isfinite(data.xpos))
            for contact_index in range(data.ncon):
                contact = data.contact[contact_index]
                bodies = tuple(
                    mujoco.mj_id2name(
                        model,
                        mujoco.mjtObj.mjOBJ_BODY,
                        int(model.geom_bodyid[geom_id]),
                    )
                    or ""
                    for geom_id in (contact.geom1, contact.geom2)
                )
                if not all(name.startswith("rh56_R_") for name in bodies):
                    continue
                contact_body_pairs.add(tuple(sorted(bodies)))
                deepest = min(deepest, float(contact.dist))

    assert contact_body_pairs == {
        ("rh56_R_index_proximal", "rh56_R_thumb_distal"),
        ("rh56_R_index_proximal", "rh56_R_thumb_intermediate"),
    }
    # A forced-qpos FK sweep bypasses contact resolution and exposes the
    # current geometry's extreme thumb/index intersection for review.
    assert -0.007 < deepest < -0.006


def test_dynamic_full_close_lateral_target_remains_finite_and_contact_resolved() -> None:
    config = ReplayConfig.load(ROOT / "configs/sim/quest_hts_jaka_mini2_live_demo.yaml")
    simulation = JakaMujocoSimulation(config, mjcf_path=RUNTIME_MODEL)
    target = {
        "thumb_lateral": RH56_THUMB_LATERAL_RANGE_RAD,
        "thumb_close": RH56_THUMB_CLOSE_RANGE_RAD,
        "index": 0.0,
        "middle": 0.0,
        "ring": 0.0,
        "pinky": 0.0,
    }
    simulation.set_hand_actuator_target(target)
    deepest = 0.0
    for _ in range(2500):
        simulation.step(0.002)
        for contact_index in range(simulation.data.ncon):
            contact = simulation.data.contact[contact_index]
            bodies = [
                mujoco.mj_id2name(
                    simulation.model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    int(simulation.model.geom_bodyid[geom_id]),
                )
                or ""
                for geom_id in (contact.geom1, contact.geom2)
            ]
            if all(name.startswith("rh56_R_") for name in bodies):
                deepest = min(deepest, float(contact.dist))

    assert np.all(np.isfinite(simulation.data.qpos))
    assert np.all(np.isfinite(simulation.data.ctrl))
    for joint_id in range(simulation.model.njnt):
        name = (
            mujoco.mj_id2name(
                simulation.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
            )
            or ""
        )
        if not name.startswith("rh56_R_"):
            continue
        qpos = float(simulation.data.qpos[simulation.model.jnt_qposadr[joint_id]])
        lower, upper = simulation.model.jnt_range[joint_id]
        # MuJoCo joint limits are soft constraints; contact impulses can leave
        # sub-milliradian solver residuals even though every command is clipped.
        assert float(lower) - 1e-3 <= qpos <= float(upper) + 1e-3
    np.testing.assert_allclose(
        simulation.data.ctrl[simulation.hand_actuator_ids],
        (
            RH56_THUMB_LATERAL_RANGE_RAD,
            RH56_THUMB_CLOSE_RANGE_RAD,
            0.0,
            0.0,
            0.0,
            0.0,
        ),
        atol=1e-12,
        rtol=0.0,
    )
    assert deepest > -0.001
