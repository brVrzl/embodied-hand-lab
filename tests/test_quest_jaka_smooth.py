from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import mujoco
import pytest

from motion_input import ReceivedHtsDatagram
from quest_jaka_sim import (
    AnalogClutchSample,
    JakaMujocoSimulation,
    ReplayConfig,
    SharedJakaTargetGenerator,
    SmoothQuestJakaSession,
)
from quest_jaka_sim.simulation import build_viewer_mjcf
from quest_jaka_sim.simulation import build_twin_viewer_mjcf
from tools.quest_jaka_mujoco_sim import _sync_physical_seed_twin_joints


def _payload(sequence: int, *, x: float = 0.0, points=None) -> bytes:
    landmarks = ",".join(
        str(value)
        for point in (points if points is not None else [(0.0, 0.0, 0.0)] * 21)
        for value in point
    )
    return (
        f"Right wrist | f = {sequence}:, {x},0,0,0,0,0,1\n"
        f"Right landmarks | f = {sequence}:, {landmarks}"
    ).encode()


def _datagram(sequence: int, timestamp_ns: int, *, x: float = 0.0, points=None) -> ReceivedHtsDatagram:
    return ReceivedHtsDatagram(
        _payload(sequence, x=x, points=points), "10.24.0.78", 9000, timestamp_ns, timestamp_ns
    )


def _head_datagram(sequence: int, timestamp_ns: int) -> ReceivedHtsDatagram:
    payload = f"Head pose | f = {sequence}:, 0,0,0,0,0,0,1".encode()
    return ReceivedHtsDatagram(payload, "10.24.0.78", 9001, timestamp_ns, timestamp_ns)


def _clutches(session: SmoothQuestJakaSession, value: float, sequence: int, timestamp_ns: int) -> None:
    session.set_clutch_samples(
        index=AnalogClutchSample(value, timestamp_ns, sequence),
        grip=AnalogClutchSample(0.0, timestamp_ns, sequence),
        left_controller_valid=True,
        provider="deterministic_test",
    )


def _dual_clutches(session, index, grip, sequence, timestamp_ns):
    session.set_clutch_samples(
        index=AnalogClutchSample(index, timestamp_ns, sequence),
        grip=AnalogClutchSample(grip, timestamp_ns, sequence),
        left_controller_valid=True,
        provider="deterministic_test",
    )


def _hand_points(fist: bool):
    points = [(0.0, 0.0, 0.0)] * 21
    points[1:5] = [(-0.02, 0.01, 0.0), (-0.03, 0.025, 0.0), (-0.04, 0.04, 0.0), (-0.05, 0.055, 0.0)]
    groups = ((5, 6, 7, 8), (9, 10, 11, 12), (13, 14, 15, 16), (17, 18, 19, 20))
    for x, indices in zip((-0.025, -0.008, 0.010, 0.027), groups, strict=True):
        values = (
            [(x, 0.025, 0.0), (x, 0.050, 0.0), (x + 0.020, 0.050, 0.0), (x + 0.020, 0.025, 0.0)]
            if fist
            else [(x, depth * 0.025, 0.0) for depth in range(1, 5)]
        )
        for joint, value in zip(indices, values, strict=True):
            points[joint] = value
    return points


def _session(tmp_path: Path) -> SmoothQuestJakaSession:
    config = replace(
        ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml"),
        engagement_schedule_s=(),
    )
    model_path = build_viewer_mjcf(config.mjcf_path, tmp_path / "viewer.xml")
    simulation = JakaMujocoSimulation(config, mjcf_path=model_path)
    return SmoothQuestJakaSession(config, simulation)


def test_physical_seed_twin_is_a_complete_offset_robot_model(tmp_path: Path) -> None:
    config = replace(
        ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml"),
        engagement_schedule_s=(),
    )
    model_path = build_twin_viewer_mjcf(
        config.mjcf_path,
        tmp_path / "twin.xml",
        twin_offset_m=0.65,
    )
    simulation = JakaMujocoSimulation(config, mjcf_path=model_path)
    twin = SharedJakaTargetGenerator(config, mjcf_path=config.mjcf_path)
    measured = [1.61381268751, 0.101789525116, -1.51445033884,
                -0.0466143095263, -0.311712070952, 0.0463764459827]
    twin.synchronize_authoritative_arm_joints(measured)
    count = _sync_physical_seed_twin_joints(simulation, twin)
    assert count > 6
    twin_base = mujoco.mj_name2id(
        simulation.model,
        mujoco.mjtObj.mjOBJ_BODY,
        "physical_seed_jaka_Link_0",
    )
    assert twin_base >= 0
    assert simulation.model.body_pos[twin_base] == pytest.approx((0.65, 0.0, 0.0))
    for index, expected in enumerate(measured, start=1):
        joint = mujoco.mj_name2id(
            simulation.model,
            mujoco.mjtObj.mjOBJ_JOINT,
            f"physical_seed_jaka_joint_{index}",
        )
        assert joint >= 0
        assert simulation.data.qpos[simulation.model.jnt_qposadr[joint]] == pytest.approx(expected)
    assert not hasattr(twin, "step")


def test_fixed_rate_reference_stale_disengage_and_no_automatic_recovery(tmp_path: Path) -> None:
    session = _session(tmp_path)
    session.ingest(_datagram(1, 0))
    session.ingest(_head_datagram(1, 0))
    _clutches(session, 0.0, 1, 0)
    session.control_tick(0)
    assert session.arm_clutch.state.value == "disengaged"

    session.ingest(_datagram(2, 33_333_333))
    _clutches(session, 1.0, 2, 33_333_333)
    session.control_tick(33_333_333)
    assert session.arm_clutch.state.value == "engaged"
    assert session.event_records[-1]["operator_delta"]["translation_m"] == (0.0, 0.0, 0.0)

    session.ingest(_datagram(3, 66_666_666, x=0.002))
    session.control_tick(66_666_666)
    assert session.accepted_targets >= 2

    session.control_tick(400_000_000)
    assert session.arm_clutch.state.value == "tracking_fault"
    assert session.arm_mapper.robot_reference is None

    accepted_before = session.accepted_targets
    session.ingest(_datagram(4, 433_333_333, x=0.003))
    _clutches(session, 1.0, 4, 433_333_333)
    session.control_tick(433_333_333)
    assert session.arm_clutch.state.value == "tracking_fault"
    assert session.accepted_targets == accepted_before


def test_smooth_session_is_deterministic_for_same_fixed_ticks(tmp_path: Path) -> None:
    summaries = []
    for name in ("a", "b"):
        session = _session(tmp_path / name)
        for sequence in range(1, 8):
            timestamp = (sequence - 1) * 33_333_333
            session.ingest(_datagram(sequence, timestamp, x=max(0, sequence - 2) * 0.0002))
            if sequence == 1:
                session.ingest(_head_datagram(1, timestamp))
            _clutches(session, 0.0 if sequence == 1 else 1.0, sequence, timestamp)
            session.control_tick(timestamp)
            session.simulation.step(0.033333333)
        summaries.append(
            (
                session.arm_clutch.state.value,
                session.accepted_targets,
                dict(session.rejections),
                session.simulation.last_safe_joint_target.tolist(),
            )
        )
    assert summaries[0] == summaries[1]


def test_independent_index_arm_and_grip_hand_references_are_continuous(tmp_path: Path) -> None:
    session = _session(tmp_path)
    session.ingest(_datagram(1, 0, points=_hand_points(False)))
    session.ingest(_head_datagram(1, 0))
    _dual_clutches(session, 0.0, 0.0, 1, 0)
    session.control_tick(0)

    # Index alone captures arm only and leaves hand frozen.
    session.ingest(_datagram(2, 20_000_000, points=_hand_points(False)))
    _dual_clutches(session, 1.0, 0.0, 2, 20_000_000)
    before = session.simulation.commanded_hand_target.copy()
    session.control_tick(20_000_000)
    assert session.arm_clutch.state.value == "engaged"
    assert session.hand_clutch.state.value == "disengaged"
    assert session.simulation.commanded_hand_target == pytest.approx(before)

    # Grip captures hand independently without changing its press frame.
    session.ingest(_datagram(3, 40_000_000, points=_hand_points(False)))
    _dual_clutches(session, 1.0, 1.0, 3, 40_000_000)
    session.control_tick(40_000_000)
    assert session.arm_clutch.state.value == "engaged"
    assert session.hand_clutch.state.value == "reacquire"
    assert session.simulation.commanded_hand_target == pytest.approx(before)

    # A held grip applies the hand delta while arm remains independent.
    session.ingest(_datagram(4, 60_000_000, points=_hand_points(True)))
    _dual_clutches(session, 1.0, 1.0, 4, 60_000_000)
    session.control_tick(60_000_000)
    assert session.simulation.commanded_hand_target[2] > before[2]
    held = session.simulation.commanded_hand_target.copy()

    # Index release freezes arm only; grip-held hand remains active.
    session.ingest(_datagram(5, 80_000_000, points=_hand_points(True)))
    _dual_clutches(session, 0.0, 1.0, 5, 80_000_000)
    session.control_tick(80_000_000)
    assert session.arm_clutch.state.value == "disengaged"
    assert session.hand_clutch.state.value in {"reacquire", "engaged"}
    held = session.simulation.commanded_hand_target.copy()

    # Grip release freezes hand without changing its last accepted target.
    session.ingest(_datagram(6, 100_000_000, points=_hand_points(False)))
    _dual_clutches(session, 0.0, 0.0, 6, 100_000_000)
    session.control_tick(100_000_000)
    assert session.hand_clutch.state.value == "disengaged"
    assert session.simulation.commanded_hand_target == pytest.approx(held)
