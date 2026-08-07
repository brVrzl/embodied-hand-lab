from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from motion_input import AnalogClutchSample, ReceivedHtsDatagram
from quest_jaka_sim import JakaMujocoSimulation, ReplayConfig, SmoothQuestJakaSession
from quest_jaka_sim.hand_retarget import (
    RH56_THUMB_CLOSE_RANGE_RAD,
    RH56_THUMB_LATERAL_RANGE_RAD,
)
from quest_jaka_sim.simulation import build_viewer_mjcf


FINGER_NAMES = ("index", "middle", "ring", "pinky")
FINGER_GROUPS = {
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}
HAND_ORDER = ("thumb_lateral", "thumb_close", *FINGER_NAMES)
HAND_LIMITS = np.asarray(
    (RH56_THUMB_LATERAL_RANGE_RAD, RH56_THUMB_CLOSE_RANGE_RAD, 1.70, 1.68, 1.70, 1.70)
)


def _points(
    *,
    closed: set[str] | None = None,
    thumb: str = "open",
    thumb_lateral_raw: float | None = None,
) -> list[tuple[float, float, float]]:
    closed = set() if closed is None else closed
    points = [(0.0, 0.0, 0.0)] * 21
    points[1:5] = [
        (-0.02, 0.01, 0.0),
        (-0.03, 0.025, 0.0),
        (-0.04, 0.04, 0.0),
        (-0.05, 0.055, 0.0),
    ]
    for name, x in zip(FINGER_NAMES, (-0.025, -0.008, 0.010, 0.027), strict=True):
        values = (
            [(x, 0.025, 0.0), (x, 0.050, 0.0), (x + 0.020, 0.050, 0.0), (x + 0.020, 0.025, 0.0)]
            if name in closed
            else [(x, depth * 0.025, 0.0) for depth in range(1, 5)]
        )
        for joint, value in zip(FINGER_GROUPS[name], values, strict=True):
            points[joint] = value
    if thumb_lateral_raw is not None:
        if thumb != "open":
            raise ValueError("thumb bend/pinch and lateral sweep are separate fixtures")
        base = np.asarray(points[1], dtype=float)
        palm_width = float(
            np.linalg.norm(np.asarray(points[17]) - np.asarray(points[5]))
        )
        tip_delta = np.asarray(
            (thumb_lateral_raw * palm_width, 0.060, 0.0),
            dtype=float,
        )
        points[1:5] = [
            tuple((base + fraction * tip_delta).tolist())
            for fraction in (0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0)
        ]
    elif thumb == "bend":
        points[1:5] = [
            (-0.02, 0.01, 0.0),
            (-0.035, 0.020, 0.0),
            (-0.050, 0.020, 0.015),
            (-0.050, 0.035, 0.015),
        ]
    elif thumb == "pinch":
        points[4] = points[8]
    elif thumb != "open":
        raise ValueError(f"unknown thumb pose {thumb!r}")
    return points


def _datagram(sequence: int, timestamp_ns: int, points, *, wrist_x: float = 0.0) -> ReceivedHtsDatagram:
    landmarks = ",".join(str(value) for point in points for value in point)
    payload = (
        f"Right wrist | f = {sequence}:,{wrist_x},0,0,0,0,0,1\n"
        f"Right landmarks | f = {sequence}:,{landmarks}"
    ).encode()
    return ReceivedHtsDatagram(payload, "127.0.0.1", 9000, timestamp_ns, timestamp_ns)


def _head(sequence: int, timestamp_ns: int) -> ReceivedHtsDatagram:
    return ReceivedHtsDatagram(
        f"Head pose | f = {sequence}:,0,0,0,0,0,0,1".encode(),
        "127.0.0.1",
        9000,
        timestamp_ns,
        timestamp_ns,
    )


def _session(tmp_path: Path) -> SmoothQuestJakaSession:
    config = replace(
        ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml"),
        engagement_schedule_s=(),
    )
    model_path = build_viewer_mjcf(config.mjcf_path, tmp_path / "h2.xml")
    return SmoothQuestJakaSession(config, JakaMujocoSimulation(config, mjcf_path=model_path))


def _clutches(
    session: SmoothQuestJakaSession,
    *,
    index: float,
    grip: float,
    sequence: int,
    timestamp_ns: int,
    index_valid: bool = True,
    grip_valid: bool = True,
) -> None:
    session.set_clutch_samples(
        index=AnalogClutchSample(index, timestamp_ns, sequence, valid=index_valid),
        grip=AnalogClutchSample(grip, timestamp_ns, sequence, valid=grip_valid),
        left_controller_valid=True,
        provider="h2_dual_clutch_test",
    )


def _tick(
    session: SmoothQuestJakaSession,
    sequence: int,
    timestamp_ns: int,
    *,
    points,
    index: float,
    grip: float,
    wrist_x: float = 0.0,
    head: bool = False,
) -> object:
    session.ingest(_datagram(sequence, timestamp_ns, points, wrist_x=wrist_x))
    if head:
        session.ingest(_head(sequence, timestamp_ns))
    _clutches(session, index=index, grip=grip, sequence=sequence, timestamp_ns=timestamp_ns)
    return session.control_tick(timestamp_ns)


def test_dual_clutch_truth_table_and_press_ticks_are_continuous(tmp_path: Path) -> None:
    session = _session(tmp_path)
    open_points = _points()
    initial = session.simulation.commanded_hand_target.copy()

    # index=0, grip=0: both frozen.
    _tick(session, 1, 0, points=open_points, index=0.0, grip=0.0, head=True)
    assert session.arm_clutch.state.value == "disengaged"
    assert session.hand_clutch.state.value == "disengaged"
    assert session.accepted_targets == 0
    assert session.simulation.commanded_hand_target == pytest.approx(initial)

    # index=1, grip=0: arm capture only; neither target jumps on press.
    arm_press = _tick(session, 2, 20_000_000, points=open_points, index=1.0, grip=0.0)
    assert session.arm_clutch.state.value == "engaged"
    assert session.hand_clutch.state.value == "disengaged"
    assert arm_press.accepted_target is not None
    assert session.simulation.commanded_hand_target == pytest.approx(initial)
    arm_accepts = session.accepted_targets

    # index=0, grip=1: arm freezes; hand captures independently without jump.
    _tick(session, 3, 40_000_000, points=open_points, index=0.0, grip=1.0)
    assert session.arm_clutch.state.value == "disengaged"
    assert session.hand_clutch.state.value == "reacquire"
    assert session.accepted_targets == arm_accepts
    assert session.simulation.commanded_hand_target == pytest.approx(initial)
    assert session._four_finger_feature_reference is not None
    assert session._thumb_close_feature_reference is not None
    assert session._thumb_lateral_feature_reference is not None

    # index=1, grip=1: both are active and their reference lifecycles remain separate.
    result = _tick(
        session, 4, 60_000_000, points=_points(closed={"index"}),
        index=1.0, grip=1.0, wrist_x=0.004, head=True,
    )
    assert session.arm_clutch.state.value == "engaged"
    assert session.hand_clutch.state.value in {"reacquire", "engaged"}
    assert result.accepted_target is not None
    assert session.simulation.commanded_hand_target[2] > initial[2]


@pytest.mark.parametrize("finger", FINGER_NAMES)
def test_grip_held_four_fingers_are_independent_and_index_does_not_control_hand(tmp_path: Path, finger: str) -> None:
    session = _session(tmp_path)
    open_points = _points()
    _tick(session, 1, 0, points=open_points, index=0.0, grip=0.0, head=True)
    _tick(session, 2, 20_000_000, points=open_points, index=0.0, grip=1.0)
    before = session.simulation.commanded_hand_target.copy()
    _tick(session, 3, 40_000_000, points=_points(closed={finger}), index=0.0, grip=1.0)
    after = session.simulation.commanded_hand_target.copy()
    active = 2 + FINGER_NAMES.index(finger)
    assert after[active] > before[active]
    for index in range(2, 6):
        if index != active:
            assert after[index] == pytest.approx(before[index])
    assert after[:2] == pytest.approx(before[:2])
    assert np.all(np.isfinite(after))


def test_thumb_close_is_relative_monotonic_with_small_bend_only_lateral_crosstalk(
    tmp_path: Path,
) -> None:
    session = _session(tmp_path)
    initial = np.asarray((0.30, 0.10, 0.0, 0.0, 0.0, 0.0))
    session.simulation.set_hand_actuator_target(dict(zip(HAND_ORDER, initial.tolist(), strict=True)))
    open_points = _points()
    _tick(session, 1, 0, points=open_points, index=0.0, grip=0.0, head=True)
    _tick(session, 2, 20_000_000, points=open_points, index=0.0, grip=1.0)
    assert session.simulation.commanded_hand_target == pytest.approx(initial)
    assert session._thumb_close_feature_reference is not None

    values = []
    for sequence in range(3, 10):
        _tick(
            session,
            sequence,
            sequence * 20_000_000,
            points=_points(thumb="bend"),
            index=0.0,
            grip=1.0,
        )
        values.append(float(session.simulation.commanded_hand_target[1]))
    assert values == sorted(values)
    assert values[-1] > initial[1]
    assert -1e-12 <= values[-1] <= RH56_THUMB_CLOSE_RANGE_RAD + 1e-12
    assert session.simulation.commanded_hand_target[0] == pytest.approx(
        initial[0], abs=0.03
    )
    debug = session.event_records[-1]["thumb_close_debug"]
    assert debug["raw_thumb_bend_rad"] is not None
    assert 0.0 <= debug["normalized_thumb_bend"] <= 1.0
    assert debug["raw_pinch_distance_m"] >= 0.0
    assert debug["raw_pinch_distance_palm"] >= 0.0
    assert 0.0 <= debug["normalized_pinch"] <= 1.0
    assert debug["base_bend_contribution"] >= 0.0
    assert debug["pinch_assist_contribution"] >= 0.0
    assert 0.0 <= debug["combined_feature_normalized"] <= 1.0
    assert debug["captured_feature_reference_rad"] is not None
    assert debug["feature_delta_rad"] is not None
    assert debug["captured_rh56_reference_rad"] == pytest.approx(initial[1])
    assert debug["requested_target_rad"] is not None
    assert -1e-12 <= debug["clipped_target_rad"] <= RH56_THUMB_CLOSE_RANGE_RAD + 1e-12
    assert debug["actual_mujoco_joint_rad"] is not None
    assert debug["joint_range_rad"] == pytest.approx(
        (0.0, RH56_THUMB_CLOSE_RANGE_RAD)
    )
    assert debug["ctrl_range_rad"] == pytest.approx(
        (0.0, RH56_THUMB_CLOSE_RANGE_RAD)
    )
    assert debug["valid_range_rad"] == pytest.approx(
        (0.0, RH56_THUMB_CLOSE_RANGE_RAD)
    )


def test_lateral_only_is_monotonic_and_does_not_change_thumb_close(
    tmp_path: Path,
) -> None:
    session = _session(tmp_path)
    initial = np.asarray((0.25, 0.12, 0.0, 0.0, 0.0, 0.0))
    session.simulation.set_hand_actuator_target(
        dict(zip(HAND_ORDER, initial.tolist(), strict=True))
    )
    reference = _points(thumb_lateral_raw=-0.60)
    _tick(session, 1, 0, points=reference, index=0.0, grip=0.0, head=True)
    _tick(session, 2, 20_000_000, points=reference, index=0.0, grip=1.0)
    assert session._thumb_lateral_feature_reference is not None
    assert session.simulation.commanded_hand_target == pytest.approx(initial)

    lateral_values = []
    for sequence, raw in enumerate((-0.40, -0.20, 0.0, 0.20, 0.25), start=3):
        _tick(
            session,
            sequence,
            sequence * 20_000_000,
            points=_points(thumb_lateral_raw=raw),
            index=0.0,
            grip=1.0,
        )
        after = session.simulation.commanded_hand_target.copy()
        lateral_values.append(float(after[0]))
        assert 0.0 <= after[0] <= RH56_THUMB_LATERAL_RANGE_RAD
        assert after[1] == pytest.approx(initial[1])
        assert after[2:] == pytest.approx(initial[2:])
        assert np.all(np.isfinite(after))
    assert lateral_values == sorted(lateral_values)
    assert lateral_values[-1] > initial[0]
    debug = session.event_records[-1]["thumb_lateral_debug"]
    assert debug["feature_normalized"] == pytest.approx(1.0)
    assert debug["feature_delta"] > 0.0
    assert debug["joint_range_rad"] == pytest.approx(
        (0.0, RH56_THUMB_LATERAL_RANGE_RAD)
    )
    assert debug["ctrl_range_rad"] == pytest.approx(
        (0.0, RH56_THUMB_LATERAL_RANGE_RAD)
    )


def test_thumb_lateral_clips_at_lower_range_without_nan(tmp_path: Path) -> None:
    session = _session(tmp_path)
    initial = np.asarray((0.01, 0.12, 0.0, 0.0, 0.0, 0.0))
    session.simulation.set_hand_actuator_target(
        dict(zip(HAND_ORDER, initial.tolist(), strict=True))
    )
    opposed = _points(thumb_lateral_raw=0.25)
    _tick(session, 1, 0, points=opposed, index=0.0, grip=0.0, head=True)
    _tick(session, 2, 20_000_000, points=opposed, index=0.0, grip=1.0)
    session.thumb_lateral_gain = 100.0
    for sequence in range(3, 30):
        _tick(
            session,
            sequence,
            sequence * 20_000_000,
            points=_points(thumb_lateral_raw=-0.60),
            index=0.0,
            grip=1.0,
        )
    target = session.simulation.commanded_hand_target
    assert np.all(np.isfinite(target))
    assert target[0] == pytest.approx(0.0)
    assert target[1:] == pytest.approx(initial[1:])
    debug = session.event_records[-1]["thumb_lateral_debug"]
    assert debug["clipped_target_rad"] == pytest.approx(0.0)
    assert debug["saturation"]


def test_release_reengage_and_single_landmark_loss_hold_hand_without_stopping_arm(tmp_path: Path) -> None:
    session = _session(tmp_path)
    _tick(session, 1, 0, points=_points(), index=0.0, grip=0.0, head=True)
    _tick(session, 2, 20_000_000, points=_points(), index=1.0, grip=1.0)
    _tick(session, 3, 40_000_000, points=_points(closed={"middle"}, thumb="pinch"), index=1.0, grip=1.0)
    moved = session.simulation.commanded_hand_target.copy()

    state = session.assembler.state(now_monotonic_ns=60_000_000)
    session.ingest_shared_state(replace(state, right=replace(state.right, joints=())))
    _clutches(session, index=1.0, grip=1.0, sequence=4, timestamp_ns=60_000_000)
    arm_result = session.control_tick(60_000_000)
    assert session.arm_clutch.state.value == "engaged"
    assert arm_result.accepted_target is not None
    assert session.simulation.commanded_hand_target == pytest.approx(moved)

    # Grip release freezes only hand; index-held arm remains active.
    session.ingest_shared_state(replace(state, host_monotonic_ns=80_000_000))
    _clutches(session, index=1.0, grip=0.0, sequence=5, timestamp_ns=80_000_000)
    result = session.control_tick(80_000_000)
    assert result.accepted_target is not None
    assert session.arm_clutch.state.value == "engaged"
    assert session.hand_clutch.state.value == "disengaged"
    assert session.simulation.commanded_hand_target == pytest.approx(moved)

    # Re-engaging grip at a new absolute hand pose captures reference, so its
    # press tick remains continuous and does not alter the active arm target.
    session.ingest_shared_state(replace(state, host_monotonic_ns=100_000_000))
    _clutches(session, index=1.0, grip=1.0, sequence=6, timestamp_ns=100_000_000)
    result = session.control_tick(100_000_000)
    assert result.accepted_target is not None
    assert session.simulation.commanded_hand_target == pytest.approx(moved)


@pytest.mark.parametrize(
    ("index_valid", "grip_valid", "arm_state", "hand_state"),
    (
        (False, True, "tracking_fault", {"reacquire", "engaged"}),
        (True, False, "engaged", {"tracking_fault"}),
    ),
)
def test_per_channel_stale_sample_faults_only_its_own_clutch(
    tmp_path: Path,
    index_valid: bool,
    grip_valid: bool,
    arm_state: str,
    hand_state: set[str],
) -> None:
    session = _session(tmp_path)
    _tick(session, 1, 0, points=_points(), index=0.0, grip=0.0, head=True)
    _tick(session, 2, 20_000_000, points=_points(), index=1.0, grip=1.0)
    _tick(session, 3, 40_000_000, points=_points(closed={"ring"}), index=1.0, grip=1.0)
    held = session.simulation.commanded_hand_target.copy()
    session.ingest(_datagram(4, 60_000_000, _points(closed={"ring"})))
    _clutches(
        session,
        index=1.0,
        grip=1.0,
        sequence=4,
        timestamp_ns=60_000_000,
        index_valid=index_valid,
        grip_valid=grip_valid,
    )
    session.control_tick(60_000_000)
    assert session.arm_clutch.state.value == arm_state
    assert session.hand_clutch.state.value in hand_state
    if grip_valid:
        assert session.simulation.commanded_hand_target[4] >= held[4]
    else:
        assert session.simulation.commanded_hand_target == pytest.approx(held)


def test_wrist_only_loss_freezes_arm_but_valid_skeleton_keeps_hand_active(tmp_path: Path) -> None:
    session = _session(tmp_path)
    _tick(session, 1, 0, points=_points(), index=0.0, grip=0.0, head=True)
    _tick(session, 2, 20_000_000, points=_points(), index=1.0, grip=1.0)
    _tick(session, 3, 40_000_000, points=_points(closed={"pinky"}), index=1.0, grip=1.0)
    state = session.assembler.state(now_monotonic_ns=60_000_000)
    session.ingest_shared_state(replace(state, right=replace(state.right, wrist_pose=None)))
    _clutches(session, index=1.0, grip=1.0, sequence=4, timestamp_ns=60_000_000)
    session.control_tick(60_000_000)
    assert session.arm_clutch.state.value == "tracking_fault"
    assert session.hand_clutch.state.value in {"reacquire", "engaged"}
    assert session.event_records[-1]["hand_command_updated"]


def test_degenerate_palm_frame_holds_entire_hand_target(tmp_path: Path) -> None:
    session = _session(tmp_path)
    _tick(session, 1, 0, points=_points(), index=0.0, grip=0.0, head=True)
    _tick(session, 2, 20_000_000, points=_points(), index=0.0, grip=1.0)
    _tick(
        session,
        3,
        40_000_000,
        points=_points(closed={"index"}, thumb_lateral_raw=0.0),
        index=0.0,
        grip=1.0,
    )
    held = session.simulation.commanded_hand_target.copy()
    degenerate = _points(closed={"middle"})
    degenerate[17] = degenerate[5]
    _tick(
        session,
        4,
        60_000_000,
        points=degenerate,
        index=0.0,
        grip=1.0,
    )
    assert session.last_hand_result is not None
    assert session.last_hand_result.rejection_reason == "DEGENERATE_PALM_FRAME"
    assert session.simulation.commanded_hand_target == pytest.approx(held)


def test_h2_clips_all_enabled_hand_channels_and_has_no_nan(tmp_path: Path) -> None:
    session = _session(tmp_path)
    initial = np.asarray((0.30, 0.49, 1.69, 1.67, 1.69, 1.69))
    session.simulation.set_hand_actuator_target(dict(zip(HAND_ORDER, initial.tolist(), strict=True)))
    _tick(session, 1, 0, points=_points(), index=0.0, grip=0.0, head=True)
    _tick(session, 2, 20_000_000, points=_points(), index=0.0, grip=1.0)
    session.four_finger_gain = 100.0
    session.thumb_close_gain = 100.0
    session.thumb_lateral_gain = 100.0
    for sequence in range(3, 80):
        _tick(
            session, sequence, sequence * 20_000_000,
            points=_points(closed=set(FINGER_NAMES), thumb="pinch"), index=0.0, grip=1.0,
        )
    target = session.simulation.commanded_hand_target
    assert np.all(np.isfinite(target))
    assert np.all(target >= 0.0)
    assert np.all(target <= HAND_LIMITS + 1e-12)
    assert target == pytest.approx(HAND_LIMITS)
