from __future__ import annotations

import copy
from dataclasses import replace
from pathlib import Path

import pytest

from motion_input import (
    AnalogClutchSample,
    HtsRawRecordingReader,
    HtsRawRecordingWriter,
    ReceivedHtsDatagram,
)
from quest_jaka_sim import (
    JakaMujocoSimulation,
    ReplayConfig,
    SharedJakaTargetGenerator,
    SmoothQuestJakaSession,
)
from quest_jaka_sim.output import RecordingArmTargetAdapter
from quest_jaka_sim.live_controller import LiveQuestControllerRouter
from quest_jaka_sim.simulation import build_viewer_mjcf


class _FakeSession:
    def __init__(self) -> None:
        self.frames: list[tuple[AnalogClutchSample, AnalogClutchSample, bool, str]] = []
        self.hand_head: list[ReceivedHtsDatagram] = []

    def ingest(self, datagram: ReceivedHtsDatagram) -> bool:
        self.hand_head.append(datagram)
        return True

    def set_clutch_samples(self, *, index, grip, left_controller_valid, provider) -> None:
        self.frames.append((index, grip, left_controller_valid, provider))


def _ctrl(
    seq: int,
    timestamp_ns: int,
    *,
    session: int = 1,
    index: float = 0.0,
    grip: float = 0.0,
    connected: bool = True,
    active: bool = True,
    tracked: bool = True,
) -> ReceivedHtsDatagram:
    payload = (
        f"CTRL,v=1,session={session},seq={seq},t_ns={timestamp_ns},"
        f"connected={int(connected)},active={int(active)},tracked={int(tracked)},"
        f"index={index:.6f},grip={grip:.6f}\n"
    ).encode()
    return ReceivedHtsDatagram(payload, "10.24.1.99", 50000, timestamp_ns, timestamp_ns)


def _hand(
    sequence: int,
    timestamp_ns: int,
    *,
    x: float = 0.0,
    index_pinch: bool = False,
    middle_closed: bool = False,
) -> ReceivedHtsDatagram:
    points = [(0.0, 0.0, 0.0)] * 21
    points[1:5] = [
        (-0.02, 0.01, 0.0),
        (-0.03, 0.025, 0.0),
        (-0.04, 0.04, 0.0),
        (-0.05, 0.055, 0.0),
    ]
    for finger_x, indices in zip(
        (-0.025, -0.008, 0.010, 0.027),
        ((5, 6, 7, 8), (9, 10, 11, 12), (13, 14, 15, 16), (17, 18, 19, 20)),
        strict=True,
    ):
        for depth, index in enumerate(indices, start=1):
            points[index] = (finger_x, depth * 0.025, 0.0)
    if index_pinch:
        points[5:9] = [
            (-0.025, 0.025, 0.0),
            (-0.025, 0.050, 0.0),
            (-0.005, 0.050, 0.0),
            (-0.005, 0.025, 0.0),
        ]
        points[4] = points[8]
    if middle_closed:
        points[9:13] = [
            (-0.008, 0.025, 0.0),
            (-0.008, 0.050, 0.0),
            (0.012, 0.050, 0.0),
            (0.012, 0.025, 0.0),
        ]
    landmarks = ",".join(str(value) for point in points for value in point)
    payload = (
        f"Right wrist | f = {sequence}:,{x},0,0,0,0,0,1\n"
        f"Right landmarks | f = {sequence}:,{landmarks}"
    ).encode()
    return ReceivedHtsDatagram(payload, "10.24.1.99", 50000, timestamp_ns, timestamp_ns)


def _head(sequence: int, timestamp_ns: int) -> ReceivedHtsDatagram:
    payload = f"Head pose | f = {sequence}:,0,0,0,0,0,0,1".encode()
    return ReceivedHtsDatagram(payload, "10.24.1.99", 50000, timestamp_ns, timestamp_ns)


def _session(
    tmp_path: Path,
    *,
    input_recovery_timeout_s: float | None = None,
) -> SmoothQuestJakaSession:
    config = replace(
        ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml"),
        engagement_schedule_s=(),
    )
    if input_recovery_timeout_s is not None:
        config = replace(
            config,
            input_recovery_timeout_s=input_recovery_timeout_s,
        )
    model = build_viewer_mjcf(config.mjcf_path, tmp_path / "viewer.xml")
    return SmoothQuestJakaSession(config, JakaMujocoSimulation(config, mjcf_path=model))


def test_router_keeps_ctrl_out_of_legacy_hts_ingest() -> None:
    session = _FakeSession()
    router = LiveQuestControllerRouter(stale_after_s=0.15)
    assert router.ingest(_ctrl(1, 1), session).kind == "controller"
    assert session.hand_head == []
    assert session.frames[-1][3] == "quest_ctrl_udp_v1"
    assert router.ingest(_hand(1, 2), session).kind == "hand_head"
    assert len(session.hand_head) == 1


def test_router_requires_both_release_after_new_session() -> None:
    session = _FakeSession()
    router = LiveQuestControllerRouter(stale_after_s=1.0)
    router.ingest(_ctrl(1, 1, session=1, index=1.0), session)
    assert not session.frames[-1][0].valid
    router.ingest(_ctrl(0, 2, session=2, index=1.0), session)
    assert not session.frames[-1][0].valid
    router.ingest(_ctrl(1, 3, session=2), session)
    assert session.frames[-1][0].valid and session.frames[-1][1].valid
    router.ingest(_ctrl(2, 4, session=2, index=1.0), session)
    assert session.frames[-1][0].valid and session.frames[-1][0].value == 1.0


def test_router_stale_and_invalid_facts_publish_both_invalid() -> None:
    session = _FakeSession()
    router = LiveQuestControllerRouter(stale_after_s=0.1)
    router.ingest(_ctrl(1, 1), session)
    router.ingest(_ctrl(2, 2, index=1.0, grip=1.0), session)
    router.poll(100_000_003, session)
    assert not session.frames[-1][2]
    assert not session.frames[-1][0].valid and not session.frames[-1][1].valid
    router.ingest(_ctrl(3, 100_000_004, connected=False), session)
    assert not session.frames[-1][2]


def test_malformed_ctrl_faults_without_entering_hts_parser() -> None:
    session = _FakeSession()
    router = LiveQuestControllerRouter(stale_after_s=1.0)
    malformed = ReceivedHtsDatagram(b"CTRL,v=1,bad\n", "10.24.1.99", 1, 1, 1)
    result = router.ingest(malformed, session)
    assert not result.accepted and result.kind == "controller"
    assert router.malformed_controller_datagrams == 1
    assert session.hand_head == []
    assert not session.frames[-1][2]


def test_live_ctrl_engages_real_sim_session_and_stale_faults_arm(tmp_path: Path) -> None:
    session = _session(tmp_path)
    router = LiveQuestControllerRouter(stale_after_s=0.15)
    router.ingest(_hand(1, 0), session)
    router.ingest(_head(1, 0), session)
    router.ingest(_ctrl(1, 0), session)
    router.poll(0, session)
    session.control_tick(0)
    assert session.arm_clutch.state.value == "disengaged"
    assert session.hand_clutch.state.value == "disengaged"

    timestamp_ns = 33_333_333
    router.ingest(_hand(2, timestamp_ns), session)
    router.ingest(_ctrl(2, timestamp_ns, index=1.0), session)
    router.poll(timestamp_ns, session)
    session.control_tick(timestamp_ns)
    assert session.arm_clutch.state.value == "engaged"
    assert session.hand_clutch.state.value == "disengaged"

    timestamp_ns = 66_666_666
    router.ingest(_hand(3, timestamp_ns), session)
    router.ingest(_ctrl(3, timestamp_ns, grip=1.0), session)
    router.poll(timestamp_ns, session)
    session.control_tick(timestamp_ns)
    assert session.arm_clutch.state.value == "disengaged"
    assert session.hand_clutch.state.value == "reacquire"
    assert session.clutch_provider == "quest_ctrl_udp_v1"

    router.poll(250_000_001, session)
    session.control_tick(250_000_001)
    assert session.arm_clutch.state.value == "tracking_fault"
    assert session.hand_clutch.state.value == "tracking_fault"
    assert session.event_records[-1]["reason"] == "QUEST_INPUT_RECOVERY_HOLD"
    assert session.event_records[-1]["heartbeat_applied"] is True
    assert session.event_records[-1]["control_state"] == "DISENGAGED"

    # Returning data never resumes from a stale reference. Both controls must
    # be released, then a fresh press captures a new reference without restart.
    timestamp_ns = 270_000_000
    router.ingest(_hand(4, timestamp_ns), session)
    router.ingest(_head(2, timestamp_ns), session)
    router.ingest(_ctrl(4, timestamp_ns), session)
    router.poll(timestamp_ns, session)
    session.control_tick(timestamp_ns)
    assert session.arm_clutch.state.value == "disengaged"
    assert session.hand_clutch.state.value == "disengaged"
    assert session.input_recovery_success_count == 1

    timestamp_ns = 300_000_000
    router.ingest(_hand(5, timestamp_ns), session)
    router.ingest(_head(3, timestamp_ns), session)
    router.ingest(_ctrl(5, timestamp_ns, index=1.0), session)
    router.poll(timestamp_ns, session)
    session.control_tick(timestamp_ns)
    assert session.arm_clutch.state.value == "engaged"
    assert session.reference_generation == 2
    assert session._input_recovery_hard_stop_reason is None


def test_input_recovery_timeout_latches_hard_stop(tmp_path: Path) -> None:
    session = _session(tmp_path, input_recovery_timeout_s=0.05)
    router = LiveQuestControllerRouter(stale_after_s=0.15)
    router.ingest(_hand(1, 1), session)
    router.ingest(_head(1, 1), session)
    router.ingest(_ctrl(1, 1), session)
    router.poll(1, session)
    session.control_tick(1)

    timestamp_ns = 20_000_000
    router.ingest(_hand(2, timestamp_ns), session)
    router.ingest(_ctrl(2, timestamp_ns, index=1.0), session)
    router.poll(timestamp_ns, session)
    session.control_tick(timestamp_ns)
    assert session.reference_generation == 1

    router.poll(200_000_000, session)
    session.control_tick(200_000_000)
    assert session.event_records[-1]["reason"] == "QUEST_INPUT_RECOVERY_HOLD"
    assert session.event_records[-1]["heartbeat_applied"] is True

    router.poll(250_000_001, session)
    tick = session.control_tick(250_000_001)
    assert tick.reason == "QUEST_INPUT_RECOVERY_TIMEOUT"
    assert tick.output_applied is False
    assert session.event_records[-1]["control_state"] == "HARD_STOP"
    assert session.input_recovery_timeout_count == 1

    # Fresh input after a terminal timeout cannot clear the hard-stop latch.
    timestamp_ns = 270_000_000
    router.ingest(_hand(3, timestamp_ns), session)
    router.ingest(_head(2, timestamp_ns), session)
    router.ingest(_ctrl(3, timestamp_ns), session)
    router.poll(timestamp_ns, session)
    tick = session.control_tick(timestamp_ns)
    assert tick.reason == "QUEST_INPUT_RECOVERY_TIMEOUT"
    assert tick.output_applied is False


def test_hand_tracking_without_controller_cannot_authorize_motion(tmp_path: Path) -> None:
    session = _session(tmp_path)
    session.ingest(_hand(1, 0))
    session.ingest(_head(1, 0))
    session.control_tick(0)
    assert session.arm_clutch.state.value == "tracking_fault"
    assert session.accepted_targets == 0
    assert session.arm_mapper.robot_reference is None


def test_physical_hand_only_mode_uses_same_router_and_never_generates_arm_target() -> None:
    class HandOutput:
        max_target_normalized = 0.8

        def __init__(self) -> None:
            self.activations = 0
            self.targets: list[tuple[float, ...]] = []
            self.holds: list[str] = []

        def activate_from_measured(self, monotonic_ns: int) -> tuple[float, ...]:
            self.activations += 1
            return (0.3, 0.3, 0.3, 0.3, 0.2, 0.1)

        def submit_target(self, target, monotonic_ns: int) -> None:
            self.targets.append(tuple(target))

        def hold(self, reason: str) -> None:
            self.holds.append(reason)

    config = replace(
        ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml"),
        engagement_schedule_s=(),
    )
    hand = HandOutput()
    arm = RecordingArmTargetAdapter()
    session = SmoothQuestJakaSession(
        config,
        SharedJakaTargetGenerator(config, mjcf_path=config.mjcf_path),
        arm_output=arm,
        normalized_hand_output=hand,
        arm_input_enabled=False,
    )
    router = LiveQuestControllerRouter(stale_after_s=1.0)
    router.ingest(_hand(1, 0), session)
    router.ingest(_head(1, 0), session)
    router.ingest(_ctrl(1, 0), session)
    session.control_tick(0)

    # A large wrist move plus held index remains unable to produce arm motion;
    # grip independently captures measured ANGLE_ACT and becomes active.
    now_ns = 10_000_000
    router.ingest(_hand(2, now_ns, x=0.25), session)
    router.ingest(_ctrl(2, now_ns, index=1.0, grip=1.0), session)
    session.control_tick(now_ns)
    assert hand.activations == 1
    now_ns = 250_000_000
    router.ingest(_hand(3, now_ns, x=0.25), session)
    router.ingest(_ctrl(3, now_ns, index=1.0, grip=1.0), session)
    session.control_tick(now_ns)
    assert hand.targets
    assert arm.targets == []
    assert session.accepted_targets == 0

    now_ns = 260_000_000
    router.ingest(_ctrl(4, now_ns, index=1.0, grip=0.0), session)
    session.control_tick(now_ns)
    assert hand.holds[-1] == "grip_not_active"


def test_physical_hand_grip_realigns_to_current_quest_pose_from_measured_activation() -> None:
    class HandOutput:
        max_target_normalized = 1.0

        def __init__(self) -> None:
            self.activations = 0
            self.targets: list[tuple[float, ...]] = []

        def activate_from_measured(self, monotonic_ns: int) -> tuple[float, ...]:
            self.activations += 1
            return (0.08, 0.12, 0.16, 0.20, 0.24, 0.28)

        def submit_target(self, target, monotonic_ns: int) -> None:
            self.targets.append(tuple(target))

        def hold(self, reason: str) -> None:
            pass

    config = ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml")
    raw = copy.deepcopy(config.raw)
    raw["hand_retargeting"]["align_on_grip"] = True
    config = replace(config, raw=raw, engagement_schedule_s=())
    hand = HandOutput()
    session = SmoothQuestJakaSession(
        config,
        SharedJakaTargetGenerator(config, mjcf_path=config.mjcf_path),
        arm_output=RecordingArmTargetAdapter(),
        normalized_hand_output=hand,
        arm_input_enabled=False,
    )
    router = LiveQuestControllerRouter(stale_after_s=1.0)
    router.ingest(_hand(1, 0), session)
    router.ingest(_ctrl(1, 0), session)
    session.control_tick(0)
    router.ingest(_hand(2, 20_000_000), session)
    router.ingest(_ctrl(2, 20_000_000, grip=1.0), session)
    session.control_tick(20_000_000)
    assert hand.activations == 1
    assert session.hand_clutch.state.value == "reacquire"
    measured_session = (0.28, 0.24, 0.08, 0.12, 0.16, 0.20)
    expected = (
        session.last_hand_result.normalized_targets["thumb_lateral"],
        session.last_hand_result.normalized_targets["thumb_close"],
        session.last_hand_result.normalized_targets["index"],
        session.last_hand_result.normalized_targets["middle"],
        session.last_hand_result.normalized_targets["ring"],
        session.last_hand_result.normalized_targets["pinky"],
    )
    assert session._hand_target_reference == pytest.approx(expected)
    assert session._hand_target_reference != pytest.approx(measured_session)

    router.ingest(_hand(3, 250_000_000), session)
    router.ingest(_ctrl(3, 250_000_000, grip=1.0), session)
    session.control_tick(250_000_000)
    assert hand.targets
    assert hand.targets[-1] != pytest.approx(measured_session)


def test_physical_index_pinch_uses_validated_contact_pose() -> None:
    class HandOutput:
        max_target_normalized = 1.0

        def __init__(self) -> None:
            self.targets: list[tuple[float, ...]] = []

        def activate_from_measured(self, monotonic_ns: int) -> tuple[float, ...]:
            return (0.05, 0.05, 0.05, 0.05, 0.05, 0.05)

        def submit_target(self, target, monotonic_ns: int) -> None:
            self.targets.append(tuple(target))

        def hold(self, reason: str) -> None:
            pass

    config = ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml")
    raw = copy.deepcopy(config.raw)
    raw["hand_retargeting"]["align_on_grip"] = True
    raw["hand_retargeting"]["align_index_pinch_to_validated_pose"] = True
    raw["hand_retargeting"]["calibration_path"] = (
        "configs/hand/quest_rh56_real_retarget.yaml"
    )
    config = replace(config, raw=raw, engagement_schedule_s=())
    hand = HandOutput()
    session = SmoothQuestJakaSession(
        config,
        SharedJakaTargetGenerator(config, mjcf_path=config.mjcf_path),
        arm_output=RecordingArmTargetAdapter(),
        normalized_hand_output=hand,
        arm_input_enabled=False,
    )
    router = LiveQuestControllerRouter(stale_after_s=1.0)
    router.ingest(_hand(1, 0), session)
    router.ingest(_ctrl(1, 0), session)
    session.control_tick(0)
    router.ingest(_hand(2, 20_000_000, index_pinch=True, middle_closed=True), session)
    router.ingest(_ctrl(2, 20_000_000, grip=1.0), session)
    session.control_tick(20_000_000)
    expected_session_triplet = (0.90, 0.40, 0.55)
    expected_canonical_triplet = (0.55, 0.40, 0.90)
    assert session.last_hand_result.pinch_diagnostics["pinch_mode"] == "index"
    assert session._hand_target_reference[:3] == pytest.approx(expected_session_triplet)
    router.ingest(_hand(3, 250_000_000, index_pinch=True, middle_closed=True), session)
    router.ingest(_ctrl(3, 250_000_000, grip=1.0), session)
    session.control_tick(250_000_000)
    assert hand.targets
    assert (
        hand.targets[-1][0],
        hand.targets[-1][4],
        hand.targets[-1][5],
    ) == pytest.approx(expected_canonical_triplet)
    assert hand.targets[-1][1:4] != pytest.approx((0.0, 0.0, 0.0))


def test_combined_session_allows_arm_and_hand_active_from_one_router() -> None:
    class HandOutput:
        max_target_normalized = 0.8

        def __init__(self) -> None:
            self.targets: list[tuple[float, ...]] = []

        def activate_from_measured(self, monotonic_ns: int) -> tuple[float, ...]:
            return (0.2, 0.2, 0.2, 0.2, 0.2, 0.2)

        def submit_target(self, target, monotonic_ns: int) -> None:
            self.targets.append(tuple(target))

        def hold(self, reason: str) -> None:
            pass

    config = replace(
        ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml"),
        engagement_schedule_s=(),
    )
    hand = HandOutput()
    arm = RecordingArmTargetAdapter()
    session = SmoothQuestJakaSession(
        config,
        SharedJakaTargetGenerator(config, mjcf_path=config.mjcf_path),
        arm_output=arm,
        normalized_hand_output=hand,
    )
    router = LiveQuestControllerRouter(stale_after_s=1.0)
    router.ingest(_hand(1, 0), session)
    router.ingest(_head(1, 0), session)
    router.ingest(_ctrl(1, 0), session)
    session.control_tick(0)
    now_ns = 10_000_000
    router.ingest(_hand(2, now_ns), session)
    router.ingest(_head(2, now_ns), session)
    router.ingest(_ctrl(2, now_ns, index=1.0, grip=1.0), session)
    session.control_tick(now_ns)
    now_ns = 250_000_000
    router.ingest(_hand(3, now_ns), session)
    router.ingest(_head(3, now_ns), session)
    router.ingest(_ctrl(3, now_ns, index=1.0, grip=1.0), session)
    session.control_tick(now_ns)
    assert arm.targets
    assert hand.targets
    assert session.arm_clutch.state.value == "engaged"
    assert session.hand_clutch.state.value == "engaged"


def test_recorded_index_and_grip_replay_through_the_live_router_together(
    tmp_path: Path,
) -> None:
    recording = tmp_path / "joint-clutch.hts.jsonl"
    packets = (
        _hand(1, 1),
        _head(1, 1),
        _ctrl(1, 1),
        _hand(2, 33_333_334),
        _ctrl(2, 33_333_334, index=1.0, grip=1.0),
    )
    with HtsRawRecordingWriter(recording) as writer:
        for packet in packets:
            writer.write(packet)

    session = _session(tmp_path)
    router = LiveQuestControllerRouter(stale_after_s=0.15)
    replayed = list(HtsRawRecordingReader(recording).datagrams())
    for packet in replayed[:3]:
        router.ingest(packet, session)
    router.poll(1, session)
    session.control_tick(1)
    for packet in replayed[3:]:
        router.ingest(packet, session)
    router.poll(33_333_334, session)
    session.control_tick(33_333_334)

    assert session.clutch_provider == "quest_ctrl_udp_v1"
    assert session.arm_clutch.state.value == "engaged"
    assert session.hand_clutch.state.value == "reacquire"
    assert session.event_records[-1]["arm_reference_capture"]
    assert session.event_records[-1]["hand_reference_capture"]
