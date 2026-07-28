from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from motion_input import AnalogClutchSample, ReceivedHtsDatagram
from quest_jaka_sim import JakaMujocoSimulation, ReplayConfig, SmoothQuestJakaSession
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


def _hand(sequence: int, timestamp_ns: int, *, x: float = 0.0) -> ReceivedHtsDatagram:
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
    landmarks = ",".join(str(value) for point in points for value in point)
    payload = (
        f"Right wrist | f = {sequence}:,{x},0,0,0,0,0,1\n"
        f"Right landmarks | f = {sequence}:,{landmarks}"
    ).encode()
    return ReceivedHtsDatagram(payload, "10.24.1.99", 50000, timestamp_ns, timestamp_ns)


def _head(sequence: int, timestamp_ns: int) -> ReceivedHtsDatagram:
    payload = f"Head pose | f = {sequence}:,0,0,0,0,0,0,1".encode()
    return ReceivedHtsDatagram(payload, "10.24.1.99", 50000, timestamp_ns, timestamp_ns)


def _session(tmp_path: Path) -> SmoothQuestJakaSession:
    config = replace(
        ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml"),
        engagement_schedule_s=(),
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


def test_hand_tracking_without_controller_cannot_authorize_motion(tmp_path: Path) -> None:
    session = _session(tmp_path)
    session.ingest(_hand(1, 0))
    session.ingest(_head(1, 0))
    session.control_tick(0)
    assert session.arm_clutch.state.value == "tracking_fault"
    assert session.accepted_targets == 0
    assert session.arm_mapper.robot_reference is None
