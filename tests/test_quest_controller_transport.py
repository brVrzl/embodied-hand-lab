from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import subprocess
import sys

import pytest

from motion_input import (
    AnalogClutchSample,
    ArmClutchMachine,
    ArmClutchState,
    ClutchAction,
    ControllerClutchAdapter,
    ControllerPacket,
    ControllerPacketError,
    ControllerProvider,
    HandClutchMachine,
    HandClutchState,
    QuestTransportKind,
    SerializationError,
    TransportClutchMonitor,
    parse_controller_datagram,
    parse_controller_line,
    parse_hts_datagram,
    parse_quest_transport_datagram,
)
from motion_input.hts_transport import ReceivedHtsDatagram
from motion_input.transport_gate import QuestTransportGate


VALID_LINE = (
    "CTRL,v=1,session=987654321,seq=123,t_ns=123456789012345,"
    "connected=1,active=1,tracked=1,index=0.123456,grip=0.654321"
)


def packet(
    *,
    session: int = 1,
    seq: int = 1,
    t_ns: int = 100,
    connected: bool = True,
    active: bool = True,
    tracked: bool = True,
    index: float = 0.0,
    grip: float = 0.0,
) -> ControllerPacket:
    return ControllerPacket(
        1, session, seq, t_ns, connected, active, tracked, index, grip
    )


def line_for(value: ControllerPacket) -> str:
    return (
        f"CTRL,v=1,session={value.session_id},seq={value.sequence_number},"
        f"t_ns={value.source_timestamp_ns},connected={int(value.connected)},"
        f"active={int(value.active)},tracked={int(value.tracked)},"
        f"index={value.index_trigger:.6f},grip={value.grip_trigger:.6f}"
    )


def datagram(payload: str | bytes, receive_ns: int) -> ReceivedHtsDatagram:
    return ReceivedHtsDatagram(
        payload.encode() if isinstance(payload, str) else payload,
        "10.24.1.99",
        50000,
        receive_ns,
        1_800_000_000_000_000_000 + receive_ns,
    )


def hand_payload() -> str:
    landmarks = ",".join(str(index / 1000.0) for index in range(63))
    return (
        "Right wrist | f = 1 | t = 10:,0,0,0,0,0,0,1\n"
        f"Right landmarks | f = 1 | t = 10:,{landmarks}\n"
    )


def head_payload() -> str:
    return "Head pose | f = 1 | t = 10:,0,0,0,0,0,0,1\n"


def monitor_step(
    provider: ControllerProvider,
    adapter: ControllerClutchAdapter,
    monitor: TransportClutchMonitor,
    value: ControllerPacket,
    now_ns: int,
):
    state = provider.update(value, host_receive_monotonic_ns=now_ns)
    frame = adapter.samples(state)
    return monitor.update(
        frame,
        now_monotonic_ns=now_ns,
        stale_after_ns=provider.stale_after_ns,
    )


# Parser coverage (requirements 1--12 plus UTF-8/trailing-content cases).
def test_valid_ctrl_decode_preserves_all_fields() -> None:
    parsed = parse_controller_datagram((VALID_LINE + "\n").encode())
    assert parsed == packet(
        session=987654321,
        seq=123,
        t_ns=123456789012345,
        index=0.123456,
        grip=0.654321,
    )


def test_fixed_field_order_is_required() -> None:
    swapped = VALID_LINE.replace(
        "connected=1,active=1", "active=1,connected=1"
    )
    with pytest.raises(ControllerPacketError, match="order mismatch"):
        parse_controller_line(swapped)


def test_wrong_version_is_rejected() -> None:
    with pytest.raises(ControllerPacketError, match="version"):
        parse_controller_line(VALID_LINE.replace("v=1", "v=2", 1))


def test_missing_field_is_rejected() -> None:
    with pytest.raises(ControllerPacketError, match="exactly nine"):
        parse_controller_line(VALID_LINE.replace(",grip=0.654321", ""))


def test_duplicate_field_is_rejected() -> None:
    duplicate = VALID_LINE.replace("grip=0.654321", "index=0.654321")
    with pytest.raises(ControllerPacketError, match="duplicate"):
        parse_controller_line(duplicate)


def test_unknown_field_is_rejected() -> None:
    unknown = VALID_LINE.replace("grip=0.654321", "squeeze=0.654321")
    with pytest.raises(ControllerPacketError, match="unknown"):
        parse_controller_line(unknown)


def test_invalid_boolean_is_rejected() -> None:
    with pytest.raises(ControllerPacketError, match="must be 0 or 1"):
        parse_controller_line(VALID_LINE.replace("tracked=1", "tracked=true"))


@pytest.mark.parametrize(
    ("field", "current", "bad"),
    [
        ("session", "987654321", "-1"),
        ("seq", "123", "1.5"),
        ("t_ns", "123456789012345", "abc"),
        ("session", "987654321", str(1 << 64)),
    ],
)
def test_invalid_uint64_is_rejected(field: str, current: str, bad: str) -> None:
    with pytest.raises(ControllerPacketError):
        parse_controller_line(VALID_LINE.replace(f"{field}={current}", f"{field}={bad}"))


@pytest.mark.parametrize("bad", ["nan", "inf"])
def test_nan_and_inf_are_rejected(bad: str) -> None:
    with pytest.raises(ControllerPacketError, match="finite float"):
        parse_controller_line(VALID_LINE.replace("index=0.123456", f"index={bad}"))


@pytest.mark.parametrize("bad", ["-0.0001", "1.0001"])
def test_out_of_range_analog_is_rejected(bad: str) -> None:
    with pytest.raises(ControllerPacketError):
        parse_controller_line(VALID_LINE.replace("grip=0.654321", f"grip={bad}"))


@pytest.mark.parametrize(
    "bad",
    ["CTRL, v=1,session=1,seq=1,t_ns=1,connected=1,active=1,tracked=1,index=0,grip=0"],
)
def test_malformed_ctrl_is_rejected_without_whitespace_normalization(bad: str) -> None:
    with pytest.raises(ControllerPacketError):
        parse_controller_line(bad)


def test_unknown_packet_is_not_misparsed_as_ctrl() -> None:
    with pytest.raises(SerializationError):
        parse_quest_transport_datagram(b"BUTTON,index=1\n")


def test_legacy_hand_head_parser_is_unchanged_and_skips_ctrl() -> None:
    original = parse_hts_datagram(hand_payload().encode())
    assert len(original) == 2
    assert parse_hts_datagram((VALID_LINE + "\n").encode()) == ()
    following = parse_hts_datagram(head_payload().encode())
    assert len(following) == 1


def test_transport_dispatch_separates_ctrl_and_hand_head() -> None:
    ctrl = parse_quest_transport_datagram((VALID_LINE + "\n").encode())
    hand = parse_quest_transport_datagram(hand_payload().encode())
    assert ctrl.kind is QuestTransportKind.CONTROLLER and ctrl.controller is not None
    assert hand.kind is QuestTransportKind.HAND_HEAD and len(hand.hand_head) == 2


def test_malformed_utf8_and_nonempty_trailing_content_are_rejected() -> None:
    with pytest.raises(ControllerPacketError, match="UTF-8"):
        parse_controller_datagram(b"CTRL,\xff")
    with pytest.raises(ControllerPacketError, match="exactly one"):
        parse_controller_datagram((VALID_LINE + "\nnot-empty\n").encode())


# Provider/session coverage (requirements 13--25).
def test_provider_fresh_valid_sample_uses_host_receive_clock() -> None:
    provider = ControllerProvider(stale_after_s=0.25)
    state = provider.update(
        packet(t_ns=(1 << 63), index=0.2), host_receive_monotonic_ns=1_000_000_000
    )
    assert state.controller_valid and not state.stale
    assert provider.snapshot(now_monotonic_ns=1_200_000_000).sample_age_ns == 200_000_000


def test_provider_stale_is_host_monotonic_only() -> None:
    provider = ControllerProvider(stale_after_s=0.25)
    provider.update(packet(t_ns=9_999_999_999_999), host_receive_monotonic_ns=10)
    assert not provider.snapshot(now_monotonic_ns=250_000_010).stale
    state = provider.snapshot(now_monotonic_ns=250_000_011)
    assert state.stale and not state.controller_valid and state.stale_event_count == 1


def test_sequence_gap_is_counted_and_forward_packet_accepted() -> None:
    provider = ControllerProvider()
    provider.update(packet(seq=4), host_receive_monotonic_ns=1)
    state = provider.update(packet(seq=8, t_ns=200), host_receive_monotonic_ns=2)
    assert state.latest and state.latest.sequence_number == 8
    assert state.sequence_gap == 3
    assert state.gap_event_count == 1 and state.missing_sequence_count == 3


def test_duplicate_does_not_override_or_refresh_latest() -> None:
    provider = ControllerProvider()
    provider.update(packet(seq=4, index=0.0), host_receive_monotonic_ns=100)
    state = provider.update(packet(seq=4, index=1.0), host_receive_monotonic_ns=200)
    assert state.duplicate_count == 1
    assert state.latest and state.latest.index_trigger == 0.0
    assert state.host_receive_monotonic_ns == 100


def test_reordered_sequence_does_not_override_latest() -> None:
    provider = ControllerProvider()
    provider.update(packet(seq=4), host_receive_monotonic_ns=100)
    state = provider.update(packet(seq=3, index=1.0), host_receive_monotonic_ns=200)
    assert state.reorder_count == 1
    assert state.latest and state.latest.sequence_number == 4


def test_new_session_accepts_sequence_restart() -> None:
    provider = ControllerProvider()
    provider.update(packet(session=11, seq=99), host_receive_monotonic_ns=100)
    state = provider.update(packet(session=12, seq=0), host_receive_monotonic_ns=200)
    assert state.latest and state.latest.session_id == 12
    assert state.session_count == 2 and state.new_session_count == 1
    assert state.active_fault == "new_session"


def test_stale_then_new_session_is_allowed() -> None:
    provider = ControllerProvider(stale_after_s=0.1)
    provider.update(packet(session=11, seq=99), host_receive_monotonic_ns=100)
    assert provider.snapshot(now_monotonic_ns=100_000_101).stale
    state = provider.update(packet(session=12, seq=0), host_receive_monotonic_ns=100_000_102)
    assert state.controller_valid and state.latest and state.latest.session_id == 12


def test_old_session_delayed_packet_cannot_override_current() -> None:
    provider = ControllerProvider()
    provider.update(packet(session=11, seq=9), host_receive_monotonic_ns=100)
    provider.update(packet(session=12, seq=0), host_receive_monotonic_ns=200)
    state = provider.update(
        packet(session=11, seq=10, index=1.0), host_receive_monotonic_ns=300
    )
    assert state.latest and state.latest.session_id == 12
    assert state.old_session_packet_count == 1
    assert state.host_receive_monotonic_ns == 200


@pytest.mark.parametrize(
    ("field", "expected"),
    [("connected", "disconnected"), ("active", "inactive"), ("tracked", "untracked")],
)
def test_each_controller_fact_is_independently_retained_and_invalid(field: str, expected: str) -> None:
    provider = ControllerProvider()
    state = provider.update(
        replace(packet(), **{field: False}), host_receive_monotonic_ns=1
    )
    assert not state.controller_valid
    assert state.invalid_reason == expected
    assert state.invalid_count == 1


def test_source_timestamp_interval_pause_and_reorder_are_diagnostics() -> None:
    provider = ControllerProvider(source_pause_after_s=0.1)
    provider.update(packet(seq=1, t_ns=1_000), host_receive_monotonic_ns=1)
    paused = provider.update(packet(seq=2, t_ns=200_001_000), host_receive_monotonic_ns=2)
    assert paused.source_interval_ns == 200_000_000 and paused.source_pause
    reordered = provider.update(packet(seq=3, t_ns=900), host_receive_monotonic_ns=3)
    assert reordered.source_reorder_count == 1
    assert reordered.controller_valid


# Existing-contract integration and independent combinations (requirements 23--32).
def test_index_only_drives_arm_transport_clutch() -> None:
    provider, adapter, monitor = ControllerProvider(), ControllerClutchAdapter(), TransportClutchMonitor()
    monitor_step(provider, adapter, monitor, packet(seq=1), 1)
    state = monitor_step(provider, adapter, monitor, packet(seq=2, index=0.9), 2)
    assert state.arm_engaged and not state.hand_engaged


def test_grip_only_drives_hand_transport_clutch() -> None:
    provider, adapter, monitor = ControllerProvider(), ControllerClutchAdapter(), TransportClutchMonitor()
    monitor_step(provider, adapter, monitor, packet(seq=1), 1)
    state = monitor_step(provider, adapter, monitor, packet(seq=2, grip=0.9), 2)
    assert not state.arm_engaged and state.hand_engaged


def test_both_and_neither_transport_combinations() -> None:
    provider, adapter, monitor = ControllerProvider(), ControllerClutchAdapter(), TransportClutchMonitor()
    released = monitor_step(provider, adapter, monitor, packet(seq=1), 1)
    both = monitor_step(provider, adapter, monitor, packet(seq=2, index=0.9, grip=0.9), 2)
    neither = monitor_step(provider, adapter, monitor, packet(seq=3, index=0.0, grip=0.0), 3)
    assert not released.arm_engaged and not released.hand_engaged
    assert both.arm_engaged and both.hand_engaged
    assert not neither.arm_engaged and not neither.hand_engaged


def test_disconnect_does_not_retain_pressed_and_requires_both_released() -> None:
    provider, adapter, monitor = ControllerProvider(), ControllerClutchAdapter(), TransportClutchMonitor()
    monitor_step(provider, adapter, monitor, packet(seq=1), 1)
    assert monitor_step(provider, adapter, monitor, packet(seq=2, index=0.9, grip=0.9), 2).arm_engaged
    invalid = monitor_step(
        provider,
        adapter,
        monitor,
        packet(seq=3, connected=False, active=False, tracked=False, index=0.9, grip=0.9),
        3,
    )
    held = monitor_step(provider, adapter, monitor, packet(seq=4, index=0.9, grip=0.9), 4)
    one_released = monitor_step(provider, adapter, monitor, packet(seq=5, index=0.0, grip=0.9), 5)
    released = monitor_step(provider, adapter, monitor, packet(seq=6), 6)
    pressed = monitor_step(provider, adapter, monitor, packet(seq=7, index=0.9), 7)
    assert not invalid.arm_engaged and not invalid.hand_engaged
    assert held.arm_release_required and one_released.arm_release_required
    assert not released.arm_release_required and not released.hand_release_required
    assert pressed.arm_engaged and not pressed.hand_engaged


def test_stale_requires_release_then_press() -> None:
    provider = ControllerProvider(stale_after_s=0.1)
    adapter, monitor = ControllerClutchAdapter(), TransportClutchMonitor()
    monitor_step(provider, adapter, monitor, packet(seq=1), 1)
    monitor_step(provider, adapter, monitor, packet(seq=2, index=0.9), 2)
    stale_state = provider.snapshot(now_monotonic_ns=100_000_003)
    stale = monitor.update(
        adapter.samples(stale_state),
        now_monotonic_ns=100_000_003,
        stale_after_ns=provider.stale_after_ns,
    )
    held = monitor_step(provider, adapter, monitor, packet(seq=3, index=0.9), 100_000_004)
    monitor_step(provider, adapter, monitor, packet(seq=4), 100_000_005)
    pressed = monitor_step(provider, adapter, monitor, packet(seq=5, index=0.9), 100_000_006)
    assert not stale.arm_engaged and held.arm_release_required
    assert pressed.arm_engaged


def test_new_session_requires_release_then_press() -> None:
    provider, adapter, monitor = ControllerProvider(), ControllerClutchAdapter(), TransportClutchMonitor()
    monitor_step(provider, adapter, monitor, packet(session=1, seq=1), 1)
    monitor_step(provider, adapter, monitor, packet(session=1, seq=2, grip=0.9), 2)
    held_restart = monitor_step(provider, adapter, monitor, packet(session=2, seq=0, grip=0.9), 3)
    monitor_step(provider, adapter, monitor, packet(session=2, seq=1), 4)
    pressed = monitor_step(provider, adapter, monitor, packet(session=2, seq=2, grip=0.9), 5)
    assert not held_restart.hand_engaged and held_restart.hand_release_required
    assert pressed.hand_engaged


def test_adapter_samples_feed_existing_arm_and_hand_state_machines() -> None:
    provider, adapter = ControllerProvider(), ControllerClutchAdapter()
    arm = ArmClutchMachine(stale_after_s=1.0)
    hand = HandClutchMachine(stale_after_s=1.0)
    released = adapter.samples(provider.update(packet(seq=1), host_receive_monotonic_ns=1))
    arm.step(released.index, now_ns=1, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True)
    hand.step(released.grip, now_ns=1, controller_valid=True, skeleton_valid=True)
    active = adapter.samples(
        provider.update(packet(seq=2, index=0.9, grip=0.0), host_receive_monotonic_ns=2)
    )
    assert arm.step(active.index, now_ns=2, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True) is ClutchAction.CAPTURE_ARM_REFERENCE
    assert hand.step(active.grip, now_ns=2, controller_valid=True, skeleton_valid=True) is ClutchAction.FREEZE
    assert arm.state is ArmClutchState.REFERENCE_CAPTURE
    assert hand.state is HandClutchState.DISENGAGED


def test_gate_maintains_independent_hand_head_and_controller_ages() -> None:
    gate = QuestTransportGate(stale_after_s=1.0, started_monotonic_ns=0)
    assert gate.ingest(datagram(hand_payload(), 100_000_000)).accepted
    assert gate.ingest(datagram(head_payload(), 150_000_000)).accepted
    assert gate.ingest(datagram(line_for(packet()) + "\n", 200_000_000)).accepted
    summary = gate.summary(300_000_000)
    assert summary["hand_packets"] == 1 and summary["head_packets"] == 1
    assert summary["ctrl_packets"] == 1
    assert summary["right_hand_age_ms"] == pytest.approx(200.0)
    assert summary["controller_sample_age_ms"] == pytest.approx(100.0)


def test_gate_no_data_timeout_requires_ctrl_and_right_hand() -> None:
    gate = QuestTransportGate(started_monotonic_ns=0)
    gate.ingest(datagram(line_for(packet()) + "\n", 1))
    assert gate.missing_required_streams_after(20.0, now_monotonic_ns=19_999_999_999) == ()
    assert gate.missing_required_streams_after(20.0, now_monotonic_ns=20_000_000_000) == ("right_hand",)


def test_no_keyboard_fallback_or_fake_live_combination_in_gate_cli() -> None:
    source = Path("tools/quest_controller_transport_gate.py").read_text(encoding="utf-8").lower()
    assert "keyboard fallback" in source
    assert "pygame" not in source and "pynput" not in source
    result = subprocess.run(
        [sys.executable, "tools/quest_controller_transport_gate.py", "--fake"],
        env={"PYTHONPATH": "src"},
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 2
    assert "unrecognized arguments: --fake" in result.stderr


def test_transport_modules_import_no_mujoco_or_hardware_packages() -> None:
    code = """
import sys
import motion_input.controller_protocol
import motion_input.controller_provider
import motion_input.transport_gate
forbidden = ('mujoco', 'quest_jaka_sim', 'jaka_driver_adapter', 'rh56_driver', 'robot_bringup')
loaded = sorted(name for name in sys.modules if any(name == p or name.startswith(p + '.') for p in forbidden))
print('\\n'.join(loaded))
raise SystemExit(bool(loaded))
"""
    result = subprocess.run(
        [sys.executable, "-c", code],
        env={"PYTHONPATH": "src"},
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout == "\n"
