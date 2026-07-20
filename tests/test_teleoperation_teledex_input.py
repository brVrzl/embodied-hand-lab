from __future__ import annotations

import json
import math

import numpy as np
import pytest

from teleoperation.contracts import DiscontinuityKind, TrackingState
from teleoperation.input.teledex import TeleDexAdapter, TeleDexPacketParser
from teleoperation.transforms.se3 import quaternion_log


def packet(**updates: object) -> dict[str, object]:
    result: dict[str, object] = {
        "position": [0.1, -0.2, 0.3],
        "rotation": np.eye(3).tolist(),
        "button": False,
        "button_secondary": False,
        "toggle": False,
    }
    result.update(updates)
    return result


def test_parser_matches_teledex_007_rotation_transpose_and_optional_source_fields() -> None:
    angle = math.pi / 2
    rotation = [
        [math.cos(angle), -math.sin(angle), 0.0],
        [math.sin(angle), math.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ]
    parser = TeleDexPacketParser(
        source_timestamp_field="capture_ns",
        source_sequence_field="frame_index",
    )
    parsed = parser.parse(
        packet(rotation=rotation, capture_ns=1234, frame_index=7, tracking_quality=0.8)
    )
    assert parsed.pose.position_m == pytest.approx((0.1, -0.2, 0.3))
    assert quaternion_log(parsed.pose.quaternion_xyzw) == pytest.approx((0.0, 0.0, -angle))
    assert parsed.source_timestamp_ns == 1234
    assert parsed.source_sequence == 7
    assert parsed.tracking_quality == pytest.approx(0.8)


@pytest.mark.parametrize(
    "bad",
    [
        {"position": [1.0, 2.0]},
        {"rotation": [[0.0] * 3] * 3},
        {"position": [0.0, float("nan"), 0.0]},
    ],
)
def test_parser_rejects_malformed_pose(bad: dict[str, object]) -> None:
    with pytest.raises(ValueError):
        TeleDexPacketParser().parse(packet(**bad))


def test_adapter_assigns_local_sequence_age_and_reconnect_discontinuity() -> None:
    adapter = TeleDexAdapter(stale_after_ns=100)
    adapter.on_connect(receive_ns=1)
    adapter.ingest(packet(button=True), receive_ns=10)
    initial = adapter.latest(now_ns=20)
    assert initial is not None and initial.pose is not None
    assert initial.pose.sequence == 1
    assert initial.pose.source_sequence is None
    assert initial.pose.timestamps.source_capture_ns is None
    assert initial.pose.sample_age_ns == 10
    assert initial.pose.tracking_state == TrackingState.UNKNOWN
    assert initial.pose.discontinuity == DiscontinuityKind.INITIAL
    assert initial.run_gate.engaged

    adapter.on_disconnect(receive_ns=30)
    disconnected = adapter.latest(now_ns=31)
    assert disconnected is not None and not disconnected.connected
    assert not disconnected.run_gate.valid
    adapter.on_connect(receive_ns=40)
    adapter.ingest(packet(button=True), receive_ns=50)
    reconnected = adapter.latest(now_ns=60)
    assert reconnected is not None and reconnected.pose is not None
    assert reconnected.pose.sequence == 2
    assert reconnected.pose.discontinuity == DiscontinuityKind.RECONNECT
    assert not reconnected.run_gate.engaged
    assert "reclutch_required" in reconnected.run_gate.reason


def test_adapter_invalid_stale_and_generation_latest_only() -> None:
    adapter = TeleDexAdapter(stale_after_ns=50)
    adapter.on_connect(receive_ns=1)
    adapter.ingest(packet(position=[1.0]), receive_ns=10)
    invalid = adapter.latest(now_ns=11)
    assert invalid is not None and invalid.pose is None
    assert not invalid.run_gate.valid

    adapter.ingest(packet(), receive_ns=20)
    current = adapter.latest(now_ns=21)
    assert current is not None
    assert adapter.latest(now_ns=22, after_generation=current.generation) is None
    stale = adapter.latest(now_ns=80)
    assert stale is not None and stale.pose is not None
    assert stale.pose.tracking_state == TrackingState.INVALID
    assert stale.reason == "sample_stale"


def test_secondary_button_is_device_neutral_recenter_rising_edge() -> None:
    adapter = TeleDexAdapter(stale_after_ns=1_000)
    adapter.on_connect(receive_ns=1)
    adapter.ingest(packet(button_secondary=True), receive_ns=10)
    first = adapter.latest(now_ns=11)
    assert first is not None and first.operator_action is not None
    assert first.operator_action.recenter_requested
    adapter.ingest(packet(button_secondary=True), receive_ns=20)
    held = adapter.latest(now_ns=21, after_generation=first.generation)
    assert held is not None and held.operator_action is not None
    assert not held.operator_action.recenter_requested
    adapter.ingest(packet(button_secondary=False), receive_ns=30)
    adapter.ingest(packet(button_secondary=True), receive_ns=40)
    rising = adapter.latest(now_ns=41)
    assert rising is not None and rising.operator_action is not None
    assert rising.operator_action.recenter_requested
