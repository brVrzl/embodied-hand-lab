from __future__ import annotations

import json

from motion_input.errors import SourceDisconnected
from motion_input.model import Side, TrackingState
from motion_input.quest import (
    InMemoryQuestSource,
    QuestMotionProvider,
    parse_quest_frame,
)

from test_motion_input_protocol import make_device


def quest_payload(
    sequence: int,
    *,
    state: str = "tracking",
    side: str = "left",
) -> dict[str, object]:
    tracked = state == "tracking"
    pose = {
        "position_m": [0.1, 1.2, 0.3],
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    return {
        "schema": "quest-hand-frame",
        "version": "1.0",
        "session_id": "session-a",
        "stream_id": f"quest/session-a/{side}",
        "sequence_number": sequence,
        "side": side,
        "reference_space": "local_floor",
        "basis": "unity",
        "capture_timestamp": {
            "nanoseconds": 10_000_000 + sequence * 16_000_000,
            "clock_id": "quest:session-a:unity_realtime",
        },
        "device_timestamp": None,
        "tracking_state": state,
        "tracking_confidence": None,
        "wrist_pose": pose if tracked else None,
        "palm_pose": pose if tracked else None,
        "articulation": (
            {
                "joints": [
                    {
                        "name": "index_tip",
                        "pose": pose,
                        "tracking_state": "tracking",
                        "confidence": None,
                        "radius_m": 0.008,
                    }
                ],
                "gestures": [],
                "pinch_strength": None,
                "grasp_strength": None,
                "confidence": None,
            }
            if tracked
            else None
        ),
        "metadata": {"confidence_scale": "unavailable"},
    }


def test_quest_provider_isolates_wire_types_and_converts_basis() -> None:
    frame = parse_quest_frame(json.dumps(quest_payload(7)))
    ticks = iter([90, 95, 100, 110])
    provider = QuestMotionProvider(
        InMemoryQuestSource([frame]),
        device=make_device(),
        monotonic_ns=lambda: next(ticks),
    )
    with provider:
        sample = provider.read()
    assert sample is not None
    assert sample.side is Side.LEFT
    assert sample.wrist_pose is not None
    assert sample.wrist_pose.position_m == (0.1, 1.2, -0.3)
    assert sample.coordinate_frame.endswith("local_floor:openxr")
    assert sample.metadata["source_basis"] == "unity"
    assert sample.device_timestamp is None
    assert sample.articulation is not None
    assert sample.articulation.joints[0].pose.position_m == (0.1, 1.2, -0.3)


def test_tracking_loss_and_recovery_are_emitted_without_stale_pose() -> None:
    frames = [
        parse_quest_frame(json.dumps(quest_payload(0))),
        parse_quest_frame(json.dumps(quest_payload(1, state="not_tracking"))),
        parse_quest_frame(json.dumps(quest_payload(2))),
    ]
    provider = QuestMotionProvider(
        InMemoryQuestSource(frames),
        device=make_device(),
        monotonic_ns=iter(range(100, 110)).__next__,
    )
    with provider:
        samples = [provider.read(), provider.read(), provider.read()]
    assert [sample.tracking_state for sample in samples] == [
        TrackingState.TRACKING,
        TrackingState.NOT_TRACKING,
        TrackingState.TRACKING,
    ]
    assert samples[1].wrist_pose is None
    assert samples[2].wrist_pose is not None


def test_device_disconnect_emits_one_event_for_each_hand() -> None:
    provider = QuestMotionProvider(
        InMemoryQuestSource([SourceDisconnected("headset offline")]),
        device=make_device(),
        monotonic_ns=lambda: 500,
    )
    with provider:
        left = provider.read()
        right = provider.read()
    assert left.tracking_state is TrackingState.DISCONNECTED
    assert right.tracking_state is TrackingState.DISCONNECTED
    assert {left.side, right.side} == {Side.LEFT, Side.RIGHT}
    assert left.wrist_pose is None and right.wrist_pose is None
    assert left.metadata["disconnect_reason"] == "headset offline"


def test_silent_stream_timeout_and_recovery() -> None:
    frame = parse_quest_frame(json.dumps(quest_payload(5)))
    ticks = iter([0, 0, 1_000_000_000, 1_000_000_010, 1_000_000_020, 1_000_000_030])
    provider = QuestMotionProvider(
        InMemoryQuestSource([None, frame]),
        device=make_device(),
        monotonic_ns=ticks.__next__,
        disconnect_timeout_s=1.0,
    )
    with provider:
        disconnected_left = provider.read()
        disconnected_right = provider.read()
        recovered = provider.read()
    assert disconnected_left.tracking_state is TrackingState.DISCONNECTED
    assert disconnected_right.tracking_state is TrackingState.DISCONNECTED
    assert recovered.tracking_state is TrackingState.TRACKING
    assert recovered.sequence_number == 5
