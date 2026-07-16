from __future__ import annotations

import json

import pytest

from motion_input.errors import ProtocolValidationError, SerializationError
from motion_input.frames import openxr_to_unity_pose, unity_to_openxr_pose
from motion_input.model import (
    DeviceDescriptor,
    GestureSample,
    HandArticulation,
    JointSample,
    MotionInputSample,
    Pose6D,
    Side,
    Timestamp,
    TrackingState,
)
from motion_input.serialization import dumps_sample, loads_sample, sample_from_dict, sample_to_dict


def make_device() -> DeviceDescriptor:
    return DeviceDescriptor(
        device_id="quest-test",
        device_type="xr_headset",
        manufacturer="Meta",
        model="Quest 3",
        metadata={"role": "test"},
    )


def make_sample(sequence: int = 0, *, state: TrackingState = TrackingState.TRACKING) -> MotionInputSample:
    pose = Pose6D((0.1, 0.2, -0.3), (0.0, 0.0, 0.0, 1.0)) if state is TrackingState.TRACKING else None
    return MotionInputSample(
        sample_id=f"sample-{sequence}",
        stream_id="stream-left",
        sequence_number=sequence,
        capture_timestamp=Timestamp(1_000_000 + sequence * 10_000, "host:monotonic"),
        receive_timestamp=Timestamp(1_005_000 + sequence * 10_000, "host:monotonic"),
        device_timestamp=Timestamp(900_000 + sequence * 10_000, "openxr:runtime"),
        processing_timestamp=Timestamp(1_006_000 + sequence * 10_000, "host:monotonic"),
        tracking_state=state,
        tracking_confidence=0.9 if pose else None,
        coordinate_frame="quest/test/local_floor:openxr",
        device=make_device(),
        side=Side.LEFT,
        wrist_pose=pose,
        palm_pose=pose,
        articulation=(
            HandArticulation(
                joints=(JointSample("index_tip", pose, state, 0.8, 0.008),),
                gestures=(GestureSample("pinch", True, 0.7, 0.6),),
                pinch_strength=0.6,
                grasp_strength=0.2,
                confidence=0.8,
            )
            if pose
            else None
        ),
        metadata={"confidence_scale": "test"},
        extensions={"example.future": {"value": 3}},
    )


def test_serialization_round_trip_preserves_typed_sample() -> None:
    sample = make_sample()
    restored = loads_sample(dumps_sample(sample))
    assert restored == sample
    assert restored.articulation is not None
    assert restored.articulation.joints[0].name == "index_tip"


def test_new_optional_fields_are_ignored_by_same_major_reader() -> None:
    payload = sample_to_dict(make_sample())
    payload["protocol_version"] = "1.99"
    payload["future_top_level_field"] = {"safe": True}
    restored = sample_from_dict(payload)
    assert restored.protocol_version == "1.99"


def test_incompatible_major_version_is_rejected() -> None:
    payload = sample_to_dict(make_sample())
    payload["protocol_version"] = "2.0"
    with pytest.raises(SerializationError, match="major version"):
        sample_from_dict(payload)


def test_tracking_loss_cannot_carry_stale_pose() -> None:
    with pytest.raises(ProtocolValidationError, match="must not contain poses"):
        MotionInputSample(
            sample_id="lost",
            stream_id="stream",
            sequence_number=1,
            capture_timestamp=Timestamp(1, "clock"),
            receive_timestamp=Timestamp(2, "clock"),
            device_timestamp=None,
            processing_timestamp=None,
            tracking_state=TrackingState.NOT_TRACKING,
            tracking_confidence=None,
            coordinate_frame="frame",
            device=make_device(),
            side=Side.LEFT,
            wrist_pose=Pose6D((0, 0, 0), (0, 0, 0, 1)),
        )


def test_pose_requires_unit_quaternion() -> None:
    with pytest.raises(ProtocolValidationError, match="unit quaternion"):
        Pose6D((0, 0, 0), (0, 0, 0, 2))


def test_timestamp_comparison_requires_identical_clock() -> None:
    with pytest.raises(ProtocolValidationError, match="different clocks"):
        Timestamp(2, "host").difference_ns(Timestamp(1, "quest"))


def test_extensions_must_be_namespaced() -> None:
    payload = sample_to_dict(make_sample())
    payload["extensions"] = {"ambiguous": 1}
    with pytest.raises(SerializationError, match="namespaced"):
        sample_from_dict(payload)


def test_unity_openxr_conversion_is_consistent_and_invertible() -> None:
    unity = Pose6D((1.0, 2.0, 3.0), (0.5, 0.5, 0.5, 0.5))
    openxr = unity_to_openxr_pose(unity)
    assert openxr.position_m == (1.0, 2.0, -3.0)
    assert openxr.orientation_xyzw == (-0.5, -0.5, 0.5, 0.5)
    assert openxr_to_unity_pose(openxr) == unity
