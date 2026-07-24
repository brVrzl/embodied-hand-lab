"""Canonical Quest state assembly for Hand Tracking Streamer observations.

HTS wrist/head poses arrive in Unity world coordinates and landmarks arrive in
the corresponding wrist-local coordinates.  This module applies the one named
Unity-to-OpenXR-style basis conversion and infers validity only from freshness.
It performs no calibration, reference capture, filtering, or robot mapping.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from types import MappingProxyType
from typing import Any, Mapping

from .errors import ProtocolValidationError, SerializationError
from .frames import unity_to_openxr_pose
from .hts_protocol import (
    HTS_JOINT_NAMES,
    HtsHeadPosePacket,
    HtsLandmarksPacket,
    HtsPacket,
    HtsWristPacket,
    normalize_quaternion,
)
from .model import Pose6D, Side, TrackingState


QUEST_WORLD_FRAME = "quest_world"
QUEST_HEAD_FRAME = "quest_head"
LEFT_WRIST_FRAME = "left_wrist"
RIGHT_WRIST_FRAME = "right_wrist"
CANONICAL_OPERATOR_FRAME = "canonical_operator"
FUTURE_ROBOT_BASE_FRAME = "future_robot_base"


@dataclass(frozen=True, slots=True)
class QuestJointObservation:
    name: str
    position_m: tuple[float, float, float]
    frame_id: str
    orientation_xyzw: tuple[float, float, float, float] | None = None
    confidence: float | None = None
    valid: bool = True


@dataclass(frozen=True, slots=True)
class QuestHandObservation:
    side: Side
    tracking_state: TrackingState
    tracking_valid: bool
    host_receive_monotonic_ns: int | None
    source_timestamp_ns: int | None
    source_sequence_number: int | None
    host_sequence_number: int | None
    stream_age_s: float | None
    wrist_pose: Pose6D | None
    wrist_frame_id: str
    joints: tuple[QuestJointObservation, ...] = ()
    confidence: float | None = None
    raw_quaternion_norm: float | None = None


@dataclass(frozen=True, slots=True)
class QuestHeadObservation:
    tracking_valid: bool
    host_receive_monotonic_ns: int
    source_timestamp_ns: int | None
    source_sequence_number: int | None
    stream_age_s: float
    pose: Pose6D
    raw_quaternion_norm: float


@dataclass(frozen=True, slots=True)
class CanonicalQuestState:
    """A timestamped, bimanual snapshot; never a robot command."""

    host_monotonic_ns: int
    source_timestamp_ns: int | None
    sequence_number: int
    coordinate_frame: str
    head: QuestHeadObservation | None
    left: QuestHandObservation
    right: QuestHandObservation
    raw_source_metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "raw_source_metadata", MappingProxyType(dict(self.raw_source_metadata))
        )


class ClutchState(str, Enum):
    DISENGAGED = "disengaged"
    ENGAGED = "engaged"
    UNAVAILABLE = "unavailable"


@dataclass(frozen=True, slots=True)
class FutureTeleopInput:
    """Inactive boundary for a later reviewed robot-control integration.

    The adapter below always outputs a disengaged clutch and emergency-neutral
    state.  It deliberately cannot enable motion.
    """

    side: Side
    operator_wrist_pose: Pose6D | None
    pose_frame: str
    clutch_state: ClutchState
    tracking_valid: bool
    stream_age_s: float | None
    emergency_neutral: bool
    finger_joints: tuple[QuestJointObservation, ...]


def prepare_inactive_future_input(
    state: CanonicalQuestState, side: Side
) -> FutureTeleopInput:
    if side not in (Side.LEFT, Side.RIGHT):
        raise ProtocolValidationError("future hand input side must be left or right")
    hand = state.left if side is Side.LEFT else state.right
    return FutureTeleopInput(
        side=side,
        operator_wrist_pose=hand.wrist_pose,
        pose_frame=QUEST_WORLD_FRAME,
        clutch_state=ClutchState.DISENGAGED,
        tracking_valid=hand.tracking_valid,
        stream_age_s=hand.stream_age_s,
        emergency_neutral=True,
        finger_joints=hand.joints,
    )


@dataclass(slots=True)
class _HandCache:
    wrist: HtsWristPacket | None = None
    landmarks: HtsLandmarksPacket | None = None
    receive_ns: int | None = None
    host_sequence: int | None = None


@dataclass(slots=True)
class _HeadCache:
    packet: HtsHeadPosePacket
    receive_ns: int


class SourceSequenceTracker:
    """Tracks per-stream source gaps without interpreting host-generated IDs."""

    def __init__(self) -> None:
        self._last: dict[str, int] = {}
        self.gaps: dict[str, int] = {}
        self.out_of_order: dict[str, int] = {}

    def observe(self, stream: str, sequence: int | None) -> int:
        if sequence is None:
            return 0
        previous = self._last.get(stream)
        self._last[stream] = max(sequence, previous if previous is not None else sequence)
        if previous is None:
            return 0
        if sequence > previous + 1:
            gap = sequence - previous - 1
            self.gaps[stream] = self.gaps.get(stream, 0) + gap
            return gap
        if sequence <= previous:
            self.out_of_order[stream] = self.out_of_order.get(stream, 0) + 1
        return 0


class HtsCanonicalAssembler:
    def __init__(self, *, stale_after_s: float = 0.25) -> None:
        if stale_after_s <= 0:
            raise ValueError("stale_after_s must be positive")
        self.stale_after_ns = int(stale_after_s * 1_000_000_000)
        self._hands = {Side.LEFT: _HandCache(), Side.RIGHT: _HandCache()}
        self._head: _HeadCache | None = None
        self._snapshot_sequence = 0
        self._host_hand_sequences = {Side.LEFT: 0, Side.RIGHT: 0}
        self.sequence_tracker = SourceSequenceTracker()
        self.last_sequence_gap = 0

    def ingest(
        self,
        packets: tuple[HtsPacket, ...],
        *,
        receive_monotonic_ns: int,
        source_endpoint: str,
        datagram_size: int,
    ) -> CanonicalQuestState:
        if receive_monotonic_ns < 0:
            raise SerializationError("host monotonic timestamp must be non-negative")
        hand_packets: dict[Side, list[HtsPacket]] = {Side.LEFT: [], Side.RIGHT: []}
        pending_head: HtsHeadPosePacket | None = None
        for packet in packets:
            if isinstance(packet, HtsHeadPosePacket):
                if pending_head is not None:
                    raise SerializationError("duplicate HTS head lines in one datagram")
                pending_head = packet
            else:
                hand_packets[packet.header.side].append(packet)

        populated_hand_sides = [side for side, values in hand_packets.items() if values]
        if pending_head is not None and populated_hand_sides:
            raise SerializationError("HTS head and hand lines must use separate datagrams")
        if len(populated_hand_sides) > 1:
            raise SerializationError("HTS left and right hands must use separate datagrams")
        if pending_head is not None:
            self._head = _HeadCache(pending_head, receive_monotonic_ns)
            self.last_sequence_gap += self.sequence_tracker.observe(
                "head", pending_head.header.source_sequence
            )

        for side, observations in hand_packets.items():
            if not observations:
                continue
            self._validate_hand_observation_set(side, observations)
            cache = self._hands[side]
            for packet in observations:
                if isinstance(packet, HtsWristPacket):
                    cache.wrist = packet
                elif isinstance(packet, HtsLandmarksPacket):
                    cache.landmarks = packet
            cache.receive_ns = receive_monotonic_ns
            cache.host_sequence = self._host_hand_sequences[side]
            self._host_hand_sequences[side] += 1
            source_sequence = _first_not_none(
                packet.header.source_sequence for packet in observations
            )
            self.last_sequence_gap += self.sequence_tracker.observe(side.value, source_sequence)

        self._snapshot_sequence += 1
        return self.state(
            now_monotonic_ns=receive_monotonic_ns,
            raw_source_metadata={
                "source": "hand_tracking_streamer",
                "source_endpoint": source_endpoint,
                "datagram_size": datagram_size,
                "source_basis": "unity_left_handed",
                "canonical_basis": "openxr_style_right_handed",
                "coordinate_validation": "verified_live_motion_sequence_2026-07-17",
            },
        )

    def state(
        self,
        *,
        now_monotonic_ns: int,
        raw_source_metadata: Mapping[str, Any] | None = None,
    ) -> CanonicalQuestState:
        left = self._hand_observation(Side.LEFT, now_monotonic_ns)
        right = self._hand_observation(Side.RIGHT, now_monotonic_ns)
        head = self._head_observation(now_monotonic_ns)
        source_timestamps = [
            timestamp
            for timestamp in (
                left.source_timestamp_ns,
                right.source_timestamp_ns,
                None if head is None else head.source_timestamp_ns,
            )
            if timestamp is not None
        ]
        return CanonicalQuestState(
            host_monotonic_ns=now_monotonic_ns,
            source_timestamp_ns=max(source_timestamps) if source_timestamps else None,
            sequence_number=self._snapshot_sequence,
            coordinate_frame=QUEST_WORLD_FRAME,
            head=head,
            left=left,
            right=right,
            raw_source_metadata=raw_source_metadata or {},
        )

    def _validate_hand_observation_set(
        self, side: Side, packets: list[HtsPacket]
    ) -> None:
        wrists = [packet for packet in packets if isinstance(packet, HtsWristPacket)]
        landmarks = [packet for packet in packets if isinstance(packet, HtsLandmarksPacket)]
        if len(wrists) != 1 or len(landmarks) != 1:
            raise SerializationError(
                f"HTS {side.value} datagram requires one wrist and one landmarks line"
            )
        sequences = {
            packet.header.source_sequence
            for packet in packets
            if packet.header.source_sequence is not None
        }
        timestamps = {
            packet.header.source_timestamp_ns
            for packet in packets
            if packet.header.source_timestamp_ns is not None
        }
        if len(sequences) > 1 or len(timestamps) > 1:
            raise SerializationError(
                f"HTS {side.value} wrist/landmark debug metadata does not pair"
            )

    def _hand_observation(self, side: Side, now_ns: int) -> QuestHandObservation:
        cache = self._hands[side]
        wrist_frame = LEFT_WRIST_FRAME if side is Side.LEFT else RIGHT_WRIST_FRAME
        if cache.receive_ns is None:
            return QuestHandObservation(
                side,
                TrackingState.NOT_TRACKING,
                False,
                None,
                None,
                None,
                None,
                None,
                None,
                wrist_frame,
            )
        age_ns = max(0, now_ns - cache.receive_ns)
        source_sequence = _first_not_none(
            packet.header.source_sequence
            for packet in (cache.wrist, cache.landmarks)
            if packet is not None
        )
        source_timestamp = _first_not_none(
            packet.header.source_timestamp_ns
            for packet in (cache.wrist, cache.landmarks)
            if packet is not None
        )
        complete = cache.wrist is not None and cache.landmarks is not None
        valid = complete and age_ns <= self.stale_after_ns
        if not valid:
            return QuestHandObservation(
                side,
                TrackingState.NOT_TRACKING,
                False,
                cache.receive_ns,
                source_timestamp,
                source_sequence,
                cache.host_sequence,
                age_ns / 1e9,
                None,
                wrist_frame,
            )
        assert cache.wrist is not None and cache.landmarks is not None
        wrist = _canonical_pose(
            cache.wrist.position_m, cache.wrist.orientation_xyzw
        )
        joints = tuple(
            QuestJointObservation(
                name=name,
                position_m=_unity_to_openxr_position(position),
                frame_id=wrist_frame,
                orientation_xyzw=None,
                confidence=None,
            )
            for name, position in zip(
                HTS_JOINT_NAMES, cache.landmarks.positions_wrist_m, strict=True
            )
        )
        return QuestHandObservation(
            side,
            TrackingState.TRACKING,
            True,
            cache.receive_ns,
            source_timestamp,
            source_sequence,
            cache.host_sequence,
            age_ns / 1e9,
            wrist,
            wrist_frame,
            joints,
            None,
            cache.wrist.quaternion_norm,
        )

    def _head_observation(self, now_ns: int) -> QuestHeadObservation | None:
        if self._head is None:
            return None
        age_ns = max(0, now_ns - self._head.receive_ns)
        if age_ns > self.stale_after_ns:
            return None
        packet = self._head.packet
        return QuestHeadObservation(
            tracking_valid=True,
            host_receive_monotonic_ns=self._head.receive_ns,
            source_timestamp_ns=packet.header.source_timestamp_ns,
            source_sequence_number=packet.header.source_sequence,
            stream_age_s=age_ns / 1e9,
            pose=_canonical_pose(packet.position_m, packet.orientation_xyzw),
            raw_quaternion_norm=packet.quaternion_norm,
        )


def _canonical_pose(
    position: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
) -> Pose6D:
    source = Pose6D(position, normalize_quaternion(quaternion))
    return unity_to_openxr_pose(source)


def _unity_to_openxr_position(
    position: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (position[0], position[1], -position[2])


def _first_not_none(values: Any) -> int | None:
    for value in values:
        if value is not None:
            return value
    return None
