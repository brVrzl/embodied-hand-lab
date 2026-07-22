"""Bounded observability for live and replayed HTS input."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

from .hts_canonical import CanonicalQuestState, HtsCanonicalAssembler, QuestHandObservation
from .hts_protocol import HtsHeadPosePacket, HtsPacket, HtsWristPacket
from .hts_transport import ReceivedHtsDatagram
from .model import Side


@dataclass(slots=True)
class _SideStats:
    frames: int = 0
    first_receive_ns: int | None = None
    last_receive_ns: int | None = None
    position_min: list[float] = field(
        default_factory=lambda: [math.inf, math.inf, math.inf]
    )
    position_max: list[float] = field(
        default_factory=lambda: [-math.inf, -math.inf, -math.inf]
    )
    quaternion_norm_min: float = math.inf
    quaternion_norm_max: float = -math.inf
    quaternion_norm_sum: float = 0.0
    missing_joints: int = 0
    source_timestamp_frames: int = 0
    tracking_losses: int = 0
    tracking_recoveries: int = 0
    last_tracking_valid: bool = False
    tracking_state_seen: bool = False
    repeated_pose_since_ns: int | None = None
    previous_signature: tuple[float, ...] | None = None
    potential_frozen_events: int = 0
    frozen_latched: bool = False


class HtsTelemetry:
    def __init__(self, *, frozen_after_s: float = 2.0) -> None:
        if frozen_after_s <= 0:
            raise ValueError("frozen_after_s must be positive")
        self.frozen_after_ns = int(frozen_after_s * 1_000_000_000)
        self.datagrams = 0
        self.datagram_bytes = 0
        self.parsed_lines = 0
        self.malformed_datagrams = 0
        self.first_receive_ns: int | None = None
        self.last_receive_ns: int | None = None
        self.head_frames = 0
        self._sides = {Side.LEFT: _SideStats(), Side.RIGHT: _SideStats()}

    def observe_malformed(self) -> None:
        self.malformed_datagrams += 1

    def observe(
        self,
        datagram: ReceivedHtsDatagram,
        packets: tuple[HtsPacket, ...],
        state: CanonicalQuestState,
    ) -> None:
        self.datagrams += 1
        self.datagram_bytes += len(datagram.payload)
        self.parsed_lines += len(packets)
        self.first_receive_ns = (
            datagram.receive_monotonic_ns
            if self.first_receive_ns is None
            else min(self.first_receive_ns, datagram.receive_monotonic_ns)
        )
        self.last_receive_ns = datagram.receive_monotonic_ns
        self.head_frames += sum(isinstance(packet, HtsHeadPosePacket) for packet in packets)

        for side, hand in ((Side.LEFT, state.left), (Side.RIGHT, state.right)):
            wrist_packets = [
                packet
                for packet in packets
                if isinstance(packet, HtsWristPacket) and packet.header.side is side
            ]
            if not wrist_packets:
                continue
            self._observe_hand(side, hand, wrist_packets[-1], datagram.receive_monotonic_ns)
        self.observe_tracking_state(state)

    def observe_tracking_state(self, state: CanonicalQuestState) -> None:
        for side, hand in ((Side.LEFT, state.left), (Side.RIGHT, state.right)):
            stats = self._sides[side]
            if stats.tracking_state_seen:
                if stats.last_tracking_valid and not hand.tracking_valid:
                    stats.tracking_losses += 1
                elif not stats.last_tracking_valid and hand.tracking_valid:
                    stats.tracking_recoveries += 1
            stats.last_tracking_valid = hand.tracking_valid
            stats.tracking_state_seen = True

    def report(
        self,
        *,
        now_monotonic_ns: int,
        assembler: HtsCanonicalAssembler,
    ) -> dict[str, Any]:
        duration_s = 0.0
        if self.first_receive_ns is not None and self.last_receive_ns is not None:
            duration_s = max(0.0, (self.last_receive_ns - self.first_receive_ns) / 1e9)
        current = assembler.state(now_monotonic_ns=now_monotonic_ns)
        return {
            "transport": "udp",
            "datagrams": self.datagrams,
            "datagram_bytes": self.datagram_bytes,
            "parsed_lines": self.parsed_lines,
            "malformed_datagrams": self.malformed_datagrams,
            "duration_s": duration_s,
            "datagram_rate_hz": None if duration_s <= 0 else self.datagrams / duration_s,
            "head_frames": self.head_frames,
            "source_sequence_gaps": dict(assembler.sequence_tracker.gaps),
            "source_out_of_order": dict(assembler.sequence_tracker.out_of_order),
            "one_way_latency": "unavailable: Quest and host monotonic clocks have no shared epoch",
            "coordinate_validation": "verified_live_motion_sequence_2026-07-17",
            "left": self._side_report(Side.LEFT, current.left),
            "right": self._side_report(Side.RIGHT, current.right),
        }

    def _observe_hand(
        self,
        side: Side,
        hand: QuestHandObservation,
        wrist: HtsWristPacket,
        receive_ns: int,
    ) -> None:
        stats = self._sides[side]
        stats.frames += 1
        stats.first_receive_ns = (
            receive_ns
            if stats.first_receive_ns is None
            else min(stats.first_receive_ns, receive_ns)
        )
        stats.last_receive_ns = receive_ns
        if hand.wrist_pose is not None:
            for axis, value in enumerate(hand.wrist_pose.position_m):
                stats.position_min[axis] = min(stats.position_min[axis], value)
                stats.position_max[axis] = max(stats.position_max[axis], value)
            stats.missing_joints += max(0, 21 - len(hand.joints))
            signature = (*hand.wrist_pose.position_m, *hand.wrist_pose.orientation_xyzw)
            if signature == stats.previous_signature:
                if stats.repeated_pose_since_ns is None:
                    stats.repeated_pose_since_ns = receive_ns
                if (
                    receive_ns - stats.repeated_pose_since_ns >= self.frozen_after_ns
                    and not stats.frozen_latched
                ):
                    stats.potential_frozen_events += 1
                    stats.frozen_latched = True
            else:
                stats.previous_signature = signature
                stats.repeated_pose_since_ns = receive_ns
                stats.frozen_latched = False
        norm = wrist.quaternion_norm
        stats.quaternion_norm_min = min(stats.quaternion_norm_min, norm)
        stats.quaternion_norm_max = max(stats.quaternion_norm_max, norm)
        stats.quaternion_norm_sum += norm
        if wrist.header.source_timestamp_ns is not None:
            stats.source_timestamp_frames += 1

    def _side_report(
        self, side: Side, current: QuestHandObservation
    ) -> dict[str, Any]:
        stats = self._sides[side]
        duration_s = 0.0
        if stats.first_receive_ns is not None and stats.last_receive_ns is not None:
            duration_s = max(0.0, (stats.last_receive_ns - stats.first_receive_ns) / 1e9)
        has_positions = stats.frames > 0 and all(math.isfinite(v) for v in stats.position_min)
        return {
            "frames": stats.frames,
            "frame_rate_hz": None if duration_s <= 0 else stats.frames / duration_s,
            "tracking_valid": current.tracking_valid,
            "stream_age_s": current.stream_age_s,
            "tracking_losses": stats.tracking_losses,
            "tracking_recoveries": stats.tracking_recoveries,
            "wrist_position_min_m": stats.position_min if has_positions else None,
            "wrist_position_max_m": stats.position_max if has_positions else None,
            "wrist_quaternion_norm": None
            if stats.frames == 0
            else {
                "min": stats.quaternion_norm_min,
                "mean": stats.quaternion_norm_sum / stats.frames,
                "max": stats.quaternion_norm_max,
            },
            "missing_joints": stats.missing_joints,
            "source_timestamp_frames": stats.source_timestamp_frames,
            "source_sequence_number": current.source_sequence_number,
            "potential_frozen_events": stats.potential_frozen_events,
        }
