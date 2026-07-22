"""Input-only Quest hand/head plus CTRL transport monitoring."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any

from .controller_protocol import (
    ControllerPacketError,
    QuestTransportKind,
    parse_quest_transport_datagram,
)
from .controller_provider import (
    ControllerClutchAdapter,
    ControllerProvider,
    ControllerProviderState,
    TransportClutchMonitor,
    TransportClutchSnapshot,
)
from .errors import SerializationError
from .hts_canonical import CanonicalQuestState, HtsCanonicalAssembler
from .hts_protocol import HtsHeadPosePacket, HtsLandmarksPacket, HtsWristPacket
from .hts_transport import ReceivedHtsDatagram


@dataclass(frozen=True, slots=True)
class GateIngestResult:
    kind: str
    accepted: bool
    error: str | None = None


class QuestTransportGate:
    """Validate and summarize UDP input without producing any robot target."""

    def __init__(
        self,
        *,
        stale_after_s: float = 0.25,
        started_monotonic_ns: int | None = None,
    ) -> None:
        self.started_monotonic_ns = (
            time.monotonic_ns()
            if started_monotonic_ns is None
            else started_monotonic_ns
        )
        self.provider = ControllerProvider(stale_after_s=stale_after_s)
        self.adapter = ControllerClutchAdapter(released_at=0.55)
        self.clutches = TransportClutchMonitor(pressed_at=0.75, released_at=0.55)
        self.assembler = HtsCanonicalAssembler(stale_after_s=stale_after_s)
        self.hand_datagram_count = 0
        self.head_datagram_count = 0
        self.malformed_ctrl_count = 0
        self.malformed_hand_head_count = 0
        self.unknown_packet_count = 0
        self.first_ctrl_receive_ns: int | None = None
        self.first_hand_receive_ns: int | None = None
        self.first_head_receive_ns: int | None = None
        self.last_ctrl_receive_ns: int | None = None
        self.last_hand_receive_ns: int | None = None
        self.last_head_receive_ns: int | None = None
        self.controller_age_samples_ns: list[int] = []
        self.clutch_transitions: list[dict[str, Any]] = []
        self._last_clutch = TransportClutchSnapshot(
            False, False, True, True, False, False, False, False
        )
        self._last_canonical = self.assembler.state(
            now_monotonic_ns=self.started_monotonic_ns
        )

    def ingest(self, datagram: ReceivedHtsDatagram) -> GateIngestResult:
        is_ctrl = datagram.payload.startswith(b"CTRL")
        try:
            dispatched = parse_quest_transport_datagram(datagram.payload)
        except (ControllerPacketError, SerializationError) as exc:
            if is_ctrl:
                self.malformed_ctrl_count += 1
                self.provider.invalidate("malformed_ctrl")
                self._update_clutches(datagram.receive_monotonic_ns)
                kind = "malformed_ctrl"
            else:
                self.malformed_hand_head_count += 1
                kind = "malformed_hand_head"
            return GateIngestResult(kind, False, str(exc))

        if dispatched.kind is QuestTransportKind.CONTROLLER:
            assert dispatched.controller is not None
            state = self.provider.update(
                dispatched.controller,
                host_receive_monotonic_ns=datagram.receive_monotonic_ns,
            )
            self.first_ctrl_receive_ns = (
                datagram.receive_monotonic_ns
                if self.first_ctrl_receive_ns is None
                else self.first_ctrl_receive_ns
            )
            self.last_ctrl_receive_ns = datagram.receive_monotonic_ns
            self._update_clutches(datagram.receive_monotonic_ns, state=state)
            return GateIngestResult("controller", True)

        packets = dispatched.hand_head
        has_hand = any(
            isinstance(packet, (HtsWristPacket, HtsLandmarksPacket))
            for packet in packets
        )
        has_head = any(isinstance(packet, HtsHeadPosePacket) for packet in packets)
        if not packets:
            self.unknown_packet_count += 1
            return GateIngestResult("ignored_ctrl_for_legacy", False)
        try:
            self._last_canonical = self.assembler.ingest(
                packets,
                receive_monotonic_ns=datagram.receive_monotonic_ns,
                source_endpoint=datagram.source_endpoint,
                datagram_size=len(datagram.payload),
            )
        except SerializationError as exc:
            self.malformed_hand_head_count += 1
            return GateIngestResult("malformed_hand_head", False, str(exc))
        if has_hand:
            self.hand_datagram_count += 1
            self.first_hand_receive_ns = (
                datagram.receive_monotonic_ns
                if self.first_hand_receive_ns is None
                else self.first_hand_receive_ns
            )
            self.last_hand_receive_ns = datagram.receive_monotonic_ns
        if has_head:
            self.head_datagram_count += 1
            self.first_head_receive_ns = (
                datagram.receive_monotonic_ns
                if self.first_head_receive_ns is None
                else self.first_head_receive_ns
            )
            self.last_head_receive_ns = datagram.receive_monotonic_ns
        return GateIngestResult("hand_head", True)

    def poll(self, now_monotonic_ns: int | None = None) -> None:
        now_ns = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
        state = self.provider.snapshot(now_monotonic_ns=now_ns)
        self._update_clutches(now_ns, state=state)
        self._last_canonical = self.assembler.state(now_monotonic_ns=now_ns)

    def summary(self, now_monotonic_ns: int | None = None) -> dict[str, Any]:
        now_ns = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
        self.poll(now_ns)
        state = self.provider.snapshot(now_monotonic_ns=now_ns)
        packet = state.latest
        if state.sample_age_ns is not None:
            self.controller_age_samples_ns.append(state.sample_age_ns)
        elapsed_s = max((now_ns - self.started_monotonic_ns) / 1e9, 1e-9)
        right = self._last_canonical.right
        head = self._last_canonical.head
        return {
            "elapsed_s": elapsed_s,
            "hand_packets": self.hand_datagram_count,
            "hand_rate_hz": self.hand_datagram_count / elapsed_s,
            "head_packets": self.head_datagram_count,
            "head_rate_hz": self.head_datagram_count / elapsed_s,
            "ctrl_packets": state.packet_count,
            "ctrl_rate_hz": state.packet_count / elapsed_s,
            "session": None if packet is None else packet.session_id,
            "seq": None if packet is None else packet.sequence_number,
            "source_t_ns": None if packet is None else packet.source_timestamp_ns,
            "source_interval_ns": state.source_interval_ns,
            "source_pause": state.source_pause,
            "connected": bool(packet is not None and packet.connected),
            "active": bool(packet is not None and packet.active),
            "tracked": bool(packet is not None and packet.tracked),
            "controller_valid": state.controller_valid,
            "index": 0.0 if packet is None else packet.index_trigger,
            "grip": 0.0 if packet is None else packet.grip_trigger,
            "arm_clutch": self._last_clutch.arm_engaged,
            "hand_clutch": self._last_clutch.hand_engaged,
            "arm_release_required": self._last_clutch.arm_release_required,
            "hand_release_required": self._last_clutch.hand_release_required,
            "controller_sample_age_ms": (
                None if state.sample_age_ns is None else state.sample_age_ns / 1e6
            ),
            "controller_stale": state.stale,
            "right_hand_valid": right.tracking_valid,
            "right_hand_age_ms": (
                None if right.stream_age_s is None else right.stream_age_s * 1e3
            ),
            "head_valid": head is not None,
            "head_age_ms": None if head is None else head.stream_age_s * 1e3,
            "gap_events": state.gap_event_count,
            "missing_sequences": state.missing_sequence_count,
            "duplicates": state.duplicate_count,
            "reorders": state.reorder_count,
            "sessions": state.session_count,
            "new_sessions": state.new_session_count,
            "old_session_packets": state.old_session_packet_count,
            "stale_events": state.stale_event_count,
            "invalid_ctrl_samples": state.invalid_count,
            "source_pauses": state.source_pause_count,
            "source_reorders": state.source_reorder_count,
            "malformed_ctrl": self.malformed_ctrl_count,
            "malformed_hand_head": self.malformed_hand_head_count,
            "active_fault": state.active_fault,
            "invalid_reason": state.invalid_reason,
        }

    def final_report(self, now_monotonic_ns: int | None = None) -> dict[str, Any]:
        now_ns = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
        report = self.summary(now_ns)
        ages_ms = [value / 1e6 for value in self.controller_age_samples_ns]
        report.update(
            {
                "controller_sample_age_mean_ms": _mean(ages_ms),
                "controller_sample_age_p95_ms": _percentile(ages_ms, 0.95),
                "controller_sample_age_max_ms": max(ages_ms) if ages_ms else None,
                "clutch_transitions": list(self.clutch_transitions),
                "first_ctrl_receive_ns": self.first_ctrl_receive_ns,
                "first_hand_receive_ns": self.first_hand_receive_ns,
                "first_head_receive_ns": self.first_head_receive_ns,
                "last_ctrl_receive_ns": self.last_ctrl_receive_ns,
                "last_hand_receive_ns": self.last_hand_receive_ns,
                "last_head_receive_ns": self.last_head_receive_ns,
                "safety": (
                    "input-only; no MuJoCo, viewer, JAKA, Inspire, RH56, IK, "
                    "or robot-target path"
                ),
            }
        )
        return report

    def missing_required_streams_after(
        self, timeout_s: float, *, now_monotonic_ns: int | None = None
    ) -> tuple[str, ...]:
        now_ns = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
        if now_ns - self.started_monotonic_ns < int(timeout_s * 1e9):
            return ()
        missing: list[str] = []
        if self.first_ctrl_receive_ns is None:
            missing.append("CTRL")
        if self.first_hand_receive_ns is None:
            missing.append("right_hand")
        return tuple(missing)

    def _update_clutches(
        self,
        now_ns: int,
        *,
        state: ControllerProviderState | None = None,
    ) -> None:
        provider_state = state or self.provider.snapshot(now_monotonic_ns=now_ns)
        frame = self.adapter.samples(provider_state)
        current = self.clutches.update(
            frame,
            now_monotonic_ns=now_ns,
            stale_after_ns=self.provider.stale_after_ns,
        )
        self._record_transition("arm", self._last_clutch.arm_engaged, current.arm_engaged, now_ns)
        self._record_transition("hand", self._last_clutch.hand_engaged, current.hand_engaged, now_ns)
        self._last_clutch = current

    def _record_transition(
        self, channel: str, previous: bool, current: bool, timestamp_ns: int
    ) -> None:
        if previous == current:
            return
        self.clutch_transitions.append(
            {
                "timestamp_monotonic_ns": timestamp_ns,
                "channel": channel,
                "previous": previous,
                "current": current,
                "reason": "press_edge" if current else "release_or_invalid",
            }
        )


def _mean(values: list[float]) -> float | None:
    return None if not values else sum(values) / len(values)


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(math.ceil(fraction * len(ordered))) - 1))
    return ordered[index]
