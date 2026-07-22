"""Session-aware host state for Quest left-controller CTRL samples."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

from .clutch import AnalogClutchSample, AnalogHoldToRun, HysteresisObservation
from .controller_protocol import ControllerPacket


@dataclass(frozen=True, slots=True)
class ControllerProviderState:
    latest: ControllerPacket | None
    host_receive_monotonic_ns: int | None
    sample_age_ns: int | None
    stale: bool
    controller_valid: bool
    invalid_reason: str | None
    active_fault: str
    sequence_gap: int
    source_interval_ns: int | None
    source_pause: bool
    packet_count: int
    accepted_count: int
    invalid_count: int
    gap_event_count: int
    missing_sequence_count: int
    duplicate_count: int
    reorder_count: int
    new_session_count: int
    session_count: int
    old_session_packet_count: int
    stale_event_count: int
    source_pause_count: int
    source_reorder_count: int

    @property
    def sample_age_s(self) -> float | None:
        return None if self.sample_age_ns is None else self.sample_age_ns / 1e9


@dataclass(frozen=True, slots=True)
class ControllerClutchFrame:
    index: AnalogClutchSample
    grip: AnalogClutchSample
    controller_valid: bool
    shared_release_required: bool
    provider_name: str = "quest_ctrl_udp_v1"


@dataclass(frozen=True, slots=True)
class TransportClutchSnapshot:
    arm_engaged: bool
    hand_engaged: bool
    arm_release_required: bool
    hand_release_required: bool
    arm_rising_edge: bool
    arm_falling_edge: bool
    hand_rising_edge: bool
    hand_falling_edge: bool


class ControllerProvider:
    """Accept only forward packets from the current non-retired session."""

    def __init__(
        self,
        *,
        stale_after_s: float = 0.25,
        source_pause_after_s: float = 0.25,
    ) -> None:
        self.stale_after_ns = _positive_seconds_ns(stale_after_s, "stale_after_s")
        self.source_pause_after_ns = _positive_seconds_ns(
            source_pause_after_s, "source_pause_after_s"
        )
        self._latest: ControllerPacket | None = None
        self._host_receive_ns: int | None = None
        self._current_session: int | None = None
        self._retired_sessions: set[int] = set()
        self._sequence_gap = 0
        self._source_interval_ns: int | None = None
        self._source_pause = False
        self._packet_count = 0
        self._accepted_count = 0
        self._invalid_count = 0
        self._gap_event_count = 0
        self._missing_sequence_count = 0
        self._duplicate_count = 0
        self._reorder_count = 0
        self._new_session_count = 0
        self._session_count = 0
        self._old_session_packet_count = 0
        self._stale_event_count = 0
        self._source_pause_count = 0
        self._source_reorder_count = 0
        self._active_fault = "no_sample"
        self._stale_latched = True
        self._forced_invalid_reason: str | None = None

    def update(
        self,
        packet: ControllerPacket,
        *,
        host_receive_monotonic_ns: int | None = None,
    ) -> ControllerProviderState:
        receive_ns = (
            time.monotonic_ns()
            if host_receive_monotonic_ns is None
            else host_receive_monotonic_ns
        )
        if receive_ns < 0:
            raise ValueError("host receive monotonic timestamp must be non-negative")
        self._packet_count += 1
        self._sequence_gap = 0
        self._source_interval_ns = None
        self._source_pause = False

        if packet.session_id in self._retired_sessions:
            self._old_session_packet_count += 1
            self._active_fault = "old_session_packet"
            return self.snapshot(now_monotonic_ns=receive_ns)

        if self._current_session is None:
            self._current_session = packet.session_id
            self._session_count = 1
            self._accept(packet, receive_ns, active_fault="initial_session")
            return self.snapshot(now_monotonic_ns=receive_ns)

        if packet.session_id != self._current_session:
            self._retired_sessions.add(self._current_session)
            self._current_session = packet.session_id
            self._new_session_count += 1
            self._session_count += 1
            self._accept(packet, receive_ns, active_fault="new_session")
            return self.snapshot(now_monotonic_ns=receive_ns)

        assert self._latest is not None
        if packet.sequence_number == self._latest.sequence_number:
            self._duplicate_count += 1
            self._active_fault = "duplicate"
            return self.snapshot(now_monotonic_ns=receive_ns)
        if packet.sequence_number < self._latest.sequence_number:
            self._reorder_count += 1
            self._active_fault = "sequence_reorder"
            return self.snapshot(now_monotonic_ns=receive_ns)
        if packet.sequence_number > self._latest.sequence_number + 1:
            self._sequence_gap = packet.sequence_number - self._latest.sequence_number - 1
            self._gap_event_count += 1
            self._missing_sequence_count += self._sequence_gap

        source_delta = packet.source_timestamp_ns - self._latest.source_timestamp_ns
        self._source_interval_ns = source_delta
        if source_delta < 0:
            self._source_reorder_count += 1
        elif source_delta > self.source_pause_after_ns:
            self._source_pause = True
            self._source_pause_count += 1

        fault = "sequence_gap" if self._sequence_gap else "none"
        if source_delta < 0:
            fault = "source_timestamp_reorder"
        elif self._source_pause:
            fault = "source_pause"
        self._accept(packet, receive_ns, active_fault=fault)
        return self.snapshot(now_monotonic_ns=receive_ns)

    def invalidate(self, reason: str) -> None:
        """Latch a transport/parser fault without fabricating a CTRL sample."""

        if not reason.strip():
            raise ValueError("controller invalid reason is required")
        self._active_fault = reason
        self._forced_invalid_reason = reason

    def snapshot(
        self, *, now_monotonic_ns: int | None = None
    ) -> ControllerProviderState:
        now_ns = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
        if now_ns < 0:
            raise ValueError("host monotonic timestamp must be non-negative")
        if self._latest is None or self._host_receive_ns is None:
            age_ns = None
            stale = True
        else:
            age_ns = max(0, now_ns - self._host_receive_ns)
            stale = now_ns < self._host_receive_ns or age_ns > self.stale_after_ns
        if stale and not self._stale_latched:
            self._stale_event_count += 1
            self._stale_latched = True
        facts_valid = bool(
            self._latest is not None
            and self._latest.facts_valid
            and self._forced_invalid_reason is None
        )
        controller_valid = facts_valid and not stale
        invalid_reason = (
            _invalid_reason(self._latest, stale) or self._forced_invalid_reason
        )
        active_fault = "stale" if stale else (
            self._forced_invalid_reason or self._active_fault
        )
        return ControllerProviderState(
            latest=self._latest,
            host_receive_monotonic_ns=self._host_receive_ns,
            sample_age_ns=age_ns,
            stale=stale,
            controller_valid=controller_valid,
            invalid_reason=invalid_reason,
            active_fault=active_fault,
            sequence_gap=self._sequence_gap,
            source_interval_ns=self._source_interval_ns,
            source_pause=self._source_pause,
            packet_count=self._packet_count,
            accepted_count=self._accepted_count,
            invalid_count=self._invalid_count,
            gap_event_count=self._gap_event_count,
            missing_sequence_count=self._missing_sequence_count,
            duplicate_count=self._duplicate_count,
            reorder_count=self._reorder_count,
            new_session_count=self._new_session_count,
            session_count=self._session_count,
            old_session_packet_count=self._old_session_packet_count,
            stale_event_count=self._stale_event_count,
            source_pause_count=self._source_pause_count,
            source_reorder_count=self._source_reorder_count,
        )

    def _accept(
        self, packet: ControllerPacket, receive_ns: int, *, active_fault: str
    ) -> None:
        self._latest = packet
        self._host_receive_ns = receive_ns
        self._forced_invalid_reason = None
        self._accepted_count += 1
        if not packet.facts_valid:
            self._invalid_count += 1
        self._active_fault = (
            _invalid_reason(packet, False) or active_fault
        )
        self._stale_latched = False


class ControllerClutchAdapter:
    """Map CTRL facts into the existing independent analog clutch contract.

    A shared controller fault or sender session change blocks both samples until
    one valid packet observes *both* controls at or below the release threshold.
    No press/release hysteresis is computed here; that remains in the existing
    per-channel ``AnalogHoldToRun`` machines.
    """

    def __init__(self, *, released_at: float = 0.55) -> None:
        if not 0.0 <= released_at < 1.0:
            raise ValueError("release threshold must be in [0, 1)")
        self.released_at = float(released_at)
        self.shared_release_required = True
        self._session_id: int | None = None

    def samples(self, state: ControllerProviderState) -> ControllerClutchFrame:
        packet = state.latest
        if packet is None or state.host_receive_monotonic_ns is None:
            self.shared_release_required = True
            invalid = AnalogClutchSample(0.0, 0, 0, valid=False)
            return ControllerClutchFrame(
                invalid, invalid, False, self.shared_release_required
            )

        if packet.session_id != self._session_id:
            self._session_id = packet.session_id
            self.shared_release_required = True
        if not state.controller_valid:
            self.shared_release_required = True
        elif self.shared_release_required and (
            packet.index_trigger <= self.released_at
            and packet.grip_trigger <= self.released_at
        ):
            self.shared_release_required = False

        sample_valid = state.controller_valid and not self.shared_release_required
        index = AnalogClutchSample(
            packet.index_trigger,
            state.host_receive_monotonic_ns,
            packet.sequence_number,
            valid=sample_valid,
            source_timestamp_ns=packet.source_timestamp_ns,
        )
        grip = AnalogClutchSample(
            packet.grip_trigger,
            state.host_receive_monotonic_ns,
            packet.sequence_number,
            valid=sample_valid,
            source_timestamp_ns=packet.source_timestamp_ns,
        )
        return ControllerClutchFrame(
            index,
            grip,
            state.controller_valid,
            self.shared_release_required,
        )


class TransportClutchMonitor:
    """Input-only visibility for two independent existing hysteresis channels."""

    def __init__(self, *, pressed_at: float = 0.75, released_at: float = 0.55) -> None:
        self.arm = AnalogHoldToRun(pressed_at=pressed_at, released_at=released_at)
        self.hand = AnalogHoldToRun(pressed_at=pressed_at, released_at=released_at)
        self._invalid_latched = False

    def update(
        self, frame: ControllerClutchFrame, *, now_monotonic_ns: int, stale_after_ns: int
    ) -> TransportClutchSnapshot:
        inputs_valid = frame.controller_valid and not frame.shared_release_required
        if not inputs_valid and not self._invalid_latched:
            self.arm.require_release()
            self.hand.require_release()
            self._invalid_latched = True
        if inputs_valid:
            self._invalid_latched = False
        arm_obs = self.arm.observe(
            frame.index,
            fresh=inputs_valid and frame.index.fresh(now_monotonic_ns, stale_after_ns),
        )
        hand_obs = self.hand.observe(
            frame.grip,
            fresh=inputs_valid and frame.grip.fresh(now_monotonic_ns, stale_after_ns),
        )
        return _transport_snapshot(arm_obs, hand_obs, frame.shared_release_required)


def _transport_snapshot(
    arm: HysteresisObservation,
    hand: HysteresisObservation,
    shared_release_required: bool,
) -> TransportClutchSnapshot:
    return TransportClutchSnapshot(
        arm_engaged=bool(arm.pressed and arm.released_observed),
        hand_engaged=bool(hand.pressed and hand.released_observed),
        arm_release_required=shared_release_required or not arm.released_observed,
        hand_release_required=shared_release_required or not hand.released_observed,
        arm_rising_edge=arm.rising_edge,
        arm_falling_edge=arm.falling_edge,
        hand_rising_edge=hand.rising_edge,
        hand_falling_edge=hand.falling_edge,
    )


def _invalid_reason(packet: ControllerPacket | None, stale: bool) -> str | None:
    if stale:
        return "stale"
    if packet is None:
        return "no_sample"
    reasons: list[str] = []
    if not packet.connected:
        reasons.append("disconnected")
    if not packet.active:
        reasons.append("inactive")
    if not packet.tracked:
        reasons.append("untracked")
    return "+".join(reasons) or None


def _positive_seconds_ns(value: float, name: str) -> int:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return int(value * 1e9)
