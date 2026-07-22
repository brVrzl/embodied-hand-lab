"""Provider-independent, hold-to-run clutch state machines.

This module deliberately knows nothing about Quest SDKs, keyboards, robots, or
MuJoCo.  A provider must supply timestamped analog samples and explicit
validity.  The two state machines consume separate samples; they are never
collapsed into a high-level teleoperation mode.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


@dataclass(frozen=True, slots=True)
class AnalogClutchSample:
    """One analog control sample in the host monotonic clock domain."""

    value: float
    host_receive_monotonic_ns: int
    sequence_number: int
    valid: bool = True
    source_timestamp_ns: int | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.value) or not 0.0 <= self.value <= 1.0:
            raise ValueError("analog clutch value must be finite and in [0, 1]")
        if self.host_receive_monotonic_ns < 0 or self.sequence_number < 0:
            raise ValueError("clutch timestamps and sequences must be non-negative")

    def fresh(self, now_ns: int, stale_after_ns: int) -> bool:
        return (
            self.valid
            and now_ns >= self.host_receive_monotonic_ns
            and now_ns - self.host_receive_monotonic_ns <= stale_after_ns
        )


@dataclass(frozen=True, slots=True)
class HysteresisObservation:
    pressed: bool | None
    rising_edge: bool
    falling_edge: bool
    released_observed: bool
    new_sample: bool
    valid: bool


class AnalogHoldToRun:
    """Analog hysteresis with edge generation and release-before-press arming."""

    def __init__(self, *, pressed_at: float = 0.75, released_at: float = 0.55) -> None:
        if not 0.0 <= released_at < pressed_at <= 1.0:
            raise ValueError("clutch thresholds must satisfy 0 <= release < press <= 1")
        self.pressed_at = float(pressed_at)
        self.released_at = float(released_at)
        self.pressed: bool | None = None
        self.released_observed = False
        self._last_sequence: int | None = None

    def require_release(self) -> None:
        """Re-arm only after a new valid released sample is observed."""

        self.released_observed = False
        self.pressed = None
        self._last_sequence = None

    def observe(self, sample: AnalogClutchSample, *, fresh: bool) -> HysteresisObservation:
        if not fresh:
            return HysteresisObservation(
                self.pressed, False, False, self.released_observed, False, False
            )
        if sample.sequence_number == self._last_sequence:
            return HysteresisObservation(
                self.pressed, False, False, self.released_observed, False, True
            )
        self._last_sequence = sample.sequence_number
        previous = self.pressed
        if sample.value >= self.pressed_at:
            self.pressed = True
        elif sample.value <= self.released_at:
            self.pressed = False
            self.released_observed = True
        # The hysteresis band preserves the prior debounced state, including
        # unknown at startup.  A high startup sample never creates a press edge.
        rising = bool(
            self.released_observed and previous is False and self.pressed is True
        )
        falling = bool(previous is True and self.pressed is False)
        return HysteresisObservation(
            self.pressed,
            rising,
            falling,
            self.released_observed,
            True,
            True,
        )


class ArmClutchState(str, Enum):
    ARMED_WAITING_FOR_RELEASE = "armed_waiting_for_release"
    DISENGAGED = "disengaged"
    REFERENCE_CAPTURE = "reference_capture"
    ENGAGED = "engaged"
    TRACKING_FAULT = "tracking_fault"


class HandClutchState(str, Enum):
    ARMED_WAITING_FOR_RELEASE = "armed_waiting_for_release"
    DISENGAGED = "disengaged"
    REACQUIRE = "reacquire"
    ENGAGED = "engaged"
    TRACKING_FAULT = "tracking_fault"


class ClutchAction(str, Enum):
    FREEZE = "freeze"
    CAPTURE_ARM_REFERENCE = "capture_arm_reference"
    START_HAND_REACQUISITION = "start_hand_reacquisition"
    UPDATE = "update"


@dataclass(frozen=True, slots=True)
class ClutchTransition:
    timestamp_monotonic_ns: int
    previous: str
    current: str
    reason: str


@dataclass(frozen=True, slots=True)
class ClutchFault:
    timestamp_monotonic_ns: int
    channel: str
    reason: str


class ArmClutchMachine:
    def __init__(self, *, stale_after_s: float, pressed_at: float = 0.75, released_at: float = 0.55) -> None:
        self.stale_after_ns = _stale_ns(stale_after_s)
        self.trigger = AnalogHoldToRun(pressed_at=pressed_at, released_at=released_at)
        self.state = ArmClutchState.ARMED_WAITING_FOR_RELEASE
        self.transitions: list[ClutchTransition] = []
        self.fault_history: list[ClutchFault] = []
        self.active_fault: ClutchFault | None = None
        self.cycle_count = 0

    def step(
        self,
        sample: AnalogClutchSample,
        *,
        now_ns: int,
        controller_valid: bool,
        continuous_inputs_valid: bool,
        capture_inputs_valid: bool,
    ) -> ClutchAction:
        fresh = controller_valid and sample.fresh(now_ns, self.stale_after_ns)
        observation = self.trigger.observe(sample, fresh=fresh)
        if self.state not in {
            ArmClutchState.ARMED_WAITING_FOR_RELEASE,
            ArmClutchState.TRACKING_FAULT,
        } and not fresh:
            self.fault(now_ns, "ARM_TRIGGER_STALE_OR_INVALID")
            return ClutchAction.FREEZE
        if self.state is ArmClutchState.ENGAGED and not continuous_inputs_valid:
            self.fault(now_ns, "RIGHT_WRIST_STALE_OR_INVALID")
            return ClutchAction.FREEZE
        if self.state is ArmClutchState.REFERENCE_CAPTURE:
            return ClutchAction.FREEZE
        if self.state is ArmClutchState.ARMED_WAITING_FOR_RELEASE:
            if observation.valid and observation.pressed is False:
                self._transition(now_ns, ArmClutchState.DISENGAGED, "VALID_RELEASE_OBSERVED")
            return ClutchAction.FREEZE
        if self.state is ArmClutchState.TRACKING_FAULT:
            if (
                observation.valid
                and observation.pressed is False
                and self.active_fault is not None
                and sample.host_receive_monotonic_ns > self.active_fault.timestamp_monotonic_ns
                and capture_inputs_valid
                and controller_valid
            ):
                self.active_fault = None
                self._transition(now_ns, ArmClutchState.DISENGAGED, "FAULT_RECOVERY_RELEASED_AND_VALID")
            return ClutchAction.FREEZE
        if self.state is ArmClutchState.DISENGAGED:
            if observation.rising_edge:
                if not fresh or not capture_inputs_valid:
                    self.fault(now_ns, "ARM_REFERENCE_INPUT_INVALID")
                    return ClutchAction.FREEZE
                self._transition(now_ns, ArmClutchState.REFERENCE_CAPTURE, "ARM_PRESS_EDGE")
                return ClutchAction.CAPTURE_ARM_REFERENCE
            return ClutchAction.FREEZE
        if observation.pressed is False:
            self._transition(now_ns, ArmClutchState.DISENGAGED, "ARM_TRIGGER_RELEASED")
            return ClutchAction.FREEZE
        return ClutchAction.UPDATE

    def reference_captured(self, now_ns: int) -> None:
        if self.state is not ArmClutchState.REFERENCE_CAPTURE:
            raise RuntimeError("arm reference capture completion is out of sequence")
        self.cycle_count += 1
        self._transition(now_ns, ArmClutchState.ENGAGED, "ARM_REFERENCE_CAPTURED")

    def fault(self, now_ns: int, reason: str) -> None:
        self.active_fault = ClutchFault(now_ns, "arm", reason)
        self.fault_history.append(self.active_fault)
        self.trigger.require_release()
        self._transition(now_ns, ArmClutchState.TRACKING_FAULT, reason)

    def _transition(self, now_ns: int, state: ArmClutchState, reason: str) -> None:
        if state is self.state:
            return
        previous = self.state
        self.state = state
        self.transitions.append(ClutchTransition(now_ns, previous.value, state.value, reason))


class HandClutchMachine:
    def __init__(
        self,
        *,
        stale_after_s: float,
        reacquisition_duration_s: float = 0.2,
        pressed_at: float = 0.75,
        released_at: float = 0.55,
    ) -> None:
        self.stale_after_ns = _stale_ns(stale_after_s)
        if not 0.0 < reacquisition_duration_s <= 1.0:
            raise ValueError("hand reacquisition duration must be in (0, 1] seconds")
        self.reacquisition_duration_ns = int(reacquisition_duration_s * 1e9)
        self.trigger = AnalogHoldToRun(pressed_at=pressed_at, released_at=released_at)
        self.state = HandClutchState.ARMED_WAITING_FOR_RELEASE
        self.transitions: list[ClutchTransition] = []
        self.fault_history: list[ClutchFault] = []
        self.active_fault: ClutchFault | None = None
        self.cycle_count = 0
        self.reacquisition_started_ns: int | None = None

    def step(
        self,
        sample: AnalogClutchSample,
        *,
        now_ns: int,
        controller_valid: bool,
        skeleton_valid: bool,
    ) -> ClutchAction:
        fresh = controller_valid and sample.fresh(now_ns, self.stale_after_ns)
        observation = self.trigger.observe(sample, fresh=fresh)
        if self.state not in {
            HandClutchState.ARMED_WAITING_FOR_RELEASE,
            HandClutchState.TRACKING_FAULT,
        } and not fresh:
            self.fault(now_ns, "HAND_GRIP_STALE_OR_INVALID")
            return ClutchAction.FREEZE
        if self.state in {HandClutchState.ENGAGED, HandClutchState.REACQUIRE} and not skeleton_valid:
            self.fault(now_ns, "RIGHT_HAND_SKELETON_STALE_OR_INVALID")
            return ClutchAction.FREEZE
        if self.state is HandClutchState.ARMED_WAITING_FOR_RELEASE:
            if observation.valid and observation.pressed is False:
                self._transition(now_ns, HandClutchState.DISENGAGED, "VALID_RELEASE_OBSERVED")
            return ClutchAction.FREEZE
        if self.state is HandClutchState.TRACKING_FAULT:
            if (
                observation.valid
                and observation.pressed is False
                and self.active_fault is not None
                and sample.host_receive_monotonic_ns > self.active_fault.timestamp_monotonic_ns
                and skeleton_valid
                and controller_valid
            ):
                self.active_fault = None
                self._transition(now_ns, HandClutchState.DISENGAGED, "FAULT_RECOVERY_RELEASED_AND_VALID")
            return ClutchAction.FREEZE
        if self.state is HandClutchState.DISENGAGED:
            if observation.rising_edge:
                if not fresh or not skeleton_valid:
                    self.fault(now_ns, "HAND_REACQUISITION_INPUT_INVALID")
                    return ClutchAction.FREEZE
                self.reacquisition_started_ns = now_ns
                self.cycle_count += 1
                self._transition(now_ns, HandClutchState.REACQUIRE, "HAND_PRESS_EDGE")
                return ClutchAction.START_HAND_REACQUISITION
            return ClutchAction.FREEZE
        if observation.pressed is False:
            self.reacquisition_started_ns = None
            self._transition(now_ns, HandClutchState.DISENGAGED, "HAND_GRIP_RELEASED")
            return ClutchAction.FREEZE
        if self.state is HandClutchState.REACQUIRE:
            assert self.reacquisition_started_ns is not None
            if now_ns - self.reacquisition_started_ns >= self.reacquisition_duration_ns:
                self._transition(now_ns, HandClutchState.ENGAGED, "HAND_REACQUISITION_COMPLETE")
            return ClutchAction.UPDATE
        return ClutchAction.UPDATE

    def reacquisition_fraction(self, now_ns: int) -> float:
        if self.reacquisition_started_ns is None:
            return 1.0
        return min(1.0, max(0.0, (now_ns - self.reacquisition_started_ns) / self.reacquisition_duration_ns))

    def fault(self, now_ns: int, reason: str) -> None:
        self.active_fault = ClutchFault(now_ns, "hand", reason)
        self.fault_history.append(self.active_fault)
        self.reacquisition_started_ns = None
        self.trigger.require_release()
        self._transition(now_ns, HandClutchState.TRACKING_FAULT, reason)

    def _transition(self, now_ns: int, state: HandClutchState, reason: str) -> None:
        if state is self.state:
            return
        previous = self.state
        self.state = state
        self.transitions.append(ClutchTransition(now_ns, previous.value, state.value, reason))


def _stale_ns(stale_after_s: float) -> int:
    if not math.isfinite(stale_after_s) or stale_after_s <= 0.0:
        raise ValueError("clutch stale timeout must be finite and positive")
    return int(stale_after_s * 1e9)
