"""Offline-safe action arbitration primitives for policy takeover rollouts.

The arbiter deliberately has no robot or serial dependencies.  It selects one
already-validated candidate per control tick; the caller remains responsible
for sending that selected action through the existing JAKA/RH56 adapters.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Callable, Sequence

from teleoperation.accepted_target import AcceptedArmTarget, ArmControlHeartbeat


def _six(values: Sequence[float], name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != 6 or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain six finite values")
    return result


class DaggerState(str, Enum):
    """Control ownership state for one policy/expert rollout."""

    POLICY_RUN = "policy_run"
    TAKEOVER_HOLD = "takeover_hold"
    EXPERT_ARMING = "expert_arming"
    EXPERT_RUN = "expert_run"
    SAFE_STOP = "safe_stop"
    COMPLETE = "complete"


class DaggerActionSource(str, Enum):
    POLICY = "policy"
    EXPERT = "expert"
    HOLD = "hold"
    INVALID = "invalid"


@dataclass(frozen=True, slots=True)
class DaggerAction:
    """One complete twelve-dimensional absolute/native action."""

    arm_q: tuple[float, ...]
    hand_target: tuple[float, ...]
    source: DaggerActionSource
    valid: bool = True
    reason: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "arm_q", _six(self.arm_q, "arm_q"))
        object.__setattr__(
            self,
            "hand_target",
            _six(self.hand_target, "hand_target"),
        )
        if not isinstance(self.source, DaggerActionSource):
            object.__setattr__(self, "source", DaggerActionSource(self.source))

    @property
    def vector(self) -> tuple[float, ...]:
        return (*self.arm_q, *self.hand_target)

    @classmethod
    def from_vector(
        cls,
        values: Sequence[float],
        *,
        source: DaggerActionSource,
        reason: str | None = None,
    ) -> "DaggerAction":
        values_tuple = tuple(float(value) for value in values)
        if len(values_tuple) != 12 or not all(math.isfinite(value) for value in values_tuple):
            raise ValueError("action must contain twelve finite values")
        return cls(
            values_tuple[:6],
            values_tuple[6:],
            source,
            valid=True,
            reason=reason,
        )


@dataclass(frozen=True, slots=True)
class DaggerTransition:
    timestamp_ns: int
    previous: DaggerState
    current: DaggerState
    reason: str
    intervention_id: str | None


@dataclass(frozen=True, slots=True)
class DaggerDecision:
    """The arbiter result for exactly one coordinator tick."""

    state: DaggerState
    action: DaggerAction | None
    intervention_id: str | None
    reason: str
    capture_requested: bool = False


class DaggerActionArbiter:
    """Select policy, hold, or expert ownership without touching hardware.

    The release-then-press handshake is intentionally explicit:

    ``request_takeover`` -> ``observe_clutch(released=True)`` ->
    ``observe_clutch(pressed=True)`` -> ``mark_expert_ready``.

    The coordinator may perform the Quest reference capture between the press
    edge and ``mark_expert_ready``.  Until then, the arbiter only returns the
    hold action.
    """

    def __init__(self, *, intervention_prefix: str = "takeover") -> None:
        if not intervention_prefix.strip():
            raise ValueError("intervention_prefix must not be empty")
        self.state = DaggerState.POLICY_RUN
        self.intervention_prefix = intervention_prefix
        self._intervention_count = 0
        self.intervention_id: str | None = None
        self.transitions: list[DaggerTransition] = []
        self._capture_requested = False

    @property
    def capture_requested(self) -> bool:
        return self._capture_requested

    def request_takeover(self, now_ns: int, *, reason: str = "operator_takeover") -> str:
        """Latch a recoverable takeover request from policy control."""

        if now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        if self.state is DaggerState.POLICY_RUN:
            self._intervention_count += 1
            self.intervention_id = f"{self.intervention_prefix}_{self._intervention_count:04d}"
            self._capture_requested = False
            self._transition(now_ns, DaggerState.TAKEOVER_HOLD, reason)
        elif self.state in {
            DaggerState.TAKEOVER_HOLD,
            DaggerState.EXPERT_ARMING,
            DaggerState.EXPERT_RUN,
        }:
            # Duplicate takeover signals are harmless and must not reset a
            # live expert session or create a second intervention id.
            pass
        else:
            raise RuntimeError(f"takeover is unavailable in state {self.state.value}")
        assert self.intervention_id is not None
        return self.intervention_id

    def observe_clutch(
        self,
        now_ns: int,
        *,
        released: bool,
        pressed: bool,
        valid: bool,
    ) -> bool:
        """Advance the re-clutch handshake and report a capture request."""

        if now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        if not valid:
            return False
        if self.state is DaggerState.TAKEOVER_HOLD and released:
            self._transition(
                now_ns,
                DaggerState.EXPERT_ARMING,
                "valid_clutch_release_observed",
            )
        if self.state is DaggerState.EXPERT_ARMING and pressed:
            self._capture_requested = True
            return True
        return False

    def mark_expert_ready(self, now_ns: int) -> None:
        """Commit ownership to Quest after reference capture succeeds."""

        if now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        if self.state is not DaggerState.EXPERT_ARMING:
            raise RuntimeError(
                f"expert reference is out of sequence in state {self.state.value}"
            )
        if not self._capture_requested:
            raise RuntimeError("expert reference cannot start before a clutch press")
        self._capture_requested = False
        self._transition(now_ns, DaggerState.EXPERT_RUN, "expert_reference_ready")

    def safe_stop(self, now_ns: int, *, reason: str) -> None:
        if now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        if not reason.strip():
            raise ValueError("safe-stop reason must not be empty")
        if self.state not in {DaggerState.COMPLETE, DaggerState.SAFE_STOP}:
            self._transition(now_ns, DaggerState.SAFE_STOP, reason)

    def complete(self, now_ns: int, *, reason: str = "task_complete") -> None:
        if now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        if self.state is DaggerState.SAFE_STOP:
            raise RuntimeError("a safe-stopped rollout cannot complete normally")
        if self.state is not DaggerState.COMPLETE:
            self._transition(now_ns, DaggerState.COMPLETE, reason)

    def resume_policy(
        self,
        now_ns: int,
        *,
        reason: str = "expert_episode_complete",
    ) -> None:
        """Finish one intervention and return ownership to the policy."""

        if now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        if not reason.strip():
            raise ValueError("policy-resume reason must not be empty")
        if self.state is not DaggerState.EXPERT_RUN:
            raise RuntimeError(
                f"policy resume is unavailable in state {self.state.value}"
            )
        self._capture_requested = False
        self._transition(now_ns, DaggerState.POLICY_RUN, reason)
        self.intervention_id = None

    def select(
        self,
        *,
        policy: DaggerAction | None,
        expert: DaggerAction | None,
        hold: DaggerAction | None,
    ) -> DaggerDecision:
        """Select exactly one valid candidate according to current ownership."""

        if self.state is DaggerState.POLICY_RUN:
            selected = self._valid(policy, DaggerActionSource.POLICY)
            if selected is not None:
                return DaggerDecision(
                    self.state,
                    selected,
                    self.intervention_id,
                    "policy_selected",
                )
            return self._hold_decision(hold, "policy_candidate_unavailable")

        if self.state in {DaggerState.TAKEOVER_HOLD, DaggerState.EXPERT_ARMING}:
            return self._hold_decision(hold, f"{self.state.value}_hold")

        if self.state is DaggerState.EXPERT_RUN:
            selected = self._valid(expert, DaggerActionSource.EXPERT)
            if selected is not None:
                return DaggerDecision(
                    self.state,
                    selected,
                    self.intervention_id,
                    "expert_selected",
                )
            return self._hold_decision(hold, "expert_candidate_unavailable")

        return DaggerDecision(
            self.state,
            None,
            self.intervention_id,
            "no_action_after_terminal_state",
        )

    def _valid(
        self,
        action: DaggerAction | None,
        expected_source: DaggerActionSource,
    ) -> DaggerAction | None:
        if action is None or not action.valid or action.source is not expected_source:
            return None
        return action

    def _hold_decision(self, hold: DaggerAction | None, reason: str) -> DaggerDecision:
        valid_hold = self._valid(hold, DaggerActionSource.HOLD)
        return DaggerDecision(
            self.state,
            valid_hold,
            self.intervention_id,
            reason if valid_hold is not None else f"{reason}_unavailable",
        )

    def _transition(self, now_ns: int, state: DaggerState, reason: str) -> None:
        previous = self.state
        if previous is state:
            return
        self.state = state
        self.transitions.append(
            DaggerTransition(
                timestamp_ns=now_ns,
                previous=previous,
                current=state,
                reason=reason,
                intervention_id=self.intervention_id,
            )
        )


class ExpertArmCandidateSink:
    """Candidate-only implementation of ``ArmTargetOutputAdapter``."""

    def __init__(self) -> None:
        self.accepted_target: AcceptedArmTarget | None = None
        self.heartbeat_value: ArmControlHeartbeat | None = None

    def reset(self) -> None:
        self.accepted_target = None
        self.heartbeat_value = None

    def apply(self, target: AcceptedArmTarget) -> bool:
        self.accepted_target = target
        self.heartbeat_value = None
        return True

    def heartbeat(self, heartbeat: ArmControlHeartbeat) -> bool:
        self.heartbeat_value = heartbeat
        return True


class ExpertHandCandidateSink:
    """Candidate-only implementation of ``NormalizedHandOutput``."""

    def __init__(
        self,
        measured_provider: Callable[[int], Sequence[float]],
        *,
        max_target_normalized: float = 1.0,
    ) -> None:
        if not math.isfinite(max_target_normalized) or max_target_normalized <= 0.0:
            raise ValueError("max_target_normalized must be positive and finite")
        self._measured_provider = measured_provider
        self.max_target_normalized = float(max_target_normalized)
        self.activation_target: tuple[float, ...] | None = None
        self.target: tuple[float, ...] | None = None
        self.hold_reason: str | None = None

    def reset(self) -> None:
        self.activation_target = None
        self.target = None
        self.hold_reason = None

    def activate_from_measured(self, monotonic_ns: int) -> tuple[float, ...]:
        measured = _six(self._measured_provider(monotonic_ns), "measured hand target")
        if any(value < 0.0 or value > self.max_target_normalized for value in measured):
            raise ValueError("measured hand target is outside the normalized command range")
        self.activation_target = measured
        self.target = measured
        self.hold_reason = None
        return measured

    def submit_target(self, target: Sequence[float], monotonic_ns: int) -> None:
        del monotonic_ns
        values = _six(target, "expert hand target")
        if any(value < 0.0 or value > self.max_target_normalized for value in values):
            raise ValueError("expert hand target is outside the normalized command range")
        self.target = values
        self.hold_reason = None

    def hold(self, reason: str) -> None:
        if not reason.strip():
            raise ValueError("hand hold reason must not be empty")
        self.hold_reason = reason
