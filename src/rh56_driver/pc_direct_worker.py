from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import statistics
import threading
import time
import traceback
from typing import Callable, Sequence

from .pc_direct_control import PcDirectFeedback, RH56PcDirectControl


@dataclass(frozen=True, slots=True)
class PendingTarget:
    values: tuple[float, ...]
    sequence: int
    submitted_monotonic_ns: int


class RH56PcDirectWorker:
    """Own the one PC-direct controller without blocking the arm producer."""

    def __init__(
        self,
        control: RH56PcDirectControl,
        *,
        record: Callable[[dict[str, object]], None] | None = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.control = control
        self.record = record
        self.max_target_normalized = control.max_close
        self._monotonic_ns = monotonic_ns
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._active_requested = False
        self._pending_target: PendingTarget | None = None
        self._hold_reason = "startup"
        self._terminal_reason: str | None = None
        self._feedback: PcDirectFeedback | None = None
        self._failure: BaseException | None = None
        self._failure_record: dict[str, object] | None = None
        self._logging_failures: deque[dict[str, object]] = deque(maxlen=16)
        self._submitted_sequence = 0
        self._observed_sequence = 0
        self._written_sequence: int | None = None
        self._last_submitted_values: tuple[float, ...] | None = None
        self._submit_count = 0
        self._activation_target_count = 0
        self._unique_submit_count = 0
        self._coalesced_count = 0
        self._stale_drop_count = 0
        self._cycle_count = 0
        self._cycle_overrun_count = 0
        self._feedback_count = 0
        self._write_count = 0
        self._first_submit_ns: int | None = None
        self._last_submit_ns: int | None = None
        self._first_unique_submit_ns: int | None = None
        self._last_unique_submit_ns: int | None = None
        self._first_feedback_ns: int | None = None
        self._last_feedback_ns: int | None = None
        self._first_write_ns: int | None = None
        self._last_write_ns: int | None = None
        diagnostics = control.config.get("diagnostics", {})
        self.diagnostics_enabled = bool(diagnostics.get("enabled", False))
        self.diagnostics_window_size = int(diagnostics.get("window_size", 256))
        self.stale_command_drop_enabled = bool(
            diagnostics.get("stale_command_drop_enabled", False)
        )
        self.stale_command_max_age_ns = int(
            round(float(diagnostics.get("stale_command_max_age_sec", 0.25)) * 1e9)
        )
        if self.stale_command_max_age_ns <= 0:
            raise ValueError("RH56 stale_command_max_age_sec must be positive")
        self._cycle_duration_ms: deque[float] = deque(
            maxlen=self.diagnostics_window_size
        )
        self._cycle_interval_ms: deque[float] = deque(
            maxlen=self.diagnostics_window_size
        )
        self._submit_interval_ms: deque[float] = deque(
            maxlen=self.diagnostics_window_size
        )
        self._unique_submit_interval_ms: deque[float] = deque(
            maxlen=self.diagnostics_window_size
        )
        self._feedback_interval_ms: deque[float] = deque(
            maxlen=self.diagnostics_window_size
        )
        self._write_interval_ms: deque[float] = deque(
            maxlen=self.diagnostics_window_size
        )
        self._last_cycle_started_ns: int | None = None

    def start(
        self, approval_token: str, *, run_in_thread: bool = True
    ) -> PcDirectFeedback:
        self.control.open(approval_token)
        feedback = self.control.poll_feedback(self._monotonic_ns())
        with self._lock:
            self._feedback = feedback
        self._note_feedback(feedback.monotonic_ns)
        if run_in_thread:
            self._thread = threading.Thread(
                target=self._run, name="rh56-pc-direct", daemon=True
            )
            self._thread.start()
        return feedback

    def activate_from_measured(self, monotonic_ns: int) -> tuple[float, ...]:
        self.raise_if_failed()
        with self._lock:
            feedback = self._feedback
            if (
                feedback is None
                or monotonic_ns - feedback.monotonic_ns
                > self.control.feedback_stale_timeout_ns
            ):
                raise RuntimeError("RH56 feedback is stale or absent at grip engagement.")
            self._active_requested = True
            # A measured activation target is safety-significant and must not
            # be dropped as stale or suppressed as an ordinary duplicate.
            self._submitted_sequence += 1
            self._activation_target_count += 1
            self._pending_target = PendingTarget(
                tuple(feedback.position_normalized),
                self._submitted_sequence,
                int(monotonic_ns),
            )
            return feedback.position_normalized

    def submit_target(self, target: Sequence[float], monotonic_ns: int) -> None:
        self.raise_if_failed()
        values = tuple(float(value) for value in target)
        if len(values) != 6:
            raise ValueError("RH56 worker target must have six canonical channels.")
        with self._lock:
            if self._terminal_reason is not None:
                return
            if (
                self._pending_target is not None
                and self._pending_target.sequence > self._observed_sequence
            ):
                self._coalesced_count += 1
            self._submitted_sequence += 1
            self._pending_target = PendingTarget(
                values, self._submitted_sequence, int(monotonic_ns)
            )
            self._submit_count += 1
            if (
                self.diagnostics_enabled
                and self._last_submit_ns is not None
                and monotonic_ns > self._last_submit_ns
            ):
                self._submit_interval_ms.append(
                    (monotonic_ns - self._last_submit_ns) / 1e6
                )
            if self._first_submit_ns is None:
                self._first_submit_ns = int(monotonic_ns)
            self._last_submit_ns = int(monotonic_ns)
            if values != self._last_submitted_values:
                self._unique_submit_count += 1
                if (
                    self.diagnostics_enabled
                    and self._last_unique_submit_ns is not None
                    and monotonic_ns > self._last_unique_submit_ns
                ):
                    self._unique_submit_interval_ms.append(
                        (monotonic_ns - self._last_unique_submit_ns) / 1e6
                    )
                if self._first_unique_submit_ns is None:
                    self._first_unique_submit_ns = int(monotonic_ns)
                self._last_unique_submit_ns = int(monotonic_ns)
            self._last_submitted_values = values

    def hold(self, reason: str) -> None:
        with self._lock:
            self._active_requested = False
            self._pending_target = None
            self._hold_reason = reason

    def arm_terminal_stop(self, reason: str) -> None:
        with self._lock:
            self._terminal_reason = reason
            self._active_requested = False
            self._pending_target = None

    @property
    def latest_feedback(self) -> PcDirectFeedback | None:
        with self._lock:
            return self._feedback

    @property
    def failed(self) -> bool:
        with self._lock:
            return self._failure is not None

    @property
    def failure_record(self) -> dict[str, object] | None:
        with self._lock:
            return None if self._failure_record is None else dict(self._failure_record)

    def raise_if_failed(self) -> None:
        with self._lock:
            failure = self._failure
        if failure is not None:
            raise RuntimeError("RH56 PC-direct worker failed") from failure

    def cleanup(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                # Closing the one owned transport is the only safe way to
                # unblock an in-progress serial read; cleanup performs no write.
                self.control.cleanup()
                self._thread.join(timeout=1.0)
                if self._thread.is_alive():
                    raise RuntimeError(
                        "RH56 PC-direct worker did not stop after serial close"
                    )
                self.control.cleanup()
                return
        self.control.cleanup()

    def run_cycle(self) -> bool:
        """Run one serialized feedback/command cycle.

        This public single-cycle form supports deterministic fake-clock/fake-
        serial verification. The production thread calls the same method.
        """

        cycle_started_ns = self._monotonic_ns()
        if (
            self.diagnostics_enabled
            and self._last_cycle_started_ns is not None
            and cycle_started_ns > self._last_cycle_started_ns
        ):
            self._cycle_interval_ms.append(
                (cycle_started_ns - self._last_cycle_started_ns) / 1e6
            )
        self._last_cycle_started_ns = cycle_started_ns
        feedback = self.control.poll_feedback(cycle_started_ns)
        self._note_feedback(feedback.monotonic_ns)
        with self._lock:
            self._feedback = feedback
            active = self._active_requested
            target = self._pending_target
            hold_reason = self._hold_reason
            terminal_reason = self._terminal_reason
            if target is not None:
                self._observed_sequence = max(
                    self._observed_sequence, target.sequence
                )
        if terminal_reason is not None:
            self.control.arm_terminal_stop(terminal_reason)
            return False
        command_now_ns = self._monotonic_ns()
        if active:
            just_activated = self.control.state.value == "HAND_HOLD"
            if just_activated:
                self.control.activate(command_now_ns)
            if target is not None:
                command_age_ns = max(
                    0, command_now_ns - target.submitted_monotonic_ns
                )
                if (
                    self.stale_command_drop_enabled
                    and not just_activated
                    and command_age_ns > self.stale_command_max_age_ns
                ):
                    self._stale_drop_count += 1
                    self.control.last_command_disposition = "stale_target_dropped"
                else:
                    written = self.control.command(
                        target.values,
                        command_now_ns,
                        submitted_monotonic_ns=target.submitted_monotonic_ns,
                        target_sequence=target.sequence,
                        force_write=just_activated,
                    )
                    if written:
                        self._written_sequence = target.sequence
                        self._write_count += 1
                        if (
                            self.diagnostics_enabled
                            and self._last_write_ns is not None
                            and command_now_ns > self._last_write_ns
                        ):
                            self._write_interval_ms.append(
                                (command_now_ns - self._last_write_ns) / 1e6
                            )
                        if self._first_write_ns is None:
                            self._first_write_ns = command_now_ns
                        self._last_write_ns = command_now_ns
        else:
            self.control.hold(hold_reason)
        record_now_ns = self._monotonic_ns()
        row = self.control.episode_record(
            record_now_ns, None if target is None else target.values
        )
        row["record_type"] = "rh56_telemetry"
        row["rh56_worker"] = (
            self.diagnostics_snapshot(include_windows=True)
            if self.diagnostics_enabled
            else None
        )
        self._emit_record(row)
        self._cycle_count += 1
        cycle_duration_ms = (self._monotonic_ns() - cycle_started_ns) / 1e6
        if cycle_duration_ms * 1e6 > self.control.command_period_ns:
            self._cycle_overrun_count += 1
        if self.diagnostics_enabled:
            self._cycle_duration_ms.append(cycle_duration_ms)
        return True

    def diagnostics_snapshot(self, *, include_windows: bool = True) -> dict[str, object]:
        with self._lock:
            pending = self._pending_target
            logging_failures = list(self._logging_failures)
        result: dict[str, object] = {
            "diagnostics_enabled": self.diagnostics_enabled,
            "mailbox_kind": "latest_only_single_slot",
            "mailbox_capacity": 1,
            "pending_target_sequence": None if pending is None else pending.sequence,
            "submitted_target_count": self._submit_count,
            "measured_activation_target_count": self._activation_target_count,
            "unique_submitted_target_count": self._unique_submit_count,
            "coalesced_unobserved_target_count": self._coalesced_count,
            "last_submitted_sequence": self._submitted_sequence,
            "last_observed_sequence": self._observed_sequence,
            "last_written_sequence": self._written_sequence,
            "successful_serial_write_count": self._write_count,
            "complete_feedback_record_count": self._feedback_count,
            "worker_cycle_count": self._cycle_count,
            "worker_cycle_overrun_count": self._cycle_overrun_count,
            "stale_command_drop_enabled": self.stale_command_drop_enabled,
            "stale_command_max_age_ms": self.stale_command_max_age_ns / 1e6,
            "stale_command_drop_count": self._stale_drop_count,
            "submitted_target_rate_hz": _rate(
                self._submit_count, self._first_submit_ns, self._last_submit_ns
            ),
            "unique_target_rate_hz": _rate(
                self._unique_submit_count,
                self._first_unique_submit_ns,
                self._last_unique_submit_ns,
            ),
            "successful_serial_write_rate_hz": _rate(
                self._write_count, self._first_write_ns, self._last_write_ns
            ),
            "complete_feedback_record_rate_hz": _rate(
                self._feedback_count,
                self._first_feedback_ns,
                self._last_feedback_ns,
            ),
            "record_callback_failure_count": len(logging_failures),
            "last_record_callback_failure": (
                None if not logging_failures else logging_failures[-1]
            ),
            "control": self.control.diagnostics_snapshot(),
        }
        if include_windows and self.diagnostics_enabled:
            result["timing_ms"] = {
                "worker_cycle_duration": _distribution(self._cycle_duration_ms),
                "worker_cycle_interval": _distribution(self._cycle_interval_ms),
                "target_submit_interval": _distribution(self._submit_interval_ms),
                "unique_target_interval": _distribution(
                    self._unique_submit_interval_ms
                ),
                "successful_write_interval": _distribution(
                    self._write_interval_ms
                ),
                "complete_feedback_interval": _distribution(
                    self._feedback_interval_ms
                ),
                "worker_cycle_interval_jitter": _jitter(
                    self._cycle_interval_ms
                ),
            }
        return result

    def _run(self) -> None:
        next_cycle_ns = self._monotonic_ns()
        try:
            while not self._stop.is_set():
                if not self.run_cycle():
                    return
                next_cycle_ns += self.control.command_period_ns
                wait_ns = next_cycle_ns - self._monotonic_ns()
                if wait_ns <= 0:
                    next_cycle_ns = self._monotonic_ns()
                    continue
                self._stop.wait(wait_ns / 1e9)
        except BaseException as exc:
            if self.control.fault_reason is None:
                self.control.transport_fault("pc_direct_worker_failure")
            failure_record: dict[str, object] = {
                "schema_version": "rh56_worker_failure.v1",
                "record_type": "rh56_worker_failure",
                "wall_time_utc": datetime.now(timezone.utc).isoformat(),
                "monotonic_ns": self._monotonic_ns(),
                "thread_name": threading.current_thread().name,
                "exception_type": type(exc).__name__,
                "message": str(exc),
                "traceback": "".join(
                    traceback.format_exception(type(exc), exc, exc.__traceback__)
                ),
                "control_state": self.control.state.value,
                "transport_state": self.control.transport_state,
                "fault_reason": self.control.fault_reason,
                "control_failure": self.control.last_failure_record,
                "worker_context": self.diagnostics_snapshot(
                    include_windows=self.diagnostics_enabled
                ),
            }
            with self._lock:
                self._failure = exc
                self._failure_record = failure_record
            self._emit_record(failure_record)

    def _emit_record(self, row: dict[str, object]) -> None:
        if self.record is None:
            return
        try:
            self.record(row)
        except BaseException as exc:
            failure = {
                "schema_version": "rh56_logging_failure.v1",
                "record_type": "rh56_logging_failure",
                "monotonic_ns": self._monotonic_ns(),
                "operation": "record_callback",
                "exception_type": type(exc).__name__,
                "message": str(exc),
            }
            with self._lock:
                self._logging_failures.append(failure)

    def _note_feedback(self, monotonic_ns: int) -> None:
        self._feedback_count += 1
        if (
            self.diagnostics_enabled
            and self._last_feedback_ns is not None
            and monotonic_ns > self._last_feedback_ns
        ):
            self._feedback_interval_ms.append(
                (monotonic_ns - self._last_feedback_ns) / 1e6
            )
        if self._first_feedback_ns is None:
            self._first_feedback_ns = int(monotonic_ns)
        self._last_feedback_ns = int(monotonic_ns)


def _rate(count: int, first_ns: int | None, last_ns: int | None) -> float | None:
    if count < 2 or first_ns is None or last_ns is None or last_ns <= first_ns:
        return None
    return (count - 1) * 1e9 / (last_ns - first_ns)


def _distribution(values: Sequence[float]) -> dict[str, float] | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)

    def percentile(fraction: float) -> float:
        return ordered[int(round((len(ordered) - 1) * fraction))]

    return {
        "p50": percentile(0.50),
        "p95": percentile(0.95),
        "p99": percentile(0.99),
        "max": ordered[-1],
    }


def _jitter(values: Sequence[float]) -> dict[str, float] | None:
    if not values:
        return None
    return {
        "standard_deviation": float(statistics.pstdev(values)),
        "max_absolute_from_median": max(
            abs(float(value) - statistics.median(values)) for value in values
        ),
    }
