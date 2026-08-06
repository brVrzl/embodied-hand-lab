from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import statistics
import threading
import time
import traceback
from typing import Callable, Sequence

from .pc_direct_control import HandOperation, PcDirectFeedback, RH56PcDirectControl


@dataclass(frozen=True, slots=True)
class PendingTarget:
    values: tuple[float, ...]
    sequence: int
    submitted_monotonic_ns: int
    measured_activation: bool = False


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
        self._wake = threading.Event()
        self._thread: threading.Thread | None = None
        self._active_requested = False
        self._pending_target: PendingTarget | None = None
        self._force_write_pending = False
        self._hold_reason = "startup"
        self._terminal_reason: str | None = None
        self._feedback: PcDirectFeedback | None = None
        self._failure: BaseException | None = None
        self._failure_record: dict[str, object] | None = None
        self._logging_failures: deque[dict[str, object]] = deque(maxlen=16)
        self._submitted_sequence = 0
        self._observed_sequence = 0
        self._written_sequence: int | None = None
        self._evaluated_sequence = 0
        self._last_submitted_values: tuple[float, ...] | None = None
        self._submit_count = 0
        self._activation_target_count = 0
        self._clamped_activation_target_count = 0
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
        self._command_due_ns: int | None = None
        self._last_command_due_ns: int | None = None
        self._last_command_actual_start_ns: int | None = None
        self._last_command_actual_end_ns: int | None = None
        self._last_command_deadline_lateness_ms: float | None = None
        self._last_submit_to_write_ms: float | None = None
        self._command_deadline_lateness_ms: deque[float] = deque(
            maxlen=self.diagnostics_window_size
        )
        self._submit_to_write_ms: deque[float] = deque(
            maxlen=self.diagnostics_window_size
        )
        self._io_busy_ns = 0
        self._first_io_start_ns: int | None = None
        self._last_io_end_ns: int | None = None
        self._full_diagnostics_next_ns: int | None = None
        self._last_record_build_duration_ns = 0
        self._maximum_record_build_duration_ns = 0
        self._feedback_schedule: dict[str, dict[str, object]] = {}
        for name in ("ANGLE", "CURRENT", "FORCE", "STATUS", "ERROR"):
            self._feedback_schedule[name] = {
                "period_ns": int(round(1e9 / control.feedback_rate_hz[name])),
                "next_due_ns": None,
                "last_attempt_ns": None,
                "last_success_ns": None,
                "failure_count": 0,
                "warning_count": 0,
                "warning_active": False,
                "success_count": 0,
                "first_success_ns": None,
                "lateness_ms": deque(maxlen=self.diagnostics_window_size),
                "age_ms": deque(maxlen=self.diagnostics_window_size),
                "interval_ms": deque(maxlen=self.diagnostics_window_size),
            }

    def start(
        self,
        operation: HandOperation,
        *,
        run_in_thread: bool = True,
    ) -> PcDirectFeedback:
        self.control.open(operation)
        feedback = self.control.poll_feedback(self._monotonic_ns())
        startup_end_ns = self._monotonic_ns()
        with self._lock:
            self._feedback = feedback
        self._note_feedback(feedback.monotonic_ns)
        for name in self._feedback_schedule:
            schedule = self._feedback_schedule[name]
            schedule["last_attempt_ns"] = startup_end_ns
            schedule["last_success_ns"] = startup_end_ns
            schedule["first_success_ns"] = startup_end_ns
            schedule["success_count"] = 1
            schedule["next_due_ns"] = startup_end_ns + int(schedule["period_ns"])
        if run_in_thread:
            self._thread = threading.Thread(
                target=self._run, name="rh56-pc-direct", daemon=True
            )
            self._thread.start()
        return feedback

    @property
    def native_thread_id(self) -> int | None:
        """OS thread id for boundary-only placement diagnostics."""

        return None if self._thread is None else self._thread.native_id

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
            measured_target = tuple(float(value) for value in feedback.position_normalized)
            activation_target = tuple(
                min(1.0, max(0.0, value)) for value in measured_target
            )
            if activation_target != measured_target:
                self._clamped_activation_target_count += 1
            # A measured activation target is safety-significant and must not
            # be dropped as stale or suppressed as an ordinary duplicate.
            # ANGLE_ACT may be outside the configured command envelope after a
            # previous bounded endpoint test. Preserve that legal mechanical
            # pose exactly for activation continuity; only later requested
            # motion is constrained by the configured command envelope.
            self._submitted_sequence += 1
            self._activation_target_count += 1
            self._pending_target = PendingTarget(
                activation_target,
                self._submitted_sequence,
                int(monotonic_ns),
                True,
            )
            self._force_write_pending = True
            self._command_due_ns = int(monotonic_ns)
            self._wake.set()
            return activation_target

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
        self._wake.set()

    def hold(self, reason: str) -> None:
        with self._lock:
            self._active_requested = False
            self._pending_target = None
            self._force_write_pending = False
            self._hold_reason = reason
        self._wake.set()

    def arm_terminal_stop(self, reason: str) -> None:
        with self._lock:
            # The first terminal event owns the stop. The producer may observe
            # a later transport symptom before this thread runs its next cycle;
            # that symptom must not replace the initiating hard-stop reason.
            if self._terminal_reason is None:
                self._terminal_reason = reason
            self._active_requested = False
            self._pending_target = None
            self._force_write_pending = False
        self._wake.set()

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
        self._wake.set()
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
        """Run the single highest-priority due serial operation."""

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
        with self._lock:
            active = self._active_requested
            target = self._pending_target
            force_write = self._force_write_pending
            hold_reason = self._hold_reason
            terminal_reason = self._terminal_reason
        if terminal_reason is not None:
            self.control.arm_terminal_stop(terminal_reason)
            return False
        now_ns = self._monotonic_ns()
        contact_relief_values: tuple[float, ...] | None = None
        if not active:
            self.control.hold(hold_reason)
            contact_relief_values = self.control.pop_contact_relief_target()
        elif self.control.state.value == "HAND_HOLD":
            self.control.activate(now_ns)

        target_requires_command = self._target_requires_command(target)
        command_due_ns = self._command_due_ns
        contact_relief_operation = contact_relief_values is not None
        command_is_due = bool(
            contact_relief_operation
            or (
                active
                and target_requires_command
                and (
                    force_write
                    or command_due_ns is None
                    or now_ns >= command_due_ns
                )
            )
        )
        operation = "idle"
        if command_is_due:
            if not contact_relief_operation:
                assert target is not None
            if force_write and not contact_relief_operation:
                with self._lock:
                    self._force_write_pending = False
            operation = "CONTACT_RELIEF" if contact_relief_operation else "COMMAND"
            due_ns = (
                now_ns
                if contact_relief_operation or command_due_ns is None
                else command_due_ns
            )
            actual_start_ns = self._monotonic_ns()
            command_age_ns = (
                0
                if contact_relief_operation
                else max(0, actual_start_ns - target.submitted_monotonic_ns)
            )
            lateness_ms = max(0.0, (actual_start_ns - due_ns) / 1e6)
            self._last_command_due_ns = due_ns
            self._last_command_actual_start_ns = actual_start_ns
            self._last_command_deadline_lateness_ms = lateness_ms
            if self.diagnostics_enabled:
                self._command_deadline_lateness_ms.append(lateness_ms)
            if (
                self.stale_command_drop_enabled
                and not force_write
                and not contact_relief_operation
                and command_age_ns > self.stale_command_max_age_ns
            ):
                self._stale_drop_count += 1
                self.control.last_command_disposition = "stale_target_dropped"
                self._evaluated_sequence = target.sequence
                actual_end_ns = self._monotonic_ns()
            else:
                command_values = (
                    contact_relief_values
                    if contact_relief_operation
                    else target.values
                )
                if not contact_relief_operation and target.measured_activation:
                    # Activation is intentionally a forced write, but the
                    # hand can move a small amount between the producer's
                    # feedback snapshot and this worker cycle. Refresh the
                    # activation payload from the worker-owned latest
                    # ANGLE_ACT immediately before the write so continuity is
                    # exact at the serial boundary rather than faulting on a
                    # stale-but-safe activation sample.
                    with self._lock:
                        feedback = self._feedback
                    if feedback is not None:
                        command_values = tuple(
                            min(1.0, max(0.0, float(value)))
                            for value in feedback.position_normalized
                        )
                written = self.control.command(
                    command_values,
                    actual_start_ns,
                    submitted_monotonic_ns=(
                        None
                        if contact_relief_operation
                        else target.submitted_monotonic_ns
                    ),
                    target_sequence=(
                        None if contact_relief_operation else target.sequence
                    ),
                    force_write=True if contact_relief_operation else force_write,
                    measured_activation_write=(
                        False if contact_relief_operation else target.measured_activation
                    ),
                    contact_relief=contact_relief_operation,
                )
                actual_end_ns = self._monotonic_ns()
                self._note_io(actual_start_ns, actual_end_ns)
                if written:
                    if not contact_relief_operation:
                        self._written_sequence = target.sequence
                    self._write_count += 1
                    submit_to_write_ms = max(
                        0.0,
                        (
                            actual_end_ns - target.submitted_monotonic_ns
                            if not contact_relief_operation
                            else 0
                        )
                        / 1e6,
                    )
                    self._last_submit_to_write_ms = submit_to_write_ms
                    if self.diagnostics_enabled:
                        self._submit_to_write_ms.append(submit_to_write_ms)
                    if (
                        self.diagnostics_enabled
                        and self._last_write_ns is not None
                        and actual_start_ns > self._last_write_ns
                    ):
                        self._write_interval_ms.append(
                            (actual_start_ns - self._last_write_ns) / 1e6
                        )
                    if self._first_write_ns is None:
                        self._first_write_ns = actual_start_ns
                    self._last_write_ns = actual_start_ns
                if (
                    not contact_relief_operation
                    and (
                        written
                        or self.control.last_command_disposition
                        == "exact_duplicate_suppressed"
                    )
                ):
                    self._evaluated_sequence = target.sequence
            if target is not None:
                self._observed_sequence = max(self._observed_sequence, target.sequence)
            self._last_command_actual_end_ns = actual_end_ns
            if self.control.last_command_disposition == "rate_limited":
                self._command_due_ns = self.control.next_command_monotonic_ns
            else:
                self._command_due_ns = self._advance_deadline(
                    due_ns, self.control.command_period_ns, actual_end_ns
                )
            self.control.next_command_monotonic_ns = self._command_due_ns
        else:
            register = self._select_feedback_register(now_ns)
            if register is not None:
                operation = register
                schedule = self._feedback_schedule[register]
                due_ns = int(schedule["next_due_ns"])
                actual_start_ns = self._monotonic_ns()
                schedule["last_attempt_ns"] = actual_start_ns
                lateness_ms = max(0.0, (actual_start_ns - due_ns) / 1e6)
                cast_lateness = schedule["lateness_ms"]
                assert isinstance(cast_lateness, deque)
                if self.diagnostics_enabled:
                    cast_lateness.append(lateness_ms)
                try:
                    feedback = self.control.poll_feedback_register(
                        register, actual_start_ns
                    )
                except BaseException:
                    schedule["failure_count"] = int(schedule["failure_count"]) + 1
                    raise
                actual_end_ns = self._monotonic_ns()
                self._note_io(actual_start_ns, actual_end_ns)
                transient_timeout_hold = (
                    self.control.last_command_disposition
                    == "feedback_timeout_hold"
                )
                previous_success_ns = schedule["last_success_ns"]
                if transient_timeout_hold:
                    schedule["failure_count"] = int(schedule["failure_count"]) + 1
                    schedule["warning_active"] = True
                else:
                    schedule["last_success_ns"] = actual_end_ns
                    schedule["success_count"] = int(schedule["success_count"]) + 1
                intervals = schedule["interval_ms"]
                assert isinstance(intervals, deque)
                if (
                    self.diagnostics_enabled
                    and not transient_timeout_hold
                    and isinstance(previous_success_ns, int)
                    and actual_end_ns > previous_success_ns
                ):
                    intervals.append((actual_end_ns - previous_success_ns) / 1e6)
                schedule["next_due_ns"] = self._advance_deadline(
                    due_ns, int(schedule["period_ns"]), actual_end_ns
                )
                if not transient_timeout_hold:
                    schedule["warning_active"] = False
                with self._lock:
                    self._feedback = feedback
                if register == "ANGLE":
                    self._note_feedback(actual_end_ns)
        self._update_feedback_ages(self._monotonic_ns())
        if self.record is not None and operation in {"COMMAND", "ANGLE", "CONTACT_RELIEF"}:
            record_now_ns = self._monotonic_ns()
            full_diagnostics = bool(
                operation == "ANGLE"
                and (
                    self._full_diagnostics_next_ns is None
                    or record_now_ns >= self._full_diagnostics_next_ns
                )
            )
            if full_diagnostics:
                self._full_diagnostics_next_ns = record_now_ns + 1_000_000_000
            record_started_ns = self._monotonic_ns()
            row = self.control.episode_record(
                record_now_ns,
                None if target is None else target.values,
                include_diagnostics=full_diagnostics,
            )
            self._last_record_build_duration_ns = self._monotonic_ns() - record_started_ns
            self._maximum_record_build_duration_ns = max(
                self._maximum_record_build_duration_ns,
                self._last_record_build_duration_ns,
            )
            row["record_type"] = "rh56_telemetry"
            row["rh56_scheduled_operation"] = operation
            row["rh56_contact_relief"] = contact_relief_operation
            row["rh56_worker"] = (
                self.diagnostics_snapshot(include_windows=True)
                if self.diagnostics_enabled and full_diagnostics
                else None
            )
            row["rh56_command_timing"] = (
                {
                    "command_due_ns": self._last_command_due_ns,
                    "command_actual_start_ns": self._last_command_actual_start_ns,
                    "command_actual_end_ns": self._last_command_actual_end_ns,
                    "command_deadline_lateness_ms": (
                        self._last_command_deadline_lateness_ms
                    ),
                    "command_age_at_write_ms": self.control.last_command_age_ms,
                    "submit_to_write_ms": self._last_submit_to_write_ms,
                }
                if operation == "COMMAND"
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
            submit_interval_ms = tuple(self._submit_interval_ms)
            unique_submit_interval_ms = tuple(self._unique_submit_interval_ms)
        result: dict[str, object] = {
            "diagnostics_enabled": self.diagnostics_enabled,
            "mailbox_kind": "latest_only_single_slot",
            "mailbox_capacity": 1,
            "pending_target_sequence": None if pending is None else pending.sequence,
            "submitted_target_count": self._submit_count,
            "measured_activation_target_count": self._activation_target_count,
            "clamped_activation_target_count": self._clamped_activation_target_count,
            "unique_submitted_target_count": self._unique_submit_count,
            "coalesced_unobserved_target_count": self._coalesced_count,
            "last_submitted_sequence": self._submitted_sequence,
            "last_observed_sequence": self._observed_sequence,
            "last_written_sequence": self._written_sequence,
            "successful_serial_write_count": self._write_count,
            "complete_feedback_record_count": self._feedback_count,
            "worker_cycle_count": self._cycle_count,
            "worker_cycle_overrun_count": self._cycle_overrun_count,
            "scheduler_profile": self.control.scheduler_profile,
            "requested_command_rate_hz": self.control.command_rate_hz,
            "command_period_ms": self.control.command_period_ns / 1e6,
            "command_due_ns": self._last_command_due_ns,
            "command_actual_start_ns": self._last_command_actual_start_ns,
            "command_actual_end_ns": self._last_command_actual_end_ns,
            "command_deadline_lateness_ms": self._last_command_deadline_lateness_ms,
            "command_age_at_write_ms": self.control.last_command_age_ms,
            "submit_to_write_ms": self._last_submit_to_write_ms,
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
            "worker_record_build_duration_ns": {
                "last": self._last_record_build_duration_ns,
                "max": self._maximum_record_build_duration_ns,
            },
            "serial_utilization_estimate": self._serial_utilization(),
            "telemetry_emission_policy": (
                "compact_each_command_full_snapshot_each_angle_feedback"
            ),
            "feedback": self._feedback_diagnostics(self._monotonic_ns()),
        }
        if include_windows and self.diagnostics_enabled:
            result["timing_ms"] = {
                "worker_cycle_duration": _distribution(self._cycle_duration_ms),
                "worker_cycle_interval": _distribution(self._cycle_interval_ms),
                "target_submit_interval": _distribution(submit_interval_ms),
                "unique_target_interval": _distribution(
                    unique_submit_interval_ms
                ),
                "successful_write_interval": _distribution(
                    self._write_interval_ms
                ),
                "command_deadline_lateness": _distribution(
                    self._command_deadline_lateness_ms
                ),
                "command_age_at_write": self.control.diagnostics_snapshot().get(
                    "command_age_ms"
                ),
                "submit_to_write": _distribution(self._submit_to_write_ms),
                "complete_feedback_interval": _distribution(
                    self._feedback_interval_ms
                ),
                "worker_cycle_interval_jitter": _jitter(
                    self._cycle_interval_ms
                ),
            }
        return result

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                if not self.run_cycle():
                    return
                self._wake.clear()
                now_ns = self._monotonic_ns()
                wake_ns = self._next_wake_ns(now_ns)
                if wake_ns > now_ns:
                    self._wake.wait((wake_ns - now_ns) / 1e9)
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

    @staticmethod
    def _advance_deadline(due_ns: int, period_ns: int, now_ns: int) -> int:
        next_due_ns = due_ns + period_ns
        if next_due_ns <= now_ns:
            next_due_ns += ((now_ns - next_due_ns) // period_ns + 1) * period_ns
        return next_due_ns

    def _select_feedback_register(self, now_ns: int) -> str | None:
        for name in ("STATUS", "ERROR"):
            last_success_ns = self._feedback_schedule[name]["last_success_ns"]
            if isinstance(last_success_ns, int) and (
                now_ns - last_success_ns >= self.control.feedback_warning_age_ns[name]
            ):
                return name
        due = [
            name
            for name, schedule in self._feedback_schedule.items()
            if isinstance(schedule["next_due_ns"], int)
            and now_ns >= int(schedule["next_due_ns"])
        ]
        if not due:
            return None
        priority = {"ANGLE": 0, "STATUS": 1, "ERROR": 2, "CURRENT": 3, "FORCE": 4}
        return min(
            due,
            key=lambda name: (
                int(self._feedback_schedule[name]["next_due_ns"]),
                priority[name],
            ),
        )

    def _next_wake_ns(self, now_ns: int) -> int:
        with self._lock:
            active = self._active_requested
            target = self._pending_target
            force_write = self._force_write_pending
        if (
            active
            and target is not None
            and self._target_requires_command(target)
            and force_write
        ):
            return now_ns
        candidates = [
            int(schedule["next_due_ns"])
            for schedule in self._feedback_schedule.values()
            if isinstance(schedule["next_due_ns"], int)
        ]
        for name in ("STATUS", "ERROR"):
            last_success_ns = self._feedback_schedule[name]["last_success_ns"]
            if isinstance(last_success_ns, int):
                candidates.append(
                    last_success_ns + self.control.feedback_warning_age_ns[name]
                )
        if (
            active
            and target is not None
            and self._target_requires_command(target)
        ):
            candidates.append(
                now_ns if self._command_due_ns is None else self._command_due_ns
            )
        return min(candidates, default=now_ns + 1_000_000)

    def _target_requires_command(self, target: PendingTarget | None) -> bool:
        if target is None:
            return False
        if target.sequence > self._evaluated_sequence:
            return True
        # A measured activation is a one-shot continuity write.  Once its
        # forced command has been evaluated, do not replay the same mailbox
        # entry merely because ANGLE_ACT moved by a count before the next
        # ordinary hand target arrived.  Replaying it without the activation
        # force flag is a safety error; a fresh submit_target() will replace
        # this entry and carry the normal command cadence forward.
        if target.measured_activation:
            return False
        effective = self.control.contact_limited_target(
            target.values, allow_release=False
        )
        return bool(
            target.sequence == self._written_sequence
            and self.control.last_command_normalized is not None
            and tuple(float(value) for value in effective)
            != self.control.last_command_normalized
        )

    def _note_io(self, started_ns: int, ended_ns: int) -> None:
        self._io_busy_ns += max(0, ended_ns - started_ns)
        if self._first_io_start_ns is None:
            self._first_io_start_ns = started_ns
        self._last_io_end_ns = ended_ns

    def _serial_utilization(self) -> float | None:
        if (
            self._first_io_start_ns is None
            or self._last_io_end_ns is None
            or self._last_io_end_ns <= self._first_io_start_ns
        ):
            return None
        return min(
            1.0,
            self._io_busy_ns / (self._last_io_end_ns - self._first_io_start_ns),
        )

    def _update_feedback_ages(self, now_ns: int) -> None:
        for name, schedule in self._feedback_schedule.items():
            last_success_ns = schedule["last_success_ns"]
            if not isinstance(last_success_ns, int):
                continue
            age_ms = max(0.0, (now_ns - last_success_ns) / 1e6)
            ages = schedule["age_ms"]
            assert isinstance(ages, deque)
            if self.diagnostics_enabled:
                ages.append(age_ms)
            overdue = now_ns - last_success_ns > self.control.feedback_warning_age_ns[name]
            if overdue and not bool(schedule["warning_active"]):
                schedule["warning_count"] = int(schedule["warning_count"]) + 1
            schedule["warning_active"] = overdue

    def _feedback_diagnostics(self, now_ns: int) -> dict[str, object]:
        result: dict[str, object] = {}
        for name, schedule in self._feedback_schedule.items():
            last_success_ns = schedule["last_success_ns"]
            success_count = int(schedule["success_count"])
            first_success_ns = schedule["first_success_ns"]
            result[name] = {
                "requested_rate_hz": self.control.feedback_rate_hz[name],
                "achieved_rate_hz": _rate(
                    success_count,
                    first_success_ns if isinstance(first_success_ns, int) else None,
                    last_success_ns if isinstance(last_success_ns, int) else None,
                ),
                "next_due_ns": schedule["next_due_ns"],
                "last_attempt_ns": schedule["last_attempt_ns"],
                "last_success_ns": last_success_ns,
                "age_ms": None
                if not isinstance(last_success_ns, int)
                else max(0.0, (now_ns - last_success_ns) / 1e6),
                "warning_age_ms": self.control.feedback_warning_age_ns[name] / 1e6,
                "warning_active": bool(schedule["warning_active"]),
                "warning_count": int(schedule["warning_count"]),
                "failure_count": int(schedule["failure_count"]),
                "deadline_lateness_ms": _distribution(schedule["lateness_ms"]),
                "observed_age_ms": _distribution(schedule["age_ms"]),
                "success_interval_ms": _distribution(schedule["interval_ms"]),
            }
        return result


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
