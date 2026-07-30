from __future__ import annotations

import threading
import time
from typing import Callable, Sequence

from .pc_direct_control import PcDirectFeedback, RH56PcDirectControl


class RH56PcDirectWorker:
    """Own the one PC-direct controller without blocking the arm producer."""

    def __init__(
        self,
        control: RH56PcDirectControl,
        *,
        record: Callable[[dict[str, object]], None] | None = None,
    ) -> None:
        self.control = control
        self.record = record
        self.max_target_normalized = control.max_close
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._active_requested = False
        self._pending_target: tuple[float, ...] | None = None
        self._hold_reason = "startup"
        self._terminal_reason: str | None = None
        self._feedback: PcDirectFeedback | None = None
        self._failure: BaseException | None = None

    def start(self, approval_token: str) -> PcDirectFeedback:
        self.control.open(approval_token)
        feedback = self.control.poll_feedback(time.monotonic_ns())
        with self._lock:
            self._feedback = feedback
        self._thread = threading.Thread(target=self._run, name="rh56-pc-direct", daemon=True)
        self._thread.start()
        return feedback

    def activate_from_measured(self, monotonic_ns: int) -> tuple[float, ...]:
        self.raise_if_failed()
        with self._lock:
            feedback = self._feedback
            if (
                feedback is None
                or monotonic_ns - feedback.monotonic_ns > self.control.feedback_stale_timeout_ns
            ):
                raise RuntimeError("RH56 feedback is stale or absent at grip engagement.")
            self._active_requested = True
            self._pending_target = feedback.position_normalized
            return feedback.position_normalized

    def submit_target(self, target: Sequence[float], monotonic_ns: int) -> None:
        del monotonic_ns
        self.raise_if_failed()
        values = tuple(float(value) for value in target)
        if len(values) != 6:
            raise ValueError("RH56 worker target must have six canonical channels.")
        with self._lock:
            if self._terminal_reason is None:
                self._pending_target = values

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
        return self._failure is not None

    def raise_if_failed(self) -> None:
        if self._failure is not None:
            raise RuntimeError("RH56 PC-direct worker failed") from self._failure

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
                    raise RuntimeError("RH56 PC-direct worker did not stop after serial close")
                self.control.cleanup()
                return
        self.control.cleanup()

    def _run(self) -> None:
        next_cycle_ns = time.monotonic_ns()
        try:
            while not self._stop.is_set():
                now_ns = time.monotonic_ns()
                feedback = self.control.poll_feedback(now_ns)
                with self._lock:
                    self._feedback = feedback
                    active = self._active_requested
                    target = self._pending_target
                    hold_reason = self._hold_reason
                    terminal_reason = self._terminal_reason
                if terminal_reason is not None:
                    self.control.arm_terminal_stop(terminal_reason)
                    return
                if active:
                    if self.control.state.value == "HAND_HOLD":
                        self.control.activate(now_ns)
                    if target is not None:
                        self.control.command(target, now_ns)
                else:
                    self.control.hold(hold_reason)
                if self.record is not None:
                    self.record(self.control.episode_record(now_ns, target))
                next_cycle_ns += self.control.command_period_ns
                wait_ns = next_cycle_ns - time.monotonic_ns()
                if wait_ns <= 0:
                    next_cycle_ns = time.monotonic_ns()
                    continue
                self._stop.wait(wait_ns / 1e9)
        except BaseException as exc:
            self._failure = exc
            if self.control.fault_reason is None:
                self.control.transport_fault("pc_direct_worker_failure")
