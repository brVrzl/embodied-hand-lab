from __future__ import annotations

from collections import Counter
import threading

import pytest

from embodiment_core.config import load_yaml
from rh56_driver.pc_direct_control import (
    FakeRH56PcDirectBackend,
    HandOperation,
    RH56PcDirectControl,
)
from rh56_driver.pc_direct_worker import RH56PcDirectWorker


class ManualClock:
    def __init__(self, now_ns: int = 1_000_000_000) -> None:
        self.now_ns = now_ns

    def __call__(self) -> int:
        return self.now_ns

    def advance_ms(self, value: float) -> None:
        self.now_ns += int(round(value * 1e6))


class ScheduledBackend(FakeRH56PcDirectBackend):
    def __init__(self, clock: ManualClock) -> None:
        super().__init__()
        self.clock = clock
        self.operations: list[str] = []
        self.duration_ms: dict[str, float] = {}
        self.angle_hook = None
        self.command_hook = None

    def _operation(self, name: str, callback):
        self.operations.append(name)
        if name == "ANGLE" and self.angle_hook is not None:
            hook = self.angle_hook
            self.angle_hook = None
            hook()
        if name == "COMMAND" and self.command_hook is not None:
            hook = self.command_hook
            self.command_hook = None
            hook()
        result = callback()
        self.clock.advance_ms(self.duration_ms.get(name, 0.0))
        return result

    def get_canonical_angles(self) -> list[float]:
        return self._operation("ANGLE", super().get_canonical_angles)

    def get_canonical_currents(self) -> list[float]:
        return self._operation("CURRENT", super().get_canonical_currents)

    def get_canonical_forces(self) -> list[float]:
        return self._operation("FORCE", super().get_canonical_forces)

    def read_register(self, address: int, length: int) -> list[int]:
        name = "STATUS" if address == self.REG["STATUS"] else "ERROR"
        return self._operation(
            name, lambda: super(ScheduledBackend, self).read_register(address, length)
        )

    def set_canonical_angles(self, values: list[int]) -> bool:
        return self._operation(
            "COMMAND",
            lambda: super(ScheduledBackend, self).set_canonical_angles(values),
        )


def _config(profile: str = "fast30") -> dict:
    rates = {
        "baseline": (15, 15, 15, 15, 15, 15),
        "fast30": (30, 15, 10, 10, 10, 10),
        "fast40": (40, 15, 10, 10, 10, 10),
    }[profile]
    return {
        "scheduler_profile": profile,
        "scheduler_profiles": {
            profile: {
                key: value
                for key, value in zip(
                    (
                        "command_rate_hz",
                        "angle_feedback_rate_hz",
                        "current_feedback_rate_hz",
                        "force_feedback_rate_hz",
                        "status_feedback_rate_hz",
                        "error_feedback_rate_hz",
                    ),
                    rates,
                )
            }
        },
        "feedback_stale_timeout_sec": 0.4,
        "serial": {"timeout_sec": 0.2},
        "hand_schema": {
            "protocol_order": [
                "index", "middle", "ring", "pinky", "thumb_close", "thumb_lateral"
            ],
            "hand_delta_limit": 0.05,
        },
        "safety": {"max_close_strength": 0.8},
        "diagnostics": {
            "enabled": True,
            "window_size": 256,
            "exact_duplicate_suppression": True,
        },
    }


def _worker(profile: str = "fast30"):
    clock = ManualClock()
    backend = ScheduledBackend(clock)
    control = RH56PcDirectControl(
        backend, _config(profile), perf_counter_ns=clock
    )
    worker = RH56PcDirectWorker(control, monotonic_ns=clock)
    first = worker.start(HandOperation.COMBINED, run_in_thread=False)
    backend.operations.clear()
    return worker, control, backend, clock, first


@pytest.mark.parametrize("profile, expected_period_ms", [("fast30", 1000 / 30), ("fast40", 25.0)])
def test_command_deadline_intervals_follow_requested_rate(
    profile: str, expected_period_ms: float
) -> None:
    worker, _control, backend, clock, first = _worker(profile)
    try:
        worker.activate_from_measured(first.monotonic_ns)
        worker.submit_target([0.1] * 6, clock())
        worker.run_cycle()
        first_due = worker.diagnostics_snapshot()["command_due_ns"]
        clock.advance_ms(expected_period_ms)
        worker.submit_target([0.2] * 6, clock())
        worker.run_cycle()
        second_due = worker.diagnostics_snapshot()["command_due_ns"]
        assert (second_due - first_due) / 1e6 == pytest.approx(expected_period_ms)
        assert backend.operations == ["COMMAND", "COMMAND"]
    finally:
        worker.cleanup()


def test_command_priority_then_status_error_max_age_prevents_starvation() -> None:
    worker, _control, backend, clock, first = _worker("fast40")
    try:
        clock.advance_ms(260)
        worker.activate_from_measured(first.monotonic_ns)
        worker.submit_target([0.05] * 6, clock())
        worker.run_cycle()
        worker.run_cycle()
        worker.run_cycle()
        assert backend.operations[:3] == ["COMMAND", "STATUS", "ERROR"]
        feedback = worker.diagnostics_snapshot()["feedback"]
        assert feedback["STATUS"]["age_ms"] == pytest.approx(0.0)
        assert feedback["ERROR"]["age_ms"] == pytest.approx(0.0)
    finally:
        worker.cleanup()


def test_feedback_registers_run_at_independent_rates() -> None:
    worker, _control, backend, clock, _first = _worker("fast30")
    try:
        for _ in range(1000):
            clock.advance_ms(1)
            while True:
                count = len(backend.operations)
                worker.run_cycle()
                if len(backend.operations) == count:
                    break
        counts = Counter(backend.operations)
        assert 14 <= counts["ANGLE"] <= 15
        for name in ("CURRENT", "FORCE", "STATUS", "ERROR"):
            assert 9 <= counts[name] <= 10
        diagnostics = worker.diagnostics_snapshot()["feedback"]
        assert diagnostics["ANGLE"]["requested_rate_hz"] == 15
        assert diagnostics["STATUS"]["requested_rate_hz"] == 10
    finally:
        worker.cleanup()


def test_slow_single_feedback_is_followed_by_due_command_not_remaining_full_poll() -> None:
    worker, _control, backend, clock, first = _worker("fast40")
    try:
        worker.activate_from_measured(first.monotonic_ns)
        worker.submit_target([0.05] * 6, clock())
        worker.run_cycle()
        backend.operations.clear()
        backend.duration_ms["ANGLE"] = 80.0
        clock.advance_ms(70)

        def submit_during_read() -> None:
            clock.advance_ms(40)
            worker.submit_target([0.3] * 6, clock())

        backend.angle_hook = submit_during_read
        worker.run_cycle()
        worker.run_cycle()
        assert backend.operations[:2] == ["ANGLE", "COMMAND"]
        assert worker.diagnostics_snapshot()["last_written_sequence"] == 3
    finally:
        worker.cleanup()


def test_one_latest_target_continues_delta_limited_progress_without_duplicate_writes() -> None:
    worker, control, backend, clock, first = _worker("fast40")
    try:
        worker.activate_from_measured(first.monotonic_ns)
        worker.submit_target([0.2] * 6, clock())
        for _ in range(4):
            worker.run_cycle()
            clock.advance_ms(25)
        assert len(backend.position_writes) == 4
        assert control.last_command_normalized == pytest.approx([0.2] * 6)
        worker.run_cycle()
        assert len(backend.position_writes) == 4
    finally:
        worker.cleanup()


def test_activation_force_is_consumed_before_concurrent_latest_target_arrives() -> None:
    worker, _control, backend, clock, first = _worker("fast40")
    try:
        worker.activate_from_measured(first.monotonic_ns)
        worker.submit_target([0.2] * 6, clock())
        backend.command_hook = lambda: worker.submit_target([0.3] * 6, clock())
        worker.run_cycle()
        assert backend.operations == ["COMMAND"]

        # The target submitted during the forced write remains latest-only,
        # but it is an ordinary target and cannot bypass the next deadline.
        worker.run_cycle()
        assert backend.operations == ["COMMAND"]
        clock.advance_ms(25)
        worker.run_cycle()
        assert backend.operations == ["COMMAND", "COMMAND"]
        assert worker.diagnostics_snapshot()["last_written_sequence"] == 3
    finally:
        worker.cleanup()


def test_measured_activation_is_not_replayed_after_angle_feedback_changes() -> None:
    worker, _control, backend, clock, first = _worker("fast40")
    try:
        worker.activate_from_measured(first.monotonic_ns)
        worker.run_cycle()
        assert backend.position_writes == [[1000] * 6]

        # A later ANGLE_ACT sample may differ by a raw count while the
        # producer has not yet submitted the first ordinary hand target.
        backend.position = [999.0] * 6
        clock.advance_ms(25)
        worker.run_cycle()

        assert backend.position_writes == [[1000] * 6]
        worker.raise_if_failed()
    finally:
        worker.cleanup()


def test_failed_write_is_not_committed_or_suppressed_in_next_authorized_session() -> None:
    class RejectOnceBackend(ScheduledBackend):
        def set_canonical_angles(self, values: list[int]) -> bool:
            self.operations.append("COMMAND")
            return False

    clock = ManualClock()
    rejected_backend = RejectOnceBackend(clock)
    control = RH56PcDirectControl(
        rejected_backend, _config("fast30"), perf_counter_ns=clock
    )
    worker = RH56PcDirectWorker(control, monotonic_ns=clock)
    first = worker.start(HandOperation.COMBINED, run_in_thread=False)
    target = [0.2] * 6
    worker.activate_from_measured(first.monotonic_ns)
    worker.submit_target(target, clock())
    with pytest.raises(RuntimeError, match="rejected"):
        worker.run_cycle()
    assert worker.diagnostics_snapshot()["last_written_sequence"] is None
    assert control.successful_command_write_count == 0
    worker.cleanup()

    # Serial/ack failure is a hard stop; retry is permitted only in a new,
    # separately authorized session. The target was not poisoned as duplicate.
    retry_worker, retry_control, retry_backend, _clock, retry_first = _worker("fast30")
    try:
        retry_worker.activate_from_measured(retry_first.monotonic_ns)
        retry_worker.submit_target(target, retry_first.monotonic_ns)
        retry_worker.run_cycle()
        assert retry_backend.position_writes
        assert retry_control.last_command_disposition == "serial_write_success"
    finally:
        retry_worker.cleanup()


def test_profiles_resolve_for_hand_only_and_combined_config_consumers() -> None:
    config = load_yaml("configs/hand/rh56_pc_direct_teleop.yaml")
    expected = {"baseline": 15, "fast30": 30, "fast40": 40, "fast50": 50}
    for profile, command_rate in expected.items():
        selected = dict(config)
        selected["scheduler_profile"] = profile
        hand_only = RH56PcDirectControl(FakeRH56PcDirectBackend(), selected)
        combined = RH56PcDirectControl(FakeRH56PcDirectBackend(), selected)
        assert hand_only.command_rate_hz == combined.command_rate_hz == command_rate
        assert hand_only.feedback_rate_hz == combined.feedback_rate_hz


def test_diagnostics_report_requested_and_achieved_command_rates() -> None:
    worker, _control, _backend, clock, first = _worker("fast30")
    try:
        worker.activate_from_measured(first.monotonic_ns)
        for index in range(4):
            worker.submit_target([0.1 + index * 0.01] * 6, clock())
            worker.run_cycle()
            clock.advance_ms(1000 / 30)
        diagnostics = worker.diagnostics_snapshot()
        assert diagnostics["requested_command_rate_hz"] == 30
        assert diagnostics["successful_serial_write_rate_hz"] == pytest.approx(30.0)
        assert diagnostics["timing_ms"]["command_deadline_lateness"]["max"] == pytest.approx(0.0)
    finally:
        worker.cleanup()


def test_logging_is_compact_per_command_and_full_only_on_angle_feedback() -> None:
    rows: list[dict[str, object]] = []
    clock = ManualClock()
    backend = ScheduledBackend(clock)
    control = RH56PcDirectControl(
        backend, _config("fast30"), perf_counter_ns=clock
    )
    worker = RH56PcDirectWorker(control, monotonic_ns=clock, record=rows.append)
    first = worker.start(HandOperation.COMBINED, run_in_thread=False)
    try:
        worker.activate_from_measured(first.monotonic_ns)
        worker.submit_target([0.05] * 6, clock())
        worker.run_cycle()
        assert rows[-1]["rh56_scheduled_operation"] == "COMMAND"
        assert rows[-1]["rh56_command_timing"] is not None
        assert rows[-1]["rh56_worker"] is None
        assert rows[-1]["rh56_diagnostics"] is None

        clock.advance_ms(70)
        worker.run_cycle()
        assert rows[-1]["rh56_scheduled_operation"] == "ANGLE"
        assert rows[-1]["rh56_command_timing"] is None
        assert rows[-1]["rh56_worker"] is not None
        assert rows[-1]["rh56_diagnostics"] is not None
    finally:
        worker.cleanup()


def test_diagnostics_snapshot_copies_producer_windows_under_mailbox_lock() -> None:
    worker, _control, _backend, clock, _first = _worker("fast30")
    failures: list[BaseException] = []

    def produce() -> None:
        try:
            for index in range(5000):
                clock.advance_ms(0.01)
                worker.submit_target([float(index % 80) / 100.0] * 6, clock())
        except BaseException as exc:
            failures.append(exc)

    producer = threading.Thread(target=produce)
    producer.start()
    try:
        while producer.is_alive():
            snapshot = worker.diagnostics_snapshot()
            assert snapshot["timing_ms"]["target_submit_interval"] is not None
        producer.join()
        assert not failures
    finally:
        producer.join()
        worker.cleanup()
