from __future__ import annotations

import io
import threading
import time

import pytest

from rh56_driver.pc_direct_control import (
    RH56_COMBINED_RUN_APPROVAL,
    RH56_HAND_ONLY_COMMAND_APPROVAL,
    FakeRH56PcDirectBackend,
    RH56PcDirectControl,
)
from rh56_driver.pc_direct_worker import RH56PcDirectWorker
from rh56_driver.serial_backend import RH56SerialBackend, RH56SerialError
from rh56_driver.telemetry import BoundedJsonlRecorder


class ManualClock:
    def __init__(self, now_ns: int = 1_000_000_000) -> None:
        self.now_ns = now_ns

    def __call__(self) -> int:
        return self.now_ns

    def advance_ms(self, value: float) -> None:
        self.now_ns += int(round(value * 1e6))


def _config(*, diagnostics: bool = True) -> dict:
    return {
        "control_frequency_hz": 15,
        "feedback_stale_timeout_sec": 0.4,
        "serial": {"timeout_sec": 0.2},
        "hand_schema": {
            "protocol_order": [
                "index",
                "middle",
                "ring",
                "pinky",
                "thumb_close",
                "thumb_lateral",
            ],
            "hand_delta_limit": 0.05,
        },
        "safety": {"max_close_strength": 0.8},
        "diagnostics": {
            "enabled": diagnostics,
            "window_size": 8,
            "exact_duplicate_suppression": True,
            "stale_command_drop_enabled": False,
            "stale_command_max_age_sec": 0.25,
        },
    }


def _manual_worker(
    *, diagnostics: bool = True, backend: FakeRH56PcDirectBackend | None = None
) -> tuple[RH56PcDirectWorker, RH56PcDirectControl, FakeRH56PcDirectBackend, ManualClock]:
    selected_backend = backend or FakeRH56PcDirectBackend()
    clock = ManualClock()
    control = RH56PcDirectControl(selected_backend, _config(diagnostics=diagnostics))
    worker = RH56PcDirectWorker(control, monotonic_ns=clock)
    first = worker.start(RH56_COMBINED_RUN_APPROVAL, run_in_thread=False)
    worker.activate_from_measured(first.monotonic_ns)
    return worker, control, selected_backend, clock


def test_latest_only_mailbox_coalesces_unobserved_targets_and_writes_latest_sequence() -> None:
    worker, _control, backend, clock = _manual_worker()
    try:
        worker.submit_target([0.01] * 6, clock())
        worker.submit_target([0.3] * 6, clock())
        clock.advance_ms(10)
        assert worker.run_cycle()

        assert backend.position_writes == [[950] * 6]
        diagnostics = worker.diagnostics_snapshot()
        assert diagnostics["mailbox_kind"] == "latest_only_single_slot"
        assert diagnostics["mailbox_capacity"] == 1
        assert diagnostics["coalesced_unobserved_target_count"] >= 1
        assert diagnostics["last_written_sequence"] == diagnostics["last_submitted_sequence"]
    finally:
        worker.cleanup()


def test_exact_duplicate_is_suppressed_but_different_and_forced_targets_write() -> None:
    worker, control, backend, clock = _manual_worker()
    try:
        worker.submit_target([0.05] * 6, clock())
        assert worker.run_cycle()
        assert len(backend.position_writes) == 1

        clock.advance_ms(70)
        assert worker.run_cycle()
        assert len(backend.position_writes) == 1
        assert control.last_command_disposition == "exact_duplicate_suppressed"

        worker.submit_target([0.0501] * 6, clock())
        clock.advance_ms(70)
        assert worker.run_cycle()
        # The distinct normalized target is not suppressed even though both
        # values quantize to the same integer protocol count.
        assert len(backend.position_writes) == 2

        clock.advance_ms(70)
        assert control.command(
            control.last_command_normalized,
            clock(),
            force_write=True,
            target_sequence=999,
        )
        assert len(backend.position_writes) == 3
        assert control.last_written_sequence == 999
    finally:
        worker.cleanup()


def test_command_sequence_age_and_feedback_register_timings_are_reported() -> None:
    worker, control, _backend, clock = _manual_worker()
    try:
        submitted_ns = clock()
        worker.submit_target([0.2] * 6, submitted_ns)
        clock.advance_ms(25)
        worker.run_cycle()

        record = control.episode_record(clock(), [0.2] * 6)
        assert record["hand_written_sequence"] == record["hand_target_sequence"]
        assert record["hand_command_age_ms"] == pytest.approx(25.0)
        assert set(record["hand_feedback_register_latency_ms"]) == {
            "ANGLE",
            "CURRENT",
            "FORCE",
            "STATUS",
            "ERROR",
        }
        diagnostics = record["rh56_diagnostics"]
        assert diagnostics["successful_serial_write_count"] == 1
        assert diagnostics["complete_feedback_record_count"] >= 2
        assert diagnostics["command_age_ms"]["p95"] == pytest.approx(25.0)
        assert diagnostics["complete_feedback_latency_ms"]["max"] >= 0.0
        worker_timing = worker.diagnostics_snapshot()["timing_ms"]
        assert worker_timing["complete_feedback_interval"]["max"] == pytest.approx(25.0)
    finally:
        worker.cleanup()


def test_command_follows_complete_feedback_poll_on_the_one_backend() -> None:
    class OrderedBackend(FakeRH56PcDirectBackend):
        def __init__(self) -> None:
            super().__init__()
            self.operations: list[str] = []

        def get_canonical_angles(self) -> list[float]:
            self.operations.append("ANGLE")
            return super().get_canonical_angles()

        def get_canonical_currents(self) -> list[float]:
            self.operations.append("CURRENT")
            return super().get_canonical_currents()

        def get_canonical_forces(self) -> list[float]:
            self.operations.append("FORCE")
            return super().get_canonical_forces()

        def read_register(self, address: int, length: int) -> list[int]:
            self.operations.append(
                "STATUS" if address == self.REG["STATUS"] else "ERROR"
            )
            return super().read_register(address, length)

        def set_canonical_angles(self, values: list[int]) -> bool:
            self.operations.append("COMMAND")
            return super().set_canonical_angles(values)

    backend = OrderedBackend()
    worker, _control, _backend, clock = _manual_worker(backend=backend)
    try:
        backend.operations.clear()
        worker.submit_target([0.2] * 6, clock())
        worker.run_cycle()
        assert backend.operations == [
            "ANGLE",
            "CURRENT",
            "FORCE",
            "STATUS",
            "ERROR",
            "COMMAND",
        ]
    finally:
        worker.cleanup()


def test_worker_is_the_only_serial_backend_caller_and_never_overlaps_io() -> None:
    class ConcurrencyGuardBackend(FakeRH56PcDirectBackend):
        def __init__(self) -> None:
            super().__init__()
            self._io_lock = threading.Lock()
            self.overlap_count = 0

        def _guarded(self, operation):
            if not self._io_lock.acquire(blocking=False):
                self.overlap_count += 1
                raise RuntimeError("concurrent fake serial access")
            try:
                time.sleep(0.0001)
                return operation()
            finally:
                self._io_lock.release()

        def get_canonical_angles(self) -> list[float]:
            return self._guarded(super().get_canonical_angles)

        def get_canonical_currents(self) -> list[float]:
            return self._guarded(super().get_canonical_currents)

        def get_canonical_forces(self) -> list[float]:
            return self._guarded(super().get_canonical_forces)

        def read_register(self, address: int, length: int) -> list[int]:
            return self._guarded(
                lambda: super(ConcurrencyGuardBackend, self).read_register(
                    address, length
                )
            )

        def set_canonical_angles(self, values: list[int]) -> bool:
            return self._guarded(
                lambda: super(ConcurrencyGuardBackend, self).set_canonical_angles(
                    values
                )
            )

    backend = ConcurrencyGuardBackend()
    config = _config()
    config["control_frequency_hz"] = 100
    control = RH56PcDirectControl(backend, config)
    worker = RH56PcDirectWorker(control)
    first = worker.start(RH56_COMBINED_RUN_APPROVAL)
    try:
        worker.activate_from_measured(first.monotonic_ns)
        for index in range(40):
            worker.submit_target([min(0.8, index / 100)] * 6, time.monotonic_ns())
        time.sleep(0.05)
        worker.raise_if_failed()
        assert backend.overlap_count == 0
    finally:
        worker.cleanup()


def test_stale_command_drop_is_default_off_and_optional_drop_exempts_activation() -> None:
    worker, _control, backend, clock = _manual_worker()
    try:
        worker.submit_target([0.2] * 6, clock())
        clock.advance_ms(300)
        worker.run_cycle()
        assert backend.position_writes
        assert worker.diagnostics_snapshot()["stale_command_drop_enabled"] is False
    finally:
        worker.cleanup()

    config = _config()
    config["diagnostics"]["stale_command_drop_enabled"] = True
    backend2 = FakeRH56PcDirectBackend()
    clock2 = ManualClock()
    control2 = RH56PcDirectControl(backend2, config)
    worker2 = RH56PcDirectWorker(control2, monotonic_ns=clock2)
    first = worker2.start(RH56_COMBINED_RUN_APPROVAL, run_in_thread=False)
    try:
        worker2.activate_from_measured(first.monotonic_ns)
        clock2.advance_ms(300)
        # Measured activation target is exempt from stale dropping.
        worker2.run_cycle()
        assert backend2.position_writes == [[1000] * 6]
        worker2.submit_target([0.2] * 6, clock2())
        clock2.advance_ms(300)
        worker2.run_cycle()
        assert backend2.position_writes == [[1000] * 6]
        assert worker2.diagnostics_snapshot()["stale_command_drop_count"] == 1
    finally:
        worker2.cleanup()


def test_diagnostics_toggle_does_not_change_target_values_or_channel_order() -> None:
    writes: list[list[list[int]]] = []
    for enabled in (False, True):
        backend = FakeRH56PcDirectBackend()
        control = RH56PcDirectControl(backend, _config(diagnostics=enabled))
        control.open(RH56_HAND_ONLY_COMMAND_APPROVAL)
        control.poll_feedback(1_000_000_000)
        control.activate(1_000_000_000)
        assert control.command([0.01, 0.02, 0.03, 0.04, 0.05, 0.06], 1_000_000_000)
        writes.append(backend.position_writes)
        record = control.episode_record(1_000_000_000)
        assert (record["rh56_diagnostics"] is not None) is enabled
        control.cleanup()
    assert writes[0] == writes[1] == [[990, 980, 970, 960, 950, 950]]


def test_feedback_timeout_and_protocol_decode_error_have_structured_context() -> None:
    backend = RH56SerialBackend({"serial": {"hand_id": 1}})
    backend._exchange = lambda payload, expected_frames: []  # type: ignore[method-assign]
    with pytest.raises(RH56SerialError) as timeout:
        backend.read_register(backend.REG["CURRENT"], 12)
    assert timeout.value.as_dict()["code"] == "timeout"
    assert timeout.value.as_dict()["register"] == "CURRENT"

    bad_frame = bytes([0x90, 0xEB, 1, 3, 0x11, 0, 0, 0])
    backend._exchange = lambda payload, expected_frames: [bad_frame]  # type: ignore[method-assign]
    with pytest.raises(RH56SerialError) as protocol:
        backend.read_register(backend.REG["ANGLE_ACT"], 12)
    assert protocol.value.as_dict()["code"] in {
        "checksum_failure",
        "response_validation_failure",
        "response_length_mismatch",
    }
    assert protocol.value.as_dict()["register"] == "ANGLE_ACT"


def test_command_write_error_is_structured_in_control_failure() -> None:
    class FailingWriteBackend(FakeRH56PcDirectBackend):
        def set_canonical_angles(self, values: list[int]) -> bool:
            raise RH56SerialError(
                "offline write failure",
                code="timeout",
                operation="write_register",
                address=1486,
                register="ANGLE_SET",
            )

    backend = FailingWriteBackend()
    control = RH56PcDirectControl(backend, _config())
    control.open(RH56_HAND_ONLY_COMMAND_APPROVAL)
    control.poll_feedback(1_000_000_000)
    control.activate(1_000_000_000)
    with pytest.raises(RH56SerialError):
        control.command([0.2] * 6, 1_000_000_000, target_sequence=4)
    failure = control.last_failure_record
    assert failure is not None
    assert failure["stage"] == "command_write"
    assert failure["serial"]["register"] == "ANGLE_SET"
    assert failure["context"]["target_sequence"] == 4


def test_logging_callback_failure_does_not_kill_serial_worker() -> None:
    worker, _control, backend, clock = _manual_worker()
    worker.record = lambda row: (_ for _ in ()).throw(OSError("disk full"))
    try:
        worker.submit_target([0.2] * 6, clock())
        assert worker.run_cycle()
        assert not worker.failed
        assert backend.position_writes
        diagnostics = worker.diagnostics_snapshot()
        assert diagnostics["record_callback_failure_count"] == 1
        assert diagnostics["last_record_callback_failure"]["exception_type"] == "OSError"
    finally:
        worker.cleanup()


def test_worker_exception_is_persisted_with_traceback_and_context() -> None:
    class FailsAfterStartup(FakeRH56PcDirectBackend):
        def __init__(self) -> None:
            super().__init__()
            self.angle_reads = 0

        def get_canonical_angles(self) -> list[float]:
            self.angle_reads += 1
            if self.angle_reads > 1:
                raise RuntimeError("injected feedback failure")
            return super().get_canonical_angles()

    records: list[dict[str, object]] = []
    backend = FailsAfterStartup()
    control = RH56PcDirectControl(backend, _config())
    worker = RH56PcDirectWorker(control, record=records.append)
    worker.start(RH56_COMBINED_RUN_APPROVAL)
    deadline = time.monotonic() + 0.5
    while not worker.failed and time.monotonic() < deadline:
        time.sleep(0.005)
    try:
        assert worker.failed
        failure = worker.failure_record
        assert failure is not None
        assert failure["record_type"] == "rh56_worker_failure"
        assert failure["exception_type"] == "RuntimeError"
        assert "injected feedback failure" in failure["message"]
        assert "Traceback" in failure["traceback"]
        assert failure["control_failure"]["stage"] == "feedback_poll"
        assert records[-1]["record_type"] == "rh56_worker_failure"
    finally:
        worker.cleanup()


class FlushCountingStream(io.StringIO):
    def __init__(self) -> None:
        super().__init__()
        self.flush_calls = 0

    def flush(self) -> None:
        self.flush_calls += 1
        super().flush()


def test_jsonl_recorder_batches_normal_rows_and_flushes_faults_and_shutdown() -> None:
    stream = FlushCountingStream()
    recorder = BoundedJsonlRecorder(
        stream,
        capacity=8,
        flush_every_records=4,
        flush_interval_sec=60.0,
    )
    for index in range(3):
        recorder({"record_type": "rh56_telemetry", "index": index})
    assert stream.flush_calls == 0
    recorder({"record_type": "rh56_telemetry", "index": 3})
    assert stream.flush_calls == 1
    recorder({"record_type": "rh56_worker_failure", "message": "fault"})
    assert stream.flush_calls == 2
    recorder({"record_type": "rh56_telemetry", "index": 4})
    assert recorder.close()
    assert stream.flush_calls == 3
    assert recorder.buffered_record_count == 0


def test_jsonl_write_and_flush_failures_are_independent_and_buffer_is_bounded() -> None:
    class WriteFailureStream(io.StringIO):
        def write(self, value: str) -> int:
            raise OSError("write failed")

    recorder = BoundedJsonlRecorder(
        WriteFailureStream(),
        capacity=3,
        flush_every_records=2,
        flush_interval_sec=60.0,
    )
    for index in range(8):
        recorder({"record_type": "rh56_telemetry", "index": index})
    assert recorder.buffered_record_count <= 3
    assert recorder.dropped_record_count > 0
    assert recorder.failures[-1].operation == "jsonl_write"

    class FlushFailureStream(io.StringIO):
        def flush(self) -> None:
            raise OSError("flush failed")

    flush_recorder = BoundedJsonlRecorder(
        FlushFailureStream(),
        capacity=3,
        flush_every_records=2,
        flush_interval_sec=60.0,
    )
    flush_recorder({"record_type": "rh56_worker_failure", "message": "fault"})
    assert flush_recorder.failures[-1].operation == "jsonl_flush"
    assert flush_recorder.buffered_record_count == 0


def test_default_command_and_feedback_rates_are_unchanged() -> None:
    config = _config()
    control = RH56PcDirectControl(FakeRH56PcDirectBackend(), config)
    worker = RH56PcDirectWorker(control)
    assert control.control_frequency_hz == 15
    assert control.command_period_ns == round(1e9 / 15)
    assert worker.stale_command_drop_enabled is False
