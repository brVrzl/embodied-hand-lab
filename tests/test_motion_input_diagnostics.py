from __future__ import annotations

from dataclasses import replace

from motion_input.diagnostics import StreamingDiagnostics
from motion_input.model import Timestamp, TrackingState
from motion_input.visualization import MotionInputVisualizer

from test_motion_input_protocol import make_sample


def test_diagnostics_measure_frequency_drops_latency_and_interruptions() -> None:
    wall = [0]
    cpu = [0]
    diagnostics = StreamingDiagnostics(
        monotonic_ns=lambda: wall[0],
        process_time_ns=lambda: cpu[0],
    )
    samples = []
    for sequence, state in ((0, TrackingState.TRACKING), (1, TrackingState.NOT_TRACKING), (3, TrackingState.TRACKING)):
        sample = make_sample(sequence, state=state)
        samples.append(
            replace(
                sample,
                capture_timestamp=Timestamp(sequence * 10_000_000, "host:monotonic"),
                receive_timestamp=Timestamp(sequence * 10_000_000 + 2_000_000, "host:monotonic"),
                processing_timestamp=Timestamp(sequence * 10_000_000 + 3_000_000, "host:monotonic"),
            )
        )
    for sample in samples:
        diagnostics.observe(sample)
    wall[0] = 100_000_000
    cpu[0] = 10_000_000
    report = diagnostics.report()
    stream = report["streams"]["stream-left:left"]
    assert stream["frame_drops"] == 1
    assert stream["tracking_interruptions"] == 1
    assert stream["tracking_recoveries"] == 1
    assert stream["latency_ms"]["mean"] == 2.0
    assert stream["processing_latency_ms"]["mean"] == 1.0
    assert report["process_cpu_percent"] == 10.0


def test_noncomparable_clocks_are_reported_not_subtracted() -> None:
    diagnostics = StreamingDiagnostics()
    sample = replace(
        make_sample(0),
        capture_timestamp=Timestamp(100, "quest"),
        receive_timestamp=Timestamp(200, "host"),
    )
    diagnostics.observe(sample)
    report = diagnostics.report()
    assert report["noncomparable_latency_samples"] == 1
    assert report["streams"]["stream-left:left"]["latency_ms"] is None


def test_out_of_order_sequence_and_timestamp_are_detected() -> None:
    diagnostics = StreamingDiagnostics()
    diagnostics.observe(make_sample(2))
    diagnostics.observe(make_sample(1))
    stream = diagnostics.report()["streams"]["stream-left:left"]
    assert stream["out_of_order_sequences"] == 1
    assert stream["out_of_order_timestamps"] == 2


def test_long_duration_streaming_has_constant_event_semantics() -> None:
    diagnostics = StreamingDiagnostics()
    for sequence in range(20_000):
        diagnostics.observe(make_sample(sequence))
    stream = diagnostics.report()["streams"]["stream-left:left"]
    assert stream["samples"] == 20_000
    assert stream["frame_drops"] == 0
    assert stream["out_of_order_sequences"] == 0
    assert stream["out_of_order_timestamps"] == 0


def test_visualizer_displays_frames_status_confidence_and_interruption() -> None:
    visualizer = MotionInputVisualizer()
    visualizer.observe(make_sample(0))
    visualizer.observe(make_sample(1, state=TrackingState.NOT_TRACKING))
    text = visualizer.render_text()
    assert "left status=not_tracking" in text
    assert "wrist=n/a" in text
    assert "tracking -> not_tracking" in text
    assert "tracking origin" in text
