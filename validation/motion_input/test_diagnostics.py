from __future__ import annotations

from dataclasses import replace

from motion_input.diagnostics import StreamingDiagnostics
from motion_input.model import Timestamp, TrackingState
from motion_input.visualization import MotionInputVisualizer

from tests.test_motion_input_protocol import make_sample


def test_diagnostics_report_stream_health_and_clock_safe_latency() -> None:
    wall = [0]
    cpu = [0]
    diagnostics = StreamingDiagnostics(
        monotonic_ns=lambda: wall[0],
        process_time_ns=lambda: cpu[0],
    )
    for sequence, state in (
        (0, TrackingState.TRACKING),
        (1, TrackingState.NOT_TRACKING),
        (3, TrackingState.TRACKING),
    ):
        sample = make_sample(sequence, state=state)
        diagnostics.observe(
            replace(
                sample,
                capture_timestamp=Timestamp(sequence * 10_000_000, "host:monotonic"),
                receive_timestamp=Timestamp(
                    sequence * 10_000_000 + 2_000_000,
                    "host:monotonic",
                ),
                processing_timestamp=Timestamp(
                    sequence * 10_000_000 + 3_000_000,
                    "host:monotonic",
                ),
            )
        )
    diagnostics.observe(
        replace(
            make_sample(4),
            capture_timestamp=Timestamp(40_000_000, "quest"),
            receive_timestamp=Timestamp(42_000_000, "host"),
        )
    )
    wall[0] = 100_000_000
    cpu[0] = 10_000_000

    report = diagnostics.report()
    stream = report["streams"]["stream-left:left"]
    assert stream["frame_drops"] == 1
    assert stream["tracking_interruptions"] == 1
    assert stream["tracking_recoveries"] == 1
    assert stream["latency_ms"]["mean"] == 2.0
    assert stream["processing_latency_ms"]["mean"] == 1.0
    assert report["noncomparable_latency_samples"] == 1
    assert report["process_cpu_percent"] == 10.0

    out_of_order = StreamingDiagnostics()
    out_of_order.observe(make_sample(2))
    out_of_order.observe(
        replace(
            make_sample(1),
            capture_timestamp=Timestamp(1_000_000, "host:monotonic"),
            receive_timestamp=Timestamp(1_005_000, "host:monotonic"),
        )
    )
    out_of_order_stream = out_of_order.report()["streams"]["stream-left:left"]
    assert out_of_order_stream["out_of_order_sequences"] == 1
    assert out_of_order_stream["out_of_order_timestamps"] == 2


def test_visualizer_exposes_tracking_interruption() -> None:
    visualizer = MotionInputVisualizer()
    visualizer.observe(make_sample(0))
    visualizer.observe(make_sample(1, state=TrackingState.NOT_TRACKING))

    text = visualizer.render_text()
    assert "left status=not_tracking" in text
    assert "wrist=n/a" in text
    assert "tracking -> not_tracking" in text
