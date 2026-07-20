#!/usr/bin/env python3
"""Stage T2 TeleDex receive/record diagnostic. Never imports or connects to JAKA."""

from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

from teleoperation.input.replay import PoseStreamRecorder
from teleoperation.input.teledex import TeleDexAdapter, TeleDexPacketParser, TeleDexWebSocketServer
from teleoperation.teledex_config import load_bounded_teleop_config


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = fraction * (len(ordered) - 1)
    lower = int(index)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (index - lower) * (ordered[upper] - ordered[lower])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/teleoperation/teledex_jaka_arm_bounded.yaml"))
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8888)
    parser.add_argument("--duration-s", type=float, default=15.0)
    parser.add_argument("--record", type=Path)
    parser.add_argument("--summary", type=Path)
    args = parser.parse_args()
    if not 0.1 <= args.duration_s <= 600.0:
        parser.error("duration must be in [0.1, 600] seconds")

    config = load_bounded_teleop_config(args.config)
    adapter = TeleDexAdapter(
        parser=TeleDexPacketParser(),
        stale_after_ns=config.validation.hold_age_ns,
        source_frame_id=config.frames.source_frame_id,
    )
    recorder = (
        None
        if args.record is None
        else PoseStreamRecorder(
            args.record,
            metadata={
                "mode": "teledex_receive_only",
                "config_sha256": config.content_sha256,
                "source_frame_semantics_confirmed": config.source_semantics_confirmed,
            },
        )
    )
    intervals_ms: list[float] = []
    ages_ms: list[float] = []
    last_receive_ns: int | None = None
    last_generation = -1
    valid = 0
    invalid = 0
    connected_seen = False
    started = time.monotonic_ns()
    deadline = started + int(args.duration_s * 1e9)
    print(f"TeleDex receive-only WebSocket endpoint: {args.host}:{args.port}")
    print("No JAKA SDK, EDG, RH56, HEBI, ROS, or robot command path is loaded.")
    print("Source axes/handedness/pose direction remain unconfirmed; observe and record all six directions.")
    try:
        with TeleDexWebSocketServer(adapter, host=args.host, port=args.port):
            while time.monotonic_ns() < deadline:
                now = time.monotonic_ns()
                snapshot = adapter.latest(now_ns=now, after_generation=last_generation)
                if snapshot is None:
                    time.sleep(0.002)
                    continue
                last_generation = snapshot.generation
                connected_seen = connected_seen or snapshot.connected
                if recorder is not None:
                    recorder.write(snapshot)
                if snapshot.pose is None or not snapshot.pose.tracking_valid:
                    invalid += 1
                    print(f"generation={snapshot.generation} valid=false reason={snapshot.reason}")
                    continue
                valid += 1
                receive_ns = snapshot.pose.timestamps.local_receive_ns
                if last_receive_ns is not None:
                    intervals_ms.append((receive_ns - last_receive_ns) / 1e6)
                last_receive_ns = receive_ns
                ages_ms.append(snapshot.pose.sample_age_ns / 1e6)
                pose = snapshot.pose.pose
                print(
                    f"seq={snapshot.pose.sequence} epoch={snapshot.pose.connection_epoch} "
                    f"age_ms={snapshot.pose.sample_age_ns / 1e6:.3f} "
                    f"p_m=({pose.position_m[0]:+.4f},{pose.position_m[1]:+.4f},{pose.position_m[2]:+.4f}) "
                    f"q_xyzw=({pose.quaternion_xyzw[0]:+.4f},{pose.quaternion_xyzw[1]:+.4f},"
                    f"{pose.quaternion_xyzw[2]:+.4f},{pose.quaternion_xyzw[3]:+.4f}) "
                    f"clutch={snapshot.run_gate.engaged} "
                    f"recenter={bool(snapshot.operator_action and snapshot.operator_action.recenter_requested)} "
                    f"discontinuity={snapshot.pose.discontinuity.value}"
                )
    except KeyboardInterrupt:
        pass
    finally:
        if recorder is not None:
            recorder.close()

    elapsed_s = max(1e-9, (time.monotonic_ns() - started) / 1e9)
    median_interval = statistics.median(intervals_ms) if intervals_ms else None
    long_gaps = (
        0
        if median_interval is None
        else sum(value > max(50.0, median_interval * 2.5) for value in intervals_ms)
    )
    summary = {
        "schema_version": "teledex_input_diagnostic.v1",
        "mode": "receive_only_no_robot",
        "config_sha256": config.content_sha256,
        "elapsed_s": elapsed_s,
        "connected_seen": connected_seen,
        "valid_samples": valid,
        "invalid_events": invalid,
        "adapter_received_packets": adapter.received_packets,
        "adapter_invalid_packets": adapter.invalid_packets,
        "observed_rate_hz": valid / elapsed_s,
        "interarrival_ms": {
            "median": median_interval,
            "p95": percentile(intervals_ms, 0.95),
            "p99": percentile(intervals_ms, 0.99),
            "maximum": max(intervals_ms) if intervals_ms else None,
            "long_gap_count": long_gaps,
        },
        "processing_age_ms": {
            "median": statistics.median(ages_ms) if ages_ms else None,
            "p95": percentile(ages_ms, 0.95),
            "p99": percentile(ages_ms, 0.99),
            "maximum": max(ages_ms) if ages_ms else None,
        },
        "source_timestamp_available": False,
        "source_sequence_available": False,
        "tracking_quality_available": False,
        "source_frame_semantics_confirmed": config.source_semantics_confirmed,
        "robot_connection_opened": False,
        "commands_issued": 0,
    }
    encoded = json.dumps(summary, indent=2, sort_keys=True)
    print(encoded)
    if args.summary is not None:
        args.summary.write_text(encoded + "\n", encoding="utf-8")
    return 0 if valid > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
