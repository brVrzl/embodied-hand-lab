#!/usr/bin/env python3
"""Exercise Python-to-native communication using synthetic 6-DoF poses only."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import tempfile
import time
from pathlib import Path

from teleoperation.runtime.synthetic import FaultSchedule, SyntheticPattern, SyntheticPoseSource
from teleoperation.wire import LatestTargetPublisher, WorkerStatusReceiver, pose_target_packet


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--worker", type=Path, default=Path("build/jaka_servo_worker/jaka_servo_worker"))
    parser.add_argument("--pattern", choices=[item.value for item in SyntheticPattern], default="fixed")
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--rate-hz", type=float, default=60.0)
    parser.add_argument("--dropout-start-s", type=float)
    parser.add_argument("--dropout-duration-s", type=float, default=0.0)
    parser.add_argument("--timestamp-jitter-ms", type=float, default=0.0)
    parser.add_argument("--duplicate-every", type=int, default=0)
    parser.add_argument("--reorder-every", type=int, default=0)
    parser.add_argument("--burst-every", type=int, default=0)
    parser.add_argument("--slowdown-after-s", type=float)
    parser.add_argument("--slowdown-factor", type=float, default=1.0)
    args = parser.parse_args()
    if args.duration_s <= 0 or args.rate_hz <= 0:
        parser.error("duration and rate must be positive")

    faults = FaultSchedule(args.dropout_start_s, args.dropout_duration_s,
                           int(args.timestamp_jitter_ms * 1e6), args.duplicate_every,
                           args.reorder_every, args.burst_every, 3,
                           args.slowdown_after_s, args.slowdown_factor)
    source = SyntheticPoseSource(SyntheticPattern(args.pattern), faults=faults)
    with tempfile.TemporaryDirectory(prefix="jaka-foundation-") as directory:
        target_path = Path(directory) / "target.sock"
        status_path = Path(directory) / "status.sock"
        metrics_path = Path(directory) / "metrics.json"
        with WorkerStatusReceiver(status_path) as statuses:
            process = subprocess.Popen([
                str(args.worker), "--mode", "dry-run", "--duration-s", str(args.duration_s + 0.25),
                "--target-socket", str(target_path), "--status-socket", str(status_path),
                "--metrics-file", str(metrics_path),
            ])
            try:
                deadline = time.monotonic() + 2.0
                while not target_path.exists() and time.monotonic() < deadline:
                    time.sleep(0.005)
                if not target_path.exists():
                    raise RuntimeError("native worker did not create its target socket")
                start = time.monotonic()
                with LatestTargetPublisher(target_path) as publisher:
                    while (elapsed := time.monotonic() - start) < args.duration_s:
                        cycle_start = time.monotonic()
                        if not source.should_drop(elapsed):
                            for target in source.samples(elapsed):
                                dispatched = target.timestamps.with_stage(dispatch_ns=time.monotonic_ns())
                                packet = pose_target_packet(type(target)(
                                    target.source_id, target.sequence, target.target_frame_id,
                                    target.pose, dispatched, target.linear_velocity_m_s,
                                    target.angular_velocity_rad_s), allow_motion=False)
                                publisher.publish(packet)
                        delay = (1.0 / args.rate_hz) * source.period_scale(elapsed)
                        time.sleep(max(0.0, delay - (time.monotonic() - cycle_start)))
                process.wait(timeout=5.0)
                latest = statuses.latest()
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=5.0)
        metrics = json.loads(metrics_path.read_text())
        metrics["python_source"] = {
            "pattern": args.pattern,
            "requested_rate_hz": args.rate_hz,
            "last_worker_status_sequence": None if latest is None else latest.last_sequence,
        }
        print(json.dumps(metrics, indent=2, sort_keys=True))
        return process.returncode


if __name__ == "__main__":
    raise SystemExit(main())
