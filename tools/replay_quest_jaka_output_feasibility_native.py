#!/usr/bin/env python3
"""Feed corrected offline AcceptedArmTargets through the fake native worker."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import time

from teleoperation.wire import (
    LatestTargetPublisher,
    joint_position_target_packet,
    stop_target_packet,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--worker", type=Path, required=True)
    parser.add_argument("--metrics", type=Path, required=True)
    parser.add_argument("--emitted", type=Path, required=True)
    args = parser.parse_args()
    replay = json.loads(args.replay.read_text(encoding="utf-8"))
    physical_replay = replay["after_physical_start"]
    stream = physical_replay["accepted_targets"]
    initial = replay["after_physical_start"]["initial_joint_position_rad"]
    velocity_boundary = float(physical_replay["output_velocity_boundary_rad_s"])
    if not stream:
        raise SystemExit("replay has no accepted targets")
    args.metrics.parent.mkdir(parents=True, exist_ok=True)
    args.emitted.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="quest_jaka_output_replay_") as directory:
        socket_path = Path(directory) / "target.sock"
        command = [
            str(args.worker),
            "--mode", "joint-teleop-dry-run",
            "--duration-s", "6.0",
            "--warning-ms", "40",
            "--hold-ms", "100",
            "--controlled-stop-ms", "500",
            "--fatal-timeout-ms", "2000",
            "--maximum-output-joint-velocity-rad-s", str(velocity_boundary),
            "--fake-initial-joints-rad", ",".join(str(value) for value in initial),
            "--target-socket", str(socket_path),
            "--metrics-file", str(args.metrics),
            "--emitted-points-file", str(args.emitted),
        ]
        process = subprocess.Popen(command)
        deadline = time.monotonic() + 2.0
        while not socket_path.exists() and time.monotonic() < deadline:
            time.sleep(0.002)
        if not socket_path.exists():
            process.terminate()
            raise SystemExit("native worker target socket did not appear")
        # The socket is bound before the fake backend completes its post-EDG
        # q_hold handoff. Give that bounded setup time to finish so the first
        # external AcceptedArmTarget timestamp is newer than the internal hold.
        time.sleep(0.05)
        first_generated_ns = int(stream[0]["generated_monotonic_ns"])
        started = time.monotonic_ns()
        # Preserve every recorded interval in the current CLOCK_MONOTONIC epoch.
        # Native startup establishes an internal hold with a current timestamp,
        # so replaying an older host epoch verbatim would correctly look stale.
        processing_epoch_ns = started
        with LatestTargetPublisher(socket_path) as publisher:
            for sequence, target in enumerate(stream, start=1):
                generated_ns = int(target["generated_monotonic_ns"])
                target_deadline = started + generated_ns - first_generated_ns
                while time.monotonic_ns() < target_deadline:
                    time.sleep(0.0005)
                dispatch_ns = time.monotonic_ns()
                replay_processing_ns = (
                    processing_epoch_ns + generated_ns - first_generated_ns
                )
                if not publisher.publish(
                    joint_position_target_packet(
                        sequence=sequence,
                        joint_position_rad=tuple(target["joint_position_rad"]),
                        local_receive_ns=replay_processing_ns,
                        processing_ns=replay_processing_ns,
                        dispatch_ns=dispatch_ns,
                        allow_motion=True,
                    )
                ):
                    process.terminate()
                    raise SystemExit(f"failed to publish replay sequence {sequence}")
            now_ns = time.monotonic_ns()
            if not publisher.publish(
                stop_target_packet(sequence=len(stream) + 1, monotonic_ns=now_ns)
            ):
                process.terminate()
                raise SystemExit("failed to publish replay stop")
        return_code = process.wait(timeout=3.0)
    metrics = json.loads(args.metrics.read_text(encoding="utf-8"))
    print(json.dumps({
        "process_exit_code": return_code,
        "outcome": metrics["outcome"],
        "accepted_targets": metrics["accepted_targets"],
        "producer_heartbeat_packets": metrics["producer_heartbeat_packets"],
        "output_speed_boundary_rejections": metrics["output_speed_boundary_rejections"],
        "output_maximum_velocity_rad_s": metrics["output_maximum_velocity_rad_s"],
        "ik_calls": metrics["ik_calls"],
        "error_code": metrics["error_code"],
        "cleanup_error_code": metrics["cleanup_error_code"],
    }, indent=2, sort_keys=True))
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
