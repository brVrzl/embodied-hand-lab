#!/usr/bin/env python3
"""Explicit E1: stream measured J1..J6 through the EDG resampler without motion."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import tempfile
import time

from teleoperation.runtime.arm_only import ArmOnlyRuntime, NativeWorkerProcess
from teleoperation.wire import (
    LatestTargetPublisher,
    FrameId,
    StatusFlags,
    TargetFlags,
    TargetKind,
    TargetPacket,
    WorkerStatusReceiver,
)


E1_APPROVAL = "I_AUTHORIZE_E1_ZERO_MOTION_EDG_RESAMPLER"
NATIVE_MOTION_ACK = "I_AUTHORIZE_P4_LIVE_QUEST_JAKA_TELEOPERATION"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", type=Path, default=Path("build/jaka_servo_worker/jaka_servo_worker"))
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--edg-state-ip", default="192.168.71.19")
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--approval", required=True)
    parser.add_argument("--estop-accessible", action="store_true")
    parser.add_argument("--workspace-clear", action="store_true")
    parser.add_argument("--rh56-command-path-absent", action="store_true")
    parser.add_argument("--expected-tool-id", type=int, default=0)
    parser.add_argument("--expected-user-frame-id", type=int, default=0)
    parser.add_argument("--metrics", type=Path, required=True)
    return parser


def _wait_connected(runtime: ArmOnlyRuntime, native: NativeWorkerProcess):
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        status = runtime.latest_status()
        if status is not None and StatusFlags(status.flags) & StatusFlags.CONNECTED:
            return status
        if native.process is not None and native.process.poll() is not None:
            raise RuntimeError("native worker exited before E1 measured-state initialization")
        time.sleep(0.005)
    raise RuntimeError("native worker did not report a connected measured state")


def main() -> int:
    args = _parser().parse_args()
    if args.approval != E1_APPROVAL:
        raise SystemExit(f"exact approval required: {E1_APPROVAL}")
    if not (args.estop_accessible and args.workspace_clear and args.rh56_command_path_absent):
        raise SystemExit("E1 requires E-stop, clear-workspace, and no-RH56-command confirmations")
    if not 0.5 <= args.duration_sec <= 10.0:
        raise SystemExit("E1 duration must be in [0.5, 10] seconds")
    args.metrics.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="jaka_edg_e1_") as directory:
        temporary = Path(directory)
        target_socket = temporary / "target.sock"
        status_socket = temporary / "status.sock"
        runtime = ArmOnlyRuntime(
            LatestTargetPublisher(target_socket), WorkerStatusReceiver(status_socket)
        )
        native = NativeWorkerProcess(
            args.worker,
            [
                "--mode", "joint-teleop", "--hardware",
                "--robot-ip", args.robot_ip,
                "--edg-state-ip", args.edg_state_ip,
                "--duration-s", str(args.duration_sec + 2.0),
                "--target-socket", str(target_socket),
                "--status-socket", str(status_socket),
                "--metrics-file", str(args.metrics),
                "--expected-tool-id", str(args.expected_tool_id),
                "--expected-user-frame-id", str(args.expected_user_frame_id),
                "--acknowledgement", NATIVE_MOTION_ACK,
                "--maximum-output-joint-velocity-rad-s", str(math.pi),
                "--diagnostic-joint-acceleration-boundary-rad-s2", str(4.0 * math.pi),
            ],
        )
        native.start()
        try:
            status = _wait_connected(runtime, native)
            measured = tuple(float(value) for value in status.joint_position_rad)
            if len(measured) != 6 or not all(math.isfinite(value) for value in measured):
                raise RuntimeError("E1 measured joints are invalid")
            started = time.monotonic()
            sequence = 0
            while time.monotonic() - started < args.duration_sec:
                sequence += 1
                generated_ns = time.monotonic_ns()
                packet = TargetPacket(
                    TargetKind.JOINT_POSITION,
                    TargetFlags.ALLOW_MOTION,
                    FrameId.NONE,
                    sequence,
                    0,
                    generated_ns,
                    generated_ns,
                    generated_ns,
                    (*measured, 0.0, 0.0),
                )
                if not runtime.dispatch_packet(packet):
                    raise RuntimeError("E1 zero-motion target transport failed")
                if native.process is None or native.process.poll() is not None:
                    raise RuntimeError("native worker exited during E1")
                deadline = started + sequence / 60.0
                time.sleep(max(0.0, deadline - time.monotonic()))
            runtime.dispatch_stop(sequence=sequence + 1)
            time.sleep(0.05)
        finally:
            native.stop()
            runtime.close()
    metrics = json.loads(args.metrics.read_text(encoding="utf-8"))
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return 0 if metrics["error_code"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
