#!/usr/bin/env python3
"""Explicit E1: hold one post-EDG measured J1..J6 through the resampler."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import tempfile

from teleoperation.runtime.arm_only import NativeWorkerProcess


def _strict_e1_evidence_is_valid(metrics: dict) -> bool:
    q_hold = metrics["post_edg_authoritative_q_hold_rad"]
    tracking = metrics["maximum_tracking_difference_rad_per_joint"]
    displacement = metrics[
        "maximum_measured_displacement_from_q_hold_rad_per_joint"
    ]
    adjacent = metrics["output_maximum_adjacent_delta_rad"]
    return (
        metrics["zero_motion_q_hold_initialized"]
        and metrics["zero_motion_command_count"] > 0
        and metrics["zero_motion_command_mismatch_count"] == 0
        and metrics["zero_motion_fixed_destination_rad"] == q_hold
        and metrics["zero_motion_first_command_rad"] == q_hold
        and metrics["zero_motion_last_command_rad"] == q_hold
        and adjacent == [0.0] * 6
        and metrics["final_resampler_endpoint_error_rad"] == [0.0] * 6
        and not metrics["resampler_active_segment"]
        and math.isclose(
            metrics["maximum_tracking_difference_rad"], max(tracking), abs_tol=1e-15
        )
        and math.isclose(
            metrics["maximum_observed_joint_delta_rad"],
            max(displacement),
            abs_tol=1e-15,
        )
        and math.isclose(
            metrics["output_maximum_adjacent_delta_rad_global"],
            max(adjacent),
            abs_tol=1e-15,
        )
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", type=Path, default=Path("build/jaka_servo_worker/jaka_servo_worker"))
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--edg-state-ip", default="192.168.71.19")
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--estop-accessible", action="store_true")
    parser.add_argument("--workspace-clear", action="store_true")
    parser.add_argument("--rh56-command-path-absent", action="store_true")
    parser.add_argument("--expected-tool-id", type=int, default=0)
    parser.add_argument("--expected-user-frame-id", type=int, default=0)
    parser.add_argument("--metrics", type=Path, required=True)
    return parser


def main() -> int:
    args = _parser().parse_args()
    if not (args.estop_accessible and args.workspace_clear and args.rh56_command_path_absent):
        raise SystemExit("E1 requires E-stop, clear-workspace, and no-RH56-command confirmations")
    if not 0.5 <= args.duration_sec <= 10.0:
        raise SystemExit("E1 duration must be in [0.5, 10] seconds")
    args.metrics.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="jaka_edg_e1_") as directory:
        temporary = Path(directory)
        target_socket = temporary / "target.sock"
        native = NativeWorkerProcess(
            args.worker,
            [
                "--mode", "joint-zero-motion", "--hardware",
                "--robot-ip", args.robot_ip,
                "--edg-state-ip", args.edg_state_ip,
                "--duration-s", str(args.duration_sec),
                "--target-socket", str(target_socket),
                "--metrics-file", str(args.metrics),
                "--expected-tool-id", str(args.expected_tool_id),
                "--expected-user-frame-id", str(args.expected_user_frame_id),
                "--maximum-output-joint-velocity-rad-s", str(math.pi),
                "--diagnostic-joint-acceleration-boundary-rad-s2", str(4.0 * math.pi),
            ],
        )
        native.start()
        try:
            assert native.process is not None
            return_code = native.process.wait(timeout=args.duration_sec + 15.0)
        finally:
            native.stop()
    metrics = json.loads(args.metrics.read_text(encoding="utf-8"))
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return 0 if (
        return_code == 0
        and metrics["error_code"] == 0
        and metrics["cleanup_error_code"] == 0
        and _strict_e1_evidence_is_valid(metrics)
    ) else 2


if __name__ == "__main__":
    raise SystemExit(main())
