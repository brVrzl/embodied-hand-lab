#!/usr/bin/env python3
"""Explicit launcher for connected JAKA probes; never defaults to hardware."""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path


ACK = "I_ACKNOWLEDGE_JAKA_HARDWARE_RISK"


def main() -> int:
    parser = argparse.ArgumentParser(epilog="Confirm E-stop access, clear workspace, and use reduced robot limits.")
    parser.add_argument("mode", choices=("state-read", "zero-motion", "minimal-motion"))
    parser.add_argument("--worker", type=Path, default=Path("build/jaka_servo_worker/jaka_servo_worker"))
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--edg-state-ip", default="0.0.0.0")
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--expected-tool-id", type=int, required=True)
    parser.add_argument("--expected-user-frame-id", type=int, required=True)
    parser.add_argument("--acknowledgement", required=True, help=f"must equal: {ACK}")
    parser.add_argument("--metrics-file", type=Path, required=True)
    parser.add_argument("--probe-joint", type=int, default=0)
    parser.add_argument("--probe-delta-rad", type=float, default=0.001)
    parser.add_argument("--workspace-min-mm", help="required minimal-mode bound: x,y,z")
    parser.add_argument("--workspace-max-mm", help="required minimal-mode bound: x,y,z")
    args = parser.parse_args()
    if args.acknowledgement != ACK:
        parser.error("exact hardware acknowledgement was not supplied")
    if args.mode == "minimal-motion":
        if not args.workspace_min_mm or not args.workspace_max_mm:
            parser.error("minimal-motion requires explicit Cartesian workspace bounds")
        answer = input("E-stop ready, workspace clear, safe joint/direction reviewed? Type EXECUTE_MINIMAL_MOTION: ")
        if answer != "EXECUTE_MINIMAL_MOTION":
            parser.error("operator confirmation failed")
    command = [str(args.worker), "--mode", args.mode, "--hardware", "--robot-ip", args.robot_ip,
               "--edg-state-ip", args.edg_state_ip, "--duration-s", str(args.duration_s),
               "--expected-tool-id", str(args.expected_tool_id), "--expected-user-frame-id",
               str(args.expected_user_frame_id), "--acknowledgement", args.acknowledgement,
               "--metrics-file", str(args.metrics_file)]
    if args.mode == "minimal-motion":
        command += ["--probe-joint", str(args.probe_joint), "--probe-delta-rad", str(args.probe_delta_rad)]
        command += ["--workspace-min-mm", args.workspace_min_mm, "--workspace-max-mm", args.workspace_max_mm]
    return subprocess.run(command, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
