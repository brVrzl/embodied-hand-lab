#!/usr/bin/env python3
"""Offline P1/P2 kinematic comparison from a saved read-only worker report.

This tool never imports the JAKA SDK and never connects to a robot.  It compares
the SDK-observed joints/TCP already recorded by the native worker with FK from
the shared target model and with MuJoCo FK evaluated at those exact same joints.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from motion_input import Pose6D
from quest_jaka_sim import JakaMujocoSimulation, ReplayConfig, SharedJakaTargetGenerator
from quest_jaka_sim.se3 import quaternion_angle_rad


def _rpy_to_quaternion_xyzw(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _pose_dict(pose: Pose6D) -> dict[str, list[float]]:
    return {
        "position_m": list(pose.position_m),
        "orientation_xyzw": list(pose.orientation_xyzw),
    }


def compare(worker_metrics: dict[str, object], config_path: Path) -> dict[str, object]:
    joints = np.asarray(worker_metrics["initial_joint_position_rad"], dtype=np.float64)
    sdk = np.asarray(worker_metrics["startup_tcp_mm_rpy_rad"], dtype=np.float64)
    if joints.shape != (6,) or sdk.shape != (6,) or not np.all(np.isfinite(joints)) or not np.all(np.isfinite(sdk)):
        raise ValueError("worker report must contain finite six-element joint and TCP arrays")
    config = ReplayConfig.load(config_path)
    shared = SharedJakaTargetGenerator(config, mjcf_path=config.mjcf_path)
    shared.synchronize_authoritative_arm_joints(joints.tolist())
    shared_tcp = shared.current_tcp_pose
    simulation = JakaMujocoSimulation(config, mjcf_path=config.mjcf_path)
    simulation.synchronize_authoritative_arm_joints(joints.tolist())
    mujoco_tcp = simulation.current_tcp_pose
    sdk_tcp = Pose6D(
        tuple(float(value / 1000.0) for value in sdk[:3]),
        _rpy_to_quaternion_xyzw(*[float(value) for value in sdk[3:]]),
    )

    def errors(left: Pose6D, right: Pose6D) -> dict[str, float]:
        return {
            "position_error_mm": 1000.0 * float(
                np.linalg.norm(np.asarray(left.position_m) - np.asarray(right.position_m))
            ),
            "orientation_error_deg": math.degrees(
                quaternion_angle_rad(left.orientation_xyzw, right.orientation_xyzw)
            ),
        }

    return {
        "schema_version": "quest_jaka_model_parity.v1",
        "measured_physical_joint_position_rad": joints.tolist(),
        "jaka_sdk_tcp": _pose_dict(sdk_tcp),
        "mujoco_fk_at_measured_joints": _pose_dict(mujoco_tcp),
        "shared_model_tcp_at_measured_joints": _pose_dict(shared_tcp),
        "kinematic_model_parity": {
            "sdk_vs_shared": errors(sdk_tcp, shared_tcp),
            "sdk_vs_mujoco_fk": errors(sdk_tcp, mujoco_tcp),
            "shared_vs_mujoco_fk": errors(shared_tcp, mujoco_tcp),
        },
        "target_parity": "not evaluated by this report; use shared accepted-target logs/tests",
        "dynamic_tracking_parity": "not evaluated by a static P1/P2 FK comparison",
        "physical_commands_sent": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker-metrics", type=Path, required=True)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/sim/quest_hts_jaka_mini2_live_demo.yaml"),
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = compare(json.loads(args.worker_metrics.read_text(encoding="utf-8")), args.config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
