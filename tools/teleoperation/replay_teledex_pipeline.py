#!/usr/bin/env python3
"""Replay a recorded pose stream through mapping/filtering/safety without JAKA."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from teleoperation.contracts import Pose3D
from teleoperation.input.replay import ReplayPoseInput
from teleoperation.processing.clutch import ClutchController
from teleoperation.processing.one_euro_se3 import OneEuroSE3Filter
from teleoperation.processing.pose_validator import PoseValidator
from teleoperation.processing.target_shaper import JerkLimitedPoseShaper
from teleoperation.runtime.teledex_arm import BoundedArmTeleoperationPipeline
from teleoperation.supervision import ArmSafetySupervisor, SafetyEnvelope
from teleoperation.teledex_config import load_bounded_teleop_config
from teleoperation.transforms.frame_mapping import RelativePoseMapper


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--config", type=Path, default=Path("configs/teleoperation/teledex_jaka_arm_bounded.yaml"))
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--maximum-wall-duration-s", type=float, default=30.0)
    args = parser.parse_args()
    config = load_bounded_teleop_config(args.config)
    robot_pose = Pose3D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    mapper = RelativePoseMapper(
        config.frames,
        translation_scale=config.translation_scale,
        rotation_scale=config.rotation_scale,
    )
    pipeline = BoundedArmTeleoperationPipeline(
        validator=PoseValidator(config.validation),
        clutch=ClutchController(mapper, poses_are_operator_frame=True),
        measurement_filter=OneEuroSE3Filter(),
        shaper=JerkLimitedPoseShaper(config.cartesian_limits),
        safety=ArmSafetySupervisor(
            SafetyEnvelope(
                robot_pose,
                config.cartesian_limits.workspace_half_extent_m,
                config.cartesian_limits.maximum_orientation_deviation_rad,
                config.maximum_session_ns,
            ),
            config.joint_limits,
        ),
    )
    replay = ReplayPoseInput(args.recording, speed=args.speed)
    deadline = time.monotonic() + args.maximum_wall_duration_s
    generation = -1
    results = 0
    targets = 0
    try:
        while time.monotonic() < deadline:
            now = time.monotonic_ns()
            snapshot = replay.latest(now_ns=now, after_generation=generation)
            if snapshot is None:
                if replay.finished:
                    break
                time.sleep(0.001)
                continue
            generation = snapshot.generation
            result = pipeline.process(snapshot, robot_tcp_pose=robot_pose, now_ns=now)
            results += 1
            targets += result.target is not None
            print(
                json.dumps(
                    {
                        "generation": generation,
                        "validation": result.validation.action.value,
                        "clutch": None if result.clutch is None else result.clutch.state.value,
                        "safety": result.safety.action.value,
                        "reason": result.reason,
                        "target": None if result.target is None else result.target.to_dict(),
                    },
                    separators=(",", ":"),
                    sort_keys=True,
                )
            )
    except KeyboardInterrupt:
        pass
    finally:
        replay.close()
        pipeline.stop()
    print(json.dumps({"processed": results, "targets": targets, "skipped_backlog": replay.skipped_backlog}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
