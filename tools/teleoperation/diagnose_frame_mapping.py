#!/usr/bin/env python3
"""Read-only synthetic six-direction frame diagnostic."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from teleoperation.contracts import Pose3D
from teleoperation.teledex_config import load_bounded_teleop_config
from teleoperation.transforms.frame_mapping import RelativePoseMapper
from teleoperation.transforms.se3 import quaternion_exp, quaternion_log


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/teleoperation/teledex_jaka_arm_bounded.yaml"))
    parser.add_argument("--translation-input-m", type=float, default=0.10)
    parser.add_argument("--rotation-input-deg", type=float, default=10.0)
    args = parser.parse_args()
    config = load_bounded_teleop_config(args.config)
    identity = Pose3D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    results: list[dict[str, object]] = []
    for kind in ("translation", "rotation"):
        for axis in range(3):
            for sign in (-1.0, 1.0):
                mapper = RelativePoseMapper(
                    config.frames,
                    translation_scale=config.translation_scale,
                    rotation_scale=config.rotation_scale,
                )
                mapper.anchor(identity, identity)
                if kind == "translation":
                    position = np.zeros(3)
                    position[axis] = sign * args.translation_input_m
                    source = Pose3D(tuple(position), identity.quaternion_xyzw)
                else:
                    rotation = np.zeros(3)
                    rotation[axis] = sign * math.radians(args.rotation_input_deg)
                    source = Pose3D(identity.position_m, quaternion_exp(rotation))
                target = mapper.map(source)
                results.append(
                    {
                        "input": f"{kind}_{'-' if sign < 0 else '+'}{'xyz'[axis]}",
                        "target_translation_m": list(target.position_m),
                        "target_rotation_vector_rad": quaternion_log(target.quaternion_xyzw).tolist(),
                    }
                )
    print(
        json.dumps(
            {
                "schema_version": "frame_mapping_diagnostic.v1",
                "config_sha256": config.content_sha256,
                "confirmed_for_shadow": config.calibration_confirmed_for_shadow,
                "confirmed_for_motion": config.calibration_confirmed_for_motion,
                "source_semantics_confirmed": config.source_semantics_confirmed,
                "directions": results,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
