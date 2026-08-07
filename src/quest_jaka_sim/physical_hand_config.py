"""Shared physical Quest-to-RH56 retarget configuration assembly."""

from __future__ import annotations

import copy
from dataclasses import replace
from pathlib import Path

from .hand_retarget import HandRetargetCalibration
from .simulation import ReplayConfig


DEFAULT_PHYSICAL_RH56_CALIBRATION = Path(
    "configs/hand/quest_rh56_real_retarget.yaml"
)


def with_physical_rh56_retarget(
    config: ReplayConfig,
    calibration_path: str | Path = DEFAULT_PHYSICAL_RH56_CALIBRATION,
) -> ReplayConfig:
    """Apply the maintained real-hand mapping to every physical RH56 entry."""

    path = Path(calibration_path)
    if not path.is_file():
        raise FileNotFoundError(f"physical RH56 calibration does not exist: {path}")
    calibration = HandRetargetCalibration.load(path)
    if not calibration.calibration_id.startswith("quest_rh56dfx_real_"):
        raise ValueError(
            "physical RH56 entry requires a quest_rh56dfx_real_* calibration"
        )
    raw = copy.deepcopy(config.raw)
    hand_values = raw.setdefault("hand_retargeting", {})
    hand_values["enabled"] = True
    hand_values["calibration_path"] = str(path)
    hand_values["align_on_grip"] = True
    hand_values["align_index_pinch_to_validated_pose"] = True
    return replace(config, raw=raw)
