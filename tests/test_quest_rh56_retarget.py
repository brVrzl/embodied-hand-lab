from __future__ import annotations

from pathlib import Path

import pytest

from motion_input import Side
from motion_input.hts_protocol import HTS_JOINT_NAMES
from quest_jaka_sim.hand_retarget import (
    HandRetargetCalibration,
    ProjectRh56Retargeter,
    QuestHandSkeleton,
    RH56_FULL_JOINT_ORDER,
    RH56_MUJOCO_ACTUATOR_ORDER,
)


def _open_points() -> list[tuple[float, float, float]]:
    points = [(0.0, 0.0, 0.0)] * 21
    points[0] = (0.0, 0.0, 0.0)
    points[1:5] = [(-0.02, 0.01, 0.0), (-0.03, 0.025, 0.0), (-0.04, 0.04, 0.0), (-0.05, 0.055, 0.0)]
    for x, indices in zip((-0.025, -0.008, 0.010, 0.027), ((5, 6, 7, 8), (9, 10, 11, 12), (13, 14, 15, 16), (17, 18, 19, 20)), strict=True):
        for depth, index in enumerate(indices, start=1):
            points[index] = (x, depth * 0.025, 0.0)
    return points


def _fist_points() -> list[tuple[float, float, float]]:
    points = _open_points()
    for x, indices in zip((-0.025, -0.008, 0.010, 0.027), ((5, 6, 7, 8), (9, 10, 11, 12), (13, 14, 15, 16), (17, 18, 19, 20)), strict=True):
        coordinates = [(x, 0.025, 0.0), (x, 0.050, 0.0), (x + 0.020, 0.050, 0.0), (x + 0.020, 0.025, 0.0)]
        for index, value in zip(indices, coordinates, strict=True):
            points[index] = value
    points[1:5] = [(-0.02, 0.01, 0.0), (-0.03, 0.025, 0.0), (-0.01, 0.03, 0.0), (-0.005, 0.05, 0.0)]
    return points


def _skeleton(points: list[tuple[float, float, float]], *, valid: bool = True) -> QuestHandSkeleton:
    return QuestHandSkeleton(1, Side.RIGHT, "right_wrist", HTS_JOINT_NAMES, tuple(points), valid, 1.0)


@pytest.mark.parametrize("backend", ["adaptive", "vector"])
def test_backends_are_deterministic_respect_order_and_limits(backend: str) -> None:
    _, calibration = HandRetargetCalibration.load("configs/sim/quest_rh56_retarget.yaml")
    first = ProjectRh56Retargeter(calibration, backend=backend).retarget(_skeleton(_open_points()))
    second = ProjectRh56Retargeter(calibration, backend=backend).retarget(_skeleton(_open_points()))
    assert first == second
    assert first.valid
    assert tuple(first.actuator_targets) == RH56_MUJOCO_ACTUATOR_ORDER
    assert tuple(first.joint_targets) == RH56_FULL_JOINT_ORDER
    assert all(value >= 0.0 for value in first.actuator_targets.values())


def test_adaptive_backend_open_fist_and_mimic_semantics() -> None:
    _, calibration = HandRetargetCalibration.load("configs/sim/quest_rh56_retarget.yaml")
    retargeter = ProjectRh56Retargeter(calibration, backend="adaptive")
    opened = retargeter.retarget(_skeleton(_open_points()))
    retargeter.reset()
    fist = retargeter.retarget(_skeleton(_fist_points()))
    assert sum(fist.actuator_targets[name] for name in ("index", "middle", "ring", "pinky")) > sum(
        opened.actuator_targets[name] for name in ("index", "middle", "ring", "pinky")
    )
    assert fist.joint_targets["rh56_R_thumb_PIP_joint"] == pytest.approx(
        0.6 * fist.actuator_targets["thumb_close"]
    )
    assert fist.joint_targets["rh56_R_index_DIP_joint"] == pytest.approx(
        fist.actuator_targets["index"]
    )


def test_tracking_loss_is_invalid_and_resets_warm_start() -> None:
    _, calibration = HandRetargetCalibration.load("configs/sim/quest_rh56_retarget.yaml")
    retargeter = ProjectRh56Retargeter(calibration, backend="adaptive")
    assert retargeter.retarget(_skeleton(_open_points())).valid
    loss = retargeter.retarget(_skeleton([], valid=False))
    assert not loss.valid
    assert loss.rejection_reason == "INVALID_RIGHT_HAND_SKELETON"
    assert retargeter._previous is None


def test_malformed_calibration_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text(
        "selected_backend: adaptive\ncalibration:\n"
        "  calibration_id: bad\n  global_scale: 1\n  palm_scale: 1\n"
        "  finger_scale: [1, 1]\n  thumb_scale: 1\n  key_vector_scale: 1\n"
        "  pinch_weight: 0.5\n  maximum_normalized_step: 0.1\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="malformed"):
        HandRetargetCalibration.load(path)
