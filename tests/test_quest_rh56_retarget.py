from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from motion_input import Side
from motion_input.hts_protocol import HTS_JOINT_NAMES
from quest_jaka_sim.hand_retarget import (
    HandRetargetCalibration,
    PinchIntentDetector,
    PinchPoseBlender,
    ThumbFirstPinchSequencer,
    ProjectRh56Retargeter,
    QuestHandSkeleton,
    RH56_FULL_JOINT_ORDER,
    RH56_MUJOCO_ACTUATOR_ORDER,
    RH56_THUMB_CLOSE_RANGE_RAD,
    calibrate_finger_feature,
    right_hand_palm_local_frame,
    thumb_close_coupled_joint_positions,
    thumb_close_bend_primary_feature,
    thumb_lateral_opposition_feature,
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


def _mcp_only_flexion_points() -> list[tuple[float, float, float]]:
    points = _open_points()
    for x, indices in zip(
        (-0.025, -0.008, 0.010, 0.027),
        ((5, 6, 7, 8), (9, 10, 11, 12), (13, 14, 15, 16), (17, 18, 19, 20)),
        strict=True,
    ):
        for offset, index in enumerate(indices):
            points[index] = (x + offset * 0.025, 0.025, 0.0)
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
    expected_pip, expected_dip = thumb_close_coupled_joint_positions(
        fist.actuator_targets["thumb_close"]
    )
    assert fist.joint_targets["rh56_R_thumb_PIP_joint"] == pytest.approx(expected_pip)
    assert fist.joint_targets["rh56_R_thumb_DIP_joint"] == pytest.approx(expected_dip)
    assert fist.joint_targets["rh56_R_index_DIP_joint"] == pytest.approx(
        fist.actuator_targets["index"]
    )


def test_adaptive_backend_observes_mcp_only_flexion() -> None:
    _, calibration = HandRetargetCalibration.load("configs/sim/quest_rh56_retarget.yaml")
    opened = ProjectRh56Retargeter(calibration, backend="adaptive").retarget(
        _skeleton(_open_points())
    )
    flexed = ProjectRh56Retargeter(calibration, backend="adaptive").retarget(
        _skeleton(_mcp_only_flexion_points())
    )
    assert all(
        flexed.actuator_targets[name] > opened.actuator_targets[name]
        for name in ("index", "middle", "ring", "pinky")
    )


def test_real_finger_calibration_has_measured_direction_and_monotonic_mapping() -> None:
    _, calibration = HandRetargetCalibration.load(
        "configs/hand/quest_rh56_real_retarget.yaml"
    )
    assert calibration.calibration_id == "quest_rh56dfx_real_20260803_v2"
    for open_feature, closed_feature, exponent in zip(
        calibration.finger_feature_open,
        calibration.finger_feature_closed,
        calibration.finger_curve_exponent,
        strict=True,
    ):
        samples = [
            calibrate_finger_feature(
                value,
                open_feature=open_feature,
                closed_feature=closed_feature,
                curve_exponent=exponent,
            )
            for value in np.linspace(open_feature - 0.1, closed_feature + 0.1, 101)
        ]
        assert samples == sorted(samples)
        assert samples[0] == pytest.approx(0.0)
        assert samples[-1] == pytest.approx(1.0)


def test_finger_calibration_rejects_zero_span_and_nonfinite_features() -> None:
    with pytest.raises(ValueError, match="exceed"):
        calibrate_finger_feature(
            0.2,
            open_feature=0.2,
            closed_feature=0.2,
            curve_exponent=1.0,
        )
    with pytest.raises(ValueError, match="finite"):
        calibrate_finger_feature(
            float("nan"),
            open_feature=0.0,
            closed_feature=1.0,
            curve_exponent=1.0,
        )


def _pinch_detector() -> PinchIntentDetector:
    return PinchIntentDetector(
        enter_distance_palm=0.15,
        exit_distance_palm=0.22,
        tripod_enter_distance_palm=0.22,
        tripod_exit_distance_palm=0.30,
        minimum_finger_curl=0.12,
        power_grasp_curl=0.70,
    )


@pytest.mark.parametrize(
    ("distances", "curls", "expected"),
    (
        ((0.051, 0.258, 0.228), (0.446, 0.362, 0.328, 0.172), "index"),
        ((0.733, 0.102, 0.757), (0.106, 0.490, 0.503, 0.050), "middle"),
        ((0.06, 0.07, 0.08), (0.40, 0.45, 0.20, 0.15), "tripod"),
    ),
)
def test_labelled_pinch_geometries_select_independent_modes(
    distances: tuple[float, float, float],
    curls: tuple[float, float, float, float],
    expected: str,
) -> None:
    mode, confidence = _pinch_detector().update(
        thumb_index_distance_palm=distances[0],
        thumb_middle_distance_palm=distances[1],
        index_middle_distance_palm=distances[2],
        index_curl=curls[0],
        middle_curl=curls[1],
        ring_curl=curls[2],
        pinky_curl=curls[3],
    )
    assert mode == expected
    assert confidence > 0.0


def test_pinch_hysteresis_rejects_power_grasp_and_tracking_loss() -> None:
    detector = _pinch_detector()
    common = {
        "thumb_middle_distance_palm": 0.4,
        "index_middle_distance_palm": 0.2,
        "index_curl": 0.4,
        "middle_curl": 0.2,
        "ring_curl": 0.2,
        "pinky_curl": 0.2,
    }
    assert detector.update(thumb_index_distance_palm=0.10, **common)[0] == "index"
    assert detector.update(thumb_index_distance_palm=0.18, **common)[0] == "index"
    assert detector.update(thumb_index_distance_palm=0.23, **common)[0] == "none"
    assert detector.update(
        thumb_index_distance_palm=0.05,
        **{**common, "ring_curl": 0.8, "pinky_curl": 0.8},
    )[0] == "none"
    detector.update(thumb_index_distance_palm=0.05, **common)
    assert detector.update(
        thumb_index_distance_palm=0.05,
        tracking_valid=False,
        **common,
    ) == ("none", 0.0)


def test_validated_index_pose_blending_is_continuous_and_reversible() -> None:
    pose = (0.575, 0.0, 0.0, 0.0, 0.375, 0.90)
    blender = PinchPoseBlender({"index": pose}, maximum_weight_step=0.05)
    continuous = np.asarray((0.2, 0.1, 0.1, 0.1, 0.2, 0.3))
    previous = continuous
    for _ in range(20):
        blended, mode, weight = blender.update(
            continuous,
            detected_mode="index",
            confidence=1.0,
        )
        assert mode == "index"
        assert np.max(np.abs(blended - previous)) <= 0.031
        previous = blended
    assert weight == pytest.approx(1.0)
    assert blended == pytest.approx(pose)

    for _ in range(20):
        blended, mode, weight = blender.update(
            continuous,
            detected_mode="none",
            confidence=0.0,
        )
    assert mode == "none"
    assert weight == pytest.approx(0.0)
    assert blended == pytest.approx(continuous)


def test_pose_blend_fades_before_switching_modes_and_tracking_loss_exits() -> None:
    poses = {
        "index": (0.575, 0.0, 0.0, 0.0, 0.375, 0.90),
        "middle": (0.0, 0.6, 0.0, 0.0, 0.4, 0.8),
    }
    blender = PinchPoseBlender(poses, maximum_weight_step=0.25)
    continuous = np.zeros(6)
    blender.update(continuous, detected_mode="index", confidence=1.0)
    blender.update(continuous, detected_mode="index", confidence=1.0)
    _, mode, weight = blender.update(
        continuous, detected_mode="middle", confidence=1.0
    )
    assert mode == "index"
    assert weight == pytest.approx(0.25)
    _, mode, weight = blender.update(
        continuous, detected_mode="middle", confidence=1.0
    )
    assert mode == "middle"
    assert weight == pytest.approx(0.0)
    _, mode, weight = blender.update(
        continuous,
        detected_mode="middle",
        confidence=1.0,
        tracking_valid=False,
    )
    assert mode == "none"
    assert weight == pytest.approx(0.0)


def test_real_calibration_disables_unverified_pose_and_uses_thumb_first_gate() -> None:
    _, calibration = HandRetargetCalibration.load(
        "configs/hand/quest_rh56_real_retarget.yaml"
    )
    assert not calibration.pinch_pose_blending_enabled
    assert calibration.thumb_first_pinch_enabled
    assert calibration.thumb_first_lateral_target == pytest.approx(0.90)
    assert calibration.thumb_first_index_activation == pytest.approx(0.50)
    assert calibration.thumb_first_thumb_close_activation == pytest.approx(0.35)
    assert calibration.thumb_first_lateral_activation == pytest.approx(0.86)
    assert calibration.thumb_lateral_pregrasp_across_palm == pytest.approx(
        -0.339631
    )
    assert calibration.thumb_lateral_pregrasp_normalized == pytest.approx(0.90)
    assert calibration.validated_pinch_poses == {
        "index": pytest.approx((0.55, 0.0, 0.0, 0.0, 0.40, 0.90))
    }


def test_thumb_lateral_pregrasp_calibration_requires_complete_anchor(
    tmp_path: Path,
) -> None:
    source = Path("configs/hand/quest_rh56_real_retarget.yaml").read_text(
        encoding="utf-8"
    )
    path = tmp_path / "incomplete_thumb_anchor.yaml"
    path.write_text(
        source.replace("    pregrasp_normalized: 0.90\n", ""),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="malformed"):
        HandRetargetCalibration.load(path)


def test_thumb_close_uses_closest_non_thumb_fingertip() -> None:
    _, calibration = HandRetargetCalibration.load("configs/sim/quest_rh56_retarget.yaml")
    far = _open_points()
    thumb_tip = far[4]
    for index in (8, 12, 16, 20):
        far[index] = (0.20, 0.20, 0.0)
    middle_pinch = list(far)
    middle_pinch[12] = thumb_tip

    far_result = ProjectRh56Retargeter(calibration, backend="adaptive").retarget(
        _skeleton(far)
    )
    pinch_result = ProjectRh56Retargeter(calibration, backend="adaptive").retarget(
        _skeleton(middle_pinch)
    )
    assert pinch_result.pinch_diagnostics["thumb_index_pinch_strength"] == 0.0
    assert pinch_result.pinch_diagnostics["thumb_closest_fingertip_pinch_strength"] == 1.0
    assert (
        pinch_result.actuator_targets["thumb_close"]
        > far_result.actuator_targets["thumb_close"] + 0.19
    )


@pytest.mark.parametrize(
    ("bend", "pinch", "feature", "base", "assist"),
    (
        (0.0, 0.0, 0.0, 0.0, 0.0),
        (1.0, 0.0, 1.0, 1.0, 0.0),
        (1.0, 1.0, 1.0, 1.0, 0.0),
        (0.7, 0.4, 0.7, 0.7, 0.0),
        (0.2, 0.8, 0.44, 0.2, 0.24),
    ),
)
def test_thumb_close_bend_primary_pinch_assist_cases(
    bend: float,
    pinch: float,
    feature: float,
    base: float,
    assist: float,
) -> None:
    actual = thumb_close_bend_primary_feature(
        bend,
        pinch,
        bend_gain=1.0,
        pinch_assist_gain=0.4,
    )
    assert actual == pytest.approx((feature, base, assist))


def test_thumb_close_bend_primary_feature_is_continuous_monotonic_and_finite() -> None:
    samples = [index / 100.0 for index in range(101)]
    for pinch in samples:
        by_bend = [
            thumb_close_bend_primary_feature(
                bend,
                pinch,
                bend_gain=1.0,
                pinch_assist_gain=0.4,
            )[0]
            for bend in samples
        ]
        assert by_bend == sorted(by_bend)
        assert all(0.0 <= value <= 1.0 for value in by_bend)
        assert all(value == value for value in by_bend)
        assert max(
            abs(current - previous)
            for previous, current in zip(by_bend, by_bend[1:], strict=False)
        ) <= 0.011
    for bend in samples:
        by_pinch = [
            thumb_close_bend_primary_feature(
                bend,
                pinch,
                bend_gain=1.0,
                pinch_assist_gain=0.4,
            )[0]
            for pinch in samples
        ]
        assert by_pinch == sorted(by_pinch)


def test_thumb_close_bend_primary_rejects_nonfinite_inputs() -> None:
    with pytest.raises(ValueError, match="finite"):
        thumb_close_bend_primary_feature(
            float("nan"),
            0.0,
            bend_gain=1.0,
            pinch_assist_gain=0.4,
        )


def test_bend_only_full_feature_reaches_full_thumb_close_actuator_range() -> None:
    _, calibration = HandRetargetCalibration.load("configs/sim/quest_rh56_retarget.yaml")
    points = _open_points()
    points[1:5] = [
        (-0.04, 0.00, 0.0),
        (-0.04, 0.02, 0.0),
        (-0.02, 0.02, 0.0),
        (-0.02, 0.00, 0.0),
    ]
    for index in (8, 12, 16, 20):
        points[index] = (0.20, 0.20, 0.0)

    result = ProjectRh56Retargeter(calibration, backend="adaptive").retarget(
        _skeleton(points)
    )

    assert result.pinch_diagnostics["thumb_normalized_bend"] == pytest.approx(1.0)
    assert result.pinch_diagnostics["thumb_normalized_pinch"] == pytest.approx(0.0)
    assert result.pinch_diagnostics["thumb_close_feature"] == pytest.approx(1.0)
    assert result.actuator_targets["thumb_close"] == pytest.approx(
        RH56_THUMB_CLOSE_RANGE_RAD
    )


def test_right_hand_palm_frame_is_orthonormal_with_positive_index_to_pinky_sign() -> None:
    points = np.asarray(_open_points(), dtype=np.float64)
    frame = right_hand_palm_local_frame(points, epsilon_m=1e-5)

    assert frame is not None
    across = np.asarray(frame.across_axis)
    forward = np.asarray(frame.forward_axis)
    normal = np.asarray(frame.normal_axis)
    assert np.linalg.norm(across) == pytest.approx(1.0)
    assert np.linalg.norm(forward) == pytest.approx(1.0)
    assert np.linalg.norm(normal) == pytest.approx(1.0)
    assert np.dot(across, forward) == pytest.approx(0.0, abs=1e-12)
    assert np.dot(across, normal) == pytest.approx(0.0, abs=1e-12)
    assert np.dot(forward, normal) == pytest.approx(0.0, abs=1e-12)
    assert np.cross(across, forward) == pytest.approx(normal)
    assert np.dot(points[17] - points[5], across) > 0.0


def test_thumb_lateral_synthetic_across_palm_sweep_is_monotonic() -> None:
    points = np.asarray(_open_points(), dtype=np.float64)
    frame = right_hand_palm_local_frame(points, epsilon_m=1e-5)
    assert frame is not None
    across = np.asarray(frame.across_axis)
    base = points[1].copy()
    values = []
    for raw in np.linspace(-0.70, 0.35, 43):
        points[4] = base + across * raw * frame.palm_width_m
        feature, measured = thumb_lateral_opposition_feature(
            points,
            frame,
            open_across_palm=-0.60,
            opposed_across_palm=0.25,
            palm_scale=1.0,
        )
        values.append(feature)
        assert measured == pytest.approx(raw)
    assert values == sorted(values)
    assert values[0] == pytest.approx(0.0)
    assert values[-1] == pytest.approx(1.0)


def test_thumb_lateral_pregrasp_anchor_preserves_endpoints_and_monotonicity() -> None:
    points = np.asarray(_open_points(), dtype=np.float64)
    frame = right_hand_palm_local_frame(points, epsilon_m=1e-5)
    assert frame is not None
    across = np.asarray(frame.across_axis)
    base = points[1].copy()
    raw_points = (-1.137268, -0.339631, 0.060326)
    expected = (0.0, 0.90, 1.0)
    measured_features = []
    for raw, target in zip(raw_points, expected, strict=True):
        points[4] = base + across * raw * frame.palm_width_m
        feature, measured = thumb_lateral_opposition_feature(
            points,
            frame,
            open_across_palm=raw_points[0],
            pregrasp_across_palm=raw_points[1],
            pregrasp_normalized=expected[1],
            opposed_across_palm=raw_points[2],
            palm_scale=1.0,
        )
        assert measured == pytest.approx(raw)
        assert feature == pytest.approx(target)
        measured_features.append(feature)
    assert measured_features == sorted(measured_features)


def test_thumb_lateral_feature_is_invariant_to_common_wrist_frame_rotation() -> None:
    points = np.asarray(_open_points(), dtype=np.float64)
    angle = 0.73
    rotation = np.asarray(
        (
            (np.cos(angle), -np.sin(angle), 0.0),
            (np.sin(angle), np.cos(angle), 0.0),
            (0.0, 0.0, 1.0),
        )
    )
    rotated = points @ rotation.T
    original_frame = right_hand_palm_local_frame(points, epsilon_m=1e-5)
    rotated_frame = right_hand_palm_local_frame(rotated, epsilon_m=1e-5)
    assert original_frame is not None and rotated_frame is not None
    original = thumb_lateral_opposition_feature(
        points,
        original_frame,
        open_across_palm=-0.60,
        opposed_across_palm=0.25,
        palm_scale=1.0,
    )
    transformed = thumb_lateral_opposition_feature(
        rotated,
        rotated_frame,
        open_across_palm=-0.60,
        opposed_across_palm=0.25,
        palm_scale=1.0,
    )
    assert transformed == pytest.approx(original)


def test_degenerate_palm_frame_is_rejected_without_nan() -> None:
    _, calibration = HandRetargetCalibration.load("configs/sim/quest_rh56_retarget.yaml")
    points = _open_points()
    points[17] = points[5]

    result = ProjectRh56Retargeter(calibration, backend="adaptive").retarget(
        _skeleton(points)
    )

    assert not result.valid
    assert result.rejection_reason == "DEGENERATE_PALM_FRAME"
    assert not result.actuator_targets


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


def test_thumb_first_pinch_passes_early_index_approach_without_intervention() -> None:
    sequencer = ThumbFirstPinchSequencer(
        enabled=True,
        lateral_target=0.90,
        lateral_tolerance=0.04,
        index_guard=0.12,
        thumb_close_guard=0.22,
    )
    continuous = np.array([0.48, 0.0, 0.0, 0.0, 0.38, 0.65])
    passed, stage = sequencer.update(
        continuous,
        detected_mode="index",
        confidence=1.0,
        tracking_valid=True,
        measured_canonical=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.40]),
    )
    assert stage == "idle"
    np.testing.assert_allclose(passed, continuous)


def test_thumb_first_pinch_approaches_verified_pose_without_guard_retreat() -> None:
    sequencer = ThumbFirstPinchSequencer(
        enabled=True,
        lateral_target=0.90,
        lateral_tolerance=0.04,
        index_guard=0.12,
        thumb_close_guard=0.22,
    )
    continuous = np.array([0.55, 0.0, 0.0, 0.0, 0.40, 0.90])
    held, stage = sequencer.update(
        continuous,
        detected_mode="index",
        confidence=1.0,
        tracking_valid=True,
        measured_canonical=np.array([0.45, 0.0, 0.0, 0.0, 0.35, 0.80]),
    )
    assert stage == "index_approach"
    np.testing.assert_allclose(held, continuous)

    advanced, stage = sequencer.update(
        continuous,
        detected_mode="index",
        confidence=1.0,
        tracking_valid=True,
        measured_canonical=np.array([0.12, 0.0, 0.0, 0.0, 0.22, 0.90]),
    )
    assert stage == "index_approach"
    np.testing.assert_allclose(advanced, continuous)


def test_thumb_first_pinch_releases_intervention_when_target_leaves_verified_pose() -> None:
    sequencer = ThumbFirstPinchSequencer(
        enabled=True,
        lateral_target=0.90,
        lateral_tolerance=0.04,
        index_guard=0.12,
        thumb_close_guard=0.22,
    )
    verified = np.array([0.55, 0.0, 0.0, 0.0, 0.40, 0.90])
    _, stage = sequencer.update(
        verified,
        detected_mode="index",
        confidence=1.0,
        tracking_valid=True,
        measured_canonical=np.array([0.12, 0.0, 0.0, 0.0, 0.22, 0.90]),
    )
    assert stage == "index_approach"
    continuous = np.array([0.48, 0.0, 0.0, 0.0, 0.38, 0.65])
    held, stage = sequencer.update(
        continuous,
        detected_mode="index",
        confidence=1.0,
        tracking_valid=True,
        measured_canonical=np.array([0.12, 0.0, 0.0, 0.0, 0.22, 0.88]),
    )
    assert stage == "idle"
    np.testing.assert_allclose(held, continuous)


def test_thumb_first_pinch_keeps_direct_target_when_lateral_is_pushed_back() -> None:
    sequencer = ThumbFirstPinchSequencer(
        enabled=True,
        lateral_target=0.90,
        lateral_tolerance=0.04,
        index_guard=0.12,
        thumb_close_guard=0.22,
    )
    continuous = np.array([0.55, 0.0, 0.0, 0.0, 0.40, 0.90])
    sequencer.update(
        continuous,
        detected_mode="index",
        confidence=1.0,
        tracking_valid=True,
        measured_canonical=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.90]),
    )
    recovered, stage = sequencer.update(
        continuous,
        detected_mode="index",
        confidence=1.0,
        tracking_valid=True,
        measured_canonical=np.array([0.12, 0.0, 0.0, 0.0, 0.22, 0.80]),
    )
    assert stage == "index_approach"
    np.testing.assert_allclose(recovered, continuous)
