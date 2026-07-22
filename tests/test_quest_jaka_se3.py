from __future__ import annotations

import math

import numpy as np
import pytest

from motion_input import Pose6D
from quest_jaka_sim.se3 import (
    OneEuroQuaternionFilter,
    OneEuroVectorFilter,
    PoseSampleBuffer,
    TimedPoseSample,
    align_quaternion_sign,
    compose_pose,
    normalize_quaternion_xyzw,
    quaternion_angle_rad,
    quaternion_slerp_xyzw,
    relative_pose,
    rotvec_to_quaternion_xyzw,
)


def test_quaternion_validation_normalization_and_sign_continuity() -> None:
    assert normalize_quaternion_xyzw((0, 0, 0, 2)) == (0.0, 0.0, 0.0, 1.0)
    assert align_quaternion_sign((0, 0, 0, -1), (0, 0, 0, 1)) == (
        0.0,
        0.0,
        0.0,
        1.0,
    )
    with pytest.raises(ValueError, match="minimum"):
        normalize_quaternion_xyzw((0, 0, 0, 0))
    with pytest.raises(ValueError, match="finite"):
        normalize_quaternion_xyzw((math.nan, 0, 0, 1))


def test_relative_pose_and_composition_use_reference_local_multiplication_order() -> None:
    qz90 = rotvec_to_quaternion_xyzw((0.0, 0.0, math.pi / 2.0))
    reference = Pose6D((1.0, 2.0, 3.0), qz90)
    # One metre along reference-local +X is parent/world +Y.
    current = Pose6D((1.0, 3.0, 3.0), qz90)
    delta = relative_pose(reference, current)
    assert delta.position_m == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)
    assert quaternion_angle_rad(delta.orientation_xyzw, (0, 0, 0, 1)) < 1e-9
    assert compose_pose(reference, delta).position_m == pytest.approx(current.position_m)
    assert quaternion_angle_rad(
        compose_pose(reference, delta).orientation_xyzw, current.orientation_xyzw
    ) < 1e-9


def test_relative_orientation_and_shortest_path_slerp() -> None:
    qx90 = rotvec_to_quaternion_xyzw((math.pi / 2.0, 0.0, 0.0))
    reference = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    current = Pose6D((0.0, 0.0, 0.0), qx90)
    assert quaternion_angle_rad(relative_pose(reference, current).orientation_xyzw, qx90) < 1e-9
    midpoint = quaternion_slerp_xyzw((0, 0, 0, 1), tuple(-x for x in qx90), 0.5)
    assert quaternion_angle_rad(
        midpoint, rotvec_to_quaternion_xyzw((math.pi / 4.0, 0.0, 0.0))
    ) < 1e-8


def test_timestamp_aware_one_euro_filters_reduce_stationary_noise_and_reset() -> None:
    vector_filter = OneEuroVectorFilter(
        min_cutoff_hz=1.0,
        beta=0.0,
        derivative_cutoff_hz=1.0,
        maximum_dt_s=0.05,
    )
    rng = np.random.default_rng(7)
    raw = rng.normal(0.0, 0.002, size=(300, 3))
    filtered = np.asarray(
        [vector_filter.filter(index * 10_000_000, value) for index, value in enumerate(raw)]
    )
    assert float(np.sqrt(np.mean(filtered[50:] ** 2))) < float(
        np.sqrt(np.mean(raw[50:] ** 2))
    )
    with pytest.raises(ValueError, match="strictly monotonic"):
        vector_filter.filter(299 * 10_000_000, (0.0, 0.0, 0.0))
    vector_filter.reset()
    assert vector_filter.filter(1, (1.0, 2.0, 3.0)) == pytest.approx((1.0, 2.0, 3.0))

    rotation_filter = OneEuroQuaternionFilter(
        min_cutoff_hz=1.0,
        beta=0.0,
        derivative_cutoff_hz=1.0,
        maximum_dt_s=0.05,
    )
    first = rotation_filter.filter(0, (0, 0, 0, 1))
    second = rotation_filter.filter(
        10_000_000, rotvec_to_quaternion_xyzw((0.0, 0.0, 0.1))
    )
    assert quaternion_angle_rad(first, second) < 0.1


def test_pose_sample_buffer_interpolates_position_and_orientation_without_extrapolation() -> None:
    buffer: PoseSampleBuffer[str] = PoseSampleBuffer(capacity=3)
    identity = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    qz90 = rotvec_to_quaternion_xyzw((0.0, 0.0, math.pi / 2.0))
    assert buffer.add(TimedPoseSample(100, 1, identity, "left"))
    assert buffer.add(TimedPoseSample(200, 2, Pose6D((2.0, 0.0, 0.0), qz90), "right"))
    middle = buffer.sample(150)
    assert middle is not None
    assert middle.pose.position_m == pytest.approx((1.0, 0.0, 0.0))
    assert quaternion_angle_rad(
        middle.pose.orientation_xyzw,
        rotvec_to_quaternion_xyzw((0.0, 0.0, math.pi / 4.0)),
    ) < 1e-8
    assert buffer.sample(300) == buffer.latest
    assert not buffer.add(TimedPoseSample(150, 3, identity, "old"))
    assert buffer.dropped_out_of_order == 1
