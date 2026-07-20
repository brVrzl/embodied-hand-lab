from __future__ import annotations

import math

import numpy as np
import pytest

from teleoperation.contracts import Pose3D
from teleoperation.transforms.frame_mapping import CentralFrameMapping, RelativePoseMapper
from teleoperation.transforms.se3 import (
    compose_pose,
    inverse_pose,
    quaternion_angle,
    quaternion_exp,
    quaternion_log,
)


IDENTITY = Pose3D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def frames(basis: tuple[tuple[float, float, float], ...] | None = None) -> CentralFrameMapping:
    return CentralFrameMapping(
        "source",
        "normalized",
        "robot_base",
        "tcp",
        basis or ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    )


@pytest.mark.parametrize("axis", range(3))
@pytest.mark.parametrize("sign", (-1.0, 1.0))
def test_all_six_translation_directions(axis: int, sign: float) -> None:
    mapper = RelativePoseMapper(frames(), translation_scale=0.05, rotation_scale=0.05)
    mapper.anchor(IDENTITY, IDENTITY)
    position = [0.0, 0.0, 0.0]
    position[axis] = sign
    target = mapper.map(Pose3D(tuple(position), IDENTITY.quaternion_xyzw))
    expected = [0.0, 0.0, 0.0]
    expected[axis] = sign * 0.05
    assert target.position_m == pytest.approx(expected)


@pytest.mark.parametrize("axis", range(3))
def test_positive_rotation_about_each_axis(axis: int) -> None:
    mapper = RelativePoseMapper(frames(), translation_scale=0.05, rotation_scale=0.10)
    mapper.anchor(IDENTITY, IDENTITY)
    vector = np.zeros(3)
    vector[axis] = 0.5
    target = mapper.map(Pose3D((0.0, 0.0, 0.0), quaternion_exp(vector)))
    assert quaternion_log(target.quaternion_xyzw) == pytest.approx(vector * 0.10)


def test_handedness_conversion_is_centralized_and_orientation_stays_proper() -> None:
    mapping = frames(((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
    pose = Pose3D((1.0, 2.0, 3.0), quaternion_exp((0.2, 0.0, 0.0)))
    converted = mapping.source_pose_to_operator(pose)
    assert converted.position_m == pytest.approx((-1.0, 2.0, 3.0))
    # Rotation vectors are axial vectors: under a reflection they transform as
    # det(C) * C * omega. Reflecting X therefore preserves an X-axis rotation.
    assert quaternion_log(converted.quaternion_xyzw) == pytest.approx((0.2, 0.0, 0.0))


def test_recenter_invariance_relative_composition_and_inverse() -> None:
    robot = Pose3D((0.4, -0.2, 0.3), quaternion_exp((0.1, -0.2, 0.3)))
    source = Pose3D((2.0, 3.0, 4.0), quaternion_exp((-0.2, 0.1, 0.4)))
    mapper = RelativePoseMapper(frames(), translation_scale=0.05, rotation_scale=0.05)
    mapper.anchor(source, robot)
    assert mapper.map(source).position_m == pytest.approx(robot.position_m)
    assert quaternion_angle(mapper.map(source).quaternion_xyzw, robot.quaternion_xyzw) < 1e-12
    composed = compose_pose(source, inverse_pose(source))
    assert composed.position_m == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    assert quaternion_angle(composed.quaternion_xyzw, IDENTITY.quaternion_xyzw) < 1e-12
