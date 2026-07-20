from __future__ import annotations

import math

import pytest

from teleoperation.contracts import (
    ArmPoseSample,
    Pose3D,
    RunGateSample,
    TimestampSet,
)
from teleoperation.processing.clutch import ClutchController, ClutchState
from teleoperation.processing.one_euro_se3 import OneEuroSE3Filter
from teleoperation.transforms.frame_mapping import CentralFrameMapping, RelativePoseMapper
from teleoperation.transforms.se3 import quaternion_angle, quaternion_exp


IDENTITY = Pose3D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def sample(sequence: int, timestamp: int, pose: Pose3D = IDENTITY) -> ArmPoseSample:
    return ArmPoseSample("source", sequence, "normalized", pose, TimestampSet(timestamp))


def gate(sequence: int, timestamp: int, engaged: bool, valid: bool = True) -> RunGateSample:
    return RunGateSample("source", sequence, timestamp, engaged, valid)


def controller() -> ClutchController:
    frames = CentralFrameMapping(
        "source", "normalized", "robot_base", "tcp",
        ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    )
    return ClutchController(
        RelativePoseMapper(frames, translation_scale=0.05, rotation_scale=0.05),
        poses_are_operator_frame=True,
    )


def test_startup_requires_release_then_fresh_engage_and_release_holds() -> None:
    clutch = controller()
    assert clutch.update(sample(1, 1), gate(1, 1, True), robot_tcp_pose=IDENTITY).state == ClutchState.WAITING_FOR_RELEASE
    assert clutch.update(sample(2, 2), gate(2, 2, False), robot_tcp_pose=IDENTITY).state == ClutchState.DISENGAGED
    engaged = clutch.update(sample(3, 3), gate(3, 3, True), robot_tcp_pose=IDENTITY)
    assert engaged.state == ClutchState.ACTIVE
    assert engaged.target_pose == IDENTITY
    released = clutch.update(sample(4, 4), gate(4, 4, False), robot_tcp_pose=IDENTITY)
    assert released.state == ClutchState.DISENGAGED
    assert released.target_pose is None


def test_discontinuity_requires_release_and_reclutch() -> None:
    clutch = controller()
    clutch.update(sample(1, 1), gate(1, 1, False), robot_tcp_pose=IDENTITY)
    clutch.update(sample(2, 2), gate(2, 2, True), robot_tcp_pose=IDENTITY)
    clutch.require_recenter("reconnect")
    held = clutch.update(sample(3, 3), gate(3, 3, True), robot_tcp_pose=IDENTITY)
    assert held.state == ClutchState.RECENTER_REQUIRED
    assert clutch.update(sample(4, 4), gate(4, 4, False), robot_tcp_pose=IDENTITY).state == ClutchState.DISENGAGED
    assert clutch.update(sample(5, 5), gate(5, 5, True), robot_tcp_pose=IDENTITY).state == ClutchState.ACTIVE


def test_one_euro_orientation_is_geodesic_and_reset_is_exact() -> None:
    filter_ = OneEuroSE3Filter()
    first = sample(1, 1_000_000_000)
    second = sample(2, 1_010_000_000, Pose3D((0.1, 0.0, 0.0), quaternion_exp((0.0, 0.0, math.pi / 2))))
    assert filter_.filter(first) == first
    filtered = filter_.filter(second)
    angle = quaternion_angle(filtered.pose.quaternion_xyzw, first.pose.quaternion_xyzw)
    assert 0.0 < angle < math.pi / 2
    assert math.isclose(sum(value * value for value in filtered.pose.quaternion_xyzw), 1.0, rel_tol=1e-12)
    filter_.reset()
    assert filter_.filter(second) == second
