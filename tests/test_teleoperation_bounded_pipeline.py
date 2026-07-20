from __future__ import annotations

import math

from teleoperation.contracts import (
    ArmPoseSample,
    OperatorActionSample,
    Pose3D,
    RunGateSample,
    TimestampSet,
    TrackingState,
)
from teleoperation.input.interface import AdapterSnapshot
from teleoperation.processing.clutch import ClutchController, ClutchState
from teleoperation.processing.one_euro_se3 import OneEuroSE3Filter
from teleoperation.processing.pose_validator import PoseValidator
from teleoperation.processing.target_shaper import JerkLimitedPoseShaper
from teleoperation.runtime.teledex_arm import BoundedArmTeleoperationPipeline
from teleoperation.supervision import ArmSafetySupervisor, SafetyEnvelope
from teleoperation.teledex_config import load_bounded_teleop_config
from teleoperation.transforms.frame_mapping import RelativePoseMapper


IDENTITY = Pose3D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def snapshot(
    sequence: int,
    timestamp_ns: int,
    *,
    engaged: bool,
    position_x: float = 0.0,
    valid: bool = True,
) -> AdapterSnapshot:
    pose = ArmPoseSample(
        "teledex_phone",
        sequence,
        "teledex_arkit_session_v0_0_7",
        Pose3D((position_x, 0.0, 0.0), IDENTITY.quaternion_xyzw),
        TimestampSet(timestamp_ns, processing_ns=timestamp_ns),
        tracking_valid=valid,
        tracking_state=TrackingState.UNKNOWN if valid else TrackingState.INVALID,
    )
    gate = RunGateSample("teledex_phone", sequence, timestamp_ns, engaged, valid)
    action = OperatorActionSample("teledex_phone", sequence, timestamp_ns, valid=valid)
    return AdapterSnapshot(pose, gate, True, sequence, "ok", action)


def make_pipeline() -> BoundedArmTeleoperationPipeline:
    config = load_bounded_teleop_config("configs/teleoperation/teledex_jaka_arm_bounded.yaml")
    mapper = RelativePoseMapper(
        config.frames,
        translation_scale=config.translation_scale,
        rotation_scale=config.rotation_scale,
    )
    return BoundedArmTeleoperationPipeline(
        validator=PoseValidator(config.validation),
        clutch=ClutchController(mapper, poses_are_operator_frame=True),
        measurement_filter=OneEuroSE3Filter(),
        shaper=JerkLimitedPoseShaper(config.cartesian_limits),
        safety=ArmSafetySupervisor(
            SafetyEnvelope(
                IDENTITY,
                config.cartesian_limits.workspace_half_extent_m,
                config.cartesian_limits.maximum_orientation_deviation_rad,
                config.maximum_session_ns,
            ),
            config.joint_limits,
        ),
        startup_tcp_relative_output=True,
    )


def test_pipeline_starts_at_zero_requires_clutch_edge_and_stops_on_invalid_input() -> None:
    pipeline = make_pipeline()
    start = 1_000_000_000
    released = pipeline.process(snapshot(1, start, engaged=False), robot_tcp_pose=IDENTITY, now_ns=start)
    assert released.target is None
    assert pipeline.clutch.state == ClutchState.DISENGAGED

    engaged = pipeline.process(
        snapshot(2, start + 16_000_000, engaged=True),
        robot_tcp_pose=IDENTITY,
        now_ns=start + 16_000_000,
    )
    assert engaged.target is not None
    assert engaged.target.target_frame_id == "startup_tcp_relative"
    assert engaged.target.pose == IDENTITY

    moved = pipeline.process(
        snapshot(3, start + 32_000_000, engaged=True, position_x=0.01),
        robot_tcp_pose=IDENTITY,
        now_ns=start + 32_000_000,
    )
    assert moved.target is not None
    assert moved.target.pose.position_m[0] > 0.0
    assert moved.target.pose.position_m[0] < 0.0005

    invalid = pipeline.process(
        snapshot(4, start + 48_000_000, engaged=False, valid=False),
        robot_tcp_pose=IDENTITY,
        now_ns=start + 48_000_000,
    )
    assert invalid.target is None
    assert pipeline.clutch.state in {ClutchState.RECENTER_REQUIRED, ClutchState.FAULT}


def test_commissioning_config_is_bounded_and_motion_gate_is_closed() -> None:
    config = load_bounded_teleop_config("configs/teleoperation/teledex_jaka_arm_bounded.yaml")
    assert config.translation_scale == 0.05
    assert config.rotation_scale == 0.05
    assert config.cartesian_limits.workspace_half_extent_m == (0.015, 0.015, 0.015)
    assert config.cartesian_limits.maximum_linear_speed_m_s == 0.008
    assert config.joint_limits.maximum_velocity_rad_s == 0.03
    assert config.maximum_session_ns == 10_000_000_000
    assert not config.motion_authorized_by_configuration
