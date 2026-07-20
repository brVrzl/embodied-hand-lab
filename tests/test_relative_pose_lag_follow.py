from __future__ import annotations

import pytest

from teleop_tools.hebi_mobile_io import (
    HebiMobileIOSnapshot,
    quat_conjugate_wxyz,
    rotate_vector_wxyz,
)
from teleop_tools.relative_pose_lag_follow import RelativePoseLagFollower, RelativePoseLagFollowConfig, TcpPose


IDENTITY_AXIS_MAP = {
    "x": {"source": "x", "sign": 1.0, "scale": 1.0},
    "y": {"source": "y", "sign": 1.0, "scale": 1.0},
    "z": {"source": "z", "sign": 1.0, "scale": 1.0},
}


def _snapshot(
    timestamp: float,
    position: list[float],
    *,
    b1: bool = True,
    quaternion_wxyz: list[float] | None = None,
) -> HebiMobileIOSnapshot:
    return HebiMobileIOSnapshot(
        timestamp_sec=timestamp,
        position_m=position,
        quaternion_wxyz=quaternion_wxyz or [1.0, 0.0, 0.0, 0.0],
        raw_inputs={"b1": b1},
    )


def _actual(position: list[float]) -> TcpPose:
    return TcpPose(position, [1.0, 0.0, 0.0, 0.0])


def _actual_pose(position: list[float], quaternion_wxyz: list[float]) -> TcpPose:
    return TcpPose(position, quaternion_wxyz)


def _config(**overrides: object) -> RelativePoseLagFollowConfig:
    defaults: dict[str, object] = {
        "target_response_mode": "direct",
        "position_scale": 1.0,
        "workspace_min_m": (-1.0, -1.0, -1.0),
        "workspace_max_m": (1.0, 1.0, 1.0),
        "phone_translation_deadband_m": 0.0,
        "phone_still_translation_m": 0.005,
        "phone_still_freeze_tracking_error_m": 0.004,
        "target_filter_time_constant_sec": 0.0,
        "max_target_velocity_m_s": 10.0,
        "max_target_jump_m": 0.0,
        "phone_to_robot_axis_map": IDENTITY_AXIS_MAP,
    }
    defaults.update(overrides)
    return RelativePoseLagFollowConfig(**defaults)


def test_direct_mode_freezes_to_actual_pose_when_phone_is_still() -> None:
    follower = RelativePoseLagFollower(_config())
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    moved = follower.step(_snapshot(0.1, [0.040, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    assert moved.palm_target_position_m == pytest.approx([0.040, 0.0, 0.0])

    frozen = follower.step(_snapshot(0.2, [0.041, 0.0, 0.0]), _actual([0.039, 0.0, 0.0]), q_current)

    assert frozen.command_deadman is True
    assert frozen.palm_target_position_m == pytest.approx([0.039, 0.0, 0.0])
    assert frozen.log["still_freeze"] is True


def test_direct_mode_does_not_freeze_when_tracking_lag_exceeds_limit() -> None:
    follower = RelativePoseLagFollower(_config(phone_still_freeze_tracking_error_m=0.001))
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    moved = follower.step(_snapshot(0.1, [0.040, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    assert moved.palm_target_position_m == pytest.approx([0.040, 0.0, 0.0])

    output = follower.step(_snapshot(0.2, [0.041, 0.0, 0.0]), _actual([-0.02, 0.0, 0.0]), q_current)

    assert output.command_deadman is True
    assert output.palm_target_position_m == pytest.approx([0.041, 0.0, 0.0])
    assert output.log["phone_still"] is True
    assert output.log["still_freeze"] is False
    assert output.log["still_freeze_tracking_error_m"] > 0.001


def test_slow_cumulative_phone_motion_is_not_frozen_by_per_frame_still() -> None:
    follower = RelativePoseLagFollower(
        _config(
            max_target_velocity_m_s=0.10,
            phone_jump_reject_translation_m=1.0,
            phone_still_translation_m=0.005,
            phone_still_freeze_tracking_error_m=0.001,
        )
    )
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    first = follower.step(_snapshot(0.1, [0.003, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    second = follower.step(_snapshot(0.2, [0.006, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)

    assert first.log["phone_still"] is True
    assert first.log["still_freeze"] is False
    assert first.palm_target_position_m == pytest.approx([0.003, 0.0, 0.0])
    assert second.log["phone_still"] is True
    assert second.log["still_freeze"] is False
    assert second.palm_target_position_m == pytest.approx([0.006, 0.0, 0.0])


def test_phone_pose_jump_rejects_command_and_resets_anchor() -> None:
    follower = RelativePoseLagFollower(_config(phone_jump_reject_translation_m=0.05))
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    output = follower.step(_snapshot(0.1, [0.20, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)

    assert output.command_deadman is False
    assert output.palm_target_position_m is None
    assert output.log["reason"] == "phone_pose_jump_rejected"
    assert follower.phone_anchor_pose is None


def test_phone_pose_jump_requires_deadman_release_when_configured() -> None:
    follower = RelativePoseLagFollower(
        _config(
            phone_jump_reject_translation_m=0.05,
            reanchor_requires_deadman_release=True,
        )
    )
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    rejected = follower.step(_snapshot(0.1, [0.20, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    locked = follower.step(_snapshot(0.2, [0.20, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    released = follower.step(
        _snapshot(0.3, [0.20, 0.0, 0.0], b1=False),
        _actual([0.0, 0.0, 0.0]),
        q_current,
    )
    reanchored = follower.step(_snapshot(0.4, [0.20, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)

    assert rejected.log["reason"] == "phone_pose_jump_rejected"
    assert locked.command_deadman is False
    assert locked.log["reason"] == "waiting_for_deadman_release_after_reject"
    assert released.command_deadman is False
    assert reanchored.command_deadman is True


def test_direct_mode_limits_target_velocity() -> None:
    follower = RelativePoseLagFollower(
        _config(
            freeze_when_phone_still=False,
            max_target_velocity_m_s=0.01,
            phone_jump_reject_translation_m=1.0,
        )
    )
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    output = follower.step(_snapshot(0.1, [0.10, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)

    assert output.command_deadman is True
    assert output.palm_target_position_m == pytest.approx([0.001, 0.0, 0.0])
    assert output.log["target_velocity_limited"] is True


def test_calibrated_rotation_matrix_maps_phone_delta_to_robot_base() -> None:
    follower = RelativePoseLagFollower(
        _config(
            freeze_when_phone_still=False,
            max_pos_tracking_error_pause_m=0.20,
            phone_jump_reject_translation_m=1.0,
            phone_to_robot_rotation_matrix=(
                (0.0, -1.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0),
            ),
        )
    )
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    output = follower.step(_snapshot(0.1, [0.10, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)

    assert output.command_deadman is True
    assert output.log["mapped_phone_delta_m"] == pytest.approx([0.0, 0.10, 0.0])
    assert output.palm_target_position_m == pytest.approx([0.0, 0.10, 0.0])


def test_calibrated_rotation_matrix_rejects_reflection() -> None:
    with pytest.raises(ValueError, match="determinant"):
        RelativePoseLagFollower(
            _config(
                phone_to_robot_rotation_matrix=(
                    (-1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                )
            )
        )


def test_direct_mode_limits_target_acceleration() -> None:
    follower = RelativePoseLagFollower(
        _config(
            freeze_when_phone_still=False,
            max_target_velocity_m_s=10.0,
            max_target_acceleration_m_s2=0.10,
            phone_jump_reject_translation_m=1.0,
        )
    )
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    output = follower.step(_snapshot(0.1, [0.10, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)

    assert output.command_deadman is True
    assert output.palm_target_position_m == pytest.approx([0.001, 0.0, 0.0])
    assert output.log["target_acceleration_limited"] is True


def test_direct_mode_deadband_does_not_discard_acceleration_limited_startup_steps() -> None:
    follower = RelativePoseLagFollower(
        _config(
            freeze_when_phone_still=False,
            max_target_velocity_m_s=10.0,
            max_target_acceleration_m_s2=0.10,
            target_update_deadband_m=0.0004,
            target_update_release_m=0.0010,
            phone_jump_reject_translation_m=1.0,
        )
    )
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    first = follower.step(_snapshot(0.02, [0.020, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    second = follower.step(_snapshot(0.04, [0.020, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)

    assert first.palm_target_position_m == pytest.approx([0.00004, 0.0, 0.0])
    assert second.palm_target_position_m == pytest.approx([0.00012, 0.0, 0.0])
    assert first.log["target_acceleration_limited"] is True
    assert first.log["target_deadband_hold"] is False


def test_direct_mode_holds_small_target_updates_until_release_threshold() -> None:
    follower = RelativePoseLagFollower(
        _config(
            freeze_when_phone_still=False,
            target_update_deadband_m=0.002,
            target_update_release_m=0.004,
            phone_jump_reject_translation_m=1.0,
        )
    )
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    first = follower.step(_snapshot(0.1, [0.010, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    held = follower.step(_snapshot(0.2, [0.011, 0.0, 0.0]), _actual([0.010, 0.0, 0.0]), q_current)
    released = follower.step(_snapshot(0.3, [0.015, 0.0, 0.0]), _actual([0.010, 0.0, 0.0]), q_current)

    assert first.palm_target_position_m == pytest.approx([0.010, 0.0, 0.0])
    assert held.palm_target_position_m == pytest.approx([0.010, 0.0, 0.0])
    assert held.log["target_deadband_hold"] is True
    assert released.palm_target_position_m == pytest.approx([0.015, 0.0, 0.0])
    assert released.log["target_deadband_hold"] is False


def test_workspace_bounds_do_not_pull_anchor_back_when_manual_pose_starts_outside_box() -> None:
    follower = RelativePoseLagFollower(
        _config(
            freeze_when_phone_still=False,
            workspace_min_m=(-1.0, -1.0, 0.25),
            workspace_max_m=(1.0, 1.0, 1.0),
        )
    )
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.20]), q_current)
    output = follower.step(_snapshot(0.1, [0.0, 0.0, -0.01]), _actual([0.0, 0.0, 0.20]), q_current)

    assert output.command_deadman is True
    assert output.palm_target_position_m == pytest.approx([0.0, 0.0, 0.20])
    assert output.log["desired_tcp_pose_workspace_bounded"]["position_m"] == pytest.approx([0.0, 0.0, 0.20])


def test_lower_workspace_allows_downward_motion_from_manual_low_pose() -> None:
    follower = RelativePoseLagFollower(
        _config(
            freeze_when_phone_still=False,
            workspace_min_m=(-1.0, -1.0, 0.16),
            workspace_max_m=(1.0, 1.0, 1.0),
        )
    )
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.20]), q_current)
    output = follower.step(_snapshot(0.1, [0.0, 0.0, -0.01]), _actual([0.0, 0.0, 0.20]), q_current)

    assert output.command_deadman is True
    assert output.palm_target_position_m == pytest.approx([0.0, 0.0, 0.19])
    assert output.log["desired_tcp_pose_workspace_bounded"]["position_m"] == pytest.approx([0.0, 0.0, 0.19])


def test_orientation_control_maps_phone_relative_rotation_to_target_quaternion() -> None:
    follower = RelativePoseLagFollower(
        _config(
            freeze_when_phone_still=False,
            orientation_control_enabled=True,
            orientation_scale=1.0,
            phone_jump_reject_rotation_rad=10.0,
            max_rot_tracking_error_pause_rad=10.0,
        )
    )
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    output = follower.step(
        _snapshot(
            0.1,
            [0.0, 0.0, 0.0],
            quaternion_wxyz=[0.9238795325, 0.0, 0.0, 0.3826834324],
        ),
        _actual([0.0, 0.0, 0.0]),
        q_current,
    )

    assert output.command_deadman is True
    assert output.palm_target_quaternion_wxyz == pytest.approx(
        [0.9238795325, 0.0, 0.0, 0.3826834324]
    )
    assert output.log["desired_tcp_pose_raw"]["quaternion_wxyz"] == pytest.approx(
        [0.9238795325, 0.0, 0.0, 0.3826834324]
    )


def test_axis_mapped_relative_orientation_uses_phone_to_robot_axis_map() -> None:
    follower = RelativePoseLagFollower(
        _config(
            freeze_when_phone_still=False,
            orientation_control_enabled=True,
            orientation_mapping_mode="axis_mapped_relative",
            orientation_scale=1.0,
            phone_to_robot_axis_map={
                "x": {"source": "y", "sign": 1.0, "scale": 1.0},
                "y": {"source": "x", "sign": -1.0, "scale": 1.0},
                "z": {"source": "z", "sign": 1.0, "scale": 1.0},
            },
            phone_jump_reject_rotation_rad=10.0,
            max_rot_tracking_error_pause_rad=10.0,
        )
    )
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    output = follower.step(
        _snapshot(
            0.1,
            [0.0, 0.0, 0.0],
            quaternion_wxyz=[0.7071067812, 0.7071067812, 0.0, 0.0],
        ),
        _actual([0.0, 0.0, 0.0]),
        q_current,
    )

    assert output.command_deadman is True
    assert output.palm_target_quaternion_wxyz == pytest.approx(
        [0.7071067812, 0.0, -0.7071067812, 0.0]
    )


def test_mounted_device_orientation_uses_phone_to_robot_world_axis_map() -> None:
    follower = RelativePoseLagFollower(
        _config(
            freeze_when_phone_still=False,
            orientation_control_enabled=True,
            orientation_mapping_mode="mounted_device",
            orientation_scale=1.0,
            phone_to_robot_axis_map={
                "x": {"source": "y", "sign": 1.0, "scale": 1.0},
                "y": {"source": "x", "sign": -1.0, "scale": 1.0},
                "z": {"source": "z", "sign": 1.0, "scale": 1.0},
            },
            phone_jump_reject_rotation_rad=10.0,
            max_rot_tracking_error_pause_rad=10.0,
        )
    )
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    output = follower.step(
        _snapshot(
            0.1,
            [0.0, 0.0, 0.0],
            quaternion_wxyz=[0.7071067812, 0.7071067812, 0.0, 0.0],
        ),
        _actual([0.0, 0.0, 0.0]),
        q_current,
    )

    assert output.command_deadman is True
    assert output.palm_target_quaternion_wxyz == pytest.approx(
        [0.7071067812, 0.0, -0.7071067812, 0.0]
    )


def test_mounted_device_orientation_honors_world_to_phone_convention() -> None:
    follower = RelativePoseLagFollower(
        _config(
            freeze_when_phone_still=False,
            orientation_control_enabled=True,
            orientation_mapping_mode="mounted_device",
            phone_quaternion_convention="world-to-phone",
            orientation_scale=1.0,
            phone_jump_reject_rotation_rad=10.0,
            max_rot_tracking_error_pause_rad=10.0,
        )
    )
    q_current = [0.0] * 6
    phone_to_world = [0.7071067812, 0.7071067812, 0.0, 0.0]
    world_to_phone = quat_conjugate_wxyz(phone_to_world).astype(float).tolist()

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    output = follower.step(
        _snapshot(0.1, [0.0, 0.0, 0.0], quaternion_wxyz=world_to_phone),
        _actual([0.0, 0.0, 0.0]),
        q_current,
    )

    assert output.command_deadman is True
    assert output.palm_target_quaternion_wxyz == pytest.approx(phone_to_world)


def test_mounted_device_orientation_preserves_robot_phone_mount_offset() -> None:
    follower = RelativePoseLagFollower(
        _config(
            freeze_when_phone_still=False,
            orientation_control_enabled=True,
            orientation_mapping_mode="mounted_device",
            orientation_scale=1.0,
            phone_jump_reject_rotation_rad=10.0,
            max_rot_tracking_error_pause_rad=10.0,
        )
    )
    q_current = [0.0] * 6
    robot_anchor = [0.7071067812, 0.0, 0.0, 0.7071067812]

    follower.step(
        _snapshot(0.0, [0.0, 0.0, 0.0]),
        _actual_pose([0.0, 0.0, 0.0], robot_anchor),
        q_current,
    )
    output = follower.step(
        _snapshot(
            0.1,
            [0.0, 0.0, 0.0],
            quaternion_wxyz=[0.7071067812, 0.7071067812, 0.0, 0.0],
        ),
        _actual_pose([0.0, 0.0, 0.0], robot_anchor),
        q_current,
    )

    assert output.command_deadman is True
    assert rotate_vector_wxyz(output.palm_target_quaternion_wxyz, [0.0, 0.0, 1.0]) == pytest.approx(
        [1.0, 0.0, 0.0]
    )


def test_phone_back_camera_orientation_maps_to_palm_normal() -> None:
    follower = RelativePoseLagFollower(
        _config(
            freeze_when_phone_still=False,
            orientation_control_enabled=True,
            orientation_mapping_mode="phone_back_camera",
            phone_back_camera_axis=(0.0, 0.0, -1.0),
            phone_jump_reject_rotation_rad=10.0,
            max_rot_tracking_error_pause_rad=10.0,
        )
    )
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    output = follower.step(_snapshot(0.1, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)

    assert output.command_deadman is True
    assert output.palm_target_quaternion_wxyz == pytest.approx(
        [0.7071067812, -0.7071067812, 0.0, 0.0]
    )
    assert rotate_vector_wxyz(output.palm_target_quaternion_wxyz, [0.0, 1.0, 0.0]) == pytest.approx(
        [0.0, 0.0, -1.0]
    )


def test_phone_back_camera_orientation_uses_configured_phone_axis() -> None:
    follower = RelativePoseLagFollower(
        _config(
            freeze_when_phone_still=False,
            orientation_control_enabled=True,
            orientation_mapping_mode="phone_back_camera",
            phone_back_camera_axis=(0.0, 1.0, 0.0),
            phone_jump_reject_rotation_rad=10.0,
            max_rot_tracking_error_pause_rad=10.0,
        )
    )
    q_current = [0.0] * 6

    follower.step(_snapshot(0.0, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)
    output = follower.step(_snapshot(0.1, [0.0, 0.0, 0.0]), _actual([0.0, 0.0, 0.0]), q_current)

    assert output.command_deadman is True
    assert rotate_vector_wxyz(output.palm_target_quaternion_wxyz, [0.0, 1.0, 0.0]) == pytest.approx(
        [0.0, 1.0, 0.0]
    )
