from __future__ import annotations

import pytest

from teleop_tools.hebi_mobile_io import HebiMobileIOSnapshot
from teleop_tools.relative_pose_lag_follow import RelativePoseLagFollower, RelativePoseLagFollowConfig, TcpPose


IDENTITY_AXIS_MAP = {
    "x": {"source": "x", "sign": 1.0, "scale": 1.0},
    "y": {"source": "y", "sign": 1.0, "scale": 1.0},
    "z": {"source": "z", "sign": 1.0, "scale": 1.0},
}


def _snapshot(timestamp: float, position: list[float], *, b1: bool = True) -> HebiMobileIOSnapshot:
    return HebiMobileIOSnapshot(
        timestamp_sec=timestamp,
        position_m=position,
        quaternion_wxyz=[1.0, 0.0, 0.0, 0.0],
        raw_inputs={"b1": b1},
    )


def _actual(position: list[float]) -> TcpPose:
    return TcpPose(position, [1.0, 0.0, 0.0, 0.0])


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
