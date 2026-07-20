from __future__ import annotations

import math
from typing import Any, Mapping

from teleop_tools.relative_pose_lag_follow import RelativePoseLagFollowConfig


def relative_pose_config_from_mapping(
    config: Mapping[str, Any],
    *,
    phone_to_robot_rotation_matrix: Any | None = None,
) -> RelativePoseLagFollowConfig:
    relative_cfg = config.get("relative_pose_lag_follow", {})
    direction_cfg = config.get("direction_calibration", {})
    return RelativePoseLagFollowConfig(
        target_response_mode=str(relative_cfg.get("target_response_mode", "direct")),
        position_scale=float(relative_cfg.get("position_scale", 1.0)),
        max_step_position_m=float(relative_cfg.get("max_step_position_m", 0.01)),
        max_step_rotation_rad=float(
            relative_cfg.get(
                "max_step_rotation_rad",
                math.radians(float(relative_cfg.get("max_step_rotation_deg", 2.0))),
            )
        ),
        max_target_lead_m=float(relative_cfg.get("max_target_lead_m", 0.08)),
        workspace_min_m=tuple(float(v) for v in relative_cfg.get("workspace_min_m", [-1.0] * 3)),
        workspace_max_m=tuple(float(v) for v in relative_cfg.get("workspace_max_m", [1.0] * 3)),
        max_pos_tracking_error_warn_m=float(relative_cfg.get("max_pos_tracking_error_warn_m", 0.03)),
        max_pos_tracking_error_pause_m=float(relative_cfg.get("max_pos_tracking_error_pause_m", 0.08)),
        max_rot_tracking_error_warn_rad=float(
            relative_cfg.get(
                "max_rot_tracking_error_warn_rad",
                math.radians(float(relative_cfg.get("max_rot_tracking_error_warn_deg", 8.0))),
            )
        ),
        max_rot_tracking_error_pause_rad=float(
            relative_cfg.get(
                "max_rot_tracking_error_pause_rad",
                math.radians(float(relative_cfg.get("max_rot_tracking_error_pause_deg", 20.0))),
            )
        ),
        max_q_tracking_error_pause_rad=float(relative_cfg.get("max_q_tracking_error_pause_rad", 0.25)),
        min_warn_time_scale=float(relative_cfg.get("min_warn_time_scale", 0.25)),
        phone_translation_deadband_m=float(relative_cfg.get("phone_translation_deadband_m", 0.003)),
        phone_rotation_deadband_rad=float(
            relative_cfg.get(
                "phone_rotation_deadband_rad",
                math.radians(float(relative_cfg.get("phone_rotation_deadband_deg", 1.0))),
            )
        ),
        phone_jump_reject_translation_m=float(relative_cfg.get("phone_jump_reject_translation_m", 0.25)),
        phone_jump_reject_rotation_rad=float(
            relative_cfg.get(
                "phone_jump_reject_rotation_rad",
                math.radians(float(relative_cfg.get("phone_jump_reject_rotation_deg", 45.0))),
            )
        ),
        phone_still_translation_m=float(relative_cfg.get("phone_still_translation_m", 0.002)),
        phone_still_rotation_rad=float(
            relative_cfg.get(
                "phone_still_rotation_rad",
                math.radians(float(relative_cfg.get("phone_still_rotation_deg", 0.5))),
            )
        ),
        phone_still_min_sec=float(relative_cfg.get("phone_still_min_sec", 0.0)),
        phone_still_freeze_tracking_error_m=float(relative_cfg.get("phone_still_freeze_tracking_error_m", 0.03)),
        freeze_when_phone_still=bool(relative_cfg.get("freeze_when_phone_still", True)),
        target_filter_time_constant_sec=float(relative_cfg.get("target_filter_time_constant_sec", 0.10)),
        max_target_velocity_m_s=float(relative_cfg.get("max_target_velocity_m_s", 0.02)),
        max_target_acceleration_m_s2=float(relative_cfg.get("max_target_acceleration_m_s2", 0.0)),
        max_target_jump_m=float(relative_cfg.get("max_target_jump_m", 0.05)),
        target_update_deadband_m=float(relative_cfg.get("target_update_deadband_m", 0.0)),
        target_update_release_m=float(relative_cfg.get("target_update_release_m", 0.0)),
        reanchor_requires_deadman_release=bool(relative_cfg.get("reanchor_requires_deadman_release", False)),
        orientation_control_enabled=bool(relative_cfg.get("orientation_control_enabled", False)),
        orientation_mapping_mode=str(relative_cfg.get("orientation_mapping_mode", "relative")),
        phone_back_camera_axis=tuple(float(v) for v in relative_cfg.get("phone_back_camera_axis", [0.0, 0.0, -1.0])),
        phone_quaternion_convention=str(relative_cfg.get("phone_quaternion_convention", "body-to-world")),
        orientation_scale=float(relative_cfg.get("orientation_scale", 1.0)),
        phone_to_robot_orientation_axis_map=direction_cfg.get("phone_to_robot_orientation"),
        phone_to_robot_axis_map=direction_cfg.get("phone_to_robot"),
        phone_to_robot_rotation_matrix=phone_to_robot_rotation_matrix,
    )
