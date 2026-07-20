from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml

from .processing.pose_validator import PoseValidationConfig
from .processing.target_shaper import CartesianMotionLimits
from .supervision import JointSafetyLimits
from .transforms.frame_mapping import CentralFrameMapping


CONFIG_SCHEMA = "teledex_jaka_bounded_arm.v1"


@dataclass(frozen=True, slots=True)
class BoundedTeleopConfig:
    frames: CentralFrameMapping
    validation: PoseValidationConfig
    cartesian_limits: CartesianMotionLimits
    joint_limits: JointSafetyLimits
    translation_scale: float
    rotation_scale: float
    maximum_session_ns: int
    calibration_confirmed_for_shadow: bool
    calibration_confirmed_for_motion: bool
    source_semantics_confirmed: bool
    content_sha256: str

    @property
    def motion_authorized_by_configuration(self) -> bool:
        return self.calibration_confirmed_for_motion and self.source_semantics_confirmed


def _triple(payload: Any, name: str) -> tuple[float, float, float]:
    values = tuple(float(value) for value in payload)
    if len(values) != 3:
        raise ValueError(f"{name} must contain three values")
    return values  # type: ignore[return-value]


def load_bounded_teleop_config(path: str | Path) -> BoundedTeleopConfig:
    raw = Path(path).read_bytes()
    payload = yaml.safe_load(raw)
    if not isinstance(payload, Mapping) or payload.get("schema_version") != CONFIG_SCHEMA:
        raise ValueError("unsupported bounded TeleDex configuration schema")
    frame_mapping = CentralFrameMapping.from_mapping(payload["frames"])
    scale = payload["mapping"]
    stale = payload["input"]["stale"]
    discontinuity = payload["input"]["discontinuity"]
    cartesian = payload["cartesian_limits"]
    joint = payload["joint_limits"]
    calibration = payload["calibration"]
    config = BoundedTeleopConfig(
        frames=frame_mapping,
        validation=PoseValidationConfig(
            expected_frame_id=frame_mapping.source_frame_id,
            warning_age_ns=int(float(stale["warning_ms"]) * 1e6),
            hold_age_ns=int(float(stale["hold_ms"]) * 1e6),
            controlled_stop_age_ns=int(float(stale["controlled_stop_ms"]) * 1e6),
            fatal_age_ns=int(float(stale["fatal_ms"]) * 1e6),
            maximum_translation_jump_m=float(discontinuity["translation_m"]),
            maximum_rotation_jump_rad=float(discontinuity["rotation_rad"]),
            maximum_linear_speed_m_s=float(discontinuity["linear_speed_m_s"]),
            maximum_angular_speed_rad_s=float(discontinuity["angular_speed_rad_s"]),
        ),
        cartesian_limits=CartesianMotionLimits(
            workspace_half_extent_m=_triple(cartesian["workspace_half_extent_m"], "workspace"),
            maximum_orientation_deviation_rad=float(cartesian["orientation_deviation_rad"]),
            maximum_linear_speed_m_s=float(cartesian["linear_speed_m_s"]),
            maximum_linear_acceleration_m_s2=float(cartesian["linear_acceleration_m_s2"]),
            maximum_linear_jerk_m_s3=float(cartesian["linear_jerk_m_s3"]),
            maximum_angular_speed_rad_s=float(cartesian["angular_speed_rad_s"]),
            maximum_angular_acceleration_rad_s2=float(cartesian["angular_acceleration_rad_s2"]),
            maximum_angular_jerk_rad_s3=float(cartesian["angular_jerk_rad_s3"]),
            maximum_step_dt_s=float(cartesian["maximum_step_dt_s"]),
        ),
        joint_limits=JointSafetyLimits(
            lower_rad=tuple(float(value) for value in joint["lower_rad"]),
            upper_rad=tuple(float(value) for value in joint["upper_rad"]),
            soft_margin_rad=float(joint["soft_margin_rad"]),
            maximum_velocity_rad_s=float(joint["velocity_rad_s"]),
            maximum_acceleration_rad_s2=float(joint["acceleration_rad_s2"]),
            maximum_jerk_rad_s3=float(joint["jerk_rad_s3"]),
        ),
        translation_scale=float(scale["translation_scale"]),
        rotation_scale=float(scale["rotation_scale"]),
        maximum_session_ns=int(float(payload["session"]["maximum_duration_s"]) * 1e9),
        calibration_confirmed_for_shadow=bool(calibration["confirmed_for_shadow"]),
        calibration_confirmed_for_motion=bool(calibration["confirmed_for_motion"]),
        source_semantics_confirmed=bool(calibration["source_semantics_confirmed"]),
        content_sha256=hashlib.sha256(raw).hexdigest(),
    )
    if not 0.0 <= config.translation_scale <= 0.10:
        raise ValueError("translation scale exceeds first-test limit")
    if not 0.0 <= config.rotation_scale <= 0.10:
        raise ValueError("rotation scale exceeds first-test limit")
    if config.maximum_session_ns > 10_000_000_000:
        raise ValueError("first-test session may not exceed 10 seconds")
    return config
