"""Shared Quest-to-JAKA kinematic target generation plus optional MuJoCo plant."""

from __future__ import annotations

from collections import Counter
import copy
from dataclasses import asdict, dataclass, field
from enum import Enum
import math
from pathlib import Path
import time
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from embodiment_core.config import load_yaml
from embodiment_core.robot_limits import (
    PERIODIC_JOINT_INDICES,
    select_nearest_equivalent_joint_branch,
    shortest_equivalent_delta_rad,
)
from jaka_driver_adapter.palm_target_ik import (
    MJCF_ARM_JOINT_NAMES,
    PalmTargetIkState,
    joint_limit_margin_blockers,
    safe_joint_limits_rad,
)
from motion_input import (
    HtsCanonicalAssembler,
    OperatorInputState,
    Pose6D,
    RightHandOperatorConfig,
    RightHandOperatorPipeline,
    SerializationError,
    parse_hts_datagram,
)
from motion_input.hts_transport import ReceivedHtsDatagram
from teleoperation.output_feasibility import (
    JointOutputContractConfig,
    JointOutputFeasibility,
    JointOutputFeasibilityTracker,
    PROJECT_DEFAULT_OUTPUT_JERK_LIMIT_RAD_S3,
)

from .mapping import MappingRejection, ProvisionalMappingConfig, ProvisionalOperatorToRobotMapper
from .se3 import (
    quaternion_angle_rad,
    quaternion_to_matrix,
    quaternion_to_rotvec,
    relative_pose,
    swing_twist_about_local_z,
)


DESIRED_MARKER_BODY = "quest_jaka_desired_tcp_marker"
ACTUAL_MARKER_BODY = "quest_jaka_actual_tcp_marker"
PROJECT_DEFAULT_STARTUP_TIMING_GRACE_CYCLES = 25


class FeasibilityReason(str, Enum):
    ACCEPTED = "ACCEPTED"
    INPUT_INVALID = "INPUT_INVALID"
    DISENGAGED = "DISENGAGED"
    OUTSIDE_OPERATOR_ENVELOPE = "OUTSIDE_OPERATOR_ENVELOPE"
    TARGET_JUMP = "TARGET_JUMP"
    OUTSIDE_ROBOT_WORKSPACE = "OUTSIDE_ROBOT_WORKSPACE"
    IK_POSITION_FAILED = "IK_POSITION_FAILED"
    IK_ORIENTATION_FAILED = "IK_ORIENTATION_FAILED"
    IK_DISCONTINUITY = "IK_DISCONTINUITY"
    OUTPUT_VELOCITY_INFEASIBLE = "OUTPUT_VELOCITY_INFEASIBLE"
    OUTPUT_ACCELERATION_INFEASIBLE = "OUTPUT_ACCELERATION_INFEASIBLE"
    CONTROL_COMPUTE_BUDGET_EXHAUSTED = "CONTROL_COMPUTE_BUDGET_EXHAUSTED"
    JOINT_LIMIT = "JOINT_LIMIT"
    NEAR_SINGULARITY = "NEAR_SINGULARITY"
    SINGULARITY_SLOWDOWN = "SINGULARITY_SLOWDOWN"
    LINEAR_VELOCITY_LIMIT = "LINEAR_VELOCITY_LIMIT"
    ANGULAR_VELOCITY_LIMIT = "ANGULAR_VELOCITY_LIMIT"
    LINEAR_ACCELERATION_LIMIT = "LINEAR_ACCELERATION_LIMIT"
    ANGULAR_ACCELERATION_LIMIT = "ANGULAR_ACCELERATION_LIMIT"
    SELF_COLLISION = "SELF_COLLISION"
    ENVIRONMENT_COLLISION = "ENVIRONMENT_COLLISION"
    JOINT_BRANCH_DISCONTINUITY = "JOINT_BRANCH_DISCONTINUITY"
    EPISODE_WINDING_EXCEEDED = "EPISODE_WINDING_EXCEEDED"


@dataclass(frozen=True, slots=True)
class FeasibilityLimits:
    ik_position_tolerance_m: float
    maximum_jacobian_condition: float
    minimum_jacobian_singular_value: float
    maximum_target_jump_m: float
    maximum_tcp_velocity_m_s: float
    maximum_tcp_angular_velocity_rad_s: float
    maximum_joint_velocity_rad_s: float
    maximum_joint_acceleration_rad_s2: float
    joint_limit_margin_rad: float
    maximum_target_displacement_m: float
    ik_orientation_tolerance_rad: float = math.pi
    maximum_target_rotation_jump_rad: float = math.pi
    maximum_joint_target_jump_rad: float = math.pi
    wrist_proximity_warning_rad: float = 0.0
    jacobian_slowdown_condition: float = math.inf
    minimum_singular_value_slowdown: float = 0.0
    jacobian_recovery_condition: float = math.inf
    minimum_singular_value_recovery: float = 0.0
    singularity_direction_hysteresis_ratio: float = 0.01

    def __post_init__(self) -> None:
        if not (
            math.isfinite(self.maximum_jacobian_condition)
            and self.maximum_jacobian_condition > 0.0
            and math.isfinite(self.minimum_jacobian_singular_value)
            and self.minimum_jacobian_singular_value >= 0.0
            and math.isfinite(self.wrist_proximity_warning_rad)
            and self.wrist_proximity_warning_rad >= 0.0
            and math.isfinite(self.singularity_direction_hysteresis_ratio)
            and self.singularity_direction_hysteresis_ratio >= 0.0
        ):
            raise ValueError("singularity feasibility thresholds must be finite and non-negative")
        if math.isfinite(self.jacobian_slowdown_condition) and not (
            0.0 < self.jacobian_recovery_condition
            < self.jacobian_slowdown_condition
            < self.maximum_jacobian_condition
        ):
            raise ValueError(
                "Jacobian condition thresholds require recovery < slowdown < hard"
            )
        if self.minimum_singular_value_slowdown > 0.0 and not (
            self.minimum_jacobian_singular_value
            < self.minimum_singular_value_slowdown
            < self.minimum_singular_value_recovery
        ):
            raise ValueError(
                "minimum-singular-value thresholds require hard < slowdown < recovery"
            )

    @classmethod
    def from_mapping(
        cls, values: Mapping[str, Any], *, maximum_target_displacement_m: float
    ) -> "FeasibilityLimits":
        return cls(
            ik_position_tolerance_m=float(values["ik_position_tolerance_m"]),
            maximum_jacobian_condition=float(values["maximum_jacobian_condition"]),
            minimum_jacobian_singular_value=float(values["minimum_jacobian_singular_value"]),
            maximum_target_jump_m=float(values["maximum_target_jump_m"]),
            maximum_tcp_velocity_m_s=float(values["maximum_tcp_velocity_m_s"]),
            maximum_tcp_angular_velocity_rad_s=float(
                values["maximum_tcp_angular_velocity_rad_s"]
            ),
            maximum_joint_velocity_rad_s=float(
                values["maximum_ik_target_velocity_rad_s"]
                if "maximum_ik_target_velocity_rad_s" in values
                else values["maximum_joint_velocity_rad_s"]
            ),
            maximum_joint_acceleration_rad_s2=float(
                values["maximum_ik_target_acceleration_rad_s2"]
                if "maximum_ik_target_acceleration_rad_s2" in values
                else values["maximum_joint_acceleration_rad_s2"]
            ),
            joint_limit_margin_rad=math.radians(float(values["joint_limit_margin_deg"])),
            maximum_target_displacement_m=maximum_target_displacement_m,
            ik_orientation_tolerance_rad=math.radians(
                float(values.get("ik_orientation_tolerance_deg", 180.0))
            ),
            maximum_target_rotation_jump_rad=math.radians(
                float(values.get("maximum_target_rotation_jump_deg", 180.0))
            ),
            maximum_joint_target_jump_rad=float(
                values.get("maximum_joint_target_jump_rad", math.pi)
            ),
            wrist_proximity_warning_rad=math.radians(
                float(
                    values.get(
                        "wrist_proximity_warning_deg",
                        # Compatibility migration: the old hard-gate setting is
                        # deliberately warning-only under the new policy.
                        values.get("minimum_wrist_bend_deg", 0.0),
                    )
                )
            ),
            jacobian_slowdown_condition=float(
                values.get(
                    "jacobian_slowdown_condition",
                    0.8 * float(values["maximum_jacobian_condition"]),
                )
            ),
            minimum_singular_value_slowdown=float(
                values.get(
                    "minimum_singular_value_slowdown",
                    1.25 * float(values["minimum_jacobian_singular_value"]),
                )
            ),
            jacobian_recovery_condition=float(
                values.get(
                    "jacobian_recovery_condition",
                    0.75 * float(values["maximum_jacobian_condition"]),
                )
            ),
            minimum_singular_value_recovery=float(
                values.get(
                    "minimum_singular_value_recovery",
                    1.35 * float(values["minimum_jacobian_singular_value"]),
                )
            ),
            singularity_direction_hysteresis_ratio=float(
                values.get("singularity_direction_hysteresis_ratio", 0.01)
            ),
        )


@dataclass(frozen=True, slots=True)
class CommandTrajectoryLimits:
    """Limits applied to the joint-position references sent to MuJoCo.

    These are deliberately separate from IK feasibility thresholds: the IK target
    may move ahead of the simulated mechanism, while ``data.ctrl`` must remain a
    physically plausible, jerk-limited trajectory.
    """

    maximum_velocity_rad_s: float
    maximum_acceleration_rad_s2: float
    maximum_jerk_rad_s3: float
    position_tracking_frequency_rad_s: float
    maximum_velocity_rad_s_per_joint: tuple[float, ...] | None = None

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "CommandTrajectoryLimits":
        result = cls(
            maximum_velocity_rad_s=float(
                values.get("command_maximum_joint_velocity_rad_s", math.pi)
            ),
            maximum_acceleration_rad_s2=float(
                values.get("command_maximum_joint_acceleration_rad_s2", 4.0 * math.pi)
            ),
            maximum_jerk_rad_s3=float(
                values.get(
                    "command_maximum_joint_jerk_rad_s3",
                    PROJECT_DEFAULT_OUTPUT_JERK_LIMIT_RAD_S3,
                )
            ),
            position_tracking_frequency_rad_s=float(
                values.get("command_position_tracking_frequency_rad_s", 10.0)
            ),
            maximum_velocity_rad_s_per_joint=(
                None
                if values.get("command_maximum_joint_velocity_rad_s_per_joint") is None
                else tuple(
                    float(value)
                    for value in values["command_maximum_joint_velocity_rad_s_per_joint"]
                )
            ),
        )
        scalar_values = (
            result.maximum_velocity_rad_s,
            result.maximum_acceleration_rad_s2,
            result.maximum_jerk_rad_s3,
            result.position_tracking_frequency_rad_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in scalar_values):
            raise ValueError("command trajectory limits must be finite and positive")
        if result.maximum_velocity_rad_s_per_joint is not None and (
            len(result.maximum_velocity_rad_s_per_joint) != 6
            or not all(
                math.isfinite(value)
                and 0.0 < value <= result.maximum_velocity_rad_s
                for value in result.maximum_velocity_rad_s_per_joint
            )
        ):
            raise ValueError("per-joint command velocity limits must contain six values")
        return result

    @property
    def velocity_boundaries_rad_s(self) -> tuple[float, ...]:
        return self.maximum_velocity_rad_s_per_joint or (self.maximum_velocity_rad_s,) * 6


@dataclass(frozen=True, slots=True)
class CandidateMetrics:
    target_displacement_m: float = 0.0
    target_jump_m: float = 0.0
    target_rotation_jump_rad: float = 0.0
    tcp_velocity_m_s: float = 0.0
    tcp_angular_velocity_rad_s: float = 0.0
    ik_error_m: float = 0.0
    ik_orientation_error_rad: float = 0.0
    maximum_joint_target_jump_rad: float = 0.0
    joint_limit_blockers: tuple[str, ...] = ()
    jacobian_condition: float = 1.0
    minimum_jacobian_singular_value: float = 1.0
    wrist_bend_from_singularity_rad: float = math.pi
    maximum_joint_velocity_rad_s: float = 0.0
    maximum_joint_acceleration_rad_s2: float = 0.0
    output_feasibility_interval_s: float = 0.0
    output_feasibility_delta_rad: tuple[float, ...] = ()
    predicted_output_joint_velocity_rad_s: tuple[float, ...] = ()
    predicted_output_maximum_joint_velocity_rad_s: float = 0.0
    output_velocity_violating_joint_indices: tuple[int, ...] = ()
    output_velocity_boundary_rad_s_per_joint: tuple[float, ...] = ()
    previous_emitted_output_joint_velocity_rad_s: tuple[float, ...] = ()
    predicted_output_joint_acceleration_rad_s2: tuple[float, ...] = ()
    predicted_output_maximum_joint_acceleration_rad_s2: float = 0.0
    output_acceleration_violating_joint_indices: tuple[int, ...] = ()
    self_collision: bool = False
    environment_collision: bool = False
    minimum_new_contact_distance_m: float | None = None
    target_tool_rotation_vector_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)
    target_tool_swing_rad: float = 0.0
    target_tool_axial_roll_rad: float = 0.0
    ik_seed_rad: tuple[float, ...] = ()
    ik_candidate_rad: tuple[float, ...] = ()
    joint_delta_rad: tuple[float, ...] = ()
    wrist_joint_delta_rad: tuple[float, ...] = ()
    j6_expected_delta_rad: float = 0.0
    j6_axial_contribution_rad: float = 0.0
    j6_axial_contribution_ratio: float | None = None
    nearest_safe_joint_limit_margin_rad: float = math.pi
    branch_switch: bool = False
    current_jacobian_condition: float = 1.0
    current_minimum_jacobian_singular_value: float = 1.0
    singularity_risk: float = 0.0
    current_singularity_risk: float = 0.0
    singularity_direction: str = "TANGENT"
    singularity_state: str = "NORMAL"
    wrist_proximity_warning: bool = False
    effective_ik_damping: float = 0.0
    current_hard_singularity: bool = False
    hard_stop_required: bool = False
    branch_reference_rad: tuple[float, ...] = ()
    branch_delta_rad: tuple[float, ...] = ()
    branch_equivalent_offset: tuple[int, ...] = ()
    episode_winding_rad: tuple[float, ...] = ()


@dataclass(frozen=True, slots=True)
class CandidateComputationTiming:
    seed_fk_ms: float = 0.0
    pre_ik_jacobian_ms: float = 0.0
    ik_iterations_ms: float = 0.0
    ik_final_fk_ms: float = 0.0
    post_ik_jacobian_ms: float = 0.0
    workspace_check_ms: float = 0.0
    collision_check_ms: float = 0.0
    output_feasibility_ms: float = 0.0
    remaining_checks_ms: float = 0.0
    total_ms: float = 0.0
    ik_iterations_completed: int = 0


def classify_candidate(
    metrics: CandidateMetrics,
    limits: FeasibilityLimits,
    *,
    check_singularity: bool = True,
) -> FeasibilityReason:
    if metrics.target_displacement_m > limits.maximum_target_displacement_m:
        return FeasibilityReason.OUTSIDE_ROBOT_WORKSPACE
    near_singularity = (
        metrics.jacobian_condition > limits.maximum_jacobian_condition
        or metrics.minimum_jacobian_singular_value
        < limits.minimum_jacobian_singular_value
    )
    # Geometry is unsafe independently of how slowly it was reached.  The old
    # velocity-qualified gate allowed a gradual trajectory to pass condition
    # 60, drive J3 through zero, and reach a measured condition number above
    # one million.  Candidate velocity remains a separate continuity check.
    if check_singularity and near_singularity:
        return FeasibilityReason.NEAR_SINGULARITY
    if (
        metrics.target_jump_m > limits.maximum_target_jump_m
        or metrics.target_rotation_jump_rad > limits.maximum_target_rotation_jump_rad
        or metrics.maximum_joint_target_jump_rad > limits.maximum_joint_target_jump_rad
    ):
        return FeasibilityReason.TARGET_JUMP
    if metrics.tcp_velocity_m_s > limits.maximum_tcp_velocity_m_s:
        return FeasibilityReason.LINEAR_VELOCITY_LIMIT
    if metrics.tcp_angular_velocity_rad_s > limits.maximum_tcp_angular_velocity_rad_s:
        return FeasibilityReason.ANGULAR_VELOCITY_LIMIT
    if metrics.maximum_joint_velocity_rad_s > limits.maximum_joint_velocity_rad_s:
        return FeasibilityReason.IK_DISCONTINUITY
    if metrics.output_velocity_violating_joint_indices:
        return FeasibilityReason.OUTPUT_VELOCITY_INFEASIBLE
    if metrics.output_acceleration_violating_joint_indices:
        return FeasibilityReason.OUTPUT_ACCELERATION_INFEASIBLE
    if metrics.maximum_joint_acceleration_rad_s2 > limits.maximum_joint_acceleration_rad_s2:
        return FeasibilityReason.LINEAR_ACCELERATION_LIMIT
    if metrics.ik_error_m > limits.ik_position_tolerance_m:
        return FeasibilityReason.IK_POSITION_FAILED
    if metrics.ik_orientation_error_rad > limits.ik_orientation_tolerance_rad:
        return FeasibilityReason.IK_ORIENTATION_FAILED
    if metrics.joint_limit_blockers:
        return FeasibilityReason.JOINT_LIMIT
    if metrics.self_collision:
        return FeasibilityReason.SELF_COLLISION
    if metrics.environment_collision:
        return FeasibilityReason.ENVIRONMENT_COLLISION
    return FeasibilityReason.ACCEPTED


def _singularity_risk(
    condition: float, sigma_min: float, limits: FeasibilityLimits
) -> float:
    """Dimensionless Jacobian risk; one is the committed hard boundary."""

    return max(
        float(condition) / limits.maximum_jacobian_condition,
        limits.minimum_jacobian_singular_value / max(float(sigma_min), 1e-12),
    )


def _hard_singularity(
    condition: float, sigma_min: float, limits: FeasibilityLimits
) -> bool:
    return (
        condition > limits.maximum_jacobian_condition
        or sigma_min < limits.minimum_jacobian_singular_value
    )


def _slowdown_singularity(
    condition: float, sigma_min: float, limits: FeasibilityLimits
) -> bool:
    return (
        condition >= limits.jacobian_slowdown_condition
        or sigma_min <= limits.minimum_singular_value_slowdown
    )


@dataclass(frozen=True, slots=True)
class FeasibilityResult:
    accepted: bool
    reason: FeasibilityReason
    joint_target_rad: tuple[float, ...] | None
    metrics: CandidateMetrics
    timing: CandidateComputationTiming = field(
        default_factory=CandidateComputationTiming
    )


def jerk_limited_position_step(
    position: np.ndarray,
    target: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    *,
    dt_s: float,
    limits: CommandTrajectoryLimits,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Advance a critically damped command trajectory by one fixed-rate step.

    This shapes the actuator *set-point*; MuJoCo's position servo then follows
    that set-point.  Position, velocity and acceleration stay continuous and the
    finite-difference jerk is bounded even when an IK target changes abruptly.
    """

    dt = float(dt_s)
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("command trajectory dt_s must be finite and positive")
    q = np.asarray(position, dtype=float)
    q_target = np.asarray(target, dtype=float)
    qd = np.asarray(velocity, dtype=float)
    qdd = np.asarray(acceleration, dtype=float)
    if not (q.shape == q_target.shape == qd.shape == qdd.shape):
        raise ValueError("command trajectory arrays must have identical shapes")
    if not all(np.all(np.isfinite(value)) for value in (q, q_target, qd, qdd)):
        raise ValueError("command trajectory arrays must be finite")

    # Critically damped third-order reference model:
    #   (D + omega)^3 q = omega^3 q_target
    # Its state is (position, velocity, acceleration), so limiting the commanded
    # third derivative bounds jerk without discontinuously resetting velocity or
    # acceleration at the target.
    omega = limits.position_tracking_frequency_rad_s
    desired_jerk = (
        omega**3 * (q_target - q)
        - 3.0 * omega**2 * qd
        - 3.0 * omega * qdd
    )
    bounded_jerk = np.clip(
        desired_jerk,
        -limits.maximum_jerk_rad_s3,
        limits.maximum_jerk_rad_s3,
    )
    next_acceleration = np.clip(
        qdd + bounded_jerk * dt,
        -limits.maximum_acceleration_rad_s2,
        limits.maximum_acceleration_rad_s2,
    )
    next_velocity = np.clip(
        qd + next_acceleration * dt,
        -np.asarray(limits.velocity_boundaries_rad_s, dtype=float),
        np.asarray(limits.velocity_boundaries_rad_s, dtype=float),
    )
    next_position = q + next_velocity * dt
    return next_position, next_velocity, next_acceleration


@dataclass(frozen=True, slots=True)
class ReplayConfig:
    raw: Mapping[str, Any]
    mapping: ProvisionalMappingConfig
    feasibility: FeasibilityLimits
    command_limits: CommandTrajectoryLimits
    output_contract: JointOutputContractConfig
    stale_after_s: float
    engagement_schedule_s: tuple[float, ...]
    mjcf_path: Path
    initial_arm_joints_rad: tuple[float, ...]
    ik_gain: float
    ik_damping: float
    ik_max_step_rad: float
    ik_iterations: int
    zero_gravity: bool
    axis_analysis: Mapping[str, Any]
    startup_timing_grace_cycles: int
    input_recovery_timeout_s: float

    @classmethod
    def load(cls, path: str | Path, *, speed_profile: str | None = None) -> "ReplayConfig":
        raw = load_yaml(path)
        if speed_profile is not None:
            profiles = {
                "current_live": {
                    # JAKA ServoJ's official outer theoretical/legal boundary;
                    # this is a simulation ceiling, not a calibrated physical
                    # teleoperation work speed.
                    "velocity": (math.pi,) * 6,
                },
                "baseline": {
                    "velocity": (1.5, 1.5, 1.5, 1.2, 1.2, 1.2),
                },
                "moderate": {
                    "velocity": (1.8, 1.8, 1.8, 1.8, 1.8, 1.8),
                },
                "fast": {
                    "velocity": (2.2, 2.2, 2.2, 2.2, 1.8, 2.5),
                },
                "latency_default": {
                    "velocity": (math.pi,) * 6,
                },
                "latency_reduced": {
                    "velocity": (math.pi,) * 6,
                    "interpolation_delay_ms": 8.0,
                    "rotation_filter": {
                        "rotation_min_cutoff": 3.0,
                        "rotation_beta": 6.0,
                        "rotation_derivative_cutoff": 2.0,
                    },
                },
                "latency_raw_diagnostic": {
                    "velocity": (math.pi,) * 6,
                    "interpolation_delay_ms": 0.0,
                    "rotation_filter": {
                        "rotation_min_cutoff": 1_000_000.0,
                        "rotation_beta": 0.0,
                        "rotation_derivative_cutoff": 1.0,
                    },
                },
                "root_cause_fix": {
                    "velocity": (math.pi,) * 6,
                    # Diagnostic-only: evaluate a 60 Hz target replacement
                    # over its own interval.  The hardware/EDG contract stays
                    # at the live 8 ms period.
                    "feasibility_acceleration_hz": 60.0,
                },
                "root_cause_fix_plus_low_latency": {
                    "velocity": (math.pi,) * 6,
                    "feasibility_acceleration_hz": 60.0,
                    "interpolation_delay_ms": 8.0,
                    "rotation_filter": {
                        "rotation_min_cutoff": 3.0,
                        "rotation_beta": 6.0,
                        "rotation_derivative_cutoff": 2.0,
                    },
                },
            }
            try:
                profile = profiles[speed_profile]
            except KeyError as exc:
                raise ValueError(f"unknown simulation speed profile {speed_profile!r}") from exc
            raw = copy.deepcopy(raw)
            if "interpolation_delay_ms" in profile:
                raw.setdefault("rates", {})["interpolation_delay_ms"] = profile[
                    "interpolation_delay_ms"
                ]
            if "jaka_transport_hz" in profile:
                raw.setdefault("rates", {})["jaka_transport_hz"] = profile[
                    "jaka_transport_hz"
                ]
            if "feasibility_acceleration_hz" in profile:
                raw.setdefault("shared_target_generation", {})[
                    "feasibility_acceleration_period_ns"
                ] = int(round(1e9 / float(profile["feasibility_acceleration_hz"])))
            rotation_filter = profile.get("rotation_filter")
            if rotation_filter:
                selected = raw.setdefault("filter", {}).get("selected_profile")
                values = raw.setdefault("filter", {}).setdefault("profiles", {}).setdefault(
                    selected, {}
                )
                values.update(rotation_filter)
            raw.setdefault("shared_target_generation", {})[
                "maximum_output_joint_velocity_rad_s_per_joint"
            ] = list(profile["velocity"])
            raw.setdefault("simulation", {})[
                "command_maximum_joint_velocity_rad_s_per_joint"
            ] = list(profile["velocity"])
        provisional = ProvisionalMappingConfig.from_mapping(raw["provisional_calibration"])
        simulation = raw["simulation"]
        shared_target = raw.get("shared_target_generation", {})
        rates = raw.get("rates", {})
        maximum_output_velocity = float(
            shared_target.get(
                "maximum_output_joint_velocity_rad_s",
                simulation.get("command_maximum_joint_velocity_rad_s", math.pi),
            )
        )
        maximum_output_velocity_per_joint_raw = shared_target.get(
            "maximum_output_joint_velocity_rad_s_per_joint"
        )
        maximum_output_velocity_per_joint = (
            None
            if maximum_output_velocity_per_joint_raw is None
            else tuple(
                float(value) for value in maximum_output_velocity_per_joint_raw
            )
        )
        maximum_output_acceleration = float(
            shared_target.get(
                "maximum_output_joint_acceleration_rad_s2",
                simulation.get("command_maximum_joint_acceleration_rad_s2", 4.0 * math.pi),
            )
        )
        servo_period_ns = int(
            round(1e9 / float(rates.get("jaka_transport_hz", 125.0)))
        )
        command_limits = CommandTrajectoryLimits.from_mapping(simulation)
        startup_timing_grace_cycles = int(
            raw.get("hardware_adapter", {}).get(
                "startup_timing_grace_cycles",
                PROJECT_DEFAULT_STARTUP_TIMING_GRACE_CYCLES,
            )
        )
        if not 1 <= startup_timing_grace_cycles <= 1_000:
            raise ValueError("startup_timing_grace_cycles must be between 1 and 1000")
        input_recovery_timeout_ms = float(
            raw.get("clutches", {}).get("input_recovery_timeout_ms", 0.0)
        )
        if (
            not math.isfinite(input_recovery_timeout_ms)
            or input_recovery_timeout_ms < 0.0
            or input_recovery_timeout_ms > 10_000.0
        ):
            raise ValueError(
                "input_recovery_timeout_ms must be finite and between 0 and 10000"
            )
        return cls(
            raw=raw,
            mapping=provisional,
            feasibility=FeasibilityLimits.from_mapping(
                simulation,
                maximum_target_displacement_m=provisional.maximum_target_displacement_m,
            ),
            command_limits=command_limits,
            output_contract=JointOutputContractConfig(
                maximum_velocity_rad_s=maximum_output_velocity,
                servo_period_ns=servo_period_ns,
                maximum_acceleration_rad_s2=maximum_output_acceleration,
                maximum_jerk_rad_s3=command_limits.maximum_jerk_rad_s3,
                maximum_velocity_rad_s_per_joint=(
                    maximum_output_velocity_per_joint
                ),
                feasibility_acceleration_period_ns=(
                    None
                    if shared_target.get("feasibility_acceleration_period_ns") is None
                    else int(shared_target["feasibility_acceleration_period_ns"])
                ),
            ),
            stale_after_s=float(raw["input"]["stale_after_ms"]) / 1000.0,
            engagement_schedule_s=tuple(
                float(value) for value in raw["input"]["engagement_schedule_s"]
            ),
            mjcf_path=Path(simulation["mjcf_path"]),
            initial_arm_joints_rad=tuple(
                float(value) for value in simulation["initial_arm_joints_rad"]
            ),
            ik_gain=float(simulation["ik_gain"]),
            ik_damping=float(simulation["ik_damping"]),
            ik_max_step_rad=float(simulation["ik_max_step_rad"]),
            ik_iterations=int(simulation["ik_iterations"]),
            zero_gravity=bool(simulation.get("zero_gravity", True)),
            axis_analysis=raw.get("axis_analysis", {}),
            startup_timing_grace_cycles=startup_timing_grace_cycles,
            input_recovery_timeout_s=input_recovery_timeout_ms / 1000.0,
        )


def build_viewer_mjcf(
    base_path: str | Path,
    output_path: str | Path,
    *,
    arm_only: bool = False,
    scene: Mapping[str, Any] | None = None,
) -> Path:
    base = Path(base_path).resolve()
    output = Path(output_path).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(base)
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("meshdir", str(base.parent))
    if arm_only:
        actuator = root.find("actuator")
        if actuator is not None:
            for child in list(actuator):
                if child.get("name", "").startswith("rh56_"):
                    actuator.remove(child)
    world = root.find("worldbody")
    if world is None:
        raise RuntimeError("MuJoCo model has no worldbody")
    if scene is not None:
        _add_workspace_scene(world, scene)
    # The committed mesh pair Link_0/Link_1 starts with four duplicate ~3 mm
    # penetrations at their shared physical joint.  It is already treated as a
    # baseline-allowed contact by feasibility checks, but leaving contact
    # response active creates artificial joint-1 stiction in the zero-gravity
    # viewer plant.  Exclude only this adjacent pair in the generated model;
    # the source asset and every non-baseline collision pair remain unchanged.
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    if not any(
        child.tag == "exclude"
        and {child.get("body1"), child.get("body2")} == {"jaka_Link_0", "jaka_Link_1"}
        for child in contact
    ):
        ET.SubElement(
            contact,
            "exclude",
            {"body1": "jaka_Link_0", "body2": "jaka_Link_1"},
        )
    for name, size, rgba in (
        (DESIRED_MARKER_BODY, "0.018", "0.05 0.35 1.0 0.90"),
        (ACTUAL_MARKER_BODY, "0.013", "0.05 0.95 0.25 0.90"),
    ):
        body = ET.SubElement(world, "body", {"name": name, "mocap": "true"})
        ET.SubElement(
            body,
            "geom",
            {
                "name": f"{name}_sphere",
                "type": "sphere",
                "size": size,
                "rgba": rgba,
                "contype": "0",
                "conaffinity": "0",
                "group": "5",
            },
        )
        # RGB cylinders expose the complete desired/actual TCP orientation.
        for axis, position, axis_rgba in (
            ("x", "0.04 0 0", "1 0.1 0.1 0.95"),
            ("y", "0 0.04 0", "0.1 1 0.1 0.95"),
            ("z", "0 0 0.04", "0.1 0.3 1 0.95"),
        ):
            attributes = {
                "name": f"{name}_{axis}_axis",
                "type": "capsule",
                "size": "0.003 0.04",
                "pos": position,
                "rgba": axis_rgba,
                "contype": "0",
                "conaffinity": "0",
                "group": "5",
            }
            if axis == "x":
                attributes["quat"] = "0.70710678 0 0.70710678 0"
            elif axis == "y":
                attributes["quat"] = "0.70710678 -0.70710678 0 0"
            ET.SubElement(body, "geom", attributes)
    tree.write(output, encoding="utf-8")
    return output


def _add_workspace_scene(world: ET.Element, scene: Mapping[str, Any]) -> None:
    workspace = load_yaml(scene["workspace_config_path"])
    table = workspace["tabletop"]
    table_size = np.asarray(table["size_xyz_m"], dtype=float)
    table_center_source = np.asarray(table["center_xyz_m"], dtype=float)
    source_yaw = math.radians(float(scene["source_frame_to_robot_base_yaw_deg"]))
    base_yaw = math.radians(float(scene["robot_base_world_yaw_deg"]))
    base_position = np.asarray(scene["robot_base_world_position_m"], dtype=float)
    source_to_base = _yaw_rotation(source_yaw)
    base_to_world = _yaw_rotation(base_yaw)

    base = next(
        (body for body in world.findall("body") if body.get("name") == "jaka_Link_0"),
        None,
    )
    if base is None:
        raise RuntimeError("MuJoCo model has no top-level jaka_Link_0 body")
    base.set("pos", _vector_text(base_position))
    base.set("quat", _yaw_quaternion_text(base_yaw))

    table_center_world = (
        base_position + base_to_world @ source_to_base @ table_center_source
    )
    table_yaw_world = base_yaw + source_yaw
    ET.SubElement(
        world,
        "geom",
        {
            "name": "quest_jaka_workspace_tabletop",
            "type": "box",
            "pos": _vector_text(table_center_world),
            "quat": _yaw_quaternion_text(table_yaw_world),
            "size": _vector_text(table_size / 2.0),
            "rgba": "0.58 0.35 0.16 1",
            "friction": "1.0 0.01 0.001",
        },
    )
    for member in workspace.get("table_frame", {}).get("identified_members", ()):
        center_source = np.asarray(member["center_P_m"], dtype=float)
        dimensions = np.asarray(member["dimensions_m"], dtype=float)
        center_world = base_position + base_to_world @ source_to_base @ center_source
        ET.SubElement(
            world,
            "geom",
            {
                "name": f"quest_jaka_{member['name']}",
                "type": "box",
                "pos": _vector_text(center_world),
                "quat": _yaw_quaternion_text(table_yaw_world),
                "size": _vector_text(dimensions / 2.0),
                "rgba": "0.72 0.76 0.80 1",
                "friction": "0.8 0.01 0.001",
            },
        )

    table_top_z = table_center_world[2] + table_size[2] / 2.0
    floor = next(
        (geom for geom in world.findall("geom") if geom.get("name") == "floor"),
        None,
    )
    if floor is not None:
        floor.set(
            "pos",
            _vector_text(
                (0.0, 0.0, table_top_z - float(table["height_above_floor_m"]))
            ),
        )
    marker = table_center_world.copy()
    marker[2] = table_top_z + 0.015
    ET.SubElement(
        world,
        "site",
        {
            "name": "quest_jaka_workspace_center",
            "type": "sphere",
            "pos": _vector_text(marker),
            "size": "0.012",
            "rgba": "1 0.75 0.05 0.9",
            "group": "5",
        },
    )
    axes = ET.SubElement(
        world,
        "body",
        {
            "name": "quest_jaka_robot_base_axes",
            "pos": _vector_text(base_position),
            "quat": _yaw_quaternion_text(base_yaw),
        },
    )
    for name, endpoint, rgba in (
        ("x", "0.14 0 0", "1 0 0 1"),
        ("y", "0 0.14 0", "0 1 0 1"),
        ("z", "0 0 0.14", "0 0 1 1"),
    ):
        ET.SubElement(
            axes,
            "site",
            {
                "name": f"quest_jaka_robot_base_{name}",
                "type": "cylinder",
                "fromto": f"0 0 0 {endpoint}",
                "size": "0.0025",
                "rgba": rgba,
                "group": "5",
            },
        )


def _yaw_rotation(yaw_rad: float) -> np.ndarray:
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.asarray(
        ((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0)),
        dtype=float,
    )


def _yaw_quaternion_text(yaw_rad: float) -> str:
    return _vector_text((math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)))


def _vector_text(values: Any) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


class SharedJakaTargetGenerator:
    """Authoritative kinematic IK/acceptance state, independent of either plant.

    The model is used only for deterministic FK, Jacobians, collision queries,
    and continuation IK.  It owns no simulator control state and never steps
    physics.  MuJoCo and JAKA plants consume its accepted joint targets through
    separate adapters.
    """

    def __init__(self, config: ReplayConfig, *, mjcf_path: str | Path | None = None) -> None:
        path = config.mjcf_path if mjcf_path is None else Path(mjcf_path)
        self.config = config
        self.ik = PalmTargetIkState(
            list(config.initial_arm_joints_rad),
            mjcf_path=path,
            ik_gain=config.ik_gain,
            ik_damping=config.ik_damping,
            ik_max_step_rad=config.ik_max_step_rad,
            ik_iterations=config.ik_iterations,
            target_workspace_radius_m=0.0,
            joint_limit_margin_rad=config.feasibility.joint_limit_margin_rad,
            orientation_ik_weight=0.0 if not config.mapping.orientation_enabled else 0.35,
            adaptive_damping_sigma_start=float(
                config.raw.get("simulation", {}).get(
                    "adaptive_damping_sigma_start", 0.0
                )
            ),
            adaptive_damping_sigma_full=float(
                config.raw.get("simulation", {}).get(
                    "adaptive_damping_sigma_full", 0.0
                )
            ),
            adaptive_damping_max=float(
                config.raw.get("simulation", {}).get(
                    "adaptive_damping_max", config.ik_damping
                )
            ),
        )
        self.model = self.ik.model
        simulation_values = config.raw.get("simulation", {})
        self.jacobian_rotation_characteristic_length_m = float(
            simulation_values.get("jacobian_rotation_characteristic_length_m", 0.25)
        )
        if not (
            math.isfinite(self.jacobian_rotation_characteristic_length_m)
            and self.jacobian_rotation_characteristic_length_m > 0.0
        ):
            raise ValueError(
                "jacobian_rotation_characteristic_length_m must be finite and positive"
            )
        self.arm_joint_ids = self.ik.arm_joint_ids.copy()
        self.arm_qpos_ids = self.ik.arm_qpos_ids.copy()
        self.arm_dof_ids = self.ik.arm_dof_ids.copy()
        self.palm_body_id = self.ik.palm_body_id
        self.initial_tcp = self._kinematic_tcp_pose
        self.last_safe_joint_target = np.asarray(config.initial_arm_joints_rad, dtype=np.float64)
        self.last_safe_target = self.initial_tcp
        self.last_safe_joint_velocity = np.zeros(6)
        self.output_feasibility = JointOutputFeasibilityTracker.from_config(
            config.output_contract
        )
        self.output_feasibility.reset(self.last_safe_joint_target)
        self._synthetic_generated_monotonic_ns = 1_000_000_000
        self._require_contact_free_authoritative_state("configured initial state")
        self.accepted_metrics: list[CandidateMetrics] = []
        self._singularity_slowdown_latched = False
        shared_target = config.raw.get("shared_target_generation", {})
        self.maximum_periodic_joint_winding_rad = float(
            shared_target.get("maximum_periodic_joint_winding_rad", 5.0)
        )
        if not (
            math.isfinite(self.maximum_periodic_joint_winding_rad)
            and self.maximum_periodic_joint_winding_rad > 0.0
            and self.maximum_periodic_joint_winding_rad < 2.0 * math.pi
        ):
            raise ValueError(
                "maximum_periodic_joint_winding_rad must be finite, positive, and below 2*pi"
            )
        self._episode_winding_reference_rad = self.last_safe_joint_target.copy()
        self._episode_winding_last_rad = self.last_safe_joint_target.copy()
        self._episode_winding_rad = np.zeros(6, dtype=np.float64)
        self._episode_winding_fault = False

    def _required_id(self, kind: mujoco.mjtObj, name: str) -> int:
        value = mujoco.mj_name2id(self.model, kind, name)
        if value < 0:
            raise KeyError(name)
        return int(value)

    @property
    def _kinematic_tcp_pose(self) -> Pose6D:
        quat_wxyz = self.ik.current_palm_quaternion_wxyz
        return Pose6D(
            tuple(float(value) for value in self.ik.current_palm_position_m),
            (float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3]), float(quat_wxyz[0])),
        )

    @property
    def current_tcp_pose(self) -> Pose6D:
        return self._kinematic_tcp_pose

    @property
    def arm_joints_rad(self) -> np.ndarray:
        return self.ik.arm_joints_rad.copy()

    def synchronize_authoritative_arm_joints(self, joints_rad: list[float]) -> None:
        """Synchronize from an authoritative plant only while disengaged."""

        joints = np.asarray(joints_rad, dtype=np.float64)
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise ValueError("authoritative arm state must contain six finite radians")
        previous_joints = self.ik.arm_joints_rad.copy()
        self.ik.set_arm_joints_rad(joints.tolist())
        try:
            self._require_contact_free_authoritative_state(
                "authoritative synchronization state"
            )
        except ValueError:
            self.ik.set_arm_joints_rad(previous_joints.tolist())
            raise
        current = self._kinematic_tcp_pose
        self.initial_tcp = current
        self.last_safe_target = current
        self.last_safe_joint_target = joints.copy()
        self.last_safe_joint_velocity[:] = 0.0
        self.output_feasibility.reset(joints)
        self._singularity_slowdown_latched = False
        self.reset_episode_winding(joints.tolist())

    def capture_reference(self) -> Pose6D:
        """Capture the current authoritative FK pose and reset derivative history."""

        current = self._kinematic_tcp_pose
        self.initial_tcp = current
        self.last_safe_target = current
        self.last_safe_joint_target = self.arm_joints_rad
        self.last_safe_joint_velocity[:] = 0.0
        self.output_feasibility.reset(self.last_safe_joint_target)
        self._singularity_slowdown_latched = False
        self.reset_episode_winding(self.last_safe_joint_target.tolist())
        return current

    @property
    def episode_winding_rad(self) -> tuple[float, ...]:
        return tuple(float(value) for value in self._episode_winding_rad)

    def reset_episode_winding(self, joints_rad: list[float]) -> None:
        joints = np.asarray(joints_rad, dtype=np.float64)
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise ValueError("episode winding reference must contain six finite radians")
        self._episode_winding_reference_rad = joints.copy()
        self._episode_winding_last_rad = joints.copy()
        self._episode_winding_rad[:] = 0.0
        self._episode_winding_fault = False

    def observe_episode_winding(self, joints_rad: list[float] | tuple[float, ...]) -> bool:
        """Accumulate shortest-angle motion since the latest fresh recapture."""

        joints = np.asarray(joints_rad, dtype=np.float64)
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise ValueError("episode winding sample must contain six finite radians")
        if self._episode_winding_fault:
            return True
        for index in PERIODIC_JOINT_INDICES:
            delta = shortest_equivalent_delta_rad(
                joints[index], self._episode_winding_last_rad[index]
            )
            self._episode_winding_rad[index] += abs(delta)
        self._episode_winding_last_rad = joints.copy()
        if any(
            self._episode_winding_rad[index]
            > self.maximum_periodic_joint_winding_rad
            for index in PERIODIC_JOINT_INDICES
        ):
            self._episode_winding_fault = True
        return self._episode_winding_fault

    def evaluate(
        self,
        target: Pose6D,
        *,
        dt_s: float,
        generated_monotonic_ns: int | None = None,
        compute_deadline_ns: int | None = None,
        fresh_measured_joint_position_rad: Sequence[float] | None = None,
    ) -> FeasibilityResult:
        evaluate_started_ns = time.perf_counter_ns()
        if (
            compute_deadline_ns is not None
            and evaluate_started_ns >= compute_deadline_ns
        ):
            return FeasibilityResult(
                False,
                FeasibilityReason.CONTROL_COMPUTE_BUDGET_EXHAUSTED,
                None,
                CandidateMetrics(),
                CandidateComputationTiming(),
            )
        limits = self.config.feasibility
        dt = max(float(dt_s), 1e-6)
        branch_reference = self.last_safe_joint_target.copy()
        if fresh_measured_joint_position_rad is not None:
            fresh_measured = np.asarray(
                fresh_measured_joint_position_rad, dtype=np.float64
            )
            if fresh_measured.shape != (6,) or not np.all(np.isfinite(fresh_measured)):
                raise ValueError(
                    "fresh measured arm state must contain six finite radians"
                )
            branch_reference = fresh_measured.copy()
        if self._episode_winding_fault:
            return FeasibilityResult(
                False,
                FeasibilityReason.EPISODE_WINDING_EXCEEDED,
                None,
                CandidateMetrics(
                    branch_reference_rad=tuple(float(value) for value in branch_reference),
                    episode_winding_rad=self.episode_winding_rad,
                ),
                CandidateComputationTiming(
                    total_ms=(time.perf_counter_ns() - evaluate_started_ns) / 1e6
                ),
            )
        if generated_monotonic_ns is None:
            # Deterministic compatibility for direct IK/offline callers: model
            # an already aligned stationary target before their first trial.
            # Runtime sessions pass the real local CLOCK_MONOTONIC tick and use
            # the explicit post-EDG/reference startup contract instead.
            if not self.output_feasibility.has_accepted_target:
                self.output_feasibility.reset(self.last_safe_joint_target)
                self.output_feasibility.commit(
                    self.output_feasibility.preview(
                        self.last_safe_joint_target,
                        generated_monotonic_ns=self._synthetic_generated_monotonic_ns,
                    )
                )
            self._synthetic_generated_monotonic_ns += max(1, int(round(dt * 1e9)))
            candidate_generated_ns = self._synthetic_generated_monotonic_ns
        else:
            candidate_generated_ns = int(generated_monotonic_ns)
        # Continuation IK: every solve starts on the previous accepted branch,
        # never on a lagging actuator state or a global/random seed.
        ik_seed = self.last_safe_joint_target.copy()
        previous_target = self.last_safe_target
        phase_started_ns = time.perf_counter_ns()
        self.ik.set_arm_joints_rad(ik_seed.tolist())
        seed_fk_ns = time.perf_counter_ns() - phase_started_ns
        phase_started_ns = time.perf_counter_ns()
        current_condition, current_sigma, _ = self._jacobian_quality()
        pre_ik_jacobian_ns = time.perf_counter_ns() - phase_started_ns
        ik_completed = self.ik.apply_position_target(
            palm_target_position_m=list(target.position_m),
            palm_target_quaternion_wxyz=(
                [target.orientation_xyzw[3], *target.orientation_xyzw[:3]]
                if self.config.mapping.orientation_enabled
                else None
            ),
            wrist_roll_velocity_rad_s=0.0,
            dt=dt,
            compute_deadline_ns=compute_deadline_ns,
        )
        if (
            ik_completed
            and compute_deadline_ns is not None
            and time.perf_counter_ns() >= compute_deadline_ns
        ):
            ik_completed = False
        if not ik_completed:
            # A partially computed candidate is never observable as the shared
            # authoritative state. Restore the last accepted seed before
            # returning the ordinary HOLD_REJECTED path to the output adapter.
            self.ik.set_arm_joints_rad(ik_seed.tolist())
            timing = CandidateComputationTiming(
                seed_fk_ms=seed_fk_ns / 1e6,
                pre_ik_jacobian_ms=pre_ik_jacobian_ns / 1e6,
                ik_iterations_ms=(
                    self.ik.last_position_target_ik_iterations_ns / 1e6
                ),
                ik_final_fk_ms=(
                    self.ik.last_position_target_final_fk_ns / 1e6
                ),
                total_ms=(time.perf_counter_ns() - evaluate_started_ns) / 1e6,
                ik_iterations_completed=(
                    self.ik.last_position_target_iterations_completed
                ),
            )
            return FeasibilityResult(
                False,
                FeasibilityReason.CONTROL_COMPUTE_BUDGET_EXHAUSTED,
                None,
                CandidateMetrics(
                    ik_seed_rad=tuple(float(value) for value in ik_seed)
                ),
                timing,
            )
        raw_candidate_q = self.ik.arm_joints_rad.copy()
        raw_joint_limit_limited = self.ik.joint_limit_limited
        raw_limited_joint_indices = tuple(self.ik.limited_joint_indices_1_based)
        selected_candidate, branch_offsets = select_nearest_equivalent_joint_branch(
            raw_candidate_q.tolist(),
            branch_reference.tolist(),
        )
        candidate_q = np.asarray(selected_candidate, dtype=np.float64)
        self.ik.set_arm_joints_rad(candidate_q.tolist())
        branch_delta = candidate_q - branch_reference
        target_displacement_from_initial = float(
            np.linalg.norm(
                np.asarray(target.position_m)
                - np.asarray(self.initial_tcp.position_m)
            )
        )
        branch_discontinuous = target_displacement_from_initial <= limits.maximum_target_displacement_m and any(
            abs(float(branch_delta[index]))
            > limits.maximum_joint_target_jump_rad
            for index in PERIODIC_JOINT_INDICES
        )
        if branch_discontinuous:
            self.ik.set_arm_joints_rad(ik_seed.tolist())
            timing = CandidateComputationTiming(
                seed_fk_ms=seed_fk_ns / 1e6,
                pre_ik_jacobian_ms=pre_ik_jacobian_ns / 1e6,
                ik_iterations_ms=self.ik.last_position_target_ik_iterations_ns / 1e6,
                ik_final_fk_ms=self.ik.last_position_target_final_fk_ns / 1e6,
                total_ms=(time.perf_counter_ns() - evaluate_started_ns) / 1e6,
                ik_iterations_completed=self.ik.last_position_target_iterations_completed,
            )
            return FeasibilityResult(
                False,
                FeasibilityReason.JOINT_BRANCH_DISCONTINUITY,
                None,
                CandidateMetrics(
                    ik_seed_rad=tuple(float(value) for value in ik_seed),
                    ik_candidate_rad=tuple(float(value) for value in candidate_q),
                    joint_delta_rad=tuple(float(value) for value in candidate_q - ik_seed),
                    branch_reference_rad=tuple(float(value) for value in branch_reference),
                    branch_delta_rad=tuple(float(value) for value in branch_delta),
                    branch_equivalent_offset=branch_offsets,
                    episode_winding_rad=self.episode_winding_rad,
                ),
                timing,
            )
        joint_target_jump = candidate_q - ik_seed
        joint_velocity = joint_target_jump / dt
        joint_acceleration = (joint_velocity - self.last_safe_joint_velocity) / dt
        target_delta = np.asarray(target.position_m) - np.asarray(previous_target.position_m)
        target_tool_delta = relative_pose(previous_target, target)
        target_tool_rotvec = quaternion_to_rotvec(target_tool_delta.orientation_xyzw)
        target_tool_swing, target_tool_axial_roll = swing_twist_about_local_z(
            target_tool_delta.orientation_xyzw
        )
        target_tool_swing_rad = quaternion_angle_rad(
            target_tool_swing, (0.0, 0.0, 0.0, 1.0)
        )
        phase_started_ns = time.perf_counter_ns()
        displacement = float(
            np.linalg.norm(np.asarray(target.position_m) - np.asarray(self.initial_tcp.position_m))
        )
        workspace_check_ns = time.perf_counter_ns() - phase_started_ns
        phase_started_ns = time.perf_counter_ns()
        condition, sigma_min, jacr = self._jacobian_quality()
        post_ik_jacobian_ns = time.perf_counter_ns() - phase_started_ns
        current_risk = _singularity_risk(current_condition, current_sigma, limits)
        candidate_risk = _singularity_risk(condition, sigma_min, limits)
        risk_delta = candidate_risk - current_risk
        if risk_delta > limits.singularity_direction_hysteresis_ratio:
            singularity_direction = "TOWARD"
        elif risk_delta < -limits.singularity_direction_hysteresis_ratio:
            singularity_direction = "AWAY"
        else:
            singularity_direction = "TANGENT"
        hard_singularity = _hard_singularity(condition, sigma_min, limits)
        current_hard_singularity = _hard_singularity(
            current_condition, current_sigma, limits
        )
        slowdown_region = _slowdown_singularity(condition, sigma_min, limits)
        recovered = (
            condition <= limits.jacobian_recovery_condition
            and sigma_min >= limits.minimum_singular_value_recovery
        )
        if recovered:
            self._singularity_slowdown_latched = False
        elif slowdown_region:
            self._singularity_slowdown_latched = True
        if hard_singularity:
            singularity_state = "HARD"
        elif self._singularity_slowdown_latched:
            singularity_state = "SLOWDOWN"
        elif abs(float(candidate_q[4])) <= limits.wrist_proximity_warning_rad:
            singularity_state = "PROXIMITY"
        else:
            singularity_state = "NORMAL"
        previous_rotation = quaternion_to_matrix(previous_target.orientation_xyzw)
        j6_axis_in_previous_tool = previous_rotation.T @ jacr[:, self.arm_dof_ids[5]]
        j6_axial_sign = float(j6_axis_in_previous_tool[2])
        j6_expected_delta = (
            target_tool_axial_roll / j6_axial_sign
            if abs(j6_axial_sign) > 1e-9
            else 0.0
        )
        j6_axial_contribution = float(joint_target_jump[5]) * j6_axial_sign
        j6_contribution_ratio = (
            abs(j6_axial_contribution / target_tool_axial_roll)
            # Ratios are not meaningful when the requested twist is only
            # floating-point/filter noise; retain the signed angles themselves.
            if abs(target_tool_axial_roll) >= 1e-4
            else None
        )
        nearest_safe_limit_margin = min(
            min(float(candidate_q[index]) - low, high - float(candidate_q[index]))
            for index, (low, high) in enumerate(
                safe_joint_limits_rad(limits.joint_limit_margin_rad)
            )
        )
        phase_started_ns = time.perf_counter_ns()
        new_contacts = self._contact_pairs(self.ik.data)
        self_collision = any(self._pair_kind(pair) == "self" for pair in new_contacts)
        environment_collision = any(
            self._pair_kind(pair) == "environment" for pair in new_contacts
        )
        contact_distances = [
            float(self.ik.data.contact[index].dist)
            for index in range(self.ik.data.ncon)
            if self._contact_pair(self.ik.data, index) in new_contacts
        ]
        collision_check_ns = time.perf_counter_ns() - phase_started_ns
        limit_blockers = joint_limit_margin_blockers(
            candidate_q, margin_rad=limits.joint_limit_margin_rad
        )
        if raw_joint_limit_limited:
            limit_blockers.extend(
                f"joint_{index}_clipped_to_safe_limit"
                for index in raw_limited_joint_indices
            )
        phase_started_ns = time.perf_counter_ns()
        output_prediction: JointOutputFeasibility = self.output_feasibility.preview(
            candidate_q,
            generated_monotonic_ns=candidate_generated_ns,
        )
        output_feasibility_ns = time.perf_counter_ns() - phase_started_ns
        remaining_checks_started_ns = time.perf_counter_ns()
        metrics = CandidateMetrics(
            target_displacement_m=displacement,
            target_jump_m=float(np.linalg.norm(target_delta)),
            target_rotation_jump_rad=_quaternion_angle(
                target.orientation_xyzw, self.last_safe_target.orientation_xyzw
            ),
            tcp_velocity_m_s=float(np.linalg.norm(target_delta)) / dt,
            tcp_angular_velocity_rad_s=_quaternion_angle(
                target.orientation_xyzw, self.last_safe_target.orientation_xyzw
            )
            / dt,
            ik_error_m=self.ik.target_error_m,
            ik_orientation_error_rad=float(self.ik.target_rotation_error_rad or 0.0),
            maximum_joint_target_jump_rad=float(np.max(np.abs(joint_target_jump))),
            joint_limit_blockers=tuple(limit_blockers),
            jacobian_condition=condition,
            minimum_jacobian_singular_value=sigma_min,
            wrist_bend_from_singularity_rad=abs(float(candidate_q[4])),
            maximum_joint_velocity_rad_s=float(np.max(np.abs(joint_velocity))),
            maximum_joint_acceleration_rad_s2=float(np.max(np.abs(joint_acceleration))),
            output_feasibility_interval_s=output_prediction.interval_ns / 1e9,
            output_feasibility_delta_rad=output_prediction.delta_rad,
            predicted_output_joint_velocity_rad_s=(
                output_prediction.predicted_velocity_rad_s
            ),
            predicted_output_maximum_joint_velocity_rad_s=(
                output_prediction.maximum_velocity_rad_s
            ),
            output_velocity_violating_joint_indices=(
                output_prediction.violating_joint_indices
            ),
            output_velocity_boundary_rad_s_per_joint=(
                output_prediction.boundary_rad_s_per_joint
            ),
            previous_emitted_output_joint_velocity_rad_s=(
                output_prediction.previous_emitted_velocity_rad_s
            ),
            predicted_output_joint_acceleration_rad_s2=(
                output_prediction.predicted_acceleration_rad_s2
            ),
            predicted_output_maximum_joint_acceleration_rad_s2=(
                output_prediction.maximum_acceleration_rad_s2
            ),
            output_acceleration_violating_joint_indices=(
                output_prediction.acceleration_violating_joint_indices
            ),
            self_collision=self_collision,
            environment_collision=environment_collision,
            minimum_new_contact_distance_m=min(contact_distances) if contact_distances else None,
            target_tool_rotation_vector_rad=tuple(float(v) for v in target_tool_rotvec),
            target_tool_swing_rad=target_tool_swing_rad,
            target_tool_axial_roll_rad=target_tool_axial_roll,
            ik_seed_rad=tuple(float(v) for v in ik_seed),
            ik_candidate_rad=tuple(float(v) for v in candidate_q),
            joint_delta_rad=tuple(float(v) for v in joint_target_jump),
            wrist_joint_delta_rad=tuple(float(v) for v in joint_target_jump[3:]),
            j6_expected_delta_rad=j6_expected_delta,
            j6_axial_contribution_rad=j6_axial_contribution,
            j6_axial_contribution_ratio=j6_contribution_ratio,
            nearest_safe_joint_limit_margin_rad=nearest_safe_limit_margin,
            branch_switch=bool(np.max(np.abs(joint_target_jump)) >= math.pi / 2.0),
            current_jacobian_condition=current_condition,
            current_minimum_jacobian_singular_value=current_sigma,
            singularity_risk=candidate_risk,
            current_singularity_risk=current_risk,
            singularity_direction=singularity_direction,
            singularity_state=singularity_state,
            wrist_proximity_warning=bool(
                limits.wrist_proximity_warning_rad > 0.0
                and abs(float(candidate_q[4])) <= limits.wrist_proximity_warning_rad
            ),
            effective_ik_damping=self.ik.last_effective_damping,
            current_hard_singularity=current_hard_singularity,
            hard_stop_required=bool(
                current_hard_singularity and singularity_direction != "AWAY"
            ),
            branch_reference_rad=tuple(float(value) for value in branch_reference),
            branch_delta_rad=tuple(float(value) for value in branch_delta),
            branch_equivalent_offset=branch_offsets,
            episode_winding_rad=self.episode_winding_rad,
        )
        hard_escape = current_hard_singularity and singularity_direction == "AWAY"
        if hard_singularity and not hard_escape:
            reason = FeasibilityReason.NEAR_SINGULARITY
        else:
            reason = classify_candidate(metrics, limits, check_singularity=False)
            if (
                reason is FeasibilityReason.ACCEPTED
                and self._singularity_slowdown_latched
                and singularity_direction == "TOWARD"
            ):
                reason = FeasibilityReason.SINGULARITY_SLOWDOWN
        if (
            compute_deadline_ns is not None
            and time.perf_counter_ns() >= compute_deadline_ns
        ):
            # All candidate checks completed, but too late for this producer
            # cycle. It remains non-authoritative and the session publishes a
            # HOLD_REJECTED heartbeat instead of a late accepted destination.
            # A current-state hard singularity is different: it is already a
            # terminal safety condition, and must not be relabelled as a
            # timing rejection merely because the diagnostic work crossed the
            # producer deadline.
            if not metrics.hard_stop_required:
                reason = FeasibilityReason.CONTROL_COMPUTE_BUDGET_EXHAUSTED
            self.ik.set_arm_joints_rad(ik_seed.tolist())
        if reason is FeasibilityReason.ACCEPTED:
            self.output_feasibility.commit(output_prediction)
            self.last_safe_joint_target = candidate_q
            self.last_safe_joint_velocity = joint_velocity
            self.last_safe_target = target
            self.accepted_metrics.append(metrics)
            joint_target = tuple(float(v) for v in candidate_q)
        else:
            joint_target = None
        remaining_checks_ns = time.perf_counter_ns() - remaining_checks_started_ns
        timing = CandidateComputationTiming(
            seed_fk_ms=seed_fk_ns / 1e6,
            pre_ik_jacobian_ms=pre_ik_jacobian_ns / 1e6,
            ik_iterations_ms=self.ik.last_position_target_ik_iterations_ns / 1e6,
            ik_final_fk_ms=self.ik.last_position_target_final_fk_ns / 1e6,
            post_ik_jacobian_ms=post_ik_jacobian_ns / 1e6,
            workspace_check_ms=workspace_check_ns / 1e6,
            collision_check_ms=collision_check_ns / 1e6,
            output_feasibility_ms=output_feasibility_ns / 1e6,
            remaining_checks_ms=remaining_checks_ns / 1e6,
            total_ms=(time.perf_counter_ns() - evaluate_started_ns) / 1e6,
            ik_iterations_completed=self.ik.last_position_target_iterations_completed,
        )
        return FeasibilityResult(
            reason is FeasibilityReason.ACCEPTED,
            reason,
            joint_target,
            metrics,
            timing,
        )

    def _jacobian_quality(self) -> tuple[float, float, np.ndarray]:
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacBody(self.model, self.ik.data, jacp, jacr, self.palm_body_id)
        position_jacobian = jacp[:, self.arm_dof_ids]
        spatial_jacobian = (
            np.vstack(
                (
                    position_jacobian,
                    self.jacobian_rotation_characteristic_length_m
                    * jacr[:, self.arm_dof_ids],
                )
            )
            if self.config.mapping.orientation_enabled
            else position_jacobian
        )
        singular_values = np.linalg.svd(spatial_jacobian, compute_uv=False)
        sigma_min = float(singular_values[-1])
        return (
            float(singular_values[0] / max(sigma_min, 1e-12)),
            sigma_min,
            jacr,
        )

    def metrics_report(self) -> dict[str, Any]:
        metrics = self.accepted_metrics
        report = {
            "ik_success_rate": None,
            "maximum_jacobian_condition": max((m.jacobian_condition for m in metrics), default=None),
            "minimum_jacobian_singular_value": min(
                (m.minimum_jacobian_singular_value for m in metrics), default=None
            ),
            "maximum_tcp_displacement_m": max(
                (m.target_displacement_m for m in metrics), default=0.0
            ),
            "maximum_tcp_velocity_m_s": max((m.tcp_velocity_m_s for m in metrics), default=0.0),
            "maximum_joint_velocity_rad_s": max(
                (m.maximum_joint_velocity_rad_s for m in metrics), default=0.0
            ),
            "maximum_joint_acceleration_rad_s2": max(
                (m.maximum_joint_acceleration_rad_s2 for m in metrics), default=0.0
            ),
            "minimum_collision_distance_m": min(
                (
                    m.minimum_new_contact_distance_m
                    for m in metrics
                    if m.minimum_new_contact_distance_m is not None
                ),
                default=None,
            ),
            "maximum_periodic_joint_winding_rad": self.maximum_periodic_joint_winding_rad,
            "episode_winding_rad": self.episode_winding_rad,
            "episode_winding_guard_tripped": self._episode_winding_fault,
        }
        if hasattr(self, "tracking_errors_m"):
            report["maximum_desired_to_simulated_tcp_error_m"] = max(
                self.tracking_errors_m, default=0.0
            )
            report["peak_actual_joint_velocity_rad_s"] = self.peak_actual_joint_velocity_rad_s.tolist()
            report["simulated_velocity_limit_hits_per_joint"] = self.command_velocity_limit_hits.tolist()
            report["simulated_acceleration_limit_hits_per_joint"] = self.command_acceleration_limit_hits.tolist()
            report["simulated_jerk_limit_hits_per_joint"] = self.command_jerk_limit_hits.tolist()
        return report

    def _contact_pairs(self, data: mujoco.MjData) -> set[tuple[int, int]]:
        return {self._contact_pair(data, index) for index in range(data.ncon)}

    def _require_contact_free_authoritative_state(self, context: str) -> None:
        contacts = self._contact_pairs(self.ik.data)
        if not contacts:
            return
        descriptions = []
        for geom1, geom2 in sorted(contacts):
            names = [
                mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
                or f"geom_{geom_id}"
                for geom_id in (geom1, geom2)
            ]
            descriptions.append("/".join(names))
        raise ValueError(
            f"{context} contains collision contacts: {', '.join(descriptions)}"
        )

    @staticmethod
    def _contact_pair(data: mujoco.MjData, index: int) -> tuple[int, int]:
        contact = data.contact[index]
        return tuple(sorted((int(contact.geom1), int(contact.geom2))))

    def _pair_kind(self, pair: tuple[int, int]) -> str:
        robot = [self._is_robot_geom(geom_id) for geom_id in pair]
        return "self" if all(robot) else "environment"

    def _is_robot_geom(self, geom_id: int) -> bool:
        body_id = int(self.model.geom_bodyid[geom_id])
        name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        return name.startswith("jaka_") or name.startswith("rh56_")


class JakaMujocoSimulation(SharedJakaTargetGenerator):
    """MuJoCo plant adapter layered after shared kinematic target generation."""

    def __init__(self, config: ReplayConfig, *, mjcf_path: str | Path | None = None) -> None:
        super().__init__(config, mjcf_path=mjcf_path)
        simulation_values = config.raw.get("simulation", {})
        if config.zero_gravity:
            self.model.opt.gravity[:] = 0.0
        integrator = str(simulation_values.get("integrator", "implicitfast")).lower()
        if integrator != "implicitfast":
            raise ValueError("Quest/JAKA simulation requires the implicitfast integrator")
        self.model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.data = mujoco.MjData(self.model)
        self.arm_actuator_ids = np.asarray(
            [
                self._required_id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_act")
                for name in MJCF_ARM_JOINT_NAMES
            ],
            dtype=np.int32,
        )
        self.hand_actuator_names = (
            "rh56_R_thumb_MCP_joint1_act",
            "rh56_R_thumb_MCP_joint2_act",
            "rh56_R_index_MCP_joint_act",
            "rh56_R_middle_MCP_joint_act",
            "rh56_R_ring_MCP_joint_act",
            "rh56_R_pinky_MCP_joint_act",
        )
        self.hand_actuator_ids = np.asarray(
            [
                actuator_id
                for name in self.hand_actuator_names
                if (actuator_id := mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)) >= 0
            ],
            dtype=np.int32,
        )
        self.hand_available = len(self.hand_actuator_ids) == 6
        self.hand_ctrl_ranges = (
            self.model.actuator_ctrlrange[self.hand_actuator_ids].copy()
            if self.hand_available
            else np.empty((0, 2), dtype=np.float64)
        )
        self._set_position_actuator_gains(
            self.arm_actuator_ids,
            kp=float(simulation_values.get("arm_position_kp", 40.0)),
            kv=float(simulation_values.get("arm_position_kv", 0.0)),
        )
        if self.hand_available:
            self._set_position_actuator_gains(
                self.hand_actuator_ids,
                kp=float(simulation_values.get("hand_position_kp", 8.0)),
                kv=float(simulation_values.get("hand_position_kv", 0.0)),
            )
        self.data.qpos[self.arm_qpos_ids] = config.initial_arm_joints_rad
        self.data.ctrl[self.arm_actuator_ids] = config.initial_arm_joints_rad
        mujoco.mj_forward(self.model, self.data)
        self.commanded_joint_target = self.last_safe_joint_target.copy()
        self.commanded_joint_velocity = np.zeros(6, dtype=np.float64)
        self.commanded_joint_acceleration = np.zeros(6, dtype=np.float64)
        self.commanded_hand_target = np.zeros(6, dtype=np.float64)
        self.commanded_hand_velocity = np.zeros(6, dtype=np.float64)
        self.tracking_errors_m: list[float] = []
        self.peak_actual_joint_velocity_rad_s = np.zeros(6, dtype=np.float64)
        self.command_velocity_limit_hits = np.zeros(6, dtype=np.int64)
        self.command_acceleration_limit_hits = np.zeros(6, dtype=np.int64)
        self.command_jerk_limit_hits = np.zeros(6, dtype=np.int64)
        self.arm_output_mode = "shaped-500hz"
        self.desired_marker_mocap_id = self._mocap_id(DESIRED_MARKER_BODY)
        self.actual_marker_mocap_id = self._mocap_id(ACTUAL_MARKER_BODY)

    def _set_position_actuator_gains(
        self, actuator_ids: np.ndarray, *, kp: float, kv: float
    ) -> None:
        if not math.isfinite(kp) or kp <= 0.0:
            raise ValueError("simulation position-actuator kp must be finite and positive")
        if not math.isfinite(kv) or kv < 0.0:
            raise ValueError("simulation position-actuator kv must be finite and non-negative")
        self.model.actuator_gainprm[actuator_ids, 0] = kp
        self.model.actuator_biasprm[actuator_ids, 1] = -kp
        self.model.actuator_biasprm[actuator_ids, 2] = -kv

    def _mocap_id(self, body_name: str) -> int:
        body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        return -1 if body < 0 else int(self.model.body_mocapid[body])

    @property
    def current_tcp_pose(self) -> Pose6D:
        quat_wxyz = np.zeros(4)
        mujoco.mju_mat2Quat(quat_wxyz, self.data.xmat[self.palm_body_id])
        return Pose6D(
            tuple(float(value) for value in self.data.xpos[self.palm_body_id]),
            (float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3]), float(quat_wxyz[0])),
        )

    @property
    def arm_joints_rad(self) -> np.ndarray:
        return self.data.qpos[self.arm_qpos_ids].copy()

    def synchronize_authoritative_arm_joints(self, joints_rad: list[float]) -> None:
        joints = np.asarray(joints_rad, dtype=np.float64)
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise ValueError("authoritative arm state must contain six finite radians")
        self.data.qpos[self.arm_qpos_ids] = joints
        self.data.ctrl[self.arm_actuator_ids] = joints
        mujoco.mj_forward(self.model, self.data)
        super().synchronize_authoritative_arm_joints(joints.tolist())
        self.commanded_joint_target = joints.copy()
        self.commanded_joint_velocity[:] = 0.0
        self.commanded_joint_acceleration[:] = 0.0

    def capture_reference(self) -> Pose6D:
        SharedJakaTargetGenerator.synchronize_authoritative_arm_joints(
            self, self.arm_joints_rad.tolist()
        )
        current = SharedJakaTargetGenerator.capture_reference(self)
        self.commanded_joint_target = self.arm_joints_rad
        self.commanded_joint_velocity[:] = 0.0
        self.commanded_joint_acceleration[:] = 0.0
        self.data.ctrl[self.arm_actuator_ids] = self.commanded_joint_target
        self.tracking_errors_m.clear()
        self.peak_actual_joint_velocity_rad_s[:] = 0.0
        self.command_velocity_limit_hits[:] = 0
        self.command_acceleration_limit_hits[:] = 0
        self.command_jerk_limit_hits[:] = 0
        return current

    def set_accepted_arm_joint_target(self, joints_rad: tuple[float, ...]) -> None:
        """MuJoCo output boundary; no mapping, filtering, IK, or shaping occurs here."""

        joints = np.asarray(joints_rad, dtype=np.float64)
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise ValueError("accepted arm target must contain six finite joint radians")
        self.commanded_joint_target = joints.copy()

    def set_accepted_arm_tcp_pose(self, pose: Any) -> None:
        self.last_safe_target = Pose6D(
            tuple(float(value) for value in pose.position_m),
            tuple(float(value) for value in pose.orientation_xyzw),
        )

    def enable_direct_125hz_arm_output(self) -> None:
        if float(self.data.time) != 0.0:
            raise RuntimeError("arm output mode must be selected before simulation starts")
        self.arm_output_mode = "jaka-equivalent-125hz"
        self.commanded_joint_velocity[:] = 0.0
        self.commanded_joint_acceleration[:] = 0.0

    def set_emitted_arm_joint_target(self, joints_rad: tuple[float, ...]) -> None:
        if self.arm_output_mode != "jaka-equivalent-125hz":
            raise RuntimeError("direct arm commands require jaka-equivalent-125hz mode")
        joints = np.asarray(joints_rad, dtype=np.float64)
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise ValueError("emitted arm target must contain six finite joint radians")
        self.commanded_joint_target = joints.copy()
        self.data.ctrl[self.arm_actuator_ids] = joints

    def set_hand_actuator_target(self, targets_rad: Mapping[str, float]) -> None:
        """Set only the six simulated RH56 actuator goals in explicit model order."""

        if not self.hand_available:
            raise RuntimeError("this arm-only simulation has no RH56 actuator path")

        order = ("thumb_lateral", "thumb_close", "index", "middle", "ring", "pinky")
        values = np.asarray([float(targets_rad[name]) for name in order], dtype=np.float64)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            raise ValueError("RH56 simulated actuator target must contain six finite values")
        lower = self.hand_ctrl_ranges[:, 0]
        upper = self.hand_ctrl_ranges[:, 1]
        if np.any(values < lower - 1e-9) or np.any(values > upper + 1e-9):
            raise ValueError("RH56 simulated actuator target violates project model limits")
        self.commanded_hand_target = values

    def step(self, dt_s: float) -> None:
        steps = max(0, int(round(max(0.0, dt_s) / self.model.opt.timestep)))
        for _ in range(steps):
            timestep = float(self.model.opt.timestep)
            if self.arm_output_mode == "shaped-500hz":
                limits = self.config.command_limits
                omega = limits.position_tracking_frequency_rad_s
                desired_jerk = (
                    omega**3 * (self.commanded_joint_target - self.data.ctrl[self.arm_actuator_ids])
                    - 3.0 * omega**2 * self.commanded_joint_velocity
                    - 3.0 * omega * self.commanded_joint_acceleration
                )
                bounded_jerk = np.clip(
                    desired_jerk, -limits.maximum_jerk_rad_s3, limits.maximum_jerk_rad_s3
                )
                acceleration_unbounded = self.commanded_joint_acceleration + bounded_jerk * timestep
                acceleration_bounded = np.clip(
                    acceleration_unbounded,
                    -limits.maximum_acceleration_rad_s2,
                    limits.maximum_acceleration_rad_s2,
                )
                velocity_unbounded = self.commanded_joint_velocity + acceleration_bounded * timestep
                velocity_bounds = np.asarray(limits.velocity_boundaries_rad_s, dtype=float)
                self.command_jerk_limit_hits += np.abs(desired_jerk) > limits.maximum_jerk_rad_s3
                self.command_acceleration_limit_hits += (
                    np.abs(acceleration_unbounded) > limits.maximum_acceleration_rad_s2
                )
                self.command_velocity_limit_hits += np.abs(velocity_unbounded) > velocity_bounds
                position, velocity, acceleration = jerk_limited_position_step(
                    self.data.ctrl[self.arm_actuator_ids],
                    self.commanded_joint_target,
                    self.commanded_joint_velocity,
                    self.commanded_joint_acceleration,
                    dt_s=timestep,
                    limits=self.config.command_limits,
                )
                self.data.ctrl[self.arm_actuator_ids] = position
                self.commanded_joint_velocity = velocity
                self.commanded_joint_acceleration = acceleration
            if self.hand_available:
                hand_error = self.commanded_hand_target - self.data.ctrl[self.hand_actuator_ids]
                desired_hand_velocity = np.clip(hand_error / timestep, -4.0, 4.0)
                self.commanded_hand_velocity += np.clip(
                    desired_hand_velocity - self.commanded_hand_velocity,
                    -40.0 * timestep,
                    40.0 * timestep,
                )
                hand_increment = self.commanded_hand_velocity * timestep
                hand_increment = np.where(
                    np.abs(hand_increment) > np.abs(hand_error), hand_error, hand_increment
                )
                self.data.ctrl[self.hand_actuator_ids] += hand_increment
            mujoco.mj_step(self.model, self.data)
            self.peak_actual_joint_velocity_rad_s = np.maximum(
                self.peak_actual_joint_velocity_rad_s,
                np.abs(self.data.qvel[self.arm_dof_ids]),
            )
        self.tracking_errors_m.append(
            float(
                np.linalg.norm(
                    np.asarray(self.last_safe_target.position_m)
                    - np.asarray(self.current_tcp_pose.position_m)
                )
            )
        )
        self.update_markers(self.last_safe_target)

    def update_markers(self, desired: Pose6D | None) -> None:
        actual = self.current_tcp_pose
        if self.actual_marker_mocap_id >= 0:
            self.data.mocap_pos[self.actual_marker_mocap_id] = actual.position_m
            self.data.mocap_quat[self.actual_marker_mocap_id] = (
                actual.orientation_xyzw[3],
                actual.orientation_xyzw[0],
                actual.orientation_xyzw[1],
                actual.orientation_xyzw[2],
            )
        if self.desired_marker_mocap_id >= 0:
            marker = actual if desired is None else desired
            self.data.mocap_pos[self.desired_marker_mocap_id] = marker.position_m
            self.data.mocap_quat[self.desired_marker_mocap_id] = (
                marker.orientation_xyzw[3],
                marker.orientation_xyzw[0],
                marker.orientation_xyzw[1],
                marker.orientation_xyzw[2],
            )
        mujoco.mj_forward(self.model, self.data)


class QuestJakaReplaySession:
    def __init__(self, config: ReplayConfig, simulation: JakaMujocoSimulation) -> None:
        self.config = config
        self.simulation = simulation
        self.assembler = HtsCanonicalAssembler(stale_after_s=config.stale_after_s)
        self.operator = RightHandOperatorPipeline(
            RightHandOperatorConfig(
                stale_after_s=config.stale_after_s,
                translation_scale=(1.0, 1.0, 1.0),
                orientation_mapping="relative",
                orientation_scale=1.0,
                filter_time_constant_s=0.02,
                jump_reject_translation_m=config.mapping.maximum_operator_displacement_m,
                jump_reject_rotation_rad=math.radians(60.0),
                workspace_min_m=(-config.mapping.maximum_operator_displacement_m,) * 3,
                workspace_max_m=(config.mapping.maximum_operator_displacement_m,) * 3,
            )
        )
        self.mapper = ProvisionalOperatorToRobotMapper(config.mapping)
        self.rejections: Counter[str] = Counter()
        self.frame_count = 0
        self.valid_input_frames = 0
        self.invalid_input_events = 0
        self.accepted_targets = 0
        self.ik_attempts = 0
        self.ik_successes = 0
        self._next_engagement = 0
        self._armed_at_sequence: int | None = None
        self._last_right_sequence: int | None = None
        self._last_event_ns: int | None = None
        self._last_right_event_ns: int | None = None
        self._first_event_ns: int | None = None
        self.axis_rows: list[tuple[float, tuple[float, ...], tuple[float, ...]]] = []
        self.right_hand_valid = False
        self.last_reason = FeasibilityReason.DISENGAGED.value
        self._scheduled_capture_pending = False
        self.event_records: list[dict[str, Any]] = []

    def process(self, datagram: ReceivedHtsDatagram) -> None:
        if self._first_event_ns is None:
            self._first_event_ns = datagram.receive_monotonic_ns
        elapsed = (datagram.receive_monotonic_ns - self._first_event_ns) / 1e9
        self._last_event_ns = datagram.receive_monotonic_ns
        try:
            state = self.assembler.ingest(
                parse_hts_datagram(datagram.payload),
                receive_monotonic_ns=datagram.receive_monotonic_ns,
                source_endpoint=datagram.source_endpoint,
                datagram_size=len(datagram.payload),
            )
        except SerializationError:
            self.operator.force_fault(
                timestamp_monotonic_ns=datagram.receive_monotonic_ns,
                reason="malformed_recorded_datagram",
            )
            self.mapper.clear_reference()
            self.invalid_input_events += 1
            self.rejections[FeasibilityReason.INPUT_INVALID.value] += 1
            return

        right = state.right
        self.right_hand_valid = right.tracking_valid
        new_right = (
            right.host_sequence_number is not None
            and right.host_sequence_number != self._last_right_sequence
        )
        if new_right:
            dt = (
                self.simulation.model.opt.timestep
                if self._last_right_event_ns is None
                else max(
                    self.simulation.model.opt.timestep,
                    (datagram.receive_monotonic_ns - self._last_right_event_ns) / 1e9,
                )
            )
            self._last_right_event_ns = datagram.receive_monotonic_ns
            self._last_right_sequence = right.host_sequence_number
            self.frame_count += 1
            if right.tracking_valid and len(right.joints) == 21:
                self.valid_input_frames += 1

        engage = False
        capture = False
        if (
            self._next_engagement < len(self.config.engagement_schedule_s)
            and elapsed >= self.config.engagement_schedule_s[self._next_engagement]
            and self.operator.state is OperatorInputState.DISENGAGED
            and right.tracking_valid
        ):
            engage = True
            self._armed_at_sequence = right.host_sequence_number
            self._scheduled_capture_pending = True
            self._next_engagement += 1
        elif (
            self._scheduled_capture_pending
            and
            self.operator.state is OperatorInputState.ARMED_REFERENCE_CAPTURE
            and new_right
            and right.tracking_valid
            and right.host_sequence_number != self._armed_at_sequence
        ):
            capture = True

        transitions_before = len(self.operator.transitions)
        operator_output = self.operator.step(
            state,
            engage_request=engage,
            capture_reference_request=capture,
        )
        new_transitions = self.operator.transitions[transitions_before:]
        for transition in new_transitions:
            if transition.current is OperatorInputState.DISENGAGED:
                self.mapper.clear_reference()
                self.invalid_input_events += 1
                self.rejections[FeasibilityReason.INPUT_INVALID.value] += 1
                self.last_reason = transition.reason

        if operator_output.reason == "reference_captured":
            self._scheduled_capture_pending = False
            self.mapper.capture_robot_reference(self.simulation.capture_reference())
        if not new_right and operator_output.reason != "reference_captured":
            return
        if not operator_output.valid_for_mapping:
            if new_right:
                self.rejections[FeasibilityReason.DISENGAGED.value] += 1
                self.last_reason = operator_output.reason
                self.event_records.append(
                    {
                        "elapsed_s": elapsed,
                        "input_sequence": right.host_sequence_number,
                        "state": self.operator.state.value,
                        "accepted": False,
                        "reason": FeasibilityReason.DISENGAGED.value,
                    }
                )
            return
        try:
            desired = self.mapper.map(operator_output)
        except MappingRejection as exc:
            self.event_records.append(
                {
                    "elapsed_s": elapsed,
                    "input_sequence": right.host_sequence_number,
                    "state": self.operator.state.value,
                    "accepted": False,
                    "reason": exc.reason,
                }
            )
            self._reject(datagram.receive_monotonic_ns, exc.reason)
            return
        self.ik_attempts += 1
        result = self.simulation.evaluate(desired, dt_s=dt)
        if result.metrics.ik_error_m <= self.config.feasibility.ik_position_tolerance_m:
            self.ik_successes += 1
        if not result.accepted:
            self.event_records.append(
                {
                    "elapsed_s": elapsed,
                    "input_sequence": right.host_sequence_number,
                    "state": self.operator.state.value,
                    "accepted": False,
                    "reason": result.reason.value,
                    "desired_tcp": {
                        "position_m": desired.position_m,
                        "orientation_xyzw": desired.orientation_xyzw,
                    },
                    "simulated_tcp_position_m": self.simulation.current_tcp_pose.position_m,
                    "metrics": asdict(result.metrics),
                }
            )
            self._reject(datagram.receive_monotonic_ns, result.reason.value)
            return
        self.accepted_targets += 1
        self.last_reason = FeasibilityReason.ACCEPTED.value
        self.axis_rows.append((elapsed, operator_output.translation_m, desired.position_m))
        self.event_records.append(
            {
                "elapsed_s": elapsed,
                "input_sequence": right.host_sequence_number,
                "state": self.operator.state.value,
                "accepted": True,
                "reason": FeasibilityReason.ACCEPTED.value,
                "operator_delta_m": operator_output.translation_m,
                "desired_tcp": {
                    "position_m": desired.position_m,
                    "orientation_xyzw": desired.orientation_xyzw,
                },
                "joint_target_rad": result.joint_target_rad,
                "simulated_tcp_position_m": self.simulation.current_tcp_pose.position_m,
                "tracking_error_m": float(
                    np.linalg.norm(
                        np.asarray(desired.position_m)
                        - np.asarray(self.simulation.current_tcp_pose.position_m)
                    )
                ),
                "metrics": asdict(result.metrics),
            }
        )

    def _reject(self, timestamp_ns: int, reason: str) -> None:
        self.rejections[reason] += 1
        self.last_reason = reason
        self.operator.force_fault(timestamp_monotonic_ns=timestamp_ns, reason=reason)
        self.mapper.clear_reference()

    def report(self, *, replay_source: str) -> dict[str, Any]:
        metrics = self.simulation.metrics_report()
        metrics["ik_success_rate"] = (
            None if self.ik_attempts == 0 else self.ik_successes / self.ik_attempts
        )
        return {
            "schema_version": "quest_jaka_offline_sim_report.v1",
            "replay_source": str(Path(replay_source).resolve()),
            "frame_count": self.frame_count,
            "valid_input_frames": self.valid_input_frames,
            "invalid_input_events": self.invalid_input_events,
            "engagement_transitions": [
                {
                    "timestamp_monotonic_ns": row.timestamp_monotonic_ns,
                    "previous": row.previous.value,
                    "current": row.current.value,
                    "reason": row.reason,
                }
                for row in self.operator.transitions
            ],
            "final_state": self.operator.state.value,
            "accepted_target_count": self.accepted_targets,
            "rejection_counts_by_reason": dict(sorted(self.rejections.items())),
            "ik_attempts": self.ik_attempts,
            "ik_successes": self.ik_successes,
            **metrics,
            "axis_response_summary": self._axis_summary(),
            "provisional_calibration": {
                "calibration_id": self.config.mapping.calibration_id,
                "calibrated": False,
                "operator_to_robot_basis": self.config.mapping.operator_to_robot_basis,
                "translation_scale_per_axis": self.config.mapping.translation_scale_per_axis,
                "orientation_enabled": self.config.mapping.orientation_enabled,
            },
            "rejected_sample_action": "DISENGAGE_AND_HOLD_LAST_SAFE_SIMULATED_TARGET",
            "hardware_connections": False,
            "hardware_commands": False,
        }

    def _axis_summary(self) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for label, operator_axis in (
            ("right_hand_left_right_s", 0),
            ("right_hand_forward_backward_s", 2),
            ("right_hand_up_down_s", 1),
        ):
            window = self.config.axis_analysis.get(label)
            if window is None:
                result[label] = {"status": "NOT_PRESENT_OR_NOT_INDEPENDENTLY_RECORDED"}
                continue
            rows = [row for row in self.axis_rows if float(window[0]) <= row[0] < float(window[1])]
            if len(rows) < 2:
                result[label] = {"status": "INSUFFICIENT_ACCEPTED_TARGETS"}
                continue
            robot = np.asarray([row[2] for row in rows])
            operator = np.asarray([row[1] for row in rows])
            ranges = np.ptp(robot, axis=0)
            dominant = int(np.argmax(ranges))
            correlation = float(np.corrcoef(operator[:, operator_axis], robot[:, dominant])[0, 1])
            result[label] = {
                "status": "OBSERVED",
                "dominant_robot_base_axis": ("X", "Y", "Z")[dominant],
                "robot_axis_ranges_m": [float(value) for value in ranges],
                "operator_to_robot_sign": "positive" if correlation >= 0 else "negative",
                "correlation": correlation,
                "accepted_samples": len(rows),
            }
            if label == "right_hand_left_right_s":
                result[label]["direction_note"] = (
                    "hand right (+canonical X) produces +robot-base X"
                )
            elif label == "right_hand_forward_backward_s":
                result[label]["direction_note"] = (
                    "hand forward (-canonical Z) produces -robot-base Y"
                )
        return result


def _quaternion_angle(left: tuple[float, ...], right: tuple[float, ...]) -> float:
    dot = min(1.0, abs(float(np.dot(left, right))))
    return 2.0 * math.acos(dot)
