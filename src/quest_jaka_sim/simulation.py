"""Deterministic, simulation-only Quest-to-JAKA replay and feasibility checks."""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass
from enum import Enum
import math
from pathlib import Path
from typing import Any, Mapping
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from embodiment_core.config import load_yaml
from jaka_driver_adapter.palm_target_ik import (
    MJCF_ARM_JOINT_NAMES,
    PalmTargetIkState,
    joint_limit_margin_blockers,
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

from .mapping import MappingRejection, ProvisionalMappingConfig, ProvisionalOperatorToRobotMapper


DESIRED_MARKER_BODY = "quest_jaka_desired_tcp_marker"
ACTUAL_MARKER_BODY = "quest_jaka_actual_tcp_marker"


class FeasibilityReason(str, Enum):
    ACCEPTED = "ACCEPTED"
    INPUT_INVALID = "INPUT_INVALID"
    DISENGAGED = "DISENGAGED"
    OUTSIDE_OPERATOR_ENVELOPE = "OUTSIDE_OPERATOR_ENVELOPE"
    TARGET_JUMP = "TARGET_JUMP"
    OUTSIDE_ROBOT_WORKSPACE = "OUTSIDE_ROBOT_WORKSPACE"
    IK_FAILED = "IK_FAILED"
    JOINT_LIMIT = "JOINT_LIMIT"
    NEAR_SINGULARITY = "NEAR_SINGULARITY"
    VELOCITY_LIMIT = "VELOCITY_LIMIT"
    ACCELERATION_LIMIT = "ACCELERATION_LIMIT"
    SELF_COLLISION = "SELF_COLLISION"
    ENVIRONMENT_COLLISION = "ENVIRONMENT_COLLISION"


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
            maximum_joint_velocity_rad_s=float(values["maximum_joint_velocity_rad_s"]),
            maximum_joint_acceleration_rad_s2=float(
                values["maximum_joint_acceleration_rad_s2"]
            ),
            joint_limit_margin_rad=math.radians(float(values["joint_limit_margin_deg"])),
            maximum_target_displacement_m=maximum_target_displacement_m,
        )


@dataclass(frozen=True, slots=True)
class CandidateMetrics:
    target_displacement_m: float = 0.0
    target_jump_m: float = 0.0
    tcp_velocity_m_s: float = 0.0
    tcp_angular_velocity_rad_s: float = 0.0
    ik_error_m: float = 0.0
    joint_limit_blockers: tuple[str, ...] = ()
    jacobian_condition: float = 1.0
    minimum_jacobian_singular_value: float = 1.0
    maximum_joint_velocity_rad_s: float = 0.0
    maximum_joint_acceleration_rad_s2: float = 0.0
    self_collision: bool = False
    environment_collision: bool = False
    minimum_new_contact_distance_m: float | None = None


def classify_candidate(metrics: CandidateMetrics, limits: FeasibilityLimits) -> FeasibilityReason:
    if metrics.target_displacement_m > limits.maximum_target_displacement_m:
        return FeasibilityReason.OUTSIDE_ROBOT_WORKSPACE
    if metrics.target_jump_m > limits.maximum_target_jump_m:
        return FeasibilityReason.TARGET_JUMP
    if (
        metrics.tcp_velocity_m_s > limits.maximum_tcp_velocity_m_s
        or metrics.tcp_angular_velocity_rad_s > limits.maximum_tcp_angular_velocity_rad_s
        or metrics.maximum_joint_velocity_rad_s > limits.maximum_joint_velocity_rad_s
    ):
        return FeasibilityReason.VELOCITY_LIMIT
    if metrics.maximum_joint_acceleration_rad_s2 > limits.maximum_joint_acceleration_rad_s2:
        return FeasibilityReason.ACCELERATION_LIMIT
    if metrics.ik_error_m > limits.ik_position_tolerance_m:
        return FeasibilityReason.IK_FAILED
    if metrics.joint_limit_blockers:
        return FeasibilityReason.JOINT_LIMIT
    if (
        metrics.jacobian_condition > limits.maximum_jacobian_condition
        or metrics.minimum_jacobian_singular_value < limits.minimum_jacobian_singular_value
    ):
        return FeasibilityReason.NEAR_SINGULARITY
    if metrics.self_collision:
        return FeasibilityReason.SELF_COLLISION
    if metrics.environment_collision:
        return FeasibilityReason.ENVIRONMENT_COLLISION
    return FeasibilityReason.ACCEPTED


@dataclass(frozen=True, slots=True)
class FeasibilityResult:
    accepted: bool
    reason: FeasibilityReason
    joint_target_rad: tuple[float, ...] | None
    metrics: CandidateMetrics


@dataclass(frozen=True, slots=True)
class ReplayConfig:
    raw: Mapping[str, Any]
    mapping: ProvisionalMappingConfig
    feasibility: FeasibilityLimits
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

    @classmethod
    def load(cls, path: str | Path) -> "ReplayConfig":
        raw = load_yaml(path)
        provisional = ProvisionalMappingConfig.from_mapping(raw["provisional_calibration"])
        simulation = raw["simulation"]
        return cls(
            raw=raw,
            mapping=provisional,
            feasibility=FeasibilityLimits.from_mapping(
                simulation,
                maximum_target_displacement_m=provisional.maximum_target_displacement_m,
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
        )


def build_viewer_mjcf(base_path: str | Path, output_path: str | Path) -> Path:
    base = Path(base_path).resolve()
    output = Path(output_path).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(base)
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("meshdir", str(base.parent))
    world = root.find("worldbody")
    if world is None:
        raise RuntimeError("MuJoCo model has no worldbody")
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
    tree.write(output, encoding="utf-8")
    return output


class JakaMujocoSimulation:
    """Separate IK scratch state and dynamically stepped position-actuator plant."""

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
        )
        self.model = self.ik.model
        if config.zero_gravity:
            self.model.opt.gravity[:] = 0.0
        self.data = mujoco.MjData(self.model)
        self.arm_joint_ids = self.ik.arm_joint_ids.copy()
        self.arm_qpos_ids = self.ik.arm_qpos_ids.copy()
        self.arm_dof_ids = self.ik.arm_dof_ids.copy()
        self.arm_actuator_ids = np.asarray(
            [
                self._required_id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_act")
                for name in MJCF_ARM_JOINT_NAMES
            ],
            dtype=np.int32,
        )
        self.palm_body_id = self.ik.palm_body_id
        self.data.qpos[self.arm_qpos_ids] = config.initial_arm_joints_rad
        self.data.ctrl[self.arm_actuator_ids] = config.initial_arm_joints_rad
        mujoco.mj_forward(self.model, self.data)
        self.initial_tcp = self.current_tcp_pose
        self.last_safe_joint_target = np.asarray(config.initial_arm_joints_rad, dtype=np.float64)
        self.last_safe_target = self.initial_tcp
        self.last_safe_joint_velocity = np.zeros(6)
        self._baseline_contacts = self._contact_pairs(self.ik.data)
        self.accepted_metrics: list[CandidateMetrics] = []
        self.tracking_errors_m: list[float] = []
        self.desired_marker_mocap_id = self._mocap_id(DESIRED_MARKER_BODY)
        self.actual_marker_mocap_id = self._mocap_id(ACTUAL_MARKER_BODY)

    def _required_id(self, kind: mujoco.mjtObj, name: str) -> int:
        value = mujoco.mj_name2id(self.model, kind, name)
        if value < 0:
            raise KeyError(name)
        return int(value)

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

    def capture_reference(self) -> Pose6D:
        """Capture the current simulated TCP and reset all derivative history."""

        current = self.current_tcp_pose
        self.initial_tcp = current
        self.last_safe_target = current
        self.last_safe_joint_target = self.arm_joints_rad
        self.last_safe_joint_velocity[:] = 0.0
        self.data.ctrl[self.arm_actuator_ids] = self.last_safe_joint_target
        self.tracking_errors_m.clear()
        return current

    def evaluate(self, target: Pose6D, *, dt_s: float) -> FeasibilityResult:
        limits = self.config.feasibility
        dt = max(float(dt_s), 1e-6)
        current_q = self.arm_joints_rad
        self.ik.set_arm_joints_rad(current_q.tolist())
        self.ik.apply_position_target(
            palm_target_position_m=list(target.position_m),
            palm_target_quaternion_wxyz=(
                [target.orientation_xyzw[3], *target.orientation_xyzw[:3]]
                if self.config.mapping.orientation_enabled
                else None
            ),
            wrist_roll_velocity_rad_s=0.0,
            dt=dt,
        )
        candidate_q = self.ik.arm_joints_rad.copy()
        joint_velocity = (candidate_q - self.last_safe_joint_target) / dt
        joint_acceleration = (joint_velocity - self.last_safe_joint_velocity) / dt
        target_delta = np.asarray(target.position_m) - np.asarray(self.last_safe_target.position_m)
        displacement = float(
            np.linalg.norm(np.asarray(target.position_m) - np.asarray(self.initial_tcp.position_m))
        )
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacBody(self.model, self.ik.data, jacp, jacr, self.palm_body_id)
        singular_values = np.linalg.svd(jacp[:, self.arm_dof_ids], compute_uv=False)
        condition = float(singular_values[0] / max(singular_values[-1], 1e-12))
        new_contacts = self._contact_pairs(self.ik.data) - self._baseline_contacts
        self_collision = any(self._pair_kind(pair) == "self" for pair in new_contacts)
        environment_collision = any(
            self._pair_kind(pair) == "environment" for pair in new_contacts
        )
        contact_distances = [
            float(self.ik.data.contact[index].dist)
            for index in range(self.ik.data.ncon)
            if self._contact_pair(self.ik.data, index) in new_contacts
        ]
        limit_blockers = joint_limit_margin_blockers(
            candidate_q, margin_rad=limits.joint_limit_margin_rad
        )
        if self.ik.joint_limit_limited:
            limit_blockers.extend(
                f"joint_{index}_clipped_to_safe_limit"
                for index in self.ik.limited_joint_indices_1_based
            )
        metrics = CandidateMetrics(
            target_displacement_m=displacement,
            target_jump_m=float(np.linalg.norm(target_delta)),
            tcp_velocity_m_s=float(np.linalg.norm(target_delta)) / dt,
            tcp_angular_velocity_rad_s=_quaternion_angle(
                target.orientation_xyzw, self.last_safe_target.orientation_xyzw
            )
            / dt,
            ik_error_m=self.ik.target_error_m,
            joint_limit_blockers=tuple(limit_blockers),
            jacobian_condition=condition,
            minimum_jacobian_singular_value=float(singular_values[-1]),
            maximum_joint_velocity_rad_s=float(np.max(np.abs(joint_velocity))),
            maximum_joint_acceleration_rad_s2=float(np.max(np.abs(joint_acceleration))),
            self_collision=self_collision,
            environment_collision=environment_collision,
            minimum_new_contact_distance_m=min(contact_distances) if contact_distances else None,
        )
        reason = classify_candidate(metrics, limits)
        if reason is FeasibilityReason.ACCEPTED:
            self.last_safe_joint_target = candidate_q
            self.last_safe_joint_velocity = joint_velocity
            self.last_safe_target = target
            self.data.ctrl[self.arm_actuator_ids] = candidate_q
            self.accepted_metrics.append(metrics)
            return FeasibilityResult(True, reason, tuple(float(v) for v in candidate_q), metrics)
        return FeasibilityResult(False, reason, None, metrics)

    def step(self, dt_s: float) -> None:
        steps = max(0, int(round(max(0.0, dt_s) / self.model.opt.timestep)))
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
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
        if self.desired_marker_mocap_id >= 0:
            self.data.mocap_pos[self.desired_marker_mocap_id] = (
                actual.position_m if desired is None else desired.position_m
            )
        mujoco.mj_forward(self.model, self.data)

    def metrics_report(self) -> dict[str, Any]:
        metrics = self.accepted_metrics
        return {
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
            "maximum_desired_to_simulated_tcp_error_m": max(
                self.tracking_errors_m, default=0.0
            ),
        }

    def _contact_pairs(self, data: mujoco.MjData) -> set[tuple[int, int]]:
        return {self._contact_pair(data, index) for index in range(data.ncon)}

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
        self._manual_request: str | None = None
        self._scheduled_capture_pending = False
        self.event_records: list[dict[str, Any]] = []

    def request_toggle(self) -> None:
        """SPACE-key control for live simulation: engage, capture, or disengage."""

        if self.operator.state is OperatorInputState.DISENGAGED:
            self._manual_request = "engage"
        elif self.operator.state is OperatorInputState.ARMED_REFERENCE_CAPTURE:
            self._manual_request = "capture"
        else:
            self._manual_request = "disengage"

    def tick(self, now_monotonic_ns: int) -> None:
        state = self.assembler.state(now_monotonic_ns=now_monotonic_ns)
        self.right_hand_valid = state.right.tracking_valid
        before = len(self.operator.transitions)
        self.operator.step(state)
        for transition in self.operator.transitions[before:]:
            if transition.current is OperatorInputState.DISENGAGED:
                self.mapper.clear_reference()
                self.invalid_input_events += 1
                self.rejections[FeasibilityReason.INPUT_INVALID.value] += 1
                self.last_reason = transition.reason

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
        disengage = False
        if self._manual_request == "engage" and right.tracking_valid:
            engage = True
            self._armed_at_sequence = right.host_sequence_number
            self._scheduled_capture_pending = False
            self._manual_request = None
        elif self._manual_request == "capture" and new_right and right.tracking_valid:
            capture = True
            self._manual_request = None
        elif self._manual_request == "disengage":
            disengage = True
            self._manual_request = None
        if (
            not engage
            and not capture
            and not disengage
            and
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
            disengage_request=disengage,
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
