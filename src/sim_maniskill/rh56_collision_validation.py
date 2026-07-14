from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import mujoco
import numpy as np
import yaml

from sim_maniskill.rh56_collision import REVIEWED_INTERNAL_EXCLUDED_BODY_PAIRS

HAND_ACTUATOR_NAMES: tuple[str, ...] = (
    "rh56_R_thumb_MCP_joint1_act",
    "rh56_R_thumb_MCP_joint2_act",
    "rh56_R_index_MCP_joint_act",
    "rh56_R_middle_MCP_joint_act",
    "rh56_R_ring_MCP_joint_act",
    "rh56_R_pinky_MCP_joint_act",
)
ARM_ACTUATOR_NAMES: tuple[str, ...] = tuple(f"jaka_joint_{idx}_act" for idx in range(1, 7))
ACTUATOR_NAMES: tuple[str, ...] = (
    "thumb_lateral",
    "thumb_close",
    "index",
    "middle",
    "ring",
    "pinky",
)
PHYSICAL_NORM_ORDER: tuple[str, ...] = ("pinky", "ring", "middle", "index", "thumb_close", "thumb_lateral")
ACTUATOR_CLOSE_CTRL: np.ndarray = np.asarray([1.10, 0.50, 1.70, 1.68, 1.70, 1.70], dtype=np.float64)

CANONICAL_PHYSICAL_POSES: dict[str, list[float]] = {
    "open": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "thumb_rotate": [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    "real_pinch_v4": [0.0, 0.0, 0.12, 0.15, 0.40, 1.0],
    "sim_best_pinch": [0.10, 0.10, 0.55, 0.60, 0.68, 1.0],
    "power_close": [0.75, 0.75, 0.80, 0.80, 0.55, 0.65],
}

REVIEWED_INTERNAL_BODY_PAIR_SET = {
    tuple(sorted(pair)) for pair in REVIEWED_INTERNAL_EXCLUDED_BODY_PAIRS
}

NON_BLOCKING_CONTACT_CATEGORIES = {
    "arm_contact",
    "hand_object_contact",
    "object_table_contact",
    "reviewed_excluded_internal_pair",
}

TARGET_SEMANTICS = {
    "free_space_target",
    "contact_terminated_target",
    "object_required_target",
    "diagnostic_only",
}

REPRESENTATION_COMPARISON_LABELS = {
    "visual_consistent_blocking_reference_modes_permissive",
    "confirmed_coacd_outward_approximation",
    "shared_visual_or_kinematic_intersection",
    "contact_timing_difference_near_visual_touching",
    "collision_model_missed_visual_intersection",
    "inconclusive",
}


@dataclass(frozen=True, slots=True)
class CommandProfile:
    name: str
    max_velocity_ctrl_per_s: tuple[float, ...]
    max_accel_ctrl_per_s2: tuple[float, ...]
    settle_seconds: float = 0.20
    hold_seconds: float = 0.25
    timeout_seconds: float = 4.0
    error_tolerance_ctrl: float = 0.035
    progress_window_seconds: float = 0.25
    progress_epsilon_ctrl: float = 0.003
    persistent_contact_seconds: float = 0.15
    transient_contact_seconds: float = 0.08
    force_blockage_threshold: float = 8.0
    hybrid_slowdown_error_ctrl: float = 0.18
    hybrid_near_contact_scale: float = 0.35


def default_command_profiles() -> dict[str, CommandProfile]:
    # The repository defines hardware speed registers in raw 0..1000 units, but
    # does not define a conversion from those units to radians per second. The
    # nominal profile is therefore derived from the live teleop software policy:
    # delta_limit=0.05 normalized units at command_hz=15, mapped through the
    # active MuJoCo actuator ranges.
    nominal_norm_per_s = 0.05 * 15.0
    slow_norm_per_s = 0.18
    fast_norm_per_s = nominal_norm_per_s
    stress_norm_per_s = 1.50
    return {
        "slow_validation": CommandProfile(
            name="slow_validation",
            max_velocity_ctrl_per_s=tuple((ACTUATOR_CLOSE_CTRL * slow_norm_per_s).tolist()),
            max_accel_ctrl_per_s2=tuple((ACTUATOR_CLOSE_CTRL * 0.90).tolist()),
            timeout_seconds=6.0,
        ),
        "nominal": CommandProfile(
            name="nominal",
            max_velocity_ctrl_per_s=tuple((ACTUATOR_CLOSE_CTRL * nominal_norm_per_s).tolist()),
            max_accel_ctrl_per_s2=tuple((ACTUATOR_CLOSE_CTRL * 2.00).tolist()),
            timeout_seconds=4.0,
        ),
        "hybrid": CommandProfile(
            name="hybrid",
            max_velocity_ctrl_per_s=tuple((ACTUATOR_CLOSE_CTRL * fast_norm_per_s).tolist()),
            max_accel_ctrl_per_s2=tuple((ACTUATOR_CLOSE_CTRL * 2.00).tolist()),
            timeout_seconds=5.0,
        ),
        "stress": CommandProfile(
            name="stress",
            max_velocity_ctrl_per_s=tuple((ACTUATOR_CLOSE_CTRL * stress_norm_per_s).tolist()),
            max_accel_ctrl_per_s2=tuple((ACTUATOR_CLOSE_CTRL * 4.00).tolist()),
            timeout_seconds=2.5,
        ),
    }


@dataclass(frozen=True, slots=True)
class MotionStage:
    name: str
    target_ctrl: tuple[float, ...]
    active_axes: tuple[int, ...] = tuple(range(6))


@dataclass(frozen=True, slots=True)
class ContactClassification:
    category: str
    region1: str
    region2: str
    relation: str
    severity: str
    reviewed: bool = False
    reason: str = ""

    @property
    def forbidden(self) -> bool:
        return self.severity == "forbidden"


@dataclass(slots=True)
class ContactRecord:
    time: float
    stage_index: int
    stage: str
    stage_elapsed_s: float
    geom1: str
    geom2: str
    body1: str
    body2: str
    region1: str
    region2: str
    category: str
    severity: str
    dist: float
    pos: list[float]
    normal: list[float]
    normal_force: float
    friction_force: float
    constraint_force_norm: float


@dataclass(slots=True)
class SampleRecord:
    step: int
    time: float
    stage_index: int
    stage: str
    stage_elapsed_s: float
    ctrl: list[float]
    target_ctrl: list[float]
    measured_ctrl_qpos: list[float]
    qpos: list[float]
    qvel: list[float]
    target_error: float
    stage_target_error: float
    command_remaining: float
    ctrl_tracking_error: float
    progress_window_ctrl: float | None
    controller_still_pushing: bool
    speed_scale: float
    qvel_norm: float
    ncon: int
    active_rh56_contact_count: int
    min_contact_dist: float | None
    max_normal_force: float
    constraint_force_norm: float
    actuator_force: list[float]
    warning_count: int


@dataclass(slots=True)
class TrajectoryResult:
    collision_mode: str
    target_name: str
    strategy: str
    profile: str
    initial_ctrl: list[float]
    target_ctrl: list[float]
    outcome: str
    blockage_kind: str
    reached: bool
    blocked: bool
    timeout: bool
    slow_progress: bool
    numerical_instability: bool
    first_contact_time: float | None
    first_contact_pair: list[str] | None
    first_thumb_index_contact_time: float | None
    first_persistent_thumb_index_contact_time: float | None
    first_blockage_time: float | None
    target_reached_time: float | None
    first_blocking_pair: list[str] | None
    first_forbidden_pair: list[str] | None
    max_penetration_m: float
    max_rh56_self_penetration_m: float
    max_thumb_index_penetration_m: float
    max_normal_force: float
    max_thumb_index_normal_force: float
    final_target_error: float
    unknown_self_pairs: list[list[str]] = field(default_factory=list)
    unreviewed_self_pairs: list[list[str]] = field(default_factory=list)
    events: list[dict[str, Any]] = field(default_factory=list)
    samples: list[SampleRecord] = field(default_factory=list)
    contacts: list[ContactRecord] = field(default_factory=list)
    states: dict[str, Any] = field(default_factory=dict)
    assumptions: dict[str, Any] = field(default_factory=dict)

    def summary_dict(self, *, include_samples: bool = False, include_contacts: bool = False) -> dict[str, Any]:
        payload = {
            "collision_mode": self.collision_mode,
            "target_name": self.target_name,
            "strategy": self.strategy,
            "profile": self.profile,
            "initial_ctrl": self.initial_ctrl,
            "target_ctrl": self.target_ctrl,
            "outcome": self.outcome,
            "blockage_kind": self.blockage_kind,
            "reached": self.reached,
            "blocked": self.blocked,
            "timeout": self.timeout,
            "slow_progress": self.slow_progress,
            "numerical_instability": self.numerical_instability,
            "first_contact_time": self.first_contact_time,
            "first_contact_pair": self.first_contact_pair,
            "first_thumb_index_contact_time": self.first_thumb_index_contact_time,
            "first_persistent_thumb_index_contact_time": self.first_persistent_thumb_index_contact_time,
            "first_blockage_time": self.first_blockage_time,
            "target_reached_time": self.target_reached_time,
            "first_blocking_pair": self.first_blocking_pair,
            "first_forbidden_pair": self.first_forbidden_pair,
            "max_penetration_m": self.max_penetration_m,
            "max_rh56_self_penetration_m": self.max_rh56_self_penetration_m,
            "max_thumb_index_penetration_m": self.max_thumb_index_penetration_m,
            "max_normal_force": self.max_normal_force,
            "max_thumb_index_normal_force": self.max_thumb_index_normal_force,
            "final_target_error": self.final_target_error,
            "unknown_self_pairs": self.unknown_self_pairs,
            "unreviewed_self_pairs": self.unreviewed_self_pairs,
            "events": self.events,
            "states": self.states,
            "assumptions": self.assumptions,
        }
        if include_samples:
            payload["samples"] = [asdict(row) for row in self.samples]
        if include_contacts:
            payload["contacts"] = [asdict(row) for row in self.contacts]
        return payload


def physical_norm_to_mujoco_ctrl(values: Sequence[float]) -> np.ndarray:
    if len(values) != 6:
        raise ValueError("Expected 6 RH56 physical normalized values.")
    pinky, ring, middle, index, thumb_close, thumb_lateral = [float(np.clip(value, 0.0, 1.0)) for value in values]
    return np.asarray(
        [
            1.10 * thumb_lateral,
            0.50 * thumb_close,
            1.70 * index,
            1.68 * middle,
            1.70 * ring,
            1.70 * pinky,
        ],
        dtype=np.float64,
    )


def canonical_target_ctrl(name: str) -> np.ndarray:
    try:
        return physical_norm_to_mujoco_ctrl(CANONICAL_PHYSICAL_POSES[name])
    except KeyError as exc:
        raise KeyError(f"Unknown RH56 canonical target {name!r}; choices={sorted(CANONICAL_PHYSICAL_POSES)}") from exc


def load_trajectory_manifest(path: str | Path) -> dict[str, Any]:
    manifest_path = Path(path)
    payload = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"Trajectory manifest {manifest_path} must contain a mapping.")
    validate_trajectory_manifest(payload)
    return payload


def validate_trajectory_manifest(payload: Mapping[str, Any]) -> None:
    trajectories = payload.get("trajectories")
    if not isinstance(trajectories, dict) or not trajectories:
        raise ValueError("Trajectory manifest must define a non-empty trajectories mapping.")
    required_names = {"open", "thumb_rotate", "real_pinch_v4", "sim_best_pinch", "power_close"}
    missing = required_names - set(trajectories)
    if missing:
        raise ValueError(f"Trajectory manifest is missing required entries: {sorted(missing)}")
    required_fields = {
        "initial_command",
        "target_command",
        "motion_orders",
        "speed_profiles",
        "target_semantics",
        "expected_terminal_condition",
        "allowed_semantic_contact_regions",
        "forbidden_structural_regions",
        "validation_status",
        "evidence",
    }
    for name, entry in trajectories.items():
        if not isinstance(entry, dict):
            raise ValueError(f"Trajectory {name!r} must be a mapping.")
        absent = required_fields - set(entry)
        if absent:
            raise ValueError(f"Trajectory {name!r} is missing fields: {sorted(absent)}")
        semantics = entry["target_semantics"]
        if semantics not in TARGET_SEMANTICS:
            raise ValueError(
                f"Trajectory {name!r} has target_semantics={semantics!r}; choices={sorted(TARGET_SEMANTICS)}"
            )
        for command_field in ("initial_command", "target_command"):
            command = entry[command_field]
            if not isinstance(command, list) or len(command) != 6:
                raise ValueError(f"Trajectory {name!r} {command_field} must be a six-element list.")
        if name in CANONICAL_PHYSICAL_POSES and not np.allclose(
            np.asarray(entry["target_command"], dtype=np.float64),
            np.asarray(CANONICAL_PHYSICAL_POSES[name], dtype=np.float64),
        ):
            raise ValueError(f"Trajectory {name!r} target_command does not match the canonical target.")


def classify_empty_hand_terminal_contact(
    *,
    target_semantics: str,
    contact_category: str,
    regions: Sequence[str],
    vendor_visuals_touch_or_intersect: bool,
    forbidden_structural_contact: bool,
    tunnelling: bool,
    numerical_instability: bool,
) -> dict[str, Any]:
    if target_semantics not in TARGET_SEMANTICS:
        raise ValueError(f"Unknown target semantics {target_semantics!r}.")
    distal_only = bool(regions) and all(region == "fingertip_pad" for region in regions)
    reviewed_distal = contact_category == "legitimate_fingertip_or_pad_contact" and distal_only
    physically_consistent = bool(vendor_visuals_touch_or_intersect and reviewed_distal)
    clean = not (forbidden_structural_contact or tunnelling or numerical_instability)
    terminal_semantics = target_semantics in {"contact_terminated_target", "object_required_target"}
    if physically_consistent and clean and terminal_semantics:
        return {
            "classification": "expected_terminal_hand_contact",
            "successful_free_space_reach": False,
            "terminal_contact": True,
            "reason": "Reviewed distal contact is visual-consistent and terminates a contact-mediated target.",
        }
    if physically_consistent and clean and target_semantics == "diagnostic_only":
        return {
            "classification": "terminal_contact_candidate_unresolved_target_semantics",
            "successful_free_space_reach": False,
            "terminal_contact": True,
            "reason": "Contact geometry is terminal-contact compatible, but target intent is unresolved.",
        }
    return {
        "classification": "not_expected_terminal_hand_contact",
        "successful_free_space_reach": False,
        "terminal_contact": False,
        "reason": "The reviewed distal, visual-consistency, safety, or target-semantics conditions are not all met.",
    }


def motion_stages(strategy: str, initial_ctrl: Sequence[float], target_ctrl: Sequence[float]) -> list[MotionStage]:
    start = np.asarray(initial_ctrl, dtype=np.float64)
    target = np.asarray(target_ctrl, dtype=np.float64)
    if start.shape != (6,) or target.shape != (6,):
        raise ValueError("initial_ctrl and target_ctrl must be 6D actuator-space vectors.")
    if strategy == "simultaneous":
        return [MotionStage("simultaneous", tuple(target.tolist()))]
    if strategy == "thumb_first":
        thumb_target = start.copy()
        thumb_target[[0, 1]] = target[[0, 1]]
        return [
            MotionStage("thumb_first_thumb", tuple(thumb_target.tolist()), (0, 1)),
            MotionStage("thumb_first_fingers", tuple(target.tolist()), (2, 3, 4, 5)),
        ]
    if strategy == "finger_first":
        finger_target = start.copy()
        finger_target[[2, 3, 4, 5]] = target[[2, 3, 4, 5]]
        return [
            MotionStage("finger_first_fingers", tuple(finger_target.tolist()), (2, 3, 4, 5)),
            MotionStage("finger_first_thumb", tuple(target.tolist()), (0, 1)),
        ]
    if strategy == "iterative_incremental":
        stages: list[MotionStage] = []
        for idx, fraction in enumerate((0.25, 0.50, 0.75, 1.00), start=1):
            staged = start + fraction * (target - start)
            stages.append(MotionStage(f"iterative_{idx}", tuple(staged.tolist())))
        return stages
    raise ValueError("strategy must be one of simultaneous, thumb_first, finger_first, iterative_incremental")


def _name(model: mujoco.MjModel, obj_type: mujoco.mjtObj, idx: int) -> str:
    return mujoco.mj_id2name(model, obj_type, int(idx)) or ""


def _geom_name(model: mujoco.MjModel, geom_id: int) -> str:
    return _name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)


def _body_name_for_geom(model: mujoco.MjModel, geom_id: int) -> str:
    body_id = int(model.geom_bodyid[int(geom_id)])
    return _name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)


def hand_region(body_name: str, geom_name: str = "") -> str:
    joined = f"{body_name} {geom_name}"
    if body_name.startswith("jaka_") or body_name in {"link0", "link1"} or "jaka_" in joined:
        return "arm"
    if geom_name in {"bench_object", "stage2_object"} or body_name in {"bench_object_body", "stage2_object_body"}:
        return "object"
    if geom_name in {"bench_table", "floor", "stage2_table"}:
        return "table"
    if not body_name.startswith("rh56_R_"):
        return "unknown"
    if body_name == "rh56_R_hand_base_link":
        return "palm"
    if "thumb_proximal_base" in body_name:
        return "thumb_base_joint_region"
    if body_name.endswith("_distal"):
        return "fingertip_pad"
    if "intermediate" in body_name:
        return "middle_segment"
    if "proximal" in body_name:
        return "proximal_segment"
    return "hand_other"


def finger_group(body_name: str) -> str:
    for group in ("thumb", "index", "middle", "ring", "pinky"):
        if f"_{group}_" in body_name or body_name.endswith(f"_{group}"):
            return group
    if body_name == "rh56_R_hand_base_link":
        return "palm"
    if body_name.startswith("rh56_R_"):
        return "hand"
    if body_name.startswith("jaka_"):
        return "arm"
    return ""


def classify_body_geom_pair(geom1: str, body1: str, geom2: str, body2: str) -> ContactClassification:
    region1 = hand_region(body1, geom1)
    region2 = hand_region(body2, geom2)
    regions = {region1, region2}
    pair = tuple(sorted((body1, body2)))
    group1 = finger_group(body1)
    group2 = finger_group(body2)
    groups = {group1, group2}

    if pair in REVIEWED_INTERNAL_BODY_PAIR_SET:
        return ContactClassification(
            "reviewed_excluded_internal_pair",
            region1,
            region2,
            "reviewed_internal",
            "allowed",
            reviewed=True,
            reason="Pair is an explicitly reviewed hidden adjacent-finger structural pair.",
        )
    if "object" in regions and "table" in regions:
        return ContactClassification("object_table_contact", region1, region2, "environment", "allowed")
    if "object" in regions and any(region not in {"object", "table", "unknown"} for region in regions):
        return ContactClassification("hand_object_contact", region1, region2, "external_contact", "allowed")
    if "table" in regions and any(region.startswith("finger") or region in {"palm", "proximal_segment", "middle_segment"} for region in regions):
        return ContactClassification("hand_table_contact", region1, region2, "environment", "review")
    if "arm" in regions:
        return ContactClassification("arm_contact", region1, region2, "arm_or_mount", "review")
    if not body1.startswith("rh56_R_") or not body2.startswith("rh56_R_"):
        return ContactClassification("unknown_or_unreviewed_pair", region1, region2, "unknown", "review")

    if "palm" in groups and any(group in {"thumb", "index", "middle", "ring", "pinky"} for group in groups):
        return ContactClassification("finger_palm_contact", region1, region2, "hand_self", "review")
    if group1 == group2:
        return ContactClassification("internal_joint_region_contact", region1, region2, "same_digit", "forbidden")
    if "fingertip_pad" in regions and groups <= {"thumb", "index", "middle", "ring", "pinky"}:
        if groups == {"thumb", "index"}:
            return ContactClassification("legitimate_fingertip_or_pad_contact", region1, region2, "thumb_index_pad", "allowed")
        return ContactClassification("finger_finger_pad_contact", region1, region2, "finger_finger", "review")
    if groups == {"thumb", "index"}:
        if "thumb_base_joint_region" in regions or "proximal_segment" in regions:
            return ContactClassification("proximal_or_dorsal_structural_contact", region1, region2, "thumb_index_structural", "forbidden")
        return ContactClassification("thumb_index_path_region_contact", region1, region2, "thumb_index_path", "review")
    if groups <= {"thumb", "index", "middle", "ring", "pinky"}:
        return ContactClassification("proximal_or_dorsal_structural_contact", region1, region2, "finger_finger_structural", "forbidden")
    return ContactClassification("unknown_or_unreviewed_pair", region1, region2, "unknown", "review")


def classify_contact(model: mujoco.MjModel, contact: mujoco.MjContact) -> ContactClassification:
    geom1 = _geom_name(model, int(contact.geom1))
    geom2 = _geom_name(model, int(contact.geom2))
    body1 = _body_name_for_geom(model, int(contact.geom1))
    body2 = _body_name_for_geom(model, int(contact.geom2))
    return classify_body_geom_pair(geom1, body1, geom2, body2)


def actuator_ids(model: mujoco.MjModel, names: Sequence[str] = HAND_ACTUATOR_NAMES) -> np.ndarray:
    ids: list[int] = []
    for name in names:
        idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if idx < 0:
            raise KeyError(f"Missing actuator {name}")
        ids.append(idx)
    return np.asarray(ids, dtype=np.int32)


def actuator_qpos_ids(model: mujoco.MjModel, ids: Sequence[int]) -> np.ndarray:
    qpos_ids: list[int] = []
    for actuator_id in ids:
        joint_id = int(model.actuator_trnid[int(actuator_id), 0])
        if joint_id < 0:
            raise RuntimeError(f"Actuator {actuator_id} is not attached to a joint.")
        qpos_ids.append(int(model.jnt_qposadr[joint_id]))
    return np.asarray(qpos_ids, dtype=np.int32)


def warning_count(data: mujoco.MjData) -> int:
    try:
        return int(sum(int(item.number) for item in data.warning))
    except Exception:
        return 0


def _contact_force(model: mujoco.MjModel, data: mujoco.MjData, contact_index: int) -> tuple[float, float]:
    force = np.zeros(6, dtype=np.float64)
    try:
        mujoco.mj_contactForce(model, data, int(contact_index), force)
    except Exception:
        return 0.0, 0.0
    return float(force[0]), float(np.linalg.norm(force[1:3]))


def _constraint_force_norm(data: mujoco.MjData) -> float:
    try:
        return float(np.linalg.norm(np.asarray(data.efc_force, dtype=np.float64)))
    except Exception:
        return 0.0


def _is_thumb_index_pair(body1: str, body2: str) -> bool:
    return {finger_group(body1), finger_group(body2)} == {"thumb", "index"}


def _has_relevant_rh56_contact(model: mujoco.MjModel, data: mujoco.MjData) -> bool:
    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        body1 = _body_name_for_geom(model, int(contact.geom1))
        body2 = _body_name_for_geom(model, int(contact.geom2))
        if body1.startswith("rh56_R_") or body2.startswith("rh56_R_"):
            classification = classify_contact(model, contact)
            if classification.category not in {"arm_contact", "reviewed_excluded_internal_pair"}:
                return True
    return False


def _step_command(
    current: np.ndarray,
    velocity: np.ndarray,
    target: np.ndarray,
    axes: Iterable[int],
    profile: CommandProfile,
    dt: float,
    speed_scale: float = 1.0,
) -> tuple[np.ndarray, np.ndarray]:
    axes_array = np.asarray(list(axes), dtype=np.int32)
    new = current.copy()
    new_velocity = velocity.copy()
    max_vel = np.asarray(profile.max_velocity_ctrl_per_s, dtype=np.float64) * float(speed_scale)
    max_accel = np.asarray(profile.max_accel_ctrl_per_s2, dtype=np.float64) * float(speed_scale)
    desired_velocity = np.zeros(6, dtype=np.float64)
    error = target - current
    desired_velocity[axes_array] = np.clip(
        error[axes_array] / max(dt, 1e-9),
        -max_vel[axes_array],
        max_vel[axes_array],
    )
    dv = np.clip(desired_velocity - velocity, -max_accel * dt, max_accel * dt)
    new_velocity += dv
    step = new_velocity * dt
    for axis in axes_array:
        if abs(step[axis]) > abs(error[axis]):
            step[axis] = error[axis]
            new_velocity[axis] = 0.0
    new += step
    return new, new_velocity


def _hybrid_speed_scale(
    profile: CommandProfile,
    target: np.ndarray,
    current: np.ndarray,
    relevant_contact: bool,
) -> float:
    if profile.name != "hybrid":
        return 1.0
    target_error = float(np.linalg.norm(target - current, ord=np.inf))
    if relevant_contact or target_error <= profile.hybrid_slowdown_error_ctrl:
        return profile.hybrid_near_contact_scale
    return 1.0


def _state_payload(
    data: mujoco.MjData,
    hand_qpos_ids: np.ndarray,
    hand_ids: np.ndarray,
    *,
    stage_index: int | None = None,
    stage: str | None = None,
    stage_elapsed_s: float | None = None,
    stage_target: np.ndarray | None = None,
    final_target: np.ndarray | None = None,
    progress_window_ctrl: float | None = None,
    controller_still_pushing: bool | None = None,
    active_rh56_contacts: Sequence[Mapping[str, Any]] = (),
) -> dict[str, Any]:
    measured = np.asarray(data.qpos[hand_qpos_ids], dtype=np.float64)
    payload: dict[str, Any] = {
        "time": float(data.time),
        "qpos": np.asarray(data.qpos, dtype=np.float64).tolist(),
        "qvel": np.asarray(data.qvel, dtype=np.float64).tolist(),
        "ctrl": np.asarray(data.ctrl, dtype=np.float64).tolist(),
        "measured_ctrl_qpos": measured.tolist(),
        "actuator_force": np.asarray(data.actuator_force[hand_ids], dtype=np.float64).tolist(),
        "constraint_force_norm": _constraint_force_norm(data),
        "warning_count": warning_count(data),
        "active_rh56_contacts": [dict(row) for row in active_rh56_contacts],
    }
    if stage_index is not None:
        payload["stage_index"] = int(stage_index)
    if stage is not None:
        payload["stage"] = stage
    if stage_elapsed_s is not None:
        payload["stage_elapsed_s"] = float(stage_elapsed_s)
    if stage_target is not None:
        payload["stage_target_ctrl"] = stage_target.tolist()
        payload["stage_target_error"] = float(np.linalg.norm(stage_target - measured, ord=np.inf))
        payload["command_stage_remaining"] = float(
            np.linalg.norm(stage_target - np.asarray(data.ctrl[hand_ids], dtype=np.float64), ord=np.inf)
        )
    if final_target is not None:
        payload["final_target_ctrl"] = final_target.tolist()
        payload["final_target_error"] = float(np.linalg.norm(final_target - measured, ord=np.inf))
        payload["command_final_remaining"] = float(
            np.linalg.norm(final_target - np.asarray(data.ctrl[hand_ids], dtype=np.float64), ord=np.inf)
        )
    payload["progress_window_ctrl"] = progress_window_ctrl
    payload["controller_still_pushing"] = controller_still_pushing
    return payload


def run_trajectory_validation(
    model: mujoco.MjModel,
    *,
    collision_mode: str,
    target_name: str,
    target_ctrl: Sequence[float],
    strategy: str = "simultaneous",
    profile: CommandProfile | None = None,
    initial_ctrl: Sequence[float] | None = None,
    arm_qpos: Sequence[float] | None = None,
    sample_stride: int = 1,
    hold_after_persistent_distal_contact_seconds: float | None = None,
) -> TrajectoryResult:
    profile = profile or default_command_profiles()["slow_validation"]
    data = mujoco.MjData(model)
    hand_ids = actuator_ids(model)
    hand_qpos_ids = actuator_qpos_ids(model, hand_ids)
    arm_ids = actuator_ids(model, ARM_ACTUATOR_NAMES)
    dt = float(model.opt.timestep)
    initial = np.zeros(6, dtype=np.float64) if initial_ctrl is None else np.asarray(initial_ctrl, dtype=np.float64)
    target = np.asarray(target_ctrl, dtype=np.float64)
    if initial.shape != (6,) or target.shape != (6,):
        raise ValueError("initial_ctrl and target_ctrl must be 6D actuator-space vectors.")

    mujoco.mj_resetData(model, data)
    if arm_qpos is not None:
        arm = np.asarray(arm_qpos, dtype=np.float64)
        if arm.shape != (6,):
            raise ValueError("arm_qpos must contain 6 JAKA joint positions.")
        data.qpos[:6] = arm
        data.ctrl[arm_ids] = arm
    data.qpos[hand_qpos_ids] = initial
    data.ctrl[hand_ids] = initial
    mujoco.mj_forward(model, data)
    for _ in range(max(0, int(math.ceil(profile.settle_seconds / dt)))):
        data.ctrl[hand_ids] = initial
        if arm_qpos is not None:
            data.ctrl[arm_ids] = np.asarray(arm_qpos, dtype=np.float64)
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    initial_state = _state_payload(data, hand_qpos_ids, hand_ids, final_target=target)
    previous_step_state = initial_state
    stages = motion_stages(strategy, initial, target)
    samples: list[SampleRecord] = []
    contacts: list[ContactRecord] = []
    events: list[dict[str, Any]] = []
    first_contact_time: float | None = None
    first_contact_pair: list[str] | None = None
    first_thumb_index_contact_time: float | None = None
    first_persistent_thumb_index_contact_time: float | None = None
    first_blockage_time: float | None = None
    target_reached_time: float | None = None
    first_blocking_pair: list[str] | None = None
    first_forbidden_pair: list[str] | None = None
    max_penetration = 0.0
    max_rh56_self_penetration = 0.0
    max_thumb_index_penetration = 0.0
    max_force = 0.0
    max_thumb_index_force = 0.0
    max_penetration_state: dict[str, Any] | None = None
    max_thumb_index_penetration_state: dict[str, Any] | None = None
    first_contact_state: dict[str, Any] | None = None
    before_first_thumb_index_contact_state: dict[str, Any] | None = None
    first_thumb_index_contact_state: dict[str, Any] | None = None
    first_persistent_thumb_index_contact_state: dict[str, Any] | None = None
    first_blockage_state: dict[str, Any] | None = None
    diagnostic_hold_start_state: dict[str, Any] | None = None
    diagnostic_hold_final_state: dict[str, Any] | None = None
    contact_duration_by_pair: dict[tuple[str, str], float] = {}
    max_consecutive_contact_by_pair: dict[tuple[str, str], float] = {}
    consecutive_blocking_contact_by_pair: dict[tuple[str, str], float] = {}
    last_contact_seen: set[tuple[str, str]] = set()
    unknown_pairs: set[tuple[str, str]] = set()
    unreviewed_pairs: set[tuple[str, str]] = set()
    blocked = False
    numerical_instability = False
    stage_timed_out = False
    slow_progress = False
    diagnostic_hold_active = False
    diagnostic_hold_completed = False
    diagnostic_hold_start_time: float | None = None
    diagnostic_hold_ctrl: np.ndarray | None = None
    current = initial.copy()
    velocity = np.zeros(6, dtype=np.float64)
    step_index = 0
    progress_window_steps = max(1, int(math.ceil(profile.progress_window_seconds / dt)))
    last_progress: float | None = None
    command_sequence: list[dict[str, Any]] = [
        {
            "stage_index": stage_index,
            "stage": stage.name,
            "target_ctrl": np.asarray(stage.target_ctrl, dtype=np.float64).round(8).tolist(),
            "active_axes": [int(axis) for axis in stage.active_axes],
            "active_axis_names": [ACTUATOR_NAMES[int(axis)] for axis in stage.active_axes],
        }
        for stage_index, stage in enumerate(stages)
    ]

    for stage_index, stage in enumerate(stages):
        stage_target = np.asarray(stage.target_ctrl, dtype=np.float64)
        stage_start_time = float(data.time)
        measured_history: list[np.ndarray] = []
        last_progress = None
        reached_hold_steps = 0
        stage_completed = False
        while float(data.time) - stage_start_time <= profile.timeout_seconds:
            relevant_contact = _has_relevant_rh56_contact(model, data)
            if diagnostic_hold_active:
                speed_scale = 0.0
                current = np.asarray(diagnostic_hold_ctrl, dtype=np.float64).copy()
                velocity.fill(0.0)
            else:
                speed_scale = _hybrid_speed_scale(profile, stage_target, current, relevant_contact)
                current, velocity = _step_command(
                    current,
                    velocity,
                    stage_target,
                    stage.active_axes,
                    profile,
                    dt,
                    speed_scale,
                )
            data.ctrl[hand_ids] = current
            if arm_qpos is not None:
                data.ctrl[arm_ids] = np.asarray(arm_qpos, dtype=np.float64)
            mujoco.mj_step(model, data)
            # mj_step integrates qpos after solving contacts. Recompute diagnostics
            # at the integrated qpos so saved contact records and same-qpos replay
            # describe exactly the same configuration.
            mujoco.mj_forward(model, data)
            step_index += 1
            stage_elapsed = float(data.time) - stage_start_time
            measured = np.asarray(data.qpos[hand_qpos_ids], dtype=np.float64).copy()
            measured_history.append(measured)
            if len(measured_history) > progress_window_steps + 1:
                measured_history.pop(0)

            current_warning_count = warning_count(data)
            if (
                not np.all(np.isfinite(data.qpos))
                or not np.all(np.isfinite(data.qvel))
                or current_warning_count > 0
            ):
                numerical_instability = True
                events.append(
                    {
                        "time": float(data.time),
                        "event": "numerical_instability_or_warning",
                        "warning_count": current_warning_count,
                    }
                )
                break

            contact_pairs_this_step: set[tuple[str, str]] = set()
            contact_rows: list[tuple[tuple[str, str], ContactClassification, ContactRecord]] = []
            representative_by_pair: dict[tuple[str, str], tuple[ContactClassification, ContactRecord]] = {}
            constraint_norm = _constraint_force_norm(data)
            min_dist: float | None = None
            step_max_force = 0.0
            step_max_penetration = 0.0
            step_max_rh56_penetration = 0.0
            step_max_thumb_index_penetration = 0.0
            for contact_index in range(data.ncon):
                contact = data.contact[contact_index]
                geom1 = _geom_name(model, int(contact.geom1))
                geom2 = _geom_name(model, int(contact.geom2))
                body1 = _body_name_for_geom(model, int(contact.geom1))
                body2 = _body_name_for_geom(model, int(contact.geom2))
                classification = classify_body_geom_pair(geom1, body1, geom2, body2)
                normal_force, friction_force = _contact_force(model, data, contact_index)
                max_force = max(max_force, normal_force)
                step_max_force = max(step_max_force, normal_force)
                dist = float(contact.dist)
                penetration = max(0.0, -dist)
                step_max_penetration = max(step_max_penetration, penetration)
                if min_dist is None or dist < min_dist:
                    min_dist = dist
                rh56_self = body1.startswith("rh56_R_") and body2.startswith("rh56_R_")
                if rh56_self:
                    step_max_rh56_penetration = max(step_max_rh56_penetration, penetration)
                thumb_index = _is_thumb_index_pair(body1, body2)
                if thumb_index:
                    step_max_thumb_index_penetration = max(step_max_thumb_index_penetration, penetration)
                    max_thumb_index_force = max(max_thumb_index_force, normal_force)
                pair = tuple(sorted((body1, body2)))
                contact_pairs_this_step.add(pair)
                if classification.category == "unknown_or_unreviewed_pair":
                    unknown_pairs.add(pair)
                if rh56_self and not classification.reviewed:
                    unreviewed_pairs.add(pair)
                record = ContactRecord(
                    time=float(data.time),
                    stage_index=stage_index,
                    stage=stage.name,
                    stage_elapsed_s=stage_elapsed,
                    geom1=geom1,
                    geom2=geom2,
                    body1=body1,
                    body2=body2,
                    region1=classification.region1,
                    region2=classification.region2,
                    category=classification.category,
                    severity=classification.severity,
                    dist=dist,
                    pos=np.asarray(contact.pos, dtype=np.float64).round(8).tolist(),
                    normal=np.asarray(contact.frame[:3], dtype=np.float64).round(8).tolist(),
                    normal_force=normal_force,
                    friction_force=friction_force,
                    constraint_force_norm=constraint_norm,
                )
                contact_rows.append((pair, classification, record))
                contacts.append(record)
                prior = representative_by_pair.get(pair)
                if prior is None or record.dist < prior[1].dist:
                    representative_by_pair[pair] = (classification, record)
                if first_contact_time is None:
                    first_contact_time = float(data.time)
                    first_contact_pair = [body1, body2]
                if classification.forbidden and first_forbidden_pair is None:
                    first_forbidden_pair = [body1, body2]
                    events.append(
                        {
                            "time": float(data.time),
                            "event": "forbidden_structural_collision",
                            "body_pair": first_forbidden_pair,
                            "category": classification.category,
                        }
                    )

            separated = last_contact_seen - contact_pairs_this_step
            for pair in sorted(separated):
                events.append({"time": float(data.time), "event": "separation", "body_pair": list(pair)})
                consecutive_blocking_contact_by_pair[pair] = 0.0
            for pair in contact_pairs_this_step:
                contact_duration_by_pair[pair] = contact_duration_by_pair.get(pair, 0.0) + dt
                classification, record = representative_by_pair[pair]
                if (
                    classification.category not in NON_BLOCKING_CONTACT_CATEGORIES
                    and record.body1.startswith("rh56_R_")
                    and record.body2.startswith("rh56_R_")
                ):
                    duration = consecutive_blocking_contact_by_pair.get(pair, 0.0) + dt
                    consecutive_blocking_contact_by_pair[pair] = duration
                    max_consecutive_contact_by_pair[pair] = max(
                        max_consecutive_contact_by_pair.get(pair, 0.0), duration
                    )
                    if first_blocking_pair is None and duration >= profile.persistent_contact_seconds:
                        first_blocking_pair = [record.body1, record.body2]
                        events.append(
                            {
                                "time": float(data.time),
                                "event": "sustained_contact",
                                "body_pair": first_blocking_pair,
                                "category": classification.category,
                                "consecutive_duration_s": duration,
                            }
                        )
            last_contact_seen = contact_pairs_this_step

            target_error = float(np.linalg.norm(target - measured, ord=np.inf))
            stage_target_error = float(np.linalg.norm(stage_target - measured, ord=np.inf))
            command_remaining = float(np.linalg.norm(stage_target - current, ord=np.inf))
            ctrl_tracking_error = float(np.linalg.norm(current - measured, ord=np.inf))
            qvel_norm = float(np.linalg.norm(np.asarray(data.qvel, dtype=np.float64)))
            progress = None
            if len(measured_history) >= progress_window_steps:
                progress = float(np.linalg.norm(measured_history[-1] - measured_history[0], ord=np.inf))
                last_progress = progress
            controller_still_pushing = bool(
                ctrl_tracking_error > profile.error_tolerance_ctrl
                or command_remaining > profile.error_tolerance_ctrl
            )
            active_rh56_contacts = [
                asdict(record)
                for _, _, record in contact_rows
                if record.body1.startswith("rh56_R_") or record.body2.startswith("rh56_R_")
            ]
            state = _state_payload(
                data,
                hand_qpos_ids,
                hand_ids,
                stage_index=stage_index,
                stage=stage.name,
                stage_elapsed_s=stage_elapsed,
                stage_target=stage_target,
                final_target=target,
                progress_window_ctrl=progress,
                controller_still_pushing=controller_still_pushing,
                active_rh56_contacts=active_rh56_contacts,
            )
            if first_contact_time == float(data.time) and first_contact_state is None:
                first_contact_state = state
                events.append(
                    {"time": float(data.time), "event": "contact_onset", "body_pair": first_contact_pair}
                )
            thumb_index_rows = [row for row in contact_rows if _is_thumb_index_pair(row[2].body1, row[2].body2)]
            if thumb_index_rows and first_thumb_index_contact_time is None:
                first_thumb_index_contact_time = float(data.time)
                before_first_thumb_index_contact_state = previous_step_state
                first_thumb_index_contact_state = state
                first_record = thumb_index_rows[0][2]
                events.append(
                    {
                        "time": float(data.time),
                        "event": "thumb_index_contact_onset",
                        "body_pair": [first_record.body1, first_record.body2],
                        "geom_pair": [first_record.geom1, first_record.geom2],
                        "command_distance_remaining": target_error,
                    }
                )
            persistent_thumb_index = any(
                _is_thumb_index_pair(*pair)
                and duration >= profile.persistent_contact_seconds
                for pair, duration in consecutive_blocking_contact_by_pair.items()
            )
            if persistent_thumb_index and first_persistent_thumb_index_contact_time is None:
                first_persistent_thumb_index_contact_time = float(data.time)
                first_persistent_thumb_index_contact_state = state
                events.append(
                    {
                        "time": float(data.time),
                        "event": "persistent_thumb_index_contact",
                        "command_distance_remaining": target_error,
                    }
                )
                persistent_distal = any(
                    row[1].category == "legitimate_fingertip_or_pad_contact"
                    and row[1].region1 == "fingertip_pad"
                    and row[1].region2 == "fingertip_pad"
                    for row in thumb_index_rows
                )
                if (
                    persistent_distal
                    and hold_after_persistent_distal_contact_seconds is not None
                    and hold_after_persistent_distal_contact_seconds > 0.0
                ):
                    diagnostic_hold_active = True
                    diagnostic_hold_start_time = float(data.time)
                    diagnostic_hold_ctrl = current.copy()
                    velocity.fill(0.0)
                    diagnostic_hold_start_state = state
                    events.append(
                        {
                            "time": float(data.time),
                            "event": "diagnostic_hold_after_persistent_distal_contact_started",
                            "held_ctrl": diagnostic_hold_ctrl.tolist(),
                            "requested_settle_seconds": hold_after_persistent_distal_contact_seconds,
                        }
                    )
            if step_max_penetration > max_penetration:
                max_penetration = step_max_penetration
                max_penetration_state = state
            max_rh56_self_penetration = max(max_rh56_self_penetration, step_max_rh56_penetration)
            if step_max_thumb_index_penetration > max_thumb_index_penetration:
                max_thumb_index_penetration = step_max_thumb_index_penetration
                max_thumb_index_penetration_state = state

            if step_index % max(1, sample_stride) == 0:
                samples.append(
                    SampleRecord(
                        step=step_index,
                        time=float(data.time),
                        stage_index=stage_index,
                        stage=stage.name,
                        stage_elapsed_s=stage_elapsed,
                        ctrl=current.round(8).tolist(),
                        target_ctrl=stage_target.round(8).tolist(),
                        measured_ctrl_qpos=measured.round(8).tolist(),
                        qpos=np.asarray(data.qpos, dtype=np.float64).round(8).tolist(),
                        qvel=np.asarray(data.qvel, dtype=np.float64).round(8).tolist(),
                        target_error=target_error,
                        stage_target_error=stage_target_error,
                        command_remaining=command_remaining,
                        ctrl_tracking_error=ctrl_tracking_error,
                        progress_window_ctrl=progress,
                        controller_still_pushing=controller_still_pushing,
                        speed_scale=speed_scale,
                        qvel_norm=qvel_norm,
                        ncon=int(data.ncon),
                        active_rh56_contact_count=len(active_rh56_contacts),
                        min_contact_dist=min_dist,
                        max_normal_force=step_max_force,
                        constraint_force_norm=constraint_norm,
                        actuator_force=np.asarray(data.actuator_force[hand_ids], dtype=np.float64).round(8).tolist(),
                        warning_count=current_warning_count,
                    )
                )

            if diagnostic_hold_active:
                assert diagnostic_hold_start_time is not None
                if (
                    float(data.time) - diagnostic_hold_start_time
                    >= float(hold_after_persistent_distal_contact_seconds)
                ):
                    diagnostic_hold_completed = True
                    diagnostic_hold_final_state = state
                    events.append(
                        {
                            "time": float(data.time),
                            "event": "diagnostic_hold_after_persistent_distal_contact_completed",
                            "settled_seconds": float(data.time) - diagnostic_hold_start_time,
                        }
                    )
                    previous_step_state = state
                    break
                previous_step_state = state
                continue

            if stage_target_error <= profile.error_tolerance_ctrl and command_remaining <= profile.error_tolerance_ctrl:
                reached_hold_steps += 1
                if reached_hold_steps >= max(1, int(math.ceil(profile.hold_seconds / dt))):
                    stage_completed = True
                    events.append(
                        {
                            "time": float(data.time),
                            "event": "stage_target_reached",
                            "stage_index": stage_index,
                            "stage": stage.name,
                            "stage_target_error": stage_target_error,
                        }
                    )
                    if stage_index == len(stages) - 1:
                        target_reached_time = float(data.time)
                        events.append(
                            {"time": float(data.time), "event": "target_reached", "target_error": target_error}
                        )
                    previous_step_state = state
                    break
            else:
                reached_hold_steps = 0

            persistent_contact = any(
                duration >= profile.persistent_contact_seconds
                for duration in consecutive_blocking_contact_by_pair.values()
            )
            if (
                progress is not None
                and persistent_contact
                and controller_still_pushing
                and stage_target_error > profile.error_tolerance_ctrl
                and progress <= profile.progress_epsilon_ctrl
                and (
                    constraint_norm >= profile.force_blockage_threshold
                    or step_max_force >= profile.force_blockage_threshold
                    or first_blocking_pair is not None
                )
            ):
                blocked = True
                first_blockage_time = float(data.time)
                first_blockage_state = state
                if first_blocking_pair is None and contact_rows:
                    record = contact_rows[0][2]
                    first_blocking_pair = [record.body1, record.body2]
                events.append(
                    {
                        "time": float(data.time),
                        "event": "stable_blockage",
                        "stage_target_error": stage_target_error,
                        "final_target_error": target_error,
                        "command_distance_remaining": target_error,
                        "progress_window_ctrl": progress,
                        "first_blocking_pair": first_blocking_pair,
                    }
                )
                previous_step_state = state
                break
            previous_step_state = state

        if numerical_instability or blocked or diagnostic_hold_completed:
            break
        if not stage_completed:
            stage_timed_out = True
            slow_progress = bool(last_progress is not None and last_progress > profile.progress_epsilon_ctrl)
            events.append(
                {
                    "time": float(data.time),
                    "event": "stage_timeout_slow_progress" if slow_progress else "stage_timeout_without_blockage",
                    "stage_index": stage_index,
                    "stage": stage.name,
                    "progress_window_ctrl": last_progress,
                }
            )
            break

    final_error = float(np.linalg.norm(target - np.asarray(data.qpos[hand_qpos_ids], dtype=np.float64), ord=np.inf))
    reached = bool(
        target_reached_time is not None
        and final_error <= profile.error_tolerance_ctrl
        and not blocked
        and not numerical_instability
    )
    timeout = bool(
        (stage_timed_out or not reached)
        and not blocked
        and not numerical_instability
        and not diagnostic_hold_completed
    )
    if first_forbidden_pair is not None:
        outcome = "forbidden_structural_collision"
        blockage_kind = "forbidden_structural_collision"
    elif diagnostic_hold_completed:
        outcome = "contact_terminated_settled"
        blockage_kind = "diagnostic_command_hold"
    elif reached:
        has_long_contact = any(
            duration >= profile.persistent_contact_seconds
            for duration in max_consecutive_contact_by_pair.values()
        )
        has_short_contact = any(
            0.0 < duration <= profile.transient_contact_seconds
            for duration in max_consecutive_contact_by_pair.values()
        )
        if has_short_contact and not has_long_contact:
            outcome = "transient_grazing_contact"
        else:
            outcome = "reached"
        blockage_kind = "none"
    elif blocked:
        outcome = "blocked"
        if first_blocking_pair and set(first_blocking_pair) <= {
            "rh56_R_thumb_distal",
            "rh56_R_thumb_intermediate",
            "rh56_R_index_distal",
            "rh56_R_index_proximal",
        }:
            blockage_kind = "expected_path_obstruction_candidate"
        else:
            blockage_kind = "persistent_mechanical_blockage"
    elif numerical_instability:
        outcome = "numerical_instability"
        blockage_kind = "unknown"
    elif slow_progress:
        outcome = "slow_progress"
        blockage_kind = "timeout_with_measured_progress"
    else:
        outcome = "timeout"
        blockage_kind = "timeout_without_stable_blockage"

    return TrajectoryResult(
        collision_mode=collision_mode,
        target_name=target_name,
        strategy=strategy,
        profile=profile.name,
        initial_ctrl=initial.round(8).tolist(),
        target_ctrl=target.round(8).tolist(),
        outcome=outcome,
        blockage_kind=blockage_kind,
        reached=reached,
        blocked=blocked,
        timeout=timeout,
        slow_progress=slow_progress,
        numerical_instability=numerical_instability,
        first_contact_time=first_contact_time,
        first_contact_pair=first_contact_pair,
        first_thumb_index_contact_time=first_thumb_index_contact_time,
        first_persistent_thumb_index_contact_time=first_persistent_thumb_index_contact_time,
        first_blockage_time=first_blockage_time,
        target_reached_time=target_reached_time,
        first_blocking_pair=first_blocking_pair if blocked else None,
        first_forbidden_pair=first_forbidden_pair,
        max_penetration_m=max_penetration,
        max_rh56_self_penetration_m=max_rh56_self_penetration,
        max_thumb_index_penetration_m=max_thumb_index_penetration,
        max_normal_force=max_force,
        max_thumb_index_normal_force=max_thumb_index_force,
        final_target_error=final_error,
        unknown_self_pairs=[list(pair) for pair in sorted(unknown_pairs)],
        unreviewed_self_pairs=[list(pair) for pair in sorted(unreviewed_pairs)],
        events=events,
        samples=samples,
        contacts=contacts,
        states={
            "initial_state": initial_state,
            "command_sequence": command_sequence,
            "first_contact_state": first_contact_state,
            "before_first_thumb_index_contact_state": before_first_thumb_index_contact_state,
            "first_thumb_index_contact_state": first_thumb_index_contact_state,
            "first_persistent_thumb_index_contact_state": first_persistent_thumb_index_contact_state,
            "first_blockage_state": first_blockage_state,
            "diagnostic_hold_start_state": diagnostic_hold_start_state,
            "diagnostic_hold_final_state": diagnostic_hold_final_state,
            "maximum_penetration_state": max_penetration_state,
            "maximum_thumb_index_penetration_state": max_thumb_index_penetration_state,
            "final_state": previous_step_state,
        },
        assumptions={
            "hardware_speed_units": (
                "Repository defines RH56 raw speed registers and defaults but no conversion from raw "
                "0..1000 speed units to rad/s."
            ),
            "nominal_profile_basis": (
                "Derived from teleop delta_limit=0.05 normalized units at command_hz=15, "
                "mapped through MuJoCo actuator ranges."
            ),
            "hybrid_profile_basis": (
                "Hybrid uses nominal free-space velocity and slows when command-space error is small "
                "or a relevant RH56 contact is present; unrelated arm-base contacts do not trigger slowdown."
            ),
            "iterative_stage_completion_rule": (
                "Each iterative stage must reach and hold its own stage target before the next stage begins."
            ),
            "dynamic_validation_rule": (
                "Intermediate hand motion uses position actuator controls plus repeated mujoco.mj_step; "
                "qpos writes are limited to reset/initial state setup."
            ),
            "visual_mesh_intersection_diagnostic": (
                "Not evaluated by this core trajectory runner; compare collision modes and optional "
                "external mesh diagnostics in the CLI report."
            ),
            "contact_response_variant": (
                "continuous_command"
                if hold_after_persistent_distal_contact_seconds is None
                else "hold_current_actuator_target_after_first_persistent_distal_thumb_index_contact"
            ),
            "contact_response_hold_seconds": hold_after_persistent_distal_contact_seconds,
        },
    )


def classify_representation_comparison(
    *,
    visual_coacd: Mapping[str, Any],
    references: Sequence[Mapping[str, Any]],
    original_visual_intersects: bool | None = None,
    original_visual_gap_m: float | None = None,
    meaningful_gap_m: float = 5e-4,
    near_touch_tolerance_m: float = 2e-4,
) -> dict[str, Any]:
    if not references:
        return {"classification": "inconclusive", "reason": "No comparison representations are available."}
    rows = [visual_coacd, *references]
    if any(bool(row.get("numerical_instability")) for row in rows):
        return {"classification": "inconclusive", "reason": "A compared trajectory was numerically unstable."}
    visual_blocks = bool(visual_coacd.get("blocked")) or visual_coacd.get("outcome") == "forbidden_structural_collision"
    refs_reach = all(bool(row.get("reached")) for row in references)
    if original_visual_intersects is True:
        if visual_blocks and refs_reach:
            return {
                "classification": "visual_consistent_blocking_reference_modes_permissive",
                "root_cause_classification": "shared_visual_or_kinematic_intersection",
                "reason": (
                    "visual_coacd blocks at a configuration where original visuals intersect; "
                    "reference reachability indicates permissive representations, not ground truth."
                ),
            }
        if all(bool(row.get("reached")) for row in rows):
            return {
                "classification": "collision_model_missed_visual_intersection",
                "reason": "All compared collision representations permit a state where original visuals intersect.",
            }
        return {
            "classification": "shared_visual_or_kinematic_intersection",
            "reason": "Original transformed vendor visual meshes intersect at the compared state.",
        }
    if original_visual_gap_m is not None:
        if original_visual_gap_m <= near_touch_tolerance_m:
            return {
                "classification": "contact_timing_difference_near_visual_touching",
                "reason": f"Original visual gap {original_visual_gap_m:.9f} m is within near-touch tolerance.",
            }
        if visual_blocks and refs_reach and original_visual_gap_m >= meaningful_gap_m:
            return {
                "classification": "confirmed_coacd_outward_approximation",
                "reason": (
                    "visual_coacd alone blocks while original visuals retain a meaningful positive gap "
                    "and both comparison representations remain permissive."
                ),
            }
    return {
        "classification": "inconclusive",
        "reason": (
            "Dynamic representation disagreement is not decisive without original-visual evidence; "
            "a reference mode reaching is not treated as ground truth."
        ),
    }


def _thumb_index_contacts_at_time(result: TrajectoryResult, time_s: float) -> list[ContactRecord]:
    return [
        row
        for row in result.contacts
        if abs(row.time - time_s) <= 1e-10 and _is_thumb_index_pair(row.body1, row.body2)
    ]


def summarize_post_contact_response(result: TrajectoryResult) -> dict[str, Any]:
    first_time = result.first_thumb_index_contact_time

    def contact_snapshot(time_s: float | None) -> dict[str, Any] | None:
        if time_s is None:
            return None
        rows = _thumb_index_contacts_at_time(result, time_s)
        if not rows:
            return {"time": time_s, "penetration_m": 0.0, "normal_force": 0.0}
        deepest = min(rows, key=lambda row: row.dist)
        return {
            "time": time_s,
            "penetration_m": max(0.0, -deepest.dist),
            "normal_force": max(row.normal_force for row in rows),
            "constraint_force_norm": max(row.constraint_force_norm for row in rows),
            "body_pair": [deepest.body1, deepest.body2],
            "geom_pair": [deepest.geom1, deepest.geom2],
        }

    penetration_by_time: dict[float, float] = {}
    force_by_time: dict[float, float] = {}
    for contact in result.contacts:
        if not _is_thumb_index_pair(contact.body1, contact.body2):
            continue
        penetration_by_time[contact.time] = max(
            penetration_by_time.get(contact.time, 0.0), max(0.0, -contact.dist)
        )
        force_by_time[contact.time] = max(force_by_time.get(contact.time, 0.0), contact.normal_force)
    series = []
    if first_time is not None:
        for sample in result.samples:
            if sample.time + 1e-10 < first_time:
                continue
            series.append(
                {
                    "time": sample.time,
                    "elapsed_after_first_contact_s": sample.time - first_time,
                    "penetration_m": penetration_by_time.get(sample.time, 0.0),
                    "normal_force": force_by_time.get(sample.time, 0.0),
                    "constraint_force_norm": sample.constraint_force_norm,
                    "ctrl": sample.ctrl,
                    "actuator_force": sample.actuator_force,
                    "target_error": sample.target_error,
                    "controller_still_pushing": sample.controller_still_pushing,
                }
            )
    maximum_time = None
    if penetration_by_time:
        maximum_time = max(penetration_by_time, key=penetration_by_time.get)
    first_sample = series[0] if series else None
    final_sample = series[-1] if series else None
    pushing_penetrations = [
        row["penetration_m"] for row in series if row["controller_still_pushing"]
    ]
    first_penetration = float(first_sample["penetration_m"]) if first_sample else 0.0
    return {
        "variant": result.assumptions.get("contact_response_variant", "continuous_command"),
        "first_contact": contact_snapshot(first_time),
        "first_persistent_contact": contact_snapshot(result.first_persistent_thumb_index_contact_time),
        "blockage": contact_snapshot(result.first_blockage_time),
        "maximum_penetration": contact_snapshot(maximum_time),
        "elapsed_first_contact_to_maximum_penetration_s": (
            None if first_time is None or maximum_time is None else maximum_time - first_time
        ),
        "penetration_increased_while_command_pushed": bool(
            pushing_penetrations and max(pushing_penetrations) > first_penetration + 1e-6
        ),
        "command_after_first_contact": {
            "first_ctrl": None if first_sample is None else first_sample["ctrl"],
            "final_ctrl": None if final_sample is None else final_sample["ctrl"],
            "ctrl_delta_inf": (
                None
                if first_sample is None or final_sample is None
                else float(
                    np.linalg.norm(
                        np.asarray(final_sample["ctrl"]) - np.asarray(first_sample["ctrl"]), ord=np.inf
                    )
                )
            ),
            "first_actuator_force": None if first_sample is None else first_sample["actuator_force"],
            "final_actuator_force": None if final_sample is None else final_sample["actuator_force"],
        },
        "post_contact_series": series,
    }


def object_contact_event_ordering(
    contacts: Sequence[ContactRecord],
    *,
    object_body_name: str = "stage2_object_body",
    object_geom_name: str = "stage2_object",
) -> dict[str, Any]:
    by_time: dict[float, list[ContactRecord]] = {}
    for contact in contacts:
        by_time.setdefault(contact.time, []).append(contact)

    def object_digit(contact: ContactRecord) -> str | None:
        if object_geom_name not in {contact.geom1, contact.geom2} and object_body_name not in {
            contact.body1,
            contact.body2,
        }:
            return None
        other_body = contact.body2 if contact.body1 == object_body_name else contact.body1
        return finger_group(other_body)

    first_thumb = min((row.time for row in contacts if object_digit(row) == "thumb"), default=None)
    first_index = min((row.time for row in contacts if object_digit(row) == "index"), default=None)
    first_self = min(
        (
            row.time
            for row in contacts
            if row.body1.startswith("rh56_R_")
            and row.body2.startswith("rh56_R_")
            and _is_thumb_index_pair(row.body1, row.body2)
        ),
        default=None,
    )
    first_forbidden = min((row.time for row in contacts if row.severity == "forbidden"), default=None)
    bilateral_times: list[float] = []
    for time_s, rows in sorted(by_time.items()):
        digits = {digit for row in rows if (digit := object_digit(row)) is not None}
        if {"thumb", "index"} <= digits:
            bilateral_times.append(time_s)
    first_bilateral = bilateral_times[0] if bilateral_times else None
    ordering = [
        {"time": time_s, "event": event}
        for time_s, event in sorted(
            (
                (time_s, event)
                for time_s, event in (
                    (first_thumb, "first_thumb_object_contact"),
                    (first_index, "first_index_object_contact"),
                    (first_bilateral, "first_bilateral_thumb_index_object_contact"),
                    (first_self, "first_thumb_index_self_contact"),
                    (first_forbidden, "first_forbidden_structural_contact"),
                )
                if time_s is not None
            ),
            key=lambda item: (float(item[0]), item[1]),
        )
    ]
    return {
        "first_thumb_object_contact_time": first_thumb,
        "first_index_object_contact_time": first_index,
        "first_bilateral_object_contact_time": first_bilateral,
        "first_thumb_index_self_contact_time": first_self,
        "first_forbidden_structural_contact_time": first_forbidden,
        "bilateral_contact_times": bilateral_times,
        "event_ordering": ordering,
    }


def summarize_object_mediated_trajectory(
    result: TrajectoryResult,
    model: mujoco.MjModel,
    *,
    object_body_name: str = "stage2_object_body",
    object_geom_name: str = "stage2_object",
    retention_threshold_s: float = 0.15,
) -> dict[str, Any]:
    object_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, object_body_name)
    if object_body_id < 0:
        raise KeyError(f"Missing object body {object_body_name!r}.")
    free_joint_ids = np.flatnonzero(model.jnt_bodyid == object_body_id)
    if not len(free_joint_ids):
        raise KeyError(f"Object body {object_body_name!r} has no joint.")
    qpos_address = int(model.jnt_qposadr[int(free_joint_ids[0])])
    contact_events = object_contact_event_ordering(
        result.contacts,
        object_body_name=object_body_name,
        object_geom_name=object_geom_name,
    )
    first_thumb = contact_events["first_thumb_object_contact_time"]
    first_index = contact_events["first_index_object_contact_time"]
    first_bilateral = contact_events["first_bilateral_object_contact_time"]
    first_self = contact_events["first_thumb_index_self_contact_time"]
    first_forbidden = contact_events["first_forbidden_structural_contact_time"]
    bilateral_times = contact_events["bilateral_contact_times"]

    def object_digit(contact: ContactRecord) -> str | None:
        if object_geom_name not in {contact.geom1, contact.geom2} and object_body_name not in {
            contact.body1,
            contact.body2,
        }:
            return None
        other_body = contact.body2 if contact.body1 == object_body_name else contact.body1
        return finger_group(other_body)

    dt = float(model.opt.timestep)
    longest_bilateral = 0.0
    current_duration = 0.0
    previous_time: float | None = None
    for time_s in bilateral_times:
        if previous_time is not None and time_s - previous_time <= dt * 1.5:
            current_duration += dt
        else:
            current_duration = dt
        longest_bilateral = max(longest_bilateral, current_duration)
        previous_time = time_s

    initial_qpos = np.asarray(result.states["initial_state"]["qpos"], dtype=np.float64)
    initial_position = initial_qpos[qpos_address : qpos_address + 3]
    positions = [np.asarray(row.qpos[qpos_address : qpos_address + 3], dtype=np.float64) for row in result.samples]
    final_position = positions[-1] if positions else initial_position
    max_displacement = max(
        (float(np.linalg.norm(position - initial_position)) for position in positions), default=0.0
    )

    thumb_object = [row for row in result.contacts if object_digit(row) == "thumb"]
    index_object = [row for row in result.contacts if object_digit(row) == "index"]
    event_times = [
        *[(row["time"], row["event"]) for row in contact_events["event_ordering"]],
        (result.first_blockage_time, "first_blockage"),
        (result.target_reached_time, "target_reached"),
    ]
    ordering = [
        {"time": time_s, "event": event}
        for time_s, event in sorted(
            ((time_s, event) for time_s, event in event_times if time_s is not None),
            key=lambda item: (float(item[0]), item[1]),
        )
    ]
    retained = longest_bilateral >= retention_threshold_s
    if first_forbidden is not None:
        interpreted_outcome = "forbidden_structural_contact"
    elif first_bilateral is not None and first_self is not None:
        interpreted_outcome = "bilateral_object_contact_then_thumb_index_self_contact"
    elif retained:
        interpreted_outcome = "bilateral_object_contact_retained"
    elif first_bilateral is not None:
        interpreted_outcome = "transient_bilateral_object_contact"
    elif first_thumb is not None or first_index is not None:
        interpreted_outcome = "unilateral_object_contact"
    else:
        interpreted_outcome = "no_object_contact"
    return {
        "runner_outcome": result.outcome,
        "interpreted_outcome": interpreted_outcome,
        "event_ordering": ordering,
        "first_thumb_object_contact_time": first_thumb,
        "first_index_object_contact_time": first_index,
        "first_bilateral_object_contact_time": first_bilateral,
        "first_thumb_index_self_contact_time": first_self,
        "first_forbidden_structural_contact_time": first_forbidden,
        "first_blockage_time": result.first_blockage_time,
        "target_reached_time": result.target_reached_time,
        "object_initial_position_m": initial_position.tolist(),
        "object_final_position_m": final_position.tolist(),
        "object_final_displacement_m": float(np.linalg.norm(final_position - initial_position)),
        "object_max_displacement_m": max_displacement,
        "object_retention_duration_s": longest_bilateral,
        "object_retained": retained,
        "object_contact_preceded_self_contact": bool(
            first_bilateral is not None and first_self is not None and first_bilateral < first_self
        ),
        "object_arrested_before_thumb_index_self_contact": bool(
            first_bilateral is not None and first_self is None
        ),
        "successful_object_mediated_closure": bool(
            retained and first_self is None and first_forbidden is None and not result.numerical_instability
        ),
        "max_thumb_object_penetration_m": max((max(0.0, -row.dist) for row in thumb_object), default=0.0),
        "max_index_object_penetration_m": max((max(0.0, -row.dist) for row in index_object), default=0.0),
        "max_rh56_self_penetration_m": result.max_rh56_self_penetration_m,
        "final_target_error": result.final_target_error,
        "max_thumb_object_normal_force": max((row.normal_force for row in thumb_object), default=0.0),
        "max_index_object_normal_force": max((row.normal_force for row in index_object), default=0.0),
        "max_actuator_force_abs": max(
            (max((abs(value) for value in sample.actuator_force), default=0.0) for sample in result.samples),
            default=0.0,
        ),
    }


def write_trajectory_artifacts(result: TrajectoryResult, out_dir: Path, *, reproduction_command: str | None = None) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    summary = result.summary_dict(include_samples=False, include_contacts=False)
    if reproduction_command:
        summary["deterministic_reproduction_command"] = reproduction_command
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    with (out_dir / "samples.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(result.samples[0]).keys()) if result.samples else ["step"])
        writer.writeheader()
        for row in result.samples:
            payload = asdict(row)
            for key in ("ctrl", "target_ctrl", "measured_ctrl_qpos", "qpos", "qvel", "actuator_force"):
                payload[key] = json.dumps(payload[key])
            writer.writerow(payload)
    with (out_dir / "contacts.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(result.contacts[0]).keys()) if result.contacts else ["time"])
        writer.writeheader()
        for row in result.contacts:
            payload = asdict(row)
            payload["pos"] = json.dumps(payload["pos"])
            payload["normal"] = json.dumps(payload["normal"])
            writer.writerow(payload)
    _write_svg_plots(result, out_dir / "plots.svg")


def _polyline(points: list[tuple[float, float]], *, width: int, height: int, pad: int) -> str:
    if not points:
        return ""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    if abs(xmax - xmin) < 1e-12:
        xmax = xmin + 1.0
    if abs(ymax - ymin) < 1e-12:
        ymax = ymin + 1.0
    coords = []
    for x, y in points:
        sx = pad + (x - xmin) / (xmax - xmin) * (width - 2 * pad)
        sy = height - pad - (y - ymin) / (ymax - ymin) * (height - 2 * pad)
        coords.append(f"{sx:.1f},{sy:.1f}")
    return " ".join(coords)


def _write_svg_plots(result: TrajectoryResult, path: Path) -> None:
    width = 900
    panel_h = 170
    pad = 34
    panels = [
        ("command max", [(row.time, max(row.ctrl) if row.ctrl else 0.0) for row in result.samples], "#555"),
        (
            "measured max",
            [
                (row.time, max(row.measured_ctrl_qpos) if row.measured_ctrl_qpos else 0.0)
                for row in result.samples
            ],
            "#885",
        ),
        ("target error", [(row.time, row.target_error) for row in result.samples], "#c43"),
        ("contact distance", [(row.time, row.min_contact_dist if row.min_contact_dist is not None else 0.0) for row in result.samples], "#276"),
        ("max normal force", [(row.time, row.max_normal_force) for row in result.samples], "#36c"),
        ("qvel norm", [(row.time, row.qvel_norm) for row in result.samples], "#737"),
    ]
    body: list[str] = []
    for idx, (label, points, color) in enumerate(panels):
        y0 = idx * panel_h
        body.append(f'<g transform="translate(0,{y0})">')
        body.append(f'<rect x="0" y="0" width="{width}" height="{panel_h}" fill="white" stroke="#ccc"/>')
        body.append(f'<text x="12" y="22" font-family="sans-serif" font-size="14">{label}</text>')
        body.append(f'<polyline fill="none" stroke="{color}" stroke-width="2" points="{_polyline(points, width=width, height=panel_h, pad=pad)}"/>')
        body.append("</g>")
    svg = f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{panel_h * len(panels)}">' + "".join(body) + "</svg>\n"
    path.write_text(svg, encoding="utf-8")
