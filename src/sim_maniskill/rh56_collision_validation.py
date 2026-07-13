from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import mujoco
import numpy as np

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
    stage: str
    ctrl: list[float]
    target_ctrl: list[float]
    measured_ctrl_qpos: list[float]
    qpos: list[float]
    qvel: list[float]
    target_error: float
    ctrl_tracking_error: float
    qvel_norm: float
    ncon: int
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
    numerical_instability: bool
    first_contact_time: float | None
    first_contact_pair: list[str] | None
    first_blocking_pair: list[str] | None
    first_forbidden_pair: list[str] | None
    max_penetration_m: float
    max_rh56_self_penetration_m: float
    max_normal_force: float
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
            "numerical_instability": self.numerical_instability,
            "first_contact_time": self.first_contact_time,
            "first_contact_pair": self.first_contact_pair,
            "first_blocking_pair": self.first_blocking_pair,
            "first_forbidden_pair": self.first_forbidden_pair,
            "max_penetration_m": self.max_penetration_m,
            "max_rh56_self_penetration_m": self.max_rh56_self_penetration_m,
            "max_normal_force": self.max_normal_force,
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


def _step_command(
    current: np.ndarray,
    velocity: np.ndarray,
    target: np.ndarray,
    axes: Iterable[int],
    profile: CommandProfile,
    dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    axes_array = np.asarray(list(axes), dtype=np.int32)
    new = current.copy()
    new_velocity = velocity.copy()
    max_vel = np.asarray(profile.max_velocity_ctrl_per_s, dtype=np.float64)
    max_accel = np.asarray(profile.max_accel_ctrl_per_s2, dtype=np.float64)
    desired_velocity = np.zeros(6, dtype=np.float64)
    error = target - current
    desired_velocity[axes_array] = np.clip(error[axes_array] / max(dt, 1e-9), -max_vel[axes_array], max_vel[axes_array])
    dv = np.clip(desired_velocity - velocity, -max_accel * dt, max_accel * dt)
    new_velocity += dv
    step = new_velocity * dt
    for axis in axes_array:
        if abs(step[axis]) > abs(error[axis]):
            step[axis] = error[axis]
            new_velocity[axis] = 0.0
    new += step
    return new, new_velocity


def _state_payload(data: mujoco.MjData, hand_qpos_ids: np.ndarray, hand_ids: np.ndarray) -> dict[str, Any]:
    return {
        "time": float(data.time),
        "qpos": np.asarray(data.qpos, dtype=np.float64).round(8).tolist(),
        "qvel": np.asarray(data.qvel, dtype=np.float64).round(8).tolist(),
        "ctrl": np.asarray(data.ctrl, dtype=np.float64).round(8).tolist(),
        "measured_ctrl_qpos": np.asarray(data.qpos[hand_qpos_ids], dtype=np.float64).round(8).tolist(),
        "actuator_force": np.asarray(data.actuator_force[hand_ids], dtype=np.float64).round(8).tolist(),
    }


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

    initial_state = _state_payload(data, hand_qpos_ids, hand_ids)
    stages = motion_stages(strategy, initial, target)
    samples: list[SampleRecord] = []
    contacts: list[ContactRecord] = []
    events: list[dict[str, Any]] = []
    first_contact_time: float | None = None
    first_contact_pair: list[str] | None = None
    first_blocking_pair: list[str] | None = None
    first_forbidden_pair: list[str] | None = None
    max_penetration = 0.0
    max_rh56_self_penetration = 0.0
    max_force = 0.0
    max_penetration_state: dict[str, Any] | None = None
    first_contact_state: dict[str, Any] | None = None
    persistent_contact_by_pair: dict[tuple[str, str], float] = {}
    blocking_contact_by_pair: dict[tuple[str, str], float] = {}
    last_contact_seen: set[tuple[str, str]] = set()
    unknown_pairs: set[tuple[str, str]] = set()
    unreviewed_pairs: set[tuple[str, str]] = set()
    blocked = False
    numerical_instability = False
    current = initial.copy()
    velocity = np.zeros(6, dtype=np.float64)
    step_index = 0
    reached_hold_steps = 0
    progress_window_steps = max(1, int(math.ceil(profile.progress_window_seconds / dt)))
    measured_history: list[np.ndarray] = []

    for stage in stages:
        stage_target = np.asarray(stage.target_ctrl, dtype=np.float64)
        stage_start_time = float(data.time)
        while float(data.time) - stage_start_time <= profile.timeout_seconds:
            current, velocity = _step_command(current, velocity, stage_target, stage.active_axes, profile, dt)
            data.ctrl[hand_ids] = current
            if arm_qpos is not None:
                data.ctrl[arm_ids] = np.asarray(arm_qpos, dtype=np.float64)
            mujoco.mj_step(model, data)
            step_index += 1
            measured = np.asarray(data.qpos[hand_qpos_ids], dtype=np.float64).copy()
            measured_history.append(measured)
            if len(measured_history) > progress_window_steps + 1:
                measured_history.pop(0)

            if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)) or warning_count(data) > 0:
                numerical_instability = True
                events.append({"time": float(data.time), "event": "numerical_instability_or_warning", "warning_count": warning_count(data)})
                break

            contact_pairs_this_step: set[tuple[str, str]] = set()
            contact_rows: list[tuple[tuple[str, str], ContactClassification, ContactRecord]] = []
            constraint_norm = _constraint_force_norm(data)
            min_dist: float | None = None
            step_max_force = 0.0
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
                if min_dist is None or dist < min_dist:
                    min_dist = dist
                if dist < 0.0 and abs(dist) > max_penetration:
                    max_penetration = abs(dist)
                    max_penetration_state = _state_payload(data, hand_qpos_ids, hand_ids)
                if body1.startswith("rh56_R_") and body2.startswith("rh56_R_") and dist < 0.0:
                    max_rh56_self_penetration = max(max_rh56_self_penetration, abs(dist))
                pair = tuple(sorted((body1, body2)))
                contact_pairs_this_step.add(pair)
                if classification.category == "unknown_or_unreviewed_pair":
                    unknown_pairs.add(pair)
                if body1.startswith("rh56_R_") and body2.startswith("rh56_R_") and not classification.reviewed:
                    unreviewed_pairs.add(pair)
                record = ContactRecord(
                    time=float(data.time),
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
                if first_contact_time is None:
                    first_contact_time = float(data.time)
                    first_contact_pair = [body1, body2]
                    first_contact_state = _state_payload(data, hand_qpos_ids, hand_ids)
                    events.append({"time": float(data.time), "event": "contact_onset", "body_pair": first_contact_pair})
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
            for pair, classification, record in contact_rows:
                persistent_contact_by_pair[pair] = persistent_contact_by_pair.get(pair, 0.0) + dt
                contacts.append(record)
                if (
                    classification.category not in NON_BLOCKING_CONTACT_CATEGORIES
                    and record.body1.startswith("rh56_R_")
                    and record.body2.startswith("rh56_R_")
                ):
                    blocking_contact_by_pair[pair] = blocking_contact_by_pair.get(pair, 0.0) + dt
                if (
                    first_blocking_pair is None
                    and blocking_contact_by_pair.get(pair, 0.0) >= profile.persistent_contact_seconds
                ):
                    first_blocking_pair = [record.body1, record.body2]
                    events.append(
                        {
                            "time": float(data.time),
                            "event": "sustained_contact",
                            "body_pair": first_blocking_pair,
                            "category": classification.category,
                            "duration_s": persistent_contact_by_pair[pair],
                        }
                    )
            separated = last_contact_seen - contact_pairs_this_step
            for pair in sorted(separated):
                events.append({"time": float(data.time), "event": "separation", "body_pair": list(pair)})
            last_contact_seen = contact_pairs_this_step

            target_error = float(np.linalg.norm(target - measured, ord=np.inf))
            ctrl_tracking_error = float(np.linalg.norm(current - measured, ord=np.inf))
            qvel_norm = float(np.linalg.norm(np.asarray(data.qvel, dtype=np.float64)))
            if step_index % max(1, sample_stride) == 0:
                samples.append(
                    SampleRecord(
                        step=step_index,
                        time=float(data.time),
                        stage=stage.name,
                        ctrl=current.round(8).tolist(),
                        target_ctrl=stage_target.round(8).tolist(),
                        measured_ctrl_qpos=measured.round(8).tolist(),
                        qpos=np.asarray(data.qpos, dtype=np.float64).round(8).tolist(),
                        qvel=np.asarray(data.qvel, dtype=np.float64).round(8).tolist(),
                        target_error=target_error,
                        ctrl_tracking_error=ctrl_tracking_error,
                        qvel_norm=qvel_norm,
                        ncon=int(data.ncon),
                        min_contact_dist=min_dist,
                        max_normal_force=step_max_force,
                        constraint_force_norm=constraint_norm,
                        actuator_force=np.asarray(data.actuator_force[hand_ids], dtype=np.float64).round(8).tolist(),
                        warning_count=warning_count(data),
                    )
                )

            if target_error <= profile.error_tolerance_ctrl and np.linalg.norm(stage_target - current, ord=np.inf) <= profile.error_tolerance_ctrl:
                reached_hold_steps += 1
                if reached_hold_steps >= max(1, int(math.ceil(profile.hold_seconds / dt))):
                    events.append({"time": float(data.time), "event": "target_reached", "target_error": target_error})
                    break
            else:
                reached_hold_steps = 0

            if len(measured_history) >= progress_window_steps:
                progress = float(np.linalg.norm(measured_history[-1] - measured_history[0], ord=np.inf))
                persistent_contact = any(duration >= profile.persistent_contact_seconds for duration in blocking_contact_by_pair.values())
                controller_still_pushing = ctrl_tracking_error > profile.error_tolerance_ctrl or np.linalg.norm(stage_target - current, ord=np.inf) > profile.error_tolerance_ctrl
                if (
                    persistent_contact
                    and controller_still_pushing
                    and target_error > profile.error_tolerance_ctrl
                    and progress <= profile.progress_epsilon_ctrl
                    and (constraint_norm >= profile.force_blockage_threshold or step_max_force >= profile.force_blockage_threshold or first_blocking_pair is not None)
                ):
                    blocked = True
                    if first_blocking_pair is None and contact_rows:
                        record = contact_rows[0][2]
                        first_blocking_pair = [record.body1, record.body2]
                    events.append(
                        {
                            "time": float(data.time),
                            "event": "stable_blockage",
                            "target_error": target_error,
                            "progress_window_ctrl": progress,
                            "first_blocking_pair": first_blocking_pair,
                        }
                    )
                    break
        if numerical_instability or blocked:
            break

    final_error = float(np.linalg.norm(target - np.asarray(data.qpos[hand_qpos_ids], dtype=np.float64), ord=np.inf))
    reached = bool(final_error <= profile.error_tolerance_ctrl and not blocked and not numerical_instability)
    timeout = bool(not reached and not blocked and not numerical_instability)
    if first_forbidden_pair is not None:
        outcome = "forbidden_structural_collision"
        blockage_kind = "forbidden_structural_collision"
    elif reached:
        has_long_contact = any(duration >= profile.persistent_contact_seconds for duration in persistent_contact_by_pair.values())
        has_short_contact = any(0.0 < duration <= profile.transient_contact_seconds for duration in persistent_contact_by_pair.values())
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
        numerical_instability=numerical_instability,
        first_contact_time=first_contact_time,
        first_contact_pair=first_contact_pair,
        first_blocking_pair=first_blocking_pair if blocked else None,
        first_forbidden_pair=first_forbidden_pair,
        max_penetration_m=max_penetration,
        max_rh56_self_penetration_m=max_rh56_self_penetration,
        max_normal_force=max_force,
        final_target_error=final_error,
        unknown_self_pairs=[list(pair) for pair in sorted(unknown_pairs)],
        unreviewed_self_pairs=[list(pair) for pair in sorted(unreviewed_pairs)],
        events=events,
        samples=samples,
        contacts=contacts,
        states={
            "initial_state": initial_state,
            "first_contact_state": first_contact_state,
            "maximum_penetration_state": max_penetration_state,
            "final_state": _state_payload(data, hand_qpos_ids, hand_ids),
        },
        assumptions={
            "hardware_speed_units": "Repository defines RH56 raw speed registers and defaults but no conversion from raw 0..1000 speed units to rad/s.",
            "nominal_profile_basis": "Derived from teleop delta_limit=0.05 normalized units at command_hz=15, mapped through MuJoCo actuator ranges.",
            "dynamic_validation_rule": "Intermediate hand motion uses position actuator controls plus repeated mujoco.mj_step; qpos writes are limited to reset/initial state setup.",
            "visual_mesh_intersection_diagnostic": "Not evaluated by this core trajectory runner; compare collision modes and optional external mesh diagnostics in the CLI report.",
        },
    )


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
