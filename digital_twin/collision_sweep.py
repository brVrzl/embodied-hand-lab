"""Deterministic offline MuJoCo collision characterization for the P-world scene.

This module intentionally contains no hardware interfaces.  It reports simulated
contacts; it is not a safety-certification implementation.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import mujoco
import numpy as np


ARM_JOINT_NAMES = [f"jaka_joint_{index}" for index in range(1, 7)]
ARM_ACTUATOR_NAMES = [f"jaka_joint_{index}_act" for index in range(1, 7)]
HAND_ACTUATOR_NAMES = [
    "rh56_R_thumb_MCP_joint1_act",
    "rh56_R_thumb_MCP_joint2_act",
    "rh56_R_index_MCP_joint_act",
    "rh56_R_middle_MCP_joint_act",
    "rh56_R_ring_MCP_joint_act",
    "rh56_R_pinky_MCP_joint_act",
]
PALM_BODY = "rh56_R_hand_base_link"
BASE_BODY = "jaka_Link_0"


@dataclass(frozen=True)
class PoseSample:
    name: str
    arm_qpos: tuple[float, ...]
    hand_ctrl: tuple[float, ...]
    source: str
    role: str = "diagnostic_coverage"


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    value = mujoco.mj_name2id(model, kind, name)
    if value < 0:
        raise KeyError(f"Missing MuJoCo {kind.name}: {name}")
    return int(value)


def name_or_id(model: mujoco.MjModel, kind: mujoco.mjtObj, index: int) -> str:
    return mujoco.mj_id2name(model, kind, int(index)) or f"unnamed_{kind.name}_{index}"


def canonical_pair(a: str, b: str) -> tuple[str, str]:
    return tuple(sorted((str(a), str(b))))


def baseline_pair_set(config: Mapping[str, Any]) -> set[tuple[str, str]]:
    return {
        canonical_pair(*entry["geom_pair"])
        for entry in config.get("baseline_pairs", [])
        if len(entry.get("geom_pair", [])) == 2
    }


def is_baseline_pair(geom_a: str, geom_b: str, config: Mapping[str, Any]) -> bool:
    return canonical_pair(geom_a, geom_b) in baseline_pair_set(config)


def environment_depth_status(depth_m: float, config: Mapping[str, Any]) -> str:
    thresholds = config["thresholds"]["environment_depth_m"]
    if depth_m <= float(thresholds["review_max"]):
        return "REVIEW"
    if depth_m <= float(thresholds["warn_max"]):
        return "WARN"
    return "FAIL"


def persistence_status(duration_s: float, config: Mapping[str, Any]) -> str | None:
    thresholds = config["thresholds"]["persistence_s"]
    if duration_s > float(thresholds["fail_above"]):
        return "FAIL"
    if duration_s > float(thresholds["warn_above"]):
        return "WARN"
    return None


def _body_kind(name: str) -> str:
    if name.startswith("jaka_"):
        return "arm"
    if name.startswith("rh56_"):
        return "hand"
    return "other"


def _is_direct_parent_child(model: mujoco.MjModel, body_a: int, body_b: int) -> bool:
    return int(model.body_parentid[body_a]) == body_b or int(model.body_parentid[body_b]) == body_a


def classify_contact_pair(
    model: mujoco.MjModel,
    geom_a_id: int,
    geom_b_id: int,
    depth_m: float,
    config: Mapping[str, Any],
) -> dict[str, Any]:
    geom_a = name_or_id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_a_id)
    geom_b = name_or_id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_b_id)
    body_a_id = int(model.geom_bodyid[geom_a_id])
    body_b_id = int(model.geom_bodyid[geom_b_id])
    body_a = name_or_id(model, mujoco.mjtObj.mjOBJ_BODY, body_a_id)
    body_b = name_or_id(model, mujoco.mjtObj.mjOBJ_BODY, body_b_id)
    if is_baseline_pair(geom_a, geom_b, config):
        return {"category": "canonical_baseline_self_contact", "status": "BASELINE", "baseline": True}

    forbidden = tuple(config["noncolliding_layers"].get("forbidden_geom_prefixes", []))
    if geom_a.startswith(forbidden) or geom_b.startswith(forbidden):
        category = "camera_placeholder_contact" if geom_a.startswith("camera_") or geom_b.startswith("camera_") else "visual_debug_geometry_contact"
        return {"category": category, "status": "FAIL", "baseline": False}

    groups = {_body_kind(body_a), _body_kind(body_b)}
    environment = config["environment_geoms"]
    table = set(environment.get("table", []))
    aluminium = set(environment.get("aluminium", []))
    floor = set(environment.get("floor", []))
    geoms = {geom_a, geom_b}
    robot_group = "hand" if "hand" in groups else "arm"
    if geoms & floor and groups & {"arm", "hand"}:
        return {"category": "robot_floor_contact", "status": "FAIL", "baseline": False}
    if geoms & table and groups & {"arm", "hand"}:
        return {"category": f"{robot_group}_table_contact", "status": environment_depth_status(depth_m, config), "baseline": False}
    if geoms & aluminium and groups & {"arm", "hand"}:
        return {"category": f"{robot_group}_aluminium_contact", "status": environment_depth_status(depth_m, config), "baseline": False}
    if groups == {"arm"}:
        return {"category": "new_arm_self_contact", "status": "FAIL", "baseline": False}
    if groups == {"hand"}:
        adjacent = _is_direct_parent_child(model, body_a_id, body_b_id)
        if adjacent:
            return {"category": "hand_self_contact", "status": "ALLOWED", "baseline": False, "adjacent": True}
        return {"category": "hand_self_contact", "status": environment_depth_status(depth_m, config), "baseline": False, "adjacent": False}
    if groups == {"arm", "hand"}:
        # The hand base is rigidly mounted below Link6; direct mount adjacency is expected.
        adjacent = _is_direct_parent_child(model, body_a_id, body_b_id)
        return {
            "category": "hand_self_contact" if adjacent else "new_arm_self_contact",
            "status": "ALLOWED" if adjacent else "FAIL",
            "baseline": False,
            "adjacent": adjacent,
        }
    return {"category": "unclassified_contact", "status": "REVIEW", "baseline": False}


def halton(index: int, base: int) -> float:
    if index <= 0 or base <= 1:
        raise ValueError("Halton index must be positive and base must exceed one.")
    result = 0.0
    factor = 1.0 / base
    while index:
        result += factor * (index % base)
        index //= base
        factor /= base
    return result


def deterministic_arm_samples(
    limits: Sequence[Sequence[float]],
    *,
    halton_samples: int = 48,
    halton_skip: int = 11,
) -> list[tuple[str, np.ndarray, str]]:
    bounds = np.asarray(limits, dtype=np.float64)
    if bounds.shape != (6, 2) or np.any(bounds[:, 0] >= bounds[:, 1]):
        raise ValueError("Expected six valid [lower, upper] joint bounds.")
    midpoint = bounds.mean(axis=1)
    rows: list[tuple[str, np.ndarray, str]] = [("zero", np.zeros(6), "reference_zero")]
    fractions = (0.0, 0.25, 0.5, 0.75, 1.0)
    for joint in range(6):
        for fraction in fractions:
            q = midpoint.copy()
            q[joint] = bounds[joint, 0] + fraction * (bounds[joint, 1] - bounds[joint, 0])
            rows.append((f"oat_j{joint + 1}_{int(fraction * 100):03d}", q, "one_joint_at_a_time"))
    for joint in range(5):
        for left in (0.1, 0.5, 0.9):
            for right in (0.1, 0.5, 0.9):
                q = midpoint.copy()
                q[joint] = bounds[joint, 0] + left * (bounds[joint, 1] - bounds[joint, 0])
                q[joint + 1] = bounds[joint + 1, 0] + right * (bounds[joint + 1, 1] - bounds[joint + 1, 0])
                rows.append((f"pair_j{joint + 1}_j{joint + 2}_{int(left*10)}{int(right*10)}", q, "adjacent_joint_pair"))
    primes = (2, 3, 5, 7, 11, 13)
    for sample in range(halton_samples):
        fractions_q = np.asarray([halton(sample + halton_skip + 1, prime) for prime in primes])
        q = bounds[:, 0] + fractions_q * (bounds[:, 1] - bounds[:, 0])
        rows.append((f"halton_{sample:03d}", q, "halton_low_discrepancy"))

    unique: list[tuple[str, np.ndarray, str]] = []
    seen: set[tuple[float, ...]] = set()
    for name, q, source in rows:
        key = tuple(np.round(q, 10))
        if key not in seen:
            seen.add(key)
            unique.append((name, q, source))
    return unique


def actuator_ids(model: mujoco.MjModel, names: Iterable[str]) -> np.ndarray:
    return np.asarray([object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in names], dtype=np.int32)


def actuator_qpos_addresses(model: mujoco.MjModel, ids: Sequence[int]) -> np.ndarray:
    addresses: list[int] = []
    for actuator_id in ids:
        joint_id = int(model.actuator_trnid[int(actuator_id), 0])
        if joint_id < 0:
            raise ValueError(f"Actuator {actuator_id} is not joint-position controlled.")
        addresses.append(int(model.jnt_qposadr[joint_id]))
    return np.asarray(addresses, dtype=np.int32)


def hand_qpos_from_ctrl(model: mujoco.MjModel, hand_ctrl: Sequence[float]) -> np.ndarray:
    """Return all 12 RH56 qpos using the model's actuator and equality mappings."""
    ctrl = np.asarray(hand_ctrl, dtype=np.float64)
    if ctrl.shape != (6,):
        raise ValueError("RH56 actuator target must have six values.")
    hand_ids = actuator_ids(model, HAND_ACTUATOR_NAMES)
    addresses = actuator_qpos_addresses(model, hand_ids)
    qpos = np.zeros(12, dtype=np.float64)
    qpos[addresses - 6] = ctrl
    # Existing MJCF equalities: thumb PIP=.6*MCP2, DIP=.8*MCP2; finger DIP=MCP.
    qpos[2] = 0.6 * qpos[1]
    qpos[3] = 0.8 * qpos[1]
    qpos[5] = qpos[4]
    qpos[7] = qpos[6]
    qpos[9] = qpos[8]
    qpos[11] = qpos[10]
    for local_index, joint_id in enumerate(range(6, model.njnt)):
        if model.jnt_limited[joint_id]:
            low, high = model.jnt_range[joint_id]
            qpos[local_index] = np.clip(qpos[local_index], low, high)
    return qpos


def set_static_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm_qpos: Sequence[float],
    hand_ctrl: Sequence[float],
) -> None:
    arm = np.asarray(arm_qpos, dtype=np.float64)
    hand = np.asarray(hand_ctrl, dtype=np.float64)
    if arm.shape != (6,) or hand.shape != (6,):
        raise ValueError("arm_qpos and hand_ctrl must each have six values.")
    mujoco.mj_resetData(model, data)
    data.qpos[:6] = arm
    data.qpos[6:18] = hand_qpos_from_ctrl(model, hand)
    data.ctrl[actuator_ids(model, ARM_ACTUATOR_NAMES)] = arm
    data.ctrl[actuator_ids(model, HAND_ACTUATOR_NAMES)] = hand
    mujoco.mj_forward(model, data)


def palm_normal_in_world(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    body = object_id(model, mujoco.mjtObj.mjOBJ_BODY, PALM_BODY)
    return data.xmat[body].reshape(3, 3) @ np.asarray([0.0, 1.0, 0.0])


def angle_degrees(a: Sequence[float], b: Sequence[float]) -> float:
    left = np.asarray(a, dtype=np.float64); right = np.asarray(b, dtype=np.float64)
    left /= np.linalg.norm(left); right /= np.linalg.norm(right)
    return float(np.degrees(np.arccos(np.clip(np.dot(left, right), -1.0, 1.0))))


def verify_operational_scene(
    model: mujoco.MjModel,
    operational: Mapping[str, Any],
    classification: Mapping[str, Any],
) -> dict[str, Any]:
    required = classification["operational_placement"]
    if not np.allclose(operational["translation_m"], required["required_translation_m"], atol=1e-12):
        raise RuntimeError("Operational robot-root translation no longer equals [0, 0, 0].")
    if not math.isclose(float(operational["yaw_deg"]), float(required["required_yaw_deg"]), abs_tol=1e-9):
        raise RuntimeError("Operational robot-root yaw is not the required 180 degrees.")
    if not np.allclose(operational["quaternion_xyzw"], required["required_quaternion_xyzw"], atol=1e-12):
        raise RuntimeError("Operational robot-root quaternion changed.")
    if not np.allclose(model.qpos0, 0.0, atol=1e-12):
        raise RuntimeError("Reference qpos is no longer the all-zero JAKA/RH56 state.")
    base = object_id(model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY)
    expected_wxyz = np.asarray([0.0, 0.0, 0.0, 1.0])
    if not np.allclose(model.body_pos[base], required["required_translation_m"], atol=1e-12):
        raise RuntimeError("Generated scene root translation disagrees with operational placement.")
    if not np.allclose(model.body_quat[base], expected_wxyz, atol=1e-10):
        raise RuntimeError("Generated scene root quaternion is not the 180-degree yaw (MuJoCo wxyz).")
    data = mujoco.MjData(model); mujoco.mj_forward(model, data)
    palm_error = angle_degrees(palm_normal_in_world(model, data), [-1.0, 0.0, 0.0])
    if palm_error > float(required["palm_alignment_tolerance_deg"]):
        raise RuntimeError(f"Palm alignment regression: {palm_error:.6f} degrees.")
    return {"translation_m": model.body_pos[base].tolist(), "yaw_deg": 180.0, "palm_error_deg": palm_error, "qpos0_all_zero": True}


def enforce_noncolliding_layers(model: mujoco.MjModel, config: Mapping[str, Any]) -> dict[str, Any]:
    forbidden = tuple(config["noncolliding_layers"].get("forbidden_geom_prefixes", []))
    forbidden_geoms: list[str] = []
    for geom_id in range(model.ngeom):
        name = name_or_id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        if name.startswith(forbidden):
            forbidden_geoms.append(name)
            if model.geom_contype[geom_id] or model.geom_conaffinity[geom_id]:
                raise RuntimeError(f"Visual/debug geom unexpectedly collides: {name}")
    camera_sites = config["noncolliding_layers"].get("required_camera_sites", [])
    for site in camera_sites:
        object_id(model, mujoco.mjtObj.mjOBJ_SITE, site)
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, site) >= 0:
            raise RuntimeError(f"Camera placeholder is a collision geom: {site}")
    return {"forbidden_geoms_present": forbidden_geoms, "camera_sites_verified": list(camera_sites)}


def contact_rows(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    config: Mapping[str, Any],
    *,
    context: Mapping[str, Any],
    consecutive_duration: Mapping[tuple[str, str], float] | None = None,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    duration_map = consecutive_duration or {}
    for index in range(data.ncon):
        contact = data.contact[index]
        geom_a_id, geom_b_id = int(contact.geom1), int(contact.geom2)
        geom_a = name_or_id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_a_id)
        geom_b = name_or_id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_b_id)
        body_a = name_or_id(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom_a_id]))
        body_b = name_or_id(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom_b_id]))
        depth = max(0.0, -float(contact.dist))
        classification = classify_contact_pair(model, geom_a_id, geom_b_id, depth, config)
        force = np.zeros(6, dtype=np.float64)
        mujoco.mj_contactForce(model, data, index, force)
        pair = canonical_pair(geom_a, geom_b)
        duration = float(duration_map.get(pair, 0.0))
        persistent = persistence_status(duration, config)
        status = classification["status"]
        if not classification["baseline"] and persistent == "FAIL":
            status = "FAIL"
        elif not classification["baseline"] and persistent == "WARN" and status in {"ALLOWED", "REVIEW"}:
            status = "WARN"
        row = {
            **dict(context),
            "simulation_time_s": float(data.time),
            "contact_index": index,
            "body_a": body_a,
            "body_b": body_b,
            "geom_a": geom_a,
            "geom_b": geom_b,
            "contact_position_m": np.asarray(contact.pos).tolist(),
            "contact_normal_world": np.asarray(contact.frame[:3]).tolist(),
            "penetration_depth_m": depth,
            "normal_force_n": float(force[0]),
            "tangent_force_1_n": float(force[1]),
            "tangent_force_2_n": float(force[2]),
            "tangent_resultant_n": float(np.linalg.norm(force[1:3])),
            "contact_duration_s": duration,
            "category": classification["category"],
            "status": status,
            "baseline": bool(classification["baseline"]),
            "qpos": np.asarray(data.qpos).tolist(),
            "qvel": np.asarray(data.qvel).tolist(),
            "ctrl": np.asarray(data.ctrl).tolist(),
        }
        rows.append(row)
    return rows


def minimum_robot_environment_distance(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    config: Mapping[str, Any],
    max_distance_m: float,
) -> tuple[float | None, list[str] | None]:
    environment_names = sum((list(value) for value in config["environment_geoms"].values()), [])
    environment_ids = [object_id(model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in environment_names]
    robot_ids = [
        geom_id for geom_id in range(model.ngeom)
        if _body_kind(name_or_id(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom_id]))) in {"arm", "hand"}
        and (model.geom_contype[geom_id] or model.geom_conaffinity[geom_id])
    ]
    best = math.inf; best_pair: list[str] | None = None
    endpoints = np.zeros(6, dtype=np.float64)
    for robot_id in robot_ids:
        for environment_id in environment_ids:
            distance = float(mujoco.mj_geomDistance(model, data, robot_id, environment_id, max_distance_m, endpoints))
            if distance < best:
                best = distance
                best_pair = [
                    name_or_id(model, mujoco.mjtObj.mjOBJ_GEOM, robot_id),
                    name_or_id(model, mujoco.mjtObj.mjOBJ_GEOM, environment_id),
                ]
    return (None if not math.isfinite(best) else best), best_pair


def smoothstep_interpolation(start: Sequence[float], target: Sequence[float], steps: int) -> np.ndarray:
    if steps < 2:
        raise ValueError("Interpolation requires at least two steps.")
    left = np.asarray(start, dtype=np.float64); right = np.asarray(target, dtype=np.float64)
    if left.shape != right.shape:
        raise ValueError("Interpolation endpoints must have equal shape.")
    u = np.linspace(0.0, 1.0, steps)
    blend = 3.0 * u**2 - 2.0 * u**3
    return left[None, :] + blend[:, None] * (right - left)[None, :]


def bounded_duration(
    start: Sequence[float],
    target: Sequence[float],
    *,
    base_duration_s: float,
    max_velocity: Sequence[float],
    max_acceleration: Sequence[float],
) -> float:
    delta = np.abs(np.asarray(target, dtype=np.float64) - np.asarray(start, dtype=np.float64))
    velocity = np.asarray(max_velocity, dtype=np.float64); acceleration = np.asarray(max_acceleration, dtype=np.float64)
    if np.any(velocity <= 0) or np.any(acceleration <= 0):
        raise ValueError("Velocity and acceleration bounds must be positive.")
    # Cubic smoothstep has maxima 1.5*delta/T and 6*delta/T^2.
    velocity_duration = float(np.max(1.5 * delta / velocity))
    acceleration_duration = float(np.max(np.sqrt(6.0 * delta / acceleration)))
    return max(float(base_duration_s), velocity_duration, acceleration_duration)


def update_consecutive_contact_durations(
    active_pairs: Iterable[tuple[str, str]],
    previous: Mapping[tuple[str, str], float],
    dt: float,
) -> dict[tuple[str, str], float]:
    active = set(active_pairs)
    return {pair: float(previous.get(pair, 0.0)) + dt for pair in active}


def early_termination_reason(
    qpos: Sequence[float],
    qvel: Sequence[float],
    contacts: Sequence[Mapping[str, Any]],
    catastrophic_penetration_m: float,
) -> str | None:
    if not np.isfinite(np.asarray(qpos, dtype=np.float64)).all() or not np.isfinite(np.asarray(qvel, dtype=np.float64)).all():
        return "nonfinite_state"
    if any(
        not bool(row.get("baseline")) and float(row.get("penetration_depth_m", 0.0)) > catastrophic_penetration_m
        for row in contacts
    ):
        return "catastrophic_penetration"
    return None


def write_csv(path: str | Path, rows: Sequence[Mapping[str, Any]], fieldnames: Sequence[str]) -> None:
    destination = Path(path); destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            serialized = {key: _csv_value(row.get(key)) for key in fieldnames}
            writer.writerow(serialized)


def _csv_value(value: Any) -> Any:
    if isinstance(value, (list, tuple, dict)):
        import json
        return json.dumps(value, separators=(",", ":"), sort_keys=isinstance(value, dict))
    if isinstance(value, np.generic):
        return value.item()
    return value
