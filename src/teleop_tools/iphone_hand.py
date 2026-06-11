from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from rh56_driver.hand_schema import (
    CANONICAL_HAND_ORDER,
    RH56_INTERNAL_ORDER,
    apply_delta,
    compute_delta,
    denormalize_canonical,
    raw_to_canonical,
)


IPHONE_CAMERA_URL = "http://admin:admin@192.168.71.157:8081/video"
HAND_TELEOP_SCHEMA_VERSION = "iphone_mediapipe_hand_teleop_v0.1"


def parse_camera_source(source: str) -> str | int:
    value = str(source).strip()
    return int(value) if value.isdigit() else value


@dataclass(frozen=True, slots=True)
class HandRetargetResult:
    target_norm: list[float]
    target_raw_count: list[int]
    features: dict[str, float]
    valid: bool
    reason: str = "ok"


def _point(landmarks: Sequence[dict[str, float]], idx: int) -> np.ndarray:
    item = landmarks[idx]
    return np.asarray(
        [float(item.get("x", 0.0)), float(item.get("y", 0.0)), float(item.get("z", 0.0))],
        dtype=np.float64,
    )


def _distance(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b))


def _angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    ba = a - b
    bc = c - b
    denom = float(np.linalg.norm(ba) * np.linalg.norm(bc))
    if denom <= 1e-9:
        return math.pi
    cos_value = float(np.clip(np.dot(ba, bc) / denom, -1.0, 1.0))
    return float(math.acos(cos_value))


def _curl_from_finger(landmarks: Sequence[dict[str, float]], indices: tuple[int, int, int, int]) -> float:
    mcp, pip, dip, tip = indices
    pip_angle = _angle(_point(landmarks, mcp), _point(landmarks, pip), _point(landmarks, dip))
    dip_angle = _angle(_point(landmarks, pip), _point(landmarks, dip), _point(landmarks, tip))
    curl = ((math.pi - pip_angle) + (math.pi - dip_angle)) / math.pi
    return float(np.clip(curl, 0.0, 1.0))


def _smoothstep(value: float) -> float:
    x = float(np.clip(value, 0.0, 1.0))
    return x * x * (3.0 - 2.0 * x)


def _calibrate_thumb_close(raw_close: float) -> float:
    return float(np.clip((raw_close - 0.10) / 0.75, 0.0, 1.0))


def _axis_projection(a: np.ndarray, b: np.ndarray) -> float:
    denom = float(np.linalg.norm(a) * np.linalg.norm(b))
    if denom <= 1e-9:
        return 0.0
    return float(np.clip(np.dot(a, b) / denom, -1.0, 1.0))


def _curl_from_axis_projection(projection: float) -> float:
    return float(np.clip((1.0 - projection) * 0.5, 0.0, 1.0))


def retarget_mediapipe_landmarks_to_rh56(
    landmarks: Sequence[dict[str, float]],
    *,
    max_close: float = 1.0,
    thumb_mode: str = "rh56_task",
) -> HandRetargetResult:
    if len(landmarks) != 21:
        return HandRetargetResult(
            target_norm=[0.0] * 6,
            target_raw_count=denormalize_canonical([0.0] * 6, raw_order=RH56_INTERNAL_ORDER),
            features={},
            valid=False,
            reason=f"expected 21 landmarks, got {len(landmarks)}",
        )

    wrist = _point(landmarks, 0)
    index_mcp = _point(landmarks, 5)
    middle_mcp = _point(landmarks, 9)
    pinky_mcp = _point(landmarks, 17)
    thumb_cmc = _point(landmarks, 1)
    thumb_mcp = _point(landmarks, 2)
    thumb_ip = _point(landmarks, 3)
    thumb_tip = _point(landmarks, 4)
    index_tip = _point(landmarks, 8)

    palm_scale = max(_distance(wrist, middle_mcp), 1e-6)
    palm_width = max(_distance(index_mcp, pinky_mcp), 1e-6)
    finger_indices = {
        "index": (5, 6, 7, 8),
        "middle": (9, 10, 11, 12),
        "ring": (13, 14, 15, 16),
        "pinky": (17, 18, 19, 20),
    }
    curls = {name: _curl_from_finger(landmarks, indices) for name, indices in finger_indices.items()}

    thumb_index_dist = _distance(thumb_tip, index_tip) / palm_scale
    thumb_index_mcp_dist = _distance(thumb_tip, index_mcp) / palm_scale
    thumb_cmc_angle = _angle(wrist, thumb_cmc, thumb_mcp)
    thumb_ip_angle = _angle(thumb_mcp, thumb_ip, thumb_tip)
    thumb_joint_curl = float(np.clip(((math.pi - thumb_cmc_angle) + (math.pi - thumb_ip_angle)) / math.pi, 0.0, 1.0))
    thumb_pinch = float(np.clip(1.0 - thumb_index_dist / 0.85, 0.0, 1.0))
    thumb_close_legacy = _smoothstep(max(thumb_joint_curl, thumb_pinch))

    thumb_base_axis = thumb_mcp - thumb_cmc
    thumb_base_vec = thumb_tip - thumb_cmc
    thumb_link_length = max(_distance(thumb_cmc, thumb_mcp) + _distance(thumb_mcp, thumb_ip) + _distance(thumb_ip, thumb_tip), 1e-6)
    thumb_extension_ratio = float(np.clip(_distance(thumb_cmc, thumb_tip) / thumb_link_length, 0.0, 1.0))
    thumb_extension_curl = 1.0 - thumb_extension_ratio
    thumb_proximal_projection = _axis_projection(thumb_mcp - thumb_cmc, thumb_ip - thumb_mcp)
    thumb_mcp_tip_projection = _axis_projection(thumb_mcp - thumb_cmc, thumb_tip - thumb_mcp)
    thumb_distal_projection = _axis_projection(thumb_ip - thumb_mcp, thumb_tip - thumb_ip)
    thumb_axis_fold_curl = max(
        _curl_from_axis_projection(thumb_proximal_projection),
        _curl_from_axis_projection(thumb_mcp_tip_projection),
        _curl_from_axis_projection(thumb_distal_projection),
        thumb_extension_curl,
    )

    thumb_side_axis = pinky_mcp - index_mcp
    palm_forward_axis = middle_mcp - wrist
    thumb_tip_side = _axis_projection(thumb_tip - thumb_cmc, thumb_side_axis)
    thumb_mcp_side = _axis_projection(thumb_mcp - thumb_cmc, thumb_side_axis)
    thumb_ip_side = _axis_projection(thumb_ip - thumb_cmc, thumb_side_axis)
    thumb_base_side_axis_cos = _axis_projection(thumb_base_axis, thumb_side_axis)
    thumb_base_forward_axis_cos = _axis_projection(thumb_base_axis, palm_forward_axis)
    thumb_rotation_angle = math.atan2(thumb_base_side_axis_cos, max(thumb_base_forward_axis_cos, 1e-6))
    thumb_lateral_linear = float(np.clip((thumb_tip_side + 0.45) / 0.90, 0.0, 1.0))
    thumb_lateral_legacy_linear = float(np.clip(1.0 - thumb_index_mcp_dist / 0.90, 0.0, 1.0))
    thumb_lateral_legacy = _smoothstep(thumb_lateral_legacy_linear)
    thumb_lateral_base = _smoothstep(thumb_lateral_linear)

    if thumb_mode == "legacy":
        thumb_close_raw = thumb_close_legacy
        thumb_lateral = thumb_lateral_legacy
    elif thumb_mode == "rh56_task":
        thumb_close_raw = max(thumb_axis_fold_curl, 0.35 * thumb_close_legacy)
        thumb_lateral = max(thumb_lateral_base, 0.30 * thumb_lateral_legacy)
    else:
        raise ValueError(f"Unsupported thumb_mode={thumb_mode!r}")

    target_norm = [
        curls["index"],
        curls["middle"],
        curls["ring"],
        curls["pinky"],
        _calibrate_thumb_close(thumb_close_raw),
        thumb_lateral,
    ]
    target_norm = [float(np.clip(value * max_close, 0.0, max_close)) for value in target_norm]
    target_raw_count = denormalize_canonical(target_norm, raw_order=RH56_INTERNAL_ORDER)
    features = {
        "index_curl": curls["index"],
        "middle_curl": curls["middle"],
        "ring_curl": curls["ring"],
        "pinky_curl": curls["pinky"],
        "thumb_index_dist_palm_norm": thumb_index_dist,
        "thumb_index_mcp_dist_palm_norm": thumb_index_mcp_dist,
        "thumb_cmc_angle_rad": thumb_cmc_angle,
        "thumb_ip_angle_rad": thumb_ip_angle,
        "thumb_joint_curl": thumb_joint_curl,
        "thumb_pinch": thumb_pinch,
        "thumb_tip_side_palm_width_norm": thumb_tip_side,
        "thumb_mcp_side_palm_width_norm": thumb_mcp_side,
        "thumb_ip_side_palm_width_norm": thumb_ip_side,
        "thumb_base_side_axis_cos": thumb_base_side_axis_cos,
        "thumb_base_forward_axis_cos": thumb_base_forward_axis_cos,
        "thumb_rotation_angle_rad": thumb_rotation_angle,
        "thumb_lateral_linear": thumb_lateral_linear,
        "thumb_lateral_legacy_linear": thumb_lateral_legacy_linear,
        "thumb_lateral_legacy": thumb_lateral_legacy,
        "thumb_lateral_base": thumb_lateral_base,
        "thumb_extension_ratio": thumb_extension_ratio,
        "thumb_extension_curl": thumb_extension_curl,
        "thumb_proximal_projection": thumb_proximal_projection,
        "thumb_mcp_tip_projection": thumb_mcp_tip_projection,
        "thumb_distal_projection": thumb_distal_projection,
        "thumb_axis_fold_curl": thumb_axis_fold_curl,
        "thumb_close_legacy": thumb_close_legacy,
        "thumb_close_raw": thumb_close_raw,
    }
    return HandRetargetResult(
        target_norm=target_norm,
        target_raw_count=target_raw_count,
        features=features,
        valid=True,
        reason="ok",
    )


def compute_delta_safely(current: Sequence[float], target: Sequence[float], *, limit: float = 0.05) -> list[float]:
    if len(current) != len(CANONICAL_HAND_ORDER) or len(target) != len(CANONICAL_HAND_ORDER):
        raise ValueError("current and target must be 6D canonical RH56 commands.")
    return compute_delta(current, target, limit=limit)


def apply_retarget_safety(
    previous_norm: Sequence[float] | None,
    target_norm: Sequence[float],
    *,
    delta_limit: float = 0.05,
    max_close: float = 1.0,
) -> list[float]:
    target = np.clip(np.asarray(target_norm, dtype=np.float64), 0.0, max_close).tolist()
    if previous_norm is None:
        return target
    delta = compute_delta_safely(previous_norm, target, limit=delta_limit)
    return apply_delta(previous_norm, delta)


def build_landmark_payload(
    *,
    timestamp: float,
    frame_id: str,
    handedness: str,
    score: float,
    landmarks: Sequence[dict[str, float]],
    image_shape: Sequence[int],
) -> dict[str, Any]:
    return {
        "schema_version": HAND_TELEOP_SCHEMA_VERSION,
        "timestamp": float(timestamp),
        "source": "iphone_ip_camera_mediapipe",
        "frame_id": frame_id,
        "handedness": handedness,
        "score": float(score),
        "landmark_format": "mediapipe_21",
        "landmarks": list(landmarks),
        "image_shape": list(image_shape),
    }


def build_rh56_target_payload(
    *,
    timestamp: float,
    frame_id: str,
    retarget: HandRetargetResult,
    safe_target_norm: Sequence[float],
    hand_detected: bool,
) -> dict[str, Any]:
    safe_target_raw_internal = denormalize_canonical(safe_target_norm, raw_order=RH56_INTERNAL_ORDER)
    return {
        "schema_version": HAND_TELEOP_SCHEMA_VERSION,
        "timestamp": float(timestamp),
        "source": "iphone_ip_camera_mediapipe",
        "frame_id": frame_id,
        "hand_detected": bool(hand_detected),
        "canonical_order": list(CANONICAL_HAND_ORDER),
        "raw_order": list(RH56_INTERNAL_ORDER),
        "target_norm": list(safe_target_norm),
        "target_raw_count": safe_target_raw_internal,
        "unsmoothed_raw_internal": list(retarget.target_raw_count),
        "raw_canonical": raw_to_canonical(safe_target_raw_internal, raw_order=RH56_INTERNAL_ORDER),
        "features": dict(retarget.features),
        "valid": retarget.valid,
        "reason": retarget.reason,
    }
