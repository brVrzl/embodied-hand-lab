from __future__ import annotations

import json
import math
from itertools import permutations, product
from pathlib import Path
from typing import Any, Mapping

import numpy as np


CALIBRATION_SCHEMA_VERSION = "teledex_jaka_frame_calibration_v0.1"


def fit_phone_to_robot_rotation(
    raw_displacements_by_robot_axis: Mapping[str, Any],
    *,
    min_displacement_m: float = 0.04,
    max_pairwise_axis_dot: float = 0.40,
    mapping_mode: str = "continuous_so3",
) -> tuple[np.ndarray, dict[str, Any]]:
    """Fit R such that R @ phone_delta points along the requested JAKA axis."""
    if mapping_mode not in {"continuous_so3", "signed_permutation"}:
        raise ValueError(
            "mapping_mode must be 'continuous_so3' or 'signed_permutation'."
        )
    vectors: list[np.ndarray] = []
    lengths: dict[str, float] = {}
    for axis in ("x", "y", "z"):
        vector = np.asarray(raw_displacements_by_robot_axis[axis], dtype=np.float64)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError(f"Calibration displacement for +{axis.upper()} must contain 3 finite values.")
        length = float(np.linalg.norm(vector))
        if length < float(min_displacement_m):
            raise ValueError(
                f"Calibration displacement for +{axis.upper()} is only {length:.3f} m; "
                f"move at least {min_displacement_m:.3f} m."
            )
        vectors.append(vector / length)
        lengths[axis] = length
    phone_basis = np.column_stack(vectors)
    pairwise_dots = {
        "xy": float(np.dot(vectors[0], vectors[1])),
        "xz": float(np.dot(vectors[0], vectors[2])),
        "yz": float(np.dot(vectors[1], vectors[2])),
    }
    if max(abs(value) for value in pairwise_dots.values()) > float(max_pairwise_axis_dot):
        raise ValueError(
            "Captured +X/+Y/+Z movements are not sufficiently perpendicular; "
            f"pairwise dots={pairwise_dots}. Repeat the capture along the JAKA base axes."
        )
    determinant = float(np.linalg.det(phone_basis))
    if determinant <= 0.0:
        raise ValueError(
            "Captured directions form a reflected/left-handed basis. Check axis labels and repeat calibration."
        )

    signed_quality: dict[str, Any] = {}
    if mapping_mode == "signed_permutation":
        candidates: list[tuple[float, np.ndarray]] = []
        for permutation in permutations(range(3)):
            for signs in product((-1.0, 1.0), repeat=3):
                candidate = np.zeros((3, 3), dtype=np.float64)
                for row, column in enumerate(permutation):
                    candidate[row, column] = signs[row]
                if float(np.linalg.det(candidate)) > 0.0:
                    score = float(np.trace(candidate @ phone_basis))
                    candidates.append((score, candidate))
        candidates.sort(key=lambda item: item[0], reverse=True)
        best_score, rotation = candidates[0]
        mapped_candidate = rotation @ phone_basis
        axis_cosines = np.diag(mapped_candidate)
        if float(np.min(axis_cosines)) < 0.80:
            raise ValueError(
                "Captured movements do not identify a confident signed axis mapping; "
                f"axis cosines={axis_cosines.astype(float).tolist()}."
            )
        signed_quality = {
            "signed_axis_alignment_cosine": {
                axis: float(axis_cosines[index])
                for index, axis in enumerate(("x", "y", "z"))
            },
            "signed_assignment_score": best_score,
            "signed_assignment_margin": best_score - candidates[1][0],
        }
    else:
        u, _singular_values, vt = np.linalg.svd(phone_basis.T)
        correction = np.eye(3)
        correction[-1, -1] = 1.0 if float(np.linalg.det(u @ vt)) >= 0.0 else -1.0
        rotation = u @ correction @ vt
    mapped = rotation @ phone_basis
    angular_errors_deg: dict[str, float] = {}
    for index, axis in enumerate(("x", "y", "z")):
        cosine = float(np.clip(mapped[index, index], -1.0, 1.0))
        angular_errors_deg[axis] = math.degrees(math.acos(cosine))
    quality = {
        "mapping_mode": mapping_mode,
        "displacement_m": lengths,
        "pairwise_unit_axis_dot": pairwise_dots,
        "raw_basis_determinant": determinant,
        "fit_angular_error_deg": angular_errors_deg,
        "max_fit_angular_error_deg": max(angular_errors_deg.values()),
        **signed_quality,
    }
    return rotation, quality


def load_teledex_calibration(path: str | Path) -> dict[str, Any]:
    calibration_path = Path(path)
    with calibration_path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if payload.get("schema_version") != CALIBRATION_SCHEMA_VERSION:
        raise ValueError(
            f"Unsupported TeleDex calibration schema in {calibration_path}: "
            f"{payload.get('schema_version')!r}."
        )
    matrix = np.asarray(payload.get("phone_to_robot_rotation_matrix"), dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"TeleDex calibration in {calibration_path} has no finite 3x3 matrix.")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=2e-3):
        raise ValueError(f"TeleDex calibration matrix in {calibration_path} is not orthonormal.")
    if not math.isclose(float(np.linalg.det(matrix)), 1.0, abs_tol=2e-3):
        raise ValueError(f"TeleDex calibration matrix in {calibration_path} must have determinant +1.")
    payload["phone_to_robot_rotation_matrix"] = matrix.astype(float).tolist()
    return payload


def calibration_from_project_config(config: Mapping[str, Any]) -> tuple[dict[str, Any] | None, str]:
    calibration_cfg = config.get("calibration", {})
    path = Path(str(calibration_cfg.get("file", "")))
    if not str(path) or str(path) == ".":
        return None, "calibration_file_not_configured"
    if not path.exists():
        return None, f"calibration_file_missing:{path}"
    try:
        return load_teledex_calibration(path), "ok"
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return None, f"calibration_invalid:{exc}"
