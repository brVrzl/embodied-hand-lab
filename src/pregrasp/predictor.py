from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Iterable

import numpy as np

from pregrasp.correll_rh56dfx import CorrellLineGraspPlanner
from pregrasp.geometry import ObjectGeometry
from pregrasp.hardware_constraints import evaluate_rh56_hardware_constraints
from pregrasp.primitives import PregraspPrimitive, rh56_default_primitives


@dataclass(frozen=True, slots=True)
class PregraspCandidate:
    primitive: PregraspPrimitive
    score: float
    target_position_xyz: list[float]
    approach_axis_xyz: list[float]
    contact_strategy: str
    reasons: list[str]

    @property
    def hand_command(self) -> list[float]:
        return self.primitive.hand_command

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["primitive"] = self.primitive.to_dict()
        result["hand_command"] = self.hand_command
        return result


class GeometryAwarePregraspPredictor:
    def __init__(
        self,
        primitives: Iterable[PregraspPrimitive] | None = None,
        *,
        use_correll_assets: bool = True,
    ) -> None:
        self.primitives = list(primitives or rh56_default_primitives())
        if not self.primitives:
            raise ValueError("At least one pregrasp primitive is required.")
        self.use_correll_assets = use_correll_assets
        self._correll_line_planner: CorrellLineGraspPlanner | None = None
        self._correll_unavailable = False

    def predict(
        self,
        geometry: ObjectGeometry,
        *,
        task_mode: str = "pick",
        top_k: int = 3,
    ) -> list[PregraspCandidate]:
        if top_k <= 0:
            raise ValueError("top_k must be positive.")
        candidates = [self._score_primitive(primitive, geometry, task_mode) for primitive in self.primitives]
        candidates.extend(self._correll_candidates(geometry, task_mode))
        candidates.sort(key=lambda item: item.score, reverse=True)
        return candidates[:top_k]

    def _correll_candidates(self, geometry: ObjectGeometry, task_mode: str) -> list[PregraspCandidate]:
        if not self.use_correll_assets or task_mode not in {"pick", "hold"}:
            return []
        object_width = _object_planar_width(geometry)
        if not (0.010 <= object_width <= 0.120):
            return []
        try:
            planner = self._get_correll_line_planner()
            plan = planner.plan_line_width(object_width)
        except Exception:
            self._correll_unavailable = True
            return []

        primitive = PregraspPrimitive(
            name="correll_line_width",
            task_modes=("pick", "hold"),
            hand_command=plan.canonical_command,
            wrist_offset_xyz_m=[-0.045, 0.0, 0.010],
            approach_axis_xyz=[1.0, 0.0, -0.05],
            contact_strategy="correll_line_width_fk",
            shape_tags=("box", "flat", "elongated"),
            min_object_width_m=0.010,
            max_object_width_m=0.120,
            expected_contacts=("thumb_close", "thumb_lateral", "index"),
            notes=(
                "Generated from Correll RH56DFX floating-hand FK model; "
                f"target_width_m={plan.target_width_m:.4f}, achieved_width_m={plan.achieved_width_m:.4f}."
            ),
        )
        hardware = evaluate_rh56_hardware_constraints(primitive.hand_command)
        width_score = float(np.clip(1.0 - plan.width_error_m / 0.010, 0.0, 1.0))
        score = 0.50 + 0.20 * width_score
        reasons = ["correll_fk_width_plan", f"width_error_m:{plan.width_error_m:.4f}"]
        if geometry.shape_hint in primitive.shape_tags:
            score += 0.12
            reasons.append(f"shape:{geometry.shape_hint}")
        elif geometry.shape_hint == "round":
            score -= 0.10
            reasons.append("round_object_line_grasp_penalty")
        if object_width < 0.045:
            score += 0.06
            reasons.append("small_object_line_bias")
        if hardware.feasible:
            score += 0.04
            reasons.append("hardware_feasible")
        else:
            score -= 0.20 * hardware.thumb_index_blocking_risk
            reasons.extend(hardware.reasons)

        position = np.asarray(geometry.centroid_xyz, dtype=np.float64) + np.asarray(
            primitive.wrist_offset_xyz_m,
            dtype=np.float64,
        )
        axis = np.asarray(primitive.approach_axis_xyz, dtype=np.float64)
        axis = axis / np.linalg.norm(axis)
        return [
            PregraspCandidate(
                primitive=primitive,
                score=float(np.clip(score, 0.0, 1.0)),
                target_position_xyz=position.astype(float).tolist(),
                approach_axis_xyz=axis.astype(float).tolist(),
                contact_strategy=primitive.contact_strategy,
                reasons=reasons,
            )
        ]

    def _get_correll_line_planner(self) -> CorrellLineGraspPlanner:
        if self._correll_unavailable:
            raise RuntimeError("Correll RH56DFX assets are unavailable.")
        if self._correll_line_planner is None:
            self._correll_line_planner = CorrellLineGraspPlanner()
        return self._correll_line_planner

    def _score_primitive(
        self,
        primitive: PregraspPrimitive,
        geometry: ObjectGeometry,
        task_mode: str,
    ) -> PregraspCandidate:
        reasons: list[str] = []
        score = 0.0

        if task_mode in primitive.task_modes:
            score += 0.38
            reasons.append(f"task_mode:{task_mode}")
        elif "pre_align" in primitive.task_modes and task_mode == "pick":
            score += 0.10
            reasons.append("pre_align_fallback")
        else:
            score -= 0.30
            reasons.append("task_mismatch")

        object_width = _object_planar_width(geometry)
        if primitive.min_object_width_m <= object_width <= primitive.max_object_width_m:
            score += 0.27
            reasons.append("width_in_range")
        else:
            span = max(primitive.max_object_width_m - primitive.min_object_width_m, 1e-6)
            if object_width < primitive.min_object_width_m:
                penalty = min((primitive.min_object_width_m - object_width) / span, 1.0)
                reasons.append("object_too_small")
            else:
                penalty = min((object_width - primitive.max_object_width_m) / span, 1.0)
                reasons.append("object_too_large")
            score -= 0.22 * penalty

        if geometry.shape_hint in primitive.shape_tags:
            score += 0.22
            reasons.append(f"shape:{geometry.shape_hint}")
        elif "box" in primitive.shape_tags and geometry.shape_hint == "unknown":
            score += 0.06
            reasons.append("unknown_shape_box_prior")
        else:
            score -= 0.08
            reasons.append("shape_mismatch")

        if geometry.flatness < 0.22 and primitive.name in {"lateral_clamp", "palm_push"}:
            score += 0.12
            reasons.append("thin_object_bias")
        if geometry.max_width_m > 0.085 and primitive.name == "power_envelope":
            score += 0.08
            reasons.append("large_object_envelope_bias")
        if geometry.max_width_m < 0.030 and primitive.name == "tripod_support":
            score += 0.08
            reasons.append("small_object_tripod_bias")

        hardware = evaluate_rh56_hardware_constraints(primitive.hand_command)
        if hardware.feasible:
            score += 0.05
            reasons.append("hardware_feasible")
        else:
            score -= 0.35 * hardware.thumb_index_blocking_risk
            reasons.extend(hardware.reasons)

        position = np.asarray(geometry.centroid_xyz, dtype=np.float64) + np.asarray(
            primitive.wrist_offset_xyz_m,
            dtype=np.float64,
        )
        axis = np.asarray(primitive.approach_axis_xyz, dtype=np.float64)
        axis = axis / np.linalg.norm(axis)
        return PregraspCandidate(
            primitive=primitive,
            score=float(np.clip(score, 0.0, 1.0)),
            target_position_xyz=position.astype(float).tolist(),
            approach_axis_xyz=axis.astype(float).tolist(),
            contact_strategy=primitive.contact_strategy,
            reasons=reasons,
        )


def _object_planar_width(geometry: ObjectGeometry) -> float:
    extents = np.asarray(geometry.extents_xyz_m, dtype=np.float64)
    if extents.size != 3:
        raise ValueError("ObjectGeometry.extents_xyz_m must have 3 values.")
    return float(max(extents[0], extents[1]))
