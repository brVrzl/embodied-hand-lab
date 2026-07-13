from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from sim_maniskill.rh56_collision_validation import (  # noqa: E402
    CommandProfile,
    canonical_target_ctrl,
    classify_body_geom_pair,
    run_trajectory_validation,
)
from view_mujoco_rh56_pose_contact import _build_pose_xml  # noqa: E402


def test_thumb_index_distal_pad_contact_is_not_forbidden() -> None:
    classification = classify_body_geom_pair(
        "rh56_R_thumb_distal_visual_coacd_collision_000",
        "rh56_R_thumb_distal",
        "rh56_R_index_distal_visual_coacd_collision_000",
        "rh56_R_index_distal",
    )

    assert classification.category == "legitimate_fingertip_or_pad_contact"
    assert classification.severity == "allowed"
    assert not classification.forbidden


def test_thumb_index_proximal_structural_contact_is_forbidden() -> None:
    classification = classify_body_geom_pair(
        "rh56_R_thumb_proximal_visual_coacd_collision_000",
        "rh56_R_thumb_proximal",
        "rh56_R_index_proximal_visual_coacd_collision_000",
        "rh56_R_index_proximal",
    )

    assert classification.category == "proximal_or_dorsal_structural_contact"
    assert classification.forbidden


def test_reviewed_internal_pair_is_reported_separately() -> None:
    classification = classify_body_geom_pair(
        "rh56_R_index_proximal_visual_coacd_collision_000",
        "rh56_R_index_proximal",
        "rh56_R_index_distal_visual_coacd_collision_000",
        "rh56_R_index_distal",
    )

    assert classification.category == "reviewed_excluded_internal_pair"
    assert classification.reviewed
    assert classification.severity == "allowed"


def test_short_dynamic_trajectory_uses_mj_step_and_records_samples(tmp_path: Path) -> None:
    out_xml = tmp_path / "visual_coacd.xml"
    _build_pose_xml(Path("data/sim_assets/jaka_rh56.xml"), out_xml, collision_mode="visual_coacd")
    model = mujoco.MjModel.from_xml_path(str(out_xml))
    target = canonical_target_ctrl("real_pinch_v4") * 0.20
    profile = CommandProfile(
        name="test_fast_smoke",
        max_velocity_ctrl_per_s=(0.45, 0.45, 0.45, 0.45, 0.45, 0.45),
        max_accel_ctrl_per_s2=(2.0, 2.0, 2.0, 2.0, 2.0, 2.0),
        settle_seconds=0.02,
        hold_seconds=0.02,
        timeout_seconds=0.40,
        error_tolerance_ctrl=0.08,
        progress_window_seconds=0.05,
        force_blockage_threshold=1000.0,
    )

    result = run_trajectory_validation(
        model,
        collision_mode="visual_coacd",
        target_name="real_pinch_v4_scaled",
        target_ctrl=target,
        strategy="simultaneous",
        profile=profile,
        arm_qpos=np.zeros(6, dtype=np.float64),
        sample_stride=1,
    )

    assert result.samples
    assert result.states["initial_state"]["time"] < result.states["final_state"]["time"]
    assert result.final_target_error < float(np.linalg.norm(target, ord=np.inf))
    assert result.assumptions["dynamic_validation_rule"].startswith("Intermediate hand motion")
