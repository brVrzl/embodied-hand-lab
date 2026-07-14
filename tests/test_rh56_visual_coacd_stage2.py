from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from sim_maniskill.rh56_collision_validation import (  # noqa: E402
    CommandProfile,
    ContactRecord,
    canonical_target_ctrl,
    classify_body_geom_pair,
    classify_empty_hand_terminal_contact,
    classify_representation_comparison,
    default_command_profiles,
    load_trajectory_manifest,
    object_contact_event_ordering,
    run_trajectory_validation,
    summarize_post_contact_response,
)
from sim_maniskill.rh56_collision_diagnostics import (  # noqa: E402
    classify_focused_root_cause,
    identify_coacd_part,
    static_cross_evaluate_qpos,
    visual_mesh_transform_matrix,
)
from view_mujoco_rh56_pose_contact import _build_pose_xml  # noqa: E402
from validate_rh56_visual_coacd_stage2 import (  # noqa: E402
    _annotate_mode_comparison,
    _build_stage2_xml,
)


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
    assert result.states["command_sequence"][0]["stage"] == "simultaneous"
    assert result.states["command_sequence"][0]["active_axis_names"] == [
        "thumb_lateral",
        "thumb_close",
        "index",
        "middle",
        "ring",
        "pinky",
    ]
    assert result.final_target_error < float(np.linalg.norm(target, ord=np.inf))
    assert result.assumptions["dynamic_validation_rule"].startswith("Intermediate hand motion")


def _summary_row(mode: str, *, reached: bool, blocked: bool = False, outcome: str = "reached") -> dict[str, object]:
    return {
        "scene": "object_free",
        "collision_mode": mode,
        "target_name": "sim_best_pinch",
        "strategy": "simultaneous",
        "outcome": outcome,
        "blockage_kind": "none" if reached else "persistent_mechanical_blockage",
        "reached": reached,
        "blocked": blocked,
        "first_contact_time": 0.1 if blocked else None,
        "first_blocking_pair": ["rh56_R_thumb_distal", "rh56_R_index_distal"] if blocked else None,
        "first_forbidden_pair": None,
        "max_rh56_self_penetration_m": 0.001 if blocked else 0.0,
        "final_target_error": 0.2 if blocked else 0.0,
    }


def test_mode_comparison_requires_visual_evidence_before_assigning_root_cause() -> None:
    rows = [
        _summary_row("visual_coacd", reached=False, blocked=True, outcome="blocked"),
        _summary_row("correll_mesh", reached=True),
        _summary_row("unifuc_pad_proxy", reached=True),
    ]

    comparisons = _annotate_mode_comparison(rows)  # type: ignore[arg-type]

    assert comparisons[0]["classification"] == "inconclusive"
    assert rows[0]["mode_comparison_classification"] == comparisons[0]["classification"]


def test_mode_comparison_labels_visual_consistent_blocking_and_permissive_references() -> None:
    rows = [
        _summary_row("visual_coacd", reached=False, blocked=True, outcome="blocked"),
        _summary_row("correll_mesh", reached=True),
        _summary_row("unifuc_pad_proxy", reached=True),
    ]
    rows[0]["original_visual_diagnostic"] = {"intersects": True}

    comparisons = _annotate_mode_comparison(rows)  # type: ignore[arg-type]

    assert comparisons[0]["classification"] == "visual_consistent_blocking_reference_modes_permissive"
    assert comparisons[0]["root_cause_classification"] == "shared_visual_or_kinematic_intersection"


def test_representation_comparison_labels_missed_visual_intersection() -> None:
    row = _summary_row("visual_coacd", reached=True)
    result = classify_representation_comparison(
        visual_coacd=row,
        references=[
            _summary_row("correll_mesh", reached=True),
            _summary_row("unifuc_pad_proxy", reached=True),
        ],
        original_visual_intersects=True,
    )

    assert result["classification"] == "collision_model_missed_visual_intersection"


def test_same_qpos_cross_evaluation_is_static_and_covers_all_modes(tmp_path: Path) -> None:
    xml_by_mode = {}
    for mode in ("visual_coacd", "correll_mesh", "unifuc_pad_proxy"):
        path = tmp_path / f"{mode}.xml"
        _build_stage2_xml(
            base_xml=Path("data/sim_assets/jaka_rh56.xml"),
            out_xml=path,
            collision_mode=mode,
            include_object=False,
        )
        xml_by_mode[mode] = path
    model = mujoco.MjModel.from_xml_path(str(xml_by_mode["visual_coacd"]))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    result = static_cross_evaluate_qpos(xml_by_mode, data.qpos, ctrl=data.ctrl)

    assert result["diagnostic_type"] == "static_same_qpos_collision_representation_cross_evaluation"
    assert result["dynamic_reachability_claim"] is False
    assert set(result["modes"]) == set(xml_by_mode)
    assert np.array_equal(np.asarray(result["qpos"]), data.qpos)


def test_coacd_geom_resolves_to_exact_manifest_part() -> None:
    part = identify_coacd_part("rh56_R_thumb_distal_visual_coacd_collision_003")

    assert part is not None
    assert part["manifest_id"] == "rh56_R_thumb_distal:003"
    assert part["collision_file"].endswith("R_thumb_distal_part003.stl")
    assert part["source_file"].endswith("R_thumb_distal.STL")
    assert identify_coacd_part("rh56_R_thumb_distal_geom_0") is None


def test_visual_mesh_transform_applies_scale_geom_then_body_transform() -> None:
    half_turn = np.sqrt(0.5)
    transform = visual_mesh_transform_matrix(
        body_position=(10.0, 0.0, 0.0),
        body_quaternion=(half_turn, 0.0, 0.0, half_turn),
        geom_position=(1.0, 0.0, 0.0),
        geom_quaternion=(1.0, 0.0, 0.0, 0.0),
        mesh_scale=(2.0, 1.0, 1.0),
    )
    transformed = transform @ np.asarray([1.0, 0.0, 0.0, 1.0])

    assert np.allclose(transformed, [10.0, 3.0, 0.0, 1.0], atol=1e-12)


def _focused_dynamic_rows() -> list[dict[str, object]]:
    rows = []
    for profile in ("slow_validation", "nominal", "hybrid"):
        rows.extend(
            [
                {
                    "profile": profile,
                    "collision_mode": "visual_coacd",
                    "blocked": True,
                    "reached": False,
                    "timeout": False,
                    "slow_progress": False,
                    "numerical_instability": False,
                },
                {
                    "profile": profile,
                    "collision_mode": "correll_mesh",
                    "blocked": False,
                    "reached": True,
                    "timeout": False,
                    "slow_progress": False,
                    "numerical_instability": False,
                },
                {
                    "profile": profile,
                    "collision_mode": "unifuc_pad_proxy",
                    "blocked": False,
                    "reached": True,
                    "timeout": False,
                    "slow_progress": False,
                    "numerical_instability": False,
                },
            ]
        )
    return rows


def _same_qpos_fixture() -> dict[str, object]:
    return {
        "modes": {
            "visual_coacd": {
                "thumb_index_contacts": [{"geom1": "thumb_part", "geom2": "index_part"}],
                "max_thumb_index_penetration_m": 0.001,
            },
            "correll_mesh": {"thumb_index_contacts": [], "max_thumb_index_penetration_m": 0.0},
            "unifuc_pad_proxy": {"thumb_index_contacts": [], "max_thumb_index_penetration_m": 0.0},
        }
    }


def test_focused_root_cause_classification_is_conservative() -> None:
    dynamic_rows = _focused_dynamic_rows()
    same_qpos = [_same_qpos_fixture()]

    confirmed = classify_focused_root_cause(
        dynamic_rows=dynamic_rows,
        same_qpos_evaluations=same_qpos,
        visual_mesh_diagnostics=[
            {"proximity": {"intersects": False, "minimum_surface_distance_m": 0.001}}
        ],
    )
    shared = classify_focused_root_cause(
        dynamic_rows=dynamic_rows,
        same_qpos_evaluations=same_qpos,
        visual_mesh_diagnostics=[
            {"proximity": {"intersects": True, "minimum_surface_distance_m": 0.0}}
        ],
    )
    near_touch = classify_focused_root_cause(
        dynamic_rows=dynamic_rows,
        same_qpos_evaluations=same_qpos,
        visual_mesh_diagnostics=[
            {"proximity": {"intersects": False, "minimum_surface_distance_m": 0.0001}}
        ],
    )
    timeout_rows = [dict(row) for row in dynamic_rows]
    timeout_rows[0]["timeout"] = True
    timeout_rows[0]["slow_progress"] = True
    inconclusive = classify_focused_root_cause(
        dynamic_rows=timeout_rows,
        same_qpos_evaluations=same_qpos,
        visual_mesh_diagnostics=[
            {"proximity": {"intersects": False, "minimum_surface_distance_m": 0.001}}
        ],
    )

    assert confirmed["classification"] == "confirmed_coacd_outward_approximation"
    assert shared["classification"] == "shared_visual_or_kinematic_intersection"
    assert near_touch["classification"] == "contact_timing_difference_near_visual_touching"
    assert inconclusive["classification"] == "inconclusive"


def test_empty_hand_distal_contact_is_terminal_but_not_free_space_reach() -> None:
    expected = classify_empty_hand_terminal_contact(
        target_semantics="contact_terminated_target",
        contact_category="legitimate_fingertip_or_pad_contact",
        regions=["fingertip_pad", "fingertip_pad"],
        vendor_visuals_touch_or_intersect=True,
        forbidden_structural_contact=False,
        tunnelling=False,
        numerical_instability=False,
    )
    unresolved = classify_empty_hand_terminal_contact(
        target_semantics="diagnostic_only",
        contact_category="legitimate_fingertip_or_pad_contact",
        regions=["fingertip_pad", "fingertip_pad"],
        vendor_visuals_touch_or_intersect=True,
        forbidden_structural_contact=False,
        tunnelling=False,
        numerical_instability=False,
    )

    assert expected["classification"] == "expected_terminal_hand_contact"
    assert expected["terminal_contact"] is True
    assert expected["successful_free_space_reach"] is False
    assert unresolved["classification"] == "terminal_contact_candidate_unresolved_target_semantics"


def _contact(
    time_s: float,
    body1: str,
    body2: str,
    *,
    severity: str = "allowed",
) -> ContactRecord:
    return ContactRecord(
        time=time_s,
        stage_index=0,
        stage="test",
        stage_elapsed_s=time_s,
        geom1="stage2_object" if body1 == "stage2_object_body" else f"{body1}_geom",
        geom2="stage2_object" if body2 == "stage2_object_body" else f"{body2}_geom",
        body1=body1,
        body2=body2,
        region1="object" if body1 == "stage2_object_body" else "fingertip_pad",
        region2="object" if body2 == "stage2_object_body" else "fingertip_pad",
        category="hand_object_contact",
        severity=severity,
        dist=-0.0001,
        pos=[0.0, 0.0, 0.0],
        normal=[1.0, 0.0, 0.0],
        normal_force=1.0,
        friction_force=0.0,
        constraint_force_norm=1.0,
    )


def test_object_mediated_contact_ordering() -> None:
    contacts = [
        _contact(0.10, "stage2_object_body", "rh56_R_thumb_distal"),
        _contact(0.20, "stage2_object_body", "rh56_R_thumb_distal"),
        _contact(0.20, "stage2_object_body", "rh56_R_index_distal"),
        _contact(0.30, "rh56_R_thumb_distal", "rh56_R_index_distal"),
        _contact(
            0.40,
            "rh56_R_thumb_proximal",
            "rh56_R_index_proximal",
            severity="forbidden",
        ),
    ]

    ordering = object_contact_event_ordering(contacts)

    assert ordering["first_thumb_object_contact_time"] == 0.10
    assert ordering["first_index_object_contact_time"] == 0.20
    assert ordering["first_bilateral_object_contact_time"] == 0.20
    assert ordering["first_thumb_index_self_contact_time"] == 0.30
    assert ordering["first_forbidden_structural_contact_time"] == 0.40


def test_stage2_trajectory_manifest_parses_and_validates() -> None:
    manifest = load_trajectory_manifest("configs/sim/rh56_stage2_trajectories.yaml")

    assert set(manifest["trajectories"]) >= {
        "open",
        "thumb_rotate",
        "real_pinch_v4",
        "sim_best_pinch",
        "power_close",
    }
    assert manifest["trajectories"]["sim_best_pinch"]["target_semantics"] == "diagnostic_only"
    assert manifest["hard_ci_assertions"] is False


def test_hold_after_contact_freezes_command_and_reports_penetration(tmp_path: Path) -> None:
    out_xml = tmp_path / "visual_coacd.xml"
    _build_pose_xml(Path("data/sim_assets/jaka_rh56.xml"), out_xml, collision_mode="visual_coacd")
    profile = default_command_profiles()["nominal"]
    result = run_trajectory_validation(
        mujoco.MjModel.from_xml_path(str(out_xml)),
        collision_mode="visual_coacd",
        target_name="sim_best_pinch",
        target_ctrl=canonical_target_ctrl("sim_best_pinch"),
        strategy="iterative_incremental",
        profile=profile,
        arm_qpos=np.zeros(6, dtype=np.float64),
        sample_stride=1,
        hold_after_persistent_distal_contact_seconds=0.04,
    )
    summary = summarize_post_contact_response(result)

    assert result.outcome == "contact_terminated_settled"
    assert result.states["diagnostic_hold_start_state"] is not None
    assert result.states["diagnostic_hold_final_state"] is not None
    assert summary["variant"].startswith("hold_current_actuator_target")
    assert summary["command_after_first_contact"]["ctrl_delta_inf"] is not None
