from __future__ import annotations

import numpy as np
import mujoco

from pregrasp import (
    CORRELL_ACTUATOR_ORDER,
    CorrellLineGraspPlanner,
    GeometryAwarePregraspPredictor,
    canonical_norm_to_correll_ctrl,
    correll_ctrl_to_canonical_norm,
    validate_correll_assets,
)
from pregrasp.correll_rh56dfx import FORCE_SENSOR_NAMES, TORQUE_SENSOR_NAMES, asset_xml_path
from pregrasp.geometry import geometry_from_point_cloud


def _box_points(extents: tuple[float, float, float], n: int = 64) -> np.ndarray:
    rng = np.random.default_rng(17)
    return rng.uniform(-0.5, 0.5, size=(n, 3)) * np.asarray(extents, dtype=np.float64)


def test_correll_assets_compile_and_expose_expected_interfaces() -> None:
    validation = validate_correll_assets()

    assert validation.valid, validation.to_dict()
    assert validation.xml_models["floating_grasp"]["nu"] == 12
    assert validation.xml_models["floating_force"]["nsensor"] == 10
    assert validation.xml_models["fixed_force"]["nsensor"] == 10
    assert CORRELL_ACTUATOR_ORDER == ("pinky", "ring", "middle", "index", "thumb_proximal", "thumb_yaw")


def test_correll_assets_fill_sites_and_sensors_missing_from_current_mounted_anchor() -> None:
    mounted = mujoco.MjModel.from_xml_path("data/sim_assets/jaka_rh56.xml")
    floating_force = mujoco.MjModel.from_xml_path(str(asset_xml_path("floating_force")))

    assert mounted.nsite == 0
    assert mounted.nsensor == 0
    assert floating_force.nsite >= 5
    assert floating_force.nsensor == 10
    for name in FORCE_SENSOR_NAMES + TORQUE_SENSOR_NAMES:
        assert mujoco.mj_name2id(floating_force, mujoco.mjtObj.mjOBJ_SENSOR, name) >= 0


def test_correll_actuator_mapping_roundtrips_project_canonical_order() -> None:
    canonical = [0.15, 0.30, 0.45, 0.60, 0.75, 0.90]

    ctrl = canonical_norm_to_correll_ctrl(canonical)
    restored = correll_ctrl_to_canonical_norm(ctrl)

    assert len(ctrl) == 6
    assert np.allclose(restored, canonical)


def test_correll_line_planner_solves_width_from_imported_fk_model() -> None:
    planner = CorrellLineGraspPlanner()

    plan_40mm = planner.plan_line_width(0.040)
    plan_80mm = planner.plan_line_width(0.080)

    assert plan_40mm.width_error_m < 0.002
    assert plan_80mm.width_error_m < 0.002
    assert len(plan_40mm.canonical_command) == 6
    assert plan_40mm.canonical_command[0] > 0.0
    assert plan_40mm.canonical_command[4] > 0.0
    assert plan_40mm.to_dict()["correll_actuator_order"] == list(CORRELL_ACTUATOR_ORDER)


def test_geometry_predictor_includes_correll_fk_candidate_for_small_box() -> None:
    geometry = geometry_from_point_cloud(_box_points((0.040, 0.035, 0.030)), shape_hint="box")
    candidates = GeometryAwarePregraspPredictor().predict(geometry, task_mode="pick", top_k=4)

    by_name = {candidate.primitive.name: candidate for candidate in candidates}
    assert "correll_line_width" in by_name
    assert "correll_fk_width_plan" in by_name["correll_line_width"].reasons
