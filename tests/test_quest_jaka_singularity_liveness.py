from __future__ import annotations

import math

import numpy as np
import pytest

from jaka_driver_adapter.palm_target_ik import PalmTargetIkState
from quest_jaka_sim import JakaMujocoSimulation, ReplayConfig, SharedJakaTargetGenerator
from quest_jaka_sim.simulation import (
    CandidateMetrics,
    FeasibilityReason,
    classify_candidate,
)


CONFIG = "configs/sim/quest_hts_jaka_mini2_live_demo.yaml"
PHYSICAL_Q = np.asarray(
    (
        1.42867414143,
        -0.131510556894,
        -1.30657090905,
        0.620202739117,
        -0.415894500363,
        -0.698044422428,
    )
)


def _pose_at(generator: SharedJakaTargetGenerator, joints: np.ndarray):
    generator.ik.set_arm_joints_rad(joints.tolist())
    return generator.current_tcp_pose


def _evaluate_j5(
    generator: SharedJakaTargetGenerator, start_deg: float, target_deg: float
):
    start = PHYSICAL_Q.copy()
    start[4] = math.radians(start_deg)
    target = start.copy()
    target[4] = math.radians(target_deg)
    generator.synchronize_authoritative_arm_joints(start.tolist())
    pose = _pose_at(generator, target)
    generator.ik.set_arm_joints_rad(start.tolist())
    return generator.evaluate(pose, dt_s=1.0 / 60.0)


def test_manual02_j5_condition_is_warning_only_not_hard_rejection() -> None:
    config = ReplayConfig.load(CONFIG)
    metrics = CandidateMetrics(
        wrist_bend_from_singularity_rad=math.radians(14.968),
        jacobian_condition=31.47,
        minimum_jacobian_singular_value=0.02182,
        wrist_proximity_warning=True,
    )
    assert classify_candidate(metrics, config.feasibility) is FeasibilityReason.ACCEPTED
    assert math.degrees(config.feasibility.wrist_proximity_warning_rad) == pytest.approx(15.0)


def test_model_manual02_proximity_is_accepted_and_diagnostic() -> None:
    result = _evaluate_j5(
        SharedJakaTargetGenerator(ReplayConfig.load(CONFIG)), -16.0, -14.968
    )
    assert result.accepted
    assert result.metrics.wrist_proximity_warning
    assert result.metrics.singularity_state == "PROXIMITY"
    assert result.metrics.jacobian_condition < 60.0
    assert result.metrics.minimum_jacobian_singular_value > 0.0125


def test_directional_slowdown_allows_away_and_tangent_but_backtracks_toward() -> None:
    config = ReplayConfig.load(CONFIG)
    generator = SharedJakaTargetGenerator(config)

    toward = _evaluate_j5(generator, -10.0, -9.5)
    assert not toward.accepted
    assert toward.reason is FeasibilityReason.SINGULARITY_SLOWDOWN
    assert toward.metrics.singularity_direction == "TOWARD"

    away = _evaluate_j5(generator, -10.0, -11.0)
    assert away.accepted
    assert away.metrics.singularity_direction == "AWAY"

    start = PHYSICAL_Q.copy()
    start[4] = math.radians(-10.0)
    tangent_target = start.copy()
    tangent_target[0] += 0.001
    generator.synchronize_authoritative_arm_joints(start.tolist())
    pose = _pose_at(generator, tangent_target)
    generator.ik.set_arm_joints_rad(start.tolist())
    tangent = generator.evaluate(pose, dt_s=1.0 / 60.0)
    assert tangent.accepted
    assert tangent.metrics.singularity_direction == "TANGENT"
    assert tangent.metrics.singularity_state == "SLOWDOWN"


def test_true_hard_boundary_rejects_deeper_motion_but_permits_retreat() -> None:
    generator = SharedJakaTargetGenerator(ReplayConfig.load(CONFIG))
    hard = _evaluate_j5(generator, -10.0, -7.5)
    assert not hard.accepted
    assert hard.reason is FeasibilityReason.NEAR_SINGULARITY
    assert hard.metrics.singularity_state == "HARD"

    retreat = _evaluate_j5(generator, -7.0, -8.5)
    assert retreat.accepted
    assert retreat.metrics.singularity_direction == "AWAY"
    assert not retreat.metrics.branch_switch

    start = PHYSICAL_Q.copy()
    start[4] = math.radians(-7.0)
    tangent_q = start.copy()
    tangent_q[0] += 0.001
    generator.synchronize_authoritative_arm_joints(start.tolist())
    tangent_pose = _pose_at(generator, tangent_q)
    generator.ik.set_arm_joints_rad(start.tolist())
    severe = generator.evaluate(tangent_pose, dt_s=1.0 / 60.0)
    assert not severe.accepted
    assert severe.metrics.current_hard_singularity
    assert severe.metrics.hard_stop_required


def test_slowdown_hysteresis_does_not_chatter_and_releases_after_recovery() -> None:
    generator = SharedJakaTargetGenerator(ReplayConfig.load(CONFIG))
    start = PHYSICAL_Q.copy()
    start[4] = math.radians(-10.0)
    generator.synchronize_authoritative_arm_joints(start.tolist())

    toward_q = start.copy()
    toward_q[4] = math.radians(-9.5)
    toward_pose = _pose_at(generator, toward_q)
    generator.ik.set_arm_joints_rad(start.tolist())
    toward = generator.evaluate(toward_pose, dt_s=1.0 / 60.0)
    assert toward.metrics.singularity_state == "SLOWDOWN"
    assert generator._singularity_slowdown_latched

    away_q = start.copy()
    away_q[4] = math.radians(-10.5)
    away_pose = _pose_at(generator, away_q)
    generator.ik.set_arm_joints_rad(start.tolist())
    away_inside_band = generator.evaluate(away_pose, dt_s=1.0 / 60.0)
    assert away_inside_band.accepted
    assert away_inside_band.metrics.singularity_state == "SLOWDOWN"

    recovered_q = start.copy()
    recovered_q[4] = math.radians(-12.0)
    recovered_pose = _pose_at(generator, recovered_q)
    generator.ik.set_arm_joints_rad(list(away_inside_band.joint_target_rad))
    recovered = generator.evaluate(recovered_pose, dt_s=1.0 / 60.0)
    assert recovered.accepted
    assert recovered.metrics.singularity_state == "PROXIMITY"
    assert not generator._singularity_slowdown_latched


def test_adaptive_damping_changes_smoothly_with_solver_sigma() -> None:
    config = ReplayConfig.load(CONFIG)
    values = config.raw["simulation"]
    ik = PalmTargetIkState(
        list(config.initial_arm_joints_rad),
        mjcf_path=config.mjcf_path,
        ik_damping=config.ik_damping,
        adaptive_damping_sigma_start=values["adaptive_damping_sigma_start"],
        adaptive_damping_sigma_full=values["adaptive_damping_sigma_full"],
        adaptive_damping_max=values["adaptive_damping_max"],
    )
    high = ik._effective_damping(np.diag([1, 1, 1, 1, 1, 0.03]))
    middle = ik._effective_damping(np.diag([1, 1, 1, 1, 1, 0.01875]))
    low = ik._effective_damping(np.diag([1, 1, 1, 1, 1, 0.0125]))
    assert high == pytest.approx(0.05)
    assert high < middle < low
    assert low == pytest.approx(0.10)


def test_simulation_and_plant_free_generator_match_directional_policy() -> None:
    config = ReplayConfig.load(CONFIG)
    simulation = JakaMujocoSimulation(config)
    hardware = SharedJakaTargetGenerator(config)
    start = PHYSICAL_Q.copy()
    start[4] = math.radians(-10.0)
    destination = start.copy()
    destination[4] = math.radians(-9.5)
    target = _pose_at(hardware, destination)
    results = []
    for generator in (simulation, hardware):
        generator.synchronize_authoritative_arm_joints(start.tolist())
        results.append(generator.evaluate(target, dt_s=1.0 / 60.0))
    left, right = results
    assert left.accepted == right.accepted
    assert left.reason is right.reason
    assert left.joint_target_rad == right.joint_target_rad
    assert left.metrics == right.metrics
