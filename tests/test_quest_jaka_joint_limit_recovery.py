from __future__ import annotations

from dataclasses import replace

import numpy as np

from motion_input import Pose6D
from quest_jaka_sim import ReplayConfig, SharedJakaTargetGenerator
from quest_jaka_sim.se3 import compose_pose, rotvec_to_quaternion_xyzw


CONFIG = "configs/sim/quest_hts_jaka_mini2_live_demo.yaml"
MEASURED_NEAR_J4_LOWER = (
    1.5624361730255556,
    -0.12716468713111112,
    -0.9298765030155555,
    -6.208851628425556,
    -1.05620343212,
    6.192044108015556,
)


def _outward_j4_pose(generator: SharedJakaTargetGenerator) -> Pose6D:
    target = list(MEASURED_NEAR_J4_LOWER)
    target[3] -= 0.001
    generator.ik.set_authoritative_arm_joints_rad(target)
    pose = generator.current_tcp_pose
    generator.ik.set_authoritative_arm_joints_rad(list(MEASURED_NEAR_J4_LOWER))
    return pose


def test_multibranch_regularized_dls_finds_local_retreat_that_baseline_rejects() -> None:
    config = ReplayConfig.load(CONFIG)
    baseline = SharedJakaTargetGenerator(
        replace(config, ik_solver_mode="directional_limit_dls")
    )
    target = _outward_j4_pose(baseline)
    baseline.synchronize_authoritative_arm_joints(list(MEASURED_NEAR_J4_LOWER))
    baseline_result = baseline.evaluate(
        target,
        dt_s=1.0 / 60.0,
        generated_monotonic_ns=1_000_000_000,
        fresh_measured_joint_position_rad=MEASURED_NEAR_J4_LOWER,
    )

    multibranch = SharedJakaTargetGenerator(
        replace(config, ik_solver_mode="joint_limit_multibranch")
    )
    multibranch.synchronize_authoritative_arm_joints(list(MEASURED_NEAR_J4_LOWER))
    multibranch_result = multibranch.evaluate(
        _outward_j4_pose(multibranch),
        dt_s=1.0 / 60.0,
        generated_monotonic_ns=1_000_000_000,
        fresh_measured_joint_position_rad=MEASURED_NEAR_J4_LOWER,
    )

    assert not baseline_result.accepted
    assert multibranch_result.accepted
    assert multibranch_result.joint_target_rad[3] > MEASURED_NEAR_J4_LOWER[3]
    assert multibranch_result.metrics.branch_switch is False
    assert multibranch_result.metrics.recovery_disposition == ""


def test_joint_limit_recovery_is_bounded_and_reports_percentiles() -> None:
    config = ReplayConfig.load(CONFIG)
    generator = SharedJakaTargetGenerator(config)
    seed = np.asarray(config.initial_arm_joints_rad, dtype=float)
    seed[5] = np.deg2rad(353.8)
    generator.synchronize_authoritative_arm_joints(seed.tolist())
    reference = generator.capture_reference()
    target = compose_pose(
        reference,
        Pose6D((0.0, 0.0, 0.0), rotvec_to_quaternion_xyzw((0.0, 0.0, -0.0174533))),
    )

    result = generator.evaluate(target, dt_s=1.0 / 60.0)
    report = generator.metrics_report()

    assert result.metrics.recovery_triggered
    assert 1 <= result.metrics.recovery_attempted_seeds <= 2
    assert result.metrics.recovery_solved_candidates >= 1
    assert result.metrics.recovery_disposition in {
        "ACCEPTED_PRIMARY",
        "ALTERNATE_SELECTED",
        "PHYSICALLY/LOCALLY_NO_SAFE_SOLUTION",
        "ALTERNATE_BRANCH_FOUND_BUT_NOT_CONTINUOUSLY_SWITCHABLE",
    }
    assert set(report["normal_ik_duration_ms"]) == {"p50", "p95", "p99", "max"}
    assert set(report["recovery_duration_ms"]) == {"p50", "p95", "p99", "max"}
    assert report["recovery_attempted_seeds"] <= 2 * report["joint_limit_recovery_count"]


def test_healthy_path_a_b_solver_modes_remain_branch_continuous() -> None:
    config = ReplayConfig.load(CONFIG)
    target_q = np.asarray(config.initial_arm_joints_rad, dtype=float)
    target_q[5] += 0.01
    results = []
    for mode in ("directional_limit_dls", "joint_limit_multibranch"):
        generator = SharedJakaTargetGenerator(replace(config, ik_solver_mode=mode))
        generator.synchronize_authoritative_arm_joints(list(config.initial_arm_joints_rad))
        generator.ik.set_authoritative_arm_joints_rad(target_q.tolist())
        target = generator.current_tcp_pose
        generator.ik.set_authoritative_arm_joints_rad(list(config.initial_arm_joints_rad))
        result = generator.evaluate(target, dt_s=1.0 / 60.0)
        assert result.accepted
        assert not result.metrics.branch_switch
        results.append(np.asarray(result.joint_target_rad))
    assert np.max(np.abs(results[0] - results[1])) < 0.05
