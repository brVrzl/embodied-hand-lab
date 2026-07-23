from __future__ import annotations

import math

import numpy as np
import pytest

from quest_jaka_sim import ReplayConfig, SharedJakaTargetGenerator
from quest_jaka_sim.se3 import bounded_pose_step, quaternion_angle_rad
from quest_jaka_sim.simulation import FeasibilityReason
from teleoperation.output_feasibility import JointOutputFeasibilityTracker


CONFIG = "configs/sim/quest_hts_jaka_mini2_live_demo.yaml"
PERIOD_NS = 8_000_000
Q212 = (
    1.6108104942490866,
    -0.1634258244684054,
    -1.1203194182446221,
    -0.2764266537763923,
    -0.8181052330451166,
    0.49270800503151646,
)
Q213 = (
    1.6082960194916314,
    -0.1623994661613463,
    -1.1220051457136564,
    -0.2685048085831845,
    -0.8152173030695117,
    0.46753787793466656,
)
Q214 = (
    1.5989877336691674,
    -0.15630842145305937,
    -1.1341166309481496,
    -0.2418470947539033,
    -0.7982159224170279,
    0.3956532760306145,
)
T212 = 666_185_828_171_068
T213 = 666_185_844_837_051
T214 = 666_185_861_504_127


def _tracker() -> JointOutputFeasibilityTracker:
    return JointOutputFeasibilityTracker(
        maximum_velocity_rad_s=math.pi,
        servo_period_ns=PERIOD_NS,
    )


def _establish_zero_baseline(tracker: JointOutputFeasibilityTracker) -> None:
    tracker.reset((0.0,) * 6)
    tracker.commit(
        tracker.preview((0.0,) * 6, generated_monotonic_ns=1_000_000_000)
    )


def test_below_and_exact_output_velocity_boundary_are_accepted() -> None:
    tracker = _tracker()
    _establish_zero_baseline(tracker)
    dt_s = 0.016
    below = tracker.preview(
        (math.pi * dt_s * 0.99, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_016_000_000,
    )
    assert below.feasible

    tracker = _tracker()
    _establish_zero_baseline(tracker)
    exact = tracker.preview(
        (math.pi * dt_s, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_016_000_000,
    )
    assert exact.feasible
    assert exact.maximum_velocity_rad_s == pytest.approx(math.pi, abs=1e-12)


def test_above_output_velocity_boundary_is_rejected_without_state_commit() -> None:
    tracker = _tracker()
    _establish_zero_baseline(tracker)
    prediction = tracker.preview(
        (math.pi * 0.016 * 1.01, 0.0, 0.0, 0.0, 0.0, 0.0),
        generated_monotonic_ns=1_016_000_000,
    )
    assert not prediction.feasible
    assert prediction.violating_joint_indices == (0,)
    with pytest.raises(ValueError, match="infeasible"):
        tracker.commit(prediction)


def test_failed_p4_prediction_includes_active_segment_replacement_residual() -> None:
    tracker = _tracker()
    tracker.reset(Q212)
    tracker.commit(tracker.preview(Q212, generated_monotonic_ns=T212))
    tracker.commit(tracker.preview(Q213, generated_monotonic_ns=T213))
    prediction = tracker.preview(Q214, generated_monotonic_ns=T214)

    direct_j6_velocity = (Q214[5] - Q213[5]) / ((T214 - T213) / 1e9)
    assert direct_j6_velocity == pytest.approx(-4.3129701876953135)
    assert prediction.predicted_velocity_rad_s[5] < direct_j6_velocity
    assert prediction.predicted_velocity_rad_s[5] == pytest.approx(
        -4.37336, abs=2e-4
    )
    assert prediction.violating_joint_indices == (5,)


def test_failed_p4_candidate_backtracks_before_accepted_target_boundary() -> None:
    config = ReplayConfig.load(CONFIG)
    generator = SharedJakaTargetGenerator(config)
    generator.synchronize_authoritative_arm_joints(list(Q213))
    generator.output_feasibility.commit(
        generator.output_feasibility.preview(Q213, generated_monotonic_ns=T213)
    )
    generator.ik.set_arm_joints_rad(list(Q214))
    desired = generator.current_tcp_pose
    generator.ik.set_arm_joints_rad(list(Q213))
    dt_s = (T214 - T213) / 1e9

    full = generator.evaluate(
        desired,
        dt_s=dt_s,
        generated_monotonic_ns=T214,
    )
    assert not full.accepted
    assert full.reason is FeasibilityReason.OUTPUT_VELOCITY_INFEASIBLE
    assert full.metrics.output_velocity_violating_joint_indices == (5,)
    assert np.asarray(generator.last_safe_joint_target) == pytest.approx(Q213)

    half, fraction = bounded_pose_step(
        generator.last_safe_target,
        desired,
        maximum_translation_m=float(
            np.linalg.norm(
                np.asarray(desired.position_m)
                - np.asarray(generator.last_safe_target.position_m)
            )
            * 0.5
        ),
        maximum_rotation_rad=(
            quaternion_angle_rad(
                generator.last_safe_target.orientation_xyzw,
                desired.orientation_xyzw,
            )
            * 0.5
        ),
    )
    assert fraction == pytest.approx(0.5)
    recovered = generator.evaluate(
        half,
        dt_s=dt_s,
        generated_monotonic_ns=T214,
    )
    assert recovered.accepted
    assert recovered.reason is FeasibilityReason.ACCEPTED
    assert recovered.metrics.predicted_output_maximum_joint_velocity_rad_s < math.pi
    assert not recovered.metrics.branch_switch


def test_output_contract_authority_is_separate_from_ik_pathology_guard() -> None:
    config = ReplayConfig.load(CONFIG)
    assert config.feasibility.maximum_joint_velocity_rad_s == pytest.approx(14.0)
    assert config.output_contract.maximum_velocity_rad_s == pytest.approx(math.pi)
    assert config.output_contract.servo_period_ns == PERIOD_NS
