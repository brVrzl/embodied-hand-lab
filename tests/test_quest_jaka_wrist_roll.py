from __future__ import annotations

from dataclasses import replace
import math

import mujoco
import numpy as np
import pytest

from motion_input import Pose6D
from quest_jaka_sim.se3 import compose_pose, rotvec_to_quaternion_xyzw
from quest_jaka_sim.simulation import (
    FeasibilityReason,
    JakaMujocoSimulation,
    ReplayConfig,
)


CONFIG_PATH = "configs/sim/quest_hts_jaka_mini2_live_demo.yaml"
CONTROL_DT_S = 1.0 / 60.0


def _simulation() -> JakaMujocoSimulation:
    config = ReplayConfig.load(CONFIG_PATH)
    # These tests isolate IK pose accuracy and wrist-branch continuity.  The
    # live continuation loop's output-acceleration contract is covered by the
    # dedicated output-feasibility tests.
    config = replace(
        config,
        output_contract=replace(
            config.output_contract,
            maximum_acceleration_rad_s2=math.inf,
            maximum_velocity_rad_s_per_joint=(math.pi,) * 6,
        ),
    )
    return JakaMujocoSimulation(config)


def _set_configuration(
    simulation: JakaMujocoSimulation, joints_rad: np.ndarray
) -> Pose6D:
    simulation.data.qpos[simulation.arm_qpos_ids] = joints_rad
    mujoco.mj_forward(simulation.model, simulation.data)
    simulation.ik.set_arm_joints_rad(joints_rad.tolist())
    return simulation.capture_reference()


def _incremental_target(
    reference: Pose6D,
    *,
    rotvec_rad: tuple[float, float, float],
    translation_m: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Pose6D:
    return compose_pose(
        reference,
        Pose6D(translation_m, rotvec_to_quaternion_xyzw(rotvec_rad)),
    )


def _solve_five_steps(
    simulation: JakaMujocoSimulation,
    reference: Pose6D,
    *,
    final_rotvec_rad: tuple[float, float, float],
    final_translation_m: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    results = []
    for step in range(1, 6):
        fraction = step / 5.0
        target = _incremental_target(
            reference,
            rotvec_rad=tuple(fraction * v for v in final_rotvec_rad),
            translation_m=tuple(fraction * v for v in final_translation_m),
        )
        result = simulation.evaluate(target, dt_s=CONTROL_DT_S)
        results.append(result)
        assert result.accepted, result.reason
    return results


def test_pure_tool_axis_roll_preserves_full_pose_accuracy_and_continuity() -> None:
    simulation = _simulation()
    reference = simulation.capture_reference()

    results = _solve_five_steps(
        simulation,
        reference,
        final_rotvec_rad=(0.0, 0.0, math.radians(5.0)),
    )
    assert results[-1].metrics.ik_error_m < 1e-5
    assert results[-1].metrics.ik_orientation_error_rad < math.radians(0.01)
    assert not any(result.metrics.branch_switch for result in results)
    assert max(
        max(abs(delta) for delta in result.metrics.joint_delta_rad)
        for result in results
    ) < simulation.config.feasibility.maximum_joint_target_jump_rad


def test_reverse_tool_roll_is_continuous_without_wrap_or_posture_drift() -> None:
    simulation = _simulation()
    reference = simulation.capture_reference()
    initial = simulation.last_safe_joint_target.copy()
    solutions = []

    for angle_deg in (1, 2, 3, 4, 5, 4, 3, 2, 1, 0):
        result = simulation.evaluate(
            _incremental_target(
                reference,
                rotvec_rad=(0.0, 0.0, math.radians(angle_deg)),
            ),
            dt_s=CONTROL_DT_S,
        )
        assert result.accepted
        assert not result.metrics.branch_switch
        solutions.append(np.asarray(result.joint_target_rad))

    joint_steps = np.diff(np.asarray(solutions), axis=0)
    assert (
        np.max(np.abs(joint_steps))
        < simulation.config.feasibility.maximum_joint_target_jump_rad
    )
    assert np.max(np.abs(solutions[-1] - initial)) < math.radians(0.01)


@pytest.mark.parametrize(
    "axis",
    [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0)],
)
def test_pitch_and_yaw_preserve_full_pose_accuracy_and_continuity(axis) -> None:
    simulation = _simulation()
    reference = simulation.capture_reference()

    results = _solve_five_steps(
        simulation,
        reference,
        final_rotvec_rad=tuple(math.radians(5.0) * v for v in axis),
    )
    assert results[-1].metrics.ik_error_m < 1e-5
    assert results[-1].metrics.ik_orientation_error_rad < math.radians(0.01)
    assert not any(result.metrics.branch_switch for result in results)
    assert max(
        max(abs(delta) for delta in result.metrics.joint_delta_rad)
        for result in results
    ) < simulation.config.feasibility.maximum_joint_target_jump_rad


def test_combined_translation_pitch_and_roll_preserves_full_6d_target() -> None:
    simulation = _simulation()
    reference = simulation.capture_reference()

    results = _solve_five_steps(
        simulation,
        reference,
        final_rotvec_rad=(math.radians(3.0), 0.0, math.radians(4.0)),
        final_translation_m=(0.005, -0.003, 0.004),
    )

    assert results[-1].metrics.ik_error_m < 1e-5
    assert results[-1].metrics.ik_orientation_error_rad < math.radians(0.01)
    assert not any(result.metrics.branch_switch for result in results)


def test_near_wrist_singularity_roll_uses_jacobian_not_fixed_j5_guard() -> None:
    simulation = _simulation()
    joints = np.asarray(simulation.config.initial_arm_joints_rad, dtype=float)
    joints[4] = math.radians(17.0)
    reference = _set_configuration(simulation, joints)
    roll_results = _solve_five_steps(
        simulation,
        reference,
        final_rotvec_rad=(0.0, 0.0, math.radians(5.0)),
    )
    assert not any(result.metrics.branch_switch for result in roll_results)
    assert all(result.accepted for result in roll_results)

    simulation = _simulation()
    joints = np.asarray(simulation.config.initial_arm_joints_rad, dtype=float)
    joints[4] = math.radians(16.0)
    _set_configuration(simulation, joints)
    below_guard = joints.copy()
    below_guard[4] = math.radians(14.0)
    simulation.data.qpos[simulation.arm_qpos_ids] = below_guard
    mujoco.mj_forward(simulation.model, simulation.data)
    target = simulation.current_tcp_pose
    simulation.data.qpos[simulation.arm_qpos_ids] = joints
    mujoco.mj_forward(simulation.model, simulation.data)

    result = simulation.evaluate(target, dt_s=CONTROL_DT_S)
    assert result.accepted
    assert result.metrics.wrist_proximity_warning
    assert result.metrics.jacobian_condition < 60.0
    assert result.metrics.minimum_jacobian_singular_value > 0.0125


def test_j6_near_safe_limit_rejects_without_wrap_or_branch_flip() -> None:
    simulation = _simulation()
    joints = np.asarray(simulation.config.initial_arm_joints_rad, dtype=float)
    joints[5] = math.radians(353.8)
    reference = _set_configuration(simulation, joints)

    first = simulation.evaluate(
        _incremental_target(
            reference, rotvec_rad=(0.0, 0.0, math.radians(-1.0))
        ),
        dt_s=CONTROL_DT_S,
    )
    assert first.accepted
    second = simulation.evaluate(
        _incremental_target(
            reference, rotvec_rad=(0.0, 0.0, math.radians(-2.0))
        ),
        dt_s=CONTROL_DT_S,
    )
    assert not second.accepted
    assert second.reason is FeasibilityReason.JOINT_LIMIT
    assert not second.metrics.branch_switch
    assert max(abs(value) for value in second.metrics.joint_delta_rad) < math.radians(1.1)


def test_recorded_slow_elbow_singularity_candidate_is_rejected() -> None:
    """Regression from live cycle 12 at input sequence 5202 -> 5203."""

    simulation = _simulation()
    seed = np.radians((-87.652, -45.065, -10.963, -184.543, 65.780, 182.647))
    previous_target = Pose6D(
        (-0.001394920118, -0.463923012793, 0.363368123391),
        (-0.483826381709, 0.019188837191, -0.006883143903, 0.874931466385),
    )
    singular_target = Pose6D(
        (0.000045448826, -0.466331555683, 0.364762361484),
        (-0.484040213963, 0.020325379446, -0.005690676863, 0.874791155885),
    )
    simulation.ik.set_arm_joints_rad(seed.tolist())
    simulation.last_safe_joint_target = seed.copy()
    simulation.last_safe_target = previous_target
    simulation.last_safe_joint_velocity[:] = 0.0
    simulation.initial_tcp = previous_target

    result = simulation.evaluate(singular_target, dt_s=CONTROL_DT_S)

    assert not result.accepted
    assert result.reason is FeasibilityReason.NEAR_SINGULARITY
    assert (
        result.metrics.jacobian_condition > 60.0
        or result.metrics.minimum_jacobian_singular_value < 0.0125
    )
    assert np.allclose(simulation.last_safe_joint_target, seed)
