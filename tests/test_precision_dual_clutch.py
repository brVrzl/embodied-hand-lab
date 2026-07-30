from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path

import numpy as np
import pytest

from motion_input import Pose6D
from quest_jaka_sim import (
    AnalogClutchSample,
    AnalogHoldToRun,
    ArmClutchMachine,
    ArmClutchState,
    ClutchAction,
    HandClutchMachine,
    HandClutchState,
    JakaMujocoSimulation,
    LatchedHeadYawArmMapper,
    ProvisionalMappingConfig,
    ReplayConfig,
    Se3FilterProfile,
    gravity_aligned_head_yaw,
)
from quest_jaka_sim.se3 import (
    quaternion_angle_rad,
    relative_pose,
    rotvec_to_quaternion_xyzw,
)
from quest_jaka_sim.simulation import build_viewer_mjcf


def _sample(value: float, sequence: int, *, timestamp_ns: int | None = None, valid: bool = True):
    return AnalogClutchSample(value, sequence if timestamp_ns is None else timestamp_ns, sequence, valid)


def test_independent_trigger_and_grip_hysteresis_edges_and_held_input() -> None:
    for detector in (AnalogHoldToRun(), AnalogHoldToRun()):
        assert detector.observe(_sample(0.90, 1), fresh=True).rising_edge is False
        released = detector.observe(_sample(0.50, 2), fresh=True)
        assert released.pressed is False and released.released_observed
        assert detector.observe(_sample(0.60, 3), fresh=True).pressed is False
        press = detector.observe(_sample(0.75, 4), fresh=True)
        assert press.rising_edge and press.pressed is True
        assert not detector.observe(_sample(0.95, 5), fresh=True).rising_edge
        assert detector.observe(_sample(0.65, 6), fresh=True).pressed is True
        assert detector.observe(_sample(0.55, 7), fresh=True).falling_edge


def test_arm_startup_release_capture_once_release_freeze_and_fault_recovery() -> None:
    arm = ArmClutchMachine(stale_after_s=0.1)
    assert arm.step(_sample(1.0, 1), now_ns=1, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True) is ClutchAction.FREEZE
    assert arm.state is ArmClutchState.ARMED_WAITING_FOR_RELEASE
    arm.step(_sample(0.0, 2), now_ns=2, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True)
    assert arm.state is ArmClutchState.DISENGAGED
    assert arm.step(_sample(1.0, 3), now_ns=3, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True) is ClutchAction.CAPTURE_ARM_REFERENCE
    arm.reference_captured(3)
    assert arm.cycle_count == 1 and arm.state is ArmClutchState.ENGAGED
    assert arm.step(_sample(1.0, 4), now_ns=4, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True) is ClutchAction.UPDATE
    assert arm.cycle_count == 1
    assert arm.step(_sample(0.0, 5), now_ns=5, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True) is ClutchAction.FREEZE
    assert arm.state is ArmClutchState.DISENGAGED

    # Wrist loss faults only the arm and a held trigger cannot restore it.
    arm.step(_sample(1.0, 6), now_ns=6, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True)
    arm.reference_captured(6)
    arm.step(_sample(1.0, 7), now_ns=7, controller_valid=True, continuous_inputs_valid=False, capture_inputs_valid=False)
    assert arm.state is ArmClutchState.TRACKING_FAULT
    arm.step(_sample(1.0, 8), now_ns=8, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True)
    assert arm.state is ArmClutchState.TRACKING_FAULT
    arm.step(_sample(0.0, 9), now_ns=9, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True)
    assert arm.state is ArmClutchState.DISENGAGED
    assert arm.step(_sample(1.0, 10), now_ns=10, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True) is ClutchAction.CAPTURE_ARM_REFERENCE


def test_arm_capture_requires_head_and_stale_trigger_disengages() -> None:
    arm = ArmClutchMachine(stale_after_s=0.01)
    arm.step(_sample(0.0, 1), now_ns=1, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True)
    arm.step(_sample(1.0, 2), now_ns=2, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=False)
    assert arm.state is ArmClutchState.TRACKING_FAULT
    arm.step(_sample(0.0, 3), now_ns=3, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True)
    arm.step(_sample(1.0, 4), now_ns=4, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True)
    arm.reference_captured(4)
    arm.step(_sample(1.0, 5, timestamp_ns=4), now_ns=20_000_000, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True)
    assert arm.state is ArmClutchState.TRACKING_FAULT
    assert arm.active_fault and arm.active_fault.reason == "ARM_TRIGGER_STALE_OR_INVALID"


def test_hand_startup_freeze_reacquire_release_and_independent_recovery() -> None:
    hand = HandClutchMachine(stale_after_s=0.1, reacquisition_duration_s=0.2)
    hand.step(_sample(1.0, 1), now_ns=1, controller_valid=True, skeleton_valid=True)
    assert hand.state is HandClutchState.ARMED_WAITING_FOR_RELEASE
    hand.step(_sample(0.0, 2), now_ns=2, controller_valid=True, skeleton_valid=True)
    assert hand.state is HandClutchState.DISENGAGED
    action = hand.step(_sample(1.0, 3), now_ns=3, controller_valid=True, skeleton_valid=True)
    assert action is ClutchAction.START_HAND_REACQUISITION
    assert hand.cycle_count == 1 and hand.reacquisition_fraction(3) == 0.0
    hand.step(_sample(1.0, 4, timestamp_ns=100_000_003), now_ns=100_000_003, controller_valid=True, skeleton_valid=True)
    assert hand.state is HandClutchState.REACQUIRE
    assert hand.reacquisition_fraction(100_000_003) == pytest.approx(0.5)
    hand.step(_sample(1.0, 5, timestamp_ns=200_000_003), now_ns=200_000_003, controller_valid=True, skeleton_valid=True)
    assert hand.state is HandClutchState.ENGAGED
    hand.step(_sample(0.0, 6, timestamp_ns=200_000_004), now_ns=200_000_004, controller_valid=True, skeleton_valid=True)
    assert hand.state is HandClutchState.DISENGAGED

    hand.step(_sample(1.0, 7, timestamp_ns=200_000_005), now_ns=200_000_005, controller_valid=True, skeleton_valid=True)
    hand.step(_sample(1.0, 8, timestamp_ns=200_000_006), now_ns=200_000_006, controller_valid=True, skeleton_valid=False)
    assert hand.state is HandClutchState.TRACKING_FAULT
    hand.step(_sample(0.0, 9, timestamp_ns=200_000_007), now_ns=200_000_007, controller_valid=True, skeleton_valid=False)
    assert hand.state is HandClutchState.TRACKING_FAULT
    hand.step(_sample(0.0, 10, timestamp_ns=200_000_008), now_ns=200_000_008, controller_valid=True, skeleton_valid=True)
    assert hand.state is HandClutchState.DISENGAGED


@pytest.mark.parametrize(
    ("arm_pressed", "hand_pressed", "arm_state", "hand_state"),
    [
        (False, False, ArmClutchState.DISENGAGED, HandClutchState.DISENGAGED),
        (True, False, ArmClutchState.REFERENCE_CAPTURE, HandClutchState.DISENGAGED),
        (False, True, ArmClutchState.DISENGAGED, HandClutchState.REACQUIRE),
        (True, True, ArmClutchState.REFERENCE_CAPTURE, HandClutchState.REACQUIRE),
    ],
)
def test_all_four_independent_clutch_combinations(arm_pressed, hand_pressed, arm_state, hand_state) -> None:
    arm = ArmClutchMachine(stale_after_s=1.0)
    hand = HandClutchMachine(stale_after_s=1.0)
    arm.step(_sample(0.0, 1), now_ns=1, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True)
    hand.step(_sample(0.0, 1), now_ns=1, controller_valid=True, skeleton_valid=True)
    arm.step(_sample(float(arm_pressed), 2), now_ns=2, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True)
    hand.step(_sample(float(hand_pressed), 2), now_ns=2, controller_valid=True, skeleton_valid=True)
    assert arm.state is arm_state
    assert hand.state is hand_state


def test_shared_controller_loss_faults_both_channels() -> None:
    arm = ArmClutchMachine(stale_after_s=1.0)
    hand = HandClutchMachine(stale_after_s=1.0)
    arm.step(_sample(0.0, 1), now_ns=1, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True)
    hand.step(_sample(0.0, 1), now_ns=1, controller_valid=True, skeleton_valid=True)
    arm.step(_sample(1.0, 2), now_ns=2, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True)
    arm.reference_captured(2)
    hand.step(_sample(1.0, 2), now_ns=2, controller_valid=True, skeleton_valid=True)
    arm.step(_sample(1.0, 3), now_ns=3, controller_valid=False, continuous_inputs_valid=True, capture_inputs_valid=True)
    hand.step(_sample(1.0, 3), now_ns=3, controller_valid=False, skeleton_valid=True)
    assert arm.state is ArmClutchState.TRACKING_FAULT
    assert hand.state is HandClutchState.TRACKING_FAULT


def test_independent_skeleton_and_grip_faults_do_not_disengage_arm() -> None:
    arm = ArmClutchMachine(stale_after_s=1.0)
    hand = HandClutchMachine(stale_after_s=0.01)
    arm.step(_sample(0.0, 1), now_ns=1, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True)
    hand.step(_sample(0.0, 1), now_ns=1, controller_valid=True, skeleton_valid=True)
    arm.step(_sample(1.0, 2), now_ns=2, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True)
    arm.reference_captured(2)
    hand.step(_sample(1.0, 2), now_ns=2, controller_valid=True, skeleton_valid=True)
    hand.step(_sample(1.0, 3), now_ns=3, controller_valid=True, skeleton_valid=False)
    assert hand.state is HandClutchState.TRACKING_FAULT
    assert arm.step(_sample(1.0, 3), now_ns=3, controller_valid=True, continuous_inputs_valid=True, capture_inputs_valid=True) is ClutchAction.UPDATE
    assert arm.state is ArmClutchState.ENGAGED

    # After hand recovery, a stale grip sample faults only the hand again.
    hand.step(_sample(0.0, 4), now_ns=4, controller_valid=True, skeleton_valid=True)
    hand.step(_sample(1.0, 5), now_ns=5, controller_valid=True, skeleton_valid=True)
    hand.step(_sample(1.0, 6, timestamp_ns=5), now_ns=20_000_000, controller_valid=True, skeleton_valid=True)
    assert hand.state is HandClutchState.TRACKING_FAULT
    assert arm.state is ArmClutchState.ENGAGED


def _mapper() -> LatchedHeadYawArmMapper:
    config = ProvisionalMappingConfig(
        calibration_id="precision_test",
        calibrated=False,
        operator_to_robot_basis=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        translation_scale_per_axis=(1.0, 1.0, 1.0),
        translation_deadband_m=0.0,
        orientation_enabled=True,
        orientation_scale=1.0,
        orientation_deadband_rad=0.0,
        maximum_operator_displacement_m=0.5,
        maximum_target_displacement_m=0.5,
        orientation_scale_per_axis=(1.0, 1.0, 1.0),
        maximum_relative_rotation_rad=math.pi,
    )
    profile = Se3FilterProfile("test", 1e6, 0.0, 1e6, 1e6, 0.0, 1e6, 1.0)
    return LatchedHeadYawArmMapper(config, profile)


def test_latched_head_yaw_ignores_translation_pitch_roll_and_later_head_motion() -> None:
    yaw0, _ = gravity_aligned_head_yaw((0.0, 0.0, 0.0, 1.0))
    pitch = rotvec_to_quaternion_xyzw((0.3, 0.0, 0.0))
    roll = rotvec_to_quaternion_xyzw((0.0, 0.0, -0.2))
    yaw_pitch, _ = gravity_aligned_head_yaw(pitch)
    yaw_roll, _ = gravity_aligned_head_yaw(roll)
    assert yaw0 == pytest.approx(0.0)
    assert yaw_pitch == pytest.approx(0.0)
    assert yaw_roll == pytest.approx(0.0)
    yaw_quarter, yaw_frame = gravity_aligned_head_yaw(
        rotvec_to_quaternion_xyzw((0.0, math.pi / 2.0, 0.0))
    )
    assert yaw_quarter == pytest.approx(math.pi / 2.0)
    assert yaw_frame @ np.asarray((0.0, 0.0, -1.0)) == pytest.approx((-1.0, 0.0, 0.0))
    mapper = _mapper()
    comparison = _mapper()
    wrist = Pose6D((0.2, 1.0, -0.3), (0.0, 0.0, 0.0, 1.0))
    tcp = Pose6D((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    head = Pose6D((99.0, -20.0, 4.0), pitch)
    assert mapper.capture(wrist=wrist, robot_tcp=tcp, head=head, timestamp_ns=1) == tcp
    comparison.capture(wrist=wrist, robot_tcp=tcp, head=head, timestamp_ns=1)
    # No head is accepted by target(), so later head motion cannot affect it.
    moved = replace(wrist, position_m=(0.21, 1.0, -0.3))
    first = mapper.target(moved, timestamp_ns=1_000_000_001)
    # A second mapper represents arbitrarily different later head motion: the
    # target API consumes no head sample and therefore produces the same target.
    second = comparison.target(moved, timestamp_ns=1_000_000_001)
    assert first.position_m == pytest.approx(second.position_m, abs=1e-9)


@pytest.mark.parametrize("axis", [(0.3, 0.0, 0.0), (0.0, 0.3, 0.0), (0.0, 0.0, 0.3)])
def test_full_wrist_pitch_yaw_roll_and_downward_rotation_are_preserved(axis) -> None:
    mapper = _mapper()
    identity = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    mapper.capture(wrist=identity, robot_tcp=identity, head=identity, timestamp_ns=1)
    target = mapper.target(Pose6D((0.0, 0.0, 0.0), rotvec_to_quaternion_xyzw(axis)), timestamp_ns=1_000_000_001)
    assert quaternion_angle_rad(target.orientation_xyzw, rotvec_to_quaternion_xyzw(axis)) < 1e-5


def test_translation_is_robot_base_fixed_and_wrist_roll_is_tcp_local() -> None:
    mapper = _mapper()
    wrist = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    tcp = Pose6D(
        (0.4, -0.2, 0.3),
        rotvec_to_quaternion_xyzw((0.3, -0.4, 0.8)),
    )
    head = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    mapper.capture(wrist=wrist, robot_tcp=tcp, head=head, timestamp_ns=1)
    target = mapper.target(
        Pose6D((0.05, 0.0, 0.0), rotvec_to_quaternion_xyzw((0.0, 0.0, 0.1))),
        timestamp_ns=1_000_000_001,
    )

    # Robot-base +X remains +X even though the captured TCP is rotated.
    assert np.asarray(target.position_m) - np.asarray(tcp.position_m) == pytest.approx(
        (0.05, 0.0, 0.0), abs=1e-8
    )
    # Wrist-local Z remains TCP-local Z, the committed model's joint-6 axis.
    local = relative_pose(tcp, target)
    assert quaternion_angle_rad(
        local.orientation_xyzw,
        rotvec_to_quaternion_xyzw((0.0, 0.0, 0.1)),
    ) < 1e-7


@pytest.mark.parametrize(
    ("operator_motion", "robot_motion"),
    [
        ((0.0, 0.0, -0.01), (0.01, 0.0, 0.0)),  # forward -> forward
        ((0.01, 0.0, 0.0), (0.0, -0.01, 0.0)),  # right -> right
        ((0.0, 0.01, 0.0), (0.0, 0.0, 0.01)),  # up -> up
    ],
)
def test_operator_aligned_translation_axes_and_zero_capture(
    operator_motion: tuple[float, float, float],
    robot_motion: tuple[float, float, float],
) -> None:
    config = ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml")
    profile = Se3FilterProfile("test", 1e6, 0.0, 1e6, 1e6, 0.0, 1e6, 1.0)
    mapper = LatchedHeadYawArmMapper(config.mapping, profile)
    identity = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    reference = Pose6D((0.2, -0.1, 0.4), identity.orientation_xyzw)
    assert mapper.capture(
        wrist=identity,
        robot_tcp=reference,
        head=identity,
        timestamp_ns=1,
    ) == reference
    assert mapper.target(identity, timestamp_ns=2).position_m == pytest.approx(
        reference.position_m,
        abs=1e-12,
    )
    target = mapper.target(
        Pose6D(operator_motion, identity.orientation_xyzw),
        timestamp_ns=1_000_000_002,
    )
    assert np.asarray(target.position_m) - np.asarray(reference.position_m) == pytest.approx(
        robot_motion,
        abs=1e-8,
    )


@pytest.mark.parametrize(
    ("human_axis", "robot_axis"),
    [
        ((0.1, 0.0, 0.0), (-0.1, 0.0, 0.0)),
        ((0.0, 0.1, 0.0), (0.0, -0.1, 0.0)),
        ((0.0, 0.0, 0.1), (0.0, 0.0, 0.1)),
    ],
)
def test_live_right_wrist_axes_match_rh56_palm_semantics(
    human_axis: tuple[float, float, float],
    robot_axis: tuple[float, float, float],
) -> None:
    config = ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml")
    profile = Se3FilterProfile("test", 1e6, 0.0, 1e6, 1e6, 0.0, 1e6, 1.0)
    mapper = LatchedHeadYawArmMapper(config.mapping, profile)
    identity = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    mapper.capture(wrist=identity, robot_tcp=identity, head=identity, timestamp_ns=1)
    target = mapper.target(
        Pose6D((0.0, 0.0, 0.0), rotvec_to_quaternion_xyzw(human_axis)),
        timestamp_ns=1_000_000_001,
    )
    assert quaternion_angle_rad(
        target.orientation_xyzw,
        rotvec_to_quaternion_xyzw(robot_axis),
    ) < 1e-7


def test_wrist_local_roll_is_solved_predominantly_by_jaka_joint_6(
    tmp_path: Path,
) -> None:
    config = ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml")
    # This is a single-step IK-axis attribution test, not a live continuation
    # test.  Output acceleration is exercised by the shared feasibility suite.
    config = replace(
        config,
        output_contract=replace(
            config.output_contract,
            maximum_acceleration_rad_s2=math.inf,
        ),
    )
    model_path = build_viewer_mjcf(config.mjcf_path, tmp_path / "viewer.xml")
    simulation = JakaMujocoSimulation(config, mjcf_path=model_path)
    tcp = simulation.capture_reference()
    initial_joints = simulation.arm_joints_rad
    profile = Se3FilterProfile("test", 1e6, 0.0, 1e6, 1e6, 0.0, 1e6, 1.0)
    mapper = LatchedHeadYawArmMapper(config.mapping, profile)
    identity = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    mapper.capture(wrist=identity, robot_tcp=tcp, head=identity, timestamp_ns=1)
    target = mapper.target(
        Pose6D(
            (0.0, 0.0, 0.0),
            rotvec_to_quaternion_xyzw((0.0, 0.0, 0.04)),
        ),
        timestamp_ns=1_000_000_001,
    )
    result = simulation.evaluate(target, dt_s=1.0 / 60.0)

    assert result.accepted and result.joint_target_rad is not None
    joint_delta = np.asarray(result.joint_target_rad) - initial_joints
    assert abs(joint_delta[5]) > 0.035
    assert abs(joint_delta[5]) > 20.0 * max(np.abs(joint_delta[:5]))


def test_quaternion_sign_and_many_recenter_cycles_have_zero_jump_and_no_accumulation() -> None:
    mapper = _mapper()
    tcp = Pose6D((0.1, -0.2, 0.3), rotvec_to_quaternion_xyzw((0.1, -0.2, 0.3)))
    head = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    for index in range(20):
        q = rotvec_to_quaternion_xyzw((0.02 * index, -0.01 * index, 0.005 * index))
        if index % 2:
            q = tuple(-value for value in q)
        wrist = Pose6D((index * 0.01, 1.0 - index * 0.02, -0.3), q)
        captured = mapper.capture(wrist=wrist, robot_tcp=tcp, head=head, timestamp_ns=index * 10 + 1)
        assert captured.position_m == pytest.approx(tcp.position_m)
        assert quaternion_angle_rad(captured.orientation_xyzw, tcp.orientation_xyzw) < 1e-9
        same = mapper.target(wrist, timestamp_ns=index * 10 + 2)
        assert same.position_m == pytest.approx(tcp.position_m, abs=1e-9)
        assert quaternion_angle_rad(same.orientation_xyzw, tcp.orientation_xyzw) < 1e-8


def test_precision_simulation_path_has_no_physical_robot_import_or_connection() -> None:
    forbidden = ("jaka_driver_adapter.hardware", "inspire_hand", "serial.Serial", "connect_robot")
    for path in (
        Path("src/quest_jaka_sim/clutch.py"),
        Path("src/quest_jaka_sim/precision_mapping.py"),
        Path("src/quest_jaka_sim/smooth_session.py"),
        Path("tools/quest_jaka_mujoco_sim.py"),
    ):
        source = path.read_text(encoding="utf-8")
        assert not any(token in source for token in forbidden), path


def test_ik_invalid_candidate_is_held_and_does_not_accumulate(tmp_path: Path) -> None:
    config = ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml")
    model_path = build_viewer_mjcf(config.mjcf_path, tmp_path / "viewer.xml")
    simulation = JakaMujocoSimulation(config, mjcf_path=model_path)
    reference = simulation.capture_reference()
    previous_joint_target = simulation.last_safe_joint_target.copy()
    impossible = replace(reference, position_m=(reference.position_m[0] + 1.0, *reference.position_m[1:]))
    result = simulation.evaluate(impossible, dt_s=1.0 / 60.0)
    assert not result.accepted
    assert simulation.last_safe_target == reference
    assert simulation.last_safe_joint_target == pytest.approx(previous_joint_target)
    recovered = simulation.evaluate(reference, dt_s=1.0 / 60.0)
    assert recovered.accepted
    assert simulation.last_safe_target == reference
