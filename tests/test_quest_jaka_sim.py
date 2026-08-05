from __future__ import annotations

import ast
from dataclasses import replace
import math
from pathlib import Path

import pytest
import numpy as np

from motion_input import (
    OfflineOperatorTarget,
    OperatorInputState,
    ReceivedHtsDatagram,
)
from quest_jaka_sim import (
    FeasibilityLimits,
    FeasibilityReason,
    JakaMujocoSimulation,
    ProvisionalMappingConfig,
    ProvisionalOperatorToRobotMapper,
    QuestJakaReplaySession,
    ReplayConfig,
)
from quest_jaka_sim.simulation import (
    CandidateMetrics,
    CommandTrajectoryLimits,
    build_viewer_mjcf,
    classify_candidate,
    jerk_limited_position_step,
)
from motion_input import Pose6D
from quest_jaka_sim.se3 import (
    compose_pose,
    quaternion_angle_rad,
    quaternion_slerp_xyzw,
    relative_pose,
    rotvec_to_quaternion_xyzw,
)


def _mapping(**overrides: object) -> ProvisionalMappingConfig:
    values = {
        "calibration_id": "uncalibrated_test",
        "calibrated": False,
        "operator_to_robot_basis": ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
        "translation_scale_per_axis": (0.04, 0.03, 0.04),
        "translation_deadband_m": 0.001,
        "orientation_enabled": False,
        "orientation_scale": 0.0,
        "orientation_deadband_rad": math.radians(2.0),
        "maximum_operator_displacement_m": 0.30,
        "maximum_target_displacement_m": 0.015,
    }
    values.update(overrides)
    return ProvisionalMappingConfig(**values)


def _operator(
    delta: tuple[float, float, float],
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> OfflineOperatorTarget:
    return OfflineOperatorTarget(
        timestamp_monotonic_ns=1,
        state=OperatorInputState.ENGAGED,
        valid_for_mapping=True,
        emergency_neutral=False,
        frame_id="canonical_operator",
        translation_m=delta,
        orientation_xyzw=orientation,
        reference_host_sequence=1,
        current_host_sequence=2,
        reason="test",
    )


def test_frame_chain_axis_permutation_sign_and_translation_scaling() -> None:
    mapper = ProvisionalOperatorToRobotMapper(_mapping())
    mapper.capture_robot_reference(Pose6D((0.1, -0.4, 0.2), (0.0, 0.0, 0.0, 1.0)))
    target = mapper.map(_operator((0.10, 0.20, -0.05)))
    # operator X -> robot X, operator Z -> robot Y, operator Y -> robot Z.
    assert target.position_m == pytest.approx((0.104, -0.402, 0.206))

    signed = ProvisionalOperatorToRobotMapper(
        _mapping(operator_to_robot_basis=((-1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0)))
    )
    signed.capture_robot_reference(Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    assert signed.map(_operator((0.10, 0.0, 0.0))).position_m[0] == pytest.approx(-0.004)


def test_reference_capture_zero_deadband_and_target_envelope() -> None:
    mapper = ProvisionalOperatorToRobotMapper(_mapping())
    reference = Pose6D((0.2, -0.3, 0.4), (0.0, 0.0, 0.0, 1.0))
    mapper.capture_robot_reference(reference)
    assert mapper.map(_operator((0.0, 0.0, 0.0))) == reference
    assert mapper.map(_operator((0.0005, 0.0, 0.0))) == reference

    bounded = ProvisionalOperatorToRobotMapper(
        _mapping(maximum_target_displacement_m=0.001)
    )
    bounded.capture_robot_reference(reference)
    with pytest.raises(RuntimeError, match="OUTSIDE_ROBOT_WORKSPACE"):
        bounded.map(_operator((0.10, 0.0, 0.0)))


def test_identity_and_90_degree_quaternion_mapping() -> None:
    mapper = ProvisionalOperatorToRobotMapper(
        _mapping(
            operator_to_robot_basis=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            orientation_enabled=True,
            orientation_scale=1.0,
        )
    )
    mapper.capture_robot_reference(Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    assert mapper.map(_operator((0.0, 0.0, 0.0))).orientation_xyzw == (0.0, 0.0, 0.0, 1.0)
    root = math.sqrt(0.5)
    target = mapper.map(_operator((0.0, 0.0, 0.0), (0.0, 0.0, root, root)))
    assert target.orientation_xyzw == pytest.approx((0.0, 0.0, root, root))


def test_robot_reference_composes_delta_on_the_right_in_reference_frame() -> None:
    mapper = ProvisionalOperatorToRobotMapper(
        _mapping(
            operator_to_robot_basis=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            orientation_enabled=True,
            orientation_scale=1.0,
        )
    )
    robot_reference_q = rotvec_to_quaternion_xyzw((0.0, 0.0, math.pi / 2.0))
    mapper.capture_robot_reference(Pose6D((1.0, 2.0, 3.0), robot_reference_q))
    target = mapper.map(
        _operator(
            (0.10, 0.0, 0.0),
            rotvec_to_quaternion_xyzw((math.pi / 4.0, 0.0, 0.0)),
        )
    )
    # Local +X at the captured TCP points along robot-base +Y.
    assert target.position_m == pytest.approx((1.0, 2.004, 3.0), abs=1e-9)
    expected = rotvec_to_quaternion_xyzw((math.pi / 4.0, 0.0, 0.0))
    # inv(reference) * target recovers the commanded local delta.
    from quest_jaka_sim.se3 import relative_pose

    assert quaternion_angle_rad(
        relative_pose(Pose6D((1.0, 2.0, 3.0), robot_reference_q), target).orientation_xyzw,
        expected,
    ) < 1e-8


def _limits() -> FeasibilityLimits:
    return FeasibilityLimits(
        ik_position_tolerance_m=0.0025,
        maximum_jacobian_condition=40.0,
        minimum_jacobian_singular_value=0.02,
        maximum_target_jump_m=0.004,
        maximum_tcp_velocity_m_s=0.025,
        maximum_tcp_angular_velocity_rad_s=0.2,
        maximum_joint_velocity_rad_s=1.2,
        maximum_joint_acceleration_rad_s2=20.0,
        joint_limit_margin_rad=math.radians(5),
        maximum_target_displacement_m=0.015,
        ik_orientation_tolerance_rad=math.radians(3),
        maximum_target_rotation_jump_rad=math.radians(8),
        maximum_joint_target_jump_rad=0.12,
    )


@pytest.mark.parametrize(
    ("metrics", "expected"),
    [
        (CandidateMetrics(target_displacement_m=0.02), FeasibilityReason.OUTSIDE_ROBOT_WORKSPACE),
        (CandidateMetrics(target_jump_m=0.005), FeasibilityReason.TARGET_JUMP),
        (
            CandidateMetrics(target_rotation_jump_rad=math.radians(9)),
            FeasibilityReason.TARGET_JUMP,
        ),
        (CandidateMetrics(tcp_velocity_m_s=0.03), FeasibilityReason.LINEAR_VELOCITY_LIMIT),
        (CandidateMetrics(maximum_joint_acceleration_rad_s2=21.0), FeasibilityReason.LINEAR_ACCELERATION_LIMIT),
        (CandidateMetrics(ik_error_m=0.003), FeasibilityReason.IK_POSITION_FAILED),
        (
            CandidateMetrics(ik_orientation_error_rad=math.radians(4.0)),
            FeasibilityReason.IK_ORIENTATION_FAILED,
        ),
        (
            CandidateMetrics(tcp_angular_velocity_rad_s=0.3),
            FeasibilityReason.ANGULAR_VELOCITY_LIMIT,
        ),
        (CandidateMetrics(joint_limit_blockers=("joint_2_above_safe_limit",)), FeasibilityReason.JOINT_LIMIT),
        (CandidateMetrics(jacobian_condition=41.0), FeasibilityReason.NEAR_SINGULARITY),
        (CandidateMetrics(self_collision=True), FeasibilityReason.SELF_COLLISION),
        (CandidateMetrics(environment_collision=True), FeasibilityReason.ENVIRONMENT_COLLISION),
    ],
)
def test_structured_feasibility_rejections(
    metrics: CandidateMetrics, expected: FeasibilityReason
) -> None:
    assert classify_candidate(metrics, _limits()) is expected


def test_singularity_gate_rejects_slow_geometric_approach() -> None:
    limits = replace(
        _limits(),
        maximum_joint_velocity_rad_s=14.0,
    )
    geometry_only = CandidateMetrics(
        jacobian_condition=41.0,
        maximum_joint_velocity_rad_s=2.0,
    )
    amplified_velocity = CandidateMetrics(
        jacobian_condition=41.0,
        maximum_joint_velocity_rad_s=math.pi + 0.01,
    )
    assert classify_candidate(geometry_only, limits) is FeasibilityReason.NEAR_SINGULARITY
    assert (
        classify_candidate(amplified_velocity, limits)
        is FeasibilityReason.NEAR_SINGULARITY
    )


def test_wrist_angle_proximity_is_warning_only_when_jacobian_is_healthy() -> None:
    limits = replace(
        _limits(),
        wrist_proximity_warning_rad=math.radians(15.0),
    )
    metrics = CandidateMetrics(
        wrist_bend_from_singularity_rad=math.radians(14.0),
        maximum_joint_velocity_rad_s=0.1,
    )
    assert classify_candidate(metrics, limits) is FeasibilityReason.ACCEPTED


def test_recorded_circle_path_stays_on_bounded_wrist_branch(tmp_path: Path) -> None:
    """Compact regression from arm-clutch cycle 1 of the 2026-07-21 live log.

    With the former 35 degree J5 start, the full 551-frame recording preserved
    nearly the same TCP orientation while J4/J6 counter-wound +5.52/-5.92 rad.
    These twelve keyframes retain that path shape without making the test depend
    on an ignored local recording.
    """

    recorded_keyframes = (
        ((-0.0227610331, -0.4603408104, 0.2370831008), (-0.7003653324, 0.0769372049, 0.0068500040, 0.7095929432)),
        ((0.0115884081, -0.4342531486, 0.2344796541), (-0.6847500007, 0.1680939669, -0.0743721409, 0.7052167323)),
        ((0.0314699472, -0.4269222016, 0.2287253612), (-0.6650264877, 0.2019184978, -0.1177147797, 0.7093038288)),
        ((-0.0237109437, -0.4590080836, 0.2312268693), (-0.6869873848, 0.1602562061, -0.0267559674, 0.7082728286)),
        ((-0.0951401149, -0.4833353161, 0.2338192275), (-0.7026667253, 0.1475501444, 0.0524309898, 0.6940745056)),
        ((-0.1175140654, -0.4898344113, 0.2409157248), (-0.7130018057, 0.1486561429, 0.1101975874, 0.6763033845)),
        ((-0.0961488517, -0.4833064224, 0.2229916699), (-0.7062119545, 0.1388209104, 0.0690994180, 0.6908101770)),
        ((-0.1295931640, -0.4430617953, 0.2238505832), (-0.6792135480, 0.1556769563, 0.0330910447, 0.7164765343)),
        ((-0.1536497689, -0.3946690101, 0.2130952379), (-0.6371986157, 0.2001998542, 0.0070240752, 0.7442100543)),
        ((-0.1082179545, -0.3995626843, 0.2140292391), (-0.6432192673, 0.1852373690, -0.0145440217, 0.7427951014)),
        ((-0.0983849508, -0.4136567102, 0.3087390121), (-0.5521841681, 0.1503628687, 0.0103063932, 0.8199862380)),
        ((-0.1373385548, -0.4240527338, 0.2174116154), (-0.6702814379, 0.1384352242, 0.1032194514, 0.7217369518)),
    )
    recorded = tuple(Pose6D(position, orientation) for position, orientation in recorded_keyframes)
    config = ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml")
    # This regression isolates IK wrist-branch continuity along the recorded
    # Cartesian path; live continuation/output-acceleration behavior is tested
    # separately at the shared pipeline boundary.
    config = replace(
        config,
        output_contract=replace(
            config.output_contract,
            maximum_acceleration_rad_s2=math.inf,
            maximum_velocity_rad_s_per_joint=(math.pi,) * 6,
        ),
    )
    model = build_viewer_mjcf(config.mjcf_path, tmp_path / "viewer.xml")
    simulation = JakaMujocoSimulation(config, mjcf_path=model)
    robot_reference = simulation.capture_reference()
    recorded_reference = recorded[0]
    joints = []
    for start, end in zip(recorded, recorded[1:]):
        for sample_index in range(1, 51):
            fraction = sample_index / 50.0
            position = tuple(
                float(value)
                for value in (
                    (1.0 - fraction) * np.asarray(start.position_m)
                    + fraction * np.asarray(end.position_m)
                )
            )
            orientation = quaternion_slerp_xyzw(
                start.orientation_xyzw, end.orientation_xyzw, fraction
            )
            relative = relative_pose(recorded_reference, Pose6D(position, orientation))
            result = simulation.evaluate(compose_pose(robot_reference, relative), dt_s=1.0 / 60.0)
            assert result.accepted, result.reason
            assert result.joint_target_rad is not None
            joints.append(result.joint_target_rad)

    trajectory = np.asarray(joints)
    spans = np.ptp(trajectory, axis=0)
    net = trajectory[-1] - trajectory[0]
    assert spans[3] < 1.6
    assert spans[5] < 1.6
    assert abs(net[3]) < 0.75
    assert abs(net[5]) < 0.75
    assert max(metric.jacobian_condition for metric in simulation.accepted_metrics) < 60.0
    assert min(
        metric.minimum_jacobian_singular_value
        for metric in simulation.accepted_metrics
    ) > 0.0125


def _hand_payload(sequence: int, x: float = 0.0) -> bytes:
    landmarks = ",".join("0" for _ in range(63))
    return (
        f"Right wrist | f = {sequence} | t = {sequence}:, {x},0,0,0,0,0,1\n"
        f"Right landmarks | f = {sequence} | t = {sequence}:, {landmarks}"
    ).encode()


def _head_payload(sequence: int) -> bytes:
    return f"Head pose | f = {sequence} | t = {sequence}:, 0,0,0,0,0,0,1".encode()


def _run_short_replay(tmp_path: Path) -> dict[str, object]:
    base = ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_offline.yaml")
    config = replace(base, engagement_schedule_s=(0.0,))
    model = build_viewer_mjcf(config.mjcf_path, tmp_path / "viewer.xml")
    simulation = JakaMujocoSimulation(config, mjcf_path=model)
    session = QuestJakaReplaySession(config, simulation)
    datagrams = [
        ReceivedHtsDatagram(_hand_payload(1), "192.0.2.10", 1, 0, 0),
        ReceivedHtsDatagram(_hand_payload(2), "192.0.2.10", 1, 50_000_000, 1),
        ReceivedHtsDatagram(_hand_payload(3, 0.01), "192.0.2.10", 1, 100_000_000, 2),
        ReceivedHtsDatagram(_head_payload(1), "192.0.2.10", 2, 400_000_000, 3),
        ReceivedHtsDatagram(_hand_payload(4, 0.02), "192.0.2.10", 1, 500_000_000, 4),
    ]
    previous = 0
    for datagram in datagrams:
        simulation.step((datagram.receive_monotonic_ns - previous) / 1e9)
        previous = datagram.receive_monotonic_ns
        session.process(datagram)
    return session.report(replay_source=str(tmp_path / "synthetic.hts.jsonl"))


def test_deterministic_replay_stale_disengagement_and_no_recovery(tmp_path: Path) -> None:
    first = _run_short_replay(tmp_path / "a")
    second = _run_short_replay(tmp_path / "b")
    keys = (
        "frame_count",
        "accepted_target_count",
        "rejection_counts_by_reason",
        "ik_successes",
        "final_state",
    )
    assert {key: first[key] for key in keys} == {key: second[key] for key in keys}
    assert first["accepted_target_count"] > 0
    assert first["ik_successes"] > 0
    assert first["final_state"] == "disengaged"
    transitions = first["engagement_transitions"]
    assert any(row["reason"] == "right_hand_stale" for row in transitions)
    assert transitions[-1]["current"] == "disengaged"


def test_generated_viewer_excludes_only_allowed_adjacent_stiction_pair(tmp_path: Path) -> None:
    config = ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml")
    generated = build_viewer_mjcf(config.mjcf_path, tmp_path / "viewer.xml")
    text = generated.read_text(encoding="utf-8")
    assert 'body1="jaka_Link_0" body2="jaka_Link_1"' in text
    simulation = JakaMujocoSimulation(config, mjcf_path=generated)
    assert simulation.data.ncon == 0
    simulation.set_hand_actuator_target(
        {
            "thumb_lateral": 0.1,
            "thumb_close": 0.1,
            "index": 0.2,
            "middle": 0.2,
            "ring": 0.2,
            "pinky": 0.2,
        }
    )
    simulation.step(0.02)
    assert simulation.data.ctrl[simulation.hand_actuator_ids].max() > 0.0


def test_offline_entrypoint_has_no_hardware_backend_imports() -> None:
    root = Path(__file__).parents[1]
    files = [root / "tools/quest_jaka_mujoco_sim.py", *sorted((root / "src/quest_jaka_sim").glob("*.py"))]
    imported: set[str] = set()
    for path in files:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module)
    forbidden = (
        "jaka_driver_adapter.servo_jog",
        "teleoperation.jaka",
        "rh56_driver",
        "robot_bringup",
        "jkrc",
        "rospy",
        "rclpy",
    )
    assert not any(name.startswith(forbidden) for name in imported)
