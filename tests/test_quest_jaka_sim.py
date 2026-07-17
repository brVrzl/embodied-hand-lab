from __future__ import annotations

import ast
from dataclasses import replace
import math
from pathlib import Path

import pytest

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
from quest_jaka_sim.simulation import CandidateMetrics, build_viewer_mjcf, classify_candidate
from motion_input import Pose6D


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
    )


@pytest.mark.parametrize(
    ("metrics", "expected"),
    [
        (CandidateMetrics(target_displacement_m=0.02), FeasibilityReason.OUTSIDE_ROBOT_WORKSPACE),
        (CandidateMetrics(target_jump_m=0.005), FeasibilityReason.TARGET_JUMP),
        (CandidateMetrics(tcp_velocity_m_s=0.03), FeasibilityReason.VELOCITY_LIMIT),
        (CandidateMetrics(maximum_joint_acceleration_rad_s2=21.0), FeasibilityReason.ACCELERATION_LIMIT),
        (CandidateMetrics(ik_error_m=0.003), FeasibilityReason.IK_FAILED),
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
        ReceivedHtsDatagram(_hand_payload(1), "10.24.0.78", 1, 0, 0),
        ReceivedHtsDatagram(_hand_payload(2), "10.24.0.78", 1, 50_000_000, 1),
        ReceivedHtsDatagram(_hand_payload(3, 0.01), "10.24.0.78", 1, 100_000_000, 2),
        ReceivedHtsDatagram(_head_payload(1), "10.24.0.78", 2, 400_000_000, 3),
        ReceivedHtsDatagram(_hand_payload(4, 0.02), "10.24.0.78", 1, 500_000_000, 4),
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
