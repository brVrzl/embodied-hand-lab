from __future__ import annotations

from dataclasses import replace
import json
import math
from pathlib import Path
import subprocess
import threading
import time

import numpy as np
import pytest

from motion_input import Pose6D, ReceivedHtsDatagram
from quest_jaka_sim import (
    AnalogClutchSample,
    CompositeArmTargetAdapter,
    JakaMujocoSimulation,
    LatchedHeadYawArmMapper,
    MujocoArmTargetAdapter,
    RecordingArmTargetAdapter,
    ReplayConfig,
    Se3FilterProfile,
    SharedJakaTargetGenerator,
    SmoothQuestJakaSession,
)
from quest_jaka_sim.output import AcceptedArmTarget, AcceptedTcpPose
from teleoperation.accepted_target import AcceptedTargetDiagnostics
from quest_jaka_sim.se3 import quaternion_angle_rad, rotvec_to_quaternion_xyzw
from quest_jaka_sim.simulation import build_viewer_mjcf
from quest_jaka_sim.simulation import CandidateMetrics, FeasibilityReason, FeasibilityResult
from teleoperation.jaka.quest_adapter import (
    E2IsolatedForwardTranslationGuard,
    JakaAcceptedJointTargetAdapter,
)
from teleoperation.wire import TargetFlags, TargetKind


CONFIG = Path("configs/sim/quest_hts_jaka_mini2_live_demo.yaml")


def _hand(sequence: int, timestamp_ns: int, pose: Pose6D) -> ReceivedHtsDatagram:
    landmarks = ",".join("0" for _ in range(63))
    p = pose.position_m
    q = pose.orientation_xyzw
    payload = (
        f"Right wrist | f = {sequence}:, {p[0]},{p[1]},{p[2]},"
        f"{q[0]},{q[1]},{q[2]},{q[3]}\n"
        f"Right landmarks | f = {sequence}:, {landmarks}"
    ).encode()
    return ReceivedHtsDatagram(payload, "10.0.0.2", 9000, timestamp_ns, timestamp_ns)


def _head(sequence: int, timestamp_ns: int, orientation=(0.0, 0.0, 0.0, 1.0)):
    payload = (
        f"Head pose | f = {sequence}:, 0,0,0,"
        f"{orientation[0]},{orientation[1]},{orientation[2]},{orientation[3]}"
    ).encode()
    return ReceivedHtsDatagram(payload, "10.0.0.2", 9001, timestamp_ns, timestamp_ns)


def _clutch(session: SmoothQuestJakaSession, value: float, sequence: int, timestamp_ns: int) -> None:
    session.set_clutch_samples(
        index=AnalogClutchSample(value, timestamp_ns, sequence),
        grip=AnalogClutchSample(0.0, timestamp_ns, sequence),
        left_controller_valid=True,
        provider="shared_pipeline_test",
    )


def _sessions(tmp_path: Path):
    config = replace(ReplayConfig.load(CONFIG), engagement_schedule_s=())
    sim_model = build_viewer_mjcf(config.mjcf_path, tmp_path / "sim.xml")
    hw_model = build_viewer_mjcf(config.mjcf_path, tmp_path / "hardware.xml")
    sim = JakaMujocoSimulation(config, mjcf_path=sim_model)
    hardware_shadow = SharedJakaTargetGenerator(config, mjcf_path=hw_model)
    sim_record = RecordingArmTargetAdapter()
    hw_record = RecordingArmTargetAdapter()
    sim_session = SmoothQuestJakaSession(
        config,
        sim,
        arm_output=CompositeArmTargetAdapter((MujocoArmTargetAdapter(sim), sim_record)),
    )
    hardware_session = SmoothQuestJakaSession(
        config, hardware_shadow, arm_output=hw_record
    )
    return config, sim_session, hardware_session, sim_record, hw_record


def test_one_shared_config_defines_target_and_jaka_transport_contract() -> None:
    config = ReplayConfig.load(CONFIG)
    rates = config.raw["rates"]
    hardware = config.raw["hardware_adapter"]
    assert config.mapping.translation_scale_per_axis == (1.0, 1.0, 1.0)
    assert config.mapping.orientation_scale == 1.0
    assert config.mapping.orientation_scale_per_axis == (1.0, 1.0, 1.0)
    assert rates["target_generation_hz"] == rates["ik_hz"] == 60
    assert rates["jaka_transport_hz"] == 125
    assert hardware["servo_period_ms"] == pytest.approx(8.0)
    assert hardware["joint_order"] == [f"jaka_joint_{index}" for index in range(1, 7)]
    assert hardware["joint_angle_unit"] == "rad"
    assert hardware["command_mode"] == "edg_servo_j_absolute"
    assert hardware["expected_tool_id"] == hardware["expected_user_frame_id"] == 0
    assert config.raw["shared_target_generation"] == {
        "continuation_enabled": True,
        "maximum_backtracks": 5,
        "minimum_continuation_fraction": 0.03125,
        "rejection_policy": "hold_last_accepted_and_allow_operator_retreat",
    }


def _assert_pose_equal(left: Pose6D | None, right: Pose6D | None) -> None:
    assert (left is None) == (right is None)
    if left is None or right is None:
        return
    assert left.position_m == pytest.approx(right.position_m, abs=1e-12)
    assert quaternion_angle_rad(left.orientation_xyzw, right.orientation_xyzw) <= 1e-12


def test_shared_pipeline_simulation_and_hardware_pre_adapter_parity(tmp_path: Path) -> None:
    _, sim, hardware, sim_targets, hardware_targets = _sessions(tmp_path)
    poses = [
        Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        Pose6D((0.004, 0.0, 0.0), rotvec_to_quaternion_xyzw((0.08, 0.0, 0.0))),
        Pose6D((0.004, 0.003, 0.0), rotvec_to_quaternion_xyzw((0.02, -0.07, 0.0))),
        Pose6D((0.004, 0.003, -0.004), rotvec_to_quaternion_xyzw((0.04, -0.05, 0.06))),
    ]
    results = []
    for index, pose in enumerate(poses, start=1):
        now_ns = index * 20_000_000
        for session in (sim, hardware):
            session.ingest(_hand(index, now_ns, pose))
            if index == 1:
                session.ingest(_head(1, now_ns))
            _clutch(session, 0.0 if index == 1 else 1.0, index, now_ns)
        left = sim.control_tick(now_ns)
        right = hardware.control_tick(now_ns)
        assert left.input_sequence == right.input_sequence
        _assert_pose_equal(left.validated_wrist, right.validated_wrist)
        _assert_pose_equal(left.relative_hand_transform, right.relative_hand_transform)
        _assert_pose_equal(left.tcp_target, right.tcp_target)
        _assert_pose_equal(left.filtered_tcp_target, right.filtered_tcp_target)
        assert left.reason == right.reason
        assert sim.arm_clutch.state == hardware.arm_clutch.state
        assert sim.reference_generation == hardware.reference_generation
        _assert_pose_equal(sim.arm_mapper.hand_reference, hardware.arm_mapper.hand_reference)
        _assert_pose_equal(sim.arm_mapper.robot_reference, hardware.arm_mapper.robot_reference)
        assert (left.accepted_target is None) == (right.accepted_target is None)
        if left.accepted_target and right.accepted_target:
            assert left.accepted_target.sequence_number == right.accepted_target.sequence_number
            assert left.accepted_target.input_sequence_number == right.accepted_target.input_sequence_number
            assert left.accepted_target.source_sequence_number == right.accepted_target.source_sequence_number
            assert left.accepted_target.source_timestamp_ns == right.accepted_target.source_timestamp_ns
            assert left.accepted_target.reference_generation == right.accepted_target.reference_generation
            assert left.accepted_target.clutch_generation == right.accepted_target.clutch_generation
            assert left.accepted_target.diagnostics == right.accepted_target.diagnostics
            assert left.accepted_target.joint_position_rad == pytest.approx(
                right.accepted_target.joint_position_rad, abs=1e-12
            )
            assert left.feasibility is not None and right.feasibility is not None
            assert left.feasibility.reason == right.feasibility.reason
            assert left.feasibility.metrics.ik_candidate_rad == pytest.approx(
                right.feasibility.metrics.ik_candidate_rad, abs=1e-12
            )
        results.append((left, right))
    assert len(sim_targets.targets) == len(hardware_targets.targets) > 0
    for left, right in zip(sim_targets.targets, hardware_targets.targets, strict=True):
        assert left.desired_tcp.position_m == pytest.approx(right.desired_tcp.position_m, abs=1e-12)
        assert quaternion_angle_rad(left.desired_tcp.orientation_xyzw, right.desired_tcp.orientation_xyzw) <= 1e-12
        assert left.joint_position_rad == pytest.approx(right.joint_position_rad, abs=1e-12)


def test_repeated_and_bursty_samples_preserve_exact_pre_adapter_sequence(tmp_path: Path) -> None:
    _, simulation, hardware, simulation_targets, hardware_targets = _sessions(tmp_path)
    identity = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    for session in (simulation, hardware):
        session.ingest(_hand(1, 10_000_000, identity))
        session.ingest(_head(1, 10_000_000))
        _clutch(session, 0.0, 1, 10_000_000)
        session.control_tick(10_000_000)
        session.ingest(_hand(2, 20_000_000, identity))
        _clutch(session, 1.0, 2, 20_000_000)
        session.control_tick(20_000_000)

    burst = [
        Pose6D((0.002, 0.0, 0.0), identity.orientation_xyzw),
        Pose6D((0.004, -0.001, 0.0), identity.orientation_xyzw),
        Pose6D((0.004, -0.001, 0.0), identity.orientation_xyzw),
    ]
    for sequence, pose in enumerate(burst, start=3):
        receive_ns = 20_000_000 + sequence * 1_000_000
        for session in (simulation, hardware):
            session.ingest(_hand(sequence, receive_ns, pose))
            _clutch(session, 1.0, sequence, receive_ns)
    for tick_ns in (40_000_000, 56_666_667, 73_333_334):
        left = simulation.control_tick(tick_ns)
        right = hardware.control_tick(tick_ns)
        assert left.reason == right.reason
        assert left.input_sequence == right.input_sequence
        if left.accepted_target is not None and right.accepted_target is not None:
            assert left.accepted_target == right.accepted_target

    assert simulation_targets.targets == hardware_targets.targets


def test_quaternion_wraparound_uses_same_shortest_arc_for_both_outputs(tmp_path: Path) -> None:
    _, simulation, hardware, _, _ = _sessions(tmp_path)
    q_positive = rotvec_to_quaternion_xyzw((0.0, 0.0, math.radians(179.0)))
    q_negative = rotvec_to_quaternion_xyzw((0.0, 0.0, math.radians(-179.0)))
    for session in (simulation, hardware):
        session.ingest(_hand(1, 20_000_000, Pose6D((0.0, 0.0, 0.0), q_positive)))
        session.ingest(_head(1, 20_000_000))
        _clutch(session, 0.0, 1, 20_000_000)
        session.control_tick(20_000_000)
        session.ingest(_hand(2, 40_000_000, Pose6D((0.0, 0.0, 0.0), q_positive)))
        _clutch(session, 1.0, 2, 40_000_000)
        session.control_tick(40_000_000)
        session.ingest(_hand(3, 60_000_000, Pose6D((0.0, 0.0, 0.0), q_negative)))
        _clutch(session, 1.0, 3, 60_000_000)
    left = simulation.control_tick(60_000_000)
    right = hardware.control_tick(60_000_000)
    _assert_pose_equal(left.relative_hand_transform, right.relative_hand_transform)
    assert left.relative_hand_transform is not None
    assert quaternion_angle_rad(
        left.relative_hand_transform.orientation_xyzw, (0.0, 0.0, 0.0, 1.0)
    ) < math.radians(2.01)
    assert left.accepted_target == right.accepted_target


def test_shared_continuation_bounds_full_pose_for_both_outputs(
    tmp_path: Path,
) -> None:
    config = replace(ReplayConfig.load(CONFIG), engagement_schedule_s=())
    sim_model = build_viewer_mjcf(config.mjcf_path, tmp_path / "recovery.xml")
    simulation = JakaMujocoSimulation(config, mjcf_path=sim_model)
    session = SmoothQuestJakaSession(config, simulation)
    identity = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    session.ingest(_hand(1, 20_000_000, identity))
    session.ingest(_head(1, 20_000_000))
    _clutch(session, 0.0, 1, 20_000_000)
    session.control_tick(20_000_000)
    session.ingest(_hand(2, 40_000_000, identity))
    _clutch(session, 1.0, 2, 40_000_000)
    captured = session.control_tick(40_000_000)
    assert captured.accepted_target is not None

    abrupt = Pose6D(
        (0.03, 0.01, 0.0),
        rotvec_to_quaternion_xyzw((0.20, -0.10, 0.30)),
    )
    session.ingest(_hand(3, 60_000_000, abrupt))
    _clutch(session, 1.0, 3, 60_000_000)
    session.control_tick(60_000_000)
    session.ingest(_hand(4, 80_000_000, abrupt))
    _clutch(session, 1.0, 4, 80_000_000)
    advanced = session.control_tick(80_000_000)
    event = session.event_records[-1]

    assert advanced.accepted_target is not None
    assert event["continuation_enabled"] is True
    assert 0.0 < event["continuation_fraction"] < 1.0
    assert event["requested_backlog_deg"] > 0.0
    assert quaternion_angle_rad(
        captured.accepted_target.filtered_tcp.orientation_xyzw,
        advanced.accepted_target.filtered_tcp.orientation_xyzw,
    ) <= config.feasibility.maximum_tcp_angular_velocity_rad_s * 0.02 + 1e-9
    _, shared_sim, shared_hardware, _, _ = _sessions(tmp_path / "shared")
    assert shared_sim.continuation_enabled is True
    assert shared_hardware.continuation_enabled is True


def test_shared_rejection_holds_both_outputs_engaged_for_operator_retreat(
    tmp_path: Path,
) -> None:
    config = replace(ReplayConfig.load(CONFIG), engagement_schedule_s=())
    sessions = []
    for name, generator_type in (("simulation", JakaMujocoSimulation), ("hardware", SharedJakaTargetGenerator)):
        model = build_viewer_mjcf(config.mjcf_path, tmp_path / name / "model.xml")
        session = SmoothQuestJakaSession(
            config,
            generator_type(config, mjcf_path=model),
            arm_output=RecordingArmTargetAdapter() if generator_type is SharedJakaTargetGenerator else None,
        )
        identity = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        session.ingest(_hand(1, 20_000_000, identity))
        session.ingest(_head(1, 20_000_000))
        _clutch(session, 0.0, 1, 20_000_000)
        session.control_tick(20_000_000)
        session.ingest(_hand(2, 40_000_000, identity))
        _clutch(session, 1.0, 2, 40_000_000)
        session.control_tick(40_000_000)
        sessions.append(session)

    simulation, hardware = sessions
    for index in range(config.raw["simulation"]["isolated_rejection_hold_count"] + 1):
        simulation._handle_rejection(60_000_000 + index, FeasibilityReason.NEAR_SINGULARITY.value)
        hardware._handle_rejection(60_000_000 + index, FeasibilityReason.NEAR_SINGULARITY.value)

    assert simulation.arm_clutch.state.value == hardware.arm_clutch.state.value == "engaged"
    assert simulation.arm_mapper.robot_reference is not None
    assert hardware.arm_mapper.robot_reference is not None


def test_target_envelope_rejection_is_identical_and_holds_last(tmp_path: Path) -> None:
    config, simulation, hardware, simulation_targets, hardware_targets = _sessions(tmp_path)
    initial_left = simulation.target_generator.last_safe_target
    initial_right = hardware.target_generator.last_safe_target
    _assert_pose_equal(initial_left, initial_right)
    outside = Pose6D(
        (
            initial_left.position_m[0] + config.mapping.maximum_target_displacement_m + 0.01,
            initial_left.position_m[1],
            initial_left.position_m[2],
        ),
        initial_left.orientation_xyzw,
    )
    left = simulation.target_generator.evaluate(outside, dt_s=1.0 / 60.0)
    right = hardware.target_generator.evaluate(outside, dt_s=1.0 / 60.0)
    assert left.reason == right.reason == FeasibilityReason.OUTSIDE_ROBOT_WORKSPACE
    assert not left.accepted and not right.accepted
    _assert_pose_equal(simulation.target_generator.last_safe_target, initial_left)
    _assert_pose_equal(hardware.target_generator.last_safe_target, initial_right)
    assert simulation_targets.targets == hardware_targets.targets == []


def test_reference_clutch_release_reengage_and_latched_head_compensation(tmp_path: Path) -> None:
    _, _, session, _, recorder = _sessions(tmp_path)
    identity = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    session.ingest(_hand(1, 20_000_000, identity))
    session.ingest(_head(1, 20_000_000))
    _clutch(session, 0.0, 1, 20_000_000)
    before_reference = session.control_tick(20_000_000)
    assert before_reference.accepted_target is None
    assert recorder.targets == []

    session.ingest(_hand(2, 40_000_000, identity))
    _clutch(session, 1.0, 2, 40_000_000)
    captured = session.control_tick(40_000_000)
    assert captured.accepted_target is not None
    assert captured.relative_hand_transform is not None
    assert captured.relative_hand_transform.position_m == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    captured_tcp = captured.accepted_target.filtered_tcp

    # Head yaw is sampled only at reference capture. Later head movement with
    # an unchanged wrist must not become hand-relative robot motion.
    session.ingest(_hand(3, 60_000_000, identity))
    session.ingest(_head(2, 60_000_000, rotvec_to_quaternion_xyzw((0.0, 0.8, 0.0))))
    _clutch(session, 1.0, 3, 60_000_000)
    head_moved = session.control_tick(60_000_000)
    assert head_moved.accepted_target is not None
    _assert_pose_equal(head_moved.accepted_target.filtered_tcp, captured_tcp)

    session.ingest(_hand(4, 80_000_000, identity))
    _clutch(session, 0.0, 4, 80_000_000)
    released = session.control_tick(80_000_000)
    target_count = len(recorder.targets)
    assert released.accepted_target is None

    new_wrist = Pose6D((0.05, 0.0, 0.0), identity.orientation_xyzw)
    session.ingest(_hand(5, 100_000_000, new_wrist))
    _clutch(session, 1.0, 5, 100_000_000)
    recaptured = session.control_tick(100_000_000)
    assert recaptured.accepted_target is not None
    assert len(recorder.targets) == target_count + 1
    assert recaptured.relative_hand_transform is not None
    assert recaptured.relative_hand_transform.position_m == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    assert captured.accepted_target.reference_generation == 1
    assert recaptured.accepted_target.reference_generation == 2


def test_tracking_loss_requires_release_then_recaptures_without_jump(tmp_path: Path) -> None:
    _, _, session, _, recorder = _sessions(tmp_path)
    identity = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    for sequence, clutch in ((1, 0.0), (2, 1.0)):
        now_ns = sequence * 20_000_000
        session.ingest(_hand(sequence, now_ns, identity))
        if sequence == 1:
            session.ingest(_head(1, now_ns))
        _clutch(session, clutch, sequence, now_ns)
        session.control_tick(now_ns)
    baseline = len(recorder.targets)
    session.control_tick(400_000_000)
    assert session.arm_clutch.state.value == "tracking_fault"

    session.ingest(_hand(3, 420_000_000, identity))
    session.ingest(_head(2, 420_000_000))
    _clutch(session, 1.0, 3, 420_000_000)
    assert session.control_tick(420_000_000).accepted_target is None
    assert len(recorder.targets) == baseline

    session.ingest(_hand(4, 440_000_000, identity))
    _clutch(session, 0.0, 4, 440_000_000)
    session.control_tick(440_000_000)
    session.ingest(_hand(5, 460_000_000, identity))
    _clutch(session, 1.0, 5, 460_000_000)
    recovered = session.control_tick(460_000_000)
    assert recovered.accepted_target is not None
    assert recovered.accepted_target.reference_generation == 2
    assert recovered.relative_hand_transform is not None
    assert recovered.relative_hand_transform.position_m == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_mujoco_render_or_plant_stall_cannot_block_hardware_target_generation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _, simulation, hardware, _, hardware_targets = _sessions(tmp_path)
    identity = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    for session in (simulation, hardware):
        session.ingest(_hand(1, 20_000_000, identity))
        session.ingest(_head(1, 20_000_000))
        _clutch(session, 0.0, 1, 20_000_000)
        session.control_tick(20_000_000)
        session.ingest(_hand(2, 40_000_000, identity))
        _clutch(session, 1.0, 2, 40_000_000)
        session.control_tick(40_000_000)

    stall_entered = threading.Event()
    release_stall = threading.Event()

    def stalled_step(_dt_s: float) -> None:
        stall_entered.set()
        release_stall.wait(timeout=2.0)

    monkeypatch.setattr(simulation.simulation, "step", stalled_step)
    stalled_thread = threading.Thread(target=simulation.simulation.step, args=(0.1,))
    stalled_thread.start()
    assert stall_entered.wait(timeout=1.0)
    try:
        moved = Pose6D((0.002, 0.0, 0.0), identity.orientation_xyzw)
        hardware.ingest(_hand(3, 60_000_000, moved))
        _clutch(hardware, 1.0, 3, 60_000_000)
        started = time.perf_counter()
        result = hardware.control_tick(60_000_000)
        elapsed = time.perf_counter() - started
        assert result.accepted_target is not None
        assert hardware_targets.targets[-1] == result.accepted_target
        assert elapsed < 0.1
    finally:
        release_stall.set()
        stalled_thread.join(timeout=1.0)
    assert not stalled_thread.is_alive()


def _mapper_from_shared_config() -> LatchedHeadYawArmMapper:
    config = ReplayConfig.load(CONFIG)
    selected = config.raw["filter"]["selected_profile"]
    profile = Se3FilterProfile.from_mapping(
        selected, config.raw["filter"]["profiles"][selected]
    )
    return LatchedHeadYawArmMapper(config.mapping, profile)


def _settled_mapper_target(mapper: LatchedHeadYawArmMapper, wrist: Pose6D) -> Pose6D:
    target = wrist
    for index in range(1, 301):
        target = mapper.target(wrist, timestamp_ns=1 + index * 16_666_667)
    return target


@pytest.mark.parametrize(
    ("quest_delta", "robot_delta"),
    [
        ((0.01, 0.0, 0.0), (-0.01, 0.0, 0.0)),
        ((0.0, 0.01, 0.0), (0.0, 0.0, 0.01)),
        ((0.0, 0.0, 0.01), (0.0, 0.01, 0.0)),
    ],
)
def test_shared_translation_axes_are_one_to_one_with_committed_signs(quest_delta, robot_delta) -> None:
    mapper = _mapper_from_shared_config()
    identity = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    mapper.capture(wrist=identity, robot_tcp=identity, head=identity, timestamp_ns=1)
    target = _settled_mapper_target(mapper, Pose6D(quest_delta, identity.orientation_xyzw))
    assert target.position_m == pytest.approx(robot_delta, abs=1e-5)


@pytest.mark.parametrize(
    "rotvec",
    [
        (0.20, 0.0, 0.0),
        (0.0, -0.20, 0.0),  # downward wrist pitch regression
        (0.0, 0.0, 0.20),
        (0.12, -0.16, 0.08),
    ],
)
def test_shared_full_6d_orientation_including_downward_wrist_rotation(rotvec) -> None:
    mapper = _mapper_from_shared_config()
    identity = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    mapper.capture(wrist=identity, robot_tcp=identity, head=identity, timestamp_ns=1)
    target = _settled_mapper_target(
        mapper,
        Pose6D((0.0, 0.0, 0.0), rotvec_to_quaternion_xyzw(rotvec)),
    )
    basis = np.diag((-1.0, -1.0, 1.0))
    expected = rotvec_to_quaternion_xyzw(basis @ np.asarray(rotvec))
    assert quaternion_angle_rad(target.orientation_xyzw, expected) < 1e-4


class _MockRuntime:
    def __init__(self, sent: bool = True) -> None:
        self.sent = sent
        self.packets = []
        self.stop_sequences = []

    def dispatch_packet(self, packet) -> bool:
        self.packets.append(packet)
        return self.sent

    def dispatch_stop(self, *, sequence: int) -> bool:
        self.stop_sequences.append(sequence)
        return True


def _accepted(sequence: int = 1) -> AcceptedArmTarget:
    pose = Pose6D((0.1, -0.2, 0.3), (0.0, 0.0, 0.0, 1.0))
    return AcceptedArmTarget(
        sequence_number=sequence,
        input_sequence_number=42,
        source_sequence_number=41,
        source_timestamp_ns=900_000,
        input_receive_monotonic_ns=1_000_000,
        generated_monotonic_ns=2_000_000,
        reference_generation=1,
        clutch_generation=1,
        desired_tcp=AcceptedTcpPose(pose.position_m, pose.orientation_xyzw),
        filtered_tcp=AcceptedTcpPose(pose.position_m, pose.orientation_xyzw),
        joint_position_rad=(0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
        diagnostics=AcceptedTargetDiagnostics(
            final_reason="ACCEPTED",
            attempted_reasons=("ACCEPTED",),
            continuation_fraction=1.0,
            continuation_backtracks=0,
            ik_position_error_m=0.0,
            ik_orientation_error_rad=0.0,
            jacobian_condition=1.0,
            minimum_jacobian_singular_value=1.0,
            nearest_safe_joint_limit_margin_rad=1.0,
        ),
    )


def test_hardware_adapter_contract_has_no_conversion_filter_or_interpolation() -> None:
    runtime = _MockRuntime()
    adapter = JakaAcceptedJointTargetAdapter(runtime, allow_motion=True)
    target = _accepted()
    assert adapter.apply(target)
    assert adapter.applied_count == 1
    assert len(runtime.packets) == 1
    packet = runtime.packets[0]
    assert packet.kind is TargetKind.JOINT_POSITION
    assert packet.flags is TargetFlags.ALLOW_MOTION
    assert packet.frame_id.value == 0
    assert packet.payload[:6] == target.joint_position_rad
    assert packet.payload[6:] == (0.0, 0.0)
    assert adapter.stop()
    assert not adapter.apply(_accepted(2))
    assert len(runtime.packets) == 1


def test_e2_guard_forwards_only_unchanged_continuous_negative_x_targets() -> None:
    runtime = _MockRuntime()
    output = JakaAcceptedJointTargetAdapter(runtime, allow_motion=True)
    guard = E2IsolatedForwardTranslationGuard(
        output, startup_alignment_tolerance_rad=0.001
    )
    baseline = _accepted()
    guard.establish_startup_joint_position(baseline.joint_position_rad)
    assert guard.apply(baseline)

    forward_pose = AcceptedTcpPose(
        (0.085, -0.2, 0.3), baseline.desired_tcp.orientation_xyzw
    )
    forward = replace(
        baseline,
        sequence_number=2,
        desired_tcp=forward_pose,
        filtered_tcp=forward_pose,
        joint_position_rad=(0.11, -0.21, 0.31, -0.41, 0.51, -0.61),
    )
    assert guard.apply(forward)
    assert runtime.packets[-1].payload[:6] == forward.joint_position_rad
    assert guard.maximum_requested_tcp_displacement_m == pytest.approx(0.015)
    assert guard.maximum_accepted_joint_displacement_rad == pytest.approx([0.01] * 6)

    lateral_pose = AcceptedTcpPose(
        (0.085, -0.194, 0.3), baseline.desired_tcp.orientation_xyzw
    )
    lateral = replace(
        forward,
        sequence_number=3,
        desired_tcp=lateral_pose,
        filtered_tcp=lateral_pose,
    )
    assert not guard.apply(lateral)
    assert guard.abort_reason == "e2_requested_cross_axis_displacement"
    assert len(runtime.packets) == 2


def test_e2_guard_observes_encoder_noise_without_replacing_startup_state() -> None:
    runtime = _MockRuntime()
    guard = E2IsolatedForwardTranslationGuard(
        JakaAcceptedJointTargetAdapter(runtime, allow_motion=True),
        startup_alignment_tolerance_rad=0.001,
    )
    target = _accepted()
    post_edg = target.joint_position_rad
    guard.establish_startup_joint_position(post_edg)
    noisy = tuple(value + 3.49066e-5 for value in post_edg)
    guard.observe_measured_joint_position(noisy)
    assert guard.startup_joint_position_rad == post_edg
    assert guard.latest_measured_joint_position_rad == noisy
    assert guard.maximum_observed_startup_difference_rad == pytest.approx(
        [3.49066e-5] * 6
    )
    assert runtime.packets == []
    assert guard.apply(target)
    assert runtime.packets[0].payload[:6] == post_edg


def test_e2_guard_uses_approved_startup_alignment_contract() -> None:
    observed_stationary_mismatch_rad = 3.49066e-5
    target = _accepted()

    accepted_runtime = _MockRuntime()
    accepted_guard = E2IsolatedForwardTranslationGuard(
        JakaAcceptedJointTargetAdapter(accepted_runtime, allow_motion=True),
        startup_alignment_tolerance_rad=0.001,
    )
    accepted_guard.establish_startup_joint_position(
        tuple(
            value + observed_stationary_mismatch_rad
            for value in target.joint_position_rad
        )
    )
    assert accepted_guard.apply(target)
    assert accepted_guard.abort_reason is None

    rejected_runtime = _MockRuntime()
    rejected_guard = E2IsolatedForwardTranslationGuard(
        JakaAcceptedJointTargetAdapter(rejected_runtime, allow_motion=True),
        startup_alignment_tolerance_rad=0.001,
    )
    rejected_guard.establish_startup_joint_position(
        tuple(value + 0.0011 for value in target.joint_position_rad)
    )
    assert not rejected_guard.apply(target)
    assert rejected_guard.abort_reason == "e2_first_target_not_continuous_with_post_edg_state"
    assert rejected_runtime.packets == []


def test_e2_startup_baseline_changes_only_through_fresh_handoff() -> None:
    first = _accepted().joint_position_rad
    second = tuple(value + 0.0002 for value in first)
    output = JakaAcceptedJointTargetAdapter(_MockRuntime(), allow_motion=True)
    armed_session = E2IsolatedForwardTranslationGuard(
        output, startup_alignment_tolerance_rad=0.001
    )
    armed_session.establish_startup_joint_position(first)
    armed_session.observe_measured_joint_position(second)
    with pytest.raises(RuntimeError, match="already established"):
        armed_session.establish_startup_joint_position(second)
    assert armed_session.startup_joint_position_rad == first

    fresh_session = E2IsolatedForwardTranslationGuard(
        JakaAcceptedJointTargetAdapter(_MockRuntime(), allow_motion=True),
        startup_alignment_tolerance_rad=0.001,
    )
    fresh_session.establish_startup_joint_position(second)
    assert fresh_session.startup_joint_position_rad == second


def test_e2_entry_synchronizes_shared_seed_only_at_post_edg_handoff() -> None:
    source = Path("tools/quest_jaka_hardware.py").read_text(encoding="utf-8")
    command_loop = source.split("while time.monotonic() - started < args.duration_sec:", 1)[1]
    command_loop = command_loop.split("except KeyboardInterrupt:", 1)[0]
    assert "synchronize_authoritative_arm_joints" not in command_loop
    assert "observe_measured_joint_position(sample)" in command_loop


def test_composite_adapters_receive_the_identical_accepted_target_object() -> None:
    identities: list[int] = []

    class _IdentityAdapter:
        def apply(self, target: AcceptedArmTarget) -> bool:
            identities.append(id(target))
            return True

    target = _accepted()
    adapter = CompositeArmTargetAdapter((_IdentityAdapter(), _IdentityAdapter()))
    assert adapter.apply(target)
    assert identities == [id(target), id(target)]


def test_jaka_adapter_imports_and_operates_when_mujoco_import_is_blocked() -> None:
    code = r'''
import builtins
import sys
sys.path.insert(0, "src")
original_import = builtins.__import__
def blocked_import(name, *args, **kwargs):
    if name == "mujoco" or name.startswith("mujoco."):
        raise AssertionError("JAKA adapter imported MuJoCo")
    return original_import(name, *args, **kwargs)
builtins.__import__ = blocked_import
from teleoperation.jaka.quest_adapter import JakaAcceptedJointTargetAdapter
assert JakaAcceptedJointTargetAdapter.__module__ == "teleoperation.jaka.quest_adapter"
'''
    result = subprocess.run(
        [".venv/bin/python", "-c", code], text=True, capture_output=True
    )
    assert result.returncode == 0, result.stderr


def test_physical_entry_has_no_mujoco_plant_adapter_or_simulation_step() -> None:
    source = Path("tools/quest_jaka_hardware.py").read_text(encoding="utf-8")
    simulation_source = Path("tools/quest_jaka_mujoco_sim.py").read_text(encoding="utf-8")
    assert "SharedJakaTargetGenerator" in source
    assert "JakaMujocoSimulation" not in source
    assert "MujocoArmTargetAdapter" not in source
    assert "build_viewer_mjcf" not in source
    assert ".step(" not in source
    assert "QuestDatagramReceiverWorker" in source
    assert "QuestDatagramReceiverWorker" in simulation_source
    assert "class _ReceiveWorker" not in simulation_source
    assert not hasattr(SharedJakaTargetGenerator, "step")
    assert not hasattr(SharedJakaTargetGenerator, "set_accepted_arm_joint_target")
    assert not hasattr(SharedJakaTargetGenerator, "set_hand_actuator_target")
    assert "native.process.poll()" in source
    assert "accepted_target_transport_failure" in source
    assert 'return 2 if abort_reason is not None or e2_failures else 0' in source


def test_invalid_or_communication_failed_target_does_not_count_as_applied() -> None:
    failed_runtime = _MockRuntime(sent=False)
    adapter = JakaAcceptedJointTargetAdapter(failed_runtime, allow_motion=False)
    assert not adapter.apply(_accepted())
    assert adapter.applied_count == 0
    with pytest.raises(ValueError, match="finite"):
        replace(_accepted(), joint_position_rad=(math.nan, 0.0, 0.0, 0.0, 0.0, 0.0))


def test_nan_tracking_dropout_stale_input_and_ik_rejection_emit_no_new_target(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _, _, session, _, recorder = _sessions(tmp_path)
    identity = Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    for sequence, value in ((1, 0.0), (2, 1.0)):
        now_ns = sequence * 20_000_000
        session.ingest(_hand(sequence, now_ns, identity))
        if sequence == 1:
            session.ingest(_head(1, now_ns))
        _clutch(session, value, sequence, now_ns)
        session.control_tick(now_ns)
    baseline = len(recorder.targets)
    assert baseline == 1

    nan_payload = (
        "Right wrist | f = 3:, nan,0,0,0,0,0,1\n"
        + "Right landmarks | f = 3:, "
        + ",".join("0" for _ in range(63))
    ).encode()
    assert not session.ingest(
        ReceivedHtsDatagram(nan_payload, "10.0.0.2", 9000, 60_000_000, 60_000_000)
    )
    session.control_tick(60_000_000)
    assert len(recorder.targets) == baseline

    # A fresh released sample is required after the malformed-data fault.
    session.control_tick(400_000_000)
    assert len(recorder.targets) == baseline

    # Recreate an engaged session and force an IK rejection; hold-last means no
    # adapter call and no new authoritative joint target.
    _, _, rejected_session, _, rejected_recorder = _sessions(tmp_path / "ik")
    for sequence, value in ((1, 0.0), (2, 1.0)):
        now_ns = sequence * 20_000_000
        rejected_session.ingest(_hand(sequence, now_ns, identity))
        if sequence == 1:
            rejected_session.ingest(_head(1, now_ns))
        _clutch(rejected_session, value, sequence, now_ns)
        rejected_session.control_tick(now_ns)
    rejected_baseline = len(rejected_recorder.targets)
    monkeypatch.setattr(
        rejected_session.simulation,
        "evaluate",
        lambda *_args, **_kwargs: FeasibilityResult(
            False,
            FeasibilityReason.IK_POSITION_FAILED,
            None,
            CandidateMetrics(ik_error_m=1.0),
        ),
    )
    now_ns = 60_000_000
    rejected_session.ingest(
        _hand(3, now_ns, Pose6D((0.002, 0.0, 0.0), identity.orientation_xyzw))
    )
    _clutch(rejected_session, 1.0, 3, now_ns)
    result = rejected_session.control_tick(now_ns)
    assert result.reason == FeasibilityReason.IK_POSITION_FAILED.value
    assert result.accepted_target is None
    assert len(rejected_recorder.targets) == rejected_baseline


def test_p4_entry_requires_exact_current_authorization_before_connection(tmp_path: Path) -> None:
    result = subprocess.run(
        [
            ".venv/bin/python",
            "tools/quest_jaka_hardware.py",
            "p4-live",
            "--robot-ip", "192.0.2.1",
            "--approval", "I_AUTHORIZE_P4_LIVE_QUEST_JAKA_MOTION",
            "--estop-accessible",
            "--workspace-clear",
            "--rh56-command-path-absent",
            "--log", str(tmp_path / "log.jsonl"),
            "--summary", str(tmp_path / "summary.json"),
            "--metrics", str(tmp_path / "metrics.json"),
            "--capture", str(tmp_path / "capture.jsonl"),
        ],
        text=True,
        capture_output=True,
    )
    assert result.returncode != 0
    assert "I_AUTHORIZE_P4_LIVE_QUEST_JAKA_TELEOPERATION" in result.stderr
    assert not (tmp_path / "metrics.json").exists()


def test_p4_entry_requires_operator_safety_gates_before_connection(tmp_path: Path) -> None:
    result = subprocess.run(
        [
            ".venv/bin/python",
            "tools/quest_jaka_hardware.py",
            "p4-live",
            "--robot-ip", "192.0.2.1",
            "--approval", "I_AUTHORIZE_P4_LIVE_QUEST_JAKA_TELEOPERATION",
            "--log", str(tmp_path / "log.jsonl"),
            "--summary", str(tmp_path / "summary.json"),
            "--metrics", str(tmp_path / "metrics.json"),
            "--capture", str(tmp_path / "capture.jsonl"),
        ],
        text=True,
        capture_output=True,
    )
    assert result.returncode != 0
    assert "E-stop, clear-workspace, and no-RH56-command confirmations" in result.stderr
    assert not (tmp_path / "metrics.json").exists()


def test_offline_model_parity_report_separates_target_kinematic_and_dynamic_error(
    tmp_path: Path,
) -> None:
    metrics = tmp_path / "p1.json"
    output = tmp_path / "model-parity.json"
    metrics.write_text(
        json.dumps(
            {
                "initial_joint_position_rad": [-1.5707963268, -0.6108652382, -1.5707963268, 0.1745329252, 0.6108652382, -0.2617993878],
                "startup_tcp_mm_rpy_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            }
        ),
        encoding="utf-8",
    )
    result = subprocess.run(
        [
            ".venv/bin/python",
            "tools/quest_jaka_model_parity.py",
            "--worker-metrics", str(metrics),
            "--output", str(output),
        ],
        text=True,
        capture_output=True,
    )
    assert result.returncode == 0, result.stderr
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["physical_commands_sent"] is False
    assert report["kinematic_model_parity"]["shared_vs_mujoco_fk"]["position_error_mm"] < 1e-9
    # Matrix-to-quaternion conversion differs only at arccos roundoff scale.
    assert report["kinematic_model_parity"]["shared_vs_mujoco_fk"]["orientation_error_deg"] < 2e-6
    assert report["target_parity"].startswith("not evaluated")
    assert report["dynamic_tracking_parity"].startswith("not evaluated")
