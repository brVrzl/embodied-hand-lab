from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import subprocess

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
    SmoothQuestJakaSession,
)
from quest_jaka_sim.output import AcceptedArmTarget
from quest_jaka_sim.se3 import quaternion_angle_rad, rotvec_to_quaternion_xyzw
from quest_jaka_sim.simulation import build_viewer_mjcf
from quest_jaka_sim.simulation import CandidateMetrics, FeasibilityReason, FeasibilityResult
from teleoperation.jaka.quest_adapter import JakaAcceptedJointTargetAdapter
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
    hardware_shadow = JakaMujocoSimulation(config, mjcf_path=hw_model)
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
        assert (left.accepted_target is None) == (right.accepted_target is None)
        if left.accepted_target and right.accepted_target:
            assert left.accepted_target.joint_position_rad == pytest.approx(
                right.accepted_target.joint_position_rad, abs=1e-12
            )
        results.append((left, right))
    assert len(sim_targets.targets) == len(hardware_targets.targets) > 0
    for left, right in zip(sim_targets.targets, hardware_targets.targets, strict=True):
        assert left.desired_tcp.position_m == pytest.approx(right.desired_tcp.position_m, abs=1e-12)
        assert quaternion_angle_rad(left.desired_tcp.orientation_xyzw, right.desired_tcp.orientation_xyzw) <= 1e-12
        assert left.joint_position_rad == pytest.approx(right.joint_position_rad, abs=1e-12)


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
        sequence,
        42,
        1_000_000,
        2_000_000,
        pose,
        pose,
        (0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
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


def test_p4_entry_is_blocked_before_connection_until_physical_mapping_confirmation(tmp_path: Path) -> None:
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
    assert "P4 blocked" in result.stderr
    assert not (tmp_path / "metrics.json").exists()
