from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import subprocess

import mujoco
import numpy as np
import pytest

from quest_jaka_sim import (
    AcceptedArmTarget,
    AcceptedTargetDiagnostics,
    AcceptedTcpPose,
    ArmOutputMode,
    JakaEquivalent125HzMujocoAdapter,
    JakaMujocoSimulation,
    ReplayConfig,
)
from quest_jaka_sim.simulation import build_viewer_mjcf
from tools.quest_jaka_mujoco_sim import _parser, _step_smooth_simulation


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module", autouse=True)
def build_production_resampler() -> None:
    subprocess.run(
        [
            "cmake",
            "-S",
            str(ROOT / "native/jaka_servo_worker"),
            "-B",
            str(ROOT / "build/jaka_servo_worker"),
            "-DCMAKE_BUILD_TYPE=Release",
        ],
        check=True,
    )
    subprocess.run(
        ["cmake", "--build", str(ROOT / "build/jaka_servo_worker"), "-j2"],
        check=True,
    )


def _simulation(tmp_path: Path) -> JakaMujocoSimulation:
    config = replace(
        ReplayConfig.load("configs/sim/quest_hts_jaka_mini2_live_demo.yaml"),
        engagement_schedule_s=(),
    )
    model = build_viewer_mjcf(
        config.mjcf_path,
        tmp_path / "output-mode.xml",
    )
    return JakaMujocoSimulation(config, mjcf_path=model)


def _target(
    simulation: JakaMujocoSimulation,
    q: tuple[float, ...],
    *,
    sequence: int = 1,
    generated_ns: int = 1_000_000_000,
    clutch_generation: int = 1,
) -> AcceptedArmTarget:
    tcp = simulation.last_safe_target
    pose = AcceptedTcpPose(tcp.position_m, tcp.orientation_xyzw)
    return AcceptedArmTarget(
        sequence_number=sequence,
        input_sequence_number=sequence,
        source_sequence_number=sequence,
        source_timestamp_ns=900_000_000,
        input_receive_monotonic_ns=950_000_000,
        generated_monotonic_ns=generated_ns,
        reference_generation=1,
        clutch_generation=clutch_generation,
        desired_tcp=pose,
        filtered_tcp=pose,
        joint_position_rad=q,
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


def test_cli_defaults_to_existing_shaped_output_for_live_and_replay() -> None:
    parser = _parser()
    live = parser.parse_args(["live-6dof"])
    shaped = parser.parse_args(["replay-6dof", "recording.hts.jsonl"])
    equivalent = parser.parse_args(
        [
            "replay-6dof",
            "recording.hts.jsonl",
            "--arm-output-mode",
            ArmOutputMode.JAKA_EQUIVALENT_125HZ.value,
        ]
    )
    assert live.arm_output_mode == ArmOutputMode.SHAPED_500HZ.value
    assert shaped.arm_output_mode == ArmOutputMode.SHAPED_500HZ.value
    assert equivalent.recording == shaped.recording


def test_production_125hz_adapter_updates_arm_on_8ms_deadlines_and_bypasses_shaper(
    tmp_path: Path,
) -> None:
    simulation = _simulation(tmp_path)
    adapter = JakaEquivalent125HzMujocoAdapter(simulation)
    session = type("Session", (), {"arm_output": adapter})()
    initial = tuple(float(value) for value in simulation.arm_joints_rad)
    destination = tuple(value + 0.0005 for value in initial)
    adapter.apply(_target(simulation, destination))

    arm_ctrl = []
    for _ in range(8):
        adapter.advance_to(float(simulation.data.time))
        arm_ctrl.append(simulation.data.ctrl[simulation.arm_actuator_ids].copy())
        simulation.step(float(simulation.model.opt.timestep))

    assert np.asarray([row["emitted_monotonic_ns"] for row in adapter.records[1:]]) - np.asarray(
        [row["emitted_monotonic_ns"] for row in adapter.records[:-1]]
    ) == pytest.approx(8_000_000)
    assert arm_ctrl[0] == pytest.approx(initial)
    assert arm_ctrl[1] == pytest.approx(initial)
    assert arm_ctrl[2] == pytest.approx(initial)
    assert arm_ctrl[3] == pytest.approx(initial)
    assert arm_ctrl[4] != pytest.approx(initial)
    assert adapter.records[-1]["transition_limited"]
    assert max(abs(value) for value in adapter.records[-1]["ddq_emit_rad_s2"]) <= (
        simulation.config.output_contract.maximum_acceleration_rad_s2 + 1e-9
    )
    assert max(abs(value) for value in adapter.records[-1]["jerk_emit_rad_s3"]) <= (
        simulation.config.command_limits.maximum_jerk_rad_s3 + 1e-9
    )
    assert simulation.command_velocity_limit_hits.tolist() == [0] * 6
    assert simulation.command_acceleration_limit_hits.tolist() == [0] * 6
    assert simulation.command_jerk_limit_hits.tolist() == [0] * 6

    simulation.set_hand_actuator_target(
        dict(zip(
            ("thumb_lateral", "thumb_close", "index", "middle", "ring", "pinky"),
            (0.1,) * 6,
            strict=True,
        ))
    )
    before = simulation.data.ctrl[simulation.hand_actuator_ids].copy()
    _step_smooth_simulation(simulation, session, 0.008)
    assert np.any(simulation.data.ctrl[simulation.hand_actuator_ids] > before)
    adapter.close()


def test_125hz_adapter_bounds_records_without_losing_cumulative_diagnostics(
    tmp_path: Path,
) -> None:
    simulation = _simulation(tmp_path)
    adapter = JakaEquivalent125HzMujocoAdapter(
        simulation,
        record_capacity=3,
    )
    initial = simulation.arm_joints_rad.copy()
    all_rows: list[dict[str, object]] = []
    accepted_timestamps: dict[int, int] = {}

    for sequence in range(1, 7):
        generated_ns = 1_000_000_000 + (sequence - 1) * 8_000_000
        destination = initial.copy()
        destination[0] += sequence * 0.0001
        accepted_timestamps[sequence] = generated_ns
        adapter.apply(
            _target(
                simulation,
                tuple(destination),
                sequence=sequence,
                generated_ns=generated_ns,
            )
        )
        adapter.advance_to((sequence - 1) * 0.008)
        all_rows.append(dict(adapter.records[-1]))

    retained = list(adapter.records)
    assert not hasattr(adapter, "_accepted")
    assert len(adapter.records) == 6
    assert len(retained) == 3
    assert [row["emitted_sequence"] for row in retained] == [4, 5, 6]
    assert [row["emitted_sequence"] for row in adapter.records[3:]] == [4, 5, 6]
    with pytest.raises(IndexError, match="evicted"):
        _ = adapter.records[0]
    for row in all_rows:
        source_sequence = int(row["source_accepted_sequence"])
        assert row["source_accepted_timestamp_ns"] == accepted_timestamps[
            source_sequence
        ]

    report = adapter.report()
    assert report["arm_emitted_count"] == 6
    assert report["arm_emitted_record_capacity"] == 3
    assert report["arm_emitted_record_retained_count"] == 3
    assert report["arm_emitted_record_dropped_count"] == 3
    assert report["maximum_emitted_joint_velocity_rad_s"] == pytest.approx(
        max(
            max(abs(value) for value in row["dq_emit_rad_s"])
            for row in all_rows
        )
    )
    assert report["maximum_emitted_joint_acceleration_rad_s2"] == pytest.approx(
        max(
            max(abs(value) for value in row["ddq_emit_rad_s2"])
            for row in all_rows
        )
    )
    assert report["transition_limited_emitted_count"] == sum(
        bool(row["transition_limited"]) for row in all_rows
    )
    adapter.close()


def test_125hz_starts_from_nonzero_q_without_historical_deadline_catchup(
    tmp_path: Path,
) -> None:
    simulation = _simulation(tmp_path)
    initial = simulation.arm_joints_rad.copy()
    adapter = JakaEquivalent125HzMujocoAdapter(simulation)

    adapter.advance_to(1.0)

    assert len(adapter.records) == 1
    assert adapter.records[0]["emitted_simulation_time_s"] == pytest.approx(1.0)
    assert adapter.records[0]["q_emit_rad"] == pytest.approx(initial)
    adapter.close()


def test_125hz_stationary_engage_and_reset_do_not_reuse_an_old_destination(
    tmp_path: Path,
) -> None:
    simulation = _simulation(tmp_path)
    initial = simulation.arm_joints_rad.copy()
    adapter = JakaEquivalent125HzMujocoAdapter(simulation)
    adapter.apply(_target(simulation, tuple(initial)))
    for tick in range(10):
        adapter.advance_to(tick * 0.008)
    assert np.asarray([row["q_emit_rad"] for row in adapter.records]) == pytest.approx(
        np.tile(initial, (10, 1))
    )

    moving = tuple(initial + 0.02)
    adapter.apply(
        _target(
            simulation,
            moving,
            sequence=2,
            generated_ns=1_016_000_000,
        )
    )
    for tick in range(10, 16):
        adapter.advance_to(tick * 0.008)
    reset_q = simulation.arm_joints_rad.copy()
    adapter.apply(
        _target(
            simulation,
            tuple(reset_q),
            sequence=3,
            generated_ns=1_048_000_000,
            clutch_generation=2,
        )
    )
    start = len(adapter.records)
    for tick in range(16, 22):
        adapter.advance_to(tick * 0.008)

    assert np.asarray(
        [row["q_emit_rad"] for row in adapter.records[start:]]
    ) == pytest.approx(np.tile(reset_q, (6, 1)))
    adapter.close()


def test_production_transition_tracks_small_latest_destinations_without_runaway(
    tmp_path: Path,
) -> None:
    simulation = _simulation(tmp_path)
    adapter = JakaEquivalent125HzMujocoAdapter(simulation)
    initial = simulation.arm_joints_rad.copy()
    accepted = []
    for sequence in range(1, 53):
        fraction = min(sequence / 52.0, 1.0)
        destination = initial.copy()
        destination[1] -= 0.08 * fraction
        destination[2] += 0.10 * fraction
        accepted.append(destination.copy())
        adapter.apply(
            _target(
                simulation,
                tuple(destination),
                sequence=sequence,
                generated_ns=1_000_000_000 + (sequence - 1) * 16_000_000,
            )
        )
        adapter.advance_to((sequence - 1) * 0.016)
        adapter.advance_to((sequence - 1) * 0.016 + 0.008)
    for tick in range(104, 250):
        adapter.advance_to(tick * 0.008)

    emitted = np.asarray([row["q_emit_rad"] for row in adapter.records])
    lower = np.minimum(initial, np.min(np.asarray(accepted), axis=0)) - 1e-9
    upper = np.maximum(initial, np.max(np.asarray(accepted), axis=0)) + 1e-9
    assert np.all(emitted >= lower)
    assert np.all(emitted <= upper)
    assert emitted[-1] == pytest.approx(accepted[-1], abs=1e-3)
    adapter.close()
