from __future__ import annotations

from pathlib import Path
import json
import subprocess

from motion_input import (
    AnalogClutchSample,
    ArmClutchMachine,
    ClutchAction,
    HtsCanonicalAssembler,
    parse_hts_datagram,
)
from motion_input.hts_transport import ReceivedHtsDatagram
from tools.quest_jaka_hardware import _apply_runtime_config, _parser
from quest_jaka_sim.live_input import QuestDatagramReceiverWorker
from rh56_driver.telemetry import AsyncJsonlRecorder, BoundedJsonlRecorder


def _datagram(payload: bytes, timestamp_ns: int) -> ReceivedHtsDatagram:
    return ReceivedHtsDatagram(
        payload=payload,
        source_address="127.0.0.1",
        source_port=9000,
        receive_monotonic_ns=timestamp_ns,
        receive_unix_ns=timestamp_ns,
    )


def _hand_payload(side: str, sequence: int) -> bytes:
    points = ",".join("0" for _ in range(63))
    return (
        f"{side} wrist | f = {sequence}:,0,0,0,0,0,0,1\n"
        f"{side} landmarks | f = {sequence}:,{points}"
    ).encode()


def _head_payload(sequence: int) -> bytes:
    return f"Head pose | f = {sequence}:,0,0,0,0,0,0,1".encode()


def _ctrl_payload(sequence: int, index: float) -> bytes:
    return (
        f"CTRL,v=1,session=1,seq={sequence},t_ns={sequence},"
        f"connected=1,active=1,tracked=1,index={index:.6f},grip=0.000000\n"
    ).encode()


def _store_hts_sequence(
    receiver: QuestDatagramReceiverWorker,
    *datagrams: ReceivedHtsDatagram,
) -> list[ReceivedHtsDatagram]:
    for datagram in datagrams:
        receiver._store_latest_hts(datagram)
    return receiver.drain()


def _runtime_yaml(path: Path, profile: str, log_dir: Path) -> None:
    path.write_text(
        "\n".join(
            (
                "collection_profile: " + profile,
                "runtime:",
                "  config: configs/sim/quest_hts_jaka_mini2_live_demo.yaml",
                "  worker: build/jaka_servo_worker/jaka_servo_worker",
                "  robot_ip: 192.0.2.1",
                "  edg_state_ip: 192.0.2.2",
                "  bind: 127.0.0.1",
                "  port: 9000",
                "  duration_sec: 1",
                "  rh56_device: /dev/serial/by-id/test",
                "  rh56_config: configs/hand/rh56_pc_direct_teleop.yaml",
                "  native_control_cpu: 0",
                "  native_control_realtime_priority: 10",
                "  episode_data_config: configs/data_collection/physical_collection.yaml",
                "  episode_root: " + str(log_dir / "episodes"),
                "  task_name: test",
                "  operator: test",
                "  run_output_joint_velocity_limits_rad_s: [1, 1, 1, 1, 1, 1]",
                "  log_dir: " + str(log_dir),
            )
        )
        + "\n",
        encoding="utf-8",
    )


def test_collection_profile_selects_diagnostic_outputs_without_hardware(tmp_path: Path) -> None:
    production_config = tmp_path / "production.yaml"
    _runtime_yaml(production_config, "production", tmp_path / "production_logs")
    production = _parser().parse_args(
        ["combined-normal-teleop", "--runtime-config", str(production_config)]
    )
    _apply_runtime_config(production)
    assert production.native_telemetry is None
    assert production.event_extract is None
    assert production.rh56_log is None
    assert production.native_status_every_cycles == 4

    diagnostic_config = tmp_path / "diagnostic.yaml"
    _runtime_yaml(diagnostic_config, "diagnostic", tmp_path / "diagnostic_logs")
    diagnostic = _parser().parse_args(
        ["combined-normal-teleop", "--runtime-config", str(diagnostic_config)]
    )
    _apply_runtime_config(diagnostic)
    assert diagnostic.native_telemetry is not None
    assert diagnostic.event_extract is not None
    assert diagnostic.rh56_log is not None
    assert diagnostic.native_status_every_cycles == 13


def test_production_wiring_publishes_compact_snapshots_above_canonical_rate() -> None:
    source = Path("tools/quest_jaka_hardware.py").read_text(encoding="utf-8")
    assert "episode_record_next_ns" not in source
    assert "latest_dataset_feedback" in source
    assert "dataset_snapshot_ns = time.monotonic_ns()" in source


def test_receiver_keeps_ctrl_fifo_and_coalesces_each_hts_modality() -> None:
    receiver = QuestDatagramReceiverWorker(
        bind="127.0.0.1", port=9000, allowed_sender=None, capacity=4
    )
    receiver.queue.put(_datagram(b"CTRL,old", 1))
    receiver.queue.put(_datagram(b"CTRL,new", 2))
    drained = _store_hts_sequence(
        receiver,
        _datagram(_head_payload(1), 3),
        _datagram(_hand_payload("Right", 1), 4),
        _datagram(_hand_payload("Right", 2), 5),
    )
    assert [item.payload for item in drained[:2]] == [b"CTRL,old", b"CTRL,new"]
    assert {item.payload for item in drained[2:]} == {
        _head_payload(1),
        _hand_payload("Right", 2),
    }
    diagnostics = receiver.diagnostics()
    assert diagnostics["quest_right_hand_superseded"] == 1
    assert diagnostics["quest_head_superseded"] == 0


def test_receiver_interleaved_right_and_head_keep_each_latest_sample() -> None:
    receiver = QuestDatagramReceiverWorker(
        bind="127.0.0.1", port=9000, allowed_sender=None
    )
    drained = _store_hts_sequence(
        receiver,
        _datagram(_hand_payload("Right", 1), 1),
        _datagram(_head_payload(1), 2),
        _datagram(_hand_payload("Right", 2), 3),
        _datagram(_head_payload(2), 4),
    )
    assert {item.payload for item in drained} == {
        _hand_payload("Right", 2),
        _head_payload(2),
    }
    diagnostics = receiver.diagnostics()
    assert diagnostics["quest_right_hand_superseded"] == 1
    assert diagnostics["quest_head_superseded"] == 1


def test_receiver_interleaved_left_right_and_head_keep_all_modalities() -> None:
    receiver = QuestDatagramReceiverWorker(
        bind="127.0.0.1", port=9000, allowed_sender=None
    )
    drained = _store_hts_sequence(
        receiver,
        _datagram(_hand_payload("Left", 1), 1),
        _datagram(_hand_payload("Right", 1), 2),
        _datagram(_head_payload(1), 3),
    )
    assert {item.payload for item in drained} == {
        _hand_payload("Left", 1),
        _hand_payload("Right", 1),
        _head_payload(1),
    }


def test_receiver_preserves_ordered_ctrl_fifo_alongside_latest_hts() -> None:
    receiver = QuestDatagramReceiverWorker(
        bind="127.0.0.1", port=9000, allowed_sender=None, capacity=4
    )
    receiver.queue.put(_datagram(_ctrl_payload(1, 1.0), 3))
    receiver.queue.put(_datagram(_ctrl_payload(2, 0.0), 5))
    drained = _store_hts_sequence(
        receiver,
        _datagram(_head_payload(1), 1),
        _datagram(_hand_payload("Right", 1), 2),
        _datagram(_hand_payload("Right", 2), 4),
    )
    # Requeue the same CTRL packets to exercise the bounded CTRL FIFO
    # independently from the modality slots.
    assert [
        item.payload
        for item in drained
        if item.payload.startswith(b"CTRL,")
    ] == [_ctrl_payload(1, 1.0), _ctrl_payload(2, 0.0)]

    receiver.queue.put(_datagram(_ctrl_payload(1, 1.0), 3))
    receiver.queue.put(_datagram(_ctrl_payload(2, 0.0), 5))
    receiver._store_latest_hts(_datagram(_head_payload(1), 1))
    receiver._store_latest_hts(_datagram(_hand_payload("Right", 2), 4))
    first = receiver.drain(max_controller_packets=1)
    second = receiver.drain(max_controller_packets=1)
    assert [
        item.payload
        for item in first
        if item.payload.startswith(b"CTRL,")
    ] == [_ctrl_payload(1, 1.0)]
    assert [item.payload for item in second] == [_ctrl_payload(2, 0.0)]
    assert {
        item.payload for item in first if not item.payload.startswith(b"CTRL,")
    } == {_head_payload(1), _hand_payload("Right", 2)}


def test_interleaved_hts_reaches_canonical_arm_capture_boundary() -> None:
    receiver = QuestDatagramReceiverWorker(
        bind="127.0.0.1", port=9000, allowed_sender=None
    )
    drained = _store_hts_sequence(
        receiver,
        _datagram(_head_payload(1), 1_000_000_000),
        _datagram(_hand_payload("Right", 1), 1_000_000_001),
        _datagram(_hand_payload("Right", 2), 1_000_000_002),
    )
    assembler = HtsCanonicalAssembler(stale_after_s=0.25)
    state = None
    for datagram in drained:
        state = assembler.ingest(
            parse_hts_datagram(datagram.payload),
            receive_monotonic_ns=datagram.receive_monotonic_ns,
            source_endpoint=datagram.source_endpoint,
            datagram_size=len(datagram.payload),
        )
    assert state is not None
    assert state.right.tracking_valid
    assert len(state.right.joints) == 21
    assert state.head is not None and state.head.tracking_valid

    arm = ArmClutchMachine(stale_after_s=0.15)
    assert (
        arm.step(
            AnalogClutchSample(0.0, 1_000_000_010, 1),
            now_ns=1_000_000_010,
            controller_valid=True,
            continuous_inputs_valid=state.right.tracking_valid,
            capture_inputs_valid=(
                state.right.tracking_valid
                and state.head is not None
                and state.head.tracking_valid
            ),
        )
        is ClutchAction.FREEZE
    )
    assert (
        arm.step(
            AnalogClutchSample(1.0, 1_000_000_020, 2),
            now_ns=1_000_000_020,
            controller_valid=True,
            continuous_inputs_valid=state.right.tracking_valid,
            capture_inputs_valid=(
                state.right.tracking_valid
                and state.head is not None
                and state.head.tracking_valid
            ),
        )
        is ClutchAction.CAPTURE_ARM_REFERENCE
    )


def test_async_rh56_diagnostic_sink_accepts_records_off_submitter(tmp_path: Path) -> None:
    stream = (tmp_path / "rh56.jsonl").open("w", encoding="utf-8")
    recorder = AsyncJsonlRecorder(
        BoundedJsonlRecorder(stream, capacity=4, flush_every_records=1),
        capacity=4,
    )
    recorder.start()
    recorder({"record_type": "rh56_telemetry", "monotonic_ns": 1})
    recorder.close()
    stream.close()
    assert recorder.telemetry_record_count == 1
    assert "rh56_telemetry" in (tmp_path / "rh56.jsonl").read_text(encoding="utf-8")


def test_native_compact_status_cadence_is_configurable(tmp_path: Path) -> None:
    metrics = tmp_path / "native.json"
    result = subprocess.run(
        [
            "build/jaka_servo_worker/jaka_servo_worker",
            "--mode",
            "dry-run",
            "--duration-s",
            "0.05",
            "--status-every-cycles",
            "4",
            "--target-socket",
            str(tmp_path / "target.sock"),
            "--metrics-file",
            str(metrics),
        ],
        check=False,
    )
    assert result.returncode == 0
    assert json.loads(metrics.read_text(encoding="utf-8"))["status_every_cycles"] == 4
