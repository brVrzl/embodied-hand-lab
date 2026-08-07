from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any, TextIO

from embodiment_core.config import load_yaml
from motion_input import HtsCanonicalAssembler, SerializationError, parse_hts_datagram
from motion_input.hts_transport import HtsRawRecordingWriter
from quest_jaka_sim import (
    ReplayConfig,
    SharedJakaTargetGenerator,
    SmoothQuestJakaSession,
    with_physical_rh56_retarget,
)
from quest_jaka_sim.live_controller import LiveQuestControllerRouter
from quest_jaka_sim.live_input import QuestDatagramReceiverWorker
from quest_jaka_sim.hand_retarget import (
    HandRetargetCalibration,
    ProjectRh56Retargeter,
    QuestHandSkeleton,
)
from quest_jaka_sim.output import RecordingArmTargetAdapter
from rh56_driver.hand_schema import CANONICAL_HAND_ORDER
from rh56_driver.pc_direct_control import (
    HandOperation,
    RH56PcDirectControl,
    inspect_serial_device,
    require_serial_by_id_path,
)
from rh56_driver.pc_direct_worker import RH56PcDirectWorker
from rh56_driver.serial_backend import RH56SerialBackend
from rh56_driver.telemetry import BoundedJsonlRecorder

MAX_READ_ONLY_DURATION_SEC = 60.0
MAX_BOUNDED_COMMAND_DURATION_SEC = 10.0
MAPPING_CHECK_POSES = (
    ("open", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("index", (0.55, 0.0, 0.0, 0.0, 0.0, 0.0)),
    ("middle", (0.0, 0.55, 0.0, 0.0, 0.0, 0.0)),
    ("ring", (0.0, 0.0, 0.55, 0.0, 0.0, 0.0)),
    ("pinky_little", (0.0, 0.0, 0.0, 0.55, 0.0, 0.0)),
    ("thumb_curve", (0.0, 0.0, 0.0, 0.0, 0.55, 0.0)),
    ("thumb_lateral", (0.0, 0.0, 0.0, 0.0, 0.0, 0.55)),
    ("open_return", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
)


def _timestamp_rate_hz(timestamps_ns: list[int]) -> float | None:
    if len(timestamps_ns) < 2 or timestamps_ns[-1] <= timestamps_ns[0]:
        return None
    return (len(timestamps_ns) - 1) * 1e9 / (
        timestamps_ns[-1] - timestamps_ns[0]
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "RH56DFX PC-direct USB/RS485 hand-only entry. This tool never imports or connects JAKA. "
            "Default is dry-run; real hardware requires --real and an explicit device path."
        )
    )
    parser.add_argument(
        "--device",
        required=False,
        default="",
        help="Preferred /dev/serial/by-id/... path, or an identity-checked custom CH341 tty.",
    )
    parser.add_argument(
        "--allow-direct-ch341-device",
        action="store_true",
        help=(
            "Explicit fallback for a single custom usb_ch341 /dev/ttyCH341USB<N> "
            "when system udev rules cannot create serial/by-id."
        ),
    )
    parser.add_argument("--config", default="configs/hand/rh56_pc_direct_teleop.yaml")
    parser.add_argument("--quest-config", default="configs/sim/quest_hts_jaka_mini2_live_demo.yaml")
    parser.add_argument(
        "--hand-calibration",
        default="configs/hand/quest_rh56_real_retarget.yaml",
        help=(
            "Quest feature calibration loaded only by this hand-only physical entry. "
            "The simulation config is not modified."
        ),
    )
    parser.add_argument(
        "--real",
        action="store_true",
        help="Opt into the real RH56 serial path; without it this command is dry-run only.",
    )
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--read-only", action="store_true", help="Stage 1 feedback-only probe (default).")
    modes.add_argument("--bounded-command", action="store_true", help="Stage 2 measured-relative one-channel test.")
    modes.add_argument(
        "--bounded-channel-target",
        action="store_true",
        help="Stage 2 measured-relative ramp of one canonical channel to an explicit normalized target.",
    )
    modes.add_argument(
        "--bounded-pose",
        action="store_true",
        help="Stage 2 measured-activated ramp to one explicit six-channel canonical normalized pose.",
    )
    modes.add_argument("--quest-teleop", action="store_true", help="Stage 3 Quest grip hand-only teleoperation.")
    modes.add_argument(
        "--mapping-check",
        action="store_true",
        help="Hand-only six-channel target sequence; every pose dwells for at least four seconds.",
    )
    modes.add_argument("--write-runtime-config", action="store_true", help="Write the explicitly selected SPEED/FORCE operation.")
    modes.add_argument(
        "--clear-error",
        action="store_true",
        help="Perform one CLEAR_ERROR write followed by feedback verification.",
    )
    modes.add_argument(
        "--force-sensor-calibration",
        action="store_true",
        help="Perform the official no-load force-sensor calibration operation.",
    )
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--hold-sec", type=float, default=1.0)
    parser.add_argument(
        "--mapping-hold-sec",
        type=float,
        default=5.0,
        help="Mapping-check dwell per target; minimum is 4 seconds.",
    )
    parser.add_argument("--feedback-period-sec", type=float, default=0.2)
    parser.add_argument("--channel", choices=CANONICAL_HAND_ORDER)
    parser.add_argument("--delta", type=float)
    parser.add_argument("--target-normalized", type=float, nargs="+")
    parser.add_argument("--pose-label", default="")
    parser.add_argument("--speed", type=int, nargs=6)
    parser.add_argument("--force", type=int, nargs=6)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--allowed-sender")
    parser.add_argument("--manual-stop-accessible", action="store_true")
    parser.add_argument("--workspace-clear", action="store_true")
    parser.add_argument("--no-auto-retry", action="store_true")
    parser.add_argument("--configuration-write-understood", action="store_true")
    parser.add_argument("--mechanical-obstruction-cleared", action="store_true")
    parser.add_argument("--calibration-no-load-confirmed", action="store_true")
    parser.add_argument("--preflight-only", action="store_true", help="Inspect by-id binding; do not open serial.")
    parser.add_argument("--jsonl", default="", help="Per-feedback/telemetry JSONL path.")
    parser.add_argument("--summary", default="", help="Summary JSON path.")
    parser.add_argument("--capture", default="", help="Stage 3 raw Quest capture path.")
    parser.add_argument("--events", default="", help="Stage 3 shared session event JSONL path.")
    parser.add_argument("--out", default="", help="Deprecated alias for --summary.")
    return parser


def _mode(args: argparse.Namespace) -> str:
    if args.mapping_check:
        return "mapping-check"
    if args.bounded_channel_target:
        return "bounded-channel-target"
    if args.bounded_pose:
        return "bounded-pose"
    if args.bounded_command:
        return "bounded-command"
    if args.quest_teleop:
        return "quest-teleop"
    if args.write_runtime_config:
        return "write-runtime-config"
    if args.clear_error:
        return "clear-error"
    if args.force_sensor_calibration:
        return "force-sensor-calibration"
    return "read-only"


def validate_gate(args: argparse.Namespace) -> HandOperation | None:
    if not args.real:
        if args.preflight_only:
            raise ValueError("--preflight-only requires --real; the default is dry-run.")
        return None
    if not args.device:
        raise ValueError("--real requires --device /dev/serial/by-id/... .")
    require_serial_by_id_path(
        args.device,
        require_exists=not args.preflight_only,
        allow_direct_ch341=args.allow_direct_ch341_device,
    )
    if args.preflight_only:
        return None
    mode = _mode(args)
    operation = _operation(args)
    if args.duration_sec <= 0.0:
        raise ValueError("--duration-sec must be positive.")
    if mode == "read-only" and args.duration_sec > MAX_READ_ONLY_DURATION_SEC:
        raise ValueError(f"Read-only duration is limited to {MAX_READ_ONLY_DURATION_SEC:g} seconds.")
    if mode in {"bounded-command", "bounded-channel-target", "bounded-pose"}:
        if args.duration_sec > MAX_BOUNDED_COMMAND_DURATION_SEC:
            raise ValueError(f"Bounded command duration is limited to {MAX_BOUNDED_COMMAND_DURATION_SEC:g} seconds.")
    if mode == "bounded-command":
        if args.channel is None or args.delta is None:
            raise ValueError("--bounded-command requires explicit --channel and --delta.")
        if not math.isfinite(args.delta) or args.delta == 0.0:
            raise ValueError("--delta must be finite and nonzero.")
        if args.hold_sec < 0.0:
            raise ValueError("--hold-sec must be nonnegative.")
    if mode == "bounded-channel-target":
        if args.channel is None or args.target_normalized is None:
            raise ValueError(
                "--bounded-channel-target requires --channel and one --target-normalized value."
            )
        if len(args.target_normalized) != 1:
            raise ValueError(
                "--bounded-channel-target requires exactly one --target-normalized value."
            )
    if mode == "bounded-pose":
        if args.target_normalized is None or len(args.target_normalized) != 6:
            raise ValueError(
                "--bounded-pose requires six canonical --target-normalized values."
            )
        if not args.pose_label.strip():
            raise ValueError("--bounded-pose requires a nonempty --pose-label.")
    if mode in {"bounded-channel-target", "bounded-pose"}:
        assert args.target_normalized is not None
        if not all(
            math.isfinite(value) and 0.0 <= value <= 1.0
            for value in args.target_normalized
        ):
            raise ValueError("Bounded normalized targets must remain within [0, 1].")
        if args.hold_sec < 0.0:
            raise ValueError("--hold-sec must be nonnegative.")
    if mode == "mapping-check" and args.mapping_hold_sec < 4.0:
        raise ValueError("--mapping-hold-sec must be at least 4 seconds.")
    if mode in {
        "bounded-command",
        "bounded-channel-target",
        "bounded-pose",
        "mapping-check",
        "quest-teleop",
        "clear-error",
        "force-sensor-calibration",
    } and not (
        args.manual_stop_accessible and args.workspace_clear and args.no_auto_retry
    ):
        raise PermissionError(
            f"--{mode} requires --manual-stop-accessible, --workspace-clear, and --no-auto-retry."
        )
    if mode == "write-runtime-config":
        if args.speed is None or args.force is None:
            raise ValueError("--write-runtime-config requires explicit six-channel --speed and --force values.")
        if not args.configuration_write_understood:
            raise PermissionError("--write-runtime-config requires --configuration-write-understood.")
    if mode == "clear-error":
        if not args.mechanical_obstruction_cleared:
            raise PermissionError(
                "--clear-error requires --mechanical-obstruction-cleared."
            )
        if args.duration_sec > MAX_BOUNDED_COMMAND_DURATION_SEC:
            raise ValueError(
                f"Fault-reset verification is limited to {MAX_BOUNDED_COMMAND_DURATION_SEC:g} seconds."
            )
    if mode == "force-sensor-calibration":
        if not args.calibration_no_load_confirmed:
            raise PermissionError(
                "--force-sensor-calibration requires --calibration-no-load-confirmed."
            )
        if not 8.0 <= args.duration_sec <= 15.0:
            raise ValueError(
                "Force-sensor calibration observation duration must be within [8, 15] seconds."
            )
    if args.feedback_period_sec <= 0.0:
        raise ValueError("--feedback-period-sec must be positive.")
    return operation


def _open_output(path: str) -> TextIO | None:
    if not path:
        return None
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    return output.open("x", encoding="utf-8")


def _write_jsonl(stream: TextIO | None, row: dict[str, Any]) -> None:
    if stream is not None:
        stream.write(json.dumps(row, sort_keys=True) + "\n")
        stream.flush()


def _write_summary(result: dict[str, Any], path_value: str) -> None:
    serialized = json.dumps(result, indent=2, ensure_ascii=False) + "\n"
    if path_value:
        path = Path(path_value)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x", encoding="utf-8") as output:
            output.write(serialized)
    print(serialized, end="")


def _operation(args: argparse.Namespace) -> HandOperation:
    return {
        "read-only": HandOperation.HAND_ONLY,
        "bounded-command": HandOperation.HAND_ONLY,
        "bounded-channel-target": HandOperation.HAND_ONLY,
        "bounded-pose": HandOperation.HAND_ONLY,
        "mapping-check": HandOperation.HAND_ONLY,
        "quest-teleop": HandOperation.HAND_ONLY,
        "write-runtime-config": HandOperation.RUNTIME_CONFIG,
        "clear-error": HandOperation.FAULT_RESET,
        "force-sensor-calibration": HandOperation.FORCE_SENSOR_CALIBRATION,
    }[_mode(args)]


def _backend_counts(backend: RH56SerialBackend) -> dict[str, int]:
    return {
        "register_write_count": int(backend.register_write_count),
        "timeout_count": int(backend.timeout_count),
        "checksum_failure_count": int(backend.checksum_failure_count),
        "protocol_error_count": int(backend.protocol_error_count),
    }


def _feedback_row(control: RH56PcDirectControl) -> dict[str, Any]:
    feedback = control.last_feedback
    assert feedback is not None
    row = control.episode_record(feedback.monotonic_ns)
    row["angle_act"] = list(feedback.position_raw)
    row["force_act"] = list(feedback.load_or_force_raw_count)
    row["current"] = list(feedback.current_raw_count)
    row["error"] = list(feedback.error)
    row["status"] = list(feedback.status)
    row["read_latency_ms"] = feedback.read_latency_ms
    return row


def _run_read_only(
    args: argparse.Namespace,
    control: RH56PcDirectControl,
    backend: RH56SerialBackend,
    recorder: BoundedJsonlRecorder,
) -> dict[str, Any]:
    positions: list[tuple[float, ...]] = []
    latencies: list[float] = []
    errors_seen: set[tuple[int, ...]] = set()
    status_seen: set[tuple[int, ...]] = set()
    started = time.monotonic()
    next_feedback = started
    while time.monotonic() - started < args.duration_sec:
        now = time.monotonic()
        if now < next_feedback:
            time.sleep(min(0.002, next_feedback - now))
            continue
        feedback = control.poll_feedback(time.monotonic_ns())
        positions.append(feedback.position_raw)
        latencies.append(feedback.read_latency_ms)
        errors_seen.add(feedback.error)
        status_seen.add(feedback.status)
        recorder(_feedback_row(control))
        print(
            f"RH56 READ angle={list(map(int, feedback.position_raw))} "
            f"current={list(map(int, feedback.current_raw_count))} "
            f"error={list(feedback.error)} status={list(feedback.status)} "
            f"latency_ms={feedback.read_latency_ms:.2f}"
        )
        next_feedback += args.feedback_period_sec
    elapsed = max(time.monotonic() - started, 1e-9)
    repeated = sum(a == b for a, b in zip(positions, positions[1:]))
    result = {
        "feedback_samples": len(positions),
        "feedback_rate_hz": len(positions) / elapsed,
        "read_latency_mean_ms": 0.0 if not latencies else sum(latencies) / len(latencies),
        "read_latency_max_ms": 0.0 if not latencies else max(latencies),
        "repeated_frame_ratio": 0.0 if len(positions) < 2 else repeated / (len(positions) - 1),
        "initial_angle_act": None if not positions else list(positions[0]),
        "final_angle_act": None if not positions else list(positions[-1]),
        "error_values_seen": [list(value) for value in sorted(errors_seen)],
        "status_values_seen": [list(value) for value in sorted(status_seen)],
    }
    result.update(_backend_counts(backend))
    return result


def _observe_service_motion(
    args: argparse.Namespace,
    control: RH56PcDirectControl,
    recorder: BoundedJsonlRecorder,
) -> dict[str, Any]:
    samples: list[dict[str, Any]] = []
    started = time.monotonic()
    next_feedback = started
    while time.monotonic() - started < args.duration_sec:
        now = time.monotonic()
        if now < next_feedback:
            time.sleep(min(0.002, next_feedback - now))
            continue
        feedback = control.poll_feedback(time.monotonic_ns())
        row = _feedback_row(control)
        recorder(row)
        samples.append(row)
        print(
            f"RH56 SERVICE angle={list(map(int, feedback.position_raw))} "
            f"current={list(map(int, feedback.current_raw_count))} "
            f"force={list(map(int, feedback.load_or_force_raw_count))} "
            f"error={list(feedback.error)} status={list(feedback.status)}"
        )
        next_feedback += args.feedback_period_sec
    if not samples:
        raise RuntimeError("RH56 service operation produced no verification feedback.")
    final = samples[-1]
    final_status = [int(value) for value in final["status"]]
    final_error = [int(value) for value in final["error"]]
    return {
        "verification_samples": len(samples),
        "final_angle_act": list(final["angle_act"]),
        "final_force_act": list(final["force_act"]),
        "final_current": list(final["current"]),
        "final_error": final_error,
        "final_status": final_status,
        "status_values_seen": sorted(
            {tuple(int(value) for value in row["status"]) for row in samples}
        ),
        "error_values_seen": sorted(
            {tuple(int(value) for value in row["error"]) for row in samples}
        ),
    }


def _run_fault_reset(
    args: argparse.Namespace,
    control: RH56PcDirectControl,
    recorder: BoundedJsonlRecorder,
) -> dict[str, Any]:
    control.clear_device_error()
    result = _observe_service_motion(args, control, recorder)
    if any(result["final_error"]) or any(
        value == 7 for value in result["final_status"]
    ):
        raise RuntimeError(
            "RH56 fault reset did not clear final ERROR/actuator-fault STATUS."
        )
    result["fault_reset_write_count"] = 1
    return result


def _run_force_sensor_calibration(
    args: argparse.Namespace,
    control: RH56PcDirectControl,
    recorder: BoundedJsonlRecorder,
) -> dict[str, Any]:
    initial = control.poll_feedback(time.monotonic_ns())
    recorder(_feedback_row(control))
    if any(initial.error) or any(value == 7 for value in initial.status):
        raise RuntimeError(
            "RH56 must have clear ERROR and no actuator-fault STATUS before force calibration."
        )
    control.start_force_sensor_calibration()
    result = _observe_service_motion(args, control, recorder)
    if any(result["final_error"]) or any(
        value == 7 for value in result["final_status"]
    ):
        raise RuntimeError(
            "RH56 force-sensor calibration ended with ERROR/actuator-fault STATUS."
        )
    result.update(
        {
            "force_sensor_calibration_write_count": 1,
            "initial_angle_act": list(initial.position_raw),
            "initial_force_act": list(initial.load_or_force_raw_count),
        }
    )
    return result


def _run_bounded(
    args: argparse.Namespace,
    control: RH56PcDirectControl,
    backend: RH56SerialBackend,
    recorder: BoundedJsonlRecorder,
) -> dict[str, Any]:
    first = control.poll_feedback(time.monotonic_ns())
    if abs(float(args.delta)) > control.delta_limit:
        raise ValueError(
            f"Stage 2 |--delta| must not exceed production per-cycle limit {control.delta_limit:g}."
        )
    target = list(first.position_normalized)
    channel_index = CANONICAL_HAND_ORDER.index(args.channel)
    target[channel_index] = float(
        max(0.0, min(control.max_close, target[channel_index] + float(args.delta)))
    )
    control.activate(time.monotonic_ns())
    started = time.monotonic()
    next_feedback = started
    while time.monotonic() - started < args.duration_sec:
        now_ns = time.monotonic_ns()
        control.command(target, now_ns)
        if time.monotonic() >= next_feedback:
            control.poll_feedback(now_ns)
            recorder(_feedback_row(control))
            next_feedback += args.feedback_period_sec
        time.sleep(0.001)
    control.hold("bounded_target_reached")
    hold_deadline = time.monotonic() + args.hold_sec
    while time.monotonic() < hold_deadline:
        control.poll_feedback(time.monotonic_ns())
        recorder(_feedback_row(control))
        time.sleep(args.feedback_period_sec)
    return {
        "channel": args.channel,
        "requested_delta_normalized": args.delta,
        "initial_target_from_measured_angle_act": list(first.position_normalized),
        "bounded_target_normalized": target,
        "final_angle_act": list(control.last_feedback.position_raw),
        **_backend_counts(backend),
    }


def _run_worker_bounded_target(
    args: argparse.Namespace,
    control: RH56PcDirectControl,
    recorder: BoundedJsonlRecorder,
) -> dict[str, Any]:
    """Use the production worker to ramp from measured activation to a target."""

    worker = RH56PcDirectWorker(control, record=recorder)
    first = worker.start(_operation(args))
    activation = worker.activate_from_measured(time.monotonic_ns())
    activation_deadline = time.monotonic() + min(
        1.0, control.feedback_stale_timeout_ns / 1e9
    )
    while time.monotonic() < activation_deadline:
        worker.raise_if_failed()
        diagnostics = worker.diagnostics_snapshot(include_windows=False)
        if int(diagnostics["successful_serial_write_count"]) >= 1:
            break
        time.sleep(0.001)
    else:
        raise RuntimeError("Measured activation write did not complete before target submission.")

    target = list(activation)
    if args.bounded_channel_target:
        assert args.channel is not None
        assert args.target_normalized is not None
        target[CANONICAL_HAND_ORDER.index(args.channel)] = float(
            args.target_normalized[0]
        )
    else:
        assert args.target_normalized is not None
        target = [float(value) for value in args.target_normalized]
    worker.submit_target(target, time.monotonic_ns())
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration_sec:
            worker.raise_if_failed()
            time.sleep(0.005)
        worker.hold("bounded_target_reached")
        hold_deadline = time.monotonic() + args.hold_sec
        while time.monotonic() < hold_deadline:
            worker.raise_if_failed()
            time.sleep(0.005)
    finally:
        worker.hold("bounded_target_end")
        worker.cleanup()
    feedback = worker.latest_feedback or first
    return {
        "pose_label": args.pose_label or args.channel,
        "channel": args.channel,
        "initial_angle_act": list(first.position_raw),
        "initial_measured_normalized": list(first.position_normalized),
        "measured_activation_target_normalized": list(activation),
        "bounded_target_normalized": target,
        "final_angle_act": list(feedback.position_raw),
        "final_measured_normalized": list(feedback.position_normalized),
        "final_current": list(feedback.current_raw_count),
        "final_force_act": list(feedback.load_or_force_raw_count),
        "final_status": list(feedback.status),
        "final_error": list(feedback.error),
        "rh56_worker_failure": worker.failure_record,
        "rh56_diagnostics": worker.diagnostics_snapshot(),
    }


def _run_quest(
    args: argparse.Namespace,
    control: RH56PcDirectControl,
    recorder: BoundedJsonlRecorder,
) -> dict[str, Any]:
    worker = RH56PcDirectWorker(control, record=recorder)
    worker.start(_operation(args))
    quest_config = _load_hand_only_quest_config(
        args.quest_config,
        args.hand_calibration,
    )
    generator = SharedJakaTargetGenerator(quest_config, mjcf_path=quest_config.mjcf_path)
    arm_record = RecordingArmTargetAdapter()
    session = SmoothQuestJakaSession(
        quest_config,
        generator,
        arm_output=arm_record,
        normalized_hand_output=worker,
        arm_input_enabled=False,
    )
    clutch = quest_config.raw["clutches"]
    router = LiveQuestControllerRouter(
        stale_after_s=float(clutch["stale_after_ms"]) / 1000.0,
        released_at=float(clutch["released_at"]),
    )
    capture_path = Path(args.capture) if args.capture else None
    if capture_path is not None:
        capture_path.parent.mkdir(parents=True, exist_ok=True)
    capture = HtsRawRecordingWriter(capture_path, metadata={"stage": "quest-rh56-hand-only"}) if capture_path else None
    if capture is not None:
        capture.open()
    receiver = QuestDatagramReceiverWorker(
        bind=args.bind,
        port=args.port,
        allowed_sender=args.allowed_sender,
        record=None if capture is None else capture.write,
    )
    events = _open_output(args.events)
    started = time.monotonic()
    next_tick = started
    try:
        receiver.start()
        while time.monotonic() - started < args.duration_sec:
            receiver.raise_if_failed()
            worker.raise_if_failed()
            for datagram in receiver.drain():
                router.ingest(datagram, session)
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(0.001, next_tick - now))
                continue
            now_ns = time.monotonic_ns()
            router.poll(now_ns, session)
            tick = session.control_tick(now_ns)
            event = dict(session.event_records[-1])
            event["physical_stage"] = "quest-rh56-hand-only"
            event["arm_input_enabled"] = False
            _write_jsonl(events, event)
            if tick.accepted_target is not None:
                raise RuntimeError("hand-only mode generated an arm AcceptedArmTarget")
            next_tick += 1.0 / float(quest_config.raw["rates"]["target_generation_hz"])
    finally:
        receiver.close()
        worker.hold("episode_end")
        worker.cleanup()
        if capture is not None:
            capture.close()
        if events is not None:
            events.close()
    return {
        "quest_receiver_count": 1,
        "jaka_sessions": 0,
        "arm_accepted_targets": len(arm_record.targets),
        "hand_telemetry_records": recorder.telemetry_record_count,
        "hand_initial_target_source": "measured_ANGLE_ACT",
        "final_hand_record": recorder.last_telemetry_record,
        "rh56_worker_failure": worker.failure_record,
        "rh56_diagnostics": worker.diagnostics_snapshot(),
        "rh56_logging": recorder.summary(),
        "quest_hand_input_frame_count": len(session.input_timestamps_ns),
        "quest_hand_input_rate_hz": _timestamp_rate_hz(
            session.input_timestamps_ns
        ),
        "quest_grip_sample_count": len(session.grip_timestamps_ns),
        "quest_grip_sample_rate_hz": _timestamp_rate_hz(
            session.grip_timestamps_ns
        ),
        "hand_retarget_count": len(session.hand_timestamps_ns),
        "hand_retarget_rate_hz": _timestamp_rate_hz(
            session.hand_timestamps_ns
        ),
        "hand_calibration_path": args.hand_calibration,
        "hand_calibration_id": (
            None
            if session.hand_retargeter is None
            else session.hand_retargeter.calibration.calibration_id
        ),
    }


def _run_mapping_check(
    args: argparse.Namespace,
    control: RH56PcDirectControl,
    recorder: BoundedJsonlRecorder,
) -> dict[str, Any]:
    """Run a slow, hand-only six-channel pose sequence with Quest diagnostics."""

    worker = RH56PcDirectWorker(control, record=recorder)
    worker.start(_operation(args))
    worker.activate_from_measured(time.monotonic_ns())
    activation_deadline = time.monotonic() + 1.0
    while time.monotonic() < activation_deadline:
        worker.raise_if_failed()
        if int(worker.diagnostics_snapshot(include_windows=False)["successful_serial_write_count"]) >= 1:
            break
        time.sleep(0.001)
    else:
        raise RuntimeError("Measured activation write did not complete before mapping check.")

    capture_path = Path(args.capture) if args.capture else None
    if capture_path is not None:
        capture_path.parent.mkdir(parents=True, exist_ok=True)
    capture = (
        HtsRawRecordingWriter(capture_path, metadata={"stage": "rh56-six-channel-mapping-check"})
        if capture_path
        else None
    )
    if capture is not None:
        capture.open()
    events = _open_output(args.events)
    receiver = QuestDatagramReceiverWorker(
        bind=args.bind,
        port=args.port,
        allowed_sender=args.allowed_sender,
        record=None if capture is None else capture.write,
    )
    assembler = HtsCanonicalAssembler(stale_after_s=0.25)
    calibration = HandRetargetCalibration.load(args.hand_calibration)
    retargeter = ProjectRh56Retargeter(calibration)
    latest_debug: dict[str, Any] | None = None
    printed_at = 0.0

    def drain_quest(now_ns: int, pose_name: str, target: tuple[float, ...]) -> None:
        nonlocal latest_debug, printed_at
        for datagram in receiver.drain():
            try:
                state = assembler.ingest(
                    parse_hts_datagram(datagram.payload),
                    receive_monotonic_ns=datagram.receive_monotonic_ns,
                    source_endpoint=datagram.source_endpoint,
                    datagram_size=len(datagram.payload),
                )
            except SerializationError:
                continue
            if not state.right.tracking_valid:
                continue
            result = retargeter.retarget(QuestHandSkeleton.from_observation(state.right))
            if not result.valid:
                continue
            diagnostics = result.pinch_diagnostics
            thumb_names = {
                "thumb_metacarpal",
                "thumb_proximal",
                "thumb_distal",
                "thumb_tip",
            }
            raw_thumb_joints = {
                joint.name: list(joint.position_m)
                for joint in state.right.joints
                if joint.name in thumb_names
            }
            latest_debug = {
                "pose": pose_name,
                "raw_quest_thumb_joints_m": raw_thumb_joints,
                "raw_thumb_curve_rad": diagnostics.get("thumb_raw_bend_rad"),
                "normalized_thumb_curve": diagnostics.get("thumb_normalized_bend"),
                "thumb_lateral": diagnostics.get("thumb_lateral_effective_feature"),
                "quest_normalized_targets": [
                    float(result.normalized_targets[name]) for name in CANONICAL_HAND_ORDER
                ],
                "rh56_target_normalized": list(target),
                "timestamp_monotonic_ns": now_ns,
            }
            if events is not None:
                _write_jsonl(events, dict(latest_debug))
            if time.monotonic() - printed_at >= 0.5:
                print(
                    "MAPPING "
                    f"pose={pose_name} "
                    f"raw_thumb_joints_m={raw_thumb_joints} "
                    f"raw_curve_rad={latest_debug['raw_thumb_curve_rad']} "
                    f"normalized_curve={latest_debug['normalized_thumb_curve']} "
                    f"thumb_lateral={latest_debug['thumb_lateral']} "
                    f"rh56_target={list(target)} "
                    f"quest_targets={latest_debug['quest_normalized_targets']}",
                    flush=True,
                )
                printed_at = time.monotonic()

    receiver.start()
    try:
        for pose_name, target in MAPPING_CHECK_POSES:
            worker.raise_if_failed()
            worker.submit_target(target, time.monotonic_ns())
            print(
                f"MAPPING_TARGET pose={pose_name} target={list(target)} "
                f"hold_sec={args.mapping_hold_sec:g}; follow this pose with the real hand.",
                flush=True,
            )
            deadline = time.monotonic() + args.mapping_hold_sec
            while time.monotonic() < deadline:
                worker.raise_if_failed()
                drain_quest(time.monotonic_ns(), pose_name, target)
                time.sleep(0.01)
    finally:
        receiver.close()
        worker.hold("mapping_check_complete")
        worker.cleanup()
        if capture is not None:
            capture.close()
        if events is not None:
            events.close()
    return {
        "jaka_sessions": 0,
        "mapping_pose_count": len(MAPPING_CHECK_POSES),
        "mapping_hold_sec": args.mapping_hold_sec,
        "mapping_poses": [name for name, _ in MAPPING_CHECK_POSES],
        "last_quest_mapping_debug": latest_debug,
        "hand_telemetry_records": recorder.telemetry_record_count,
        "rh56_worker_failure": worker.failure_record,
        "rh56_diagnostics": worker.diagnostics_snapshot(),
        "rh56_logging": recorder.summary(),
    }


def _load_hand_only_quest_config(
    quest_config_path: str,
    hand_calibration_path: str,
) -> ReplayConfig:
    """Load the shared maintained physical Quest-to-RH56 mapping."""

    return with_physical_rh56_retarget(
        ReplayConfig.load(quest_config_path),
        hand_calibration_path,
    )


def main() -> None:
    args = _build_parser().parse_args()
    operation = validate_gate(args)
    summary_path = args.summary or args.out
    if not args.real:
        _write_summary(
            {
                "mode": "dry-run",
                "requested_mode": _mode(args),
                "real_hardware": False,
                "rh56_connected": False,
                "jaka_sessions": 0,
                "operation": None,
                "hardware_validation": "not_run",
            },
            summary_path,
        )
        return
    identity = inspect_serial_device(
        args.device,
        allow_direct_ch341=args.allow_direct_ch341_device,
    )
    if args.allow_direct_ch341_device and not args.device.startswith("/dev/serial/by-id/"):
        if (
            identity.get("usb_vid") != "1a86"
            or identity.get("usb_pid") != "7523"
            or identity.get("usb_driver") != "usb_ch341"
        ):
            raise RuntimeError(
                "Direct CH341 fallback identity mismatch; no serial protocol was attempted."
            )
    if not args.preflight_only:
        if not identity.get("current_user_can_read") or not identity.get("current_user_can_write"):
            raise PermissionError(
                "Current user lacks read/write access to the RH56 tty; check the device group "
                "and active group membership or install an administrator-managed udev rule."
            )
        if identity.get("occupied_pids"):
            raise RuntimeError(
                f"RH56 tty is already occupied by PID(s) {identity['occupied_pids']}."
            )
    result: dict[str, Any] = {
        "mode": "preflight-only" if args.preflight_only else _mode(args),
        "requested_device": identity.get("requested_device", identity["requested_by_id"]),
        "requested_device_by_id": (
            identity["requested_by_id"]
            if args.device.startswith("/dev/serial/by-id/")
            else None
        ),
        "device_binding_kind": (
            "stable_by_id"
            if args.device.startswith("/dev/serial/by-id/")
            else "explicit_custom_ch341_fallback"
        ),
        "resolved_tty": identity["resolved_tty"],
        "usb_vid": identity.get("usb_vid"),
        "usb_pid": identity.get("usb_pid"),
        "usb_serial": identity.get("usb_serial"),
        "usb_vendor": identity.get("usb_vendor"),
        "usb_model": identity.get("usb_model"),
        "usb_driver": identity.get("usb_driver"),
        "device_group": identity.get("device_group"),
        "current_user_can_read": identity.get("current_user_can_read"),
        "current_user_can_write": identity.get("current_user_can_write"),
        "occupied_pids": identity.get("occupied_pids"),
        "canonical_channel_order": list(CANONICAL_HAND_ORDER),
        "protocol_channel_order": [
            "pinky", "ring", "middle", "index", "thumb_close", "thumb_lateral"
        ],
        "feedback_conversion": "protocol_order_to_canonical_order",
        "duration_sec": args.duration_sec,
        "hardware_validation": "not_run" if args.preflight_only else "operator_executed",
    }
    if args.preflight_only:
        _write_summary(result, summary_path)
        return
    config = load_yaml(args.config)
    config["mode"] = "real"
    config["backend_type"] = "serial_protocol"
    config.setdefault("serial", {})["port"] = args.device
    backend = RH56SerialBackend(config)
    control = RH56PcDirectControl(backend, config)
    stream = _open_output(args.jsonl)
    diagnostics = config.get("diagnostics", {})
    recorder = BoundedJsonlRecorder(
        stream,
        capacity=int(diagnostics.get("telemetry_buffer_capacity", 64)),
        flush_every_records=int(diagnostics.get("telemetry_flush_every_records", 16)),
        flush_interval_sec=float(diagnostics.get("telemetry_flush_interval_sec", 1.0)),
    )
    started_ns = time.monotonic_ns()
    try:
        assert operation is not None
        if _mode(args) == "quest-teleop":
            result["operation"] = operation.value
            result.update(_run_quest(args, control, recorder))
        elif _mode(args) == "mapping-check":
            result["operation"] = operation.value
            result.update(_run_mapping_check(args, control, recorder))
        elif args.bounded_channel_target or args.bounded_pose:
            result["operation"] = operation.value
            result.update(_run_worker_bounded_target(args, control, recorder))
        else:
            control.open(operation)
            result["operation"] = operation.value
            if args.write_runtime_config:
                control.write_runtime_config(args.speed, args.force)
                result["runtime_config_write"] = {"speed": args.speed, "force": args.force}
            elif args.clear_error:
                result.update(_run_fault_reset(args, control, recorder))
            elif args.force_sensor_calibration:
                result.update(
                    _run_force_sensor_calibration(args, control, recorder)
                )
            elif args.bounded_command:
                result.update(_run_bounded(args, control, backend, recorder))
            else:
                result.update(_run_read_only(args, control, backend, recorder))
            control.hold("duration_elapsed")
            control.cleanup()
        result["outcome"] = "completed"
    except KeyboardInterrupt:
        result["outcome"] = "operator_keyboard_stop"
        result["hand_fault_reason"] = control.fault_reason
    except Exception as exc:
        result["outcome"] = "fault"
        result["error"] = str(exc)
        result["hand_fault_reason"] = control.fault_reason
        raise
    finally:
        if control.state.value != "HAND_DISABLED":
            control.cleanup()
        recorder.close()
        if stream is not None:
            stream.close()
        result["elapsed_sec"] = (time.monotonic_ns() - started_ns) / 1e9
        result["final_state"] = control.state.value
        result["final_transport"] = control.transport_state
        result.update(_backend_counts(backend))
        result["rh56_logging"] = recorder.summary()
        _write_summary(result, summary_path)


if __name__ == "__main__":
    main()
