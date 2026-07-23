#!/usr/bin/env python3
"""Run Quest/JAKA P2 command shadow or explicitly approved P4 arm teleoperation.

P0 never invokes this entry point.  P2 connects read-only and sends accepted
joint packets to the worker's no-EDG shadow mode.  P4 is separately gated and
uses the identical Python target-generation session with a thin joint adapter.
The Inspire hand is never imported or commanded by this tool.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from motion_input.hts_transport import HtsRawRecordingWriter
from quest_jaka_sim import (
    ReplayConfig,
    SharedJakaTargetGenerator,
    SmoothQuestJakaSession,
)
from quest_jaka_sim.live_controller import LiveQuestControllerRouter
from quest_jaka_sim.live_input import QuestDatagramReceiverWorker
from teleoperation.jaka.quest_adapter import (
    E2IsolatedForwardTranslationGuard,
    JakaAcceptedJointTargetAdapter,
)
from teleoperation.runtime.arm_only import ArmOnlyRuntime, NativeWorkerProcess
from teleoperation.wire import LatestTargetPublisher, StatusFlags, WorkerStatusReceiver


P2_APPROVAL = "I_AUTHORIZE_P2_QUEST_JAKA_COMMAND_SHADOW"
E2_APPROVAL = "I_AUTHORIZE_E2_ONE_SMALL_TCP_TRANSLATION"
P4_APPROVAL = "I_AUTHORIZE_P4_LIVE_QUEST_JAKA_TELEOPERATION"
POST_PAYLOAD_APPROVAL = "I_AUTHORIZE_ONE_POST_PAYLOAD_TELEOP_RERUN"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("p2-shadow", "e2-isolated", "p4-live", "post-payload-diagnostic"))
    parser.add_argument("--config", type=Path, default=Path("configs/sim/quest_hts_jaka_mini2_live_demo.yaml"))
    parser.add_argument("--worker", type=Path, default=Path("build/jaka_servo_worker/jaka_servo_worker"))
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--edg-state-ip", default="192.168.71.19")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--allowed-sender")
    parser.add_argument("--duration-sec", type=float, default=60.0)
    parser.add_argument("--approval", required=True)
    parser.add_argument("--estop-accessible", action="store_true")
    parser.add_argument("--workspace-clear", action="store_true")
    parser.add_argument("--rh56-command-path-absent", action="store_true")
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--metrics", type=Path, required=True)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--native-telemetry", type=Path)
    parser.add_argument("--event-extract", type=Path)
    parser.add_argument("--run-output-joint-velocity-limit-rad-s", type=float)
    parser.add_argument("--abort-on-diagnostic-acceleration-boundary", action="store_true")
    return parser


def _wait_status(runtime: ArmOnlyRuntime, native: NativeWorkerProcess, timeout_s: float = 8.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = runtime.latest_status()
        if status is not None and StatusFlags(status.flags) & StatusFlags.CONNECTED:
            return status
        if native.process is not None and native.process.poll() is not None:
            raise RuntimeError("JAKA worker exited before reporting connected state")
        time.sleep(0.005)
    raise RuntimeError("JAKA worker did not report connected state")


def _control_output_failed(*, reason: str, output_applied: bool) -> bool:
    """Treat accepted-target or heartbeat transport failure as fatal, not disengagement."""

    return reason != "DISENGAGED" and not output_applied


def main() -> int:
    args = _parser().parse_args()
    if args.duration_sec <= 0.0:
        raise SystemExit("duration must be positive")
    config = replace(ReplayConfig.load(args.config), engagement_schedule_s=())
    if args.run_output_joint_velocity_limit_rad_s is not None:
        if not 0.0 < args.run_output_joint_velocity_limit_rad_s <= config.output_contract.maximum_velocity_rad_s:
            raise SystemExit("run output velocity limit must be positive and no greater than the shared contract")
        config = replace(
            config,
            output_contract=replace(
                config.output_contract,
                maximum_velocity_rad_s=args.run_output_joint_velocity_limit_rad_s,
            ),
        )
    hardware = config.raw["hardware_adapter"]
    live = args.stage in ("e2-isolated", "p4-live", "post-payload-diagnostic")
    expected_approval = (
        E2_APPROVAL if args.stage == "e2-isolated"
        else POST_PAYLOAD_APPROVAL if args.stage == "post-payload-diagnostic"
        else P4_APPROVAL if live else P2_APPROVAL
    )
    if args.approval != expected_approval:
        raise SystemExit(f"exact approval required: {expected_approval}")
    if live:
        if not bool(hardware["physical_mapping_confirmed"]):
            raise SystemExit("P4 blocked: physical Quest/JAKA mapping has not been confirmed after P1/P2")
        if not (args.estop_accessible and args.workspace_clear and args.rh56_command_path_absent):
            raise SystemExit("P4 requires E-stop, clear-workspace, and no-RH56-command confirmations")
    if args.stage == "post-payload-diagnostic":
        if args.duration_sec > 60.0:
            raise SystemExit("post-payload diagnostic is limited to 60 seconds")
        if args.native_telemetry is None or args.event_extract is None:
            raise SystemExit("post-payload diagnostic requires native telemetry and event extract paths")
        if args.run_output_joint_velocity_limit_rad_s is None or args.run_output_joint_velocity_limit_rad_s > 1.0:
            raise SystemExit("post-payload diagnostic requires a shared output limit no greater than 1.0 rad/s")
        if not args.abort_on_diagnostic_acceleration_boundary:
            raise SystemExit("post-payload diagnostic requires the pre-SDK acceleration abort")
    print(f"STAGE={args.stage} STOP=Ctrl+C or release the left-index arm clutch")

    rates = config.raw["rates"]
    target_hz = float(rates["target_generation_hz"])
    if target_hz <= 0.0:
        raise SystemExit("target_generation_hz must be positive")
    if float(rates["ik_hz"]) != target_hz:
        raise SystemExit("shared IK and target-generation rates must match")
    if not math.isclose(
        float(hardware["servo_period_ms"]),
        1000.0 / float(rates["jaka_transport_hz"]),
        abs_tol=1e-12,
    ):
        raise SystemExit("JAKA transport rate and servo period disagree")
    args.log.parent.mkdir(parents=True, exist_ok=True)
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.capture.parent.mkdir(parents=True, exist_ok=True)
    if args.native_telemetry is not None:
        args.native_telemetry.parent.mkdir(parents=True, exist_ok=True)
    if args.event_extract is not None:
        args.event_extract.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="quest_jaka_hardware_") as directory:
        temporary = Path(directory)
        target_generator = SharedJakaTargetGenerator(config, mjcf_path=config.mjcf_path)
        target_socket = temporary / "target.sock"
        status_socket = temporary / "status.sock"
        runtime = ArmOnlyRuntime(
            LatestTargetPublisher(target_socket),
            WorkerStatusReceiver(status_socket),
        )
        base_jaka_adapter = JakaAcceptedJointTargetAdapter(
            runtime,
            allow_motion=live,
            joint_order=tuple(hardware["joint_order"]),
            joint_angle_unit=str(hardware["joint_angle_unit"]),
            command_mode=str(hardware["command_mode"]),
        )
        jaka_adapter = (
            E2IsolatedForwardTranslationGuard(
                base_jaka_adapter,
                startup_alignment_tolerance_rad=float(
                    hardware["startup_alignment_tolerance_rad"]
                ),
            )
            if args.stage == "e2-isolated"
            else base_jaka_adapter
        )
        session = SmoothQuestJakaSession(
            config,
            target_generator,
            arm_output=jaka_adapter,
        )
        clutch = config.raw["clutches"]
        router = LiveQuestControllerRouter(
            stale_after_s=float(clutch["stale_after_ms"]) / 1000.0,
            released_at=float(clutch["released_at"]),
        )
        worker_mode = "joint-teleop" if live else "joint-shadow"
        worker_args = [
            "--mode", worker_mode,
            "--hardware",
            "--robot-ip", args.robot_ip,
            "--edg-state-ip", args.edg_state_ip,
            "--duration-s", str(args.duration_sec + 2.0),
            "--target-socket", str(target_socket),
            "--status-socket", str(status_socket),
            "--metrics-file", str(args.metrics),
            "--expected-tool-id", str(hardware["expected_tool_id"]),
            "--expected-user-frame-id", str(hardware["expected_user_frame_id"]),
            # E2 is an additional launcher gate around the already-audited
            # native joint-teleop mode; the native process retains its P4 risk
            # acknowledgement rather than gaining a second motion mode.
            "--acknowledgement", P4_APPROVAL if live else expected_approval,
            "--warning-ms", str(hardware["command_stream_warning_ms"]),
            "--hold-ms", str(hardware["command_stream_timeout_ms"]),
            "--controlled-stop-ms", str(hardware["controlled_stop_timeout_ms"]),
            "--fatal-timeout-ms", str(hardware["fatal_communication_timeout_ms"]),
            "--excessive-tracking-error-abort-rad", str(hardware["excessive_tracking_error_abort_rad"]),
            "--excessive-tracking-error-consecutive-cycles", str(hardware["excessive_tracking_error_consecutive_cycles"]),
            "--startup-alignment-tolerance-rad", str(hardware["startup_alignment_tolerance_rad"]),
            # One authority: the EDG pass-through diagnostic consumes the
            # existing shared command contract rather than hardware-only copies.
            "--maximum-output-joint-velocity-rad-s", str(config.output_contract.maximum_velocity_rad_s),
            "--diagnostic-joint-acceleration-boundary-rad-s2", str(config.output_contract.maximum_acceleration_rad_s2),
        ]
        if live:
            worker_args.append("--monitor-controller-health-each-cycle")
        if args.native_telemetry is not None:
            worker_args.extend(("--cycle-telemetry-file", str(args.native_telemetry)))
        if args.abort_on_diagnostic_acceleration_boundary:
            worker_args.append("--abort-on-diagnostic-acceleration-boundary")
        native = NativeWorkerProcess(args.worker, worker_args)
        accepted = 0
        stop_reason = "duration_complete"
        abort_reason: str | None = None
        prior_engaged = False
        started = time.monotonic()
        next_tick = started
        status = None
        receiver: QuestDatagramReceiverWorker | None = None
        maximum_quest_displacement_m = 0.0
        minimum_continuation_fraction = 1.0
        clutch_release_monotonic_ns: int | None = None
        measured_joint_samples: list[tuple[float, ...]] = []
        native.start()
        try:
            status = _wait_status(runtime, native)
            measured_joint_samples.append(tuple(status.joint_position_rad))
            if isinstance(jaka_adapter, E2IsolatedForwardTranslationGuard):
                jaka_adapter.establish_startup_joint_position(
                    tuple(status.joint_position_rad)
                )
            target_generator.synchronize_authoritative_arm_joints(list(status.joint_position_rad))
            with HtsRawRecordingWriter(
                args.capture,
                metadata={"stage": args.stage, "hardware_commands": live},
            ) as capture, args.log.open("x", encoding="utf-8") as log:
                receiver = QuestDatagramReceiverWorker(
                    bind=args.bind,
                    port=args.port,
                    allowed_sender=args.allowed_sender,
                    record=capture.write,
                )
                receiver.start()
                started = time.monotonic()
                next_tick = started
                try:
                    while time.monotonic() - started < args.duration_sec:
                        if native.process is None or native.process.poll() is not None:
                            return_code = None if native.process is None else native.process.returncode
                            abort_reason = f"native_worker_exited:{return_code}"
                            stop_reason = abort_reason
                            jaka_adapter.stop()
                            break
                        receiver.raise_if_failed()
                        for datagram in receiver.drain():
                            router.ingest(datagram, session)
                        now = time.monotonic()
                        if now < next_tick:
                            time.sleep(min(0.001, next_tick - now))
                            continue
                        now_ns = time.monotonic_ns()
                        router.poll(now_ns, session)
                        latest_status = runtime.latest_status()
                        if latest_status is not None:
                            status = latest_status
                            sample = tuple(status.joint_position_rad)
                            if not measured_joint_samples or sample != measured_joint_samples[-1]:
                                measured_joint_samples.append(sample)
                            if isinstance(
                                jaka_adapter, E2IsolatedForwardTranslationGuard
                            ):
                                jaka_adapter.observe_measured_joint_position(sample)
                        tick = session.control_tick(now_ns)
                        dispatch_failed = _control_output_failed(
                            reason=tick.reason,
                            output_applied=tick.output_applied,
                        )
                        if dispatch_failed:
                            control_state = session.event_records[-1].get(
                                "control_state"
                            )
                            if control_state == "HARD_STOP":
                                abort_reason = f"shared_hard_stop:{tick.reason}"
                            elif tick.accepted_target is None:
                                abort_reason = "control_heartbeat_transport_failure"
                            else:
                                abort_reason = (
                                    jaka_adapter.abort_reason
                                    if isinstance(
                                        jaka_adapter,
                                        E2IsolatedForwardTranslationGuard,
                                    )
                                    and jaka_adapter.abort_reason is not None
                                    else "accepted_target_transport_failure"
                                )
                            stop_reason = abort_reason
                            jaka_adapter.stop()
                        engaged = session.arm_clutch.state.value == "engaged"
                        disengaged = prior_engaged and not engaged
                        if disengaged:
                            jaka_adapter.stop()
                            clutch_release_monotonic_ns = now_ns
                            stop_reason = session.arm_clutch.active_fault.reason if session.arm_clutch.active_fault else "operator_clutch_released"
                        prior_engaged = engaged
                        accepted += int(tick.accepted_target is not None and tick.output_applied)
                        event = dict(session.event_records[-1])
                        operator_delta = event.get("operator_delta")
                        if operator_delta is not None:
                            maximum_quest_displacement_m = max(
                                maximum_quest_displacement_m,
                                math.sqrt(sum(
                                    float(value) ** 2
                                    for value in operator_delta["translation_m"]
                                )),
                            )
                        if event.get("continuation_fraction") is not None:
                            minimum_continuation_fraction = min(
                                minimum_continuation_fraction,
                                float(event["continuation_fraction"]),
                            )
                        event.update(
                            physical_stage=args.stage,
                            measured_joint_position_rad=None if status is None else list(status.joint_position_rad),
                            accepted_endpoint_minus_measured_joint_rad=None if status is None or tick.accepted_target is None else [
                                command - measured
                                for command, measured in zip(
                                    tick.accepted_target.joint_position_rad,
                                    status.joint_position_rad,
                                    strict=True,
                                )
                            ],
                            command_timestamp_ns=None if status is None else status.command_monotonic_ns,
                            stop_or_abort_reason=(
                                stop_reason if disengaged or dispatch_failed else None
                            ),
                        )
                        log.write(json.dumps(event, sort_keys=True) + "\n")
                        if disengaged or dispatch_failed:
                            break
                        skipped = max(0, int((now - next_tick) * target_hz))
                        next_tick += (skipped + 1) / target_hz
                finally:
                    receiver.close()
        except KeyboardInterrupt:
            stop_reason = "operator_keyboard_stop"
            jaka_adapter.stop()
        finally:
            if not jaka_adapter.stopped:
                jaka_adapter.stop()
            if native.process is not None and native.process.poll() is None:
                try:
                    native.process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    native.stop()
            else:
                native.stop()
            runtime.close()

    metrics = json.loads(args.metrics.read_text(encoding="utf-8"))
    if args.event_extract is not None and args.native_telemetry is not None:
        _write_event_extract(
            args.native_telemetry,
            args.event_extract,
            metrics,
            clutch_release_monotonic_ns=clutch_release_monotonic_ns,
        )
    measured_tcp = _measured_tcp_motion(target_generator, measured_joint_samples)
    e2_guard = (
        jaka_adapter
        if isinstance(jaka_adapter, E2IsolatedForwardTranslationGuard)
        else None
    )
    if args.stage == "e2-isolated":
        e2_failures = []
        if e2_guard is None or e2_guard.baseline is None:
            e2_failures.append("no_fresh_engagement_target")
        if maximum_quest_displacement_m < 0.005:
            e2_failures.append("no_small_translation_observed")
        if stop_reason != "operator_clutch_released":
            e2_failures.append("clutch_release_not_observed")
        if metrics["error_code"] != 0 or metrics["cleanup_error_code"] != 0:
            e2_failures.append("native_or_cleanup_failure")
        if metrics["hard_timing_misses"] != 0:
            e2_failures.append("timing_hard_fault")
        if any(metrics["output_speed_boundary_rejections"]):
            e2_failures.append("output_speed_boundary_rejection")
        if abort_reason is not None:
            e2_failures.append(abort_reason)
    else:
        e2_failures = []
    summary = {
        "schema_version": "quest_jaka_physical_gate.v1",
        "stage": args.stage,
        "shared_config": str(args.config),
        "accepted_targets_dispatched": accepted,
        "adapter_dispatch_count": jaka_adapter.applied_count,
        "native_mode": metrics["mode"],
        "native_outcome": metrics["outcome"],
        "native_ik_calls": metrics["ik_calls"],
        "native_command_max_ns": metrics["statistics"]["command_write_duration"]["max_ns"],
        "tracking_difference_definition": "current_8ms_emitted_command_minus_same_cycle_measured_joint_shortest_valid_revolute_delta",
        "authoritative_tracking_metrics_source": "native_worker_cycle_telemetry",
        "active_controller_payload_operator_report": {
            "mass_kg": 0.8,
            "center_of_mass_mm": [9.289, 12.427, 36.961],
            "written_by_this_process": False,
        },
        "installation_operator_report": {"type": "upright", "x_deg": 0.0, "z_deg": 0.0, "written_by_this_process": False},
        "tcp_status_operator_report": {"tcp1_through_tcp10": "zero", "written_by_this_process": False},
        "controller_alarm_history_programmatically_available": False,
        "joint_specific_servo_alarm_code_programmatically_available": metrics.get("joint_specific_servo_alarm_code_available", False),
        "pre_adapter_target_source": "single_shared_immutable_AcceptedArmTarget",
        "maximum_pre_adapter_joint_difference_rad": 0.0,
        "maximum_pre_adapter_tcp_position_difference_m": 0.0,
        "maximum_pre_adapter_tcp_orientation_difference_rad": 0.0,
        "mujoco_plant_instantiated": False,
        "shared_continuation_enabled": session.continuation_enabled,
        "quest_receive_dropped": 0 if receiver is None else receiver.dropped,
        "stop_reason": stop_reason,
        "abort_reason": abort_reason,
        "rh56_commands": 0,
        "maximum_quest_displacement_m": maximum_quest_displacement_m,
        "minimum_continuation_fraction": minimum_continuation_fraction,
        "continuation_backtrack_count": session.continuation_backtrack_count,
        "ik_rejections": dict(sorted(session.rejections.items())),
        "measured_joint_fk_tcp_motion": measured_tcp,
        "e2_maximum_requested_tcp_displacement_m": (
            None if e2_guard is None else e2_guard.maximum_requested_tcp_displacement_m
        ),
        "e2_maximum_accepted_tcp_displacement_m": (
            None if e2_guard is None else e2_guard.maximum_accepted_tcp_displacement_m
        ),
        "e2_maximum_accepted_joint_displacement_rad": (
            None if e2_guard is None else e2_guard.maximum_accepted_joint_displacement_rad
        ),
        "e2_startup_alignment_difference_rad": (
            None if e2_guard is None else e2_guard.startup_alignment_difference_rad
        ),
        "e2_startup_alignment_tolerance_rad": (
            None if e2_guard is None else e2_guard.startup_alignment_tolerance_rad
        ),
        "e2_maximum_observed_startup_difference_rad": (
            None
            if e2_guard is None
            else e2_guard.maximum_observed_startup_difference_rad
        ),
        "e2_failures": e2_failures,
        "e2_pass": args.stage == "e2-isolated" and not e2_failures,
    }
    args.summary.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 2 if abort_reason is not None or e2_failures else 0


def _measured_tcp_motion(
    target_generator: SharedJakaTargetGenerator,
    samples: list[tuple[float, ...]],
) -> dict[str, object] | None:
    """Post-cleanup FK evidence only; never runs on the command-critical path."""

    if not samples:
        return None
    positions: list[tuple[float, float, float]] = []
    for joints in samples:
        target_generator.synchronize_authoritative_arm_joints(list(joints))
        positions.append(target_generator.current_tcp_pose.position_m)
    baseline = positions[0]
    deltas = [
        tuple(value - start for value, start in zip(position, baseline, strict=True))
        for position in positions
    ]
    maximum = max(deltas, key=lambda item: math.sqrt(sum(value * value for value in item)))
    return {
        "sample_count": len(samples),
        "maximum_displacement_vector_m": list(maximum),
        "maximum_displacement_norm_m": math.sqrt(sum(value * value for value in maximum)),
        "direction": "robot_base_negative_x" if maximum[0] < 0.0 else "robot_base_positive_x_or_zero",
    }


def _write_event_extract(
    telemetry_path: Path,
    output_path: Path,
    metrics: dict[str, object],
    *,
    clutch_release_monotonic_ns: int | None,
) -> None:
    """Write bounded native-cycle windows without touching the command path."""

    rows = [
        json.loads(line)
        for line in telemetry_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not rows:
        output_path.write_text("", encoding="utf-8")
        return
    focus: dict[str, int] = {}
    alarm_ns = int(metrics.get("controller_event_monotonic_ns", 0) or 0)
    if int(metrics.get("controller_alarm_events", 0) or 0) and alarm_ns:
        focus["controller_alarm"] = alarm_ns
    for joint_index, name in ((3, "maximum_j4_tracking_lag"), (5, "maximum_j6_tracking_lag")):
        row = max(
            rows,
            key=lambda item: abs(item["emitted_minus_measured_tracking_difference_rad"][joint_index]),
        )
        focus[name] = int(row["host_monotonic_ns"])
    acceleration_row = max(
        rows,
        key=lambda item: max(abs(value) for value in item["emitted_acceleration_rad_s2"][3:]),
    )
    focus["maximum_wrist_acceleration"] = int(acceleration_row["host_monotonic_ns"])
    if clutch_release_monotonic_ns is not None:
        focus["operator_clutch_release"] = clutch_release_monotonic_ns

    selected: dict[int, set[str]] = {}
    for name, timestamp_ns in focus.items():
        for index, row in enumerate(rows):
            delta = int(row["host_monotonic_ns"]) - timestamp_ns
            if -2_000_000_000 <= delta <= 1_000_000_000:
                selected.setdefault(index, set()).add(name)
    with output_path.open("x", encoding="utf-8") as output:
        for index in sorted(selected):
            output.write(json.dumps({
                "focus_events": sorted(selected[index]),
                "telemetry": rows[index],
            }, sort_keys=True) + "\n")


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    raise SystemExit(main())
