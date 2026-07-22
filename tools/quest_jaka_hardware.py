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
from teleoperation.jaka.quest_adapter import JakaAcceptedJointTargetAdapter
from teleoperation.runtime.arm_only import ArmOnlyRuntime, NativeWorkerProcess
from teleoperation.wire import LatestTargetPublisher, StatusFlags, WorkerStatusReceiver


P2_APPROVAL = "I_AUTHORIZE_P2_QUEST_JAKA_COMMAND_SHADOW"
P4_APPROVAL = "I_AUTHORIZE_P4_LIVE_QUEST_JAKA_TELEOPERATION"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("p2-shadow", "p4-live"))
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


def main() -> int:
    args = _parser().parse_args()
    if args.duration_sec <= 0.0:
        raise SystemExit("duration must be positive")
    config = replace(ReplayConfig.load(args.config), engagement_schedule_s=())
    hardware = config.raw["hardware_adapter"]
    live = args.stage == "p4-live"
    expected_approval = P4_APPROVAL if live else P2_APPROVAL
    if args.approval != expected_approval:
        raise SystemExit(f"exact approval required: {expected_approval}")
    if live:
        if not bool(hardware["physical_mapping_confirmed"]):
            raise SystemExit("P4 blocked: physical Quest/JAKA mapping has not been confirmed after P1/P2")
        if not (args.estop_accessible and args.workspace_clear and args.rh56_command_path_absent):
            raise SystemExit("P4 requires E-stop, clear-workspace, and no-RH56-command confirmations")
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

    with tempfile.TemporaryDirectory(prefix="quest_jaka_hardware_") as directory:
        temporary = Path(directory)
        target_generator = SharedJakaTargetGenerator(config, mjcf_path=config.mjcf_path)
        target_socket = temporary / "target.sock"
        status_socket = temporary / "status.sock"
        runtime = ArmOnlyRuntime(
            LatestTargetPublisher(target_socket),
            WorkerStatusReceiver(status_socket),
        )
        jaka_adapter = JakaAcceptedJointTargetAdapter(
            runtime,
            allow_motion=live,
            joint_order=tuple(hardware["joint_order"]),
            joint_angle_unit=str(hardware["joint_angle_unit"]),
            command_mode=str(hardware["command_mode"]),
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
            "--acknowledgement", expected_approval,
            "--warning-ms", str(hardware["command_stream_warning_ms"]),
            "--hold-ms", str(hardware["command_stream_timeout_ms"]),
            "--controlled-stop-ms", str(hardware["controlled_stop_timeout_ms"]),
            "--fatal-timeout-ms", str(hardware["fatal_communication_timeout_ms"]),
            "--excessive-tracking-error-abort-rad", str(hardware["excessive_tracking_error_abort_rad"]),
            "--excessive-tracking-error-consecutive-cycles", str(hardware["excessive_tracking_error_consecutive_cycles"]),
            "--startup-alignment-tolerance-rad", str(hardware["startup_alignment_tolerance_rad"]),
        ]
        native = NativeWorkerProcess(args.worker, worker_args)
        accepted = 0
        stop_reason = "duration_complete"
        prior_engaged = False
        started = time.monotonic()
        next_tick = started
        status = None
        receiver: QuestDatagramReceiverWorker | None = None
        native.start()
        try:
            status = _wait_status(runtime, native)
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
                try:
                    while time.monotonic() - started < args.duration_sec:
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
                        engaged_before_tick = session.arm_clutch.state.value == "engaged"
                        if not engaged_before_tick and status is not None:
                            target_generator.synchronize_authoritative_arm_joints(
                                list(status.joint_position_rad)
                            )
                        tick = session.control_tick(now_ns)
                        engaged = session.arm_clutch.state.value == "engaged"
                        disengaged = prior_engaged and not engaged
                        if disengaged:
                            jaka_adapter.stop()
                            stop_reason = session.arm_clutch.active_fault.reason if session.arm_clutch.active_fault else "operator_clutch_released"
                        prior_engaged = engaged
                        accepted += int(tick.accepted_target is not None and tick.output_applied)
                        event = dict(session.event_records[-1])
                        event.update(
                            physical_stage=args.stage,
                            measured_joint_position_rad=None if status is None else list(status.joint_position_rad),
                            joint_tracking_error_rad=None if status is None or tick.accepted_target is None else [
                                command - measured
                                for command, measured in zip(
                                    tick.accepted_target.joint_position_rad,
                                    status.joint_position_rad,
                                    strict=True,
                                )
                            ],
                            command_timestamp_ns=None if status is None else status.command_monotonic_ns,
                            stop_or_abort_reason=stop_reason if disengaged else None,
                        )
                        log.write(json.dumps(event, sort_keys=True) + "\n")
                        if disengaged:
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
            native.stop()
            runtime.close()

    metrics = json.loads(args.metrics.read_text(encoding="utf-8"))
    summary = {
        "schema_version": "quest_jaka_physical_gate.v1",
        "stage": args.stage,
        "shared_config": str(args.config),
        "accepted_targets_dispatched": accepted,
        "adapter_dispatch_count": jaka_adapter.applied_count,
        "native_mode": metrics["mode"],
        "native_ik_calls": metrics["ik_calls"],
        "native_command_max_ns": metrics["statistics"]["command_write_duration"]["max_ns"],
        "pre_adapter_target_source": "single_shared_immutable_AcceptedArmTarget",
        "maximum_pre_adapter_joint_difference_rad": 0.0,
        "maximum_pre_adapter_tcp_position_difference_m": 0.0,
        "maximum_pre_adapter_tcp_orientation_difference_rad": 0.0,
        "mujoco_plant_instantiated": False,
        "shared_continuation_enabled": session.continuation_enabled,
        "quest_receive_dropped": 0 if receiver is None else receiver.dropped,
        "stop_reason": stop_reason,
        "rh56_commands": 0,
    }
    args.summary.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    raise SystemExit(main())
