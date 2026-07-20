#!/usr/bin/env python3
"""Run T3 command shadow or an explicitly approved bounded T4 session."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

from teleoperation.contracts import Pose3D, SafetyAction
from teleoperation.input.replay import PoseStreamRecorder
from teleoperation.input.teledex import TeleDexAdapter, TeleDexPacketParser, TeleDexWebSocketServer
from teleoperation.processing.clutch import ClutchController, ClutchState
from teleoperation.processing.one_euro_se3 import OneEuroSE3Filter
from teleoperation.processing.pose_validator import PoseValidator
from teleoperation.processing.target_shaper import JerkLimitedPoseShaper
from teleoperation.runtime.arm_only import (
    ArmOnlyRuntime,
    BOUNDED_MOTION_DISPATCH_ACK,
    NativeWorkerProcess,
)
from teleoperation.runtime.teledex_arm import BoundedArmTeleoperationPipeline
from teleoperation.supervision import ArmSafetySupervisor, SafetyEnvelope
from teleoperation.teledex_config import load_bounded_teleop_config
from teleoperation.transforms.frame_mapping import RelativePoseMapper
from teleoperation.wire import LatestTargetPublisher, StatusFlags, WorkerStatusReceiver


SHADOW_ACK = "I_ACKNOWLEDGE_JAKA_COMMAND_SHADOW_NO_EDG"
MOTION_ACK = "I_ACKNOWLEDGE_BOUNDED_TELEDEX_JAKA_MOTION"
T4_APPROVAL = "I_APPROVE_T4_BOUNDED_TELEDEX_JAKA_MOTION"


def wait_for_initial_input(adapter: TeleDexAdapter, timeout_s: float) -> object:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snapshot = adapter.latest(now_ns=time.monotonic_ns())
        if snapshot is not None and snapshot.pose is not None and snapshot.pose.tracking_valid:
            return snapshot
        time.sleep(0.005)
    raise RuntimeError("no fresh valid TeleDex pose arrived before timeout")


def load_t2_receipt(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema_version") != "teledex_input_diagnostic.v1":
        raise RuntimeError("T2 receipt has the wrong schema")
    if not payload.get("connected_seen") or int(payload.get("valid_samples", 0)) <= 0:
        raise RuntimeError("T2 receipt does not prove a live valid TeleDex stream")
    if payload.get("robot_connection_opened") is not False or payload.get("commands_issued") != 0:
        raise RuntimeError("T2 receipt violated receive-only scope")
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("command-shadow", "bounded-t4"))
    parser.add_argument("--config", type=Path, default=Path("configs/teleoperation/teledex_jaka_arm_bounded.yaml"))
    parser.add_argument("--worker", type=Path, default=Path("build/jaka_servo_worker/jaka_servo_worker"))
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--edg-state-ip", default="192.168.71.19")
    parser.add_argument("--expected-tool-id", type=int, default=0)
    parser.add_argument("--expected-user-frame-id", type=int, default=0)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--port", type=int, default=8888)
    parser.add_argument("--metrics-file", type=Path, required=True)
    parser.add_argument("--summary-file", type=Path, required=True)
    parser.add_argument("--target-log", type=Path, required=True)
    parser.add_argument("--record-input", type=Path)
    parser.add_argument("--t2-receipt", type=Path, required=True)
    parser.add_argument("--acknowledgement", required=True)
    parser.add_argument("--allow-unconfirmed-source-semantics-for-no-motion-shadow", action="store_true")
    parser.add_argument("--execute-t4", action="store_true")
    parser.add_argument("--operator-approval", default="")
    parser.add_argument("--estop-accessible", action="store_true")
    parser.add_argument("--workspace-clear", action="store_true")
    parser.add_argument("--rh56-command-path-absent", action="store_true")
    args = parser.parse_args()
    if not 0.1 < args.duration_s <= 10.0:
        parser.error("session duration must be in (0.1, 10] seconds")
    config = load_bounded_teleop_config(args.config)
    t2 = load_t2_receipt(args.t2_receipt)
    is_motion = args.stage == "bounded-t4"
    expected_ack = MOTION_ACK if is_motion else SHADOW_ACK
    if args.acknowledgement != expected_ack:
        parser.error(f"exact acknowledgement required: {expected_ack}")
    if is_motion:
        if not args.execute_t4 or args.operator_approval != T4_APPROVAL:
            parser.error(f"T4 requires --execute-t4 and --operator-approval {T4_APPROVAL}")
        if not (args.estop_accessible and args.workspace_clear and args.rh56_command_path_absent):
            parser.error("T4 requires all physical safety confirmation flags")
        if not config.motion_authorized_by_configuration:
            parser.error("configuration calibration/source-semantics gates do not authorize motion")
    elif not config.source_semantics_confirmed and not args.allow_unconfirmed_source_semantics_for_no_motion_shadow:
        parser.error("unconfirmed source semantics require the explicit no-motion shadow override")

    adapter = TeleDexAdapter(
        parser=TeleDexPacketParser(),
        stale_after_ns=config.validation.hold_age_ns,
        source_frame_id=config.frames.source_frame_id,
    )
    recorder = None if args.record_input is None else PoseStreamRecorder(
        args.record_input,
        metadata={"stage": args.stage, "config_sha256": config.content_sha256},
    )
    identity = Pose3D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    mapper = RelativePoseMapper(
        config.frames,
        translation_scale=config.translation_scale,
        rotation_scale=config.rotation_scale,
    )
    clutch = ClutchController(mapper, poses_are_operator_frame=True)
    pipeline = BoundedArmTeleoperationPipeline(
        validator=PoseValidator(config.validation),
        clutch=clutch,
        measurement_filter=OneEuroSE3Filter(),
        shaper=JerkLimitedPoseShaper(config.cartesian_limits),
        safety=ArmSafetySupervisor(
            SafetyEnvelope(
                identity,
                config.cartesian_limits.workspace_half_extent_m,
                config.cartesian_limits.maximum_orientation_deviation_rad,
                config.maximum_session_ns,
            ),
            config.joint_limits,
        ),
        startup_tcp_relative_output=True,
    )

    with tempfile.TemporaryDirectory(prefix="jaka_teledex_") as directory:
        target_socket = Path(directory) / "target.sock"
        status_socket = Path(directory) / "status.sock"
        status_receiver = WorkerStatusReceiver(status_socket)
        publisher = LatestTargetPublisher(target_socket)
        arm_runtime = ArmOnlyRuntime(publisher, status_receiver)
        worker_mode = "bounded-teleop" if is_motion else "command-shadow"
        worker_arguments = [
            "--mode", worker_mode,
            "--hardware",
            "--robot-ip", args.robot_ip,
            "--edg-state-ip", args.edg_state_ip,
            "--expected-tool-id", str(args.expected_tool_id),
            "--expected-user-frame-id", str(args.expected_user_frame_id),
            "--duration-s", str(args.duration_s),
            "--target-socket", str(target_socket),
            "--status-socket", str(status_socket),
            "--metrics-file", str(args.metrics_file),
            "--acknowledgement", expected_ack,
            "--warning-ms", str(config.validation.warning_age_ns / 1e6),
            "--hold-ms", str(config.validation.hold_age_ns / 1e6),
            "--controlled-stop-ms", str(config.validation.controlled_stop_age_ns / 1e6),
            "--fatal-timeout-ms", str(config.validation.fatal_age_ns / 1e6),
            "--relative-translation-limit-m", str(max(config.cartesian_limits.workspace_half_extent_m)),
            "--relative-rotation-limit-rad", str(config.cartesian_limits.maximum_orientation_deviation_rad),
            "--joint-velocity-limit-rad-s", str(config.joint_limits.maximum_velocity_rad_s),
            "--joint-acceleration-limit-rad-s2", str(config.joint_limits.maximum_acceleration_rad_s2),
            "--joint-jerk-limit-rad-s3", str(config.joint_limits.maximum_jerk_rad_s3),
            "--joint-soft-margin-rad", str(config.joint_limits.soft_margin_rad),
        ]
        native = NativeWorkerProcess(args.worker, worker_arguments)
        generation = -1
        dispatched = 0
        stop_sent = False
        edg_seen = False
        connected_seen = False
        prior_clutch = clutch.state
        operator_outcome = "duration_or_clutch_stop"
        started_ns = time.monotonic_ns()
        try:
            with TeleDexWebSocketServer(adapter, port=args.port):
                initial_snapshot = wait_for_initial_input(adapter, timeout_s=60.0)
                if recorder is not None:
                    recorder.write(initial_snapshot)  # type: ignore[arg-type]
                generation = initial_snapshot.generation  # type: ignore[attr-defined]
                print("Fresh TeleDex stream validated. Connecting the no-hand JAKA worker.")
                if is_motion:
                    for remaining in range(5, 0, -1):
                        print(f"Bounded T4 starts in {remaining}...")
                        time.sleep(1.0)
                native.start()
                worker_deadline = time.monotonic() + 5.0
                while time.monotonic() < worker_deadline:
                    status = arm_runtime.latest_status()
                    if status is not None:
                        connected_seen = bool(StatusFlags(status.flags) & StatusFlags.CONNECTED)
                        if connected_seen:
                            break
                    if native.process is not None and native.process.poll() is not None:
                        raise RuntimeError("native worker exited during JAKA startup")
                    time.sleep(0.005)
                if not connected_seen:
                    raise RuntimeError("native worker did not report connected state")

                # Feed the already validated sample so a released Button A can
                # satisfy the mandatory startup release edge.
                pending = initial_snapshot
                session_deadline = time.monotonic_ns() + int(args.duration_s * 1e9)
                with args.target_log.open("x", encoding="utf-8") as target_log:
                    while time.monotonic_ns() < session_deadline:
                        now_ns = time.monotonic_ns()
                        snapshot = pending
                        pending = None
                        if snapshot is None:
                            snapshot = adapter.latest(now_ns=now_ns, after_generation=generation)
                        if snapshot is None:
                            status = arm_runtime.latest_status()
                            if status is not None:
                                edg_seen = edg_seen or bool(StatusFlags(status.flags) & StatusFlags.EDG_ACTIVE)
                            if native.process is not None and native.process.poll() is not None:
                                break
                            time.sleep(0.001)
                            continue
                        generation = snapshot.generation
                        if recorder is not None:
                            recorder.write(snapshot)
                        action = snapshot.operator_action
                        if action is not None and action.recenter_requested:
                            # First bounded sessions recenter by ending this
                            # disposable worker. Relaunch captures a fresh robot
                            # TCP and requires a fresh Button-A edge.
                            stop_sent = arm_runtime.dispatch_stop(
                                sequence=snapshot.run_gate.sequence + 1
                            )
                            operator_outcome = "recenter_requested_session_restart_required"
                            break
                        if action is not None and action.stop_requested:
                            stop_sent = arm_runtime.dispatch_stop(
                                sequence=snapshot.run_gate.sequence + 1
                            )
                            operator_outcome = "operator_stop_requested"
                            break
                        result = pipeline.process(snapshot, robot_tcp_pose=identity, now_ns=now_ns)
                        target_log.write(json.dumps({
                            "generation": generation,
                            "validation": result.validation.action.value,
                            "clutch": None if result.clutch is None else result.clutch.state.value,
                            "safety": result.safety.action.value,
                            "reason": result.reason,
                            "target": None if result.target is None else result.target.to_dict(),
                        }, separators=(",", ":"), sort_keys=True) + "\n")
                        current_clutch = clutch.state
                        if result.target is not None and result.safety.action == SafetyAction.ALLOW:
                            sent = (
                                arm_runtime.dispatch_authorized(
                                    result.target,
                                    acknowledgement=BOUNDED_MOTION_DISPATCH_ACK,
                                )
                                if is_motion
                                else arm_runtime.dispatch(result.target)
                            )
                            dispatched += int(sent)
                        if prior_clutch == ClutchState.ACTIVE and current_clutch != ClutchState.ACTIVE:
                            stop_sent = arm_runtime.dispatch_stop(sequence=snapshot.run_gate.sequence + 1)
                            break
                        if result.safety.action in {SafetyAction.CONTROLLED_STOP, SafetyAction.ABORT}:
                            stop_sent = arm_runtime.dispatch_stop(sequence=snapshot.run_gate.sequence + 1)
                            break
                        prior_clutch = current_clutch
                        status = arm_runtime.latest_status()
                        if status is not None:
                            edg_seen = edg_seen or bool(StatusFlags(status.flags) & StatusFlags.EDG_ACTIVE)
        except KeyboardInterrupt:
            stop_sent = arm_runtime.dispatch_stop(sequence=max(1, generation + 2))
        finally:
            if not stop_sent and native.process is not None and native.process.poll() is None:
                arm_runtime.dispatch_stop(sequence=max(1, generation + 2))
                stop_sent = True
            if stop_sent and native.process is not None and native.process.poll() is None:
                try:
                    native.process.wait(timeout=1.5)
                except subprocess.TimeoutExpired:
                    pass
            worker_return = native.stop()
            pipeline.stop()
            if recorder is not None:
                recorder.close()
            arm_runtime.close()

    metrics = json.loads(args.metrics_file.read_text(encoding="utf-8"))
    summary = {
        "schema_version": "teledex_jaka_session.v1",
        "stage": "T4" if is_motion else "T3",
        "config_sha256": config.content_sha256,
        "t2_receipt": str(args.t2_receipt),
        "t2_valid_samples": t2["valid_samples"],
        "worker_return_code": worker_return,
        "jaka_connected_seen": connected_seen,
        "targets_dispatched": dispatched,
        "edg_seen": edg_seen,
        "worker_mode": metrics.get("mode"),
        "worker_outcome": metrics.get("outcome"),
        "ik_calls": metrics.get("ik_calls"),
        "maximum_intentional_command_delta_rad": metrics.get("maximum_intentional_command_delta_rad"),
        "command_write_max_ns": metrics["statistics"]["command_write_duration"]["max_ns"],
        "rh56_loaded": False,
        "operator_outcome": operator_outcome,
        "elapsed_s": (time.monotonic_ns() - started_ns) / 1e9,
    }
    if not is_motion:
        if edg_seen or summary["maximum_intentional_command_delta_rad"] != 0 or summary["command_write_max_ns"] != 0:
            raise RuntimeError("T3 command-shadow invariant violated: EDG/command activity detected")
    args.summary_file.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if worker_return == 0 and dispatched > 0 and int(summary["ik_calls"] or 0) > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
