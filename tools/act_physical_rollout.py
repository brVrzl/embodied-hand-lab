#!/usr/bin/env python3
"""Run one bounded physical ACT baseline rollout.

This entry point deliberately reuses the command-disabled ACT model worker,
the existing JAKA native worker/adapter, the production RH56 PC-direct worker,
and the existing RealSense reader.  It adds no policy-side grasp, waypoint, or
force logic.  The policy emits the recorded absolute 12-D action directly.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import json
from pathlib import Path
import queue
import signal
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any

import cv2
import numpy as np

from act_live_shadow import CameraReader, CameraSample, _model_request, preprocess_live_rgb
from embodiment_core.config import load_yaml
from quest_jaka_sim import ReplayConfig
from quest_jaka_hardware import (
    COMBINED_CONTROL_REALTIME_PRIORITY,
    _configure_cpu_isolation,
    _native_velocity_limit_args,
    _require_realtime_priority_limit,
    _resolve_output_jerk_limit,
    _validate_control_cpu,
    _wait_status,
)
from rh56_driver.pc_direct_control import (
    HandOperation,
    RH56PcDirectControl,
    inspect_serial_device,
    require_serial_by_id_path,
)
from rh56_driver.pc_direct_worker import PcDirectDatasetFeedback, RH56PcDirectWorker
from rh56_driver.serial_backend import RH56SerialBackend
from teleoperation.jaka.quest_adapter import JakaAcceptedJointTargetAdapter
from teleoperation.runtime.arm_only import ArmOnlyRuntime, NativeWorkerProcess
from teleoperation.wire import (
    LatestTargetPublisher,
    StatusFlags,
    WorkerStatusPacket,
    WorkerStatusReceiver,
)


MODEL_WORKER = Path(__file__).with_name("act_shadow_model_worker.py")
ENVIRONMENT_STATE_KEY = "observation.environment_state"
JAKA_LOWER = np.asarray([-6.28, -2.09, -2.27, -6.28, -2.09, -6.28], dtype=np.float64)
JAKA_UPPER = np.asarray([6.28, 2.09, 2.27, 6.28, 2.09, 6.28], dtype=np.float64)
RH56_CHANNEL_NAMES = (
    "index",
    "middle",
    "ring",
    "pinky",
    "thumb_close",
    "thumb_lateral",
)


@dataclass(frozen=True)
class Observation:
    workspace: CameraSample
    wrist: CameraSample
    status: WorkerStatusPacket
    feedback: PcDirectDatasetFeedback
    state: np.ndarray
    force: np.ndarray
    ready_ns: int


@dataclass(frozen=True)
class Prediction:
    sequence: int
    query_start_ns: int
    query_end_ns: int
    observation: Observation
    chunk: np.ndarray
    timing_ms: dict[str, float]


class AsyncRolloutWriter:
    """Bounded persistence sink; no video or JSON I/O runs in the command loop."""

    def __init__(self, root: Path, *, capacity: int = 512) -> None:
        self.root = root
        self.root.mkdir(parents=True, exist_ok=False)
        self._queue: queue.Queue[tuple[str, Any] | None] = queue.Queue(maxsize=capacity)
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._run, name="act-rollout-writer", daemon=True)
        self.error: str | None = None
        self.command_count = 0

    def start(self) -> None:
        self._thread.start()
        if not self._ready.wait(2.0):
            raise RuntimeError("rollout writer did not initialize")
        if self.error is not None:
            raise RuntimeError(self.error)

    def _run(self) -> None:
        command_file = None
        query_file = None
        workspace_writer = None
        wrist_writer = None
        try:
            command_file = (self.root / "commands.jsonl").open("x", encoding="utf-8")
            query_file = (self.root / "queries.jsonl").open("x", encoding="utf-8")
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            workspace_writer = cv2.VideoWriter(
                str(self.root / "workspace.mp4"), fourcc, 30.0, (640, 480)
            )
            wrist_writer = cv2.VideoWriter(
                str(self.root / "wrist.mp4"), fourcc, 30.0, (640, 480)
            )
            if not workspace_writer.isOpened() or not wrist_writer.isOpened():
                raise RuntimeError("could not open rollout video writers")
            self._ready.set()
            while True:
                item = self._queue.get()
                if item is None:
                    break
                kind, value = item
                if kind == "query":
                    query_file.write(json.dumps(value, separators=(",", ":")) + "\n")
                elif kind == "command":
                    row, workspace_rgb, wrist_rgb = value
                    command_file.write(json.dumps(row, separators=(",", ":")) + "\n")
                    workspace_writer.write(cv2.cvtColor(workspace_rgb, cv2.COLOR_RGB2BGR))
                    wrist_writer.write(cv2.cvtColor(wrist_rgb, cv2.COLOR_RGB2BGR))
                    self.command_count += 1
                else:
                    raise RuntimeError(f"unknown rollout writer item {kind!r}")
            command_file.flush()
            query_file.flush()
        except BaseException as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self._ready.set()
        finally:
            if command_file is not None:
                command_file.close()
            if query_file is not None:
                query_file.close()
            if workspace_writer is not None:
                workspace_writer.release()
            if wrist_writer is not None:
                wrist_writer.release()

    def submit(self, kind: str, value: Any) -> None:
        if self.error is not None:
            raise RuntimeError(self.error)
        try:
            self._queue.put_nowait((kind, value))
        except queue.Full as exc:
            raise RuntimeError("rollout recorder queue full") from exc

    def finish(self) -> None:
        self._queue.put(None, timeout=5.0)
        self._thread.join(timeout=8.0)
        if self._thread.is_alive():
            raise RuntimeError("rollout writer did not stop")
        if self.error is not None:
            raise RuntimeError(self.error)


class ModelWorker:
    def __init__(self, checkpoint: Path, root: Path, *, container_image: str | None = None) -> None:
        self.checkpoint = checkpoint
        self.root = root
        self.container_image = container_image
        self.socket = root / "model.sock"
        self.ready = root / "model_ready"
        self.summary = root / "model_summary.json"
        config_path = checkpoint / "config.json"
        if not config_path.is_file():
            raise FileNotFoundError(f"ACT checkpoint config is missing: {config_path}")
        checkpoint_config = json.loads(config_path.read_text(encoding="utf-8"))
        input_features = checkpoint_config.get("input_features", {})
        self.requires_environment_state = ENVIRONMENT_STATE_KEY in input_features
        self.process: subprocess.Popen[str] | None = None
        self.connection = None

    def start(self) -> None:
        if self.container_image is None:
            command = [str(Path(sys.executable)), str(MODEL_WORKER)]
            checkpoint = self.checkpoint
        else:
            repo_root = Path(__file__).resolve().parents[1]
            try:
                relative_checkpoint = self.checkpoint.relative_to(repo_root)
            except ValueError as exc:
                raise ValueError(
                    "containerized ACT inference requires checkpoint under the repository root"
                ) from exc
            checkpoint = Path("/workspace/embodied_lab") / relative_checkpoint
            command = [
                "docker", "run", "--rm", "--runtime=nvidia", "--ipc=host",
                "--network=none",
                "-v", f"{repo_root}:/workspace/embodied_lab:ro",
                "-v", f"{self.root}:{self.root}",
                "-w", "/workspace/embodied_lab",
                self.container_image,
                "python", str(Path("/workspace/embodied_lab") / MODEL_WORKER.relative_to(repo_root)),
            ]
        self.process = subprocess.Popen(
            [
                *command,
                "--checkpoint", str(checkpoint),
                "--socket", str(self.socket),
                "--ready-file", str(self.ready),
                "--summary", str(self.summary),
            ],
            text=True,
        )
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError(f"ACT model worker exited {self.process.returncode}")
            if self.ready.exists():
                break
            time.sleep(0.02)
        else:
            raise TimeoutError("ACT model worker did not become ready")
        import socket

        self.connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.connection.connect(str(self.socket))

    def request(self, observation: dict[str, np.ndarray]) -> dict[str, Any]:
        if self.connection is None:
            raise RuntimeError("ACT model worker is not connected")
        response = _model_request(self.connection, observation)
        if "error" in response:
            raise RuntimeError(str(response["error"]))
        return response

    def warmup(self, count: int = 2) -> None:
        """Warm CUDA/processor kernels before any physical command is enabled."""

        observation = {
            "observation.images.workspace": np.zeros((3, 240, 320), dtype=np.float32),
            "observation.images.wrist": np.zeros((3, 240, 320), dtype=np.float32),
            "observation.state": np.zeros((12,), dtype=np.float32),
        }
        if self.requires_environment_state:
            observation[ENVIRONMENT_STATE_KEY] = np.zeros((6,), dtype=np.float32)
        for _ in range(count):
            response = self.request(observation)
            prediction = np.asarray(response.get("prediction"))
            if prediction.shape != (16, 12) or not np.isfinite(prediction).all():
                raise RuntimeError("ACT warmup returned an invalid [16,12] prediction")

    def stop(self) -> None:
        if self.connection is not None:
            try:
                _model_request(self.connection, {"command": "stop"})
            except BaseException:
                pass
            self.connection.close()
            self.connection = None
        if self.process is not None:
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGTERM)
                try:
                    self.process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2.0)


class InferenceThread:
    def __init__(self, model: ModelWorker, cameras: tuple[CameraReader, CameraReader], runtime: ArmOnlyRuntime, rh56: RH56PcDirectWorker, writer: AsyncRolloutWriter, *, initial_status: WorkerStatusPacket, rate_hz: float, max_age_ms: float) -> None:
        self.model = model
        self.workspace, self.wrist = cameras
        self.runtime = runtime
        self.rh56 = rh56
        self.writer = writer
        self._last_status = initial_status
        self.period_ns = int(round(1e9 / rate_hz))
        self.max_age_ns = int(round(max_age_ms * 1e6))
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name="act-inference", daemon=True)
        self._lock = threading.Lock()
        self.latest: Prediction | None = None
        self.error: str | None = None
        self.query_count = 0
        self.skipped_deadlines = 0
        self._next_sequence = 1

    def start(self) -> None:
        self.thread.start()

    def snapshot(self) -> Observation:
        workspace = self.workspace.latest()
        wrist = self.wrist.latest()
        status = self.runtime.latest_status()
        if status is not None:
            self._last_status = status
        status = self._last_status
        feedback = self.rh56.latest_dataset_feedback
        if feedback is None:
            raise RuntimeError("RH56 feedback is not available")
        # Take the reference time after the independent cache reads.  A newer
        # source sample may arrive while this snapshot is being assembled; it
        # must be checked against the completion time of the snapshot rather
        # than an earlier time captured before those reads.
        now_ns = time.monotonic_ns()
        flags = StatusFlags(status.flags)
        if not (flags & StatusFlags.CONNECTED) or not (flags & StatusFlags.EDG_ACTIVE):
            raise RuntimeError(f"JAKA worker is not live: flags={int(status.flags)}")
        if status.error_code:
            raise RuntimeError(f"JAKA worker reported error code {status.error_code}")
        source_times = (
            workspace.host_monotonic_ns,
            wrist.host_monotonic_ns,
            status.observation_monotonic_ns,
            feedback.feedback.monotonic_ns,
        )
        register_times = tuple(
            timestamp
            for timestamp in (
                feedback.angle_act_timestamp_ns,
                feedback.force_act_timestamp_ns,
            )
            if timestamp is not None
        )
        if any(timestamp > now_ns for timestamp in (*source_times, *register_times)):
            raise RuntimeError("future host timestamp in live observation")
        ages = [now_ns - timestamp for timestamp in source_times]
        if max(ages) > self.max_age_ns:
            raise RuntimeError(f"live observation source is stale: {max(ages) / 1e6:.1f} ms")
        state = np.asarray(
            (*status.joint_position_rad, *feedback.feedback.position_normalized),
            dtype=np.float32,
        )
        force = np.asarray(feedback.feedback.load_or_force_raw_count, dtype=np.float32)
        if state.shape != (12,) or force.shape != (6,) or not np.isfinite(state).all() or not np.isfinite(force).all():
            raise RuntimeError("live JAKA/RH56 state is not finite with expected shape")
        return Observation(workspace, wrist, status, feedback, state, force, now_ns)

    def _run(self) -> None:
        next_due_ns = time.monotonic_ns()
        try:
            while not self.stop_event.is_set():
                now_ns = time.monotonic_ns()
                if now_ns < next_due_ns:
                    self.stop_event.wait((next_due_ns - now_ns) / 1e9)
                    if self.stop_event.is_set():
                        break
                query_start_ns = time.monotonic_ns()
                observation = self.snapshot()
                request = {
                    "observation.images.workspace": preprocess_live_rgb(observation.workspace.rgb),
                    "observation.images.wrist": preprocess_live_rgb(observation.wrist.rgb),
                    "observation.state": observation.state,
                }
                if self.model.requires_environment_state:
                    request[ENVIRONMENT_STATE_KEY] = observation.force
                response = self.model.request(request)
                query_end_ns = time.monotonic_ns()
                chunk = np.asarray(response["prediction"], dtype=np.float32)
                if chunk.shape != (16, 12) or not np.isfinite(chunk).all():
                    raise RuntimeError(f"ACT output must be finite [16,12], got {chunk.shape}")
                prediction = Prediction(
                    self._next_sequence,
                    query_start_ns,
                    query_end_ns,
                    observation,
                    chunk,
                    {key: float(value) for key, value in response["timing_ms"].items()},
                )
                self._next_sequence += 1
                with self._lock:
                    self.latest = prediction
                self.writer.submit(
                    "query",
                    {
                        "query_sequence": prediction.sequence,
                        "query_start_ns": query_start_ns,
                        "query_end_ns": query_end_ns,
                        "observation_ready_ns": observation.ready_ns,
                        "workspace_frame_number": observation.workspace.frame_number,
                        "wrist_frame_number": observation.wrist.frame_number,
                        "workspace_host_ns": observation.workspace.host_monotonic_ns,
                        "wrist_host_ns": observation.wrist.host_monotonic_ns,
                        "jaka_observation_ns": observation.status.observation_monotonic_ns,
                        "rh56_angle_ns": observation.feedback.angle_act_timestamp_ns,
                        "rh56_force_ns": observation.feedback.force_act_timestamp_ns,
                        "force": observation.force.tolist(),
                        "timing_ms": prediction.timing_ms,
                    },
                )
                self.query_count += 1
                next_due_ns += self.period_ns
                now_ns = time.monotonic_ns()
                if next_due_ns <= now_ns:
                    self.skipped_deadlines += max(1, (now_ns - next_due_ns) // self.period_ns + 1)
                    next_due_ns = now_ns + self.period_ns
        except BaseException as exc:
            self.error = f"{type(exc).__name__}: {exc}"

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=5.0)
        if self.thread.is_alive():
            raise RuntimeError("ACT inference thread did not stop")

    def get_latest(self) -> Prediction | None:
        with self._lock:
            return self.latest


def _runtime_values(path: Path) -> tuple[dict[str, Any], ReplayConfig]:
    document = load_yaml(path)
    runtime = document.get("runtime", document)
    if not isinstance(runtime, dict):
        raise ValueError("runtime config must contain a mapping")
    config_path = Path(str(runtime["config"]))
    config = ReplayConfig.load(config_path)
    velocity = tuple(float(value) for value in runtime["run_output_joint_velocity_limits_rad_s"])
    if len(velocity) != 6:
        raise ValueError("run_output_joint_velocity_limits_rad_s must contain six values")
    config = replace(
        config,
        output_contract=replace(
            config.output_contract,
            maximum_velocity_rad_s_per_joint=velocity,
        ),
    )
    return runtime, config


def _native_arguments(runtime: dict[str, Any], config: ReplayConfig, target_socket: Path, status_socket: Path, metrics: Path, duration_sec: float) -> list[str]:
    hardware = config.raw["hardware_adapter"]
    args = [
        "--mode", "joint-teleop", "--hardware",
        "--robot-ip", str(runtime["robot_ip"]),
        "--edg-state-ip", str(runtime["edg_state_ip"]),
        "--duration-s", str(duration_sec + 2.0),
        "--target-socket", str(target_socket),
        "--status-socket", str(status_socket),
        "--metrics-file", str(metrics),
        "--expected-tool-id", str(hardware["expected_tool_id"]),
        "--expected-user-frame-id", str(hardware["expected_user_frame_id"]),
        "--servo-step-num", str(hardware.get("servo_step_num", 1)),
        "--warning-ms", str(hardware["command_stream_warning_ms"]),
        "--hold-ms", str(hardware["command_stream_timeout_ms"]),
        "--controlled-stop-ms", str(hardware["controlled_stop_timeout_ms"]),
        "--fatal-timeout-ms", str(hardware["fatal_communication_timeout_ms"]),
        "--excessive-tracking-error-abort-rad", str(hardware["excessive_tracking_error_abort_rad"]),
        "--excessive-tracking-error-consecutive-cycles", str(hardware["excessive_tracking_error_consecutive_cycles"]),
        "--startup-alignment-tolerance-rad", str(hardware["startup_alignment_tolerance_rad"]),
        "--diagnostic-joint-acceleration-boundary-rad-s2", str(config.output_contract.maximum_acceleration_rad_s2),
        "--maximum-output-joint-acceleration-rad-s2", str(config.output_contract.maximum_acceleration_rad_s2),
        "--output-joint-jerk-limit-rad-s3", str(_resolve_output_jerk_limit(config)),
        "--output-acceleration-hold-degraded-ms", str(hardware["output_acceleration_hold_degraded_ms"]),
        "--output-acceleration-hold-hard-stop-ms", str(hardware["output_acceleration_hold_hard_stop_ms"]),
        "--maximum-consecutive-output-acceleration-hold-cycles", str(hardware["maximum_consecutive_output_acceleration_hold_cycles"]),
        "--startup-timing-grace-cycles", str(config.startup_timing_grace_cycles),
    ]
    args.extend(_native_velocity_limit_args(config))
    args.extend(("--monitor-controller-health-each-cycle", "--status-every-cycles", "4", "--recover-output-acceleration-transition"))
    args.extend(("--control-cpu", str(runtime["native_control_cpu"]), "--control-realtime-priority", str(runtime["native_control_realtime_priority"])))
    return args


def _assert_policy_action(action: np.ndarray) -> None:
    if action.shape != (12,) or not np.isfinite(action).all():
        raise RuntimeError("ACT action is not finite [12]")
    if np.any(action[:6] < JAKA_LOWER) or np.any(action[:6] > JAKA_UPPER):
        raise RuntimeError("ACT JAKA action exceeds native manufacturer position envelope")
    if np.any(action[6:] < 0.0) or np.any(action[6:] > 1.0):
        raise RuntimeError("ACT RH56 action is outside normalized [0,1] command domain")


def _project_rh56_command(
    action: np.ndarray,
    *,
    legal_min: float,
    legal_max: float,
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    """Project only the policy RH56 command at the actuator boundary.

    The model output and saved ``act_predictions.npz`` remain untouched.  The
    returned action is the value submitted to the existing RH56 worker; its
    feedback freshness, contact stop, delta, raw-count, and serial safety
    checks still run unchanged downstream.
    """

    projected = np.asarray(action, dtype=np.float64).copy()
    raw_rh56 = projected[6:].copy()
    projected_rh56 = np.clip(raw_rh56, float(legal_min), float(legal_max))
    projected[6:] = projected_rh56
    events = [
        {
            "channel": channel,
            "channel_index": index,
            "raw_value": float(raw_value),
            "projected_value": float(projected_value),
            "delta": float(projected_value - raw_value),
        }
        for index, (channel, raw_value, projected_value) in enumerate(
            zip(RH56_CHANNEL_NAMES, raw_rh56, projected_rh56, strict=True),
            start=6,
        )
        if float(raw_value) != float(projected_value)
    ]
    return projected, events


def _command_status_sample(
    last_policy: Prediction | None,
    startup_status: WorkerStatusPacket,
) -> WorkerStatusPacket:
    """Use the status snapshot already consumed by the inference thread.

    ``WorkerStatusReceiver`` is a bounded latest-only datagram receiver.  The
    command loop must not drain it again just to annotate a command row, or it
    can starve ``InferenceThread.snapshot`` and make the policy reuse an old
    JAKA observation.
    """

    return startup_status if last_policy is None else last_policy.observation.status


def run(args: argparse.Namespace) -> dict[str, Any]:
    runtime, config = _runtime_values(args.runtime_config)
    if int(runtime["native_control_realtime_priority"]) != COMBINED_CONTROL_REALTIME_PRIORITY:
        raise RuntimeError("physical ACT rollout requires the validated native realtime priority 10")
    _require_realtime_priority_limit(int(runtime["native_control_realtime_priority"]))
    control_cpu = int(runtime["native_control_cpu"])
    _validate_control_cpu(control_cpu)
    if args.duration_sec <= 0.0 or args.duration_sec > 60.0:
        raise ValueError("duration_sec must be within (0,60]")
    output = args.output.resolve()
    writer = AsyncRolloutWriter(output)
    model = ModelWorker(
        args.checkpoint.resolve(), output, container_image=args.model_container
    )
    native: NativeWorkerProcess | None = None
    runtime_arm: ArmOnlyRuntime | None = None
    jaka_adapter: JakaAcceptedJointTargetAdapter | None = None
    rh56_worker: RH56PcDirectWorker | None = None
    cameras: list[CameraReader] = []
    inference: InferenceThread | None = None
    predictions: list[np.ndarray] = []
    projection_events: list[dict[str, Any]] = []
    rh56_legal_min = 0.0
    rh56_legal_max = 1.0
    abort_reason: str | None = None
    command_count = 0
    next_command_ns = 0
    last_policy: Prediction | None = None
    policy_index = 2
    policy_consumed_sequence = 0
    startup_sent = False
    try:
        # Match the maintained combined physical launcher: isolate Python
        # before creating the model, serial, camera, or native-worker tasks.
        _configure_cpu_isolation(control_cpu)
        writer.start()
        model.start()
        model.warmup()
        collection_document = load_yaml(args.runtime_config)
        camera_config = collection_document.get("cameras", {})
        workspace_serial = str(camera_config["workspace"]["serial"])
        wrist_serial = str(camera_config["wrist"]["serial"])
        rh56_device = str(runtime["rh56_device"])
        require_serial_by_id_path(
            rh56_device,
            require_exists=True,
            allow_direct_ch341=bool(runtime.get("allow_direct_ch341_device", False)),
        )
        serial_identity = inspect_serial_device(
            rh56_device,
            allow_direct_ch341=bool(runtime.get("allow_direct_ch341_device", False)),
        )
        occupied_pids = serial_identity.get("occupied_pids", [])
        if occupied_pids:
            raise RuntimeError(f"RH56 serial device is already in use by PIDs {occupied_pids}")
        rh56_config = load_yaml(Path(str(runtime["rh56_config"])))
        rh56_config["mode"] = "real"
        rh56_config["backend_type"] = "serial_protocol"
        rh56_config.setdefault("serial", {})["port"] = rh56_device
        rh56_config.setdefault("diagnostics", {})["enabled"] = True
        rh56_control = RH56PcDirectControl(RH56SerialBackend(rh56_config), rh56_config)
        rh56_legal_max = rh56_control.max_close
        rh56_worker = RH56PcDirectWorker(rh56_control)
        rh56_worker.start(HandOperation.COMBINED)
        cameras = [
            CameraReader("workspace", workspace_serial),
            CameraReader("wrist", wrist_serial),
        ]
        for camera in cameras:
            camera.start()
        for camera in cameras:
            camera.wait_after(None, 15.0)

        with tempfile.TemporaryDirectory(prefix="act_physical_rollout_") as temporary:
            temporary_path = Path(temporary)
            runtime_arm = ArmOnlyRuntime(
                LatestTargetPublisher(temporary_path / "target.sock"),
                WorkerStatusReceiver(temporary_path / "status.sock"),
            )
            jaka_adapter = JakaAcceptedJointTargetAdapter(runtime_arm, allow_motion=True)
            native_metrics = output / "native_metrics.json"
            native = NativeWorkerProcess(
                Path(str(runtime["worker"])),
                _native_arguments(
                    runtime,
                    config,
                    temporary_path / "target.sock",
                    temporary_path / "status.sock",
                    native_metrics,
                    args.duration_sec,
                ),
            )
            native.start()
            status = _wait_status(runtime_arm, native)
            if not jaka_adapter.apply_joint_position(status.joint_position_rad):
                raise RuntimeError("failed to publish native startup alignment target")
            startup_sent = True
            rh56_worker.activate_from_measured(time.monotonic_ns())
            inference = InferenceThread(
                model,
                (cameras[0], cameras[1]),
                runtime_arm,
                rh56_worker,
                writer,
                initial_status=status,
                rate_hz=args.query_rate_hz,
                max_age_ms=args.max_source_age_ms,
            )
            inference.start()
            next_command_ns = time.monotonic_ns()
            command_period_ns = int(round(1e9 / args.command_rate_hz))
            deadline = next_command_ns + int(args.duration_sec * 1e9)
            while time.monotonic_ns() < deadline:
                now_ns = time.monotonic_ns()
                if now_ns < next_command_ns:
                    time.sleep((next_command_ns - now_ns) / 1e9)
                command_start_ns = time.monotonic_ns()
                if native.process is None or native.process.poll() is not None:
                    raise RuntimeError("native JAKA worker exited during ACT rollout")
                if rh56_worker.failed:
                    raise RuntimeError("RH56 worker failed during ACT rollout")
                if inference.error is not None:
                    raise RuntimeError(inference.error)
                current = inference.get_latest()
                if current is not None and current.sequence != policy_consumed_sequence:
                    if current.sequence > policy_consumed_sequence:
                        last_policy = current
                        predictions.append(current.chunk.copy())
                        policy_consumed_sequence = current.sequence
                        policy_index = 0
                if last_policy is None:
                    action = np.asarray(status.joint_position_rad, dtype=np.float64).copy()
                    hand_target = rh56_worker.latest_feedback
                    if hand_target is None:
                        raise RuntimeError("RH56 activation feedback disappeared")
                    action = np.concatenate([action, np.asarray(hand_target.position_normalized, dtype=np.float64)])
                    source_sequence = 0
                    chunk_index = -1
                else:
                    if command_start_ns - last_policy.query_end_ns > args.max_policy_age_ms * 1e6:
                        raise RuntimeError("ACT inference result exceeded policy freshness limit")
                    chunk_index = min(policy_index, 1)
                    action = last_policy.chunk[chunk_index].astype(np.float64, copy=False)
                    policy_index += 1
                    source_sequence = last_policy.sequence
                raw_action = action.copy()
                action, command_projection_events = _project_rh56_command(
                    raw_action,
                    legal_min=rh56_legal_min,
                    legal_max=rh56_legal_max,
                )
                for event in command_projection_events:
                    projection_events.append(
                        {
                            "command_index": command_count,
                            "policy_query_sequence": source_sequence,
                            "policy_chunk_index": chunk_index,
                            **event,
                        }
                    )
                _assert_policy_action(action)
                if jaka_adapter is None or not jaka_adapter.apply_joint_position(
                    tuple(float(value) for value in action[:6]),
                    source_capture_ns=(time.monotonic_ns() if last_policy is None else last_policy.observation.ready_ns),
                    local_receive_ns=command_start_ns,
                    processing_ns=command_start_ns,
                ):
                    raise RuntimeError("JAKA target publication failed")
                rh56_worker.submit_target(tuple(float(value) for value in action[6:]), command_start_ns)
                workspace_sample = cameras[0].latest()
                wrist_sample = cameras[1].latest()
                status_sample = _command_status_sample(last_policy, status)
                feedback_sample = rh56_worker.latest_dataset_feedback
                writer.submit(
                    "command",
                    (
                        {
                            "command_index": command_count,
                            "command_start_ns": command_start_ns,
                            "policy_query_sequence": source_sequence,
                            "policy_chunk_index": chunk_index,
                            "raw_policy_action": (
                                None if chunk_index < 0 else raw_action.tolist()
                            ),
                            "projected_command": action.tolist(),
                            "action": action.tolist(),
                            "jaka_status": {
                                "observation_ns": status_sample.observation_monotonic_ns,
                                "flags": int(status_sample.flags),
                            },
                            "rh56_angle_ns": (None if feedback_sample is None else feedback_sample.angle_act_timestamp_ns),
                            "rh56_force_ns": (None if feedback_sample is None else feedback_sample.force_act_timestamp_ns),
                            "rh56_force": (None if feedback_sample is None else feedback_sample.feedback.load_or_force_raw_count),
                            "workspace_frame_number": workspace_sample.frame_number,
                            "wrist_frame_number": wrist_sample.frame_number,
                            "workspace_host_ns": workspace_sample.host_monotonic_ns,
                            "wrist_host_ns": wrist_sample.host_monotonic_ns,
                        },
                        workspace_sample.rgb,
                        wrist_sample.rgb,
                    ),
                )
                command_count += 1
                next_command_ns += command_period_ns
                now_ns = time.monotonic_ns()
                if next_command_ns <= now_ns:
                    next_command_ns = now_ns + command_period_ns
            if inference is not None:
                inference.stop()
        if inference is not None and inference.error is not None:
            raise RuntimeError(inference.error)
    except BaseException as exc:
        abort_reason = f"{type(exc).__name__}: {exc}"
    finally:
        # Stop the native arm worker before waiting on camera, RH56, or model
        # cleanup.  Otherwise the producer can stop publishing while those
        # bounded cleanup operations run and the native liveness watchdog
        # reports a false terminal command-stream timeout.
        if jaka_adapter is not None and not jaka_adapter.stopped:
            jaka_adapter.stop()
        if native is not None:
            native.stop(timeout_s=8.0)
        if inference is not None:
            inference.stop_event.set()
            if inference.thread.is_alive():
                inference.thread.join(timeout=5.0)
        for camera in cameras:
            camera.stop()
        if rh56_worker is not None:
            if abort_reason is not None:
                rh56_worker.arm_terminal_stop(abort_reason)
            else:
                rh56_worker.hold("act_rollout_complete")
        if runtime_arm is not None:
            runtime_arm.close()
        if rh56_worker is not None:
            rh56_worker.cleanup()
        model.stop()
        if predictions:
            np.savez_compressed(output / "act_predictions.npz", chunks=np.stack(predictions))
        try:
            writer.finish()
        except BaseException as exc:
            if abort_reason is None:
                abort_reason = f"writer_failure: {type(exc).__name__}: {exc}"
        effective_label = args.label
        if effective_label == "UNLABELED" and abort_reason is not None:
            effective_label = "CONTROL_OR_SOFTWARE_ABORT"
        summary = {
            "schema_version": "act_physical_rollout.v1",
            "checkpoint": str(args.checkpoint.resolve()),
            "command_enabled": True,
            "duration_sec": args.duration_sec,
            "policy_query_rate_hz": args.query_rate_hz,
            "command_rate_hz": args.command_rate_hz,
            "chunk_size": 16,
            "consumed_chunk_indices": [0, 1],
            "force_used_by_policy": model.requires_environment_state,
            "policy_environment_state_key": (
                ENVIRONMENT_STATE_KEY if model.requires_environment_state else None
            ),
            "rh56_command_projection": {
                "enabled": True,
                "legal_min": [rh56_legal_min] * 6,
                "legal_max": [rh56_legal_max] * 6,
                "event_count": len(projection_events),
                "events": projection_events,
            },
            "startup_alignment_target_sent": startup_sent,
            "command_count": command_count,
            "policy_query_count": 0 if inference is None else inference.query_count,
            "policy_deadline_skips": 0 if inference is None else inference.skipped_deadlines,
            "abort_reason": abort_reason,
            "label": effective_label,
            "notes": args.notes,
            "camera": {camera.role: camera.summary() for camera in cameras},
            "rh56_diagnostics": None if rh56_worker is None else rh56_worker.diagnostics_snapshot(),
            "native_metrics_path": str((output / "native_metrics.json").resolve()),
        }
        (output / "rollout_summary.json").write_text(json.dumps(summary, indent=2, default=str) + "\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--model-container",
        default=None,
        help="CUDA LeRobot image for the inference-only model worker",
    )
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--query-rate-hz", type=float, default=15.0)
    parser.add_argument("--command-rate-hz", type=float, default=30.0)
    parser.add_argument("--max-source-age-ms", type=float, default=250.0)
    parser.add_argument("--max-policy-age-ms", type=float, default=250.0)
    parser.add_argument("--label", default="UNLABELED")
    parser.add_argument("--notes", default="")
    args = parser.parse_args()
    if not (0.0 < args.query_rate_hz <= 15.0):
        raise SystemExit("query rate must be within (0,15]")
    if not (0.0 < args.command_rate_hz <= 30.0):
        raise SystemExit("command rate must be within (0,30]")
    summary = run(args)
    print(json.dumps(summary, indent=2, default=str))
    return 0 if summary["abort_reason"] is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
