#!/usr/bin/env python3
"""Run one bounded physical ACT baseline rollout.

This entry point deliberately reuses the command-disabled ACT model worker,
the existing JAKA native worker/adapter, the production RH56 PC-direct worker,
and the existing RealSense reader.  It adds no policy-side grasp, waypoint, or
force logic.  The policy emits the recorded absolute 12-D action directly.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass, replace
import json
import pickle
from pathlib import Path
import queue
import select
import signal
import socket
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any

import cv2
import numpy as np

from act_live_shadow import CameraReader, CameraSample, _model_request, _recv_exact, preprocess_live_rgb
from embodiment_core.act_contract import ActCheckpointContract
from embodiment_core.act_temporal_executor import AbsoluteTimeTemporalEnsembler, TemporalSelection
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
from teleoperation.accepted_target import ArmControlHeartbeat, ArmControlState
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
RH56_MAX_PROJECTION_CORRECTION = 0.02
CONTROL_TIMING_STAGE_NAMES = (
    "camera_acquisition",
    "jaka_measured_state_acquisition",
    "rh56_measured_state_acquisition",
    "observation_assembly",
    "preprocessing",
    "policy_request_serialization",
    "policy_socket_send",
    "policy_response_deserialization",
    "policy_transport_metadata_receive",
    "policy_worker_receive_decode",
    "policy_worker_batch_wrap",
    "checkpoint_preprocessing",
    "model_forward",
    "checkpoint_postprocessing",
    "policy_worker_output_to_host",
    "policy_worker_response_serialization",
    "policy_worker_socket_send",
    "policy_socket_receive",
    "policy_worker_receive_wait",
    "temporal_ensemble",
    "action_split",
    "rh56_projection",
    "jaka_command_call",
    "rh56_command_call",
    "logging_provenance_enqueue",
    "critical_path",
)


def _timed_model_request(connection: socket.socket, value: Any) -> tuple[Any, dict[str, float]]:
    """Send one model request while timing host and worker transport stages."""

    serialization_started_ns = time.perf_counter_ns()
    payload = pickle.dumps(value, protocol=5)
    serialization_ended_ns = time.perf_counter_ns()
    send_started_ns = time.perf_counter_ns()
    connection.sendall(struct.pack("!Q", len(payload)) + payload)
    send_ended_ns = time.perf_counter_ns()
    receive_started_ns = time.perf_counter_ns()
    size = struct.unpack("!Q", _recv_exact(connection, 8))[0]
    response_payload = _recv_exact(connection, size)
    receive_ended_ns = time.perf_counter_ns()
    response_deserialization_started_ns = time.perf_counter_ns()
    response = pickle.loads(response_payload)
    response_deserialization_ended_ns = time.perf_counter_ns()
    transport_metadata_receive_ms = 0.0
    if response.pop("_has_transport_timing_frame", False):
        metadata_receive_started_ns = time.perf_counter_ns()
        metadata_size = struct.unpack("!Q", _recv_exact(connection, 8))[0]
        metadata = pickle.loads(_recv_exact(connection, metadata_size))
        transport_metadata_receive_ms = (
            time.perf_counter_ns() - metadata_receive_started_ns
        ) / 1e6
        response.setdefault("timing_ms", {}).update(
            metadata.get("transport_timing_ms", {})
        )
    timings = {
        "policy_request_serialization": (
            serialization_ended_ns - serialization_started_ns
        )
        / 1e6,
        "policy_socket_send": (send_ended_ns - send_started_ns) / 1e6,
        "policy_socket_receive": (receive_ended_ns - receive_started_ns) / 1e6,
        "policy_response_deserialization": (
            response_deserialization_ended_ns - response_deserialization_started_ns
        )
        / 1e6,
        "policy_transport_metadata_receive": transport_metadata_receive_ms,
    }
    return response, timings


class BoundedStageTiming:
    """Bounded online timing samples; percentile work happens at shutdown."""

    def __init__(self, *, capacity: int = 4096) -> None:
        self._samples = {
            name: deque(maxlen=capacity) for name in CONTROL_TIMING_STAGE_NAMES
        }

    def add(self, values: dict[str, float]) -> None:
        for name, value in values.items():
            if name in self._samples and np.isfinite(value):
                self._samples[name].append(float(value))

    @staticmethod
    def _summary(values: deque[float]) -> dict[str, float | int | None]:
        if not values:
            return {
                "count": 0,
                "mean": None,
                "p50": None,
                "p90": None,
                "p95": None,
                "p99": None,
                "max": None,
            }
        array = np.asarray(values, dtype=np.float64)
        return {
            "count": int(array.size),
            "mean": float(np.mean(array)),
            "p50": float(np.quantile(array, 0.50)),
            "p90": float(np.quantile(array, 0.90)),
            "p95": float(np.quantile(array, 0.95)),
            "p99": float(np.quantile(array, 0.99)),
            "max": float(np.max(array)),
        }

    def summary(self) -> dict[str, dict[str, float | int | None]]:
        return {name: self._summary(values) for name, values in self._samples.items()}


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
    query_command_tick: int
    observation: Observation
    chunk: np.ndarray
    timing_ms: dict[str, float]
    stage_timing_ms: dict[str, float]


@dataclass
class ActionChunkConsumer:
    """Consume a bounded prefix before adopting the newest policy chunk."""

    consume_actions: int
    chunk_size: int = 16
    active: Prediction | None = None
    next_index: int = 0
    superseded_predictions: int = 0

    def __post_init__(self) -> None:
        if self.chunk_size < 1:
            raise ValueError("chunk_size must be positive")
        if not 1 <= self.consume_actions <= self.chunk_size:
            raise ValueError(
                f"consume_actions must be within [1,{self.chunk_size}]"
            )

    def select(self, latest: Prediction | None) -> tuple[Prediction, int] | None:
        if latest is not None and (
            self.active is None or self.next_index >= self.consume_actions
        ):
            if self.active is None or latest.sequence > self.active.sequence:
                if self.active is not None:
                    self.superseded_predictions += max(
                        0, latest.sequence - self.active.sequence - 1
                    )
                self.active = latest
                self.next_index = 0
        if self.active is None:
            return None
        index = min(self.next_index, self.consume_actions - 1)
        self.next_index += 1
        return self.active, index


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
        self.submit_count = 0
        self.queue_high_watermark = 0
        self.queue_drop_count = 0

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
            self.queue_drop_count += 1
            raise RuntimeError("rollout recorder queue full") from exc
        self.submit_count += 1
        self.queue_high_watermark = max(self.queue_high_watermark, self._queue.qsize())

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
        # AF_UNIX paths are limited to roughly 108 bytes on Linux.  Keep the
        # IPC endpoint independent of the user-facing artifact directory so a
        # descriptive rollout label cannot prevent model startup.
        self._socket_dir = Path(tempfile.mkdtemp(prefix="act_model_", dir="/tmp"))
        self.socket = self._socket_dir / "model.sock"
        self.ready = root / "model_ready"
        self.summary = root / "model_summary.json"
        self.contract = ActCheckpointContract.from_checkpoint(checkpoint)
        self.requires_environment_state = self.contract.requires_environment_state
        self.process: subprocess.Popen[str] | None = None
        self.connection = None
        self.last_request_timing: dict[str, float] = {}

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
                "-e", "PYTHONPATH=/workspace/embodied_lab/src",
                "-v", f"{repo_root}:/workspace/embodied_lab:ro",
                "-v", f"{self.root}:{self.root}",
                "-v", f"{self._socket_dir}:{self._socket_dir}",
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
        response, self.last_request_timing = _timed_model_request(
            self.connection, observation
        )
        if "error" in response:
            raise RuntimeError(str(response["error"]))
        return response

    def warmup(self, count: int = 2) -> None:
        """Warm CUDA/processor kernels before any physical command is enabled."""

        observation = {
            key: np.zeros(tuple(self.contract.input_features[key]["shape"]), dtype=np.float32)
            for key in (*self.contract.image_keys, self.contract.state_key)
        }
        if self.contract.environment_state_key is not None:
            observation[self.contract.environment_state_key] = np.zeros((6,), dtype=np.float32)
        for _ in range(count):
            response = self.request(observation)
            prediction = np.asarray(response.get("prediction"))
            expected = (self.contract.chunk_size, self.contract.action_dim)
            if prediction.shape != expected or not np.isfinite(prediction).all():
                raise RuntimeError(f"ACT warmup returned an invalid {expected} prediction")

    def stop(self) -> None:
        try:
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
        finally:
            shutil.rmtree(self._socket_dir, ignore_errors=True)


class InferenceThread:
    def __init__(self, model: ModelWorker, cameras: tuple[CameraReader, CameraReader], runtime: ArmOnlyRuntime, rh56: RH56PcDirectWorker, writer: AsyncRolloutWriter, *, initial_status: WorkerStatusPacket, rate_hz: float, max_age_ms: float, command_epoch_ns: int, command_period_ns: int, pending_capacity: int = 64) -> None:
        self.model = model
        self.workspace, self.wrist = cameras
        self.runtime = runtime
        self.rh56 = rh56
        self.writer = writer
        self._last_status = initial_status
        self.period_ns = int(round(1e9 / rate_hz))
        self.command_epoch_ns = int(command_epoch_ns)
        self.command_period_ns = int(command_period_ns)
        if pending_capacity < 1:
            raise ValueError("pending prediction capacity must be positive")
        self.pending_capacity = int(pending_capacity)
        self.max_age_ns = int(round(max_age_ms * 1e6))
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name="act-inference", daemon=True)
        self.started = False
        self._lock = threading.Lock()
        self.latest: Prediction | None = None
        self.error: str | None = None
        self.query_count = 0
        self.skipped_deadlines = 0
        self._next_sequence = 1
        self._chunks: list[np.ndarray] = []
        self._pending: queue.Queue[Prediction] = queue.Queue(maxsize=self.pending_capacity)
        self._recent: dict[int, Prediction] = {}
        self.pending_drops = 0

    def start(self) -> None:
        self.started = True
        self.thread.start()

    def snapshot(self, stage_timing_ms: dict[str, float] | None = None) -> Observation:
        def record(name: str, started_ns: int) -> None:
            if stage_timing_ms is not None:
                stage_timing_ms[name] = (time.perf_counter_ns() - started_ns) / 1e6

        started_ns = time.perf_counter_ns()
        workspace = self.workspace.latest()
        wrist = self.wrist.latest()
        record("camera_acquisition", started_ns)

        started_ns = time.perf_counter_ns()
        status = self.runtime.latest_status()
        if status is not None:
            self._last_status = status
        status = self._last_status
        record("jaka_measured_state_acquisition", started_ns)

        started_ns = time.perf_counter_ns()
        feedback = self.rh56.latest_dataset_feedback
        if feedback is None:
            raise RuntimeError("RH56 feedback is not available")
        record("rh56_measured_state_acquisition", started_ns)
        started_ns = time.perf_counter_ns()
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
        record("observation_assembly", started_ns)
        return Observation(workspace, wrist, status, feedback, state, force, now_ns)

    def query_once(self, *, command_tick: int | None = None, publish_pending: bool = True) -> Prediction:
        """Run one contract-validated ACT query and persist its provenance."""

        query_start_ns = time.monotonic_ns()
        stage_timing_ms: dict[str, float] = {}
        observation = self.snapshot(stage_timing_ms)
        camera_samples = {
            self.model.contract.workspace_image_key: observation.workspace,
            self.model.contract.wrist_image_key: observation.wrist,
        }
        preprocessing_started_ns = time.perf_counter_ns()
        request = {
            key: preprocess_live_rgb(sample.rgb)
            for key, sample in camera_samples.items()
        }
        request[self.model.contract.state_key] = observation.state
        if self.model.contract.environment_state_key is not None:
            request[self.model.contract.environment_state_key] = observation.force
        stage_timing_ms["preprocessing"] = (
            time.perf_counter_ns() - preprocessing_started_ns
        ) / 1e6
        response = self.model.request(request)
        stage_timing_ms.update(self.model.last_request_timing)
        query_end_ns = time.monotonic_ns()
        chunk = np.asarray(response["prediction"], dtype=np.float32)
        expected = (self.model.contract.chunk_size, self.model.contract.action_dim)
        if chunk.shape != expected or not np.isfinite(chunk).all():
            raise RuntimeError(f"ACT output must be finite {expected}, got {chunk.shape}")
        query_command_tick = (
            int(command_tick)
            if command_tick is not None
            else max(0, int(np.ceil((query_end_ns - self.command_epoch_ns) / self.command_period_ns)))
        )
        response_timing = {
            key: float(value) for key, value in response["timing_ms"].items()
        }
        for response_key, stage_key in (
            ("worker_receive_decode", "policy_worker_receive_decode"),
            ("worker_receive_wait", "policy_worker_receive_wait"),
            ("worker_batch_wrap", "policy_worker_batch_wrap"),
            ("checkpoint_preprocessing", "checkpoint_preprocessing"),
            ("act_inference", "model_forward"),
            ("checkpoint_postprocessing", "checkpoint_postprocessing"),
            ("worker_output_to_host", "policy_worker_output_to_host"),
            ("worker_response_serialization", "policy_worker_response_serialization"),
            ("worker_socket_send", "policy_worker_socket_send"),
        ):
            if response_key in response_timing:
                stage_timing_ms[stage_key] = response_timing[response_key]
        prediction = Prediction(
            self._next_sequence,
            query_start_ns,
            query_end_ns,
            query_command_tick,
            observation,
            chunk,
            response_timing,
            stage_timing_ms,
        )
        self._next_sequence += 1
        with self._lock:
            self.latest = prediction
            self._chunks.append(prediction.chunk.copy())
            self._recent[prediction.sequence] = prediction
            if len(self._recent) > self.pending_capacity * 2:
                del self._recent[min(self._recent)]
        if publish_pending:
            try:
                self._pending.put_nowait(prediction)
            except queue.Full:
                # Never block the query producer on a diagnostic queue.
                try:
                    self._pending.get_nowait()
                except queue.Empty:
                    pass
                self.pending_drops += 1
                self._pending.put_nowait(prediction)
        self.writer.submit(
            "query",
            {
                "query_sequence": prediction.sequence,
                "query_start_ns": query_start_ns,
                "query_end_ns": query_end_ns,
                "query_command_tick": prediction.query_command_tick,
                "chunk_size": int(prediction.chunk.shape[0]),
                "observation_ready_ns": observation.ready_ns,
                "workspace_frame_number": observation.workspace.frame_number,
                "wrist_frame_number": observation.wrist.frame_number,
                "workspace_host_ns": observation.workspace.host_monotonic_ns,
                "wrist_host_ns": observation.wrist.host_monotonic_ns,
                "jaka_observation_ns": observation.status.observation_monotonic_ns,
                "rh56_angle_ns": observation.feedback.angle_act_timestamp_ns,
                "rh56_force_ns": observation.feedback.force_act_timestamp_ns,
                "state": observation.state.tolist(),
                "force": observation.force.tolist(),
                "timing_ms": prediction.timing_ms,
                "stage_timing_ms": prediction.stage_timing_ms,
            },
        )
        self.query_count += 1
        return prediction

    def _run(self) -> None:
        next_due_ns = time.monotonic_ns()
        try:
            while not self.stop_event.is_set():
                now_ns = time.monotonic_ns()
                if now_ns < next_due_ns:
                    self.stop_event.wait((next_due_ns - now_ns) / 1e9)
                    if self.stop_event.is_set():
                        break
                self.query_once()
                next_due_ns += self.period_ns
                now_ns = time.monotonic_ns()
                if next_due_ns <= now_ns:
                    self.skipped_deadlines += max(1, (now_ns - next_due_ns) // self.period_ns + 1)
                    next_due_ns = now_ns + self.period_ns
        except BaseException as exc:
            self.error = f"{type(exc).__name__}: {exc}"

    def stop(self) -> None:
        self.stop_event.set()
        if not self.started:
            return
        self.thread.join(timeout=5.0)
        if self.thread.is_alive():
            raise RuntimeError("ACT inference thread did not stop")

    def get_latest(self) -> Prediction | None:
        with self._lock:
            return self.latest

    def drain_predictions(self) -> list[Prediction]:
        values: list[Prediction] = []
        while True:
            try:
                values.append(self._pending.get_nowait())
            except queue.Empty:
                return values

    def get_prediction(self, sequence: int) -> Prediction | None:
        with self._lock:
            return self._recent.get(int(sequence))

    def saved_chunks(self) -> np.ndarray | None:
        with self._lock:
            if not self._chunks:
                return None
            return np.stack(self._chunks)


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
    maximum_correction: float = RH56_MAX_PROJECTION_CORRECTION,
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    """Project only the policy RH56 command at the actuator boundary.

    The model output and saved ``act_predictions.npz`` remain untouched.  The
    returned action is the value submitted to the existing RH56 worker; its
    feedback freshness, contact stop, delta, raw-count, and serial safety
    checks still run unchanged downstream.
    """

    projected = np.asarray(action, dtype=np.float64).copy()
    if projected.shape != (12,) or not np.isfinite(projected).all():
        raise ValueError("policy action must be finite [12] before RH56 projection")
    if not np.isfinite((legal_min, legal_max, maximum_correction)).all():
        raise ValueError("RH56 projection bounds and tolerance must be finite")
    if legal_min >= legal_max or maximum_correction < 0.0:
        raise ValueError("invalid RH56 projection bounds or tolerance")
    raw_rh56 = projected[6:].copy()
    projected_rh56 = np.clip(raw_rh56, float(legal_min), float(legal_max))
    correction = np.abs(projected_rh56 - raw_rh56)
    # Preserve the stated closed boundary despite subtraction round-off at,
    # for example, 1.02 - 1.0.  The numerical allowance is many orders below
    # one RH56 normalized command count and does not widen the policy limit.
    excessive = np.flatnonzero(
        correction > float(maximum_correction) + 1e-12
    )
    if excessive.size:
        index = int(excessive[0])
        raise ValueError(
            "RH56 prediction exceeds bounded projection tolerance: "
            f"channel={RH56_CHANNEL_NAMES[index]} raw={raw_rh56[index]:.9g} "
            f"projected={projected_rh56[index]:.9g} "
            f"correction={correction[index]:.9g} "
            f"tolerance={maximum_correction:.9g}"
        )
    projected[6:] = projected_rh56
    events = [
        {
            "channel": channel,
            "channel_index": index,
            "raw_value": float(raw_value),
            "projected_value": float(projected_value),
            "delta": float(projected_value - raw_value),
            "correction_magnitude": float(abs(projected_value - raw_value)),
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


def _temporal_selection_ages_ms(
    selection: TemporalSelection, now_ns: int
) -> tuple[float | None, float | None]:
    """Return (oldest, newest) contributor ages without rejecting valid history.

    Temporal ensembling intentionally retains older predictions for the
    current absolute command tick.  The policy freshness gate therefore
    applies to the newest contributing query; the oldest age remains a
    diagnostic and must not make a healthy overlapping ensemble fail.
    """

    if not selection.contributors:
        return None, None
    ages_ms = [
        max(0.0, (int(now_ns) - int(contributor.query_timestamp_ns)) / 1e6)
        for contributor in selection.contributors
    ]
    return max(ages_ms), min(ages_ms)


def _resolve_execution_options(
    *,
    requested_mode: str | None,
    chunk_size: int,
    command_rate_hz: float,
    query_rate_hz: float | None,
    consume_actions: int | None,
    temporal_ensemble_coeff: float | None,
    max_source_horizon: int | None,
    max_prediction_age_ticks: int | None,
    temporal_buffer_capacity: int | None,
) -> dict[str, Any]:
    """Resolve policy semantics without exposing legacy knobs in canonical mode."""

    execution_mode = requested_mode
    if execution_mode is None:
        execution_mode = (
            "canonical_temporal_ensemble" if chunk_size >= 60 else "consume_k"
        )
    if execution_mode == "canonical_temporal_ensemble":
        if query_rate_hz is not None and not np.isclose(
            query_rate_hz, command_rate_hz, rtol=0.0, atol=1e-9
        ):
            raise ValueError(
                "canonical_temporal_ensemble derives query rate from command rate; "
                "an independent --query-rate-hz is invalid"
            )
        if consume_actions is not None:
            raise ValueError(
                "--consume-actions is only valid with the legacy consume_k mode"
            )
        if max_source_horizon is not None:
            raise ValueError(
                "canonical_temporal_ensemble derives max source horizon from checkpoint"
            )
        if max_prediction_age_ticks is not None:
            raise ValueError(
                "canonical_temporal_ensemble does not accept a prediction-age override"
            )
        if temporal_buffer_capacity is not None:
            raise ValueError(
                "canonical_temporal_ensemble uses its internal bounded buffer"
            )
        return {
            "execution_mode": execution_mode,
            "query_rate_hz": command_rate_hz,
            "consume_actions": None,
            "temporal_ensemble_coeff": (
                0.01 if temporal_ensemble_coeff is None else temporal_ensemble_coeff
            ),
            "max_source_horizon": None,
            "max_prediction_age_ticks": None,
            "temporal_buffer_capacity": 4096,
        }
    return {
        "execution_mode": execution_mode,
        "query_rate_hz": 15.0 if query_rate_hz is None else query_rate_hz,
        "consume_actions": 2 if consume_actions is None else consume_actions,
        "temporal_ensemble_coeff": (
            0.01 if temporal_ensemble_coeff is None else temporal_ensemble_coeff
        ),
        "max_source_horizon": max_source_horizon,
        "max_prediction_age_ticks": max_prediction_age_ticks,
        "temporal_buffer_capacity": (
            4096 if temporal_buffer_capacity is None else temporal_buffer_capacity
        ),
    }


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
    projection_events: list[dict[str, Any]] = []
    rh56_legal_min = 0.0
    rh56_legal_max = 1.0
    abort_reason: str | None = None
    command_count = 0
    next_command_ns = 0
    stage_timing = BoundedStageTiming()
    write_path_mode = args.write_path_mode
    jaka_writes_enabled = write_path_mode in {"both", "jaka_only"}
    rh56_writes_enabled = write_path_mode in {"both", "rh56_only"}
    if write_path_mode != "both" and not args.stationary_write_test:
        raise ValueError(
            "write-path isolation requires --stationary-write-test; "
            "it is not a task-rollout mode"
        )
    options = _resolve_execution_options(
        requested_mode=args.act_execution_mode,
        chunk_size=model.contract.chunk_size,
        command_rate_hz=args.command_rate_hz,
        query_rate_hz=args.query_rate_hz,
        consume_actions=args.consume_actions,
        temporal_ensemble_coeff=args.temporal_ensemble_coeff,
        max_source_horizon=args.max_source_horizon,
        max_prediction_age_ticks=args.max_prediction_age_ticks,
        temporal_buffer_capacity=args.temporal_buffer_capacity,
    )
    execution_mode = options["execution_mode"]
    query_rate_hz = options["query_rate_hz"]
    consume_actions = options["consume_actions"]
    temporal_ensemble_coeff = options["temporal_ensemble_coeff"]
    max_source_horizon = options["max_source_horizon"]
    max_prediction_age_ticks = options["max_prediction_age_ticks"]
    temporal_buffer_capacity = options["temporal_buffer_capacity"]
    args.query_rate_hz = query_rate_hz
    args.consume_actions = consume_actions
    args.temporal_ensemble_coeff = temporal_ensemble_coeff
    args.max_source_horizon = max_source_horizon
    args.max_prediction_age_ticks = max_prediction_age_ticks
    args.temporal_buffer_capacity = temporal_buffer_capacity
    consumer = (
        None
        if execution_mode == "canonical_temporal_ensemble"
        else ActionChunkConsumer(consume_actions, model.contract.chunk_size)
    )
    temporal_ensembler: AbsoluteTimeTemporalEnsembler | None = None
    command_epoch_ns = 0
    command_period_ns = int(round(1e9 / args.command_rate_hz))
    if execution_mode in {
        "canonical_temporal_ensemble",
        "temporal_ensemble",
        "async_temporal_ensemble",
    }:
        temporal_ensembler = AbsoluteTimeTemporalEnsembler(
            action_dim=model.contract.action_dim,
            chunk_size=model.contract.chunk_size,
            coefficient=args.temporal_ensemble_coeff,
            max_source_horizon=max_source_horizon,
            max_prediction_age_ticks=max_prediction_age_ticks,
            capacity=temporal_buffer_capacity,
        )
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
            if rh56_writes_enabled:
                rh56_worker.activate_from_measured(time.monotonic_ns())
            next_command_ns = time.monotonic_ns()
            command_epoch_ns = next_command_ns
            inference = InferenceThread(
                model,
                (cameras[0], cameras[1]),
                runtime_arm,
                rh56_worker,
                writer,
                initial_status=status,
                rate_hz=args.query_rate_hz,
                max_age_ms=args.max_source_age_ms,
                command_epoch_ns=command_epoch_ns,
                command_period_ns=command_period_ns,
                pending_capacity=args.temporal_buffer_capacity,
            )
            if execution_mode != "canonical_temporal_ensemble":
                inference.start()
            deadline = next_command_ns + int(args.duration_sec * 1e9)
            command_tick = 0
            while time.monotonic_ns() < deadline:
                now_ns = time.monotonic_ns()
                if now_ns < next_command_ns:
                    time.sleep((next_command_ns - now_ns) / 1e9)
                command_start_ns = time.monotonic_ns()
                tick_started_ns = time.perf_counter_ns()
                if native.process is None or native.process.poll() is not None:
                    raise RuntimeError("native JAKA worker exited during ACT rollout")
                if rh56_worker.failed:
                    raise RuntimeError("RH56 worker failed during ACT rollout")
                if inference.error is not None:
                    raise RuntimeError(inference.error)
                selected_policy: Prediction | None = None
                selection: TemporalSelection | None = None
                startup_fallback = False
                tick_stage_timing_ms: dict[str, float] = {}
                if execution_mode == "canonical_temporal_ensemble":
                    # Canonical ACT semantics: one complete chunk is queried
                    # for this control tick, then exactly one action for the
                    # current absolute tick is selected.  The chunk is not
                    # reduced to a consume-K prefix.
                    prediction = inference.query_once(
                        command_tick=command_tick,
                        publish_pending=False,
                    )
                    temporal_started_ns = time.perf_counter_ns()
                    temporal_ensembler.add_prediction(
                        query_id=prediction.sequence,
                        query_timestamp_ns=prediction.query_end_ns,
                        query_command_tick=command_tick,
                        chunk=prediction.chunk,
                    )
                    selection = temporal_ensembler.select(
                        command_tick=command_tick,
                        command_timestamp_ns=command_start_ns,
                    )
                    if selection.action is None:
                        raise RuntimeError(
                            "canonical temporal ensemble produced no current action"
                        )
                    action = selection.action
                    source_sequence = (
                        selection.contributors[-1].query_id
                        if selection.contributors
                        else 0
                    )
                    selected_policy = (
                        inference.get_prediction(source_sequence)
                        if source_sequence != 0
                        else None
                    )
                    chunk_index = -1
                    oldest_contributor_age_ms, newest_contributor_age_ms = (
                        _temporal_selection_ages_ms(selection, command_start_ns)
                    )
                    if (
                        newest_contributor_age_ms is not None
                        and newest_contributor_age_ms > args.max_policy_age_ms
                    ):
                        raise RuntimeError(
                            "ACT temporal-ensemble result exceeded policy freshness limit"
                        )
                    tick_stage_timing_ms.update(prediction.stage_timing_ms)
                    tick_stage_timing_ms["temporal_ensemble"] = (
                        time.perf_counter_ns() - temporal_started_ns
                    ) / 1e6
                elif temporal_ensembler is not None:
                    for pending in inference.drain_predictions():
                        temporal_ensembler.add_prediction(
                            query_id=pending.sequence,
                            query_timestamp_ns=pending.query_end_ns,
                            query_command_tick=pending.query_command_tick,
                            chunk=pending.chunk,
                        )
                    selection = temporal_ensembler.select(
                        command_tick=command_tick,
                        command_timestamp_ns=command_start_ns,
                    )
                    if selection.action is None:
                        startup_fallback = True
                        action = np.asarray(status.joint_position_rad, dtype=np.float64).copy()
                        hand_target = rh56_worker.latest_feedback
                        if hand_target is None:
                            raise RuntimeError("RH56 activation feedback disappeared")
                        action = np.concatenate([action, np.asarray(hand_target.position_normalized, dtype=np.float64)])
                        source_sequence = 0
                        chunk_index = -1
                    else:
                        action = selection.action
                        if selection.contributors:
                            source_sequence = selection.contributors[-1].query_id
                            selected_policy = inference.get_prediction(source_sequence)
                        else:
                            source_sequence = 0
                        chunk_index = -1
                    oldest_contributor_age_ms, newest_contributor_age_ms = (
                        _temporal_selection_ages_ms(selection, command_start_ns)
                    )
                    if (
                        newest_contributor_age_ms is not None
                        and newest_contributor_age_ms > args.max_policy_age_ms
                    ):
                        raise RuntimeError("ACT temporal-ensemble result exceeded policy freshness limit")
                else:
                    current = inference.get_latest()
                    assert consumer is not None
                    selected = consumer.select(current)
                    if selected is None:
                        startup_fallback = True
                        action = np.asarray(status.joint_position_rad, dtype=np.float64).copy()
                        hand_target = rh56_worker.latest_feedback
                        if hand_target is None:
                            raise RuntimeError("RH56 activation feedback disappeared")
                        action = np.concatenate([action, np.asarray(hand_target.position_normalized, dtype=np.float64)])
                        source_sequence = 0
                        chunk_index = -1
                    else:
                        selected_policy, chunk_index = selected
                        if command_start_ns - selected_policy.query_end_ns > args.max_policy_age_ms * 1e6:
                            raise RuntimeError("ACT inference result exceeded policy freshness limit")
                        action = selected_policy.chunk[chunk_index].astype(np.float64, copy=False)
                        source_sequence = selected_policy.sequence
                if selected_policy is not None:
                    tick_stage_timing_ms.update(selected_policy.stage_timing_ms)
                if startup_fallback:
                    action = np.asarray(status.joint_position_rad, dtype=np.float64).copy()
                    hand_target = rh56_worker.latest_feedback
                    if hand_target is None:
                        raise RuntimeError("RH56 activation feedback disappeared")
                    action = np.concatenate([action, np.asarray(hand_target.position_normalized, dtype=np.float64)])
                action_split_started_ns = time.perf_counter_ns()
                model_selected_action = action.copy()
                if args.stationary_write_test:
                    current_feedback = rh56_worker.latest_dataset_feedback
                    if current_feedback is None:
                        raise RuntimeError("RH56 feedback disappeared during stationary audit")
                    current_status = (
                        status if selected_policy is None else selected_policy.observation.status
                    )
                    action = np.concatenate(
                        [
                            np.asarray(current_status.joint_position_rad, dtype=np.float64),
                            np.asarray(
                                current_feedback.feedback.position_normalized,
                                dtype=np.float64,
                            ),
                        ]
                    )
                raw_action = action.copy()
                tick_stage_timing_ms["action_split"] = (
                    time.perf_counter_ns() - action_split_started_ns
                ) / 1e6
                projection_started_ns = time.perf_counter_ns()
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
                tick_stage_timing_ms["rh56_projection"] = (
                    time.perf_counter_ns() - projection_started_ns
                ) / 1e6
                jaka_call_started_ns = time.perf_counter_ns()
                if jaka_adapter is None:
                    raise RuntimeError("JAKA adapter is unavailable")
                if jaka_writes_enabled:
                    if not jaka_adapter.apply_joint_position(
                        tuple(float(value) for value in action[:6]),
                        source_capture_ns=(time.monotonic_ns() if selected_policy is None else selected_policy.observation.ready_ns),
                        local_receive_ns=command_start_ns,
                        processing_ns=command_start_ns,
                    ):
                        raise RuntimeError("JAKA target publication failed")
                else:
                    heartbeat = ArmControlHeartbeat(
                        input_sequence_number=command_count + 1,
                        input_receive_monotonic_ns=command_start_ns,
                        generated_monotonic_ns=command_start_ns,
                        reference_generation=1,
                        clutch_generation=1,
                        state=ArmControlState.ACTIVE,
                        reason="hardware_write_timing_audit_hold",
                        last_accepted_target_sequence=jaka_adapter.last_sequence,
                    )
                    if not jaka_adapter.heartbeat(heartbeat):
                        raise RuntimeError("JAKA heartbeat publication failed")
                tick_stage_timing_ms["jaka_command_call"] = (
                    time.perf_counter_ns() - jaka_call_started_ns
                ) / 1e6
                rh56_call_started_ns = time.perf_counter_ns()
                if rh56_writes_enabled:
                    rh56_worker.submit_target(
                        tuple(float(value) for value in action[6:]), command_start_ns
                    )
                tick_stage_timing_ms["rh56_command_call"] = (
                    time.perf_counter_ns() - rh56_call_started_ns
                ) / 1e6
                workspace_sample = cameras[0].latest()
                wrist_sample = cameras[1].latest()
                status_sample = _command_status_sample(selected_policy, status)
                feedback_sample = rh56_worker.latest_dataset_feedback
                rh56_trace = rh56_worker.control.command_trace()
                logging_started_ns = time.perf_counter_ns()
                writer.submit(
                    "command",
                    (
                        {
                            "command_index": command_count,
                            "command_start_ns": command_start_ns,
                            "command_tick": command_tick,
                            "execution_mode": execution_mode,
                            "policy_query_sequence": source_sequence,
                            "policy_chunk_index": chunk_index,
                            "raw_policy_action": (
                                None if source_sequence == 0 else raw_action.tolist()
                            ),
                            "policy_raw_prediction": (
                                None
                                if source_sequence == 0
                                else model_selected_action.tolist()
                            ),
                            "model_selected_action": (
                                None
                                if source_sequence == 0
                                else model_selected_action.tolist()
                            ),
                            "stationary_write_test": args.stationary_write_test,
                            "policy_requested_target": (
                                None
                                if source_sequence == 0
                                else model_selected_action[6:].tolist()
                            ),
                            "pre_safety_target": raw_action.tolist(),
                            "stage_timing_ms": tick_stage_timing_ms,
                            "oldest_contributor_age_ms": (
                                oldest_contributor_age_ms
                                if selection is not None
                                else None
                            ),
                            "newest_contributor_age_ms": (
                                newest_contributor_age_ms
                                if selection is not None
                                else None
                            ),
                            "ensemble_raw_target": (
                                raw_action.tolist() if selection is not None and source_sequence != 0 else None
                            ),
                            "temporal_ensemble": (
                                None if selection is None else selection.as_dict()
                            ),
                            "post_projection_target": (
                                None if source_sequence == 0 else action[6:].tolist()
                            ),
                            "post_filter_target": (
                                None if source_sequence == 0 else action[6:].tolist()
                            ),
                            "rh56_command_trace_latest": rh56_trace,
                            "post_delta_limit_target": rh56_trace.get("post_delta_limit_target"),
                            "post_contact_safety_selected_target": rh56_trace.get("post_contact_safety_selected_target"),
                            "actually_written_target": rh56_trace.get("actually_written_target"),
                            "projected_command": action.tolist(),
                            "action": action.tolist(),
                            "jaka_status": {
                                "observation_ns": status_sample.observation_monotonic_ns,
                                "flags": int(status_sample.flags),
                            },
                            "rh56_angle_ns": (None if feedback_sample is None else feedback_sample.angle_act_timestamp_ns),
                            "rh56_force_ns": (None if feedback_sample is None else feedback_sample.force_act_timestamp_ns),
                            "rh56_force": (None if feedback_sample is None else feedback_sample.feedback.load_or_force_raw_count),
                            "rh56_position": (None if feedback_sample is None else feedback_sample.feedback.position_normalized),
                            "workspace_frame_number": workspace_sample.frame_number,
                            "wrist_frame_number": wrist_sample.frame_number,
                            "workspace_host_ns": workspace_sample.host_monotonic_ns,
                            "wrist_host_ns": wrist_sample.host_monotonic_ns,
                        },
                        workspace_sample.rgb,
                        wrist_sample.rgb,
                    ),
                )
                tick_stage_timing_ms["logging_provenance_enqueue"] = (
                    time.perf_counter_ns() - logging_started_ns
                ) / 1e6
                tick_stage_timing_ms["critical_path"] = (
                    time.perf_counter_ns() - tick_started_ns
                ) / 1e6
                stage_timing.add(tick_stage_timing_ms)
                command_count += 1
                command_tick += 1
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
        saved_chunks = None if inference is None else inference.saved_chunks()
        if saved_chunks is not None:
            np.savez_compressed(
                output / "act_predictions.npz",
                chunks=saved_chunks,
                query_sequences=np.arange(1, len(saved_chunks) + 1, dtype=np.int64),
            )
        try:
            writer.finish()
        except BaseException as exc:
            if abort_reason is None:
                abort_reason = f"writer_failure: {type(exc).__name__}: {exc}"
        effective_label = args.label
        if effective_label == "UNLABELED" and abort_reason is not None:
            effective_label = "CONTROL_OR_SOFTWARE_ABORT"
        summary = {
            "schema_version": "act_physical_rollout.v2",
            "runtime_config": str(args.runtime_config.resolve()),
            "checkpoint": str(args.checkpoint.resolve()),
            "model_container": args.model_container,
            "command_enabled": True,
            "duration_sec": args.duration_sec,
            "policy_query_rate_hz": args.query_rate_hz,
            "command_rate_hz": args.command_rate_hz,
            "chunk_size": model.contract.chunk_size,
            "action_dim": model.contract.action_dim,
            "model_input_keys": list(model.contract.input_features),
            "checkpoint_contract": model.contract.summary(),
            "act_execution_mode": execution_mode,
            "write_path_mode": write_path_mode,
            "stationary_write_test": args.stationary_write_test,
            "consume_actions": args.consume_actions,
            "consumed_chunk_indices": (
                None
                if args.consume_actions is None
                else list(range(args.consume_actions))
            ),
            "policy_predictions_superseded_before_adoption": (
                0 if consumer is None else consumer.superseded_predictions
            ),
            "temporal_ensemble": (
                None
                if temporal_ensembler is None
                else {
                    "coefficient": args.temporal_ensemble_coeff,
                    "max_source_horizon": temporal_ensembler.max_source_horizon,
                    "max_prediction_age_ticks": temporal_ensembler.max_prediction_age_ticks,
                    "capacity": temporal_ensembler.capacity,
                    "stats": temporal_ensembler.stats(),
                    "pending_prediction_drops": 0 if inference is None else inference.pending_drops,
                }
            ),
            "force_used_by_policy": model.requires_environment_state,
            "policy_environment_state_key": (
                ENVIRONMENT_STATE_KEY if model.requires_environment_state else None
            ),
            "rh56_command_projection": {
                "enabled": True,
                "legal_min": [rh56_legal_min] * 6,
                "legal_max": [rh56_legal_max] * 6,
                "maximum_correction": RH56_MAX_PROJECTION_CORRECTION,
                "event_count": len(projection_events),
                "events": projection_events,
            },
            "startup_alignment_target_sent": startup_sent,
            "command_count": command_count,
            "policy_query_count": 0 if inference is None else inference.query_count,
            "policy_deadline_skips": 0 if inference is None else inference.skipped_deadlines,
            "writer": {
                "kind": type(writer).__name__,
                "submit_count": writer.submit_count,
                "queue_high_watermark": writer.queue_high_watermark,
                "queue_drop_count": writer.queue_drop_count,
            },
            "control_tick_timing": stage_timing.summary(),
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
    parser.add_argument(
        "--write-path-mode",
        choices=("none", "jaka_only", "rh56_only", "both"),
        default="both",
        help=(
            "safe stationary timing isolation: none, jaka_only, rh56_only, or both; "
            "normal collection uses both"
        ),
    )
    parser.add_argument(
        "--stationary-write-test",
        action="store_true",
        help="hold the currently measured JAKA/RH56 state during write-path isolation",
    )
    parser.add_argument(
        "--query-rate-hz",
        type=float,
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--command-rate-hz", type=float, default=30.0)
    parser.add_argument(
        "--act-execution-mode",
        choices=(
            "canonical_temporal_ensemble",
            "consume_k",
            "temporal_ensemble",
            "async_temporal_ensemble",
        ),
        default=None,
        help=(
            "ACT executor semantics; canonical_temporal_ensemble is the default "
            "for Strong ACT and performs one full-chunk query/action per control tick"
        ),
    )
    parser.add_argument(
        "--consume-actions",
        type=int,
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--temporal-ensemble-coeff", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--max-source-horizon", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--max-prediction-age-ticks", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--temporal-buffer-capacity", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--max-source-age-ms", type=float, default=250.0)
    parser.add_argument("--max-policy-age-ms", type=float, default=250.0)
    parser.add_argument("--label", default="UNLABELED")
    parser.add_argument("--notes", default="")
    args = parser.parse_args()
    if args.query_rate_hz is not None and not (0.0 < args.query_rate_hz <= 30.0):
        raise SystemExit("query rate must be within (0,30]")
    if not (0.0 < args.command_rate_hz <= 30.0):
        raise SystemExit("command rate must be within (0,30]")
    summary = run(args)
    print(json.dumps(summary, indent=2, default=str))
    return 0 if summary["abort_reason"] is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
