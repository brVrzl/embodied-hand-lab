#!/usr/bin/env python3
"""Read live physical observations and benchmark ACT without command authority.

The process opens two RGB cameras, the read-only JAKA diagnostic, and RH56
read-only feedback registers.  Predictions are saved for analysis and are
never forwarded to a robot or hand command interface.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import pickle
import signal
import socket
import struct
import subprocess
import threading
import time
from typing import Any

import cv2
import numpy as np
import yaml

from rh56_driver.hand_schema import (
    CANONICAL_HAND_ORDER,
    HandDofCalibration,
    normalize_raw,
)
from rh56_driver.pc_direct_control import inspect_serial_device
from rh56_driver.serial_backend import RH56SerialBackend


WORKSPACE_KEY = "observation.images.workspace"
WRIST_KEY = "observation.images.wrist"
STATE_KEY = "observation.state"
ENVIRONMENT_STATE_KEY = "observation.environment_state"
JAKA_LOWER = np.asarray([-6.28, -2.09, -2.27, -6.28, -2.09, -6.28], dtype=np.float64)
JAKA_UPPER = np.asarray([6.28, 2.09, 2.27, 6.28, 2.09, 6.28], dtype=np.float64)


def preprocess_live_rgb(rgb: np.ndarray) -> np.ndarray:
    """Match the validated RGB -> resize -> CHW float training path."""
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("live camera input must be uint8 HWC RGB")
    resized = cv2.resize(rgb, (320, 240), interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray(resized.transpose(2, 0, 1), dtype=np.float32) / 255.0


def _recv_exact(connection: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = connection.recv(size - len(chunks))
        if not chunk:
            raise EOFError("ACT model worker disconnected")
        chunks.extend(chunk)
    return bytes(chunks)


def _model_request(connection: socket.socket, value: Any) -> Any:
    payload = pickle.dumps(value, protocol=5)
    connection.sendall(struct.pack("!Q", len(payload)) + payload)
    size = struct.unpack("!Q", _recv_exact(connection, 8))[0]
    response = pickle.loads(_recv_exact(connection, size))
    if response.pop("_has_transport_timing_frame", False):
        metadata_size = struct.unpack("!Q", _recv_exact(connection, 8))[0]
        metadata = pickle.loads(_recv_exact(connection, metadata_size))
        response.setdefault("timing_ms", {}).update(
            metadata.get("transport_timing_ms", {})
        )
    return response


def _timed_model_request(
    connection: socket.socket, value: Any
) -> tuple[Any, dict[str, float]]:
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
    metadata_receive_ms = 0.0
    if response.pop("_has_transport_timing_frame", False):
        metadata_started_ns = time.perf_counter_ns()
        metadata_size = struct.unpack("!Q", _recv_exact(connection, 8))[0]
        metadata = pickle.loads(_recv_exact(connection, metadata_size))
        metadata_receive_ms = (time.perf_counter_ns() - metadata_started_ns) / 1e6
        response.setdefault("timing_ms", {}).update(
            metadata.get("transport_timing_ms", {})
        )
    return response, {
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
        "policy_transport_metadata_receive": metadata_receive_ms,
    }


def _percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    return float(np.quantile(np.asarray(values, dtype=np.float64), quantile))


def _distribution(values: list[float]) -> dict[str, float | int | None]:
    return {
        "count": len(values),
        "mean": None if not values else float(np.mean(values)),
        "p50": _percentile(values, 0.50),
        "p90": _percentile(values, 0.90),
        "p95": _percentile(values, 0.95),
        "p99": _percentile(values, 0.99),
        "max": None if not values else float(np.max(values)),
    }


@dataclass(frozen=True)
class CameraSample:
    sequence: int
    frame_number: int
    device_timestamp_ms: float
    host_monotonic_ns: int
    rgb: np.ndarray


class CameraReader:
    def __init__(self, role: str, serial_number: str) -> None:
        self.role = role
        self.serial_number = serial_number
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"shadow-camera-{role}", daemon=True)
        self._sample: CameraSample | None = None
        self._error: BaseException | None = None
        self._pipeline: Any | None = None
        self.received = 0
        self.frame_number_gaps = 0
        self.late_intervals = 0
        self.receive_timestamps_ns: list[int] = []

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        try:
            import pyrealsense2 as rs

            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(self.serial_number)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
            profile = pipeline.start(config)
            actual_serial = profile.get_device().get_info(rs.camera_info.serial_number)
            if actual_serial != self.serial_number:
                raise RuntimeError(
                    f"{self.role} camera identity mismatch: {actual_serial} != {self.serial_number}"
                )
            self._pipeline = pipeline
            previous_frame: int | None = None
            previous_host_ns: int | None = None
            sequence = 0
            while not self._stop.is_set():
                frames = pipeline.wait_for_frames(1000)
                color = frames.get_color_frame()
                if not color:
                    continue
                host_ns = time.monotonic_ns()
                frame_number = int(color.get_frame_number())
                if previous_frame is not None and frame_number > previous_frame + 1:
                    self.frame_number_gaps += frame_number - previous_frame - 1
                if previous_host_ns is not None and host_ns - previous_host_ns > 50_000_000:
                    self.late_intervals += 1
                previous_frame = frame_number
                previous_host_ns = host_ns
                rgb = np.asanyarray(color.get_data()).copy()
                sample = CameraSample(
                    sequence=sequence,
                    frame_number=frame_number,
                    device_timestamp_ms=float(color.get_timestamp()),
                    host_monotonic_ns=host_ns,
                    rgb=rgb,
                )
                sequence += 1
                with self._condition:
                    self._sample = sample
                    self.received += 1
                    self.receive_timestamps_ns.append(host_ns)
                    self._condition.notify_all()
        except BaseException as exc:
            with self._condition:
                self._error = exc
                self._condition.notify_all()
        finally:
            if self._pipeline is not None:
                self._pipeline.stop()

    def wait_after(self, sequence: int | None, timeout_sec: float = 0.2) -> CameraSample:
        deadline = time.monotonic() + timeout_sec
        with self._condition:
            while True:
                if self._error is not None:
                    raise RuntimeError(f"{self.role} camera failed") from self._error
                if self._sample is not None and (
                    sequence is None or self._sample.sequence > sequence
                ):
                    sample = self._sample
                    return CameraSample(
                        sequence=sample.sequence,
                        frame_number=sample.frame_number,
                        device_timestamp_ms=sample.device_timestamp_ms,
                        host_monotonic_ns=sample.host_monotonic_ns,
                        rgb=sample.rgb.copy(),
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"timed out waiting for a new {self.role} frame")
                self._condition.wait(remaining)

    def latest(self) -> CameraSample:
        """Return the newest acquired frame without waiting for another frame."""

        with self._condition:
            if self._error is not None:
                raise RuntimeError(f"{self.role} camera failed") from self._error
            if self._sample is None:
                raise RuntimeError(f"{self.role} camera frame is not yet available")
            sample = self._sample
            return CameraSample(
                sequence=sample.sequence,
                frame_number=sample.frame_number,
                device_timestamp_ms=sample.device_timestamp_ms,
                host_monotonic_ns=sample.host_monotonic_ns,
                rgb=sample.rgb.copy(),
            )

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)

    def summary(self) -> dict[str, Any]:
        timestamps = self.receive_timestamps_ns
        actual_fps = None
        if len(timestamps) > 1 and timestamps[-1] > timestamps[0]:
            actual_fps = (len(timestamps) - 1) * 1e9 / (timestamps[-1] - timestamps[0])
        return {
            "role": self.role,
            "serial_number": self.serial_number,
            "configured_format": "RGB8",
            "configured_resolution": [640, 480],
            "configured_rate_hz": 30,
            "received_frames": self.received,
            "actual_receive_rate_hz": actual_fps,
            "frame_number_gap_count": self.frame_number_gaps,
            "late_interarrival_over_50ms_count": self.late_intervals,
            "error": None if self._error is None else f"{type(self._error).__name__}: {self._error}",
        }


class JakaReadOnlyStream:
    def __init__(
        self,
        binary: Path,
        robot_ip: str,
        sample_file: Path,
        metrics_file: Path,
    ) -> None:
        self.binary = binary
        self.robot_ip = robot_ip
        self.sample_file = sample_file
        self.metrics_file = metrics_file
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sample: tuple[int, np.ndarray] | None = None
        self._error: BaseException | None = None
        self._thread = threading.Thread(target=self._tail, name="shadow-jaka-tail", daemon=True)
        self.process: subprocess.Popen[str] | None = None
        self.sample_count = 0

    def start(self) -> None:
        self.sample_file.parent.mkdir(parents=True, exist_ok=True)
        self.sample_file.unlink(missing_ok=True)
        self.metrics_file.unlink(missing_ok=True)
        self.process = subprocess.Popen(
            [
                str(self.binary),
                "--mode",
                "connected",
                "--robot-ip",
                self.robot_ip,
                "--duration-s",
                "600",
                "--poll-hz",
                "30",
                "--slow-poll-hz",
                "1",
                "--max-samples",
                "100000",
                "--sample-stream-file",
                str(self.sample_file),
                "--metrics-file",
                str(self.metrics_file),
            ],
            text=True,
        )
        self._thread.start()

    def _tail(self) -> None:
        try:
            deadline = time.monotonic() + 15.0
            while not self.sample_file.exists():
                if self.process is not None and self.process.poll() is not None:
                    raise RuntimeError(f"JAKA read-only diagnostic exited {self.process.returncode}")
                if time.monotonic() >= deadline:
                    raise TimeoutError("JAKA sample stream was not created")
                time.sleep(0.02)
            with self.sample_file.open("r", encoding="utf-8") as handle:
                partial_line = ""
                while not self._stop.is_set():
                    line = handle.readline()
                    if not line:
                        if self.process is not None and self.process.poll() is not None:
                            raise RuntimeError(f"JAKA read-only diagnostic exited {self.process.returncode}")
                        time.sleep(0.002)
                        continue
                    if not line.endswith("\n"):
                        partial_line += line
                        continue
                    line = partial_line + line
                    partial_line = ""
                    row = json.loads(line)
                    joints = np.asarray(row["joint_position_rad"], dtype=np.float64)
                    if joints.shape != (6,) or not np.isfinite(joints).all():
                        raise ValueError("invalid JAKA measured joint sample")
                    with self._lock:
                        self._sample = (int(row["host_monotonic_ns"]), joints)
                        self.sample_count += 1
        except BaseException as exc:
            with self._lock:
                self._error = exc

    def latest(self) -> tuple[int, np.ndarray]:
        with self._lock:
            if self._error is not None:
                raise RuntimeError("JAKA read-only stream failed") from self._error
            if self._sample is None:
                raise RuntimeError("JAKA measured state is not yet available")
            return self._sample[0], self._sample[1].copy()

    def wait_ready(self, timeout_sec: float = 15.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                self.latest()
                return
            except RuntimeError:
                time.sleep(0.02)
        self.latest()

    def stop(self) -> None:
        self._stop.set()
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=3.0)
        self._thread.join(timeout=2.0)


class RH56ReadOnlyStream:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.backend = RH56SerialBackend(config)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="shadow-rh56-readonly", daemon=True)
        self._angle: tuple[int, np.ndarray] | None = None
        self._force: tuple[int, np.ndarray] | None = None
        self._error: BaseException | None = None
        self.successes = {name: 0 for name in ("ANGLE", "FORCE", "CURRENT", "STATUS", "ERROR")}
        self.calibration = {
            name: HandDofCalibration(**self.config["hand_schema"]["dof_calibration"][name])
            for name in CANONICAL_HAND_ORDER
        }

    def start(self) -> None:
        self.backend.connect()
        self._thread.start()

    def _run(self) -> None:
        rates = {
            "ANGLE": float(self.config["scheduler"]["angle_feedback_rate_hz"]),
            "FORCE": float(self.config["scheduler"]["force_feedback_rate_hz"]),
            "CURRENT": float(self.config["scheduler"]["current_feedback_rate_hz"]),
            "STATUS": float(self.config["scheduler"]["status_feedback_rate_hz"]),
            "ERROR": float(self.config["scheduler"]["error_feedback_rate_hz"]),
        }
        due = {name: time.monotonic_ns() for name in rates}
        try:
            while not self._stop.is_set():
                now_ns = time.monotonic_ns()
                ready = [name for name, deadline in due.items() if now_ns >= deadline]
                if not ready:
                    time.sleep(min(0.001, (min(due.values()) - now_ns) / 1e9))
                    continue
                priority = {"ANGLE": 0, "STATUS": 1, "ERROR": 2, "CURRENT": 3, "FORCE": 4}
                name = min(ready, key=lambda item: (due[item], priority[item]))
                if name == "ANGLE":
                    raw = self.backend.get_canonical_angles()
                    value = np.asarray(
                        normalize_raw(
                            raw,
                            raw_order=CANONICAL_HAND_ORDER,
                            calibration=self.calibration,
                        ),
                        dtype=np.float64,
                    )
                    timestamp_ns = time.monotonic_ns()
                    with self._lock:
                        self._angle = (timestamp_ns, value)
                elif name == "FORCE":
                    value = np.asarray(self.backend.get_canonical_forces(), dtype=np.float64)
                    timestamp_ns = time.monotonic_ns()
                    with self._lock:
                        self._force = (timestamp_ns, value)
                elif name == "CURRENT":
                    self.backend.get_canonical_currents()
                elif name == "STATUS":
                    self.backend.read_register(self.backend.REG["STATUS"], 6)
                else:
                    errors = self.backend.read_register(self.backend.REG["ERROR"], 6)
                    if any(errors):
                        raise RuntimeError(f"RH56 reported nonzero ERROR during read-only shadow: {errors}")
                self.successes[name] += 1
                completed_ns = time.monotonic_ns()
                period_ns = int(round(1e9 / rates[name]))
                next_due = due[name] + period_ns
                if next_due <= completed_ns:
                    next_due += ((completed_ns - next_due) // period_ns + 1) * period_ns
                due[name] = next_due
        except BaseException as exc:
            with self._lock:
                self._error = exc

    def latest(self) -> tuple[int, np.ndarray, int | None, np.ndarray | None]:
        with self._lock:
            if self._error is not None:
                raise RuntimeError("RH56 read-only stream failed") from self._error
            if self._angle is None:
                raise RuntimeError("RH56 ANGLE_ACT is not yet available")
            force_timestamp = None if self._force is None else self._force[0]
            force = None if self._force is None else self._force[1].copy()
            return self._angle[0], self._angle[1].copy(), force_timestamp, force

    def wait_ready(self, timeout_sec: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                angle = self.latest()
                if angle[3] is not None:
                    return
            except RuntimeError:
                pass
            time.sleep(0.02)
        self.latest()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.backend.close_port()
        if self.backend.register_write_count != 0:
            raise RuntimeError("RH56 read-only shadow attempted a register write")


def _load_training_envelope(path: Path) -> tuple[np.ndarray, np.ndarray]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    stats = payload.get("action_stats", payload.get("action", payload))
    minimum = stats.get("min", stats.get("minimum"))
    maximum = stats.get("max", stats.get("maximum"))
    if minimum is None or maximum is None:
        raise ValueError(f"{path} does not contain action min/max statistics")
    return (
        np.asarray(minimum, dtype=np.float64),
        np.asarray(maximum, dtype=np.float64),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-socket", type=Path, required=True)
    parser.add_argument("--workspace-camera-serial", required=True)
    parser.add_argument("--wrist-camera-serial", required=True)
    parser.add_argument("--jaka-ip", required=True)
    parser.add_argument("--jaka-readonly-binary", type=Path, required=True)
    parser.add_argument("--rh56-config", type=Path, required=True)
    parser.add_argument("--rh56-device", required=True)
    parser.add_argument("--allow-direct-ch341-device", action="store_true")
    parser.add_argument("--training-validation", type=Path, required=True)
    parser.add_argument("--queries", type=int, default=300)
    parser.add_argument("--warmup-queries", type=int, default=20)
    parser.add_argument("--query-rate-hz", type=float, default=30.0)
    parser.add_argument(
        "--environment-state-input",
        action="store_true",
        help="send the six raw FORCE_ACT values to an ACT+Force model worker",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not (300 <= args.queries <= 1000):
        raise SystemExit("--queries must be within [300,1000]")
    if not (0 < args.query_rate_hz <= 30):
        raise SystemExit("--query-rate-hz must be in (0,30]")
    if args.workspace_camera_serial == args.wrist_camera_serial:
        raise SystemExit("workspace and wrist camera serials must differ")

    args.output.mkdir(parents=True, exist_ok=False)
    with args.rh56_config.open("r", encoding="utf-8") as handle:
        rh56_config = yaml.safe_load(handle)
    identity = inspect_serial_device(
        args.rh56_device, allow_direct_ch341=args.allow_direct_ch341_device
    )
    if identity["occupied_pids"]:
        raise SystemExit(f"RH56 serial device in use by PIDs {identity['occupied_pids']}")
    rh56_config["serial"]["port"] = args.rh56_device

    workspace = CameraReader("workspace", args.workspace_camera_serial)
    wrist = CameraReader("wrist", args.wrist_camera_serial)
    jaka = JakaReadOnlyStream(
        args.jaka_readonly_binary,
        args.jaka_ip,
        args.output / "jaka_samples.jsonl",
        args.output / "jaka_readonly_metrics.json",
    )
    rh56 = RH56ReadOnlyStream(rh56_config)
    connection: socket.socket | None = None
    predictions: list[np.ndarray] = []
    states: list[np.ndarray] = []
    forces: list[np.ndarray] = []
    query_started_ns: list[int] = []
    timing: dict[str, list[float]] = {
        name: []
        for name in (
            "observation_acquisition",
            "image_preprocessing",
            "state_assembly",
            "checkpoint_preprocessing",
            "act_inference",
            "checkpoint_postprocessing",
            "model_roundtrip",
            "total_end_to_end",
            "workspace_age",
            "wrist_age",
            "jaka_state_age",
            "rh56_state_age",
            "camera_pair_skew",
            "policy_request_serialization",
            "policy_socket_send",
            "policy_socket_receive",
            "policy_response_deserialization",
            "policy_transport_metadata_receive",
            "worker_receive_wait",
            "worker_receive_decode",
            "worker_batch_wrap",
            "worker_response_serialization",
            "worker_socket_send",
            "worker_output_to_host",
        )
    }
    stale_counts = {"workspace": 0, "wrist": 0, "jaka": 0, "rh56": 0}
    repeated_counts = {"jaka": 0, "rh56": 0}
    last_jaka_timestamp: int | None = None
    last_rh56_timestamp: int | None = None
    previous_workspace_sequence: int | None = None
    previous_wrist_sequence: int | None = None
    cpu_samples: list[float] = []
    last_observation: dict[str, np.ndarray] | None = None
    deterministic_equal = False
    deterministic_max_abs_diff: float | None = None
    prediction_shape: tuple[int, int] | None = None
    try:
        workspace.start()
        wrist.start()
        jaka.start()
        rh56.start()
        workspace_sample = workspace.wait_after(None, 15.0)
        wrist_sample = wrist.wait_after(None, 15.0)
        jaka.wait_ready()
        rh56.wait_ready()
        previous_workspace_sequence = workspace_sample.sequence
        previous_wrist_sequence = wrist_sample.sequence
        connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        connection.connect(str(args.model_socket))
        try:
            import psutil
            psutil.cpu_percent(interval=None)
        except ImportError:
            psutil = None

        total_iterations = args.warmup_queries + args.queries
        next_due_ns = time.monotonic_ns()
        period_ns = int(round(1e9 / args.query_rate_hz))
        for iteration in range(total_iterations):
            now_ns = time.monotonic_ns()
            if now_ns < next_due_ns:
                time.sleep((next_due_ns - now_ns) / 1e9)
            query_start_ns = time.monotonic_ns()
            acquire_started_ns = time.perf_counter_ns()
            workspace_sample = workspace.wait_after(previous_workspace_sequence)
            wrist_sample = wrist.wait_after(previous_wrist_sequence)
            previous_workspace_sequence = workspace_sample.sequence
            previous_wrist_sequence = wrist_sample.sequence
            acquire_ended_ns = time.perf_counter_ns()

            image_started_ns = time.perf_counter_ns()
            workspace_chw = preprocess_live_rgb(workspace_sample.rgb)
            wrist_chw = preprocess_live_rgb(wrist_sample.rgb)
            image_ended_ns = time.perf_counter_ns()

            state_started_ns = time.perf_counter_ns()
            jaka_timestamp, jaka_state = jaka.latest()
            rh56_timestamp, rh56_state, _, force = rh56.latest()
            state = np.concatenate([jaka_state, rh56_state]).astype(np.float32)
            if state.shape != (12,) or not np.isfinite(state).all():
                raise ValueError("assembled live state is not finite [12]")
            state_ended_ns = time.perf_counter_ns()
            observation_ready_ns = time.monotonic_ns()
            observation = {
                WORKSPACE_KEY: workspace_chw,
                WRIST_KEY: wrist_chw,
                STATE_KEY: state,
            }
            if args.environment_state_input:
                if force is None:
                    raise RuntimeError("ACT+Force shadow requires a fresh FORCE_ACT sample")
                observation[ENVIRONMENT_STATE_KEY] = np.asarray(force, dtype=np.float32)
            roundtrip_started_ns = time.perf_counter_ns()
            response, host_request_timing = _timed_model_request(connection, observation)
            roundtrip_ended_ns = time.perf_counter_ns()
            if "error" in response:
                raise RuntimeError(response["error"])
            prediction = np.asarray(response["prediction"], dtype=np.float32)
            if prediction.ndim != 2 or prediction.shape[1] != 12 or not np.isfinite(prediction).all():
                raise ValueError(f"invalid shadow prediction [H,12]: {prediction.shape}")
            if prediction_shape is None:
                prediction_shape = (int(prediction.shape[0]), int(prediction.shape[1]))
            elif prediction.shape != prediction_shape:
                raise ValueError(
                    f"shadow prediction shape changed from {prediction_shape} to {prediction.shape}"
                )
            response_ended_ns = time.perf_counter_ns()
            last_observation = observation

            if iteration >= args.warmup_queries:
                query_started_ns.append(query_start_ns)
                predictions.append(prediction)
                states.append(state)
                forces.append(
                    np.full(6, np.nan, dtype=np.float64) if force is None else force
                )
                timing["observation_acquisition"].append((acquire_ended_ns - acquire_started_ns) / 1e6)
                timing["image_preprocessing"].append((image_ended_ns - image_started_ns) / 1e6)
                timing["state_assembly"].append((state_ended_ns - state_started_ns) / 1e6)
                timing["model_roundtrip"].append((roundtrip_ended_ns - roundtrip_started_ns) / 1e6)
                timing["total_end_to_end"].append((response_ended_ns - acquire_started_ns) / 1e6)
                for name, value in host_request_timing.items():
                    timing[name].append(float(value))
                for name in (
                    "checkpoint_preprocessing",
                    "act_inference",
                    "checkpoint_postprocessing",
                    "worker_receive_wait",
                    "worker_receive_decode",
                    "worker_batch_wrap",
                    "worker_response_serialization",
                    "worker_socket_send",
                    "worker_output_to_host",
                ):
                    if name in response["timing_ms"]:
                        timing[name].append(float(response["timing_ms"][name]))
                ages = {
                    "workspace": (observation_ready_ns - workspace_sample.host_monotonic_ns) / 1e6,
                    "wrist": (observation_ready_ns - wrist_sample.host_monotonic_ns) / 1e6,
                    "jaka": (observation_ready_ns - jaka_timestamp) / 1e6,
                    "rh56": (observation_ready_ns - rh56_timestamp) / 1e6,
                }
                timing["workspace_age"].append(ages["workspace"])
                timing["wrist_age"].append(ages["wrist"])
                timing["jaka_state_age"].append(ages["jaka"])
                timing["rh56_state_age"].append(ages["rh56"])
                timing["camera_pair_skew"].append(
                    abs(workspace_sample.host_monotonic_ns - wrist_sample.host_monotonic_ns) / 1e6
                )
                thresholds_ms = {
                    "workspace": 2 * 1000 / 30,
                    "wrist": 2 * 1000 / 30,
                    "jaka": 2 * 1000 / 30,
                    "rh56": 2 * 1000 / 15,
                }
                for name, age in ages.items():
                    stale_counts[name] += int(age > thresholds_ms[name])
                repeated_counts["jaka"] += int(last_jaka_timestamp == jaka_timestamp)
                repeated_counts["rh56"] += int(last_rh56_timestamp == rh56_timestamp)
                last_jaka_timestamp = jaka_timestamp
                last_rh56_timestamp = rh56_timestamp
                if psutil is not None:
                    cpu_samples.append(float(psutil.cpu_percent(interval=None)))
            next_due_ns += period_ns
            if next_due_ns < time.monotonic_ns() - period_ns:
                next_due_ns = time.monotonic_ns()

        assert last_observation is not None
        first_repeat = _model_request(connection, last_observation)["prediction"]
        second_repeat = _model_request(connection, last_observation)["prediction"]
        repeat_delta = np.abs(np.asarray(first_repeat) - np.asarray(second_repeat))
        deterministic_equal = bool(np.array_equal(first_repeat, second_repeat))
        deterministic_max_abs_diff = float(np.max(repeat_delta))
        _model_request(connection, {"command": "stop"})
    finally:
        if connection is not None:
            connection.close()
        workspace.stop()
        wrist.stop()
        jaka.stop()
        rh56.stop()

    prediction_array = np.stack(predictions)
    state_array = np.stack(states)
    force_array = np.stack(forces)
    training_min, training_max = _load_training_envelope(args.training_validation)
    outside_training = (prediction_array < training_min) | (prediction_array > training_max)
    hardware_min = np.concatenate([JAKA_LOWER, np.zeros(6)])
    hardware_max = np.concatenate([JAKA_UPPER, np.ones(6)])
    outside_hardware = (prediction_array < hardware_min) | (prediction_array > hardware_max)
    consecutive_first_step = np.abs(np.diff(prediction_array[:, 0, :], axis=0))
    overlap = np.abs(prediction_array[:-1, 1:, :] - prediction_array[1:, :-1, :])
    query_intervals_ms = np.diff(np.asarray(query_started_ns, dtype=np.int64)) / 1e6
    np.savez_compressed(
        args.output / "shadow_predictions.npz",
        predictions=prediction_array,
        states=state_array,
        force_act=force_array,
        query_started_ns=np.asarray(query_started_ns, dtype=np.int64),
    )
    report = {
        "schema_version": "act_live_shadow.v1",
        "command_disabled": True,
        "command_api_present": False,
        "environment_state_input": args.environment_state_input,
        "queries": len(predictions),
        "warmup_queries": args.warmup_queries,
        "requested_query_rate_hz": args.query_rate_hz,
        "achieved_query_rate_hz": (
            None
            if len(query_started_ns) < 2
            else (len(query_started_ns) - 1) * 1e9 / (query_started_ns[-1] - query_started_ns[0])
        ),
        "query_start_interval_ms": _distribution(query_intervals_ms.tolist()),
        "output_shape": list(prediction_array.shape),
        "per_query_output_shape": list(prediction_shape) if prediction_shape is not None else None,
        "finite_output_failures": int((~np.isfinite(prediction_array)).any(axis=(1, 2)).sum()),
        "timing_ms": {name: _distribution(values) for name, values in timing.items()},
        "camera": {"workspace": workspace.summary(), "wrist": wrist.summary()},
        "state": {
            "ordering": [
                "JAKA_J1",
                "JAKA_J2",
                "JAKA_J3",
                "JAKA_J4",
                "JAKA_J5",
                "JAKA_J6",
                "RH56_index",
                "RH56_middle",
                "RH56_ring",
                "RH56_pinky",
                "RH56_thumb_close",
                "RH56_thumb_lateral",
            ],
            "shape": [12],
            "stale_threshold_definition": "age greater than two configured source periods",
            "stale_counts": stale_counts,
            "repeated_source_timestamp_counts": repeated_counts,
            "jaka_readonly_samples": jaka.sample_count,
            "rh56_read_successes": rh56.successes,
            "rh56_register_writes": rh56.backend.register_write_count,
        },
        "prediction_native_space": {
            "jaka_min": prediction_array[:, :, :6].min(axis=(0, 1)).tolist(),
            "jaka_max": prediction_array[:, :, :6].max(axis=(0, 1)).tolist(),
            "rh56_min": prediction_array[:, :, 6:].min(axis=(0, 1)).tolist(),
            "rh56_max": prediction_array[:, :, 6:].max(axis=(0, 1)).tolist(),
            "training_min": training_min.tolist(),
            "training_max": training_max.tolist(),
            "fraction_outside_training_envelope_total": float(outside_training.mean()),
            "fraction_outside_training_envelope_per_dimension": outside_training.mean(axis=(0, 1)).tolist(),
            "hardware_min": hardware_min.tolist(),
            "hardware_max": hardware_max.tolist(),
            "fraction_outside_hardware_limits_total": float(outside_hardware.mean()),
            "fraction_outside_hardware_limits_per_dimension": outside_hardware.mean(axis=(0, 1)).tolist(),
            "unclamped": True,
        },
        "temporal_smoothness": {
            "consecutive_query_first_action_abs_delta_mean_per_dimension": consecutive_first_step.mean(axis=0).tolist(),
            "consecutive_query_first_action_abs_delta_p95_per_dimension": np.quantile(consecutive_first_step, 0.95, axis=0).tolist(),
            "consecutive_query_first_action_abs_delta_max_per_dimension": consecutive_first_step.max(axis=0).tolist(),
            "overlapping_chunk_alignment_mae": float(overlap.mean()),
        },
        "deterministic_fixed_observation": {
            "identical": deterministic_equal,
            "max_abs_diff": deterministic_max_abs_diff,
        },
        "cpu_utilization_percent": _distribution(cpu_samples),
        "rh56_device_identity": identity,
        "preprocessing_contract": {
            "camera_input_format": "RealSense RGB8; no BGR conversion",
            "color_conversion_count": 0,
            "resize": "640x480 to 320x240 with OpenCV INTER_AREA",
            "tensor": "float32 CHW divided by 255 once, then saved checkpoint processor",
        },
        "artifacts": {
            "predictions": str(args.output / "shadow_predictions.npz"),
            "jaka_samples": str(args.output / "jaka_samples.jsonl"),
            "jaka_metrics": str(args.output / "jaka_readonly_metrics.json"),
        },
    }
    (args.output / "shadow_benchmark.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
