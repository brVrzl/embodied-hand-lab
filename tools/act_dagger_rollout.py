#!/usr/bin/env python3
"""ACT/Quest DAgger rollout entry point and offline arbitration smoke test.

The physical path owns the ACT worker, both cameras, Quest receiver, JAKA
native worker, RH56 worker, and episode recorder in one process.  Quest is
used only to produce an expert candidate; :class:`DaggerCoordinator` is the
single boundary that chooses the command source.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
from typing import Any, Mapping, Sequence

from teleoperation.dagger_arbiter import (
    DaggerAction,
    DaggerActionArbiter,
    DaggerActionSource,
    DaggerDecision,
    DaggerState,
)


EXPERT_CONTROL_RATE_HZ = 60.0


class DaggerProvenanceWriter:
    """Bounded asynchronous JSONL writer for policy/expert provenance."""

    def __init__(self, path: str | Path, *, capacity: int = 512) -> None:
        if capacity <= 0:
            raise ValueError("provenance writer capacity must be positive")
        self.path = Path(path).resolve()
        self.capacity = int(capacity)
        self._queue: queue.Queue[dict[str, Any] | None] = queue.Queue(
            maxsize=self.capacity
        )
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="dagger-provenance-writer",
            daemon=True,
        )
        self._file: Any | None = None
        self._started = False
        self._finished = False
        self.error: str | None = None
        self.drop_count = 0
        self.write_count = 0

    def start(self) -> None:
        if self._started:
            raise RuntimeError("provenance writer already started")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("x", encoding="utf-8")
        self._started = True
        self._thread.start()

    def write(self, record: Mapping[str, Any]) -> bool:
        if not self._started or self._finished:
            raise RuntimeError("provenance writer is not active")
        value = dict(record)
        try:
            self._queue.put_nowait(value)
        except queue.Full:
            self.drop_count += 1
            return False
        return True

    def submit(self, kind: str, record: Mapping[str, Any]) -> bool:
        """Accept the query-writer interface used by the ACT inference thread."""

        if not kind.strip():
            raise ValueError("provenance record kind must not be empty")
        return self.write({"record_type": kind, **dict(record)})

    def finish(self) -> None:
        if not self._started or self._finished:
            return
        self._finished = True
        self._queue.put(None, timeout=5.0)
        self._thread.join(timeout=8.0)
        if self._thread.is_alive():
            raise RuntimeError("provenance writer did not stop")
        if self.error is not None:
            raise RuntimeError(self.error)

    def _run(self) -> None:
        assert self._file is not None
        try:
            while True:
                record = self._queue.get()
                try:
                    if record is None:
                        return
                    self._file.write(
                        json.dumps(
                            record,
                            ensure_ascii=False,
                            allow_nan=False,
                            separators=(",", ":"),
                            sort_keys=True,
                        )
                        + "\n"
                    )
                    self._file.flush()
                    self.write_count += 1
                finally:
                    self._queue.task_done()
        except BaseException as exc:
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            self._file.close()


def _physical_episode_release_ready(
    *,
    arm_clutch_state: str,
    hand_clutch_state: str,
    latest_event: Mapping[str, object],
    dispatch_failed: bool,
) -> bool:
    """Apply the unchanged physical-teleop release gate inside DAgger."""

    release_inputs_valid = bool(
        latest_event.get("right_wrist_valid")
        and latest_event.get("hand_skeleton_valid")
        and not latest_event.get("input_recovery_active")
        and not dispatch_failed
    )
    return (
        arm_clutch_state == "disengaged"
        and hand_clutch_state == "disengaged"
        and release_inputs_valid
    )


def _dagger_control_period_ns(
    state: DaggerState,
    *,
    policy_rate_hz: float,
) -> int:
    """Return the state-owned outer period without changing ACT semantics."""

    rate_hz = (
        float(policy_rate_hz)
        if state is DaggerState.POLICY_RUN
        else EXPERT_CONTROL_RATE_HZ
    )
    if not math.isfinite(rate_hz) or rate_hz <= 0.0:
        raise ValueError("control rate must be positive and finite")
    return int(round(1e9 / rate_hz))


class _DaggerEventLog:
    """Bounded non-blocking event JSONL sink copied from physical teleop."""

    def __init__(self, path: Path, *, capacity: int = 256) -> None:
        self.path = path
        self._queue: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=capacity)
        self._file: Any | None = None
        self._thread = threading.Thread(
            target=self._run, name="dagger-event-log", daemon=True
        )
        self.started = False
        self.drop_count = 0
        self.error_count = 0

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        try:
            self._file = self.path.open("x", encoding="utf-8")
            self._thread.start()
            self.started = True
        except BaseException:
            self.error_count += 1
            if self._file is not None:
                self._file.close()
            self._file = None

    def write(self, record: Mapping[str, Any]) -> None:
        if not self.started:
            return
        try:
            self._queue.put_nowait(dict(record))
        except queue.Full:
            self.drop_count += 1

    def close(self) -> None:
        if not self.started:
            return
        try:
            self._queue.put(None, timeout=1.0)
        except queue.Full:
            self.drop_count += self._queue.qsize()
        self._thread.join(timeout=1.0)
        if self._thread.is_alive():
            self.error_count += 1
        if self._file is not None:
            try:
                self._file.close()
            except BaseException:
                self.error_count += 1
        self._file = None
        self.started = False

    def _run(self) -> None:
        assert self._file is not None
        while True:
            record = self._queue.get()
            try:
                if record is None:
                    return
                self._file.write(
                    json.dumps(
                        record,
                        sort_keys=True,
                        ensure_ascii=False,
                        default=str,
                    )
                    + "\n"
                )
                self._file.flush()
            except BaseException:
                self.error_count += 1
            finally:
                self._queue.task_done()


def _dagger_task_placement(
    *, component: str, process_id: int, thread_id: int, thread_name: str
) -> dict[str, object]:
    result: dict[str, object] = {
        "component": component,
        "process_id": process_id,
        "thread_id": thread_id,
        "thread_name": thread_name,
    }
    if not (
        sys.platform.startswith("linux")
        and hasattr(os, "sched_getaffinity")
        and hasattr(os, "sched_getscheduler")
        and hasattr(os, "sched_getparam")
    ):
        result.update({"supported": False, "reason": "Linux scheduling telemetry unavailable"})
        return result
    try:
        stat = Path(f"/proc/{process_id}/task/{thread_id}/stat").read_text(
            encoding="utf-8"
        )
        closing = stat.rfind(")")
        fields = stat[closing + 2 :].split()
        result.update(
            {
                "current_cpu": int(fields[36]),
                "scheduler_policy": int(os.sched_getscheduler(thread_id)),
                "scheduler_priority": int(os.sched_getparam(thread_id).sched_priority),
                "nice_value": int(os.getpriority(os.PRIO_PROCESS, thread_id)),
                "affinity_mask": sorted(os.sched_getaffinity(thread_id)),
                "supported": True,
            }
        )
    except (IndexError, OSError, ValueError) as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    return result


def _dagger_component_snapshot(
    *, boundary: str, native: Any, receiver: Any | None, rh56_worker: Any | None
) -> dict[str, object]:
    process_id = os.getpid()
    known_threads: dict[int, tuple[str, str]] = {
        threading.get_native_id(): ("python_dagger_wrapper", "main"),
    }
    if receiver is not None and receiver.thread.native_id is not None:
        known_threads[int(receiver.thread.native_id)] = (
            "quest_receiver", receiver.thread.name
        )
    if rh56_worker is not None and rh56_worker.native_thread_id is not None:
        known_threads[int(rh56_worker.native_thread_id)] = (
            "rh56_serial_and_logging", "rh56-pc-direct"
        )
    try:
        task_ids = sorted(
            int(path.name) for path in Path(f"/proc/{process_id}/task").iterdir()
        )
    except OSError:
        task_ids = sorted(known_threads)
    tasks = []
    for thread_id in task_ids:
        component, name = known_threads.get(
            thread_id, ("other_python_thread", "unknown")
        )
        tasks.append(
            _dagger_task_placement(
                component=component,
                process_id=process_id,
                thread_id=thread_id,
                thread_name=name,
            )
        )
    if getattr(native, "process", None) is not None:
        tasks.append(
            _dagger_task_placement(
                component="native_jaka_worker_process",
                process_id=native.process.pid,
                thread_id=native.process.pid,
                thread_name="native-main-control",
            )
        )
    return {
        "boundary": boundary,
        "monotonic_ns": time.monotonic_ns(),
        "tasks": tasks,
        "logging_execution": "diagnostic_async_only",
    }


def _dagger_timing_summary(
    rows: Sequence[Mapping[str, float]],
) -> dict[str, dict[str, float]]:
    result: dict[str, dict[str, float]] = {}
    for name in sorted({name for row in rows for name in row}):
        values = sorted(float(row[name]) for row in rows if name in row)
        if not values:
            continue

        def percentile(percent: float) -> float:
            position = (len(values) - 1) * percent / 100.0
            lower = int(math.floor(position))
            upper = int(math.ceil(position))
            if lower == upper:
                return values[lower]
            fraction = position - lower
            return values[lower] * (1.0 - fraction) + values[upper] * fraction

        result[name] = {
            "count": len(values),
            "mean": sum(values) / len(values),
            "p50": percentile(50.0),
            "p95": percentile(95.0),
            "p99": percentile(99.0),
            "max": values[-1],
        }
    return result


def _write_dagger_event_extract(
    event_path: Path,
    output_path: Path,
    *,
    native_telemetry_path: Path | None = None,
) -> None:
    """Write event and native-cycle windows without touching control I/O."""

    selected: list[dict[str, Any]] = []
    try:
        event_lines = event_path.read_text(encoding="utf-8").splitlines()
    except (FileNotFoundError, OSError):
        event_lines = []
    event_rows: list[dict[str, Any]] = []
    for line in event_lines:
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if not isinstance(row, dict):
            continue
        event_rows.append(row)
        focus: list[str] = []
        if row.get("collection_boundary_event"):
            focus.append("collection_boundary")
        if row.get("clutch_edge_reason"):
            focus.append("clutch_edge")
        if row.get("dispatch_failed"):
            focus.append("dispatch_failed")
        if row.get("controller_fault"):
            focus.append("controller_fault")
        if row.get("native_output_acceleration_hold"):
            focus.append("native_output_acceleration_hold")
        if row.get("native_output_acceleration_recovered"):
            focus.append("native_output_acceleration_recovered")
        if focus:
            selected.append({"focus_events": focus, "event": row})
    if native_telemetry_path is not None:
        try:
            telemetry_rows = [
                json.loads(line)
                for line in native_telemetry_path.read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
        except (FileNotFoundError, OSError, json.JSONDecodeError):
            telemetry_rows = []
        event_focus: list[tuple[int, str]] = []
        for row in event_rows:
            if not isinstance(row, Mapping):
                continue
            timestamp = row.get("timestamp_ns")
            if not isinstance(timestamp, (int, float)):
                continue
            focus = []
            if row.get("collection_boundary_event"):
                focus.append("collection_boundary")
            if row.get("clutch_edge_reason"):
                focus.append("clutch_edge")
            if row.get("dispatch_failed"):
                focus.append("dispatch_failed")
            if row.get("controller_fault"):
                focus.append("controller_fault")
            if row.get("native_output_acceleration_hold"):
                focus.append("native_output_acceleration_hold")
            if row.get("native_output_acceleration_recovered"):
                focus.append("native_output_acceleration_recovered")
            event_focus.extend((int(timestamp), name) for name in focus)
        for telemetry in telemetry_rows:
            if not isinstance(telemetry, Mapping):
                continue
            timestamp = telemetry.get("host_monotonic_ns")
            if not isinstance(timestamp, (int, float)):
                continue
            focus = sorted(
                {
                    name
                    for event_timestamp, name in event_focus
                    if -2_000_000_000 <= int(timestamp) - event_timestamp <= 1_000_000_000
                }
            )
            if focus:
                selected.append(
                    {"focus_events": focus, "telemetry": dict(telemetry)}
                )
    output_path.write_text(
        "".join(
            json.dumps(row, sort_keys=True, ensure_ascii=False, default=str) + "\n"
            for row in selected
        ),
        encoding="utf-8",
    )


def _dagger_native_arguments(
    native_arguments: Any,
    runtime: Mapping[str, Any],
    config: Any,
    target_socket: Path,
    status_socket: Path,
    metrics: Path,
    telemetry: Path | None,
    duration_sec: float,
) -> list[str]:
    """Reuse ACT's native safety arguments and optional diagnostic telemetry."""

    arguments = native_arguments(
        dict(runtime), config, target_socket, status_socket, metrics, duration_sec
    )
    if telemetry is not None:
        arguments.extend(("--cycle-telemetry-file", str(telemetry)))
    return arguments


def _wait_for_native_target_acceptance(
    runtime_arm: Any,
    native: Any,
    target_sequence: int,
    *,
    timeout_s: float = 2.0,
) -> Any:
    """Wait until the native worker has consumed a startup target.

    The target transport is latest-only.  Starting ACT inference immediately
    after publishing the startup hold can therefore replace that hold before
    the native worker sees it.  The worker's acknowledged sequence is the
    synchronization barrier that makes the first policy target safe to send.
    """

    if target_sequence < 1:
        raise ValueError("startup target sequence must be positive")
    if timeout_s <= 0.0:
        raise ValueError("startup target acknowledgement timeout must be positive")

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = runtime_arm.latest_status()
        if status is not None and int(status.last_sequence) >= target_sequence:
            return status
        process = getattr(native, "process", None)
        if process is not None and process.poll() is not None:
            raise RuntimeError(
                "JAKA worker exited before acknowledging startup alignment target"
            )
        time.sleep(0.005)
    raise RuntimeError("JAKA worker did not acknowledge startup alignment target")


@dataclass(frozen=True, slots=True)
class DaggerTick:
    timestamp_ns: int
    decision: DaggerDecision


class DaggerCoordinator:
    """Coordinate one arbitration tick and persist its provenance."""

    def __init__(
        self,
        arbiter: DaggerActionArbiter | None = None,
        *,
        provenance: DaggerProvenanceWriter | None = None,
    ) -> None:
        self.arbiter = arbiter or DaggerActionArbiter()
        self.provenance = provenance
        self.tick_count = 0

    def request_takeover(self, now_ns: int, *, reason: str = "operator_takeover") -> str:
        return self.arbiter.request_takeover(now_ns, reason=reason)

    def observe_clutch(
        self,
        now_ns: int,
        *,
        released: bool,
        pressed: bool,
        valid: bool,
    ) -> bool:
        return self.arbiter.observe_clutch(
            now_ns,
            released=released,
            pressed=pressed,
            valid=valid,
        )

    def mark_expert_ready(self, now_ns: int) -> None:
        self.arbiter.mark_expert_ready(now_ns)

    def safe_stop(self, now_ns: int, *, reason: str) -> None:
        self.arbiter.safe_stop(now_ns, reason=reason)

    def complete(self, now_ns: int, *, reason: str = "task_complete") -> None:
        self.arbiter.complete(now_ns, reason=reason)

    def resume_policy(
        self,
        now_ns: int,
        *,
        reason: str = "expert_episode_complete",
    ) -> None:
        self.arbiter.resume_policy(now_ns, reason=reason)

    def tick(
        self,
        now_ns: int,
        *,
        policy: DaggerAction | None,
        expert: DaggerAction | None,
        hold: DaggerAction | None,
        policy_query_sequence: int | None = None,
        metadata: Mapping[str, Any] | None = None,
    ) -> DaggerTick:
        if now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        decision = self.arbiter.select(policy=policy, expert=expert, hold=hold)
        self.tick_count += 1
        if self.provenance is not None:
            self.provenance.write(
                {
                    "record_type": "control_tick",
                    "timestamp_ns": int(now_ns),
                    "tick_index": self.tick_count - 1,
                    "dagger_state": decision.state.value,
                    "intervention_id": decision.intervention_id,
                    "action_source": (
                        None
                        if decision.action is None
                        else decision.action.source.value
                    ),
                    "action_reason": decision.reason,
                    "policy_query_sequence": policy_query_sequence,
                    "policy_action": None if policy is None else list(policy.vector),
                    "expert_action": None if expert is None else list(expert.vector),
                    "hold_action": None if hold is None else list(hold.vector),
                    "selected_action": (
                        None
                        if decision.action is None
                        else list(decision.action.vector)
                    ),
                    "action_label_valid": bool(
                        decision.action is not None
                        and decision.action.source is DaggerActionSource.EXPERT
                    ),
                    **dict(metadata or {}),
                }
            )
        return DaggerTick(timestamp_ns=now_ns, decision=decision)


class _Rh56OwnershipDispatch:
    """Keep the physical RH56 worker active across DAgger owner changes."""

    def __init__(self, worker: Any) -> None:
        self.worker = worker
        self.active = False

    def activate_from_measured(self, monotonic_ns: int) -> tuple[float, ...]:
        measured = self.worker.activate_from_measured(monotonic_ns)
        self.active = True
        return tuple(float(value) for value in measured)

    def submit_target(self, target: Sequence[float], monotonic_ns: int) -> None:
        if not self.active:
            self.activate_from_measured(monotonic_ns)
        self.worker.submit_target(target, monotonic_ns)

    def hold(self, reason: str) -> None:
        self.worker.hold(reason)
        self.active = False


@dataclass(frozen=True, slots=True)
class _PolicyCameraSample:
    """The small camera contract consumed by ``InferenceThread``."""

    sequence: int
    frame_number: int
    device_timestamp_ms: float
    host_monotonic_ns: int
    rgb: Any


class _EpisodeCameraReader:
    """Read RGB snapshots from the recorder-owned shared-memory camera ring."""

    def __init__(self, camera: Any) -> None:
        self.camera = camera

    def latest(self) -> _PolicyCameraSample:
        reference = self.camera.latest
        if reference is None:
            raise RuntimeError("episode camera has not published a frame")
        sample = reference.snapshot()
        return _PolicyCameraSample(
            sequence=int(reference.sequence),
            frame_number=int(reference.rgb_frame_number),
            device_timestamp_ms=float(reference.device_rgb_timestamp_ms),
            host_monotonic_ns=int(reference.host_monotonic_ns),
            rgb=sample.rgb,
        )


def _run_physical(args: argparse.Namespace) -> dict[str, Any]:
    """Run one physical rollout with one owner for all command-capable I/O."""

    # Keep the offline entry point importable without CUDA/RealSense/serial
    # dependencies.  These imports are deliberately inside the hardware path.
    try:
        from act_physical_rollout import (
            InferenceThread,
            ModelWorker,
            _assert_policy_action,
            _native_arguments,
            _project_rh56_command,
            _resolve_execution_options,
            _runtime_values,
            _temporal_selection_ages_ms,
            _validate_rollout_duration,
        )
    except ModuleNotFoundError:
        from tools.act_physical_rollout import (
            InferenceThread,
            ModelWorker,
            _assert_policy_action,
            _native_arguments,
            _project_rh56_command,
            _resolve_execution_options,
            _runtime_values,
            _temporal_selection_ages_ms,
            _validate_rollout_duration,
        )
    try:
        from act_physical_rollout import RH56_MAX_PROJECTION_CORRECTION
    except ModuleNotFoundError:
        from tools.act_physical_rollout import RH56_MAX_PROJECTION_CORRECTION

    import numpy as np

    from embodiment_core.act_temporal_executor import AbsoluteTimeTemporalEnsembler
    from embodiment_core.config import load_yaml
    from episode_dataset.collector import CaptureState
    from episode_dataset.episode import (
        ControlSample,
        EpisodeStatus,
        PHYSICAL_SCHEMA_VERSION,
    )
    from episode_dataset.runtime import EpisodeDataRuntime
    from quest_jaka_hardware import (
        COMBINED_CONTROL_REALTIME_PRIORITY,
        _apply_target_displacement_policy,
        _configure_cpu_isolation,
        _require_realtime_priority_limit,
        _validate_control_cpu,
        _wait_status,
    )
    from quest_jaka_sim import (
        SharedJakaTargetGenerator,
        SmoothQuestJakaSession,
        with_physical_rh56_retarget,
    )
    from quest_jaka_sim.live_controller import LiveQuestControllerRouter
    from quest_jaka_sim.live_input import QuestDatagramReceiverWorker
    from rh56_driver.pc_direct_control import (
        HandOperation,
        RH56PcDirectControl,
        inspect_serial_device,
        require_serial_by_id_path,
    )
    from rh56_driver.pc_direct_worker import RH56PcDirectWorker
    from rh56_driver.serial_backend import RH56SerialBackend
    from teleoperation.dagger_arbiter import (
        DaggerActionSource,
        ExpertArmCandidateSink,
        ExpertHandCandidateSink,
    )
    from teleoperation.jaka.quest_adapter import JakaAcceptedJointTargetAdapter
    from teleoperation.runtime.arm_only import ArmOnlyRuntime, NativeWorkerProcess
    from teleoperation.wire import (
        LatestTargetPublisher,
        StatusFlags,
        WorkerStatusReceiver,
    )

    collection_document = load_yaml(args.runtime_config)
    runtime, config = _runtime_values(args.runtime_config)
    collection_profile = collection_document.get(
        "collection_profile", runtime.get("collection_profile", "production")
    )
    if collection_profile not in {"production", "diagnostic"}:
        raise ValueError("collection_profile must be production or diagnostic")
    configured_total_duration_sec = float(runtime["dagger_total_duration_sec"])
    total_duration_source = (
        "runtime.dagger_total_duration_sec"
        if args.total_duration_sec is None
        else "command_line"
    )
    args.total_duration_sec = (
        configured_total_duration_sec
        if args.total_duration_sec is None
        else float(args.total_duration_sec)
    )
    args.episode_preview = bool(
        collection_profile == "diagnostic" and runtime.get("episode_preview", False)
    )
    if not bool(runtime.get("recover_output_acceleration_transition", False)):
        raise ValueError(
            "physical DAgger requires native recoverable output acceleration transitions"
        )
    native_status_every_cycles = int(runtime["native_status_every_cycles"])
    if native_status_every_cycles <= 0:
        raise ValueError("native_status_every_cycles must be positive")
    if (
        int(runtime["native_control_realtime_priority"])
        != COMBINED_CONTROL_REALTIME_PRIORITY
    ):
        raise RuntimeError("physical DAgger requires validated native realtime priority 10")
    _require_realtime_priority_limit(int(runtime["native_control_realtime_priority"]))
    control_cpu = int(runtime["native_control_cpu"])
    _validate_control_cpu(control_cpu)
    _validate_rollout_duration(args.total_duration_sec)
    if not 0.0 < args.command_rate_hz <= 30.0:
        raise ValueError("command_rate_hz must be within (0,30]")
    # Resolve canonical ACT semantics through the maintained physical rollout
    # rather than carrying a second copy of its defaults and validation.
    execution_options = _resolve_execution_options(
        requested_mode="canonical_temporal_ensemble",
        command_rate_hz=args.command_rate_hz,
        query_rate_hz=args.query_rate_hz,
        temporal_ensemble_coeff=args.temporal_ensemble_coeff,
        max_source_horizon=args.max_source_horizon,
        max_prediction_age_ticks=args.max_prediction_age_ticks,
        temporal_buffer_capacity=args.temporal_buffer_capacity,
    )
    args.query_rate_hz = execution_options["query_rate_hz"]
    args.temporal_ensemble_coeff = execution_options["temporal_ensemble_coeff"]
    args.max_source_horizon = execution_options["max_source_horizon"]
    args.max_prediction_age_ticks = execution_options["max_prediction_age_ticks"]
    args.temporal_buffer_capacity = execution_options["temporal_buffer_capacity"]

    target_displacement_limit_enabled = runtime.get(
        "enforce_clutch_target_displacement_limit", True
    )
    if type(target_displacement_limit_enabled) is not bool:
        raise ValueError(
            "enforce_clutch_target_displacement_limit must be a boolean"
        )
    config = _apply_target_displacement_policy(
        config,
        enabled=target_displacement_limit_enabled,
    )
    config = with_physical_rh56_retarget(config)
    hardware = config.raw["hardware_adapter"]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)

    episode_runtime: EpisodeDataRuntime | None = None
    provenance: DaggerProvenanceWriter | None = None
    coordinator: DaggerCoordinator | None = None
    model: Any | None = None
    native: Any | None = None
    runtime_arm: Any | None = None
    jaka_adapter: Any | None = None
    rh56_worker: Any | None = None
    receiver: Any | None = None
    inference: Any | None = None
    temporal: Any | None = None
    writer_finished = False
    provenance_path: Path | None = None
    episode_result: str | None = None
    act_predictions_path: Path | None = None
    abort_reason: str | None = None
    stop_reason = "duration_complete"
    episode_capture_failed = False
    episode_capture_abort_reason: str | None = None
    event_log: _DaggerEventLog | None = None
    event_log_path = output / "quest_events.jsonl"
    event_extract_path = (
        output / "event_extract.jsonl"
        if collection_profile == "diagnostic"
        else None
    )
    native_telemetry_path = (
        output / "native_telemetry.jsonl"
        if collection_profile == "diagnostic"
        else None
    )
    component_placement_snapshots: list[dict[str, object]] = []
    producer_timing_rows: deque[dict[str, float]] = deque(maxlen=512)
    event_log_diagnostics: dict[str, int] = {"drop_count": 0, "error_count": 0}
    episode_recorder_diagnostics: dict[str, object] | None = None
    episode_camera_diagnostics: dict[str, object] | None = None
    episode_quality_diagnostics: dict[str, object] | None = None
    episode_preview_diagnostics: dict[str, object] | None = None
    quest_control_timing: dict[str, object] | None = None
    native_output_acceleration_hold_status_count = 0
    native_output_acceleration_recovery_status_count = 0
    native_output_acceleration_hold_active = False
    command_count = 0
    policy_query_count = 0
    policy_fallback_count = 0
    episode_rotation_count = 0
    policy_resume_count = 0
    selected_source_counts = {source.value: 0 for source in DaggerActionSource}
    started_ns = time.monotonic_ns()

    def mark_episode_capture_failed(reason: str) -> None:
        nonlocal episode_capture_failed, episode_capture_abort_reason
        if not episode_capture_failed:
            episode_capture_failed = True
            episode_capture_abort_reason = reason

    try:
        _configure_cpu_isolation(control_cpu)
        episode_config = Path(str(runtime["episode_data_config"]))
        episode_root = runtime["episode_root"]
        task_name = str(runtime["task_name"])
        operator = str(runtime["operator"])
        episode_runtime = EpisodeDataRuntime.start(
            episode_config,
            episode_root=episode_root,
            task_name=task_name,
            operator=operator,
            control_config_path=Path(str(runtime["config"])),
            maximum_start_delta_rad=float(
                hardware["startup_alignment_tolerance_rad"]
            ),
            schema_version=PHYSICAL_SCHEMA_VERSION,
            preview_enabled=bool(args.episode_preview),
            forbidden_cpu=control_cpu,
            metadata={
                "simulation_only": False,
                "physically_validated": False,
                "dagger": {
                    "enabled": True,
                    "policy_checkpoint": str(args.checkpoint.resolve()),
                    "expert_source": "quest",
                    "label_source": "expert",
                    "provenance_convention": (
                        "audit/chunk-000/<episode-id>/dagger_control.jsonl"
                    ),
                },
            },
        )
        provenance_path = (
            episode_runtime.collector.writer.root
            / "audit"
            / "chunk-000"
            / episode_runtime.collector.writer.temporary_id
            / "dagger_control.jsonl"
        )
        provenance = DaggerProvenanceWriter(
            provenance_path, capacity=int(args.provenance_capacity)
        )
        provenance.start()
        coordinator = DaggerCoordinator(provenance=provenance)
        event_log = _DaggerEventLog(event_log_path)
        event_log.start()

        model = ModelWorker(args.checkpoint.resolve(), output, container_image=args.model_container)
        model.start()
        model.warmup()

        rh56_device = str(runtime["rh56_device"])
        allow_direct_ch341 = bool(runtime.get("allow_direct_ch341_device", False))
        require_serial_by_id_path(
            rh56_device,
            require_exists=True,
            allow_direct_ch341=allow_direct_ch341,
        )
        identity = inspect_serial_device(
            rh56_device, allow_direct_ch341=allow_direct_ch341
        )
        occupied_pids = identity.get("occupied_pids", [])
        if occupied_pids:
            raise RuntimeError(f"RH56 serial device is already in use by PIDs {occupied_pids}")
        hand_config = load_yaml(Path(str(runtime["rh56_config"])))
        hand_config["mode"] = "real"
        hand_config["backend_type"] = "serial_protocol"
        hand_config.setdefault("serial", {})["port"] = rh56_device
        hand_config.setdefault("diagnostics", {})["enabled"] = True
        rh56_control = RH56PcDirectControl(RH56SerialBackend(hand_config), hand_config)
        rh56_worker = RH56PcDirectWorker(rh56_control)
        rh56_worker.start(HandOperation.COMBINED)
        rh56_dispatch = _Rh56OwnershipDispatch(rh56_worker)

        with tempfile.TemporaryDirectory(prefix="act_dagger_rollout_") as temporary:
            temporary_path = Path(temporary)
            runtime_arm = ArmOnlyRuntime(
                LatestTargetPublisher(temporary_path / "target.sock"),
                WorkerStatusReceiver(temporary_path / "status.sock"),
            )
            jaka_adapter = JakaAcceptedJointTargetAdapter(
                runtime_arm,
                allow_motion=True,
                joint_order=tuple(hardware["joint_order"]),
                joint_angle_unit=str(hardware["joint_angle_unit"]),
                command_mode=str(hardware["command_mode"]),
            )
            native_metrics = output / "native_metrics.json"
            native = NativeWorkerProcess(
                Path(str(runtime["worker"])),
                _dagger_native_arguments(
                    _native_arguments,
                    runtime,
                    config,
                    temporary_path / "target.sock",
                    temporary_path / "status.sock",
                    native_metrics,
                    native_telemetry_path,
                    args.total_duration_sec,
                ),
            )
            target_generator = SharedJakaTargetGenerator(
                config, mjcf_path=config.mjcf_path
            )
            expert_arm = ExpertArmCandidateSink()
            hand_state: dict[str, tuple[float, ...]] = {
                "last_observation": (0.0,) * 6,
            }

            def measured_hand(_now_ns: int) -> Sequence[float]:
                feedback = rh56_worker.latest_feedback
                if feedback is None:
                    # Keep the Quest candidate path alive during a temporary
                    # feedback gap; the dataset records hand_source=unavailable
                    # for that tick and the RH56 worker remains the fault owner.
                    return hand_state["last_observation"]
                hand_state["last_observation"] = tuple(feedback.position_normalized)
                return feedback.position_normalized

            expert_hand = ExpertHandCandidateSink(
                measured_hand,
                max_target_normalized=float(rh56_worker.max_target_normalized),
            )
            session = SmoothQuestJakaSession(
                config,
                target_generator,
                arm_output=expert_arm,
                control_compute_budget_ms=float(
                    config.raw["shared_target_generation"]["control_compute_budget_ms"]
                ),
                normalized_hand_output=expert_hand,
            )
            clutches = config.raw["clutches"]
            router = LiveQuestControllerRouter(
                stale_after_s=float(clutches["stale_after_ms"]) / 1000.0,
                released_at=float(clutches["released_at"]),
            )
            native.start()
            status = _wait_status(runtime_arm, native)
            target_generator.synchronize_authoritative_arm_joints(
                list(status.joint_position_rad)
            )
            if not jaka_adapter.apply_joint_position(status.joint_position_rad):
                raise RuntimeError("failed to publish native startup alignment target")
            status = _wait_for_native_target_acceptance(
                runtime_arm,
                native,
                jaka_adapter.last_sequence,
            )
            rh56_dispatch.activate_from_measured(time.monotonic_ns())

            camera_readers = (
                _EpisodeCameraReader(episode_runtime.cameras["workspace"]),
                _EpisodeCameraReader(episode_runtime.cameras["wrist"]),
            )
            policy_command_period_ns = _dagger_control_period_ns(
                DaggerState.POLICY_RUN,
                policy_rate_hz=args.command_rate_hz,
            )
            command_epoch_ns = time.monotonic_ns()
            # Keep the policy path identical to the maintained physical ACT
            # rollout: one complete model query is performed synchronously at
            # the beginning of each command tick.  DAgger still computes the
            # Quest candidate in the same tick, but it must not introduce a
            # second asynchronous policy scheduler or drain JAKA status ahead
            # of ``InferenceThread.snapshot``.
            inference = InferenceThread(
                model,
                camera_readers,
                runtime_arm,
                rh56_worker,
                provenance,
                initial_status=status,
                rate_hz=(
                    args.command_rate_hz
                    if args.query_rate_hz is None
                    else args.query_rate_hz
                ),
                max_age_ms=args.max_source_age_ms,
                command_epoch_ns=command_epoch_ns,
                command_period_ns=policy_command_period_ns,
                pending_capacity=args.temporal_buffer_capacity,
            )
            temporal = AbsoluteTimeTemporalEnsembler(
                action_dim=model.contract.action_dim,
                chunk_size=model.contract.chunk_size,
                coefficient=args.temporal_ensemble_coeff,
                max_source_horizon=args.max_source_horizon,
                max_prediction_age_ticks=args.max_prediction_age_ticks,
                capacity=args.temporal_buffer_capacity,
            )
            receiver = QuestDatagramReceiverWorker(
                bind=str(runtime["bind"]),
                port=int(runtime["port"]),
                allowed_sender=(
                    None
                    if runtime.get("allowed_sender") is None
                    else str(runtime["allowed_sender"])
                ),
            )
            receiver.start()
            component_placement_snapshots.append(
                _dagger_component_snapshot(
                    boundary="dagger_workers_started",
                    native=native,
                    receiver=receiver,
                    rh56_worker=rh56_worker,
                )
            )

            control_tick = 0
            policy_command_tick = 0
            next_tick_ns = time.monotonic_ns()
            deadline_ns = next_tick_ns + int(args.total_duration_sec * 1e9)
            prior_engaged = False
            # Same finite-difference history used by the verified
            # combined-normal-teleop collector.  This belongs to the dataset
            # stream, not to the policy observation stream.
            previous_episode_q: tuple[float, ...] | None = None
            previous_episode_observation_ns: int | None = None
            expert_release_stop_delay_ns = 5_000_000_000
            episode_idle_started_ns: int | None = None
            episode_rotation_in_flight = False
            episode_capture_active = False
            event_health_next_ns: int | None = None
            rh56_full_diagnostics_next_ns: int | None = None
            event_boundary_pending = False
            prior_collector_state = episode_runtime.collector.state
            while time.monotonic_ns() < deadline_ns:
                outer_tick_started_ns = time.perf_counter_ns()
                now_ns = time.monotonic_ns()
                if now_ns < next_tick_ns:
                    time.sleep((next_tick_ns - now_ns) / 1e9)
                now_ns = time.monotonic_ns()
                receiver.raise_if_failed()
                if native.process is None or native.process.poll() is not None:
                    raise RuntimeError("native JAKA worker exited during DAgger rollout")
                if rh56_worker.failed:
                    raise RuntimeError("RH56 worker failed during DAgger rollout")
                if inference.error is not None:
                    raise RuntimeError(inference.error)
                collector = episode_runtime.collector
                if (
                    not episode_capture_failed
                    and collector.state is CaptureState.DONE
                    and collector.completion_status is not EpisodeStatus.COMPLETED
                ):
                    mark_episode_capture_failed(
                        collector.termination_reason or "episode_capture_failure"
                    )
                if (
                    episode_rotation_in_flight
                    and collector.writer.sample_count == 0
                    and collector.writer.temporary_id.startswith("episode_")
                ):
                    episode_rotation_in_flight = False
                    previous_episode_q = None
                    previous_episode_observation_ns = None
                    event_boundary_pending = True

                receiver_started_ns = time.perf_counter_ns()
                for datagram in receiver.drain(max_controller_packets=32):
                    router.ingest(datagram, session)
                receiver_drain_ns = time.perf_counter_ns() - receiver_started_ns

                selection = None
                policy_prediction = None
                policy_query_sequence: int | None = None
                policy_action = None
                policy_tick = coordinator.arbiter.state is DaggerState.POLICY_RUN
                if policy_tick:
                    # This is the canonical ACT execution order used by
                    # ``act_physical_rollout.py``.  Supplying the absolute
                    # command tick keeps chunk horizons aligned with the
                    # command stream, while ``publish_pending=False`` avoids
                    # creating an unused asynchronous prediction queue.
                    #
                    # Once the clutch requests takeover, the arbiter leaves
                    # POLICY_RUN.  From the next tick onward this block is
                    # skipped completely: ACT is interrupted rather than
                    # continuing to produce unused policy candidates during
                    # expert control.
                    prediction = inference.query_once(
                        command_tick=policy_command_tick,
                        publish_pending=False,
                    )
                    temporal.add_prediction(
                        query_id=prediction.sequence,
                        query_timestamp_ns=prediction.query_end_ns,
                        query_command_tick=prediction.query_command_tick,
                        chunk=prediction.chunk,
                    )
                    selection = temporal.select(
                        command_tick=policy_command_tick,
                        command_timestamp_ns=now_ns,
                    )
                    if selection.action is None:
                        raise RuntimeError(
                            "canonical ACT temporal ensemble produced no current action"
                        )
                    policy_prediction = prediction
                    oldest_age_ms, newest_age_ms = _temporal_selection_ages_ms(
                        selection, now_ns
                    )
                    if (
                        newest_age_ms is not None
                        and newest_age_ms > args.max_policy_age_ms
                    ):
                        raise RuntimeError("ACT policy action exceeded freshness limit")
                    if selection.contributors:
                        policy_query_sequence = selection.contributors[-1].query_id
                        policy_prediction = inference.get_prediction(policy_query_sequence)
                    raw_policy = np.asarray(selection.action, dtype=np.float64)
                    projected, projection_events = _project_rh56_command(
                        raw_policy,
                        legal_min=0.0,
                        legal_max=float(rh56_control.max_close),
                        maximum_correction=RH56_MAX_PROJECTION_CORRECTION,
                    )
                    _assert_policy_action(projected)
                    policy_action = DaggerAction.from_vector(
                        projected,
                        source=DaggerActionSource.POLICY,
                    )
                    policy_fallback_count += int(selection.fallback_used)
                    policy_query_count += int(policy_query_sequence is not None)
                    del oldest_age_ms, projection_events
                else:
                    # During expert takeover there is no policy snapshot to
                    # drain the latest-only native status socket.  Keep the
                    # manual-control and hold paths on the same fresh status
                    # stream without issuing another ACT query.
                    latest_status = runtime_arm.latest_status()
                    if latest_status is not None:
                        status = latest_status

                # Quest input is always polled for clutch ownership.  Its
                # mapped target is only dispatched after takeover; it never
                # holds the physical adapter while policy owns the rollout.
                expert_arm.reset()
                expert_hand.reset()
                router_poll_started_ns = time.perf_counter_ns()
                router.poll(now_ns, session)
                router_poll_ns = time.perf_counter_ns() - router_poll_started_ns
                # ``InferenceThread.snapshot`` is the sole consumer of the
                # latest-only JAKA status socket, just as in the ordinary ACT
                # rollout.  Re-reading it here would drain a fresh packet
                # before the next query and force the inference path to reuse
                # an older observation.
                if policy_prediction is not None:
                    status = policy_prediction.observation.status
                status_flags = StatusFlags(status.flags)
                native_acceleration_transition = False
                if (
                    status_flags & StatusFlags.OUTPUT_ACCELERATION_HOLD
                    and not native_output_acceleration_hold_active
                ):
                    native_output_acceleration_hold_status_count += 1
                    native_output_acceleration_hold_active = True
                    native_acceleration_transition = True
                if status_flags & StatusFlags.OUTPUT_ACCELERATION_RECOVERED:
                    native_output_acceleration_recovery_status_count += 1
                    native_output_acceleration_hold_active = False
                    native_acceleration_transition = True
                session_control_tick_started_ns = time.perf_counter_ns()
                session.control_tick(
                    now_ns,
                    fresh_measured_joint_position_rad=tuple(status.joint_position_rad),
                )
                session_control_tick_ns = (
                    time.perf_counter_ns() - session_control_tick_started_ns
                )
                engaged = session.arm_clutch.state.value == "engaged"
                pressed_edge = engaged and not prior_engaged
                released_edge = prior_engaged and not engaged
                clutch_valid = bool(session.left_controller_valid)
                if coordinator.arbiter.state is DaggerState.POLICY_RUN and pressed_edge:
                    coordinator.request_takeover(
                        now_ns, reason="operator_clutch_takeover"
                    )
                elif coordinator.arbiter.state is DaggerState.TAKEOVER_HOLD:
                    coordinator.observe_clutch(
                        now_ns,
                        released=not engaged,
                        pressed=False,
                        valid=clutch_valid,
                    )
                elif coordinator.arbiter.state is DaggerState.EXPERT_ARMING:
                    coordinator.observe_clutch(
                        now_ns,
                        released=False,
                        pressed=pressed_edge,
                        valid=clutch_valid,
                    )
                    if (
                        coordinator.arbiter.capture_requested
                        and expert_arm.accepted_target is not None
                        and not episode_rotation_in_flight
                    ):
                        coordinator.mark_expert_ready(now_ns)
                        episode_capture_active = True
                prior_engaged = engaged

                latest_event = dict(session.latest_event_record)
                release_boundary_pending = False
                both_clutches_released = _physical_episode_release_ready(
                    arm_clutch_state=session.arm_clutch.state.value,
                    hand_clutch_state=session.hand_clutch.state.value,
                    latest_event=latest_event,
                    dispatch_failed=False,
                )
                if coordinator.arbiter.state is DaggerState.EXPERT_RUN:
                    has_recorded_samples = collector.writer.sample_count > 0
                    if both_clutches_released and has_recorded_samples:
                        if episode_idle_started_ns is None:
                            episode_idle_started_ns = now_ns
                            episode_capture_active = False
                        elif (
                            now_ns - episode_idle_started_ns
                            >= expert_release_stop_delay_ns
                        ):
                            release_boundary_pending = True
                    elif (
                        both_clutches_released
                        and collector.state is CaptureState.ARMING
                    ):
                        episode_capture_active = False
                    elif not both_clutches_released:
                        episode_idle_started_ns = None
                        if not episode_rotation_in_flight:
                            episode_capture_active = True
                clutch_edge_reason = None
                if pressed_edge:
                    clutch_edge_reason = "operator_clutch_pressed"
                elif released_edge:
                    clutch_edge_reason = "operator_clutch_released"
                event_due = (
                    collection_profile == "diagnostic"
                    or event_health_next_ns is None
                    or now_ns >= event_health_next_ns
                    or clutch_edge_reason is not None
                    or event_boundary_pending
                    or native_acceleration_transition
                )
                if event_due:
                    event_health_next_ns = now_ns + 1_000_000_000
                boundary_event = event_boundary_pending
                event_boundary_pending = False
                rh56_telemetry = None
                if (
                    event_due
                    and (
                        collection_profile == "diagnostic"
                        or rh56_full_diagnostics_next_ns is None
                        or now_ns >= rh56_full_diagnostics_next_ns
                    )
                ):
                    rh56_full_diagnostics_next_ns = now_ns + 1_000_000_000
                    rh56_telemetry = rh56_control.episode_record(
                        now_ns,
                        include_diagnostics=True,
                    )
                event_record = {
                    **latest_event,
                    "timestamp_ns": now_ns,
                    "command_tick": control_tick,
                    "policy_command_tick": (
                        policy_command_tick if policy_tick else None
                    ),
                    "control_rate_hz": (
                        args.command_rate_hz
                        if policy_tick
                        else EXPERT_CONTROL_RATE_HZ
                    ),
                    "dagger_state": coordinator.arbiter.state.value,
                    "clutch_edge_reason": clutch_edge_reason,
                    "collection_boundary_event": boundary_event,
                    "dispatch_failed": False,
                    "controller_fault": bool(status.error_code),
                    "native_output_acceleration_hold": bool(
                        status_flags & StatusFlags.OUTPUT_ACCELERATION_HOLD
                    ),
                    "native_output_acceleration_recovered": bool(
                        status_flags & StatusFlags.OUTPUT_ACCELERATION_RECOVERED
                    ),
                    "measured_joint_position_rad": list(status.joint_position_rad),
                    "command_timestamp_ns": status.command_monotonic_ns,
                    "router": router.telemetry(),
                    "rh56_telemetry": rh56_telemetry,
                }

                expert_action = None
                hand_engaged = session.hand_clutch.state.value in {
                    "reacquire",
                    "engaged",
                }
                expert_target = (
                    expert_arm.accepted_target
                    if engaged
                    else session.last_accepted_target
                )
                if (engaged or hand_engaged) and expert_target is not None:
                    hand_target = (
                        expert_hand.target
                        if hand_engaged and expert_hand.target is not None
                        else measured_hand(now_ns)
                    )
                    expert_action = DaggerAction.from_vector(
                        (*expert_target.joint_position_rad, *hand_target),
                        source=DaggerActionSource.EXPERT,
                    )
                feedback_snapshot = rh56_worker.latest_dataset_feedback
                feedback = None if feedback_snapshot is None else feedback_snapshot.feedback
                if feedback is not None:
                    hand_state["last_observation"] = tuple(feedback.position_normalized)
                measured_hand_target = hand_state["last_observation"]
                hold_action = DaggerAction.from_vector(
                    (*status.joint_position_rad, *measured_hand_target),
                    source=DaggerActionSource.HOLD,
                )
                dagger_tick = coordinator.tick(
                    now_ns,
                    policy=policy_action,
                    expert=expert_action,
                    hold=hold_action,
                    policy_query_sequence=policy_query_sequence,
                    metadata={
                        "command_tick": control_tick,
                        "policy_command_tick": (
                            policy_command_tick if policy_tick else None
                        ),
                        "control_rate_hz": (
                            args.command_rate_hz
                            if policy_tick
                            else EXPERT_CONTROL_RATE_HZ
                        ),
                        "arm_clutch_state": session.arm_clutch.state.value,
                        "hand_clutch_state": session.hand_clutch.state.value,
                        "capture_active": episode_capture_active,
                        "temporal_selection": (
                            None if selection is None else selection.as_dict()
                        ),
                        "policy_fallback": (
                            False if selection is None else selection.fallback_used
                        ),
                    },
                )
                decision = dagger_tick.decision
                source = None if decision.action is None else decision.action.source
                if source is not None:
                    selected_source_counts[source.value] += 1

                dispatch_ok = True
                if source is DaggerActionSource.POLICY:
                    assert decision.action is not None
                    dispatch_ok = jaka_adapter.apply_joint_position(
                        decision.action.arm_q,
                        source_capture_ns=(
                            time.monotonic_ns()
                            if policy_prediction is None
                            else policy_prediction.observation.ready_ns
                        ),
                        local_receive_ns=now_ns,
                        processing_ns=now_ns,
                    )
                    if dispatch_ok:
                        # ``hold`` deactivates the PC-direct worker.  The
                        # ownership dispatcher restores measured activation
                        # before the first ACT target following a takeover.
                        rh56_dispatch.submit_target(decision.action.hand_target, now_ns)
                elif source is DaggerActionSource.EXPERT:
                    if engaged:
                        if expert_arm.accepted_target is None:
                            raise RuntimeError(
                                "engaged expert arm has no accepted Quest target"
                            )
                        dispatch_ok = jaka_adapter.apply(expert_arm.accepted_target)
                    else:
                        dispatch_ok = jaka_adapter.pause()
                    if dispatch_ok:
                        if hand_engaged:
                            # Quest writes into a candidate sink under DAgger
                            # arbitration.  Activate the selected physical
                            # output here before forwarding its expert target.
                            rh56_dispatch.submit_target(
                                decision.action.hand_target,
                                now_ns,
                            )
                        else:
                            rh56_dispatch.hold("expert_hand_clutch_released")
                elif source is DaggerActionSource.HOLD:
                    dispatch_ok = jaka_adapter.pause()
                    rh56_dispatch.hold(decision.reason)
                else:
                    dispatch_ok = jaka_adapter.pause()
                    rh56_dispatch.hold("dagger_terminal_state")

                dispatch_failed = not dispatch_ok
                event_record["dispatch_failed"] = dispatch_failed
                event_record["dispatch_reason"] = None if dispatch_ok else "command_publication_failed"
                if release_boundary_pending and _physical_episode_release_ready(
                    arm_clutch_state=session.arm_clutch.state.value,
                    hand_clutch_state=session.hand_clutch.state.value,
                    latest_event=latest_event,
                    dispatch_failed=dispatch_failed,
                ):
                    if (
                        episode_runtime.dataset_format == "lerobot_staging_v1"
                        and not episode_rotation_in_flight
                    ):
                        collector.rotate_episode(
                            "both_clutches_released_5s",
                            release_ns=episode_idle_started_ns,
                        )
                        episode_rotation_in_flight = True
                        episode_rotation_count += 1
                        event_boundary_pending = True
                    temporal.reset()
                    coordinator.resume_policy(
                        now_ns,
                        reason="both_clutches_released_5s",
                    )
                    episode_capture_active = False
                    episode_idle_started_ns = None
                    policy_resume_count += 1
                if coordinator.arbiter.state is DaggerState.SAFE_STOP:
                    raise RuntimeError("DAgger entered safe stop")

                capture_active = episode_capture_active
                # Local copy of the verified combined-normal-teleop recording
                # block.  The original hardware entry point remains
                # unchanged; DAgger keeps the same canonical sample fields.
                record_episode_sample = (
                    status is not None
                    and not episode_capture_failed
                    and collector.state is not CaptureState.DONE
                    and (
                        capture_active
                        or (
                            collector.state is CaptureState.REC
                            and not episode_rotation_in_flight
                        )
                    )
                )
                if record_episode_sample:
                    measured_q = tuple(float(value) for value in status.joint_position_rad)
                    observation_ns = int(status.observation_monotonic_ns)
                    if (
                        previous_episode_q is None
                        or previous_episode_observation_ns is None
                        or observation_ns <= previous_episode_observation_ns
                    ):
                        measured_dq = (0.0,) * 6
                    else:
                        dt = (
                            observation_ns - previous_episode_observation_ns
                        ) / 1e9
                        measured_dq = tuple(
                            (current - old) / dt
                            for current, old in zip(
                                measured_q, previous_episode_q, strict=True
                            )
                        )
                    previous_episode_q = measured_q
                    previous_episode_observation_ns = observation_ns
                    # Match the standard collector: persist the most recent
                    # accepted Quest target, including on a hold tick.
                    accepted_target = (
                        expert_arm.accepted_target
                        if expert_arm.accepted_target is not None
                        else session.last_accepted_target
                    )
                    if accepted_target is None:
                        accepted_q = measured_q
                        tcp = target_generator.current_tcp_pose
                        action_source = "measured_hold_reference"
                        accepted_sequence = None
                    else:
                        accepted_q = tuple(accepted_target.joint_position_rad)
                        tcp = accepted_target.filtered_tcp
                        action_source = "accepted_target"
                        accepted_sequence = accepted_target.sequence_number
                    action_status = (
                        "held_rejected"
                        if latest_event.get("control_state") == "HOLD_REJECTED"
                        else "accepted"
                    )
                    hand_grip = session.hand_clutch.state.value in {
                        "reacquire",
                        "engaged",
                    }
                    if feedback is None:
                        hand_observation = (0.0,) * 6
                        hand_target = hand_observation
                        hand_source = "unavailable"
                        hand_grip = False
                        force_observation = None
                    else:
                        hand_observation = feedback.position_normalized
                        hand_target = (
                            feedback.position_normalized
                            if rh56_control.last_command_normalized is None
                            else rh56_control.last_command_normalized
                        )
                        hand_source = "measured"
                        force_observation = feedback.load_or_force_raw_count
                    source_timestamps = {
                        "jaka_observation": observation_ns,
                        "jaka_command": int(status.command_monotonic_ns),
                    }
                    source_timestamp_domains = {
                        "jaka_observation": "host_monotonic_ns",
                        "jaka_command": "host_monotonic_ns",
                    }
                    if feedback_snapshot is not None:
                        source_timestamps.update(
                            {
                                "rh56_angle_act": feedback_snapshot.angle_act_timestamp_ns
                                or int(feedback.monotonic_ns),
                                "rh56_force_act": feedback_snapshot.force_act_timestamp_ns,
                            }
                        )
                        source_timestamp_domains.update(
                            {
                                "rh56_angle_act": "host_monotonic_ns",
                                "rh56_force_act": "host_monotonic_ns",
                            }
                        )
                    episode_runtime.collector.ingest_control(
                        ControlSample(
                            host_monotonic_ns=time.monotonic_ns(),
                            accepted_arm_q=accepted_q,
                            arm_q_measured=measured_q,
                            arm_dq_measured=measured_dq,
                            arm_dq_source="estimated",
                            tcp_pose_xyzw=(
                                *tcp.position_m,
                                *tcp.orientation_xyzw,
                            ),
                            tcp_pose_source="commanded",
                            hand_observation=hand_observation,
                            hand_source=hand_source,
                            hand_target=hand_target,
                            arm_trigger=engaged,
                            hand_grip=hand_grip,
                            arm_action_status=action_status,
                            arm_action_source=action_source,
                            accepted_target_sequence=accepted_sequence,
                            reference_generation=(
                                session.reference_generation
                                if session.reference_generation > 0
                                else None
                            ),
                            source_timestamps_ns=source_timestamps,
                            source_timestamp_domains=source_timestamp_domains,
                            control_heartbeat_valid=dispatch_ok,
                            controller_fault=bool(status.error_code),
                            force_observation=force_observation,
                        ),
                        reference_established=session.reference_generation > 0,
                        capture_active=(
                            True
                            if collector.state is CaptureState.REC
                            else capture_active
                        ),
                        raw_records=None,
                    )
                collector_state = episode_runtime.collector.state
                if collector_state is not prior_collector_state:
                    event_boundary_pending = True
                    prior_collector_state = collector_state
                timing_row = {
                    "receiver_drain": receiver_drain_ns / 1e6,
                    "receiver_drain_and_router_ingest": (
                        receiver_drain_ns + router_poll_ns
                    )
                    / 1e6,
                    "controller_router_poll": router_poll_ns / 1e6,
                    "shared_session_control_tick": session_control_tick_ns / 1e6,
                    "policy_query": (
                        0.0
                        if policy_prediction is None
                        else float(policy_prediction.stage_timing_ms.get("total", 0.0))
                    ),
                    "event_enqueue": 0.0 if not event_due else 1.0,
                    "complete_outer_tick": (
                        time.perf_counter_ns() - outer_tick_started_ns
                    )
                    / 1e6,
                }
                producer_timing_rows.append(timing_row)
                event_record["producer_outer_timing_ms"] = timing_row
                session.add_control_timing(
                    "quest_input_duration_ns",
                    receiver_drain_ns + router_poll_ns,
                )
                # ``ControlTimingRecorder`` is owned by the unchanged Quest
                # implementation and only accepts its canonical timing fields.
                # The DAgger-specific shared-session duration is already kept
                # in ``producer_timing_rows`` above; do not pass a new field
                # name into the Thor timing recorder (it raises ``KeyError``
                # on the first outer tick).
                session.finalize_control_timing(
                    time.perf_counter_ns() - outer_tick_started_ns
                )
                session.update_control_timing_context(
                    {
                        "event_log_enqueued": bool(event_due),
                        "camera_health": "managed_by_episode_runtime",
                    }
                )
                if record_episode_sample:
                    episode_runtime.update_preview(
                        arm_trigger=engaged,
                        hand_grip=hand_grip,
                    )
                if (
                    (event_due or dispatch_failed or bool(event_record["controller_fault"]))
                    and event_log is not None
                ):
                    event_record["collector_state"] = collector_state.value
                    event_record["episode_capture_active"] = capture_active
                    event_record["rh56_feedback_available"] = feedback is not None
                    event_log.write(event_record)
                if dispatch_failed:
                    mark_episode_capture_failed("control_heartbeat_transport_failure")
                    abort_reason = "control_heartbeat_transport_failure"
                    stop_reason = abort_reason
                    component_placement_snapshots.append(
                        _dagger_component_snapshot(
                            boundary=abort_reason,
                            native=native,
                            receiver=receiver,
                            rh56_worker=rh56_worker,
                        )
                    )
                    jaka_adapter.stop()
                    rh56_worker.arm_terminal_stop(abort_reason)
                command_count += 1
                control_tick += 1
                if policy_tick:
                    policy_command_tick += 1
                next_control_period_ns = _dagger_control_period_ns(
                    coordinator.arbiter.state,
                    policy_rate_hz=args.command_rate_hz,
                )
                next_tick_ns += next_control_period_ns
                if next_tick_ns <= time.monotonic_ns():
                    next_tick_ns = time.monotonic_ns() + next_control_period_ns

                if dispatch_failed:
                    # The invalid-heartbeat sample has already been handed to
                    # the collector above.  Match standard teleop: stop the
                    # producer after that sample and let collector cleanup
                    # perform the deferred abort/finalization.
                    break
            if inference is not None:
                inference.stop()
            quest_control_timing = session.control_timing_report()
    except KeyboardInterrupt:
        stop_reason = "operator_keyboard_stop"
    except BaseException as exc:
        abort_reason = f"{type(exc).__name__}: {exc}"
        stop_reason = abort_reason
        component_placement_snapshots.append(
            _dagger_component_snapshot(
                boundary=abort_reason,
                native=native,
                receiver=receiver,
                rh56_worker=rh56_worker,
            )
        )
        if coordinator is not None:
            try:
                coordinator.safe_stop(time.monotonic_ns(), reason=abort_reason)
            except BaseException:
                pass
        print(f"DAgger rollout aborted: {abort_reason}", flush=True)
    finally:
        if inference is not None:
            try:
                inference.stop()
            except BaseException as exc:
                abort_reason = abort_reason or f"inference_cleanup:{type(exc).__name__}:{exc}"
        if receiver is not None:
            receiver.close()
        if jaka_adapter is not None and not jaka_adapter.stopped:
            try:
                if abort_reason is not None:
                    jaka_adapter.stop()
                else:
                    jaka_adapter.pause()
                    jaka_adapter.stop()
            except BaseException as exc:
                abort_reason = abort_reason or f"jaka_cleanup:{type(exc).__name__}:{exc}"
        if rh56_worker is not None:
            try:
                if abort_reason is not None:
                    rh56_worker.arm_terminal_stop(abort_reason)
                else:
                    rh56_worker.hold(stop_reason)
            except BaseException as exc:
                abort_reason = abort_reason or f"rh56_cleanup:{type(exc).__name__}:{exc}"
        if native is not None:
            try:
                native.stop(timeout_s=8.0)
            except BaseException as exc:
                abort_reason = abort_reason or f"native_cleanup:{type(exc).__name__}:{exc}"
        if runtime_arm is not None:
            runtime_arm.close()
        if rh56_worker is not None:
            try:
                rh56_worker.cleanup()
            except BaseException as exc:
                abort_reason = abort_reason or f"rh56_close:{type(exc).__name__}:{exc}"
        if model is not None:
            try:
                model.stop()
            except BaseException as exc:
                abort_reason = abort_reason or f"model_cleanup:{type(exc).__name__}:{exc}"
        if inference is not None:
            try:
                saved_chunks = inference.saved_chunks()
                if saved_chunks is not None:
                    act_predictions_path = output / "act_predictions.npz"
                    np.savez_compressed(
                        act_predictions_path,
                        chunks=saved_chunks,
                        query_sequences=np.arange(
                            1, len(saved_chunks) + 1, dtype=np.int64
                        ),
                    )
            except BaseException as exc:
                abort_reason = abort_reason or f"prediction_cleanup:{type(exc).__name__}:{exc}"
        if episode_runtime is not None:
            try:
                episode_runtime.stop_camera_forwarder()
                collector = episode_runtime.collector
                if collector.state is CaptureState.REC:
                    # Keep the verified staging shutdown rule: an outer
                    # rollout timeout/keyboard stop must not accidentally
                    # promote an unfinished staging episode.  An explicit
                    # DAgger clutch release is an episode boundary and is
                    # finalized normally below.
                    if (
                        abort_reason is None
                        and stop_reason in {"duration_complete", "operator_keyboard_stop"}
                        and episode_runtime.dataset_format == "lerobot_staging_v1"
                    ):
                        collector.discard_current(
                            "outer_session_ended_before_episode_boundary"
                        )
                    elif abort_reason is None:
                        collector.finish(stop_reason)
                    else:
                        collector.abort(abort_reason)
                elif collector.state is not CaptureState.DONE:
                    collector.shutdown(abort_reason or stop_reason)
                collector.finalize_pending()
                # Capture recorder/camera diagnostics while their IPC handles
                # are still open.  ``EpisodeDataRuntime.close()`` closes the
                # recorder status queue and camera rings, so asking the proxy
                # for diagnostics afterwards is not safe and can lose the
                # same lifecycle details that standard teleop includes.
                episode_recorder_diagnostics = collector.writer.diagnostics()
                episode_quality_diagnostics = collector.diagnostics()
                episode_camera_diagnostics = {
                    role: camera.profile_metadata()
                    for role, camera in episode_runtime.cameras.items()
                }
                episode_preview_diagnostics = (
                    None
                    if episode_runtime.preview is None
                    else episode_runtime.preview.diagnostics()
                )
                if episode_quality_diagnostics is not None:
                    episode_quality_diagnostics["camera_forwarder_error"] = (
                        episode_runtime.camera_forwarder_error
                    )
                episode_result = (
                    None if collector.result is None else str(collector.result)
                )
            except BaseException as exc:
                abort_reason = abort_reason or f"episode_cleanup:{type(exc).__name__}:{exc}"
            finally:
                try:
                    episode_runtime.close()
                except BaseException as exc:
                    abort_reason = abort_reason or f"episode_close:{type(exc).__name__}:{exc}"
        if event_log is not None:
            try:
                event_log.close()
                event_log_diagnostics = {
                    "drop_count": event_log.drop_count,
                    "error_count": event_log.error_count,
                }
                if event_extract_path is not None:
                    _write_dagger_event_extract(
                        event_log_path,
                        event_extract_path,
                        native_telemetry_path=native_telemetry_path,
                    )
            except BaseException as exc:
                abort_reason = abort_reason or f"event_log_cleanup:{type(exc).__name__}:{exc}"
        component_placement_snapshots.append(
            _dagger_component_snapshot(
                boundary="dagger_workers_shutdown",
                native=native,
                receiver=receiver,
                rh56_worker=rh56_worker,
            )
        )
        if provenance is not None and not writer_finished:
            try:
                provenance.finish()
                writer_finished = True
            except BaseException as exc:
                abort_reason = abort_reason or f"provenance_cleanup:{type(exc).__name__}:{exc}"

    summary = {
        "schema_version": "act_dagger_rollout.v1",
        "runtime_config": str(args.runtime_config.resolve()),
        "checkpoint": str(args.checkpoint.resolve()),
        "output": str(output),
        "total_duration_sec": args.total_duration_sec,
        "total_duration_source": total_duration_source,
        "collection_profile": collection_profile,
        "command_rate_hz": args.command_rate_hz,
        "expert_control_rate_hz": EXPERT_CONTROL_RATE_HZ,
        "policy_query_rate_hz": args.query_rate_hz or args.command_rate_hz,
        "act_execution_mode": "canonical_temporal_ensemble",
        "temporal_ensemble": (
            None
            if temporal is None
            else {
                "coefficient": args.temporal_ensemble_coeff,
                "max_source_horizon": temporal.max_source_horizon,
                "max_prediction_age_ticks": temporal.max_prediction_age_ticks,
                "capacity": temporal.capacity,
                "stats": temporal.stats(),
            }
        ),
        "native_status_every_cycles": native_status_every_cycles,
        "command_count": command_count,
        "policy_query_count": policy_query_count,
        "policy_fallback_count": policy_fallback_count,
        "selected_source_counts": selected_source_counts,
        "provenance": None if provenance_path is None else str(provenance_path),
        "provenance_rows": None if provenance is None else provenance.write_count,
        "provenance_drop_count": None if provenance is None else provenance.drop_count,
        "episode_result": episode_result,
        "act_predictions": (
            None if act_predictions_path is None else str(act_predictions_path)
        ),
        "abort_reason": abort_reason,
        "stop_reason": stop_reason,
        "episode_capture_failed": episode_capture_failed,
        "episode_capture_abort_reason": episode_capture_abort_reason,
        "episode_rotation_count": episode_rotation_count,
        "policy_resume_count": policy_resume_count,
        "router_telemetry": None if receiver is None else router.telemetry(),
        "episode_recorder_diagnostics": episode_recorder_diagnostics,
        "episode_quality_diagnostics": episode_quality_diagnostics,
        "episode_camera_diagnostics": episode_camera_diagnostics,
        "episode_preview_diagnostics": episode_preview_diagnostics,
        "episode_camera_forwarder_error": (
            None
            if episode_quality_diagnostics is None
            else episode_quality_diagnostics.get("camera_forwarder_error")
        ),
        "rh56_worker_failure": (
            None if rh56_worker is None else rh56_worker.failure_record
        ),
        "rh56_diagnostics": (
            None if rh56_worker is None else rh56_worker.diagnostics_snapshot()
        ),
        "event_log": str(event_log_path),
        "event_extract": (
            None if event_extract_path is None else str(event_extract_path)
        ),
        "native_metrics": str(output / "native_metrics.json"),
        "native_telemetry": (
            None if native_telemetry_path is None else str(native_telemetry_path)
        ),
        "event_log_diagnostics": event_log_diagnostics,
        "component_placement_snapshots": component_placement_snapshots,
        "native_output_acceleration_hold_status_count": (
            native_output_acceleration_hold_status_count
        ),
        "native_output_acceleration_recovery_status_count": (
            native_output_acceleration_recovery_status_count
        ),
        "producer_timing_ms": _dagger_timing_summary(producer_timing_rows),
        "quest_control_timing": quest_control_timing,
    }
    (output / "dagger_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False, default=str) + "\n",
        encoding="utf-8",
    )
    return summary


def _demo_action(source: DaggerActionSource, value: float) -> DaggerAction:
    return DaggerAction.from_vector([value] * 12, source=source)


def _run_offline_demo(output: Path) -> dict[str, Any]:
    """Run the deterministic state-machine smoke path without hardware."""

    writer = DaggerProvenanceWriter(output)
    writer.start()
    coordinator = DaggerCoordinator(provenance=writer)
    hold = _demo_action(DaggerActionSource.HOLD, 0.0)
    policy = _demo_action(DaggerActionSource.POLICY, 0.1)
    expert = _demo_action(DaggerActionSource.EXPERT, 0.2)
    decisions: list[str] = []
    try:
        decisions.append(coordinator.tick(1, policy=policy, expert=None, hold=hold).decision.reason)
        coordinator.request_takeover(2)
        decisions.append(coordinator.tick(3, policy=policy, expert=None, hold=hold).decision.reason)
        coordinator.observe_clutch(4, released=True, pressed=False, valid=True)
        coordinator.observe_clutch(5, released=False, pressed=True, valid=True)
        coordinator.mark_expert_ready(6)
        decisions.append(coordinator.tick(7, policy=policy, expert=expert, hold=hold).decision.reason)
        coordinator.resume_policy(8)
        decisions.append(coordinator.tick(9, policy=policy, expert=None, hold=hold).decision.reason)
    finally:
        writer.finish()
    return {
        "status": "passed",
        "states": [transition.current.value for transition in coordinator.arbiter.transitions],
        "decision_reasons": decisions,
        "provenance": str(output.resolve()),
        "provenance_rows": writer.write_count,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--offline-demo",
        action="store_true",
        help="run the no-hardware arbitration/provenance smoke path",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
    )
    parser.add_argument("--runtime-config", type=Path)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument(
        "--model-container",
        default=None,
        help="CUDA LeRobot image for the inference-only ACT worker",
    )
    parser.add_argument("--command-rate-hz", type=float, default=30.0)
    parser.add_argument(
        "--total-duration-sec",
        type=float,
        default=None,
        help="total DAgger session duration; defaults to runtime YAML",
    )
    parser.add_argument("--query-rate-hz", type=float, default=None)
    parser.add_argument("--max-source-age-ms", type=float, default=250.0)
    parser.add_argument("--max-policy-age-ms", type=float, default=250.0)
    parser.add_argument("--temporal-ensemble-coeff", type=float, default=None)
    parser.add_argument("--max-source-horizon", type=int, default=None)
    parser.add_argument("--max-prediction-age-ticks", type=int, default=None)
    parser.add_argument("--temporal-buffer-capacity", type=int, default=None)
    parser.add_argument("--provenance-capacity", type=int, default=2048)
    args = parser.parse_args()
    if args.offline_demo:
        offline_output = args.output or Path("logs/dagger/offline_provenance.jsonl")
        print(json.dumps(_run_offline_demo(offline_output), indent=2, ensure_ascii=False))
        return 0
    if args.runtime_config is None or args.checkpoint is None:
        raise SystemExit(
            "physical DAgger requires --runtime-config and --checkpoint; "
            "use --offline-demo for the no-hardware smoke path"
        )
    if args.query_rate_hz is not None and not 0.0 < args.query_rate_hz <= 30.0:
        raise SystemExit("query rate must be within (0,30]")
    if (
        args.temporal_buffer_capacity is not None
        and args.temporal_buffer_capacity < 1
    ) or args.provenance_capacity < 1:
        raise SystemExit("buffer capacities must be positive")
    if args.output is None:
        args.output = Path("logs/dagger") / (
            "act_dagger_" + time.strftime("%Y%m%d_%H%M%S")
        )
    summary = _run_physical(args)
    print(json.dumps(summary, indent=2, ensure_ascii=False, default=str))
    return 0 if summary["abort_reason"] is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
