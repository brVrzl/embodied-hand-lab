#!/usr/bin/env python3
# Native acceleration-transition faults are classified explicitly; the legacy
# accepted_target_transport_failure label is retained only for compatibility
# with older summaries and is never used for this event.
"""Run the maintained bounded arm-only or combined Quest/JAKA path.

The shared Python target-generation session feeds either the arm-only JAKA
adapter or the combined JAKA/RH56 path. Historical commissioning and research
stages are intentionally not part of this operator entry point.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import queue
import resource
import signal
import subprocess
import sys
import tempfile
import threading
import time

from quest_jaka_sim import (
    ReplayConfig,
    SharedJakaTargetGenerator,
    SmoothQuestJakaSession,
    with_physical_rh56_retarget,
)
from quest_jaka_sim.live_controller import LiveQuestControllerRouter
from quest_jaka_sim.live_input import QuestDatagramReceiverWorker
from teleoperation.jaka.quest_adapter import JakaAcceptedJointTargetAdapter
from teleoperation.runtime.arm_only import ArmOnlyRuntime, NativeWorkerProcess
from teleoperation.wire import LatestTargetPublisher, StatusFlags, WorkerStatusReceiver
from teleoperation.output_feasibility import (
    NATIVE_DEFENSIVE_OUTPUT_JERK_LIMIT_RAD_S3,
)
from embodiment_core.config import load_yaml
from episode_dataset.collector import CaptureState
from episode_dataset.episode import (
    ControlSample,
    EpisodeStatus,
    PHYSICAL_SCHEMA_VERSION,
)
from episode_dataset.runtime import EpisodeDataRuntime
from rh56_driver.pc_direct_control import (
    HandOperation,
    RH56PcDirectControl,
    inspect_serial_device,
    require_serial_by_id_path,
)
from rh56_driver.pc_direct_worker import RH56PcDirectWorker
from rh56_driver.serial_backend import RH56SerialBackend
from rh56_driver.telemetry import BoundedJsonlRecorder


COMBINED_CONTROL_REALTIME_PRIORITY = 10
RECOVERABLE_CLUTCH_STAGES = frozenset((
    "bounded-normal-teleop",
    "combined-normal-teleop",
))


class _AsyncEventLog:
    """Bounded event-log sink; producer publication is strictly non-blocking."""

    def __init__(self, path: Path, *, capacity: int = 256) -> None:
        self.path = path
        self._queue: queue.Queue[dict[str, object] | None] = queue.Queue(maxsize=capacity)
        self._stop = threading.Event()
        self.drop_count = 0
        self.error_count = 0
        self._file = None
        self._started = False
        self._thread = threading.Thread(target=self._run, name="teleop-event-log", daemon=True)

    def __enter__(self) -> "_AsyncEventLog":
        try:
            self._file = self.path.open("x", encoding="utf-8")
            self._thread.start()
            self._started = True
        except BaseException:
            self.error_count += 1
            if self._file is not None:
                try:
                    self._file.close()
                except BaseException:
                    self.error_count += 1
            self._file = None
        return self

    def write(self, record: dict[str, object]) -> None:
        if not self._started:
            return
        try:
            self._queue.put_nowait(dict(record))
        except queue.Full:
            self.drop_count += 1

    def __exit__(self, *_: object) -> None:
        if not self._started:
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
            finally:
                self._file = None

    def _run(self) -> None:
        assert self._file is not None
        while True:
            record = self._queue.get()
            try:
                if record is None:
                    return
                self._file.write(json.dumps(record, sort_keys=True) + "\n")
                self._file.flush()
            except Exception:
                self.error_count += 1
            finally:
                self._queue.task_done()


def _timestamp_rate_hz(timestamps_ns: list[int]) -> float | None:
    if len(timestamps_ns) < 2 or timestamps_ns[-1] <= timestamps_ns[0]:
        return None
    return (len(timestamps_ns) - 1) * 1e9 / (
        timestamps_ns[-1] - timestamps_ns[0]
    )


def _control_compute_budget_summary(
    session: SmoothQuestJakaSession,
) -> dict[str, float | int | None]:
    """Return only budget counters maintained by the production session."""

    return {
        "control_compute_budget_ms": session.control_compute_budget_ms,
        "control_compute_budget_exhausted_count": (
            session.control_compute_budget_exhausted_count
        ),
    }


def _task_placement(
    *, component: str, process_id: int, thread_id: int, thread_name: str
) -> dict[str, object]:
    """Read one bounded /proc scheduling record outside command-critical work."""

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
        result.update(
            {
                "supported": False,
                "reason": "Linux procfs scheduling telemetry is unavailable",
            }
        )
        return result
    try:
        stat = Path(f"/proc/{process_id}/task/{thread_id}/stat").read_text(
            encoding="utf-8"
        )
        closing = stat.rfind(")")
        fields = stat[closing + 2 :].split()
        # The first split field is Linux task-stat field 3 (state), while
        # processor is field 39.
        result["current_cpu"] = int(fields[36])
        result["scheduler_policy"] = int(os.sched_getscheduler(thread_id))
        result["scheduler_priority"] = int(
            os.sched_getparam(thread_id).sched_priority
        )
        result["nice_value"] = int(os.getpriority(os.PRIO_PROCESS, thread_id))
        result["affinity_mask"] = sorted(os.sched_getaffinity(thread_id))
        result["supported"] = True
    except (IndexError, OSError, ValueError) as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    return result


def _component_placement_snapshot(
    *,
    boundary: str,
    native: NativeWorkerProcess,
    receiver: QuestDatagramReceiverWorker | None,
    rh56_worker: RH56PcDirectWorker | None,
) -> dict[str, object]:
    """Capture bounded component placement without touching control locks."""

    process_id = os.getpid()
    known_threads: dict[int, tuple[str, str]] = {
        threading.get_native_id(): ("python_combined_wrapper", "main"),
    }
    if receiver is not None and receiver.thread.native_id is not None:
        known_threads[int(receiver.thread.native_id)] = (
            "quest_receiver",
            receiver.thread.name,
        )
    if rh56_worker is not None and rh56_worker.native_thread_id is not None:
        known_threads[int(rh56_worker.native_thread_id)] = (
            "rh56_serial_and_logging",
            "rh56-pc-direct",
        )
    tasks: list[dict[str, object]] = []
    try:
        task_ids = sorted(
            int(path.name) for path in Path(f"/proc/{process_id}/task").iterdir()
        )
    except OSError:
        task_ids = sorted(known_threads)
    for thread_id in task_ids:
        component, name = known_threads.get(
            thread_id, ("other_python_thread", "unknown")
        )
        tasks.append(
            _task_placement(
                component=component,
                process_id=process_id,
                thread_id=thread_id,
                thread_name=name,
            )
        )
    if native.process is not None:
        tasks.append(
            _task_placement(
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
        "logging_execution": "synchronous_on_rh56_serial_worker",
    }


def _validate_control_cpu(control_cpu: int | None) -> tuple[set[int], set[int]]:
    if hasattr(os, "sched_getaffinity"):
        allowed = set(os.sched_getaffinity(0))
    else:
        allowed = set(range(os.cpu_count() or 1))
    if control_cpu is None:
        return allowed, allowed
    if control_cpu not in allowed:
        raise SystemExit(
            f"native control CPU {control_cpu} is not in the allowed affinity "
            f"mask {sorted(allowed)}"
        )
    non_realtime = allowed - {control_cpu}
    if not non_realtime:
        raise SystemExit("CPU isolation requires at least two allowed CPUs")
    return allowed, non_realtime


def _require_realtime_priority_limit(priority: int) -> dict[str, int]:
    """Fail before hardware I/O unless the native child can enter SCHED_FIFO."""

    if not hasattr(resource, "RLIMIT_RTPRIO"):
        raise SystemExit(
            "combined teleoperation requires Linux RLIMIT_RTPRIO support"
        )
    soft, hard = resource.getrlimit(resource.RLIMIT_RTPRIO)
    unlimited = resource.RLIM_INFINITY
    if soft != unlimited and soft < priority:
        raise SystemExit(
            "combined teleoperation requires inherited RLIMIT_RTPRIO >= "
            f"{priority} before any hardware I/O; current soft limit is {soft}"
        )
    return {
        "required_priority": priority,
        "soft_limit": soft,
        "hard_limit": hard,
    }


def _configure_cpu_isolation(control_cpu: int | None) -> dict[str, object]:
    """Reserve one CPU for native control and move current Python tasks away."""

    allowed, non_realtime = _validate_control_cpu(control_cpu)
    if control_cpu is None:
        return {
            "enabled": False,
            "native_control_cpu": None,
            "python_affinity_mask": sorted(allowed),
        }
    if not (
        sys.platform.startswith("linux")
        and hasattr(os, "sched_setaffinity")
        and hasattr(os, "sched_getaffinity")
    ):
        raise SystemExit(
            "native control CPU isolation requires Linux scheduling APIs"
        )
    process_id = os.getpid()
    try:
        task_ids = sorted(
            int(path.name) for path in Path(f"/proc/{process_id}/task").iterdir()
        )
        for thread_id in task_ids:
            try:
                os.sched_setaffinity(thread_id, non_realtime)
            except ProcessLookupError:
                continue
        failed = [
            thread_id
            for thread_id in task_ids
            if Path(f"/proc/{process_id}/task/{thread_id}").exists()
            and control_cpu in os.sched_getaffinity(thread_id)
        ]
    except OSError as exc:
        raise SystemExit(f"failed to configure non-real-time CPU affinity: {exc}") from exc
    if failed:
        raise SystemExit(
            "non-real-time CPU affinity verification failed for task ids "
            + ",".join(str(value) for value in failed)
        )
    return {
        "enabled": True,
        "native_control_cpu": control_cpu,
        "python_affinity_mask": sorted(non_realtime),
        "reassigned_existing_python_tasks": len(task_ids),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    # The live-demo YAML retains its target envelope for simulation/replay. A
    # physical collection runtime may explicitly disable only that
    # clutch-relative envelope; all other shared and native safety checks stay
    # enabled. This is intentionally not a command-line switch.
    parser.set_defaults(
        config=None,
        enforce_clutch_target_displacement_limit=True,
        episode_data_config=None,
        episode_preview=False,
        episode_root=None,
        event_extract=None,
        log=None,
        metrics=None,
        native_control_cpu=None,
        native_control_realtime_priority=None,
        native_telemetry=None,
        operator=None,
        output_generator=None,
        recover_output_acceleration_transition=False,
        rh56_config=None,
        rh56_device=None,
        rh56_log=None,
        robot_ip=None,
        run_output_joint_velocity_limits_rad_s=None,
        summary=None,
        task_name=None,
        worker=None,
        allowed_sender=None,
        bind=None,
        port=None,
        duration_sec=None,
        edg_state_ip=None,
        allow_direct_ch341_device=False,
    )
    parser.add_argument(
        "stage",
        choices=(
            "bounded-normal-teleop",
            "combined-normal-teleop",
        ),
    )
    parser.add_argument("--runtime-config", type=Path, help="host-specific runtime YAML for physical collection")
    parser.add_argument("--rh56-command-path-absent", action="store_true")
    return parser


def _apply_target_displacement_policy(
    config: ReplayConfig, *, enabled: bool
) -> ReplayConfig:
    """Apply the explicit collection-only target-envelope policy."""

    if type(enabled) is not bool:
        raise ValueError("enforce_clutch_target_displacement_limit must be a boolean")
    if enabled:
        return config
    return replace(
        config,
        feasibility=replace(
            config.feasibility,
            target_displacement_limit_enabled=False,
        ),
    )


def _apply_runtime_config(args: argparse.Namespace) -> None:
    """Resolve stable host/collection values from one explicit YAML file."""

    if args.runtime_config is None:
        raise SystemExit(
            f"{args.stage} requires --runtime-config; "
            "device, control, and collection values are YAML-owned"
        )
    runtime: dict[str, object] = {}
    if args.runtime_config is not None:
        try:
            document = load_yaml(args.runtime_config)
        except (FileNotFoundError, OSError, TypeError, ValueError) as exc:
            raise SystemExit(
                f"invalid runtime config before hardware I/O: {args.runtime_config}: {exc}"
            ) from exc
        candidate = document.get("runtime", document)
        if not isinstance(candidate, dict):
            raise SystemExit("runtime config must contain a mapping at root.runtime")
        runtime = candidate

    duration_key = (
        "duration_sec"
        if args.stage == "combined-normal-teleop"
        else "bounded_duration_sec"
    )
    velocity_key = (
        "run_output_joint_velocity_limits_rad_s"
        if args.stage == "combined-normal-teleop"
        else "bounded_run_output_joint_velocity_limits_rad_s"
    )
    required_keys = [
        "config",
        "worker",
        "robot_ip",
        "edg_state_ip",
        "bind",
        "port",
        duration_key,
        velocity_key,
    ]
    if args.stage == "combined-normal-teleop":
        required_keys.extend(
            (
                "rh56_device",
                "rh56_config",
                "native_control_cpu",
                "native_control_realtime_priority",
                "episode_data_config",
                "episode_root",
                "task_name",
                "operator",
            )
        )
    missing = [key for key in required_keys if key not in runtime]
    if missing:
        raise SystemExit(
            "runtime config is missing required keys: " + ", ".join(missing)
        )

    def resolve(name: str, key: str) -> object | None:
        value = runtime.get(key)
        if value is None and key not in runtime:
            return None
        return value

    for name, key in (
        ("config", "config"),
        ("worker", "worker"),
        ("robot_ip", "robot_ip"),
        ("edg_state_ip", "edg_state_ip"),
        ("bind", "bind"),
        ("port", "port"),
        ("duration_sec", duration_key),
        ("allowed_sender", "allowed_sender"),
    ):
        value = resolve(name, key)
        if value is not None:
            setattr(args, name, value)
    if args.stage == "combined-normal-teleop":
        for name in (
            "rh56_device",
            "rh56_config",
            "native_control_cpu",
            "native_control_realtime_priority",
            "episode_data_config",
            "episode_root",
            "task_name",
            "operator",
        ):
            setattr(args, name, runtime[name])

    velocity_limits = runtime[velocity_key]
    if args.run_output_joint_velocity_limits_rad_s is None:
        try:
            args.run_output_joint_velocity_limits_rad_s = tuple(
                float(value) for value in velocity_limits
            )
        except (TypeError, ValueError) as exc:
            raise SystemExit(
                f"runtime config {velocity_key} must contain six numbers"
            ) from exc

    args.allow_direct_ch341_device = bool(runtime.get("allow_direct_ch341_device", False))
    if bool(runtime.get("allow_direct_ch341_device", False)):
        args.allow_direct_ch341_device = True
    if bool(runtime.get("episode_preview", False)):
        args.episode_preview = True
    if bool(runtime.get("recover_output_acceleration_transition", False)):
        args.recover_output_acceleration_transition = True
    if "enforce_clutch_target_displacement_limit" in runtime:
        value = runtime["enforce_clutch_target_displacement_limit"]
        if type(value) is not bool:
            raise SystemExit(
                "runtime config enforce_clutch_target_displacement_limit must be boolean"
            )
        args.enforce_clutch_target_displacement_limit = value

    if args.config is not None and not isinstance(args.config, Path):
        args.config = Path(str(args.config))
    if args.worker is not None and not isinstance(args.worker, Path):
        args.worker = Path(str(args.worker))
    if args.rh56_config is not None and not isinstance(args.rh56_config, Path):
        args.rh56_config = Path(str(args.rh56_config))
    for name in (
        "episode_data_config",
        "episode_root",
        "runtime_config",
        "log",
        "summary",
        "metrics",
        "native_telemetry",
        "event_extract",
        "rh56_log",
    ):
        value = getattr(args, name)
        if value is not None and not isinstance(value, Path):
            setattr(args, name, Path(str(value)))

    log_dir = Path(str(runtime.get("log_dir", "logs")))
    stamp = time.strftime("%Y%m%d_%H%M%S")
    prefix = log_dir / f"quest_jaka_{args.stage}_{stamp}_{os.getpid()}"
    generated_outputs = {
        "log": f"{prefix}.events.jsonl",
        "summary": f"{prefix}.summary.json",
        "metrics": f"{prefix}.native_metrics.json",
        "native_telemetry": f"{prefix}.native_cycles.jsonl",
        "event_extract": f"{prefix}.event_extract.jsonl",
        "rh56_log": f"{prefix}.rh56.jsonl",
    }
    for name, path in generated_outputs.items():
        if getattr(args, name) is None:
            setattr(args, name, Path(path))

    if args.robot_ip is None:
        raise SystemExit("runtime config must define robot_ip")


def _wait_status(runtime: ArmOnlyRuntime, native: NativeWorkerProcess, timeout_s: float = 8.0):
    required_flags = (
        StatusFlags.CONNECTED
        | StatusFlags.EDG_ACTIVE
        | StatusFlags.STARTUP_REFERENCE_READY
    )
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = runtime.latest_status()
        if status is not None and (
            StatusFlags(status.flags) & required_flags
        ) == required_flags:
            return status
        if native.process is not None and native.process.poll() is not None:
            raise RuntimeError("JAKA worker exited before reporting connected state")
        time.sleep(0.005)
    raise RuntimeError("JAKA worker did not report startup reference ready state")


def _control_output_failed(*, reason: str, output_applied: bool) -> bool:
    """Treat accepted-target or heartbeat transport failure as fatal, not disengagement."""

    return reason != "DISENGAGED" and not output_applied


def _producer_timing_summary(
    rows: list[dict[str, float]],
) -> dict[str, dict[str, float]]:
    result: dict[str, dict[str, float]] = {}
    for name in sorted({key for row in rows for key in row}):
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
            "p99_9": percentile(99.9),
            "max": values[-1],
        }
    return result


def _synchronize_paused_stopped_reference(
    *,
    stage: str,
    status_flags: StatusFlags,
    target_generator: SharedJakaTargetGenerator,
    measured_joint_position_rad: tuple[float, ...],
) -> None:
    """Refresh the resume seed only after native braking is complete."""

    if stage in RECOVERABLE_CLUTCH_STAGES and status_flags & StatusFlags.STOPPED_READY:
        target_generator.synchronize_authoritative_arm_joints(
            list(measured_joint_position_rad)
        )


def _classify_worker_exit(metrics_path: Path, return_code: int | None) -> str:
    """Prefer the worker's typed terminal reason over a generic transport symptom."""

    try:
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return f"worker_exit:{return_code}"
    classification = metrics.get("stop_classification")
    return (
        str(classification)
        if classification
        else f"worker_exit:{return_code}"
    )


def _native_terminal_reason_if_ready(metrics_path: Path) -> str | None:
    """Read a typed native fault even during the process-reap race.

    The worker writes its metrics before the parent necessarily observes a
    non-None ``poll()`` result.  A transport send failure in that small window
    must not hide an already-authoritative native stop classification.
    """

    try:
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None
    classification = metrics.get("stop_classification")
    return str(classification) if classification else None


def _reconcile_terminal_transport_symptom(
    abort_reason: str | None,
    metrics: dict[str, object],
) -> tuple[str | None, str | None]:
    """Replace a secondary IPC symptom with a completed native fault record.

    A Unix-datagram send can fail after the worker has stopped but before
    ``poll()`` or the metrics file is observable in the producer loop. Cleanup
    waits for the worker and then loads its durable metrics, so this is the last
    point at which the primary stop can be classified without guessing. Keep
    the transport symptom separately for timeline/provenance.
    """

    transport_symptoms = frozenset((
        "control_heartbeat_transport_failure",
        "IPC_failure",
    ))
    if abort_reason not in transport_symptoms:
        return abort_reason, None
    try:
        error_code = int(metrics.get("error_code", 0))
    except (TypeError, ValueError):
        return abort_reason, None
    classification = metrics.get("stop_classification")
    if (
        error_code == 0
        or not classification
        or classification in {
            "normal_completion",
            "normal_clutch_release",
            "worker_exit",
        }
    ):
        return abort_reason, None
    return str(classification), abort_reason


def _resolve_output_jerk_limit(config: ReplayConfig) -> float:
    """Validate the YAML-owned jerk boundary before any I/O."""

    configured = config.command_limits.maximum_jerk_rad_s3
    value = configured
    if (
        not math.isfinite(value)
        or value <= 0.0
        or value > NATIVE_DEFENSIVE_OUTPUT_JERK_LIMIT_RAD_S3
    ):
        raise SystemExit(
            "output jerk shaper must be finite, positive, and no greater than "
            f"{NATIVE_DEFENSIVE_OUTPUT_JERK_LIMIT_RAD_S3:g} rad/s^3"
        )
    return value


def _native_velocity_limit_args(config: ReplayConfig) -> tuple[str, str]:
    """Serialize the exact shared output boundary for the native final gate."""

    contract = config.output_contract
    if contract.maximum_velocity_rad_s_per_joint is None:
        return (
            "--maximum-output-joint-velocity-rad-s",
            str(contract.maximum_velocity_rad_s),
        )
    return (
        "--maximum-output-joint-velocity-rad-s-per-joint",
        ",".join(str(value) for value in contract.velocity_boundaries_rad_s),
    )


def _configured_pwl_output_generator(config: ReplayConfig) -> str:
    """Return the operator label matching the configured ServoJ period."""

    period_ms = config.output_contract.servo_period_ns / 1e6
    return f"pwl-{period_ms:g}ms"


def main() -> int:
    args = _parser().parse_args()
    _apply_runtime_config(args)
    if args.duration_sec <= 0.0:
        raise SystemExit("duration must be positive")
    if args.episode_root is not None and args.episode_data_config is None:
        raise SystemExit("--episode-root requires --episode-data-config")
    try:
        config = replace(ReplayConfig.load(args.config), engagement_schedule_s=())
        config = _apply_target_displacement_policy(
            config,
            enabled=args.enforce_clutch_target_displacement_limit,
        )
        if args.stage == "combined-normal-teleop":
            config = with_physical_rh56_retarget(config)
    except (KeyError, TypeError, ValueError) as exc:
        raise SystemExit(f"invalid Quest/JAKA configuration before I/O: {exc}") from exc
    except FileNotFoundError as exc:
        raise SystemExit(f"missing physical RH56 calibration before I/O: {exc}") from exc
    jerk_limit = _resolve_output_jerk_limit(config)
    if args.run_output_joint_velocity_limits_rad_s is not None:
        run_boundaries = tuple(args.run_output_joint_velocity_limits_rad_s)
        hard_boundaries = config.output_contract.velocity_boundaries_rad_s
        if not all(
            math.isfinite(value)
            and 0.0 < value <= hard
            for value, hard in zip(
                run_boundaries, hard_boundaries, strict=True
            )
        ):
            raise SystemExit(
                "each run output velocity limit must be finite, positive, "
                "and no greater than its shared per-joint hard contract"
            )
        config = replace(
            config,
            output_contract=replace(
                config.output_contract,
                maximum_velocity_rad_s_per_joint=run_boundaries,
            ),
        )
    hardware = config.raw["hardware_adapter"]
    servo_step_num = int(hardware.get("servo_step_num", 1))
    configured_output_generator = _configured_pwl_output_generator(config)
    args.output_generator = configured_output_generator
    if args.stage == "bounded-normal-teleop" and not args.rh56_command_path_absent:
        raise SystemExit(
            "bounded arm-only teleoperation requires no-RH56-command confirmation"
        )
    maximum_duration_sec = (
        300.0 if args.stage == "combined-normal-teleop" else 60.0
    )
    if args.duration_sec > maximum_duration_sec:
        raise SystemExit(
            f"{args.stage} is limited to {maximum_duration_sec:g} seconds"
        )
    if not args.recover_output_acceleration_transition:
        raise SystemExit(
            "maintained teleoperation requires native recoverable output "
            "acceleration transitions"
        )
    hand_identity: dict[str, object] | None = None
    hand_config: dict[str, object] | None = None
    realtime_preflight: dict[str, int] | None = None
    if args.stage == "combined-normal-teleop":
        if args.native_control_cpu is None:
            raise SystemExit(
                "combined teleoperation requires --native-control-cpu; "
                "unisolated SCHED_OTHER operation does not satisfy the required gate"
            )
        if (
            args.native_control_realtime_priority
            != COMBINED_CONTROL_REALTIME_PRIORITY
        ):
            raise SystemExit(
                "combined teleoperation requires "
                "--native-control-realtime-priority "
                f"{COMBINED_CONTROL_REALTIME_PRIORITY}"
            )
        realtime_preflight = _require_realtime_priority_limit(
            args.native_control_realtime_priority
        )
        if args.rh56_device is None or args.rh56_log is None:
            raise SystemExit("combined teleoperation requires --rh56-device and --rh56-log")
        try:
            require_serial_by_id_path(
                args.rh56_device,
                require_exists=True,
                allow_direct_ch341=args.allow_direct_ch341_device,
            )
        except (ValueError, PermissionError) as exc:
            raise SystemExit(f"invalid combined RH56 gate before any I/O: {exc}") from exc
        hand_identity = inspect_serial_device(
            args.rh56_device,
            allow_direct_ch341=args.allow_direct_ch341_device,
        )
        if args.allow_direct_ch341_device and not args.rh56_device.startswith(
            "/dev/serial/by-id/"
        ):
            if (
                hand_identity.get("usb_vid") != "1a86"
                or hand_identity.get("usb_pid") != "7523"
                or hand_identity.get("usb_driver") != "usb_ch341"
            ):
                raise SystemExit(
                    "direct CH341 identity mismatch before any hardware I/O"
                )
        hand_config = load_yaml(args.rh56_config)
        hand_config["mode"] = "real"
        hand_config["backend_type"] = "serial_protocol"
        hand_config.setdefault("serial", {})["port"] = args.rh56_device
    clutch_behavior = "release left-index to pause; press again to resume"
    if args.stage == "combined-normal-teleop" and args.episode_data_config is not None:
        print(
            "ENTRY=physical-collection CONTROL=combined-arm-rh56 "
            f"STOP=Ctrl+C; CLUTCH={clutch_behavior}"
        )
    else:
        print(f"STAGE={args.stage} STOP=Ctrl+C; CLUTCH={clutch_behavior}")

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
    # The local configuration now has one output-acceleration authority under
    # shared_target_generation.  Use the already parsed contract for the
    # native final hard check instead of requiring a duplicate hardware key.
    native_hard_acceleration = float(
        config.output_contract.maximum_acceleration_rad_s2
    )
    hold_degraded_ms = float(
        hardware["output_acceleration_hold_degraded_ms"]
    )
    hold_hard_stop_ms = float(
        hardware["output_acceleration_hold_hard_stop_ms"]
    )
    maximum_hold_cycles = int(
        hardware["maximum_consecutive_output_acceleration_hold_cycles"]
    )
    if not (
        math.isfinite(native_hard_acceleration)
        and native_hard_acceleration > 0.0
        and 0.0 < hold_degraded_ms < hold_hard_stop_ms
        and 2 <= maximum_hold_cycles <= 10_000
    ):
        raise SystemExit("output acceleration recovery policy is invalid")
    allowed_cpus, non_realtime_cpus = _validate_control_cpu(
        args.native_control_cpu
    )
    args.log.parent.mkdir(parents=True, exist_ok=True)
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    if args.native_telemetry is not None:
        args.native_telemetry.parent.mkdir(parents=True, exist_ok=True)
    if args.event_extract is not None:
        args.event_extract.parent.mkdir(parents=True, exist_ok=True)
    if args.rh56_log is not None:
        args.rh56_log.parent.mkdir(parents=True, exist_ok=True)
    cpu_isolation = _configure_cpu_isolation(args.native_control_cpu)

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
            allow_motion=True,
            joint_order=tuple(hardware["joint_order"]),
            joint_angle_unit=str(hardware["joint_angle_unit"]),
            command_mode=str(hardware["command_mode"]),
        )
        jaka_adapter = base_jaka_adapter
        rh56_worker: RH56PcDirectWorker | None = None
        rh56_control: RH56PcDirectControl | None = None
        rh56_backend: RH56SerialBackend | None = None
        rh56_log = None
        rh56_recorder: BoundedJsonlRecorder | None = None
        if args.stage == "combined-normal-teleop":
            assert hand_config is not None and args.rh56_log is not None
            rh56_backend = RH56SerialBackend(hand_config)
            rh56_control = RH56PcDirectControl(rh56_backend, hand_config)
            rh56_log = args.rh56_log.open("x", encoding="utf-8")
            rh56_diagnostics = hand_config.get("diagnostics", {})
            rh56_recorder = BoundedJsonlRecorder(
                rh56_log,
                capacity=int(
                    rh56_diagnostics.get("telemetry_buffer_capacity", 64)
                ),
                flush_every_records=int(
                    rh56_diagnostics.get("telemetry_flush_every_records", 16)
                ),
                flush_interval_sec=float(
                    rh56_diagnostics.get("telemetry_flush_interval_sec", 1.0)
                ),
            )
            rh56_worker = RH56PcDirectWorker(
                rh56_control, record=rh56_recorder
            )
        session = SmoothQuestJakaSession(
            config,
            target_generator,
            arm_output=jaka_adapter,
            control_compute_budget_ms=float(
                config.raw["shared_target_generation"]["control_compute_budget_ms"]
            ),
            normalized_hand_output=rh56_worker,
        )
        clutch = config.raw["clutches"]
        router = LiveQuestControllerRouter(
            stale_after_s=float(clutch["stale_after_ms"]) / 1000.0,
            released_at=float(clutch["released_at"]),
        )
        worker_mode = "joint-teleop"
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
            "--servo-step-num", str(servo_step_num),
            "--warning-ms", str(hardware["command_stream_warning_ms"]),
            "--hold-ms", str(hardware["command_stream_timeout_ms"]),
            "--controlled-stop-ms", str(hardware["controlled_stop_timeout_ms"]),
            "--fatal-timeout-ms", str(hardware["fatal_communication_timeout_ms"]),
            "--excessive-tracking-error-abort-rad", str(hardware["excessive_tracking_error_abort_rad"]),
            "--excessive-tracking-error-consecutive-cycles", str(hardware["excessive_tracking_error_consecutive_cycles"]),
            "--startup-alignment-tolerance-rad", str(hardware["startup_alignment_tolerance_rad"]),
            # One authority: the EDG pass-through diagnostic consumes the
            # existing shared command contract rather than hardware-only copies.
            "--diagnostic-joint-acceleration-boundary-rad-s2", str(config.output_contract.maximum_acceleration_rad_s2),
            "--maximum-output-joint-acceleration-rad-s2", str(
                native_hard_acceleration
            ),
            "--output-joint-jerk-limit-rad-s3", str(jerk_limit),
            "--output-acceleration-hold-degraded-ms", str(
                hardware["output_acceleration_hold_degraded_ms"]
            ),
            "--output-acceleration-hold-hard-stop-ms", str(
                hardware["output_acceleration_hold_hard_stop_ms"]
            ),
            "--maximum-consecutive-output-acceleration-hold-cycles", str(
                hardware[
                    "maximum_consecutive_output_acceleration_hold_cycles"
                ]
            ),
            "--startup-timing-grace-cycles",
            str(config.startup_timing_grace_cycles),
        ]
        worker_args.extend(_native_velocity_limit_args(config))
        worker_args.append("--monitor-controller-health-each-cycle")
        if args.native_telemetry is not None:
            worker_args.extend(("--cycle-telemetry-file", str(args.native_telemetry)))
        if args.recover_output_acceleration_transition:
            worker_args.append("--recover-output-acceleration-transition")
        if args.native_control_cpu is not None:
            worker_args.extend(("--control-cpu", str(args.native_control_cpu)))
        if args.native_control_realtime_priority is not None:
            worker_args.extend(
                (
                    "--control-realtime-priority",
                    str(args.native_control_realtime_priority),
                )
            )
        native = NativeWorkerProcess(args.worker, worker_args)
        accepted = 0
        stop_reason = "duration_complete"
        abort_reason: str | None = None
        prior_engaged = False
        started = time.monotonic()
        next_tick = started
        status = None
        receiver: QuestDatagramReceiverWorker | None = None
        component_placement_snapshots: list[dict[str, object]] = []
        maximum_quest_displacement_m = 0.0
        minimum_continuation_fraction = 1.0
        clutch_release_monotonic_ns: int | None = None
        arm_clutch_pause_count = 0
        measured_joint_samples: list[tuple[float, ...]] = []
        native_output_acceleration_hold_status_count = 0
        native_output_acceleration_recovery_status_count = 0
        native_output_acceleration_hold_active = False
        producer_timing_rows: list[dict[str, float]] = []
        pending_receiver_drain_and_ingest_ns = 0
        arm_commands_while_index_released = 0
        native_started = False
        episode_runtime: EpisodeDataRuntime | None = None
        episode_capture_failed = False
        episode_capture_abort_reason: str | None = None
        episode_recorder_diagnostics: dict[str, object] | None = None
        episode_camera_diagnostics: dict[str, object] | None = None
        episode_quality_diagnostics: dict[str, object] | None = None
        episode_preview_diagnostics: dict[str, object] | None = None
        event_log_diagnostics: dict[str, int] = {"drop_count": 0, "error_count": 0}
        episode_record_next_ns: int | None = None
        episode_record_period_ns = 1_000_000_000 // 30
        episode_capture_active = True
        episode_idle_started_ns: int | None = None
        episode_rotation_in_flight = False
        episode_rotation_count = 0
        episode_boundary_release_ns = 5_000_000_000
        event_log_next_ns: int | None = None
        rh56_full_diagnostics_next_ns: int | None = None
        previous_episode_q: tuple[float, ...] | None = None
        previous_episode_observation_ns: int | None = None

        def mark_episode_capture_failed(reason: str) -> None:
            nonlocal episode_capture_failed, episode_capture_abort_reason
            if not episode_capture_failed:
                episode_capture_failed = True
                episode_capture_abort_reason = reason

        try:
            if args.episode_data_config is not None:
                episode_runtime = EpisodeDataRuntime.start(
                    args.episode_data_config,
                    episode_root=args.episode_root,
                    task_name=args.task_name,
                    operator=args.operator,
                    control_config_path=args.config,
                    maximum_start_delta_rad=float(
                        hardware["startup_alignment_tolerance_rad"]
                    ),
                    schema_version=PHYSICAL_SCHEMA_VERSION,
                    preview_enabled=args.episode_preview,
                    forbidden_cpu=args.native_control_cpu,
                    metadata={
                        "units": {
                            "arm_q": "rad",
                            "arm_dq": "rad/s",
                            "tcp_translation": "m",
                            "tcp_orientation": "quaternion_xyzw",
                            "hand": "normalized_closure_0_to_1",
                            "rh56_angle_act": "raw_count",
                            "rh56_current": "raw_count",
                            "rh56_force_act": "raw_count",
                            "rh56_error": "raw_code",
                            "rh56_status": "raw_code",
                            "depth_raw": "device_units_uint16",
                            "timestamp": "host_monotonic_ns",
                        },
                        "raw_streams": {
                            "quest_raw_datagram": "unavailable",
                            "quest_decoded_input": "unavailable",
                            "accepted_arm_target_60hz": "commanded",
                            "emitted_arm_command_125hz": "measured_external_native_log",
                            "jaka_arm_q": "measured",
                            "jaka_arm_dq": "estimated_finite_difference",
                            "native_telemetry": "measured_external_native_log",
                            "rh56_target": (
                                "unavailable"
                                if args.rh56_log is None
                                else "commanded"
                            ),
                            "rh56_feedback": (
                                "unavailable"
                                if args.rh56_log is None
                                else "measured_raw_registers"
                            ),
                            "workspace_rgbd": "measured",
                            "wrist_rgbd": "measured",
                            "fault_events": "measured",
                        },
                        "simulation_only": False,
                        "physically_validated": False,
                        "physical_log_paths": {
                            "native_telemetry": str(args.native_telemetry.resolve()),
                            "rh56_telemetry": (
                                None
                                if args.rh56_log is None
                                else str(args.rh56_log.resolve())
                            ),
                            "combined_events": str(args.log.resolve()),
                        },
                    },
                )
                # The control producer normally runs at about 60 Hz, while
                # the dataset clock is 30 Hz.  Recorder work (raw JSONL
                # enqueueing and state snapshot assembly) must not consume the
                # spare half-cycle needed by Quest/IK control.  Keep camera
                # draining continuous, but submit one recorder control sample
                # per dataset period; the native/JAKA/RH56 control loop remains
                # unchanged.
                episode_record_period_ns = episode_runtime.collector.clock.period_ns
                print(
                    f"EPISODE_CAPTURE=IDLE id={episode_runtime.collector.writer.temporary_id} "
                    f"root={episode_runtime.collector.writer.root}",
                    flush=True,
                )
            if rh56_worker is not None:
                rh56_worker.start(HandOperation.COMBINED)
            native.start()
            native_started = True
            status = _wait_status(runtime, native)
            measured_joint_samples.append(tuple(status.joint_position_rad))
            target_generator.synchronize_authoritative_arm_joints(list(status.joint_position_rad))
            with _AsyncEventLog(args.log) as log:
                receiver = QuestDatagramReceiverWorker(
                    bind=args.bind,
                    port=args.port,
                    allowed_sender=args.allowed_sender,
                )
                receiver.start()
                component_placement_snapshots.append(
                    _component_placement_snapshot(
                        boundary="combined_workers_started",
                        native=native,
                        receiver=receiver,
                        rh56_worker=rh56_worker,
                    )
                )
                started = time.monotonic()
                next_tick = started
                try:
                    while time.monotonic() - started < args.duration_sec:
                        if rh56_worker is not None and rh56_worker.failed:
                            abort_reason = "rh56_transport_or_feedback_fault"
                            stop_reason = abort_reason
                            component_placement_snapshots.append(
                                _component_placement_snapshot(
                                    boundary=abort_reason,
                                    native=native,
                                    receiver=receiver,
                                    rh56_worker=rh56_worker,
                                )
                            )
                            jaka_adapter.stop()
                            break
                        if native.process is None or native.process.poll() is not None:
                            return_code = None if native.process is None else native.process.returncode
                            abort_reason = _classify_worker_exit(
                                args.metrics, return_code
                            )
                            stop_reason = abort_reason
                            component_placement_snapshots.append(
                                _component_placement_snapshot(
                                    boundary=abort_reason,
                                    native=native,
                                    receiver=receiver,
                                    rh56_worker=rh56_worker,
                                )
                            )
                            jaka_adapter.stop()
                            if rh56_worker is not None:
                                rh56_worker.arm_terminal_stop(abort_reason)
                            break
                        receiver.raise_if_failed()
                        receiver_started_ns = time.perf_counter_ns()
                        for datagram in receiver.drain():
                            router.ingest(datagram, session)
                        pending_receiver_drain_and_ingest_ns += (
                            time.perf_counter_ns() - receiver_started_ns
                        )
                        now = time.monotonic()
                        if now < next_tick:
                            time.sleep(min(0.001, next_tick - now))
                            continue
                        now_ns = time.monotonic_ns()
                        if (
                            episode_runtime is not None
                            and not episode_capture_failed
                            and not episode_rotation_in_flight
                        ):
                            try:
                                episode_runtime.ingest_cameras()
                                collector = episode_runtime.collector
                                if (
                                    collector.state is CaptureState.DONE
                                    and collector.completion_status is not EpisodeStatus.COMPLETED
                                ):
                                    mark_episode_capture_failed(
                                        collector.termination_reason
                                        or "episode_capture_failure"
                                    )
                            except BaseException as exc:
                                # Camera/recorder infrastructure is outside
                                # the native heartbeat and cannot turn a
                                # healthy robot into a control fault.
                                mark_episode_capture_failed(
                                    f"recording_runtime_failure:{type(exc).__name__}:{exc}"
                                )
                        outer_tick_started_ns = time.perf_counter_ns()
                        poll_started_ns = time.perf_counter_ns()
                        router.poll(now_ns, session)
                        router_poll_ns = time.perf_counter_ns() - poll_started_ns
                        status_started_ns = time.perf_counter_ns()
                        latest_status = runtime.latest_status()
                        if latest_status is not None:
                            status = latest_status
                            status_flags = StatusFlags(status.flags)
                            if (
                                status_flags
                                & StatusFlags.OUTPUT_ACCELERATION_HOLD
                                and not native_output_acceleration_hold_active
                            ):
                                native_output_acceleration_hold_status_count += 1
                                native_output_acceleration_hold_active = True
                            if (
                                status_flags
                                & StatusFlags.OUTPUT_ACCELERATION_RECOVERED
                            ):
                                native_output_acceleration_recovery_status_count += 1
                                native_output_acceleration_hold_active = False
                            sample = tuple(status.joint_position_rad)
                            if not measured_joint_samples or sample != measured_joint_samples[-1]:
                                measured_joint_samples.append(sample)
                            _synchronize_paused_stopped_reference(
                                stage=args.stage,
                                status_flags=status_flags,
                                target_generator=target_generator,
                                measured_joint_position_rad=sample,
                            )
                        status_sync_ns = time.perf_counter_ns() - status_started_ns
                        session_started_ns = time.perf_counter_ns()
                        tick = session.control_tick(
                            now_ns,
                            fresh_measured_joint_position_rad=(
                                None
                                if status is None
                                else tuple(status.joint_position_rad)
                            ),
                        )
                        session_control_tick_ns = (
                            time.perf_counter_ns() - session_started_ns
                        )
                        dispatch_failed = _control_output_failed(
                            reason=tick.reason,
                            output_applied=tick.output_applied,
                        )
                        if dispatch_failed:
                            control_state = session.latest_event_record.get(
                                "control_state"
                            )
                            native_return_code = (
                                None
                                if native.process is None
                                else native.process.poll()
                            )
                            native_terminal_reason = (
                                _native_terminal_reason_if_ready(args.metrics)
                            )
                            if (
                                native_return_code is not None
                            ):
                                # The worker's typed terminal reason is
                                # authoritative. Do not collapse a native
                                # output-feasibility fault into IPC failure
                                # merely because the producer no longer has a
                                # live target socket.
                                abort_reason = _classify_worker_exit(
                                    args.metrics,
                                    native_return_code,
                                )
                            elif native_terminal_reason is not None:
                                # Metrics can be durably written just before
                                # the parent observes process reaping. Prefer
                                # that typed native reason over generic IPC.
                                abort_reason = native_terminal_reason
                            elif control_state == "HARD_STOP":
                                abort_reason = f"shared_hard_stop:{tick.reason}"
                            elif tick.accepted_target is None:
                                abort_reason = "control_heartbeat_transport_failure"
                            else:
                                abort_reason = "IPC_failure"
                            stop_reason = abort_reason
                            component_placement_snapshots.append(
                                _component_placement_snapshot(
                                    boundary=abort_reason,
                                    native=native,
                                    receiver=receiver,
                                    rh56_worker=rh56_worker,
                                )
                            )
                            jaka_adapter.stop()
                            if rh56_worker is not None:
                                rh56_worker.arm_terminal_stop(abort_reason)
                        engaged = session.arm_clutch.state.value == "engaged"
                        disengaged = prior_engaged and not engaged
                        clutch_edge_reason: str | None = None
                        pause_failed = False
                        if disengaged and not dispatch_failed:
                            if not jaka_adapter.pause():
                                abort_reason = "recoverable_pause_transport_failure"
                                stop_reason = abort_reason
                                pause_failed = True
                                clutch_edge_reason = stop_reason
                            else:
                                arm_clutch_pause_count += 1
                                clutch_edge_reason = "operator_clutch_paused"
                                clutch_edge_reason = stop_reason
                            clutch_release_monotonic_ns = now_ns
                        prior_engaged = engaged
                        accepted += int(tick.accepted_target is not None and tick.output_applied)
                        arm_commands_while_index_released += int(
                            tick.accepted_target is not None
                            and tick.output_applied
                            and not engaged
                        )
                        event_log_due = (
                            episode_runtime is None
                            or event_log_next_ns is None
                            or now_ns >= event_log_next_ns
                            or dispatch_failed
                            or clutch_edge_reason is not None
                        )
                        rh56_include_diagnostics = bool(
                            rh56_control is not None
                            and event_log_due
                            and (
                                rh56_full_diagnostics_next_ns is None
                                or now_ns >= rh56_full_diagnostics_next_ns
                            )
                        )
                        if rh56_include_diagnostics:
                            rh56_full_diagnostics_next_ns = now_ns + 1_000_000_000
                        outer_event_diagnostic_started_ns = time.perf_counter_ns()
                        rh56_feedback_duration_ns = 0
                        episode_metadata_duration_ns = 0
                        event = dict(session.latest_event_record)
                        arm_released = (
                            session.arm_clutch.state.value == "disengaged"
                        )
                        hand_released = (
                            session.hand_clutch.state.value == "disengaged"
                        )
                        release_inputs_valid = bool(
                            event.get("right_wrist_valid")
                            and event.get("hand_skeleton_valid")
                            and not event.get("input_recovery_active")
                            and not dispatch_failed
                        )
                        both_clutches_released = (
                            arm_released and hand_released and release_inputs_valid
                        )
                        if (
                            episode_runtime is not None
                            and episode_runtime.dataset_format == "lerobot_staging_v1"
                            and not episode_capture_failed
                        ):
                            collector = episode_runtime.collector
                            has_recorded_samples = collector.writer.sample_count > 0
                            if both_clutches_released and has_recorded_samples:
                                if episode_idle_started_ns is None:
                                    episode_idle_started_ns = now_ns
                                    episode_capture_active = False
                                elif (
                                    not episode_rotation_in_flight
                                    and now_ns - episode_idle_started_ns
                                    >= episode_boundary_release_ns
                                ):
                                    episode_runtime.collector.rotate_episode(
                                        "both_clutches_released_5s",
                                        release_ns=episode_idle_started_ns,
                                    )
                                    episode_rotation_in_flight = True
                                    episode_rotation_count += 1
                                    episode_idle_started_ns = None
                                    episode_capture_active = False
                            elif (
                                both_clutches_released
                                and collector.state is CaptureState.ARMING
                            ):
                                # A staging start candidate failed a safety
                                # gate.  Do not feed release samples into the
                                # collector's trigger state; wait for the
                                # operator's next valid press and retry the
                                # same gate from fresh measured state.
                                episode_capture_active = False
                            elif not both_clutches_released:
                                episode_idle_started_ns = None
                                if not episode_rotation_in_flight:
                                    episode_capture_active = True
                            if episode_rotation_in_flight:
                                # The recorder child finalizes the previous
                                # episode asynchronously.  Once its status
                                # exposes a new temporary id, the next valid
                                # clutch press may start a fresh episode.
                                if (
                                    collector.writer.sample_count == 0
                                    and collector.writer.temporary_id.startswith(
                                        "episode_"
                                    )
                                    and not both_clutches_released
                                ):
                                    episode_rotation_in_flight = False
                                    episode_capture_active = True
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
                        rh56_telemetry = None
                        if rh56_control is not None:
                            rh56_feedback_started_ns = time.perf_counter_ns()
                            rh56_telemetry = rh56_control.episode_record(
                                now_ns,
                                include_diagnostics=rh56_include_diagnostics,
                            )
                            rh56_feedback_duration_ns += (
                                time.perf_counter_ns() - rh56_feedback_started_ns
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
                            native_output_acceleration_hold=(
                                False
                                if status is None
                                else bool(
                                    StatusFlags(status.flags)
                                    & StatusFlags.OUTPUT_ACCELERATION_HOLD
                                )
                            ),
                            native_output_acceleration_recovered=(
                                False
                                if status is None
                                else bool(
                                    StatusFlags(status.flags)
                                    & StatusFlags.OUTPUT_ACCELERATION_RECOVERED
                                )
                            ),
                            stop_or_abort_reason=(
                                stop_reason
                                if dispatch_failed
                                else clutch_edge_reason
                            ),
                            rh56_telemetry=rh56_telemetry,
                        )
                        record_episode_sample = (
                            episode_runtime is not None
                            and status is not None
                            and not episode_capture_failed
                            and episode_runtime.collector.state
                            is not CaptureState.DONE
                            and episode_capture_active
                            and (
                                episode_record_next_ns is None
                                or now_ns >= episode_record_next_ns
                            )
                        )
                        if record_episode_sample:
                            feedback = (
                                None
                                if rh56_control is None
                                else rh56_control.last_feedback
                            )
                            held_target = (
                                tick.accepted_target
                                if tick.accepted_target is not None
                                else session.last_accepted_target
                            )
                            measured_q = tuple(status.joint_position_rad)
                            observation_ns = int(status.observation_monotonic_ns)
                            if (
                                previous_episode_q is None
                                or previous_episode_observation_ns is None
                                or observation_ns <= previous_episode_observation_ns
                            ):
                                measured_dq = (0.0,) * 6
                            else:
                                dt = (observation_ns - previous_episode_observation_ns) / 1e9
                                measured_dq = tuple(
                                    (value - previous) / dt
                                    for value, previous in zip(
                                        measured_q, previous_episode_q, strict=True
                                    )
                                )
                            previous_episode_q = measured_q
                            previous_episode_observation_ns = observation_ns
                            if held_target is None:
                                arm_target = measured_q
                                tcp = target_generator.current_tcp_pose
                                arm_action_source = "measured_hold_reference"
                                accepted_target_sequence = None
                            else:
                                arm_target = held_target.joint_position_rad
                                tcp = held_target.filtered_tcp
                                arm_action_source = "accepted_target"
                                accepted_target_sequence = held_target.sequence_number
                            raw_records = {
                                "jaka_state": {
                                    "read_host_monotonic_ns": observation_ns,
                                    "record_host_monotonic_ns": now_ns,
                                    "command_host_monotonic_ns": status.command_monotonic_ns,
                                    "accepted_joint_target_rad": list(arm_target),
                                    "joint_target_source": arm_action_source,
                                    "measured_joint_position_rad": list(measured_q),
                                    "estimated_joint_velocity_rad_s": list(measured_dq),
                                    "commanded_tcp_pose_xyzw": [
                                        *tcp.position_m,
                                        *tcp.orientation_xyzw,
                                    ],
                                },
                            }
                            if feedback is None:
                                hand_observation = (0.0,) * 6
                                hand_target = hand_observation
                                hand_source = "unavailable"
                                hand_grip = False
                            else:
                                hand_observation = feedback.position_normalized
                                hand_target = (
                                    feedback.position_normalized
                                    if rh56_control.last_command_normalized is None
                                    else rh56_control.last_command_normalized
                                )
                                hand_source = "measured"
                                hand_grip = session.hand_clutch.state.value in {
                                    "reacquire",
                                    "engaged",
                                }
                                rh56_feedback_started_ns = time.perf_counter_ns()
                                rh56_record = rh56_control.episode_record(
                                    now_ns, include_diagnostics=False
                                )
                                rh56_feedback_duration_ns += (
                                    time.perf_counter_ns() - rh56_feedback_started_ns
                                )
                                raw_records["rh56_feedback"] = rh56_record
                            source_timestamps_ns = {
                                "jaka_observation": observation_ns,
                                "jaka_command": status.command_monotonic_ns,
                            }
                            source_timestamp_domains = {
                                "jaka_observation": "host_monotonic_ns",
                                "jaka_command": "host_monotonic_ns",
                            }
                            if feedback is not None:
                                source_timestamps_ns["rh56_angle_act"] = feedback.monotonic_ns
                                source_timestamp_domains["rh56_angle_act"] = "host_monotonic_ns"
                            episode_metadata_started_ns = time.perf_counter_ns()
                            try:
                                episode_runtime.collector.ingest_control(
                                    ControlSample(
                                        host_monotonic_ns=now_ns,
                                        accepted_arm_q=arm_target,
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
                                        arm_action_status=(
                                            "held_rejected"
                                            if event.get("control_state") == "HOLD_REJECTED"
                                            else "accepted"
                                        ),
                                        arm_action_source=arm_action_source,
                                        accepted_target_sequence=accepted_target_sequence,
                                        source_timestamps_ns=source_timestamps_ns,
                                        source_timestamp_domains=source_timestamp_domains,
                                        control_heartbeat_valid=not dispatch_failed,
                                        controller_fault=bool(status.error_code),
                                    ),
                                    reference_established=True,
                                    capture_active=episode_capture_active,
                                    raw_records=raw_records,
                                )
                                episode_runtime.update_preview(
                                    arm_trigger=engaged,
                                    hand_grip=hand_grip,
                                )
                                episode_record_next_ns = now_ns + episode_record_period_ns
                            except BaseException as exc:
                                mark_episode_capture_failed(
                                    f"recording_runtime_failure:{type(exc).__name__}:{exc}"
                                )
                            episode_metadata_duration_ns += (
                                time.perf_counter_ns() - episode_metadata_started_ns
                            )
                        event["producer_outer_timing_ms"] = {
                            "receiver_drain_and_router_ingest": (
                                pending_receiver_drain_and_ingest_ns / 1e6
                            ),
                            "controller_router_poll": router_poll_ns / 1e6,
                            "native_status_and_pause_sync": status_sync_ns / 1e6,
                            "shared_session_control_tick": (
                                session_control_tick_ns / 1e6
                            ),
                            "pre_log_outer_tick": (
                                time.perf_counter_ns() - outer_tick_started_ns
                            )
                            / 1e6,
                        }
                        receiver_ingest_duration_ns = (
                            pending_receiver_drain_and_ingest_ns
                        )
                        pending_receiver_drain_and_ingest_ns = 0
                        serialize_ns = 0
                        write_ns = 0
                        if event_log_due:
                            log_record = dict(event)
                            # JSON encoding and filesystem writes run in the
                            # bounded event-log worker, outside control.
                            serialize_ns = 0
                            write_started_ns = time.perf_counter_ns()
                            log.write(log_record)
                            write_ns = time.perf_counter_ns() - write_started_ns
                            if episode_runtime is not None:
                                event_log_next_ns = now_ns + episode_record_period_ns
                        outer_event_diagnostic_duration_ns = max(
                            0,
                            time.perf_counter_ns()
                            - outer_event_diagnostic_started_ns
                            - rh56_feedback_duration_ns
                            - episode_metadata_duration_ns,
                        )
                        session.add_control_timing(
                            "quest_input_duration_ns",
                            receiver_ingest_duration_ns + router_poll_ns,
                        )
                        session.add_control_timing(
                            "rh56_feedback_duration_ns",
                            rh56_feedback_duration_ns,
                        )
                        session.add_control_timing(
                            "episode_metadata_publish_duration_ns",
                            episode_metadata_duration_ns,
                        )
                        session.add_control_timing(
                            "event_diagnostic_duration_ns",
                            outer_event_diagnostic_duration_ns,
                        )
                        outer_control_total_ns = (
                            time.perf_counter_ns() - outer_tick_started_ns
                        )
                        session.finalize_control_timing(outer_control_total_ns)
                        session.update_control_timing_context(
                            {
                                "rh56_feedback_read": rh56_feedback_duration_ns > 0,
                                "episode_metadata_published": episode_metadata_duration_ns > 0,
                                "event_log_enqueued": bool(event_log_due),
                                "camera_health": (
                                    "disabled"
                                    if episode_runtime is None
                                    else "degraded"
                                    if episode_capture_failed
                                    else "healthy"
                                ),
                                "recorder_health": (
                                    "disabled"
                                    if episode_runtime is None
                                    else "degraded"
                                    if episode_capture_failed
                                    else "healthy"
                                ),
                            }
                        )
                        producer_timing_rows.append({
                            **event["producer_outer_timing_ms"],
                            "event_json_serialize": serialize_ns / 1e6,
                            "event_log_write": write_ns / 1e6,
                            "complete_outer_tick": (
                                time.perf_counter_ns() - outer_tick_started_ns
                            )
                            / 1e6,
                        })
                        # The physical path has already persisted this complete
                        # event to JSONL.  Keeping a second, unbounded in-memory
                        # copy makes cyclic-GC scans grow with episode length and
                        # can pause the command producer long enough to trip the
                        # native liveness watchdog.  Simulation/replay retain
                        # their event history for report generation; only this
                        # streaming hardware path releases persisted records.
                        session.event_records.clear()
                        event.clear()
                        if dispatch_failed or pause_failed:
                            break
                        skipped = max(0, int((now - next_tick) * target_hz))
                        next_tick += (skipped + 1) / target_hz
                finally:
                    component_placement_snapshots.append(
                        _component_placement_snapshot(
                            boundary="combined_receiver_shutdown",
                            native=native,
                            receiver=receiver,
                            rh56_worker=rh56_worker,
                        )
                    )
                    receiver.close()
            event_log_diagnostics = {
                "drop_count": log.drop_count,
                "error_count": log.error_count,
            }
        except KeyboardInterrupt:
            stop_reason = "operator_keyboard_stop"
            jaka_adapter.stop()
        except Exception:
            # An exception escaping the producer must never be recorded as a
            # duration-complete episode.  Preserve the traceback for the
            # operator, but let the lifecycle cleanup finalize the active
            # writer as aborted so the directory cannot look trainable.
            if abort_reason is None:
                abort_reason = (
                    "rh56_transport_or_feedback_fault"
                    if rh56_worker is not None and rh56_worker.failed
                    else "teleop_runtime_exception"
                )
                stop_reason = abort_reason
            raise
        finally:
            if rh56_worker is not None:
                if abort_reason is not None:
                    rh56_worker.arm_terminal_stop(abort_reason)
                else:
                    rh56_worker.hold(stop_reason)
            if not jaka_adapter.stopped:
                jaka_adapter.stop()
            if native_started and native.process is not None and native.process.poll() is None:
                try:
                    native.process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    native.stop()
            elif native_started:
                native.stop()
            runtime.close()
            if rh56_worker is not None:
                rh56_worker.cleanup()
            if rh56_recorder is not None:
                rh56_recorder.close()
            if rh56_log is not None:
                rh56_log.close()
            if episode_runtime is not None:
                try:
                    if episode_runtime.collector.state is CaptureState.REC:
                        if (
                            abort_reason is None
                            and stop_reason in {"duration_complete", "operator_keyboard_stop"}
                            and episode_runtime.dataset_format == "lerobot_staging_v1"
                        ):
                            episode_runtime.collector.discard_current(
                                "outer_session_ended_before_episode_boundary"
                            )
                        elif abort_reason is None and stop_reason in {
                            "duration_complete",
                            "operator_keyboard_stop",
                        }:
                            episode_runtime.collector.finish(stop_reason)
                        else:
                            episode_runtime.collector.abort(
                                abort_reason or stop_reason
                            )
                    else:
                        episode_runtime.collector.shutdown(
                            abort_reason or stop_reason
                        )
                    # Hardware/control cleanup is complete before this
                    # bounded recorder drain.  Recorder finalization errors
                    # are recording-only and must not replace a robot fault.
                    episode_runtime.collector.finalize_pending()
                    episode_recorder_diagnostics = (
                        episode_runtime.collector.writer.diagnostics()
                    )
                    episode_camera_diagnostics = {
                        role: camera.profile_metadata()
                        for role, camera in episode_runtime.cameras.items()
                    }
                    episode_quality_diagnostics = (
                        episode_runtime.collector.diagnostics()
                    )
                    episode_preview_diagnostics = (
                        None
                        if episode_runtime.preview is None
                        else episode_runtime.preview.diagnostics()
                    )
                    if episode_runtime.preview_failure_reason is not None:
                        if episode_preview_diagnostics is None:
                            episode_preview_diagnostics = {}
                        episode_preview_diagnostics["failure_reason"] = (
                            episode_runtime.preview_failure_reason
                        )
                    print(
                        f"EPISODE_RESULT={episode_runtime.collector.result}",
                        flush=True,
                    )
                except BaseException as exc:
                    mark_episode_capture_failed(
                        f"recording_cleanup_failure:{type(exc).__name__}:{exc}"
                    )
                finally:
                    try:
                        episode_runtime.close()
                    except BaseException as exc:
                        mark_episode_capture_failed(
                            f"recording_close_failure:{type(exc).__name__}:{exc}"
                        )

    metrics = json.loads(args.metrics.read_text(encoding="utf-8"))
    abort_reason, transport_symptom_reason = (
        _reconcile_terminal_transport_symptom(abort_reason, metrics)
    )
    if transport_symptom_reason is not None:
        stop_reason = abort_reason or stop_reason
    if args.event_extract is not None and args.native_telemetry is not None:
        _write_event_extract(
            args.native_telemetry,
            args.event_extract,
            metrics,
            clutch_release_monotonic_ns=clutch_release_monotonic_ns,
        )
    measured_tcp = _measured_tcp_motion(target_generator, measured_joint_samples)
    summary = {
        "schema_version": "quest_jaka_physical_gate.v1",
        "stage": args.stage,
        "requested_duration_sec": args.duration_sec,
        "shared_config": str(args.config),
        "runtime_config": (
            None if args.runtime_config is None else str(args.runtime_config)
        ),
        "target_displacement_limit_enabled": (
            config.feasibility.target_displacement_limit_enabled
        ),
        "output_generator": (
            args.output_generator
            if args.output_generator is not None
            else configured_output_generator
        ),
        "transport_mode": hardware.get("transport_mode", "jaka_125hz_step1"),
        "transport_hz": float(config.raw["rates"]["jaka_transport_hz"]),
        "servo_period_ms": float(hardware["servo_period_ms"]),
        "servo_step_num": servo_step_num,
        "run_output_joint_velocity_limits_rad_s": list(
            config.output_contract.velocity_boundaries_rad_s
        ),
        "accepted_targets_dispatched": accepted,
        "adapter_dispatch_count": jaka_adapter.applied_count,
        "native_mode": metrics["mode"],
        "output_joint_jerk_limit_rad_s3": jerk_limit,
        "output_joint_jerk_limit_provenance": (
            "config"
            if "command_maximum_joint_jerk_rad_s3"
            in config.raw.get("simulation", {})
            else "project_default"
        ),
        "native_outcome": metrics["outcome"],
        "native_stop_classification": metrics.get(
            "stop_classification", "worker_exit"
        ),
        "recoverable_output_acceleration_hold_count": metrics.get(
            "recoverable_output_acceleration_hold_count", 0
        ),
        "transition_limited_progress_points": metrics.get(
            "transition_limited_progress_points", 0
        ),
        "true_output_hold_count": metrics.get("true_output_hold_count", 0),
        "recovered_from_true_output_hold_count": metrics.get(
            "recovered_from_true_output_hold_count", 0
        ),
        "recovered_from_output_acceleration_hold_count": metrics.get(
            "recovered_from_output_acceleration_hold_count", 0
        ),
        "output_acceleration_hold_total_duration_ns": metrics.get(
            "output_acceleration_hold_total_duration_ns", 0
        ),
        "output_acceleration_hold_longest_duration_ns": metrics.get(
            "output_acceleration_hold_longest_duration_ns", 0
        ),
        "native_output_acceleration_hold_status_count": (
            native_output_acceleration_hold_status_count
        ),
        "native_output_acceleration_recovery_status_count": (
            native_output_acceleration_recovery_status_count
        ),
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
        "arm_transport_packets_sent": runtime.publisher.sent,
        "arm_transport_packets_dropped": runtime.publisher.dropped,
        "stop_reason": stop_reason,
        "abort_reason": abort_reason,
        "episode_capture_failed": episode_capture_failed,
        "episode_rotation_count": episode_rotation_count,
        "episode_capture_abort_reason": episode_capture_abort_reason,
        "episode_recorder_diagnostics": episode_recorder_diagnostics,
        "episode_camera_diagnostics": episode_camera_diagnostics,
        "episode_quality_diagnostics": episode_quality_diagnostics,
        "episode_preview_diagnostics": episode_preview_diagnostics,
        "event_log_diagnostics": event_log_diagnostics,
        "transport_symptom_reason": transport_symptom_reason,
        "arm_clutch_pause_count": arm_clutch_pause_count,
        "rh56_commands": (
            0
            if rh56_backend is None
            else rh56_backend.register_write_count
        ),
        "arm_commands_while_index_released": arm_commands_while_index_released,
        "rh56_device": hand_identity,
        "rh56_hand_initial_target_source": (
            None if rh56_control is None else "measured_ANGLE_ACT"
        ),
        "rh56_hand_calibration_path": (
            None
            if rh56_control is None or session.hand_retargeter is None
            else str(config.raw["hand_retargeting"]["calibration_path"])
        ),
        "rh56_hand_calibration_id": (
            None
            if rh56_control is None or session.hand_retargeter is None
            else session.hand_retargeter.calibration.calibration_id
        ),
        "rh56_align_on_grip": (
            None if rh56_control is None else session.hand_align_on_grip
        ),
        "rh56_align_index_pinch_to_validated_pose": (
            None
            if rh56_control is None
            else session.hand_align_index_pinch_to_validated_pose
        ),
        "rh56_feedback_records": (
            0 if rh56_recorder is None else rh56_recorder.telemetry_record_count
        ),
        "rh56_final_record": (
            None if rh56_recorder is None else rh56_recorder.last_telemetry_record
        ),
        "rh56_fault_reason": None if rh56_control is None else rh56_control.fault_reason,
        "rh56_worker_failure": (
            None if rh56_worker is None else rh56_worker.failure_record
        ),
        "rh56_diagnostics": (
            None if rh56_worker is None else rh56_worker.diagnostics_snapshot()
        ),
        "rh56_logging": (
            None if rh56_recorder is None else rh56_recorder.summary()
        ),
        "rh56_quest_input_frame_count": len(session.input_timestamps_ns),
        "rh56_quest_input_rate_hz": _timestamp_rate_hz(
            session.input_timestamps_ns
        ),
        "rh56_grip_sample_count": len(session.grip_timestamps_ns),
        "rh56_grip_sample_rate_hz": _timestamp_rate_hz(
            session.grip_timestamps_ns
        ),
        "rh56_hand_retarget_count": len(session.hand_timestamps_ns),
        "rh56_hand_retarget_rate_hz": _timestamp_rate_hz(
            session.hand_timestamps_ns
        ),
        "combined_episode_valid": (
            args.stage != "combined-normal-teleop"
            or (
                abort_reason is None
                and not episode_capture_failed
                and rh56_control is not None
                and rh56_control.fault_reason is None
            )
        ),
        "maximum_quest_displacement_m": maximum_quest_displacement_m,
        "minimum_continuation_fraction": minimum_continuation_fraction,
        "continuation_backtrack_count": session.continuation_backtrack_count,
        "quest_input_recovery_timeout_s": config.input_recovery_timeout_s,
        "quest_input_recovery_count": session.input_recovery_count,
        "quest_input_recovery_success_count": (
            session.input_recovery_success_count
        ),
        "quest_input_recovery_timeout_count": (
            session.input_recovery_timeout_count
        ),
        "ik_rejections": dict(sorted(session.rejections.items())),
        **_control_compute_budget_summary(session),
        "control_timing": session.control_timing_report(),
        "producer_timing_ms": _producer_timing_summary(producer_timing_rows),
        "component_placement_snapshots": component_placement_snapshots,
        "native_worker_placement": metrics.get("worker_placement"),
        "native_system_boundary_observer": metrics.get(
            "system_boundary_observer"
        ),
        "cpu_isolation": cpu_isolation,
        "native_control_realtime": realtime_preflight,
        "measured_joint_fk_tcp_motion": measured_tcp,
    }
    outer_timing = summary["producer_timing_ms"].get("complete_outer_tick", {})
    summary["control_loop_duration_ns"] = {
        key: (
            int(round(float(value) * 1e6))
            if key in {"p50", "p95", "p99", "max"}
            else value
        )
        for key, value in outer_timing.items()
        if key in {"count", "p50", "p95", "p99", "max"}
    }
    summary["control_deadline_miss_count"] = int(
        metrics.get("deadline_miss_count", 0)
    )
    summary["control_hard_miss_count"] = int(metrics.get("hard_timing_misses", 0))
    args.summary.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    if episode_capture_failed:
        return 2
    return 2 if abort_reason is not None else 0


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
