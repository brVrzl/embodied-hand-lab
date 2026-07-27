"""One semantic evaluator for offline PWL, Ruckig, and B/C references.

This module has no hardware or SDK imports.  It evaluates generated joint
commands against tracked AcceptedJointTarget fixtures on one causal 125 Hz
timeline and one explicit MuJoCo palm-model frame.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import statistics
import time
from typing import Iterable, Literal, Sequence

import mujoco
import numpy as np
import ruckig

from .cpp_shaping import CppReferenceShaper, StopReason
from .shapers import (
    JerkBoundedPositionServo,
    ResolvedRateVelocityServo,
    ShaperLimits,
)


PERIOD_NS = 8_000_000
PERIOD_S = PERIOD_NS / 1e9
Joint = tuple[float, float, float, float, float, float]
BackendName = Literal[
    "historical_pre_gate_pwl_emitted_resampled",
    "production_style_pwl_reconstruction",
    "selected_ruckig_position_otg",
    "candidate_b_resolved_rate_reference",
    "candidate_c_position_reference",
    "candidate_c_cpp_reference",
]
ALL_BACKENDS: tuple[BackendName, ...] = (
    "historical_pre_gate_pwl_emitted_resampled",
    "production_style_pwl_reconstruction",
    "selected_ruckig_position_otg",
    "candidate_b_resolved_rate_reference",
    "candidate_c_position_reference",
)
JOINT_NAMES = tuple(f"jaka_joint_{index}" for index in range(1, 7))
PALM_FRAME = "rh56_R_hand_base_link"


def _joint(values: Iterable[float]) -> Joint:
    result = tuple(float(value) for value in values)
    if len(result) != 6 or not all(math.isfinite(value) for value in result):
        raise ValueError("expected six finite joint values")
    return result  # type: ignore[return-value]


def _percentiles(values: Sequence[float]) -> dict[str, float]:
    array = np.asarray(values, dtype=float)
    if not len(array) or not np.all(np.isfinite(array)):
        raise ValueError("percentiles require finite samples")
    return {
        "mean": float(np.mean(array)),
        "p50": float(np.percentile(array, 50)),
        "p95": float(np.percentile(array, 95)),
        "p99": float(np.percentile(array, 99)),
        "p99_9": float(np.percentile(array, 99.9)),
        "max": float(np.max(array)),
    }


@dataclass(frozen=True, slots=True)
class AcceptedJointTarget:
    sequence: int
    control_monotonic_ns: int
    accepted_joint_target_rad: Joint
    accepted: bool
    output_applied: bool
    target_state: str


@dataclass(frozen=True, slots=True)
class AcceptedFixture:
    path: Path
    sha256: str
    targets: tuple[AcceptedJointTarget, ...]
    timestamp_monotonic: bool

    def metadata(self) -> dict[str, object]:
        intervals_ms = [
            (right.control_monotonic_ns - left.control_monotonic_ns) / 1e6
            for left, right in zip(self.targets, self.targets[1:])
        ]
        return {
            "path": str(self.path),
            "sha256": self.sha256,
            "sample_count": len(self.targets),
            "first_sequence": self.targets[0].sequence,
            "last_sequence": self.targets[-1].sequence,
            "first_timestamp_ns": self.targets[0].control_monotonic_ns,
            "last_timestamp_ns": self.targets[-1].control_monotonic_ns,
            "timestamp_monotonic": self.timestamp_monotonic,
            "all_accepted": all(target.accepted for target in self.targets),
            "all_output_applied": all(target.output_applied for target in self.targets),
            "target_state_counts": {
                state: sum(target.target_state == state for target in self.targets)
                for state in sorted({target.target_state for target in self.targets})
            },
            "target_interval_ms": _percentiles(intervals_ms),
            "initial_joint_rad": list(self.targets[0].accepted_joint_target_rad),
            "final_joint_rad": list(self.targets[-1].accepted_joint_target_rad),
            "joint_order": list(JOINT_NAMES),
        }


def load_accepted_joint_targets(path: Path) -> AcceptedFixture:
    targets: list[AcceptedJointTarget] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        row = json.loads(line)
        accepted = bool(row.get("accepted"))
        output_applied = bool(row.get("output_applied"))
        if not (accepted and output_applied):
            continue
        sequence = int(row.get("accepted_target_sequence", len(targets) + 1))
        timestamp_ns = int(row["control_monotonic_ns"])
        if sequence <= 0 or timestamp_ns <= 0:
            raise ValueError(f"invalid accepted target identity on line {line_number}")
        targets.append(
            AcceptedJointTarget(
                sequence=sequence,
                control_monotonic_ns=timestamp_ns,
                accepted_joint_target_rad=_joint(row["accepted_joint_target_rad"]),
                accepted=accepted,
                output_applied=output_applied,
                target_state=str(row.get("target_state", "ACTIVE")),
            )
        )
    if len(targets) < 2:
        raise ValueError("fixture requires at least two accepted/applied targets")
    monotonic = all(
        right.control_monotonic_ns > left.control_monotonic_ns
        for left, right in zip(targets, targets[1:])
    )
    if not monotonic:
        raise ValueError("accepted target timestamps must increase")
    if any(right.sequence <= left.sequence for left, right in zip(targets, targets[1:])):
        raise ValueError("accepted target sequences must increase")
    return AcceptedFixture(
        path=path,
        sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        targets=tuple(targets),
        timestamp_monotonic=True,
    )


@dataclass(frozen=True, slots=True)
class TimeGrid:
    replay_start_ns: int
    active_ns: tuple[int, ...]
    settling_ns: tuple[int, ...]
    release_ns: int
    period_ns: int


def build_time_grid(
    fixture: AcceptedFixture,
    *,
    period_ns: int = PERIOD_NS,
    settling_duration_s: float = 2.0,
) -> TimeGrid:
    if period_ns <= 0 or settling_duration_s <= 0.0:
        raise ValueError("time grid period and settling duration must be positive")
    start = fixture.targets[0].control_monotonic_ns
    final_target_ns = fixture.targets[-1].control_monotonic_ns
    final_active_offset = (
        (final_target_ns - start + period_ns - 1) // period_ns
    ) * period_ns
    active = tuple(range(start, start + final_active_offset + 1, period_ns))
    settling_count = int(round(settling_duration_s * 1e9 / period_ns))
    settling = tuple(active[-1] + period_ns * index for index in range(1, settling_count + 1))
    release_offset = ((final_target_ns - start) // 2 // period_ns) * period_ns
    release = start + release_offset
    return TimeGrid(start, active, settling, release, period_ns)


def causal_target_index(targets: Sequence[AcceptedJointTarget], tick_ns: int) -> int:
    timestamps = [target.control_monotonic_ns for target in targets]
    index = bisect_right(timestamps, tick_ns) - 1
    if index < 0:
        raise ValueError("no causal accepted target exists for tick")
    return index


def _interpolated_target(targets: Sequence[AcceptedJointTarget], tick_ns: int) -> Joint:
    if tick_ns <= targets[0].control_monotonic_ns:
        return targets[0].accepted_joint_target_rad
    if tick_ns >= targets[-1].control_monotonic_ns:
        return targets[-1].accepted_joint_target_rad
    right = bisect_right([target.control_monotonic_ns for target in targets], tick_ns)
    left_target, right_target = targets[right - 1], targets[right]
    alpha = (tick_ns - left_target.control_monotonic_ns) / (
        right_target.control_monotonic_ns - left_target.control_monotonic_ns
    )
    return _joint(
        left + alpha * (right_value - left)
        for left, right_value in zip(
            left_target.accepted_joint_target_rad,
            right_target.accepted_joint_target_rad,
            strict=True,
        )
    )


class PalmModel:
    def __init__(self, model_path: Path) -> None:
        self.path = model_path
        self.sha256 = hashlib.sha256(model_path.read_bytes()).hexdigest()
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.joint_ids = tuple(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in JOINT_NAMES
        )
        if any(joint_id < 0 for joint_id in self.joint_ids):
            raise ValueError("model is missing a JAKA joint")
        self.qpos_addresses = tuple(int(self.model.jnt_qposadr[joint_id]) for joint_id in self.joint_ids)
        self.joint_axes = tuple(
            tuple(float(value) for value in self.model.jnt_axis[joint_id])
            for joint_id in self.joint_ids
        )
        self.palm_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, PALM_FRAME
        )
        if self.palm_body_id < 0:
            raise ValueError(f"model is missing {PALM_FRAME}")

    def pose(self, joints: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
        self.data.qpos[list(self.qpos_addresses)] = joints
        mujoco.mj_forward(self.model, self.data)
        return (
            self.data.xpos[self.palm_body_id].copy(),
            self.data.xquat[self.palm_body_id].copy(),
        )

    def metadata(self) -> dict[str, object]:
        return {
            "path": str(self.path),
            "sha256": self.sha256,
            "joint_order": list(JOINT_NAMES),
            "joint_qpos_addresses": list(self.qpos_addresses),
            "joint_axes": [list(axis) for axis in self.joint_axes],
        }


def _orientation_error(left_wxyz: np.ndarray, right_wxyz: np.ndarray) -> float:
    dot = abs(float(np.dot(left_wxyz, right_wxyz)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


@dataclass(frozen=True, slots=True)
class BackendPoint:
    position: Joint
    velocity: Joint | None
    acceleration: Joint | None
    jerk: Joint | None
    cpu_us: float


class _Backend:
    name: BackendName
    limits: dict[str, Joint]
    reported_dynamics: bool = True
    stop_supported: bool = True

    def accept(self, target: AcceptedJointTarget, tick_ns: int) -> None:
        raise NotImplementedError

    def tick(self, tick_ns: int) -> BackendPoint:
        raise NotImplementedError

    def request_stop(self) -> None:
        raise NotImplementedError


class _ReferenceShaperBackend(_Backend):
    def __init__(self, name: BackendName, initial: Joint) -> None:
        limits = ShaperLimits()
        self.name = name
        self.servo = (
            ResolvedRateVelocityServo(initial, limits)
            if name == "candidate_b_resolved_rate_reference"
            else JerkBoundedPositionServo(initial, limits)
        )
        uniform = lambda value: _joint((value,) * 6)
        self.limits = {
            "velocity_rad_s": uniform(limits.maximum_velocity_rad_s),
            "acceleration_rad_s2": uniform(limits.maximum_acceleration_rad_s2),
            "jerk_rad_s3": uniform(limits.maximum_jerk_rad_s3),
        }

    def accept(self, target: AcceptedJointTarget, tick_ns: int) -> None:
        del tick_ns
        self.servo.set_target(
            target.accepted_joint_target_rad,
            timestamp_ns=target.control_monotonic_ns,
        )

    def tick(self, tick_ns: int) -> BackendPoint:
        del tick_ns
        started = time.perf_counter_ns()
        point = self.servo.tick()
        elapsed = (time.perf_counter_ns() - started) / 1e3
        return BackendPoint(
            point.position_rad,
            point.velocity_rad_s,
            point.acceleration_rad_s2,
            point.jerk_rad_s3,
            elapsed,
        )

    def request_stop(self) -> None:
        self.servo.request_controlled_stop()


class _CppReferenceBackend(_Backend):
    name: BackendName = "candidate_c_cpp_reference"

    def __init__(self, initial: Joint, library_path: Path, initialize_ns: int) -> None:
        limits = ShaperLimits()
        self.shaper = CppReferenceShaper(library_path)
        uniform = lambda value: _joint((value,) * 6)
        self.limits = {
            "velocity_rad_s": uniform(limits.maximum_velocity_rad_s),
            "acceleration_rad_s2": uniform(limits.maximum_acceleration_rad_s2),
            "jerk_rad_s3": uniform(limits.maximum_jerk_rad_s3),
        }
        self.shaper.initialize(
            position_rad=initial,
            velocity_rad_s=(0.0,) * 6,
            acceleration_rad_s2=(0.0,) * 6,
            minimum_position_rad=(-3.2,) * 6,
            maximum_position_rad=(3.2,) * 6,
            maximum_velocity_rad_s=self.limits["velocity_rad_s"],
            maximum_acceleration_rad_s2=self.limits["acceleration_rad_s2"],
            maximum_jerk_rad_s3=self.limits["jerk_rad_s3"],
            now_ns=initialize_ns,
            safety_epoch=1,
        )
        self.last_tick_ns = initialize_ns - PERIOD_NS
        self.last_sequence = 0
        self.previous_acceleration = _joint((0.0,) * 6)

    def accept(self, target: AcceptedJointTarget, tick_ns: int) -> None:
        self.shaper.replace_target(
            target.accepted_joint_target_rad,
            sequence=target.sequence,
            source_monotonic_ns=target.control_monotonic_ns,
            accepted_monotonic_ns=tick_ns,
            valid_until_monotonic_ns=tick_ns + 10_000_000_000,
        )
        self.last_sequence = target.sequence

    def tick(self, tick_ns: int) -> BackendPoint:
        started = time.perf_counter_ns()
        point = self.shaper.tick(tick_ns)
        elapsed = (time.perf_counter_ns() - started) / 1e3
        acceleration = _joint(point.acceleration_rad_s2)
        jerk = _joint(
            (new - previous) / PERIOD_S
            for new, previous in zip(
                acceleration, self.previous_acceleration, strict=True
            )
        )
        self.previous_acceleration = acceleration
        self.last_tick_ns = tick_ns
        return BackendPoint(
            _joint(point.position_rad),
            _joint(point.velocity_rad_s),
            acceleration,
            jerk,
            elapsed,
        )

    def request_stop(self) -> None:
        self.shaper.request_controlled_stop(
            release_sequence=self.last_sequence + 1,
            now_ns=self.last_tick_ns,
            reason=StopReason.CLUTCH_RELEASE,
        )


class _PwlBackend(_Backend):
    name: BackendName = "production_style_pwl_reconstruction"
    reported_dynamics = False
    stop_supported = False

    def __init__(self, initial: Joint) -> None:
        self.position = initial
        self.start = initial
        self.destination = initial
        self.segment_start_ns = 0
        self.segment_end_ns = 0
        self.last_tick_ns: int | None = None
        self.last_target_ns: int | None = None
        self.limits = {
            "velocity_rad_s": _joint((math.pi,) * 6),
            "acceleration_rad_s2": _joint((4.0 * math.pi,) * 6),
            "jerk_rad_s3": _joint((20.0 * math.pi,) * 6),
        }

    def accept(self, target: AcceptedJointTarget, tick_ns: int) -> None:
        duration_ns = (
            PERIOD_NS
            if self.last_target_ns is None
            else target.control_monotonic_ns - self.last_target_ns
        )
        if duration_ns <= 0:
            raise ValueError("PWL target timestamps must increase")
        self.start = self.position
        self.destination = target.accepted_joint_target_rad
        self.segment_start_ns = self.last_tick_ns if self.last_tick_ns is not None else tick_ns
        self.segment_end_ns = self.segment_start_ns + duration_ns
        self.last_target_ns = target.control_monotonic_ns

    def tick(self, tick_ns: int) -> BackendPoint:
        started = time.perf_counter_ns()
        if self.segment_end_ns <= self.segment_start_ns:
            alpha = 1.0
        else:
            alpha = min(
                1.0,
                max(
                    0.0,
                    (tick_ns - self.segment_start_ns)
                    / (self.segment_end_ns - self.segment_start_ns),
                ),
            )
        self.position = _joint(
            start + alpha * (target - start)
            for start, target in zip(self.start, self.destination, strict=True)
        )
        self.last_tick_ns = tick_ns
        elapsed = (time.perf_counter_ns() - started) / 1e3
        return BackendPoint(self.position, None, None, None, elapsed)

    def request_stop(self) -> None:
        raise RuntimeError("historical PWL transport has no reconstructable controlled-stop trace")


class _RuckigBackend(_Backend):
    name: BackendName = "selected_ruckig_position_otg"

    def __init__(self, initial: Joint) -> None:
        self.otg = ruckig.Ruckig(6, PERIOD_S)
        self.input = ruckig.InputParameter(6)
        self.output = ruckig.OutputParameter(6)
        velocity = _joint((2.125, 2.125, 2.125, 2.38, 1.7, 2.38))
        acceleration = _joint((8.0, 8.0, 8.0, 8.0, 6.4, 8.0))
        jerk = _joint((45.0, 45.0, 45.0, 37.5, 30.0, 37.5))
        self.limits = {
            "velocity_rad_s": velocity,
            "acceleration_rad_s2": acceleration,
            "jerk_rad_s3": jerk,
        }
        self.input.current_position = list(initial)
        self.input.current_velocity = [0.0] * 6
        self.input.current_acceleration = [0.0] * 6
        self.input.target_position = list(initial)
        self.input.target_velocity = [0.0] * 6
        self.input.target_acceleration = [0.0] * 6
        self.input.max_velocity = list(velocity)
        self.input.max_acceleration = list(acceleration)
        self.input.max_jerk = list(jerk)
        self.input.control_interface = ruckig.ControlInterface.Position
        self.input.synchronization = ruckig.Synchronization.Time
        self.input.duration_discretization = ruckig.DurationDiscretization.Discrete
        self.previous_acceleration = _joint((0.0,) * 6)

    def accept(self, target: AcceptedJointTarget, tick_ns: int) -> None:
        del tick_ns
        self.input.target_position = list(target.accepted_joint_target_rad)
        self.input.target_velocity = [0.0] * 6
        self.input.target_acceleration = [0.0] * 6
        self.input.control_interface = ruckig.ControlInterface.Position

    def tick(self, tick_ns: int) -> BackendPoint:
        del tick_ns
        if not self.otg.validate_input(self.input, False, True):
            raise RuntimeError("Ruckig rejected unified evaluator input")
        started = time.perf_counter_ns()
        result = self.otg.update(self.input, self.output)
        elapsed = (time.perf_counter_ns() - started) / 1e3
        if int(result) < 0:
            raise RuntimeError(f"Ruckig failed with result {result}")
        position = _joint(self.output.new_position)
        velocity = _joint(self.output.new_velocity)
        acceleration = _joint(self.output.new_acceleration)
        jerk = _joint(
            (new - previous) / PERIOD_S
            for new, previous in zip(acceleration, self.previous_acceleration, strict=True)
        )
        self.previous_acceleration = acceleration
        self.output.pass_to_input(self.input)
        return BackendPoint(position, velocity, acceleration, jerk, elapsed)

    def request_stop(self) -> None:
        self.input.control_interface = ruckig.ControlInterface.Velocity
        self.input.target_velocity = [0.0] * 6
        self.input.target_acceleration = [0.0] * 6


def _make_backend(
    name: BackendName,
    initial: Joint,
    *,
    cpp_library_path: Path | None = None,
    initialize_ns: int = 0,
) -> _Backend:
    if name == "historical_pre_gate_pwl_emitted_resampled":
        return _PwlBackend(initial)
    if name == "production_style_pwl_reconstruction":
        return _PwlBackend(initial)
    if name == "selected_ruckig_position_otg":
        return _RuckigBackend(initial)
    if name == "candidate_c_cpp_reference":
        if cpp_library_path is None:
            raise ValueError("C++ reference backend requires an explicit built library path")
        return _CppReferenceBackend(initial, cpp_library_path, initialize_ns)
    return _ReferenceShaperBackend(name, initial)


def _backend_configuration(name: BackendName, backend: _Backend) -> dict[str, object]:
    common: dict[str, object] = {
        "output_period_s": PERIOD_S,
        "initial_state": "fixture first accepted joint target with zero velocity/acceleration",
        "limits": {key: list(value) for key, value in backend.limits.items()},
    }
    if name == "historical_pre_gate_pwl_emitted_resampled":
        common.update(
            {
                "control_interface": "recorded joint position output",
                "target_horizon_s": None,
                "target_replacement": "recorded pre-gate worker semantics",
            }
        )
    elif name == "production_style_pwl_reconstruction":
        common.update(
            {
                "control_interface": "joint position PWL",
                "target_horizon_s": None,
                "segment_duration": "accepted target timestamp interval; first segment 8 ms",
                "target_replacement": "start from last emitted point; latest destination only",
            }
        )
    elif name == "selected_ruckig_position_otg":
        common.update(
            {
                "dependency": "ruckig 0.19.4 (MIT)",
                "control_interface": "position",
                "synchronization": "time",
                "duration_discretization": "discrete",
                "target_velocity_policy": "zero",
                "target_acceleration_policy": "zero",
                "target_horizon_s": None,
                "target_replacement": "replace destination while carrying generated q/dq/ddq state",
            }
        )
    elif name == "candidate_c_cpp_reference":
        common.update(
            {
                "control_interface": "ABI v1 C++ reference joint command model",
                "active_tracking": "independent C++ port conforming to the Python Candidate C reference law",
                "controlled_stop": "separate time-synchronized analytic jerk-limited braking mode",
                "target_horizon_s": 0.25,
                "target_replacement": "latest accepted target with source timestamp feed-forward",
                "dependency": "C++17 standard library only",
            }
        )
    else:
        common.update(
            {
                "control_interface": "reference joint command model",
                "target_horizon_s": 0.25,
                "target_replacement": "latest target with continuous generated q/dq/ddq",
                "source_timestamp_feed_forward": name == "candidate_c_position_reference",
            }
        )
    return common


def _historical_pwl_source(fixture: AcceptedFixture) -> Path:
    suffix = "_accepted_targets_20260722.jsonl"
    if not fixture.path.name.endswith(suffix):
        raise ValueError("no historical PWL output mapping exists for fixture")
    prefix = fixture.path.name[: -len(suffix)]
    path = fixture.path.parent / f"{prefix}_native_fake_20260722/corrected_fake_worker_emitted.jsonl"
    if not path.is_file():
        raise ValueError(f"historical PWL output is unavailable: {path}")
    return path


def _historical_pwl_active_points(
    fixture: AcceptedFixture, grid: TimeGrid
) -> tuple[list[BackendPoint], dict[str, object]]:
    source_path = _historical_pwl_source(fixture)
    rows = [
        json.loads(line)
        for line in source_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if len(rows) < 2:
        raise ValueError("historical PWL output has too few samples")
    raw_timestamps = np.asarray([int(row["servo_time_ns"]) for row in rows], dtype=np.int64)
    if np.any(np.diff(raw_timestamps) <= 0):
        raise ValueError("historical PWL output timestamps must increase")
    relative_ns = raw_timestamps - raw_timestamps[0]
    requested_ns = np.asarray(
        [tick - grid.replay_start_ns for tick in grid.active_ns], dtype=np.int64
    )
    if requested_ns[-1] > relative_ns[-1]:
        raise ValueError("historical PWL output timeline does not cover the common active window")
    raw_positions = np.asarray([row["joint_position_rad"] for row in rows], dtype=float)
    positions = np.column_stack(
        [
            np.interp(requested_ns, relative_ns, raw_positions[:, joint])
            for joint in range(6)
        ]
    )
    points = [
        BackendPoint(_joint(position), None, None, None, 0.0)
        for position in positions
    ]
    intervals_ns = np.diff(raw_timestamps)
    evidence = {
        "path": str(source_path),
        "sha256": hashlib.sha256(source_path.read_bytes()).hexdigest(),
        "raw_sample_count": len(rows),
        "raw_first_timestamp_ns": int(raw_timestamps[0]),
        "raw_last_timestamp_ns": int(raw_timestamps[-1]),
        "raw_interval_ns": _percentiles(intervals_ns.astype(float).tolist()),
        "common_grid_resampling": "per-joint linear interpolation in relative servo time",
        "common_active_window_covered": True,
        "settling_window_covered": False,
    }
    return points, evidence


def _run_backend(
    fixture: AcceptedFixture,
    grid: TimeGrid,
    name: BackendName,
    cpp_library_path: Path | None = None,
) -> tuple[list[BackendPoint], list[BackendPoint], int]:
    backend = _make_backend(
        name,
        fixture.targets[0].accepted_joint_target_rad,
        cpp_library_path=cpp_library_path,
        initialize_ns=grid.replay_start_ns,
    )
    target_index = 0
    active: list[BackendPoint] = []
    settling: list[BackendPoint] = []
    replacements_in_same_tick = 0
    for tick_ns in (*grid.active_ns, *grid.settling_ns):
        consumed = 0
        while (
            target_index < len(fixture.targets)
            and fixture.targets[target_index].control_monotonic_ns <= tick_ns
        ):
            backend.accept(fixture.targets[target_index], tick_ns)
            target_index += 1
            consumed += 1
        replacements_in_same_tick += max(0, consumed - 1)
        point = backend.tick(tick_ns)
        (active if tick_ns <= grid.active_ns[-1] else settling).append(point)
    return active, settling, replacements_in_same_tick


def _finite_difference(
    points: Sequence[BackendPoint], initial_position: Joint
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    positions = np.asarray([point.position for point in points], dtype=float)
    previous_positions = np.vstack((np.asarray(initial_position), positions[:-1]))
    velocity = (positions - previous_positions) / PERIOD_S
    previous_velocity = np.vstack((np.zeros(6), velocity[:-1]))
    acceleration = (velocity - previous_velocity) / PERIOD_S
    previous_acceleration = np.vstack((np.zeros(6), acceleration[:-1]))
    jerk = (acceleration - previous_acceleration) / PERIOD_S
    return velocity, acceleration, jerk


def _dynamics_summary(
    points: Sequence[BackendPoint],
    initial_position: Joint,
    backend: _Backend,
) -> dict[str, object]:
    finite_velocity, finite_acceleration, finite_jerk = _finite_difference(points, initial_position)

    def summary(velocity: np.ndarray, acceleration: np.ndarray, jerk: np.ndarray) -> dict[str, object]:
        limit_rows = (
            (velocity, np.asarray(backend.limits["velocity_rad_s"])),
            (acceleration, np.asarray(backend.limits["acceleration_rad_s2"])),
            (jerk, np.asarray(backend.limits["jerk_rad_s3"])),
        )
        saturation: dict[str, object] = {}
        for label, (values, limits) in zip(("velocity", "acceleration", "jerk"), limit_rows, strict=True):
            tolerance = np.maximum(1e-7, limits * 1e-6)
            saturated = np.abs(values) >= limits - tolerance
            violated = np.abs(values) > limits + tolerance
            transitions = np.sum(saturated[1:] != saturated[:-1], axis=0) if len(values) > 1 else np.zeros(6, dtype=int)
            saturation[label] = {
                "time_at_limit_percent_per_joint": (100.0 * np.mean(saturated, axis=0)).tolist(),
                "transition_count_per_joint": transitions.astype(int).tolist(),
                "limit_violation_count_per_joint": np.sum(violated, axis=0).astype(int).tolist(),
            }
        signs = np.sign(velocity)
        reversals = np.sum((signs[1:] * signs[:-1]) < 0.0, axis=0) if len(signs) > 1 else np.zeros(6, dtype=int)
        return {
            "sample_count": len(values),
            "joint_velocity_rad_s_peak": np.max(np.abs(velocity), axis=0).tolist(),
            "joint_acceleration_rad_s2_peak": np.max(np.abs(acceleration), axis=0).tolist(),
            "joint_jerk_rad_s3_peak": np.max(np.abs(jerk), axis=0).tolist(),
            "direction_reversals_per_joint": reversals.astype(int).tolist(),
            "saturation": saturation,
        }

    reported = None
    if backend.reported_dynamics and all(
        point.velocity is not None and point.acceleration is not None and point.jerk is not None
        for point in points
    ):
        reported = summary(
            np.asarray([point.velocity for point in points], dtype=float),
            np.asarray([point.acceleration for point in points], dtype=float),
            np.asarray([point.jerk for point in points], dtype=float),
        )
    return {
        "finite_difference_definition": (
            "backward first difference on the exact 8 ms grid; first sample uses "
            "fixture initial q, zero initial velocity, and zero initial acceleration"
        ),
        "backend_reported_available": reported is not None,
        "backend_reported": reported,
        "output_position_finite_difference": summary(
            finite_velocity, finite_acceleration, finite_jerk
        ),
        "difference_note": (
            "backend state derivatives may differ from finite differences because "
            "of integration/discretization; PWL exposes positions only"
        ),
    }


def _tracking_summary(
    outputs: Sequence[BackendPoint],
    references: Sequence[Joint],
    model: PalmModel,
    *,
    method: str,
    window_start_ns: int,
    window_end_ns: int,
) -> dict[str, object]:
    position_errors: list[float] = []
    orientation_errors: list[float] = []
    for output, reference in zip(outputs, references, strict=True):
        output_position, output_quaternion = model.pose(output.position)
        reference_position, reference_quaternion = model.pose(reference)
        position_errors.append(float(np.linalg.norm(output_position - reference_position)))
        orientation_errors.append(_orientation_error(output_quaternion, reference_quaternion))
    return {
        "reference_method": method,
        "sample_count": len(outputs),
        "window_start_ns": window_start_ns,
        "window_end_ns": window_end_ns,
        "window_duration_ms": (window_end_ns - window_start_ns) / 1e6,
        "palm_model_command_separation_m": {
            "rms": math.sqrt(statistics.fmean(value * value for value in position_errors)),
            "peak": max(position_errors),
        },
        "palm_model_orientation_separation_rad": {
            "rms": math.sqrt(statistics.fmean(value * value for value in orientation_errors)),
            "peak": max(orientation_errors),
        },
    }


def _settling_summary(
    points: Sequence[BackendPoint],
    final_target: Joint,
    model: PalmModel,
    ticks_ns: Sequence[int],
    final_target_timestamp_ns: int,
) -> dict[str, object]:
    final_position, final_quaternion = model.pose(final_target)
    position_error: list[float] = []
    orientation_error: list[float] = []
    joint_error: list[float] = []
    for point in points:
        position, quaternion = model.pose(point.position)
        position_error.append(float(np.linalg.norm(position - final_position)))
        orientation_error.append(_orientation_error(quaternion, final_quaternion))
        joint_error.append(max(abs(left - right) for left, right in zip(point.position, final_target, strict=True)))
    thresholds = {
        "loose": (0.002, 0.005, 0.005),
        "nominal": (0.0005, 0.001, 0.001),
        "strict": (0.0001, 0.0002, 0.0002),
    }
    time_to_threshold: dict[str, float | None] = {}
    time_from_window_start: dict[str, float | None] = {}
    for label, (position_limit, orientation_limit, joint_limit) in thresholds.items():
        first: int | None = None
        suffix = True
        for index in range(len(points) - 1, -1, -1):
            within = (
                position_error[index] <= position_limit
                and orientation_error[index] <= orientation_limit
                and joint_error[index] <= joint_limit
            )
            suffix = suffix and within
            if suffix:
                first = index
        time_to_threshold[label] = (
            None
            if first is None
            else (ticks_ns[first] - final_target_timestamp_ns) / 1e6
        )
        time_from_window_start[label] = (
            None if first is None else (ticks_ns[first] - ticks_ns[0]) / 1e6
        )
    velocities, _, _ = _finite_difference(points, points[0].position)
    signs = np.sign(velocities)
    reversals = np.sum((signs[1:] * signs[:-1]) < 0.0, axis=0) if len(signs) > 1 else np.zeros(6, dtype=int)
    return {
        "sample_count": len(points),
        "window_start_ns": ticks_ns[0],
        "window_end_ns": ticks_ns[-1],
        "window_duration_ms": len(points) * PERIOD_S * 1000.0,
        "start": {
            "palm_position_error_m": position_error[0],
            "palm_orientation_error_rad": orientation_error[0],
            "joint_max_abs_error_rad": joint_error[0],
        },
        "endpoint": {
            "palm_position_error_m": position_error[-1],
            "palm_orientation_error_rad": orientation_error[-1],
            "joint_max_abs_error_rad": joint_error[-1],
        },
        "thresholds": {
            label: {
                "palm_position_m": values[0],
                "palm_orientation_rad": values[1],
                "joint_max_abs_rad": values[2],
            }
            for label, values in thresholds.items()
        },
        "time_to_sustained_threshold_ms": time_to_threshold,
        "time_from_settling_window_start_ms": time_from_window_start,
        "position_error_overshoot_m": max(0.0, max(position_error) - position_error[0]),
        "orientation_error_overshoot_rad": max(0.0, max(orientation_error) - orientation_error[0]),
        "direction_reversals_per_joint": reversals.astype(int).tolist(),
        "final_joint_error_rad": [
            point - target for point, target in zip(points[-1].position, final_target, strict=True)
        ],
        "final_palm_error_m": position_error[-1],
    }


def _trajectory_shift(
    outputs: Sequence[BackendPoint],
    references: Sequence[Joint],
    model: PalmModel,
) -> dict[str, object]:
    output_pose = [model.pose(point.position) for point in outputs]
    reference_pose = [model.pose(reference) for reference in references]
    maximum = min(int(round(0.4 / PERIOD_S)), len(outputs) // 3)
    translation_scores: list[float] = []
    orientation_scores: list[float] = []
    for lag in range(maximum + 1):
        reference_slice = reference_pose[: len(reference_pose) - lag or None]
        output_slice = output_pose[lag:]
        translation = [
            float(np.linalg.norm(reference[0] - output[0]))
            for reference, output in zip(reference_slice, output_slice, strict=True)
        ]
        orientation = [
            _orientation_error(reference[1], output[1])
            for reference, output in zip(reference_slice, output_slice, strict=True)
        ]
        translation_scores.append(math.sqrt(statistics.fmean(value * value for value in translation)))
        orientation_scores.append(math.sqrt(statistics.fmean(value * value for value in orientation)))
    return {
        "classification": "non-causal trajectory-shift estimator; not latency",
        "algorithm": "discrete non-negative lag minimizing RMS on timestamp-interpolated palm poses",
        "target_representation": "joint-linear timestamp interpolation transformed to palm model pose",
        "window": "same active ticks as tracking",
        "maximum_search_ms": maximum * PERIOD_S * 1000.0,
        "translation_shift_ms": int(np.argmin(translation_scores)) * PERIOD_S * 1000.0,
        "orientation_shift_ms": int(np.argmin(orientation_scores)) * PERIOD_S * 1000.0,
    }


def _stop_summary(
    fixture: AcceptedFixture,
    grid: TimeGrid,
    name: BackendName,
    model: PalmModel,
    cpp_library_path: Path | None = None,
) -> dict[str, object]:
    backend = _make_backend(
        name,
        fixture.targets[0].accepted_joint_target_rad,
        cpp_library_path=cpp_library_path,
        initialize_ns=grid.replay_start_ns,
    )
    if not backend.stop_supported:
        return {
            "available": False,
            "release_event_ns": grid.release_ns,
            "reason": "historical production cleanup has no position-command stop trace; not reconstructed",
        }
    target_index = 0
    release_point: BackendPoint | None = None
    for tick_ns in grid.active_ns:
        while target_index < len(fixture.targets) and fixture.targets[target_index].control_monotonic_ns <= tick_ns:
            backend.accept(fixture.targets[target_index], tick_ns)
            target_index += 1
        release_point = backend.tick(tick_ns)
        if tick_ns >= grid.release_ns:
            break
    assert release_point is not None
    backend.request_stop()
    stop_points: list[BackendPoint] = []
    stable = 0
    for index in range(1000):
        point = backend.tick(grid.release_ns + (index + 1) * PERIOD_NS)
        stop_points.append(point)
        velocity = point.velocity or (0.0,) * 6
        acceleration = point.acceleration or (0.0,) * 6
        if max(map(abs, velocity)) < 1e-3 and max(map(abs, acceleration)) < 1e-2:
            stable += 1
            if stable >= 5:
                break
        else:
            stable = 0
    release_palm, _ = model.pose(release_point.position)
    palm_displacement = max(
        float(np.linalg.norm(model.pose(point.position)[0] - release_palm))
        for point in stop_points
    )
    return {
        "available": True,
        "release_event_ns": grid.release_ns,
        "release_peak_abs_velocity_rad_s": max(map(abs, release_point.velocity or (0.0,) * 6)),
        "completion_definition": "5 consecutive ticks below 1e-3 rad/s and 1e-2 rad/s^2",
        "completed": stable >= 5,
        "time_ms": len(stop_points) * PERIOD_S * 1000.0,
        "palm_stop_displacement_m": palm_displacement,
        "dynamics": _dynamics_summary(stop_points, release_point.position, backend),
    }


def _model_equivalence(
    fixture: AcceptedFixture,
    primary: PalmModel,
    alternate_path: Path,
) -> dict[str, object]:
    alternate = PalmModel(alternate_path)
    maximum_position = 0.0
    maximum_orientation = 0.0
    for target in fixture.targets:
        primary_pose = primary.pose(target.accepted_joint_target_rad)
        alternate_pose = alternate.pose(target.accepted_joint_target_rad)
        maximum_position = max(maximum_position, float(np.linalg.norm(primary_pose[0] - alternate_pose[0])))
        maximum_orientation = max(maximum_orientation, _orientation_error(primary_pose[1], alternate_pose[1]))
    return {
        "alternate_model": alternate.metadata(),
        "joint_order_equal": primary.qpos_addresses == alternate.qpos_addresses,
        "joint_axes_equal": primary.joint_axes == alternate.joint_axes,
        "reference_sample_count": len(fixture.targets),
        "maximum_palm_position_difference_m": maximum_position,
        "maximum_palm_orientation_difference_rad": maximum_orientation,
        "position_tolerance_m": 1e-12,
        "orientation_tolerance_rad": 1e-12,
        "equivalent": maximum_position <= 1e-12 and maximum_orientation <= 1e-12,
    }


def evaluate_unified_fixture(
    fixture: AcceptedFixture,
    *,
    model_path: Path,
    repository_commit: str,
    working_tree_dirty: bool,
    settling_duration_s: float = 2.0,
    backends: Sequence[BackendName] = ALL_BACKENDS,
    cpp_library_path: Path | None = None,
) -> dict[str, object]:
    grid = build_time_grid(fixture, settling_duration_s=settling_duration_s)
    model = PalmModel(model_path)
    causal_references = [
        fixture.targets[causal_target_index(fixture.targets, tick)].accepted_joint_target_rad
        for tick in grid.active_ns
    ]
    interpolated_references = [
        _interpolated_target(fixture.targets, tick) for tick in grid.active_ns
    ]
    ages_ms = [
        (tick - fixture.targets[causal_target_index(fixture.targets, tick)].control_monotonic_ns) / 1e6
        for tick in grid.active_ns
    ]
    benchmark_rows: list[dict[str, object]] = []
    for name in backends:
        backend = _make_backend(
            name,
            fixture.targets[0].accepted_joint_target_rad,
            cpp_library_path=cpp_library_path,
            initialize_ns=grid.replay_start_ns,
        )
        backend_evidence: dict[str, object] | None = None
        if name == "historical_pre_gate_pwl_emitted_resampled":
            active, backend_evidence = _historical_pwl_active_points(fixture, grid)
            settling: list[BackendPoint] | None = None
            same_tick_replacements = 0
        else:
            active, settling, same_tick_replacements = _run_backend(
                fixture, grid, name, cpp_library_path
            )
        windows = {
            "replay_start_ns": grid.replay_start_ns,
            "active": {
                "start_ns": grid.active_ns[0],
                "end_ns": grid.active_ns[-1],
                "duration_ms": (grid.active_ns[-1] - grid.active_ns[0]) / 1e6,
                "sample_count": len(grid.active_ns),
            },
            "release_event_ns": grid.release_ns,
            "settling": {
                "start_ns": grid.settling_ns[0],
                "end_ns": grid.settling_ns[-1],
                "duration_ms": len(grid.settling_ns) * PERIOD_S * 1000.0,
                "sample_count": len(grid.settling_ns),
            },
            "active_dynamics": "exactly active ticks only",
            "stop_dynamics": "release-exclusive generated stop ticks only",
        }
        limitations = [
            "offline joint-command and palm-FK model separation; no physical plant",
            "algorithm CPU excludes scheduler wake-up jitter, IPC, serialization, SDK send, controller, and network",
        ]
        if name == "historical_pre_gate_pwl_emitted_resampled":
            limitations.extend(
                [
                    "recorded pre-acceleration-gate incident output; not current production behavior",
                    "raw relative servo timeline is linearly resampled to the common grid",
                    "recorded output does not cover the common settling or controlled-stop windows",
                ]
            )
        elif name == "production_style_pwl_reconstruction":
            limitations.extend(
                [
                    "causal research reconstruction of the production PWL algorithm, not a current production-gate result",
                    "historical failed targets predate the acceleration gate; current gated behavior cannot be recovered from accepted-only fixtures",
                    "controlled cleanup output after clutch release is not present and is not invented",
                ]
            )
        elif name == "selected_ruckig_position_otg":
            limitations.append(
                "Ruckig 0.19.4 selected_time_zero position-interface configuration; limits differ from B/C"
            )
        else:
            limitations.append("architecture reference shaper, not a production candidate")
            if name == "candidate_c_cpp_reference":
                limitations.append(
                    "ctypes call CPU is executable/reference evidence, not scheduler, IPC, or realtime proof"
                )
        benchmark_rows.append(
            {
                "schema_version": "teleop_unified_benchmark.record.v1",
                "repository_commit": repository_commit,
                "working_tree_dirty": working_tree_dirty,
                "backend": name,
                "backend_evidence": backend_evidence,
                "backend_configuration": _backend_configuration(name, backend),
                "fixture": str(fixture.path),
                "fixture_sha256": fixture.sha256,
                "model": model.metadata(),
                "frame": {
                    "name": PALM_FRAME,
                    "metric_name": "palm_model_command_separation",
                    "not_physical_tcp": True,
                },
                "period_s": PERIOD_S,
                "target_reference_semantics": "latest accepted/applied target with control_monotonic_ns <= servo tick",
                "windows": windows,
                "tracking": _tracking_summary(
                    active,
                    causal_references,
                    model,
                    method="causal latest accepted target at or before tick",
                    window_start_ns=grid.active_ns[0],
                    window_end_ns=grid.active_ns[-1],
                ),
                "interpolated_tracking": {
                    **_tracking_summary(
                        active,
                        interpolated_references,
                        model,
                        method="non-causal joint-linear interpolation at tick; endpoint clamped",
                        window_start_ns=grid.active_ns[0],
                        window_end_ns=grid.active_ns[-1],
                    ),
                    "classification": "non-causal evaluator metric; not online tracking",
                    "interpolation_method": "per-joint linear interpolation in source monotonic time",
                    "endpoint_behavior": "clamp to first/final accepted joint target",
                },
                "settling": (
                    {
                        "available": False,
                        "reason": "recorded PWL output does not cover the common 2 s settling window",
                    }
                    if settling is None
                    else {
                        "available": True,
                        **_settling_summary(
                            settling,
                            fixture.targets[-1].accepted_joint_target_rad,
                            model,
                            grid.settling_ns,
                            fixture.targets[-1].control_monotonic_ns,
                        ),
                    }
                ),
                "dynamics": {
                    "active": _dynamics_summary(
                        active,
                        fixture.targets[0].accepted_joint_target_rad,
                        backend,
                    ),
                    "settling": (
                        None
                        if settling is None
                        else _dynamics_summary(
                            settling,
                            active[-1].position,
                            backend,
                        )
                    ),
                    "combined_window_is_primary": False,
                    "limits": {key: list(value) for key, value in backend.limits.items()},
                },
                "command_age": {
                    "definition": "tick_time - latest causal accepted target timestamp",
                    "ms": _percentiles(ages_ms),
                },
                "trajectory_shift_estimator": _trajectory_shift(
                    active, interpolated_references, model
                ),
                "cpu": {
                    "algorithm_tick_us": (
                        None
                        if settling is None
                        else _percentiles([point.cpu_us for point in (*active, *settling)])
                    ),
                    "unavailable_reason": (
                        "recorded output has no comparable algorithm-only timing"
                        if settling is None
                        else None
                    ),
                    "measurement_scope": (
                        "Python perf_counter around one ctypes call into the C++ reference executable library"
                        if name == "candidate_c_cpp_reference"
                        else "Python perf_counter around one backend algorithm tick"
                    ),
                    "excludes": [
                        "scheduler wake-up jitter", "IPC", "serialization", "SDK send",
                        "controller", "network",
                    ],
                },
                "mailbox": {
                    "policy": "latest accepted target wins at each tick",
                    "maximum_depth": 1,
                    "same_tick_superseded_targets": same_tick_replacements,
                    "backlog": 0,
                },
                "stop": (
                    {
                        "available": False,
                        "release_event_ns": grid.release_ns,
                        "reason": "recorded PWL output contains no matching moving clutch-release trace",
                    }
                    if name == "historical_pre_gate_pwl_emitted_resampled"
                    else _stop_summary(
                        fixture, grid, name, model, cpp_library_path
                    )
                ),
                "limitations": limitations,
            }
        )
    alternate_path = model_path.with_name("jaka_rh56_visual_coacd.xml")
    report: dict[str, object] = {
        "schema_version": "teleop_unified_benchmark.v1",
        "repository_commit": repository_commit,
        "working_tree_dirty": working_tree_dirty,
        "fixture": fixture.metadata(),
        "period_s": PERIOD_S,
        "time_grid": {
            "target_activation_rule": "control_monotonic_ns <= servo tick",
            "active_start_ns": grid.active_ns[0],
            "active_end_ns": grid.active_ns[-1],
            "release_event_ns": grid.release_ns,
            "settling_end_ns": grid.settling_ns[-1],
        },
        "frame_equivalence": _model_equivalence(fixture, model, alternate_path),
        "benchmarks": benchmark_rows,
        "historical_evidence": {
            "moveit": (
                "historical differential-IK plus Ruckig evaluation associated with the "
                "MoveIt workstream, not an actual MoveIt Servo runtime benchmark; flange "
                "metrics are not ranked against palm metrics"
            ),
            "current_production_gated_pwl": (
                "not recoverable from accepted-only historical fixtures because rejected "
                "candidate targets and the resulting hold timeline are absent"
            ),
        },
        "physical_connections": False,
        "physical_commands": False,
        "jaka_sdk_loaded": False,
        "rh56_commands": 0,
    }
    json.dumps(report, allow_nan=False)
    return report


def render_benchmark_markdown_rows(report: dict[str, object]) -> list[str]:
    rows: list[str] = []
    for record in report["benchmarks"]:  # type: ignore[index]
        tracking = record["tracking"]  # type: ignore[index]
        interpolated = record["interpolated_tracking"]  # type: ignore[index]
        causal_position = tracking["palm_model_command_separation_m"]["rms"] * 1000  # type: ignore[index]
        causal_orientation = tracking["palm_model_orientation_separation_rad"]["rms"] * 1000  # type: ignore[index]
        interpolated_position = interpolated["palm_model_command_separation_m"]["rms"] * 1000  # type: ignore[index]
        interpolated_orientation = interpolated["palm_model_orientation_separation_rad"]["rms"] * 1000  # type: ignore[index]
        fixture_name = Path(str(record["fixture"])).name
        rows.append(
            f"| {fixture_name} | {record['backend']} | {causal_position:.3f} | {causal_orientation:.3f} "
            f"| {interpolated_position:.3f} | {interpolated_orientation:.3f} |"  # type: ignore[index]
        )
    return rows
