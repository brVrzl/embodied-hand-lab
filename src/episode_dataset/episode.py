from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
from typing import Any, Callable, Mapping, Sequence, TextIO
import uuid

import numpy as np


SCHEMA_VERSION = "embodied_lab.single_episode.v1"
ACTION_ORDER = ("J1", "J2", "J3", "J4", "J5", "J6", "H1", "H2", "H3", "H4", "H5", "H6")
OBSERVATION_STATE_ORDER = (
    "arm_q_measured.J1",
    "arm_q_measured.J2",
    "arm_q_measured.J3",
    "arm_q_measured.J4",
    "arm_q_measured.J5",
    "arm_q_measured.J6",
    "arm_dq_measured.J1",
    "arm_dq_measured.J2",
    "arm_dq_measured.J3",
    "arm_dq_measured.J4",
    "arm_dq_measured.J5",
    "arm_dq_measured.J6",
    "tcp_pose.x_m",
    "tcp_pose.y_m",
    "tcp_pose.z_m",
    "tcp_pose.qx",
    "tcp_pose.qy",
    "tcp_pose.qz",
    "tcp_pose.qw",
    "hand.H1",
    "hand.H2",
    "hand.H3",
    "hand.H4",
    "hand.H5",
    "hand.H6",
)


class EpisodeStatus(str, Enum):
    COMPLETED = "completed"
    ABORTED = "aborted"
    INVALID = "invalid"


def _six(values: Sequence[float] | None, name: str) -> tuple[float, ...] | None:
    if values is None:
        return None
    result = tuple(float(value) for value in values)
    if len(result) != 6 or not all(np.isfinite(result)):
        raise ValueError(f"{name} must contain six finite values")
    return result


@dataclass(frozen=True, slots=True)
class ControlSample:
    host_monotonic_ns: int
    accepted_arm_q: tuple[float, ...] | None
    arm_q_measured: tuple[float, ...] | None
    arm_dq_measured: tuple[float, ...] | None
    tcp_pose_xyzw: tuple[float, ...] | None
    hand_observation: tuple[float, ...] | None
    hand_source: str
    hand_target: tuple[float, ...] | None
    arm_trigger: bool
    hand_grip: bool
    arm_q_source: str = "measured"
    arm_dq_source: str = "measured"
    tcp_pose_source: str = "measured"
    arm_action_status: str = "accepted"
    accepted_target_sequence: int | None = None
    reference_generation: int | None = None
    source_timestamps_ns: Mapping[str, int | None] | None = None
    source_timestamp_domains: Mapping[str, str] | None = None
    control_heartbeat_valid: bool = True
    tracking_hard_fault: bool = False
    controller_fault: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "accepted_arm_q", _six(self.accepted_arm_q, "accepted_arm_q"))
        object.__setattr__(self, "arm_q_measured", _six(self.arm_q_measured, "arm_q_measured"))
        object.__setattr__(self, "arm_dq_measured", _six(self.arm_dq_measured, "arm_dq_measured"))
        object.__setattr__(self, "hand_observation", _six(self.hand_observation, "hand_observation"))
        object.__setattr__(self, "hand_target", _six(self.hand_target, "hand_target"))
        if self.tcp_pose_xyzw is not None:
            pose = tuple(float(value) for value in self.tcp_pose_xyzw)
            if len(pose) != 7 or not all(np.isfinite(pose)):
                raise ValueError("tcp_pose_xyzw must contain seven finite values")
            object.__setattr__(self, "tcp_pose_xyzw", pose)
        if self.hand_source not in {"measured", "commanded", "estimated", "unavailable"}:
            raise ValueError("hand_source must describe provenance")
        for name in ("arm_q_source", "arm_dq_source", "tcp_pose_source"):
            if getattr(self, name) not in {"measured", "commanded", "estimated", "unavailable"}:
                raise ValueError(f"{name} must describe provenance")
        if self.arm_action_status not in {"accepted", "held_rejected"}:
            raise ValueError("arm_action_status must be accepted or held_rejected")


@dataclass(frozen=True, slots=True)
class CameraSample:
    role: str
    host_monotonic_ns: int
    rgb: np.ndarray
    depth_raw: np.ndarray
    device_rgb_timestamp_ms: float
    device_depth_timestamp_ms: float
    rgb_frame_number: int
    depth_frame_number: int
    rgb_timestamp_domain: str
    depth_timestamp_domain: str
    depth_aligned_to_rgb: np.ndarray | None = None

    def __post_init__(self) -> None:
        if self.role not in {"workspace", "wrist"}:
            raise ValueError("camera role must be workspace or wrist")
        if self.rgb.dtype != np.uint8 or self.rgb.ndim != 3 or self.rgb.shape[2] != 3:
            raise ValueError("rgb must be HWC uint8 RGB")
        if self.depth_raw.dtype != np.uint16 or self.depth_raw.ndim != 2:
            raise ValueError("depth_raw must be uint16")
        if self.depth_aligned_to_rgb is not None and (
            self.depth_aligned_to_rgb.dtype != np.uint16 or self.depth_aligned_to_rgb.ndim != 2
        ):
            raise ValueError("depth_aligned_to_rgb must be uint16")


@dataclass(frozen=True, slots=True)
class StartPrerequisites:
    trigger_press_monotonic_ns: int
    reference_established: bool
    accepted: ControlSample
    workspace: CameraSample
    wrist: CameraSample
    maximum_start_delta_rad: float
    maximum_hand_start_delta_rad: float

    def validate(self, *, camera_max_age_ns: int) -> None:
        if not self.reference_established:
            raise ValueError("arm reference was not established")
        if self.accepted.accepted_arm_q is None:
            raise ValueError("first AcceptedArmTarget is unavailable")
        if self.accepted.arm_q_measured is None:
            raise ValueError("initial measured arm state is unavailable")
        if self.accepted.arm_dq_measured is None:
            raise ValueError("initial measured/estimated arm velocity is unavailable")
        if self.accepted.tcp_pose_xyzw is None:
            raise ValueError("initial measured/estimated TCP pose is unavailable")
        if not self.accepted.arm_trigger:
            raise ValueError("arm trigger is not held")
        delta = max(
            abs(target - measured)
            for target, measured in zip(
                self.accepted.accepted_arm_q, self.accepted.arm_q_measured, strict=True
            )
        )
        if delta > self.maximum_start_delta_rad:
            raise ValueError(
                f"first accepted target continuity delta {delta:.9f} rad exceeds "
                f"{self.maximum_start_delta_rad:.9f} rad"
            )
        if self.accepted.hand_target is None:
            raise ValueError("initial RH56 hold/target is unavailable")
        if self.accepted.hand_observation is None:
            raise ValueError("initial RH56 observation/hold state is unavailable")
        hand_delta = max(
            abs(target - observed)
            for target, observed in zip(
                self.accepted.hand_target, self.accepted.hand_observation, strict=True
            )
        )
        if hand_delta > self.maximum_hand_start_delta_rad:
            raise ValueError(
                f"initial RH56 target delta {hand_delta:.9f} rad exceeds "
                f"{self.maximum_hand_start_delta_rad:.9f} rad"
            )
        for camera in (self.workspace, self.wrist):
            if camera.host_monotonic_ns < self.trigger_press_monotonic_ns:
                raise ValueError(f"{camera.role} camera frame predates trigger press")
            age_ns = self.accepted.host_monotonic_ns - camera.host_monotonic_ns
            if age_ns < 0 or age_ns > camera_max_age_ns:
                raise ValueError(f"{camera.role} camera frame is not fresh and causal")


@dataclass(frozen=True, slots=True)
class CanonicalSample:
    frame_index: int
    timestamp_ns: int
    control: ControlSample
    workspace: CameraSample
    wrist: CameraSample
    source_offsets_ns: Mapping[str, int]
    synchronization_valid: bool
    stale_sources: tuple[str, ...] = ()
    dropped_sources: tuple[str, ...] = ()
    nominal_slot_index: int | None = None
    missed_slots_before: int = 0
    missed_slots_after: int = 0


class CanonicalEpisodeWriter:
    """Lossless single-episode staging writer with atomic finalization."""

    def __init__(
        self,
        root: str | Path,
        *,
        task_name: str,
        operator: str,
        dataset_fps: int = 30,
        metadata: Mapping[str, Any] | None = None,
    ) -> None:
        self.root = Path(root).resolve()
        self.task_name = task_name
        self.operator = operator
        self.dataset_fps = int(dataset_fps)
        self.episode_uuid = str(uuid.uuid4())
        self.temporary_id = self.episode_uuid[:8]
        self.partial_dir = self.root / f".episode-{self.episode_uuid}.partial"
        self.final_dir = self.root / f"episode-{self.episode_uuid}"
        self._metadata_extra = dict(metadata or {})
        self._calibration_files = [
            Path(path).resolve() for path in self._metadata_extra.pop("calibration_files", [])
        ]
        self._started = False
        self._finalized = False
        self._sample_count = 0
        self._start_ns: int | None = None
        self._last_timestamp_ns: int | None = None
        self._trigger_press_ns: int | None = None
        self._total_missed_slots = 0
        self._raw_camera_keys: set[tuple[str, int, int]] = set()
        self._raw_camera_paths: dict[
            tuple[str, int, int], dict[str, Path | None]
        ] = {}
        self._final_metadata_provider: Callable[[], Mapping[str, Any]] | None = None
        self._raw_handles: dict[str, TextIO] = {}

    @property
    def sample_count(self) -> int:
        return self._sample_count

    @property
    def start_monotonic_ns(self) -> int | None:
        return self._start_ns

    def set_final_metadata_provider(
        self, provider: Callable[[], Mapping[str, Any]]
    ) -> None:
        if self._started:
            raise RuntimeError("final metadata provider must be set before episode start")
        self._final_metadata_provider = provider

    def begin(self, prerequisites: StartPrerequisites, *, camera_max_age_ns: int) -> None:
        if self._started:
            raise RuntimeError("episode already started")
        prerequisites.validate(camera_max_age_ns=camera_max_age_ns)
        missing_calibration = [str(path) for path in self._calibration_files if not path.is_file()]
        if missing_calibration:
            raise ValueError(f"calibration snapshot files do not exist: {missing_calibration}")
        calibration_names = [path.name for path in self._calibration_files]
        duplicate_names = sorted(
            {
                name
                for name in calibration_names
                if calibration_names.count(name) > 1
            }
        )
        if duplicate_names:
            raise ValueError(
                "calibration snapshot basenames must be unique: "
                + ", ".join(duplicate_names)
            )
        if self.final_dir.exists() or self.partial_dir.exists():
            raise FileExistsError(self.final_dir)
        for path in (
            self.partial_dir / "raw",
            self.partial_dir / "raw" / "cameras" / "workspace" / "rgb",
            self.partial_dir / "raw" / "cameras" / "workspace" / "depth_raw",
            self.partial_dir / "raw" / "cameras" / "workspace" / "depth_aligned_to_rgb",
            self.partial_dir / "raw" / "cameras" / "wrist" / "rgb",
            self.partial_dir / "raw" / "cameras" / "wrist" / "depth_raw",
            self.partial_dir / "raw" / "cameras" / "wrist" / "depth_aligned_to_rgb",
            self.partial_dir / "canonical" / "frames" / "workspace" / "rgb",
            self.partial_dir / "canonical" / "frames" / "workspace" / "depth_raw",
            self.partial_dir / "canonical" / "frames" / "workspace" / "depth_aligned_to_rgb",
            self.partial_dir / "canonical" / "frames" / "wrist" / "rgb",
            self.partial_dir / "canonical" / "frames" / "wrist" / "depth_raw",
            self.partial_dir / "canonical" / "frames" / "wrist" / "depth_aligned_to_rgb",
            self.partial_dir / "calibration",
            self.partial_dir / "exports",
        ):
            path.mkdir(parents=True, exist_ok=False)
        self._started = True
        self._start_ns = prerequisites.accepted.host_monotonic_ns
        self._trigger_press_ns = prerequisites.trigger_press_monotonic_ns
        metadata = self._base_metadata(prerequisites)
        snapshots = []
        for source in self._calibration_files:
            destination = self.partial_dir / "calibration" / source.name
            shutil.copy2(source, destination)
            snapshots.append(
                {
                    "path": destination.relative_to(self.partial_dir).as_posix(),
                    "source_name": source.name,
                    "sha256": file_sha256(destination),
                }
            )
        metadata["calibration_snapshot"] = {
            "files": snapshots,
            "version": metadata.get("calibration_snapshot", {}).get("version"),
        }
        self._write_json(self.partial_dir / "metadata.json", metadata)

    def append_raw(self, stream: str, record: Mapping[str, Any]) -> None:
        if not self._started or self._finalized:
            raise RuntimeError("no active episode")
        safe = stream.replace("/", "_").replace("..", "_")
        handle = self._raw_handles.get(safe)
        if handle is None:
            handle = (self.partial_dir / "raw" / f"{safe}.jsonl").open(
                "a", encoding="utf-8"
            )
            self._raw_handles[safe] = handle
        handle.write(json.dumps(record, allow_nan=False, separators=(",", ":"), sort_keys=True) + "\n")
        handle.flush()

    def append_raw_batch(self, records: Sequence[tuple[str, Mapping[str, Any]]]) -> None:
        for stream, record in records:
            self.append_raw(stream, record)

    def append_sample(self, sample: CanonicalSample) -> None:
        if not self._started or self._finalized:
            raise RuntimeError("no active episode")
        if sample.frame_index != self._sample_count:
            raise ValueError(
                f"frame_index must be contiguous: expected {self._sample_count}, got {sample.frame_index}"
            )
        if self._last_timestamp_ns is not None and sample.timestamp_ns <= self._last_timestamp_ns:
            raise ValueError("canonical timestamp must increase strictly")
        if not sample.synchronization_valid:
            raise ValueError("invalid synchronization cannot enter canonical training data")
        if sample.missed_slots_before < 0 or sample.missed_slots_after < 0:
            raise ValueError("missed canonical slot counts must be non-negative")
        nominal_slot = (
            sample.frame_index
            if sample.nominal_slot_index is None
            else int(sample.nominal_slot_index)
        )
        if nominal_slot < sample.frame_index:
            raise ValueError("nominal_slot_index cannot precede frame_index")
        paths: dict[str, dict[str, str | None]] = {}
        for camera in (sample.workspace, sample.wrist):
            paths[camera.role] = self._write_camera_sample(camera, sample.frame_index)
        record = self._canonical_record(sample, paths)
        path = self.partial_dir / "canonical" / "samples.jsonl"
        with path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, allow_nan=False, separators=(",", ":"), sort_keys=True) + "\n")
            handle.flush()
            os.fsync(handle.fileno())
        self._last_timestamp_ns = sample.timestamp_ns
        self._total_missed_slots += int(sample.missed_slots_after)
        self._sample_count += 1

    def append_raw_camera(self, camera: CameraSample) -> None:
        if not self._started or self._finalized:
            raise RuntimeError("no active episode")
        key = (camera.role, camera.rgb_frame_number, camera.depth_frame_number)
        if key in self._raw_camera_keys:
            return
        self._raw_camera_keys.add(key)
        stem = f"{camera.host_monotonic_ns}-{camera.rgb_frame_number}-{camera.depth_frame_number}.npy"
        base = self.partial_dir / "raw" / "cameras" / camera.role
        rgb_path = base / "rgb" / stem
        depth_path = base / "depth_raw" / stem
        np.save(rgb_path, camera.rgb, allow_pickle=False)
        np.save(depth_path, camera.depth_raw, allow_pickle=False)
        aligned_path = None
        if camera.depth_aligned_to_rgb is not None:
            aligned_path = base / "depth_aligned_to_rgb" / stem
            np.save(aligned_path, camera.depth_aligned_to_rgb, allow_pickle=False)
        self._raw_camera_paths[key] = {
            "rgb": rgb_path,
            "depth_raw": depth_path,
            "depth_aligned_to_rgb": aligned_path,
        }
        self.append_raw(
            f"camera_{camera.role}",
            {
                "host_monotonic_ns": camera.host_monotonic_ns,
                "rgb_device_timestamp_ms": camera.device_rgb_timestamp_ms,
                "depth_device_timestamp_ms": camera.device_depth_timestamp_ms,
                "rgb_timestamp_domain": camera.rgb_timestamp_domain,
                "depth_timestamp_domain": camera.depth_timestamp_domain,
                "rgb_frame_number": camera.rgb_frame_number,
                "depth_frame_number": camera.depth_frame_number,
                "rgb": rgb_path.relative_to(self.partial_dir).as_posix(),
                "depth_raw": depth_path.relative_to(self.partial_dir).as_posix(),
                "depth_aligned_to_rgb": (
                    None if aligned_path is None else aligned_path.relative_to(self.partial_dir).as_posix()
                ),
            },
        )

    def finalize(
        self,
        status: EpisodeStatus,
        *,
        termination_reason: str,
        trigger_release_monotonic_ns: int | None,
        report: Mapping[str, Any] | None = None,
    ) -> Path:
        if not self._started or self._finalized:
            raise RuntimeError("no active episode")
        if status is EpisodeStatus.COMPLETED and self._sample_count == 0:
            raise ValueError("empty episode cannot be completed")
        self._close_raw_handles()
        end_ns = self._last_timestamp_ns if self._last_timestamp_ns is not None else time.monotonic_ns()
        metadata_path = self.partial_dir / "metadata.json"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        validation = self.validate_staging()
        if status is EpisodeStatus.COMPLETED and not validation["valid"]:
            status = EpisodeStatus.INVALID
            termination_reason = "final_validation_failed"
        metadata.update(
            {
                "end_host_monotonic_ns": end_ns,
                "duration_s": None if self._start_ns is None else (end_ns - self._start_ns) / 1e9,
                "sample_count": self._sample_count,
                "canonical_missed_slot_count": self._total_missed_slots,
                "trigger_release_host_monotonic_ns": trigger_release_monotonic_ns,
                "completion_status": status.value,
                "termination_reason": termination_reason,
                "success_label": "unlabeled",
                "finalized": True,
            }
        )
        if self._final_metadata_provider is not None:
            try:
                metadata.update(dict(self._final_metadata_provider()))
            except Exception as exc:
                metadata["final_metadata_error"] = f"{type(exc).__name__}: {exc}"
                if status is EpisodeStatus.COMPLETED:
                    status = EpisodeStatus.INVALID
                    termination_reason = "final_metadata_snapshot_failed"
                    metadata["completion_status"] = status.value
                    metadata["termination_reason"] = termination_reason
        self._write_json(metadata_path, metadata)
        validation.update(dict(report or {}))
        self._write_json(self.partial_dir / "validation_report.json", validation)
        self._fsync_tree_metadata()
        self.partial_dir.rename(self.final_dir)
        self._finalized = True
        return self.final_dir

    def discard_rejected_start(self, reason: str) -> Path:
        """Write a report only; no episode directory is created."""
        self._close_raw_handles()
        if self.partial_dir.is_dir():
            shutil.rmtree(self.partial_dir)
        self._started = False
        self.root.mkdir(parents=True, exist_ok=True)
        report = self.root / f"rejected-start-{self.episode_uuid}.json"
        self._write_json(
            report,
            {
                "schema_version": SCHEMA_VERSION,
                "episode_uuid": self.episode_uuid,
                "completion_status": EpisodeStatus.INVALID.value,
                "termination_reason": reason,
                "episode_created": False,
            },
        )
        return report

    def validate_staging(self) -> dict[str, Any]:
        errors: list[str] = []
        sample_path = self.partial_dir / "canonical" / "samples.jsonl"
        rows = []
        if sample_path.exists():
            for line_number, line in enumerate(sample_path.read_text(encoding="utf-8").splitlines(), 1):
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError as exc:
                    errors.append(f"samples.jsonl:{line_number}: {exc}")
        if len(rows) != self._sample_count:
            errors.append("canonical sample count mismatch")
        previous = None
        for index, row in enumerate(rows):
            if row.get("frame_index") != index:
                errors.append(f"non-contiguous frame_index at row {index}")
            timestamp_ns = row.get("timestamp_host_monotonic_ns")
            if previous is not None and timestamp_ns <= previous:
                errors.append(f"non-monotonic canonical timestamp at row {index}")
            previous = timestamp_ns
            for role in ("workspace", "wrist"):
                for key in ("rgb", "depth_raw"):
                    relative = row["observation"]["images"][role][key]
                    if not (self.partial_dir / relative).is_file():
                        errors.append(f"missing {role} {key} frame {index}")
        return {
            "schema_version": SCHEMA_VERSION,
            "offline_validation": True,
            "physically_validated": False,
            "sample_count": len(rows),
            "errors": errors,
            "valid": not errors,
        }

    def _base_metadata(self, prerequisites: StartPrerequisites) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "episode_uuid": self.episode_uuid,
            "task_name": self.task_name,
            "operator": self.operator,
            "start_host_monotonic_ns": self._start_ns,
            "end_host_monotonic_ns": None,
            "dataset_fps": self.dataset_fps,
            "sample_count": 0,
            "trigger_press_host_monotonic_ns": self._trigger_press_ns,
            "trigger_release_host_monotonic_ns": None,
            "robot_model": "JAKA Mini2",
            "hand_model": "Inspire RH56DFX",
            "arm_initial_measured_q_rad": list(prerequisites.accepted.arm_q_measured or ()),
            "hand_initial_state": list(prerequisites.accepted.hand_observation or ()),
            "hand_initial_state_source": prerequisites.accepted.hand_source,
            "camera_serials": {"workspace": None, "wrist": None},
            "camera_profiles": {"workspace": None, "wrist": None},
            "calibration_snapshot": {"files": [], "version": None},
            "control_config": {"path": None, "sha256": None},
            "raw_streams": {
                "quest_raw_datagram": "unavailable",
                "quest_decoded_input": "unavailable",
                "accepted_arm_target_60hz": "unavailable",
                "emitted_arm_command_125hz": "unavailable",
                "jaka_arm_q": "unavailable",
                "jaka_arm_dq": "unavailable",
                "native_telemetry": "unavailable",
                "rh56_target": "unavailable",
                "rh56_feedback": "unavailable",
                "workspace_rgbd": "measured",
                "wrist_rgbd": "measured",
                "fault_events": "unavailable",
            },
            "action_order": list(ACTION_ORDER),
            "observation_state_order": list(OBSERVATION_STATE_ORDER),
            "units": {
                "arm_q": "rad",
                "arm_dq": "rad/s",
                "tcp_translation": "m",
                "tcp_orientation": "quaternion_xyzw",
                "hand": "rad",
                "depth_raw": "device_units_uint16",
                "timestamp": "host_monotonic_ns",
            },
            "time_alignment": {
                "policy": "latest_sample_at_or_before_canonical_timestamp",
                "future_samples_allowed": False,
                "stale_samples_copied": False,
                "missed_canonical_slots": "recorded_not_compressed",
            },
            "completion_status": None,
            "termination_reason": None,
            "success_label": "unlabeled",
            "finalized": False,
            "code": _git_state(Path(__file__).resolve().parents[2]),
            **self._metadata_extra,
        }

    def _canonical_record(
        self, sample: CanonicalSample, paths: Mapping[str, Mapping[str, str | None]]
    ) -> dict[str, Any]:
        control = sample.control
        assert control.accepted_arm_q is not None
        assert control.arm_q_measured is not None
        assert control.arm_dq_measured is not None
        assert control.tcp_pose_xyzw is not None
        assert control.hand_observation is not None
        assert control.hand_target is not None
        source_timestamps = {
            **dict(control.source_timestamps_ns or {}),
            "control": control.host_monotonic_ns,
            "workspace": sample.workspace.host_monotonic_ns,
            "wrist": sample.wrist.host_monotonic_ns,
        }
        source_domains = {
            **dict(control.source_timestamp_domains or {}),
            "control": "host_monotonic_ns",
            "workspace": "host_monotonic_ns",
            "wrist": "host_monotonic_ns",
        }
        signed_offsets: dict[str, int | None] = dict(sample.source_offsets_ns)
        for name, timestamp_ns in source_timestamps.items():
            if name not in signed_offsets:
                signed_offsets[name] = (
                    int(timestamp_ns) - sample.timestamp_ns
                    if timestamp_ns is not None
                    and source_domains.get(name) == "host_monotonic_ns"
                    else None
                )
        return {
            "timestamp": (sample.timestamp_ns - int(self._start_ns or sample.timestamp_ns)) / 1e9,
            "timestamp_host_monotonic_ns": sample.timestamp_ns,
            "frame_index": sample.frame_index,
            "observation": {
                "images": paths,
                "state": {
                    "arm_q_measured": list(control.arm_q_measured),
                    "arm_q_source": control.arm_q_source,
                    "arm_dq_measured": list(control.arm_dq_measured),
                    "arm_dq_source": control.arm_dq_source,
                    "tcp_pose": list(control.tcp_pose_xyzw),
                    "tcp_pose_source": control.tcp_pose_source,
                    "hand": list(control.hand_observation),
                    "hand_source": control.hand_source,
                    "arm_trigger": control.arm_trigger,
                    "hand_grip": control.hand_grip,
                },
            },
            "action": {
                "arm_q_target": list(control.accepted_arm_q),
                "hand_target": list(control.hand_target),
                "arm_status": control.arm_action_status,
            },
            "timing": {
                "source_timestamps_ns": source_timestamps,
                "source_timestamp_domains": source_domains,
                "signed_offsets_ns": signed_offsets,
                "synchronization_valid": sample.synchronization_valid,
                "stale_sources": list(sample.stale_sources),
                "dropped_sources": list(sample.dropped_sources),
                "nominal_slot_index": (
                    sample.frame_index
                    if sample.nominal_slot_index is None
                    else int(sample.nominal_slot_index)
                ),
                "missed_slots_before": int(sample.missed_slots_before),
                "missed_slots_after": int(sample.missed_slots_after),
                "timing_valid": (
                    sample.missed_slots_before == 0
                    and sample.missed_slots_after == 0
                ),
            },
            "camera": {
                role: {
                    "rgb_device_timestamp_ms": camera.device_rgb_timestamp_ms,
                    "depth_device_timestamp_ms": camera.device_depth_timestamp_ms,
                    "rgb_timestamp_domain": camera.rgb_timestamp_domain,
                    "depth_timestamp_domain": camera.depth_timestamp_domain,
                    "rgb_frame_number": camera.rgb_frame_number,
                    "depth_frame_number": camera.depth_frame_number,
                }
                for role, camera in (("workspace", sample.workspace), ("wrist", sample.wrist))
            },
        }

    def _write_camera_sample(self, camera: CameraSample, index: int) -> dict[str, str | None]:
        stem = f"{index:06d}.npy"
        base = self.partial_dir / "canonical" / "frames" / camera.role
        key = (camera.role, camera.rgb_frame_number, camera.depth_frame_number)
        raw_paths = self._raw_camera_paths.get(key)
        if raw_paths is None:
            raise RuntimeError(f"canonical camera sample {key} was not retained in raw layer")
        linked: dict[str, Path | None] = {}
        for name in ("rgb", "depth_raw", "depth_aligned_to_rgb"):
            source = raw_paths[name]
            if source is None:
                linked[name] = None
                continue
            destination = base / name / stem
            os.link(source, destination)
            linked[name] = destination
        return {
            name: None if path is None else path.relative_to(self.partial_dir).as_posix()
            for name, path in linked.items()
        }

    def _fsync_tree_metadata(self) -> None:
        for relative in ("metadata.json", "validation_report.json", "canonical/samples.jsonl"):
            path = self.partial_dir / relative
            if path.exists():
                with path.open("rb") as handle:
                    os.fsync(handle.fileno())

    def _close_raw_handles(self) -> None:
        handles, self._raw_handles = self._raw_handles, {}
        for handle in handles.values():
            try:
                handle.flush()
            finally:
                handle.close()

    @staticmethod
    def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)


def file_sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git_state(cwd: Path) -> dict[str, Any]:
    explicit_revision = os.environ.get("EMBODIED_LAB_SOURCE_REVISION")
    if explicit_revision:
        return {
            "commit": explicit_revision,
            "dirty": None,
            "source": "EMBODIED_LAB_SOURCE_REVISION",
        }
    if not (cwd / ".git").exists():
        return {"commit": None, "dirty": None, "source": "no_repository_metadata"}
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=cwd, check=True, text=True, capture_output=True
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                ["git", "status", "--porcelain"], cwd=cwd, check=True, text=True, capture_output=True
            ).stdout
        )
        return {"commit": commit, "dirty": dirty, "source": "git"}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": None, "dirty": None, "source": "git_unavailable"}
