"""Small RGB-only physical episode writer.

The physical collection path uses this writer when the dataset configuration
selects ``raw_episode_v1``.  The canonical v1/v2 writer remains available for
simulation and existing offline archives.  This format deliberately keeps the
training view small: two RGB videos, one Parquet state/action table, and one
episode metadata file.  JAKA/RH56 records, when supplied, are written to an
optional audit directory; Quest records are ignored by design.
"""

from __future__ import annotations

from collections import OrderedDict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import time
from typing import Any, Callable, Mapping
import uuid

import numpy as np

from .episode import (
    CameraFrameRef,
    CameraRecord,
    CameraSample,
    CanonicalSample,
    EpisodeStatus,
    StartPrerequisites,
    _git_state,
)


RAW_EPISODE_FORMAT_VERSION = "raw_episode_v1"


def _load_video_dependencies() -> tuple[Any, Any]:
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - depends on environment
        raise RuntimeError(
            "raw_episode_v1 requires OpenCV; install embodied-lab[dataset-collection]"
        ) from exc
    return cv2, np


def _load_parquet_dependencies() -> tuple[Any, Any]:
    try:
        import pyarrow as pa
        import pyarrow.parquet as parquet
    except ImportError as exc:  # pragma: no cover - depends on environment
        raise RuntimeError(
            "raw_episode_v1 requires PyArrow; install embodied-lab[dataset-collection]"
        ) from exc
    return pa, parquet


def _camera_key(camera: CameraRecord) -> tuple[Any, ...]:
    if isinstance(camera, CameraFrameRef):
        return (camera.role, "sequence", int(camera.sequence))
    return (
        camera.role,
        "frame",
        int(camera.host_monotonic_ns),
        int(camera.rgb_frame_number),
    )


def _materialize_camera(camera: CameraRecord) -> CameraSample:
    if isinstance(camera, CameraFrameRef):
        return camera.snapshot()
    return camera


class RawEpisodeWriter:
    """Write the four-file RGB/action episode format used by raw_episodes."""

    def __init__(
        self,
        root: str | Path,
        *,
        task_name: str,
        operator: str,
        dataset_fps: int = 30,
        metadata: Mapping[str, Any] | None = None,
        video_codec: str = "mp4v",
    ) -> None:
        self.root = Path(root).resolve()
        self.task_name = str(task_name)
        self.operator = str(operator)
        self._dataset_fps = int(dataset_fps)
        if self._dataset_fps <= 0:
            raise ValueError("dataset_fps must be positive")
        self.video_codec = str(video_codec)
        if len(self.video_codec) != 4:
            raise ValueError("video_codec must be a four-character OpenCV codec")
        self.episode_uuid = str(uuid.uuid4())
        self.temporary_id = self.episode_uuid[:8]
        self.partial_dir = self.root / f".episode-{self.episode_uuid}.partial"
        self.final_dir = self.root / f"episode-{self.episode_uuid}"
        self._metadata_extra = dict(metadata or {})
        self._started = False
        self._finalized = False
        self._start_ns: int | None = None
        self._last_timestamp_ns: int | None = None
        self._trigger_release_ns: int | None = None
        self._rows: list[dict[str, Any]] = []
        self._frame_quality_rows: list[dict[str, Any]] = []
        self._quality_events: list[dict[str, Any]] = []
        self._camera_cache: OrderedDict[tuple[Any, ...], CameraSample] = OrderedDict()
        self._audit_handles: dict[str, Any] = {}
        self._videos: dict[str, Any] = {}
        self._video_frame_counts = {"fixed_camera": 0, "wrist_camera": 0}
        self._video_shape: tuple[int, int] | None = None
        self._bytes_written = 0
        self._final_metadata_provider: Callable[[], Mapping[str, Any]] | None = None

    @property
    def sample_count(self) -> int:
        return len(self._rows)

    @property
    def dataset_fps(self) -> int:
        return self._dataset_fps

    @property
    def start_monotonic_ns(self) -> int | None:
        return self._start_ns

    @property
    def bytes_written(self) -> int:
        return self._bytes_written

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
        if self.final_dir.exists() or self.partial_dir.exists():
            raise FileExistsError(self.final_dir)
        self.partial_dir.mkdir(parents=True, exist_ok=False)
        (self.partial_dir / "audit").mkdir()
        self._started = True
        self._start_ns = prerequisites.accepted.host_monotonic_ns
        self._write_episode_metadata(
            {
                "format_version": RAW_EPISODE_FORMAT_VERSION,
                "episode_uuid": self.episode_uuid,
                "episode_index": None,
                "recording_status": "recording",
                "task_instruction": self.task_name,
                "task_name": self.task_name,
                "operator": self.operator,
                "success": None,
                "success_label": "unlabeled",
                "fps": self._dataset_fps,
                "num_frames": 0,
                "duration_s": 0.0,
                "timestamp_semantics": {
                    "clock": "monotonic_ns",
                    "video_alignment": (
                        "video frame i aligns with frames.parquet row "
                        "frame_index=i"
                    ),
                },
                "channel_order": {
                    "observation.state": [
                        "arm_q1_actual",
                        "arm_q2_actual",
                        "arm_q3_actual",
                        "arm_q4_actual",
                        "arm_q5_actual",
                        "arm_q6_actual",
                        "hand_h1_actual",
                        "hand_h2_actual",
                        "hand_h3_actual",
                        "hand_h4_actual",
                        "hand_h5_actual",
                        "hand_h6_actual",
                    ],
                    "action": [
                        "arm_q1_target_sent",
                        "arm_q2_target_sent",
                        "arm_q3_target_sent",
                        "arm_q4_target_sent",
                        "arm_q5_target_sent",
                        "arm_q6_target_sent",
                        "hand_h1_target_sent",
                        "hand_h2_target_sent",
                        "hand_h3_target_sent",
                        "hand_h4_target_sent",
                        "hand_h5_target_sent",
                        "hand_h6_target_sent",
                    ],
                },
                "units": {
                    "observation.state[0:6]": "rad",
                    "observation.state[6:12]": "normalized_0_1",
                    "action[0:6]": "rad",
                    "action[6:12]": "normalized_0_1",
                },
                "files": {
                    "fixed_camera": "fixed_camera.mp4",
                    "wrist_camera": "wrist_camera.mp4",
                    "frames": "frames.parquet",
                },
                "video": {
                    "codec": self.video_codec,
                    "fps": self._dataset_fps,
                },
                "depth_recorded": False,
                "quest_recorded": False,
                "tcp_recorded": False,
                "code": _git_state(Path(__file__).resolve().parents[2]),
                **self._metadata_extra,
            }
        )

    def append_raw(self, stream: str, record: Mapping[str, Any]) -> bool:
        if not self._started or self._finalized:
            raise RuntimeError("no active episode")
        # Quest is a control input, not part of the standard episode.  Do not
        # retain the mutable decoded event or raw packets in this writer.
        if stream.startswith("quest"):
            return True
        if stream == "data_quality":
            self._quality_events.append(dict(record))
            self._append_audit(stream, record)
            return True
        if stream in {"jaka_state", "rh56_feedback"}:
            self._append_audit(stream, record)
        return True

    def append_raw_batch(self, records: list[tuple[str, Mapping[str, Any]]]) -> bool:
        for stream, record in records:
            self.append_raw(stream, record)
        return True

    def append_raw_camera(self, camera: CameraRecord) -> bool:
        if not self._started or self._finalized:
            raise RuntimeError("no active episode")
        materialized = _materialize_camera(camera)
        self._camera_cache[_camera_key(camera)] = materialized
        self._camera_cache.move_to_end(_camera_key(camera))
        while len(self._camera_cache) > 64:
            self._camera_cache.popitem(last=False)
        return True

    def append_sample(self, sample: CanonicalSample) -> bool:
        if not self._started or self._finalized:
            raise RuntimeError("no active episode")
        expected = len(self._rows)
        if sample.frame_index != expected:
            raise ValueError(f"frame_index must be contiguous: expected {expected}, got {sample.frame_index}")
        if self._last_timestamp_ns is not None and sample.timestamp_ns <= self._last_timestamp_ns:
            raise ValueError("canonical timestamp must increase strictly")
        control = sample.control
        if (
            control.arm_q_measured is None
            or control.hand_observation is None
            or control.accepted_arm_q is None
            or control.hand_target is None
        ):
            raise ValueError("raw_episode_v1 requires measured state and final action")
        workspace = self._get_camera(sample.workspace)
        wrist = self._get_camera(sample.wrist)
        self._write_video("fixed_camera", workspace.rgb)
        self._write_video("wrist_camera", wrist.rgb)
        self._rows.append(
            {
                "frame_index": int(sample.frame_index),
                "timestamp_ns": int(sample.timestamp_ns),
                "observation.state": [
                    *control.arm_q_measured,
                    *control.hand_observation,
                ],
                "action": [
                    *control.accepted_arm_q,
                    *control.hand_target,
                ],
            }
        )
        self._frame_quality_rows.append(
            {
                "frame_index": int(sample.frame_index),
                "action_status": control.arm_action_status,
                "timing_valid": bool(
                    sample.missed_slots_before == 0
                    and sample.missed_slots_after == 0
                ),
                "arm_trigger": bool(control.arm_trigger),
                "hand_grip": bool(control.hand_grip),
                "nominal_slot_index": int(sample.nominal_slot_index or sample.frame_index),
                "missed_slots_before": int(sample.missed_slots_before),
                "missed_slots_after": int(sample.missed_slots_after),
            }
        )
        self._last_timestamp_ns = sample.timestamp_ns
        return True

    def flush_pending(self) -> None:
        for handle in self._audit_handles.values():
            handle.flush()

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
        if status is EpisodeStatus.COMPLETED and not self._rows:
            raise ValueError("empty episode cannot be completed")
        self._trigger_release_ns = trigger_release_monotonic_ns
        self._close_videos()
        self._close_audit_handles()
        self._write_frames_parquet()
        self._write_quality_parquet_if_needed()
        metadata = json.loads(
            (self.partial_dir / "episode.json").read_text(encoding="utf-8")
        )
        end_ns = self._last_timestamp_ns or time.monotonic_ns()
        metadata.update(
            {
                "recording_status": "complete" if status is EpisodeStatus.COMPLETED else status.value,
                "completion_status": status.value,
                "termination_reason": termination_reason,
                "num_frames": len(self._rows),
                "duration_s": None if self._start_ns is None else (end_ns - self._start_ns) / 1e9,
                "end_timestamp_ns": int(end_ns),
                "trigger_release_timestamp_ns": trigger_release_monotonic_ns,
                "finalized_timestamp_ns": time.monotonic_ns(),
                "quality": {
                    "timing_invalid_frame_count": sum(
                        not bool(row["timing_valid"])
                        for row in self._frame_quality_rows
                    ),
                    "held_rejected_frame_count": sum(
                        row["action_status"] == "held_rejected"
                        for row in self._frame_quality_rows
                    ),
                },
            }
        )
        audit_files = sorted(
            path.relative_to(self.partial_dir).as_posix()
            for path in (self.partial_dir / "audit").glob("*.jsonl")
            if path.is_file()
        )
        if audit_files:
            metadata.setdefault("files", {})["audit"] = audit_files
        if self._final_metadata_provider is not None:
            metadata.update(dict(self._final_metadata_provider()))
        if report:
            metadata["recorder_report"] = dict(report)
        self._write_episode_metadata(metadata)
        self._fsync_file(self.partial_dir / "episode.json")
        self.partial_dir.rename(self.final_dir)
        self._bytes_written = sum(
            path.stat().st_size for path in self.final_dir.rglob("*") if path.is_file()
        )
        self._finalized = True
        return self.final_dir

    def discard_rejected_start(self, reason: str) -> Path:
        self._close_videos()
        self._close_audit_handles()
        if self.partial_dir.exists():
            shutil.rmtree(self.partial_dir)
        self.root.mkdir(parents=True, exist_ok=True)
        report = self.root / f"rejected-start-{self.episode_uuid}.json"
        self._write_json(
            report,
            {
                "format_version": RAW_EPISODE_FORMAT_VERSION,
                "episode_uuid": self.episode_uuid,
                "recording_status": "invalid",
                "termination_reason": reason,
                "episode_created": False,
            },
        )
        self._started = False
        return report

    def close(self) -> None:
        if self._started and not self._finalized:
            raise RuntimeError("active raw episode must be finalized or discarded")

    def diagnostics(self) -> dict[str, Any]:
        return {
            "format_version": RAW_EPISODE_FORMAT_VERSION,
            "sample_count": len(self._rows),
            "video_frame_counts": dict(self._video_frame_counts),
            "quality_row_count": len(self._quality_events),
            "quest_recorded": False,
        }

    def _get_camera(self, camera: CameraRecord) -> CameraSample:
        key = _camera_key(camera)
        cached = self._camera_cache.pop(key, None)
        if cached is not None:
            return cached
        return _materialize_camera(camera)

    def _write_video(self, name: str, rgb: np.ndarray) -> None:
        cv2, _ = _load_video_dependencies()
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f"{name} requires HWC uint8 RGB")
        height, width = int(rgb.shape[0]), int(rgb.shape[1])
        if self._video_shape is None:
            self._video_shape = (width, height)
        elif self._video_shape != (width, height):
            raise ValueError("workspace and wrist RGB frames must have one shared video size")
        writer = self._videos.get(name)
        if writer is None:
            path = self.partial_dir / f"{name}.mp4"
            writer = cv2.VideoWriter(
                str(path),
                cv2.VideoWriter_fourcc(*self.video_codec),
                float(self._dataset_fps),
                (width, height),
            )
            if not writer.isOpened():
                raise RuntimeError(f"cannot open video writer for {path}")
            self._videos[name] = writer
        writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        self._video_frame_counts[name] += 1

    def _close_videos(self) -> None:
        for writer in self._videos.values():
            writer.release()
        self._videos.clear()

    def _append_audit(self, stream: str, record: Mapping[str, Any]) -> None:
        handle = self._audit_handles.get(stream)
        if handle is None:
            path = self.partial_dir / "audit" / f"{stream}.jsonl"
            handle = path.open("a", encoding="utf-8")
            self._audit_handles[stream] = handle
        handle.write(
            json.dumps(record, allow_nan=False, separators=(",", ":"), sort_keys=True)
            + "\n"
        )

    def _close_audit_handles(self) -> None:
        handles, self._audit_handles = self._audit_handles, {}
        for handle in handles.values():
            handle.flush()
            handle.close()

    def _write_frames_parquet(self) -> None:
        pa, parquet = _load_parquet_dependencies()
        path = self.partial_dir / "frames.parquet"
        table = pa.table(
            {
                "frame_index": pa.array(
                    [row["frame_index"] for row in self._rows], type=pa.int64()
                ),
                "timestamp_ns": pa.array(
                    [row["timestamp_ns"] for row in self._rows], type=pa.int64()
                ),
                "observation.state": pa.array(
                    [row["observation.state"] for row in self._rows],
                    type=pa.list_(pa.float32(), 12),
                ),
                "action": pa.array(
                    [row["action"] for row in self._rows],
                    type=pa.list_(pa.float32(), 12),
                ),
            }
        )
        parquet.write_table(table, path, compression="zstd")

    def _write_quality_parquet_if_needed(self) -> None:
        abnormal = any(
            row["action_status"] != "accepted" or not row["timing_valid"]
            for row in self._frame_quality_rows
        )
        if not abnormal:
            return
        pa, parquet = _load_parquet_dependencies()
        path = self.partial_dir / "quality.parquet"
        columns = {
            name: [row[name] for row in self._frame_quality_rows]
            for name in (
                "frame_index",
                "action_status",
                "timing_valid",
                "arm_trigger",
                "hand_grip",
                "nominal_slot_index",
                "missed_slots_before",
                "missed_slots_after",
            )
        }
        table = pa.table(
            {
                "frame_index": pa.array(columns["frame_index"], type=pa.int64()),
                "action_status": pa.array(columns["action_status"], type=pa.string()),
                "timing_valid": pa.array(columns["timing_valid"], type=pa.bool_()),
                "arm_trigger": pa.array(columns["arm_trigger"], type=pa.bool_()),
                "hand_grip": pa.array(columns["hand_grip"], type=pa.bool_()),
                "nominal_slot_index": pa.array(columns["nominal_slot_index"], type=pa.int64()),
                "missed_slots_before": pa.array(columns["missed_slots_before"], type=pa.int64()),
                "missed_slots_after": pa.array(columns["missed_slots_after"], type=pa.int64()),
            }
        )
        parquet.write_table(table, path, compression="zstd")
        metadata = json.loads(
            (self.partial_dir / "episode.json").read_text(encoding="utf-8")
        )
        metadata.setdefault("files", {})["quality"] = "quality.parquet"
        self._write_episode_metadata(metadata)

    def _write_episode_metadata(self, payload: Mapping[str, Any]) -> None:
        self._write_json(self.partial_dir / "episode.json", payload)

    @staticmethod
    def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False, sort_keys=True)
            + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)

    @staticmethod
    def _fsync_file(path: Path) -> None:
        with path.open("rb") as handle:
            os.fsync(handle.fileno())
