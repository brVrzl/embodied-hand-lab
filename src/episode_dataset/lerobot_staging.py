"""Review-first, LeRobot-shaped episode staging.

The physical recorder writes this format during collection.  It deliberately
does not import PyArrow or create Parquet.  Low-dimensional rows are appended
to the episode-named JSONL file and are converted to the selected final
dataset format only after offline review.
"""

from __future__ import annotations

from collections import OrderedDict
import html
import json
import os
from pathlib import Path
import shutil
import time
from typing import Any, Callable, Mapping

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
from .raw_episode import _camera_key, _load_video_dependencies, _materialize_camera


LEROBOT_STAGING_FORMAT_VERSION = "lerobot_episode_staging_v1"


def _staging_episode_name(episode: str | int) -> str:
    if isinstance(episode, int):
        return f"episode_{episode:06d}"
    value = str(episode)
    if value.startswith("episode_"):
        return value
    return f"episode_{int(value):06d}"


def _staging_episode_metadata_path(root: Path, episode: str | int) -> Path:
    return root / "meta" / "episodes" / "chunk-000" / f"{_staging_episode_name(episode)}.json"


def _write_staging_metadata(path: Path, metadata: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(dict(metadata), indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def stage_review_html(
    root: str | Path,
    episode: str | int,
    *,
    output: str | Path | None = None,
) -> Path:
    """Create an offline, human-viewable review page without touching capture."""

    dataset_root = Path(root).resolve()
    name = _staging_episode_name(episode)
    metadata_path = _staging_episode_metadata_path(dataset_root, name)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("completion_status") != EpisodeStatus.COMPLETED.value:
        raise ValueError("only completed staging episodes can be reviewed")
    review_path = (
        Path(output).resolve()
        if output is not None
        else dataset_root / "review" / name / "index.html"
    )
    review_path.parent.mkdir(parents=True, exist_ok=True)
    video_base = Path("../../videos")
    workspace = (
        video_base
        / "observation.images.workspace"
        / "chunk-000"
        / f"{name}.mp4"
    )
    wrist = (
        video_base
        / "observation.images.wrist"
        / "chunk-000"
        / f"{name}.mp4"
    )
    title = html.escape(f"Review {name}")
    summary = html.escape(
        json.dumps(
            {
                key: metadata.get(key)
                for key in (
                    "episode_name",
                    "num_frames",
                    "duration_s",
                    "termination_reason",
                    "recording_status",
                    "review_status",
                )
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    body = f"""<!doctype html>
<meta charset="utf-8">
<title>{title}</title>
<h1>{title}</h1>
<p>Inspect both synchronized recordings, then run the offline approve command.</p>
<video controls preload="metadata" width="640" src="{html.escape(str(workspace))}"></video>
<video controls preload="metadata" width="640" src="{html.escape(str(wrist))}"></video>
<pre>{summary}</pre>
"""
    review_path.write_text(body, encoding="utf-8")
    return review_path


def set_staging_review(
    root: str | Path,
    episode: str | int,
    *,
    status: str,
    notes: str = "",
) -> Path:
    """Record the human review decision in episode metadata and its index."""

    if status not in {"approved", "rejected"}:
        raise ValueError("staging review status must be approved or rejected")
    dataset_root = Path(root).resolve()
    name = _staging_episode_name(episode)
    metadata_path = _staging_episode_metadata_path(dataset_root, name)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("completion_status") != EpisodeStatus.COMPLETED.value:
        raise ValueError("only completed staging episodes can be approved")
    metadata["review_status"] = status
    metadata["review_notes"] = notes
    metadata["reviewed_at_unix_ns"] = time.time_ns()
    _write_staging_metadata(metadata_path, metadata)
    index_path = dataset_root / "meta" / "episodes.jsonl"
    if index_path.exists():
        rows = []
        for line in index_path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            row = json.loads(line)
            if row.get("episode_name") == name:
                row["review_status"] = status
                row["review_notes"] = notes
            rows.append(row)
        temporary = index_path.with_suffix(index_path.suffix + ".tmp")
        temporary.write_text(
            "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
            encoding="utf-8",
        )
        os.replace(temporary, index_path)
    return metadata_path


def materialize_staging_episode(
    root: str | Path,
    episode: str | int,
    output_root: str | Path,
) -> Path:
    """Convert an approved staging episode to a LeRobot-shaped Parquet shard."""

    try:
        import pyarrow as pa
        import pyarrow.parquet as parquet
    except ImportError as exc:
        raise RuntimeError(
            "staging conversion requires the dataset-collection dependencies"
        ) from exc
    dataset_root = Path(root).resolve()
    output = Path(output_root).resolve()
    name = _staging_episode_name(episode)
    metadata_path = _staging_episode_metadata_path(dataset_root, name)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("completion_status") != EpisodeStatus.COMPLETED.value:
        raise ValueError("only completed staging episodes can be materialized")
    if metadata.get("review_status") != "approved":
        raise ValueError("episode must be human-approved before Parquet conversion")
    source_rows = dataset_root / "data" / "chunk-000" / f"{name}.jsonl"
    rows = [
        json.loads(line)
        for line in source_rows.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not rows:
        raise ValueError("cannot materialize an empty staging episode")
    data_path = output / "data" / "chunk-000" / f"{name}.parquet"
    if data_path.exists():
        raise FileExistsError(data_path)
    for role in ("workspace", "wrist"):
        source_video = (
            dataset_root
            / "videos"
            / f"observation.images.{role}"
            / "chunk-000"
            / f"{name}.mp4"
        )
        if not source_video.is_file():
            raise FileNotFoundError(source_video)
    data_path.parent.mkdir(parents=True, exist_ok=True)
    table = pa.table(
        {
            "frame_index": pa.array([int(row["frame_index"]) for row in rows], type=pa.int64()),
            "timestamp": pa.array([float(row["timestamp"]) for row in rows], type=pa.float64()),
            "timestamp_ns": pa.array([int(row["timestamp_ns"]) for row in rows], type=pa.int64()),
            "episode_index": pa.array([int(metadata["episode_index"])] * len(rows), type=pa.int64()),
            "observation.state": pa.array(
                [row["observation.state"] for row in rows],
                type=pa.list_(pa.float32(), 12),
            ),
            "action": pa.array(
                [row["action"] for row in rows],
                type=pa.list_(pa.float32(), 12),
            ),
            "arm_trigger": pa.array([bool(row["arm_trigger"]) for row in rows], type=pa.bool_()),
            "hand_grip": pa.array([bool(row["hand_grip"]) for row in rows], type=pa.bool_()),
            "action_status": pa.array([str(row["action_status"]) for row in rows], type=pa.string()),
        }
    )
    parquet.write_table(table, data_path, compression="zstd")
    for role in ("workspace", "wrist"):
        destination = (
            output
            / "videos"
            / f"observation.images.{role}"
            / "chunk-000"
            / f"{name}.mp4"
        )
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(
            dataset_root
            / "videos"
            / f"observation.images.{role}"
            / "chunk-000"
            / f"{name}.mp4",
            destination,
        )
    output_meta = output / "meta"
    output_meta.mkdir(parents=True, exist_ok=True)
    info = dict(metadata)
    info.update(
        {
            "format_version": "lerobot_parquet_shard_v1",
            "parquet_materialized": True,
            "source_staging_root": str(dataset_root),
            "files": {
                "data": f"data/chunk-000/{name}.parquet",
                "workspace_video": f"videos/observation.images.workspace/chunk-000/{name}.mp4",
                "wrist_video": f"videos/observation.images.wrist/chunk-000/{name}.mp4",
            },
        }
    )
    _write_staging_metadata(output_meta / "episodes" / "chunk-000" / f"{name}.json", info)
    source_info = dataset_root / "meta" / "info.json"
    output_info = {}
    if source_info.exists():
        output_info = json.loads(source_info.read_text(encoding="utf-8"))
    output_info.update(
        {
            "format": "lerobot_parquet_shard_v1",
            "parquet_materialized": True,
            "total_episodes": 1,
            "source_staging_root": str(dataset_root),
        }
    )
    _write_staging_metadata(output_meta / "info.json", output_info)
    source_tasks = dataset_root / "meta" / "tasks.jsonl"
    if source_tasks.exists():
        shutil.copy2(source_tasks, output_meta / "tasks.jsonl")
    episode_index = output_meta / "episodes.jsonl"
    episode_index.parent.mkdir(parents=True, exist_ok=True)
    with episode_index.open("a", encoding="utf-8") as handle:
        handle.write(
            json.dumps(
                {
                    "episode_index": metadata["episode_index"],
                    "episode_name": name,
                    "length": len(rows),
                    "task": metadata.get("task"),
                    "review_status": "approved",
                    "data": f"data/chunk-000/{name}.parquet",
                },
                sort_keys=True,
            )
            + "\n"
        )
    return data_path


class LeRobotStagingWriter:
    """Write one reviewable episode using LeRobot-style names and paths."""

    def __init__(
        self,
        root: str | Path,
        *,
        episode_index: int,
        task_name: str,
        operator: str,
        dataset_fps: int = 30,
        metadata: Mapping[str, Any] | None = None,
        video_codec: str = "mp4v",
    ) -> None:
        self.dataset_root = Path(root).resolve()
        self.episode_index = int(episode_index)
        if self.episode_index < 0:
            raise ValueError("episode_index must be non-negative")
        self.episode_name = f"episode_{self.episode_index:06d}"
        self.root = self.dataset_root
        self.temporary_id = self.episode_name
        self.task_name = str(task_name)
        self.operator = str(operator)
        self._dataset_fps = int(dataset_fps)
        if self._dataset_fps <= 0:
            raise ValueError("dataset_fps must be positive")
        self.video_codec = str(video_codec)
        if len(self.video_codec) != 4:
            raise ValueError("video_codec must be a four-character OpenCV codec")
        self._metadata_extra = dict(metadata or {})
        self._started = False
        self._finalized = False
        self._start_ns: int | None = None
        self._last_timestamp_ns: int | None = None
        self._trigger_release_ns: int | None = None
        self._sample_count = 0
        self._quality_count = 0
        self._camera_cache: OrderedDict[tuple[Any, ...], CameraSample] = OrderedDict()
        self._audit_handles: dict[str, Any] = {}
        self._sample_handle: Any | None = None
        self._quality_handle: Any | None = None
        self._videos: dict[str, Any] = {}
        self._video_frame_counts = {"workspace": 0, "wrist": 0}
        self._video_shape: tuple[int, int] | None = None
        self._bytes_written = 0
        self._final_metadata_provider: Callable[[], Mapping[str, Any]] | None = None

        chunk = self.dataset_root / "meta" / "episodes" / "chunk-000"
        data_chunk = self.dataset_root / "data" / "chunk-000"
        self._meta_dir = self.dataset_root / "meta"
        self._episode_meta_dir = chunk
        self._data_dir = data_chunk
        self._audit_dir = self.dataset_root / "audit" / "chunk-000" / self.episode_name
        self._sample_path = data_chunk / f"{self.episode_name}.jsonl"
        self._sample_partial_path = data_chunk / f"{self.episode_name}.jsonl.partial"
        self._quality_path = data_chunk / f"{self.episode_name}.quality.jsonl"
        self._quality_partial_path = data_chunk / f"{self.episode_name}.quality.jsonl.partial"
        self._episode_meta_path = chunk / f"{self.episode_name}.json"
        self._episode_meta_partial_path = chunk / f"{self.episode_name}.json.partial"
        self._video_paths = {
            role: self.dataset_root
            / "videos"
            / f"observation.images.{role}"
            / "chunk-000"
            / f"{self.episode_name}.mp4"
            for role in ("workspace", "wrist")
        }
        self._video_partial_paths = {
            # OpenCV selects its container from the final suffix.  Keep the
            # temporary file hidden but retain `.mp4` as the actual suffix;
            # `.mp4.partial` makes VideoWriter reject an otherwise valid
            # codec before the first frame.
            role: path.with_name(f".{path.stem}.partial{path.suffix}")
            for role, path in self._video_paths.items()
        }

    @property
    def sample_count(self) -> int:
        return self._sample_count

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
        self._prepare_paths()
        self._sample_handle = self._sample_partial_path.open("w", encoding="utf-8")
        self._start_ns = int(prerequisites.accepted.host_monotonic_ns)
        self._started = True
        self._write_batch_metadata()
        self._write_episode_metadata(
            {
                "format_version": LEROBOT_STAGING_FORMAT_VERSION,
                "episode_index": self.episode_index,
                "episode_name": self.episode_name,
                "recording_status": "recording",
                "task": self.task_name,
                "operator": self.operator,
                "fps": self._dataset_fps,
                "num_frames": 0,
                "timestamp_semantics": {
                    "clock": "host_monotonic_ns",
                    "video_alignment": "video frame i aligns with JSONL frame_index",
                },
                "features": {
                    "observation.state": {
                        "dtype": "float32",
                        "shape": [12],
                        "names": [
                            "arm_q1_actual", "arm_q2_actual", "arm_q3_actual",
                            "arm_q4_actual", "arm_q5_actual", "arm_q6_actual",
                            "hand_h1_actual", "hand_h2_actual", "hand_h3_actual",
                            "hand_h4_actual", "hand_h5_actual", "hand_h6_actual",
                        ],
                    },
                    "action": {
                        "dtype": "float32",
                        "shape": [12],
                        "names": [
                            "arm_q1_target", "arm_q2_target", "arm_q3_target",
                            "arm_q4_target", "arm_q5_target", "arm_q6_target",
                            "hand_h1_target", "hand_h2_target", "hand_h3_target",
                            "hand_h4_target", "hand_h5_target", "hand_h6_target",
                        ],
                    },
                },
                "units": {
                    "observation.state.arm_q": "rad",
                    "observation.state.hand": "normalized_0_1",
                    "action.arm_q": "rad",
                    "action.hand": "normalized_0_1",
                },
                "files": {
                    "data": f"data/chunk-000/{self.episode_name}.jsonl",
                    "workspace_video": (
                        f"videos/observation.images.workspace/chunk-000/{self.episode_name}.mp4"
                    ),
                    "wrist_video": (
                        f"videos/observation.images.wrist/chunk-000/{self.episode_name}.mp4"
                    ),
                },
                "video": {"codec": self.video_codec, "fps": self._dataset_fps},
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
        if stream.startswith("quest"):
            return True
        if stream == "data_quality":
            self._quality_count += 1
            self._append_quality(record)
        elif stream in {"jaka_state", "rh56_feedback"}:
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
        key = _camera_key(camera)
        self._camera_cache[key] = materialized
        self._camera_cache.move_to_end(key)
        while len(self._camera_cache) > 64:
            self._camera_cache.popitem(last=False)
        return True

    def append_sample(self, sample: CanonicalSample) -> bool:
        if not self._started or self._finalized:
            raise RuntimeError("no active episode")
        control = sample.control
        if (
            control.arm_q_measured is None
            or control.hand_observation is None
            or control.accepted_arm_q is None
            or control.hand_target is None
        ):
            raise ValueError("staging episode requires measured state and final action")
        if self._last_timestamp_ns is not None and sample.timestamp_ns <= self._last_timestamp_ns:
            raise ValueError("staging timestamp must increase strictly")
        workspace = self._get_camera(sample.workspace)
        wrist = self._get_camera(sample.wrist)
        self._write_video("workspace", workspace.rgb)
        self._write_video("wrist", wrist.rgb)
        start_ns = self._start_ns or sample.timestamp_ns
        row = {
            "frame_index": self._sample_count,
            "timestamp": (int(sample.timestamp_ns) - start_ns) / 1e9,
            "timestamp_ns": int(sample.timestamp_ns),
            "observation.state": [*control.arm_q_measured, *control.hand_observation],
            "action": [*control.accepted_arm_q, *control.hand_target],
            "arm_trigger": bool(control.arm_trigger),
            "hand_grip": bool(control.hand_grip),
            "action_status": control.arm_action_status,
            "camera_frame_index": {
                "workspace": int(workspace.rgb_frame_number),
                "wrist": int(wrist.rgb_frame_number),
            },
            "camera_timestamp_ns": {
                "workspace": int(workspace.host_monotonic_ns),
                "wrist": int(wrist.host_monotonic_ns),
            },
        }
        assert self._sample_handle is not None
        self._sample_handle.write(
            json.dumps(row, allow_nan=False, separators=(",", ":"), sort_keys=True)
            + "\n"
        )
        self._sample_count += 1
        self._last_timestamp_ns = int(sample.timestamp_ns)
        return True

    def flush_pending(self) -> None:
        if self._sample_handle is not None:
            self._sample_handle.flush()
        if self._quality_handle is not None:
            self._quality_handle.flush()
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
        if status is EpisodeStatus.COMPLETED and self._sample_count == 0:
            raise ValueError("empty episode cannot be completed")
        self._trigger_release_ns = trigger_release_monotonic_ns
        self._close_videos()
        self._close_handles()
        if self._sample_partial_path.exists():
            os.replace(self._sample_partial_path, self._sample_path)
        if self._quality_partial_path.exists():
            os.replace(self._quality_partial_path, self._quality_path)
        for role in ("workspace", "wrist"):
            partial = self._video_partial_paths[role]
            if partial.exists():
                os.replace(partial, self._video_paths[role])
        end_ns = self._last_timestamp_ns or time.monotonic_ns()
        metadata = json.loads(self._episode_meta_partial_path.read_text(encoding="utf-8"))
        metadata.update(
            {
                "recording_status": "complete" if status is EpisodeStatus.COMPLETED else status.value,
                "completion_status": status.value,
                "termination_reason": termination_reason,
                "num_frames": self._sample_count,
                "duration_s": None if self._start_ns is None else (end_ns - self._start_ns) / 1e9,
                "end_timestamp_ns": int(end_ns),
                "trigger_release_timestamp_ns": trigger_release_monotonic_ns,
                "quality_count": self._quality_count,
                "review_status": "pending",
            }
        )
        if self._final_metadata_provider is not None:
            metadata.update(dict(self._final_metadata_provider()))
        if report:
            metadata["recorder_report"] = dict(report)
        self._write_json(self._episode_meta_partial_path, metadata)
        os.replace(self._episode_meta_partial_path, self._episode_meta_path)
        self._append_episode_index(metadata)
        self._finalized = True
        self._bytes_written = sum(
            path.stat().st_size
            for path in (
                self._sample_path,
                self._quality_path,
                self._episode_meta_path,
                self._video_paths["workspace"],
                self._video_paths["wrist"],
            )
            if path.exists()
        )
        return self._episode_meta_path

    def discard_rejected_start(self, reason: str) -> Path:
        self._close_videos()
        self._close_handles()
        for path in (
            self._sample_partial_path,
            self._quality_partial_path,
            self._episode_meta_partial_path,
            self._video_partial_paths["workspace"],
            self._video_partial_paths["wrist"],
        ):
            path.unlink(missing_ok=True)
        rejected = self._meta_dir / "rejected" / f"{self.episode_name}.json"
        self._write_json(
            rejected,
            {
                "format_version": LEROBOT_STAGING_FORMAT_VERSION,
                "episode_name": self.episode_name,
                "recording_status": "invalid",
                "termination_reason": reason,
                "episode_created": False,
            },
        )
        self._started = False
        return rejected

    def close(self) -> None:
        if self._started and not self._finalized:
            raise RuntimeError("active staging episode must be finalized or discarded")

    def diagnostics(self) -> dict[str, Any]:
        return {
            "format_version": LEROBOT_STAGING_FORMAT_VERSION,
            "episode_name": self.episode_name,
            "sample_count": self._sample_count,
            "video_frame_counts": dict(self._video_frame_counts),
            "quality_row_count": self._quality_count,
            "quest_recorded": False,
            "parquet_materialized": False,
            # Keep the recorder-process report contract shared with the
            # canonical writers.  Staging intentionally does no materializing
            # or Parquet work in the capture process.
            "frame_materialization_duration_ns": 0,
            "canonical_metadata_duration_ns": 0,
        }

    def _prepare_paths(self) -> None:
        for directory in (
            self._meta_dir,
            self._episode_meta_dir,
            self._data_dir,
            self._audit_dir,
            self._sample_path.parent,
            self._video_paths["workspace"].parent,
            self._video_paths["wrist"].parent,
            self._meta_dir / "rejected",
        ):
            directory.mkdir(parents=True, exist_ok=True)
        for path in (
            self._sample_partial_path,
            self._quality_partial_path,
            self._episode_meta_partial_path,
            self._video_partial_paths["workspace"],
            self._video_partial_paths["wrist"],
        ):
            if path.exists():
                raise FileExistsError(path)

    def _write_batch_metadata(self) -> None:
        info = self._meta_dir / "info.json"
        if not info.exists():
            self._write_json(
                info,
                {
                    "format": LEROBOT_STAGING_FORMAT_VERSION,
                    "fps": self._dataset_fps,
                    "features": ["timestamp", "observation.state", "action"],
                    "video_features": [
                        "observation.images.workspace",
                        "observation.images.wrist",
                    ],
                    "parquet_materialized": False,
                    "code": _git_state(Path(__file__).resolve().parents[2]),
                },
            )
        tasks = self._meta_dir / "tasks.jsonl"
        task_row = {"task_index": 0, "task": self.task_name}
        existing = tasks.read_text(encoding="utf-8") if tasks.exists() else ""
        if json.dumps(task_row, sort_keys=True) not in existing:
            with tasks.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(task_row, sort_keys=True) + "\n")

    def _append_episode_index(self, metadata: Mapping[str, Any]) -> None:
        index = self._meta_dir / "episodes.jsonl"
        row = {
            "episode_index": self.episode_index,
            "episode_name": self.episode_name,
            "length": self._sample_count,
            "task": self.task_name,
            "status": metadata.get("completion_status"),
            "data": f"data/chunk-000/{self.episode_name}.jsonl",
            "videos": {
                role: (
                    f"videos/observation.images.{role}/chunk-000/"
                    f"{self.episode_name}.mp4"
                )
                for role in ("workspace", "wrist")
            },
        }
        with index.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(row, allow_nan=False, sort_keys=True) + "\n")

    def _write_episode_metadata(self, payload: Mapping[str, Any]) -> None:
        self._write_json(self._episode_meta_partial_path, payload)

    @staticmethod
    def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

    def _get_camera(self, camera: CameraRecord) -> CameraSample:
        key = _camera_key(camera)
        cached = self._camera_cache.pop(key, None)
        if cached is not None:
            return cached
        return _materialize_camera(camera)

    def _write_video(self, role: str, rgb: np.ndarray) -> None:
        cv2, _ = _load_video_dependencies()
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f"{role} requires HWC uint8 RGB")
        height, width = int(rgb.shape[0]), int(rgb.shape[1])
        if self._video_shape is None:
            self._video_shape = (width, height)
        elif self._video_shape != (width, height):
            raise ValueError("workspace and wrist RGB frames must share one video size")
        writer = self._videos.get(role)
        if writer is None:
            writer = cv2.VideoWriter(
                str(self._video_partial_paths[role]),
                cv2.VideoWriter_fourcc(*self.video_codec),
                float(self._dataset_fps),
                (width, height),
            )
            if not writer.isOpened():
                raise RuntimeError(
                    f"cannot open staging video writer for {self._video_partial_paths[role]}"
                )
            self._videos[role] = writer
        writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        self._video_frame_counts[role] += 1

    def _close_videos(self) -> None:
        for writer in self._videos.values():
            writer.release()
        self._videos.clear()

    def _append_quality(self, record: Mapping[str, Any]) -> None:
        if self._quality_handle is None:
            self._quality_handle = self._quality_partial_path.open("a", encoding="utf-8")
        self._quality_handle.write(
            json.dumps(record, allow_nan=False, separators=(",", ":"), sort_keys=True)
            + "\n"
        )

    def _append_audit(self, stream: str, record: Mapping[str, Any]) -> None:
        handle = self._audit_handles.get(stream)
        if handle is None:
            path = self._audit_dir / f"{stream}.jsonl"
            handle = path.open("a", encoding="utf-8")
            self._audit_handles[stream] = handle
        handle.write(
            json.dumps(record, allow_nan=False, separators=(",", ":"), sort_keys=True)
            + "\n"
        )

    def _close_handles(self) -> None:
        handles = list(self._audit_handles.values())
        self._audit_handles.clear()
        if self._sample_handle is not None:
            handles.append(self._sample_handle)
            self._sample_handle = None
        if self._quality_handle is not None:
            handles.append(self._quality_handle)
            self._quality_handle = None
        for handle in handles:
            handle.flush()
            handle.close()
