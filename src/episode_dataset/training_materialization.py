"""Deterministic offline materialization for physical LeRobot staging data.

This module is deliberately separate from the live recorder.  It reads the
immutable staging tree, applies an explicit episode manifest and causal row
checks, and writes one master Parquet/video tree plus lightweight policy views.
No hardware, synchronization worker, or raw file is opened for writing.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
import yaml

from rh56_driver.hand_schema import CANONICAL_HAND_ORDER


TRAINING_SCHEMA_VERSION = "embodied_lab.physical_training.v1"
TRAINING_CONFIG_VERSION = "embodied_lab.physical_training_config.v1"
TASK_PROMPT = "Pick up the bottle and place it on the cardboard box."

STATE_ORDER = (
    "jaka_joint_1",
    "jaka_joint_2",
    "jaka_joint_3",
    "jaka_joint_4",
    "jaka_joint_5",
    "jaka_joint_6",
    "rh56_index",
    "rh56_middle",
    "rh56_ring",
    "rh56_pinky",
    "rh56_thumb_close",
    "rh56_thumb_lateral",
)
ACTION_ORDER = (
    "jaka_target_1",
    "jaka_target_2",
    "jaka_target_3",
    "jaka_target_4",
    "jaka_target_5",
    "jaka_target_6",
    "rh56_target_index",
    "rh56_target_middle",
    "rh56_target_ring",
    "rh56_target_pinky",
    "rh56_target_thumb_close",
    "rh56_target_thumb_lateral",
)
FORCE_ORDER = (
    "rh56_force_index",
    "rh56_force_middle",
    "rh56_force_ring",
    "rh56_force_pinky",
    "rh56_force_thumb_close",
    "rh56_force_thumb_lateral",
)


@dataclass(frozen=True, slots=True)
class MaterializationResult:
    output_root: Path
    report_path: Path | None
    summary: Mapping[str, Any]
    reused: bool = False


def _episode_name(value: int | str) -> str:
    if isinstance(value, int):
        return f"episode_{value:06d}"
    text = str(value)
    if text.startswith("episode_"):
        return text
    return f"episode_{int(text):06d}"


def _episode_id(value: int | str) -> int:
    text = str(value)
    return int(text.removeprefix("episode_"))


def _json_bytes(value: Any) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _write_jsonl(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(
                json.dumps(row, sort_keys=True, ensure_ascii=False, allow_nan=False)
                + "\n"
            )
    temporary.replace(path)


def _finite_vector(value: Any, width: int) -> bool:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return False
    return array.shape == (width,) and bool(np.isfinite(array).all())


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _read_rows(path: Path) -> tuple[list[dict[str, Any]], list[str]]:
    rows: list[dict[str, Any]] = []
    errors: list[str] = []
    try:
        with path.open("r", encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, 1):
                if not line.strip():
                    continue
                try:
                    value = json.loads(line)
                except json.JSONDecodeError as exc:
                    errors.append(f"line {line_number}: malformed JSON: {exc.msg}")
                    continue
                if not isinstance(value, dict):
                    errors.append(f"line {line_number}: row is not an object")
                    continue
                rows.append(value)
    except OSError as exc:
        errors.append(f"cannot read {path}: {exc}")
    return rows, errors


def _resolve_config_path(config_path: str | Path, value: str | Path) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path.resolve()
    return (Path(config_path).resolve().parent / path).resolve()


def load_training_config(config_path: str | Path) -> tuple[Path, dict[str, Any]]:
    path = Path(config_path).resolve()
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("training config must be a mapping")
    if payload.get("schema_version") != TRAINING_CONFIG_VERSION:
        raise ValueError("unsupported physical training config schema")
    for key in ("source_root", "output_root", "episodes", "task"):
        if key not in payload:
            raise ValueError(f"training config is missing {key!r}")
    if not isinstance(payload["episodes"], list):
        raise ValueError("training config episodes must be a list")
    task = payload["task"]
    if not isinstance(task, dict) or not task.get("id") or not task.get("prompt"):
        raise ValueError("training config task requires id and prompt")
    return path, payload


def _episode_paths(source_root: Path, episode_id: int) -> dict[str, Path]:
    name = _episode_name(episode_id)
    return {
        "metadata": source_root / "meta" / "episodes" / "chunk-000" / f"{name}.json",
        "rows": source_root / "data" / "chunk-000" / f"{name}.jsonl",
        "workspace": source_root / "videos" / "observation.images.workspace" / "chunk-000" / f"{name}.mp4",
        "wrist": source_root / "videos" / "observation.images.wrist" / "chunk-000" / f"{name}.mp4",
    }


def _source_inventory(source_root: Path, ids: Sequence[int]) -> dict[str, Any]:
    inventory: dict[str, Any] = {}
    for episode_id in ids:
        paths = _episode_paths(source_root, episode_id)
        episode_inventory: dict[str, Any] = {}
        for role, path in paths.items():
            if not path.is_file():
                episode_inventory[role] = {"path": str(path), "missing": True}
                continue
            episode_inventory[role] = {
                "path": str(path),
                "size": path.stat().st_size,
                "sha256": _sha256_file(path),
            }
        inventory[str(episode_id)] = episode_inventory
    return inventory


def _config_fingerprint(config: Mapping[str, Any], inventory: Mapping[str, Any]) -> str:
    return hashlib.sha256(_json_bytes({"config": config, "sources": inventory})).hexdigest()


def _source_timing(row: Mapping[str, Any]) -> tuple[int, dict[str, Any], dict[str, Any], dict[str, Any]]:
    timing = row.get("timing")
    if not isinstance(timing, Mapping):
        return 0, {}, {}, {}
    try:
        canonical = int(row["timestamp_ns"])
    except (KeyError, TypeError, ValueError):
        canonical = 0
    timestamps = timing.get("source_timestamps_ns")
    domains = timing.get("source_timestamp_domains")
    validity = timing.get("source_validity")
    return (
        canonical,
        dict(timestamps) if isinstance(timestamps, Mapping) else {},
        dict(domains) if isinstance(domains, Mapping) else {},
        dict(validity) if isinstance(validity, Mapping) else {},
    )


def _row_reasons(row: Mapping[str, Any], *, require_force: bool) -> list[str]:
    reasons: list[str] = []
    canonical, timestamps, domains, validity = _source_timing(row)
    timing_data = row.get("timing")
    timing_data = timing_data if isinstance(timing_data, Mapping) else {}
    camera_data = row.get("camera")
    camera_data = camera_data if isinstance(camera_data, Mapping) else {}
    if canonical <= 0:
        reasons.append("missing_canonical_timestamp")
    if timing_data.get("canonical_host_monotonic_ns") != canonical:
        reasons.append("canonical_timestamp_mismatch")
    if not _finite_vector(row.get("observation.state"), 12):
        reasons.append("state_shape_or_finite")
    if not _finite_vector(row.get("action"), 12):
        reasons.append("action_shape_or_finite")
    force_ok = _finite_vector(row.get("observation.force"), 6)
    if not force_ok:
        reasons.append("force_shape_or_finite")
    if timing_data.get("synchronization_valid") is not True:
        reasons.append("synchronization_invalid")
    for name, timestamp_value in timestamps.items():
        if timestamp_value is None:
            continue
        try:
            timestamp = int(timestamp_value)
        except (TypeError, ValueError):
            reasons.append(f"invalid_source_timestamp:{name}")
            continue
        if domains.get(name) != "host_monotonic_ns":
            reasons.append(f"source_domain_invalid:{name}")
        if timestamp > canonical:
            reasons.append(f"future_source:{name}")
    required_sources = ("jaka_observation", "rh56_angle_act", "workspace", "wrist")
    for name in required_sources:
        if validity.get(name) is not True:
            reasons.append(f"invalid_source:{name}")
    for role in ("workspace", "wrist"):
        camera = camera_data.get(role, {})
        camera = camera if isinstance(camera, Mapping) else {}
        if not isinstance(camera, Mapping) or camera.get("valid") is not True:
            reasons.append(f"invalid_camera:{role}")
        if camera.get("rgb_timestamp_domain") not in {"global_time", "hardware_clock", "system_time"}:
            reasons.append(f"invalid_camera_timestamp_domain:{role}")
        if not isinstance(camera.get("rgb_frame_number"), int) or camera.get("rgb_frame_number", -1) < 0:
            reasons.append(f"invalid_camera_frame:{role}")
    if require_force and validity.get("rh56_force_act") is not True:
        reasons.append("invalid_source:rh56_force_act")
    return sorted(set(reasons))


def _crop_rows(
    rows: Sequence[Mapping[str, Any]],
    metadata: Mapping[str, Any],
    entry: Mapping[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, Any], list[dict[str, Any]]]:
    episode_id = int(metadata["episode_index"])
    timestamps = [int(row.get("timestamp_ns", 0)) for row in rows]
    manual = entry.get("crop")
    manual = manual if isinstance(manual, Mapping) else {}
    start_value = manual.get("start_timestamp_ns")
    end_value = manual.get("end_timestamp_ns")
    start_source = "first_canonical_row"
    end_source = "trigger_release_timestamp_ns"
    try:
        start_ns = int(start_value) if start_value is not None else timestamps[0]
    except (IndexError, TypeError, ValueError):
        start_ns = 0
        start_source = "crop_review_required"
    if start_value is not None:
        start_source = "manual_override"
    release_value = metadata.get("trigger_release_timestamp_ns")
    try:
        end_ns = int(end_value) if end_value is not None else int(release_value)
    except (TypeError, ValueError):
        end_ns = 0
        end_source = "crop_review_required"
    if end_value is not None:
        end_source = "manual_override"
    report = {
        "episode_index": episode_id,
        "raw_frame_count": len(rows),
        "raw_start_timestamp_ns": timestamps[0] if timestamps else None,
        "raw_end_timestamp_ns": timestamps[-1] if timestamps else None,
        "training_start_timestamp_ns": start_ns or None,
        "training_end_timestamp_ns": end_ns or None,
        "training_start_source": start_source,
        "training_end_source": end_source,
        "crop_review_required": not bool(start_ns and end_ns and end_ns >= start_ns),
        "cropped_duration_s": (end_ns - start_ns) / 1e9 if start_ns and end_ns and end_ns >= start_ns else None,
        "excluded_tail_rows": 0,
        "excluded_rows": 0,
    }
    if report["crop_review_required"]:
        return [], report, [{
            "episode_index": episode_id,
            "raw_frame_index": None,
            "timestamp_ns": None,
            "reasons": ["crop_review_required"],
            "views": {"act": False, "act_force": False},
        }]
    candidates: list[dict[str, Any]] = []
    excluded: list[dict[str, Any]] = []
    for row in rows:
        timestamp = int(row["timestamp_ns"])
        frame_index = int(row.get("frame_index", -1))
        if timestamp < start_ns:
            excluded.append({
                "episode_index": episode_id,
                "raw_frame_index": frame_index,
                "timestamp_ns": timestamp,
                "reasons": ["before_training_start"],
                "views": {"act": False, "act_force": False},
            })
        elif timestamp > end_ns:
            report["excluded_tail_rows"] += 1
            excluded.append({
                "episode_index": episode_id,
                "raw_frame_index": frame_index,
                "timestamp_ns": timestamp,
                "reasons": ["post_task_release_tail"],
                "views": {"act": False, "act_force": False},
            })
        else:
            candidates.append(dict(row))
    report["training_start_frame_index"] = (
        int(candidates[0].get("frame_index", -1)) if candidates else None
    )
    report["training_end_frame_index"] = (
        int(candidates[-1].get("frame_index", -1)) if candidates else None
    )
    report["cropped_frame_count_before_quality_filter"] = len(candidates)
    report["excluded_rows"] = len(excluded)
    return candidates, report, excluded


def _stats(values: Sequence[Sequence[float]], names: Sequence[str], *, source_split: str) -> dict[str, Any]:
    if not values:
        raise ValueError("cannot compute statistics from zero rows")
    array = np.asarray(values, dtype=np.float64)
    mean = array.mean(axis=0)
    std = array.std(axis=0)
    minimum = array.min(axis=0)
    maximum = array.max(axis=0)
    zero_std = [int(index) for index, value in enumerate(std) if value < 1e-12]
    return {
        "schema_version": "embodied_lab.training_stats.v1",
        "source_split": source_split,
        "sample_count": int(array.shape[0]),
        "names": list(names),
        "mean": mean.tolist(),
        "std": std.tolist(),
        "minimum": minimum.tolist(),
        "maximum": maximum.tolist(),
        "zero_std_indices": zero_std,
        "safe_std": [1.0 if index in zero_std else float(value) for index, value in enumerate(std)],
    }


def _feature_info() -> dict[str, Any]:
    image_info = {
        "dtype": "video",
        "shape": [3, 480, 640],
        "names": ["channel", "height", "width"],
        "video_info": {"video.fps": 30, "video.codec": "mp4v"},
    }
    return {
        "observation.images.workspace": image_info,
        "observation.images.wrist": image_info,
        "observation.state": {"dtype": "float32", "shape": [12], "names": list(STATE_ORDER)},
        "observation.force": {"dtype": "float32", "shape": [6], "names": list(FORCE_ORDER)},
        "action": {"dtype": "float32", "shape": [12], "names": list(ACTION_ORDER)},
        "force_age_s": {"dtype": "float32", "shape": [1], "names": ["force_age_s"]},
        "force_valid": {"dtype": "bool", "shape": [1], "names": ["force_valid"]},
        "force_timestamp_ns": {"dtype": "int64", "shape": [1], "names": ["force_timestamp_ns"]},
        "timestamp": {"dtype": "float32", "shape": [1], "names": ["timestamp"]},
        "timestamp_ns": {"dtype": "int64", "shape": [1], "names": ["timestamp_ns"]},
        "episode_index": {"dtype": "int64", "shape": [1], "names": ["episode_index"]},
        "frame_index": {"dtype": "int64", "shape": [1], "names": ["frame_index"]},
        "raw_frame_index": {"dtype": "int64", "shape": [1], "names": ["raw_frame_index"]},
        "segment_id": {"dtype": "int64", "shape": [1], "names": ["segment_id"]},
        "task_index": {"dtype": "int64", "shape": [1], "names": ["task_index"]},
    }


def _parquet_table(rows: Sequence[Mapping[str, Any]]) -> Any:
    try:
        import pyarrow as pa
    except ImportError as exc:
        raise RuntimeError("training materialization requires pyarrow") from exc
    return pa.table(
        {
            "index": pa.array([int(row["index"]) for row in rows], type=pa.int64()),
            "episode_index": pa.array([int(row["episode_index"]) for row in rows], type=pa.int64()),
            "frame_index": pa.array([int(row["frame_index"]) for row in rows], type=pa.int64()),
            "raw_frame_index": pa.array([int(row["raw_frame_index"]) for row in rows], type=pa.int64()),
            "segment_id": pa.array([int(row["segment_id"]) for row in rows], type=pa.int64()),
            "timestamp": pa.array([float(row["timestamp"]) for row in rows], type=pa.float32()),
            "timestamp_ns": pa.array([int(row["timestamp_ns"]) for row in rows], type=pa.int64()),
            "task_index": pa.array([0] * len(rows), type=pa.int64()),
            "task": pa.array([str(row["task"]) for row in rows], type=pa.string()),
            "observation.state": pa.array([row["observation.state"] for row in rows], type=pa.list_(pa.float32(), 12)),
            "action": pa.array([row["action"] for row in rows], type=pa.list_(pa.float32(), 12)),
            "observation.force": pa.array([row["observation.force"] for row in rows], type=pa.list_(pa.float32(), 6)),
            "force_age_s": pa.array([row["force_age_s"] for row in rows], type=pa.float32()),
            "force_valid": pa.array([bool(row["force_valid"]) for row in rows], type=pa.bool_()),
            "force_timestamp_ns": pa.array([row["force_timestamp_ns"] for row in rows], type=pa.int64()),
            "action_status": pa.array([str(row["action_status"]) for row in rows], type=pa.string()),
            "arm_trigger": pa.array([bool(row["arm_trigger"]) for row in rows], type=pa.bool_()),
            "hand_grip": pa.array([bool(row["hand_grip"]) for row in rows], type=pa.bool_()),
            "sync_valid": pa.array([bool(row["sync_valid"]) for row in rows], type=pa.bool_()),
            "camera_frame_index_workspace": pa.array([int(row["camera_frame_index_workspace"]) for row in rows], type=pa.int64()),
            "camera_frame_index_wrist": pa.array([int(row["camera_frame_index_wrist"]) for row in rows], type=pa.int64()),
            "timing": pa.array([str(row["timing"]) for row in rows], type=pa.string()),
        }
    )


def _write_video_subset(source: Path, destination: Path, raw_indices: Sequence[int], fps: int) -> None:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("cropped video materialization requires opencv") from exc
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise ValueError(f"cannot open source video {source}")
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = cv2.VideoWriter(
        str(destination), cv2.VideoWriter_fourcc(*"mp4v"), float(fps), (width, height)
    )
    if not writer.isOpened():
        capture.release()
        raise RuntimeError(f"cannot create training video {destination}")
    wanted = iter(raw_indices)
    next_index = next(wanted, None)
    index = 0
    written = 0
    try:
        while next_index is not None:
            ok, frame = capture.read()
            if not ok:
                raise ValueError(f"source video ended before frame {next_index}: {source}")
            if index == next_index:
                writer.write(frame)
                written += 1
                next_index = next(wanted, None)
            index += 1
    finally:
        capture.release()
        writer.release()
    if written != len(raw_indices):
        raise ValueError(f"wrote {written} of {len(raw_indices)} frames from {source}")


def _link_or_crop_video(
    source: Path,
    destination: Path,
    raw_indices: Sequence[int],
    fps: int,
    *,
    source_frame_count: int | None = None,
) -> str:
    destination.parent.mkdir(parents=True, exist_ok=True)
    contiguous = bool(raw_indices) and list(raw_indices) == list(range(raw_indices[-1] + 1))
    if contiguous and source_frame_count == len(raw_indices):
        destination.symlink_to(os.path.relpath(source, destination.parent))
        return "relative_symlink_full_source_prefix"
    _write_video_subset(source, destination, raw_indices, fps)
    return "materialized_cropped_video"


def _split_ids(config: Mapping[str, Any], included: Sequence[int]) -> dict[str, list[int]]:
    policy = config.get("split_policy", {})
    if not isinstance(policy, Mapping):
        raise ValueError("split_policy must be a mapping")
    mode = policy.get("mode", "explicit")
    result = {name: [int(value) for value in policy.get(name, [])] for name in ("train", "val", "test")}
    if mode == "explicit":
        assigned = [episode_id for values in result.values() for episode_id in values]
        if len(assigned) != len(set(assigned)):
            raise ValueError("an episode appears in more than one split")
        if set(assigned) != set(included):
            missing = sorted(set(included) - set(assigned))
            extra = sorted(set(assigned) - set(included))
            raise ValueError(f"explicit splits do not match included episodes; missing={missing}, extra={extra}")
        return result
    if mode != "hash":
        raise ValueError("split_policy.mode must be explicit or hash")
    train_fraction = float(policy.get("train_fraction", 0.8))
    val_fraction = float(policy.get("val_fraction", 0.1))
    seed = str(policy.get("seed", "embodied-lab-v1"))
    if not (0.0 < train_fraction < 1.0 and 0.0 <= val_fraction < 1.0 and train_fraction + val_fraction < 1.0):
        raise ValueError("invalid hash split fractions")
    result = {"train": [], "val": [], "test": []}
    for episode_id in sorted(included):
        value = int.from_bytes(hashlib.sha256(f"{seed}:{episode_id}".encode()).digest()[:8], "big") / float(1 << 64)
        result["train" if value < train_fraction else "val" if value < train_fraction + val_fraction else "test"].append(episode_id)
    return result


def _report_markdown(summary: Mapping[str, Any]) -> str:
    lines = [
        "# Physical bottle training dataset v1",
        "",
        "This is an offline materialization report. Raw staging episodes were not modified.",
        "",
        f"- Master schema: `{summary['schema_version']}`",
        f"- Task: `{summary['task_id']}` — {summary['task_prompt']}",
        f"- Included episodes: {summary['included_episode_count']}",
        f"- Master rows: {summary['master_row_count']}",
        f"- Cropped duration: {summary['master_duration_s']:.3f} s",
        f"- Master image frames: {summary['master_image_count']} (workspace + wrist)",
        f"- ACT+Force-valid rows: {summary['act_force_row_count']}",
        f"- Train/val/test episodes: `{summary['splits']['train']}` / `{summary['splits']['val']}` / `{summary['splits']['test']}`",
        "",
        "## Per-episode materialization",
        "",
        "| episode | raw duration | raw frames | crop start | crop end | crop frames | excluded | force valid | recovery/manual flag | status |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---|---|",
    ]
    for episode in summary["episodes"]:
        crop = episode["crop"]
        force_ratio = episode.get("force_valid_ratio")
        force_text = "n/a" if force_ratio is None else f"{force_ratio:.3f}"
        lines.append(
            f"| {episode['episode_index']} | {episode.get('raw_duration_s', 'n/a')} | {crop['raw_frame_count']} | {crop.get('training_start_timestamp_ns')} | {crop.get('training_end_timestamp_ns')} | {crop.get('cropped_frame_count_before_quality_filter', episode.get('included_row_count', 0))} | {episode.get('excluded_row_count', crop.get('excluded_tail_rows', 0) + episode.get('excluded_quality_rows', 0))} | {force_text} | "
            f"{episode.get('recovery_heavy', False) or episode.get('review_required', False)} | {episode['status']} |"
        )
    lines.extend([
        "",
        "## Semantics",
        "",
        "The master stores absolute/native 12-D actions. ACT reads images, state, and action; ACT+Force reads the same rows plus six raw FORCE_ACT counts, force age, and force validity. The initial openpi adapter reads images, state, task prompt, and absolute action, and does not read force.",
        "",
        "Normalization files are computed only from the configured train episodes. Force statistics use only rows with `force_valid=true`.",
        "",
        "A short final both-clutches-released debounce/reset tail is excluded using the persisted release timestamp. No fixed five-second subtraction or vision-based crop was used.",
        "",
    ])
    return "\n".join(lines)


def materialize_training_dataset(config_path: str | Path, *, replace: bool = False) -> MaterializationResult:
    config_path, config = load_training_config(config_path)
    source_root = _resolve_config_path(config_path, str(config["source_root"]))
    output_root = _resolve_config_path(config_path, str(config["output_root"]))
    report_value = config.get("report_path")
    report_path = _resolve_config_path(config_path, str(report_value)) if report_value else None
    if (
        output_root == source_root
        or output_root.is_relative_to(source_root)
        or source_root.is_relative_to(output_root)
    ):
        raise ValueError("training output root must be disjoint from immutable raw source root")
    entries = {int(item["id"]): dict(item) for item in config["episodes"]}
    if len(entries) != len(config["episodes"]):
        raise ValueError("training manifest contains duplicate episode ids")
    all_ids = sorted(entries)
    inventory = _source_inventory(source_root, all_ids)
    fingerprint = _config_fingerprint(config, inventory)
    marker = output_root / "master" / "meta" / "materialization.json"
    if marker.is_file():
        existing = _read_json(marker)
        if existing.get("fingerprint") == fingerprint and not replace:
            summary = _read_json(output_root / "reports" / "materialization_summary.json")
            return MaterializationResult(output_root, report_path, summary, reused=True)
        if not replace:
            raise FileExistsError(f"materialized dataset exists with a different fingerprint: {output_root}")
    if output_root.exists() and any(output_root.iterdir()):
        if not replace:
            raise FileExistsError(f"output root is not empty: {output_root}; use --replace for a generated root")
        if (
            output_root.resolve() == source_root.resolve()
            or output_root.resolve().is_relative_to(source_root.resolve())
            or source_root.resolve().is_relative_to(output_root.resolve())
        ):
            raise ValueError("refusing to replace a root that contains raw source data")
        shutil.rmtree(output_root)
    output_root.parent.mkdir(parents=True, exist_ok=True)
    temporary_root = Path(tempfile.mkdtemp(prefix=f".{output_root.name}.", dir=output_root.parent))
    try:
        master_root = temporary_root / "master"
        task = config["task"]
        task_id = str(task["id"])
        task_prompt = str(task["prompt"])
        if task_prompt != TASK_PROMPT:
            raise ValueError(f"current physical bottle task prompt must be {TASK_PROMPT!r}")
        included_ids = [episode_id for episode_id, entry in entries.items() if bool(entry.get("include", False))]
        splits = _split_ids(config, included_ids)
        split_by_episode = {episode_id: split for split, ids in splits.items() for episode_id in ids}
        output_episode_rows: dict[int, list[dict[str, Any]]] = {}
        episode_reports: list[dict[str, Any]] = []
        excluded_rows: list[dict[str, Any]] = []
        global_index = 0
        for episode_id in all_ids:
            entry = entries[episode_id]
            paths = _episode_paths(source_root, episode_id)
            base_report = {
                "episode_index": episode_id,
                "include_requested": bool(entry.get("include", False)),
                "clean_demo": bool(entry.get("clean_demo", False)),
                "recovery_heavy": bool(entry.get("recovery_heavy", False)),
                "review_required": bool(entry.get("review_required", False)),
                "notes": entry.get("notes"),
                "status": "excluded_by_manifest" if not entry.get("include", False) else "pending",
            }
            if not all(paths[name].is_file() for name in ("metadata", "rows", "workspace", "wrist")):
                base_report["status"] = "excluded_missing_source"
                base_report["crop"] = {"raw_frame_count": None, "crop_review_required": True}
                episode_reports.append(base_report)
                excluded_rows.append({"episode_index": episode_id, "raw_frame_index": None, "timestamp_ns": None, "reasons": ["missing_source_file"], "views": {"act": False, "act_force": False}})
                continue
            metadata = _read_json(paths["metadata"])
            rows, read_errors = _read_rows(paths["rows"])
            if read_errors or not rows or metadata.get("completion_status") != "completed":
                base_report["status"] = "excluded_malformed_or_incomplete"
                base_report["read_errors"] = read_errors
                base_report["crop"] = {"raw_frame_count": len(rows), "crop_review_required": True}
                episode_reports.append(base_report)
                continue
            candidates, crop_report, crop_excluded = _crop_rows(rows, metadata, entry)
            base_report["crop"] = crop_report
            excluded_rows.extend(crop_excluded)
            if crop_report["crop_review_required"]:
                base_report["status"] = "crop_review_required"
                episode_reports.append(base_report)
                continue
            if not entry.get("include", False):
                base_report.update({
                    "status": "excluded_by_manifest",
                    "included_row_count": 0,
                    "excluded_quality_rows": 0,
                    "excluded_row_count": len(crop_excluded) + len(candidates),
                    "force_valid_ratio": (
                        sum(
                            bool(
                                isinstance(candidate.get("timing"), Mapping)
                                and candidate["timing"].get("rh56_force_act_valid", False)
                            )
                            for candidate in candidates
                        )
                        / len(candidates)
                        if candidates
                        else None
                    ),
                    "raw_duration_s": metadata.get("duration_s"),
                    "task_success_label": entry.get("success_label"),
                    "exclusion_reasons": ["manifest_include_false"],
                })
                episode_reports.append(base_report)
                continue
            output_rows: list[dict[str, Any]] = []
            previous_timestamp: int | None = None
            previous_raw_frame_index: int | None = None
            segment_id = 0
            expected_period_ns = round(1_000_000_000 / int(config.get("fps", 30)))
            for row in candidates:
                raw_frame_index = int(row.get("frame_index", -1))
                reasons = _row_reasons(row, require_force=False)
                timestamp = int(row["timestamp_ns"])
                if previous_timestamp is not None and timestamp <= previous_timestamp:
                    reasons.append("non_monotonic_canonical_timestamp")
                previous_timestamp = timestamp
                timing = row.get("timing")
                timing = timing if isinstance(timing, Mapping) else {}
                source_validity = timing.get("source_validity")
                source_validity = source_validity if isinstance(source_validity, Mapping) else {}
                force_valid = bool(timing.get("rh56_force_act_valid", False)) and bool(source_validity.get("rh56_force_act", False))
                force_reasons = _row_reasons(row, require_force=True)
                if reasons:
                    excluded_rows.append({"episode_index": episode_id, "raw_frame_index": raw_frame_index, "timestamp_ns": timestamp, "reasons": sorted(set(reasons)), "views": {"act": False, "act_force": False}})
                    continue
                if not force_valid:
                    excluded_rows.append({"episode_index": episode_id, "raw_frame_index": raw_frame_index, "timestamp_ns": timestamp, "reasons": sorted(set(force_reasons or ["invalid_source:rh56_force_act"])), "views": {"act": True, "act_force": False}})
                force_values = [float(value) for value in row["observation.force"]]
                source_timestamps = timing.get("source_timestamps_ns")
                source_timestamps = source_timestamps if isinstance(source_timestamps, Mapping) else {}
                source_force_timestamp = source_timestamps.get("rh56_force_act")
                source_ages = timing.get("source_age_ns")
                source_ages = source_ages if isinstance(source_ages, Mapping) else {}
                force_age_ns = source_ages.get("rh56_force_act")
                if previous_raw_frame_index is not None and (
                    raw_frame_index != previous_raw_frame_index + 1
                    or timestamp - int(previous_timestamp or timestamp) > expected_period_ns * 3 // 2
                ):
                    segment_id += 1
                master_row = {
                    "index": global_index,
                    "episode_index": episode_id,
                    "frame_index": len(output_rows),
                    "raw_frame_index": raw_frame_index,
                    "segment_id": segment_id,
                    "timestamp": (timestamp - int(crop_report["training_start_timestamp_ns"])) / 1e9,
                    "timestamp_ns": timestamp,
                    "task": task_prompt,
                    "observation.state": [float(value) for value in row["observation.state"]],
                    "action": [float(value) for value in row["action"]],
                    "observation.force": force_values,
                    "force_age_s": None if force_age_ns is None else float(force_age_ns) / 1e9,
                    "force_valid": force_valid,
                    "force_timestamp_ns": None if source_force_timestamp is None else int(source_force_timestamp),
                    "action_status": str(row.get("action_status", "accepted")),
                    "arm_trigger": bool(row.get("arm_trigger", False)),
                    "hand_grip": bool(row.get("hand_grip", False)),
                    "sync_valid": bool(timing.get("synchronization_valid", False)),
                    "camera_frame_index_workspace": int(row["camera_frame_index"]["workspace"]),
                    "camera_frame_index_wrist": int(row["camera_frame_index"]["wrist"]),
                    "timing": json.dumps(timing, sort_keys=True, separators=(",", ":")),
                }
                output_rows.append(master_row)
                global_index += 1
                previous_raw_frame_index = raw_frame_index
            if not output_rows:
                base_report["status"] = "excluded_no_valid_rows"
                episode_reports.append(base_report)
                continue
            output_episode_rows[episode_id] = output_rows
            base_report.update({
                "status": "included",
                "included_row_count": len(output_rows),
                "excluded_quality_rows": len(candidates) - len(output_rows),
                "excluded_row_count": len(crop_excluded) + len(candidates) - len(output_rows),
                "split": split_by_episode[episode_id],
                "force_valid_ratio": sum(row["force_valid"] for row in output_rows) / len(output_rows),
                "raw_duration_s": metadata.get("duration_s"),
                "task_success_label": entry.get("success_label"),
            })
            episode_reports.append(base_report)
        if not output_episode_rows:
            raise ValueError("no episodes produced master rows")
        for episode_id, rows_for_episode in sorted(output_episode_rows.items()):
            name = _episode_name(episode_id)
            data_path = master_root / "data" / "chunk-000" / f"{name}.parquet"
            data_path.parent.mkdir(parents=True, exist_ok=True)
            try:
                import pyarrow.parquet as parquet
            except ImportError as exc:
                raise RuntimeError("training materialization requires pyarrow") from exc
            parquet.write_table(_parquet_table(rows_for_episode), data_path, compression="zstd")
            paths = _episode_paths(source_root, episode_id)
            raw_indices = [int(row["raw_frame_index"]) for row in rows_for_episode]
            episode_crop = next(
                report["crop"]
                for report in episode_reports
                if report["episode_index"] == episode_id
            )
            video_modes = {}
            for role in ("workspace", "wrist"):
                destination = master_root / "videos" / f"observation.images.{role}" / "chunk-000" / f"{name}.mp4"
                video_modes[role] = _link_or_crop_video(
                    paths[role],
                    destination,
                    raw_indices,
                    int(config.get("fps", 30)),
                    source_frame_count=int(episode_crop.get("raw_frame_count", 0)),
                )
            for episode_report in episode_reports:
                if episode_report["episode_index"] == episode_id:
                    episode_report["video_modes"] = video_modes
                    break
        _write_json(master_root / "meta" / "info.json", {
            "codebase_version": "embodied_lab",
            "schema_version": TRAINING_SCHEMA_VERSION,
            "fps": int(config.get("fps", 30)),
            "robot_type": "jaka_mini2_rh56dfx",
            "total_episodes": len(output_episode_rows),
            "total_frames": global_index,
            "total_tasks": 1,
            "total_videos": 2,
            "data_path": "data/chunk-{chunk_index:03d}/episode_{episode_index:06d}.parquet",
            "video_path": "videos/{video_key}/chunk-{chunk_index:03d}/episode_{episode_index:06d}.mp4",
            "features": _feature_info(),
            "task_prompt": task_prompt,
            "task_id": task_id,
            "action_semantics": "absolute_native_target",
            "observation_state_units": {"arm": "rad", "rh56": "normalized_closure_0_to_1"},
            "observation_force_units": "rh56_force_act_raw_count",
            "rh56_channel_order": list(CANONICAL_HAND_ORDER),
        })
        _write_jsonl(master_root / "meta" / "tasks.jsonl", [{"task_index": 0, "task": task_prompt, "task_id": task_id}])
        _write_jsonl(master_root / "meta" / "episodes.jsonl", [
            {"episode_index": episode_id, "episode_name": _episode_name(episode_id), "length": len(rows_for_episode), "task_index": 0, "task": task_prompt, "data": f"data/chunk-000/{_episode_name(episode_id)}.parquet", "videos": {role: f"videos/observation.images.{role}/chunk-000/{_episode_name(episode_id)}.mp4" for role in ("workspace", "wrist")}}
            for episode_id, rows_for_episode in sorted(output_episode_rows.items())
        ])
        split_payload = {name: sorted(ids) for name, ids in splits.items()}
        _write_json(temporary_root / "manifests" / "episodes.json", {"schema_version": "embodied_lab.training_episode_manifest.v1", "config": str(config_path), "episodes": episode_reports})
        _write_json(temporary_root / "manifests" / "splits.json", {"schema_version": "embodied_lab.training_splits.v1", "unit": "episode", "splits": split_payload})
        train_rows = [row for episode_id, rows_for_episode in output_episode_rows.items() if episode_id in splits["train"] for row in rows_for_episode]
        force_rows = [row for row in train_rows if row["force_valid"]]
        all_force_rows = [row for rows_for_episode in output_episode_rows.values() for row in rows_for_episode if row["force_valid"]]
        stats = {
            "state": _stats([row["observation.state"] for row in train_rows], STATE_ORDER, source_split="train"),
            "action": _stats([row["action"] for row in train_rows], ACTION_ORDER, source_split="train"),
            "force": _stats([row["observation.force"] for row in force_rows], FORCE_ORDER, source_split="train") if force_rows else None,
        }
        _write_json(temporary_root / "stats" / "state_stats.json", stats["state"])
        _write_json(temporary_root / "stats" / "action_stats.json", stats["action"])
        if stats["force"] is not None:
            _write_json(temporary_root / "stats" / "force_stats.json", stats["force"])
        _write_json(temporary_root / "stats" / "openpi_stats.json", {"state": stats["state"], "action": stats["action"]})
        summary = {
            "schema_version": TRAINING_SCHEMA_VERSION,
            "fingerprint": fingerprint,
            "source_root": str(source_root),
            "output_root": str(output_root),
            "task_id": task_id,
            "task_prompt": task_prompt,
            "master_row_count": global_index,
            "master_duration_s": sum(
                (
                    int(rows_for_episode[-1]["timestamp_ns"])
                    - int(rows_for_episode[0]["timestamp_ns"])
                )
                / 1e9
                for rows_for_episode in output_episode_rows.values()
                if rows_for_episode
            ),
            "master_image_count": global_index * 2,
            "act_force_row_count": len(all_force_rows),
            "included_episode_count": len(output_episode_rows),
            "splits": split_payload,
            "episodes": episode_reports,
            "source_inventory": inventory,
            "normalization": {"train_episodes": splits["train"], "force_valid_rows": len(force_rows)},
            "raw_episodes_immutable": True,
        }
        _write_json(temporary_root / "reports" / "materialization_summary.json", summary)
        _write_json(temporary_root / "reports" / "episode_crop_report.json", {"episodes": episode_reports})
        _write_jsonl(temporary_root / "reports" / "excluded_rows.jsonl", excluded_rows)
        _write_json(master_root / "meta" / "materialization.json", {"schema_version": TRAINING_SCHEMA_VERSION, "fingerprint": fingerprint, "source_root": str(source_root), "source_inventory": inventory, "raw_episodes_immutable": True})
        report_text = _report_markdown(summary)
        report_inside = temporary_root / "reports" / "materialization_report.md"
        report_inside.parent.mkdir(parents=True, exist_ok=True)
        report_inside.write_text(report_text, encoding="utf-8")
        if report_path is not None:
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report_path.write_text(report_text, encoding="utf-8")
        os.replace(temporary_root, output_root)
        temporary_root = Path()
    finally:
        if str(temporary_root) not in {"", "."} and temporary_root.exists():
            shutil.rmtree(temporary_root)
    return MaterializationResult(output_root, report_path, summary, reused=False)


def validate_training_dataset(root: str | Path) -> dict[str, Any]:
    """Validate the generated master with pyarrow and the local video reader."""

    root = Path(root).resolve()
    master = root / "master"
    info = _read_json(master / "meta" / "info.json")
    if info.get("schema_version") != TRAINING_SCHEMA_VERSION:
        raise ValueError("unsupported training master schema")
    try:
        import pyarrow.parquet as parquet
    except ImportError as exc:
        raise RuntimeError("training validation requires pyarrow") from exc
    errors: list[str] = []
    total = 0
    force_valid = 0
    for line in (master / "meta" / "episodes.jsonl").read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        episode = json.loads(line)
        episode_id = int(episode["episode_index"])
        table = parquet.read_table(master / episode["data"])
        rows = table.to_pylist()
        total += len(rows)
        force_valid += sum(bool(row["force_valid"]) for row in rows)
        previous = None
        for row in rows:
            if previous is not None and int(row["timestamp_ns"]) <= previous:
                errors.append(f"episode {episode_id}: timestamp not strictly increasing")
            previous = int(row["timestamp_ns"])
            if len(row["observation.state"]) != 12 or len(row["action"]) != 12 or len(row["observation.force"]) != 6:
                errors.append(f"episode {episode_id}: feature dimension mismatch")
            if not all(math.isfinite(float(value)) for key in ("observation.state", "action", "observation.force") for value in row[key]):
                errors.append(f"episode {episode_id}: non-finite feature")
            if row["force_valid"] and row["force_timestamp_ns"] is None:
                errors.append(f"episode {episode_id}: valid force has no source timestamp")
            timing = json.loads(row["timing"])
            canonical = int(row["timestamp_ns"])
            for name, timestamp in timing.get("source_timestamps_ns", {}).items():
                if timestamp is not None and int(timestamp) > canonical:
                    errors.append(f"episode {episode_id} frame {row['frame_index']}: future source {name}")
        for role in ("workspace", "wrist"):
            video = master / f"videos/observation.images.{role}/chunk-000/{_episode_name(episode_id)}.mp4"
            if not video.exists():
                errors.append(f"episode {episode_id}: missing {role} video")
    loader_status: dict[str, Any]
    try:
        import cv2
        decoded = 0
        for line in (master / "meta" / "episodes.jsonl").read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            episode = json.loads(line)
            for role in ("workspace", "wrist"):
                video = master / f"videos/observation.images.{role}/chunk-000/{_episode_name(int(episode['episode_index']))}.mp4"
                capture = cv2.VideoCapture(str(video))
                if not capture.isOpened():
                    errors.append(f"episode {episode['episode_index']}: cannot open {role} video")
                    continue
                count = 0
                while True:
                    ok, _ = capture.read()
                    if not ok:
                        break
                    count += 1
                capture.release()
                if count != int(episode["length"]):
                    errors.append(f"episode {episode['episode_index']}: {role} decoded {count} != {episode['length']}")
                decoded += count
        loader_status = {"status": "passed", "decoded_video_frames": decoded}
    except ImportError:
        loader_status = {"status": "skipped", "reason": "opencv not installed"}
    lerobot_status: dict[str, Any]
    try:
        import lerobot  # type: ignore[import-not-found]
        lerobot_status = {"status": "available", "version": getattr(lerobot, "__version__", None)}
    except ImportError:
        lerobot_status = {"status": "skipped", "reason": "lerobot is not installed"}
    result = {
        "schema_version": TRAINING_SCHEMA_VERSION,
        "status": "passed" if not errors else "failed",
        "errors": errors,
        "episode_count": len([line for line in (master / "meta" / "episodes.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]),
        "row_count": total,
        "force_valid_count": force_valid,
        "loader": loader_status,
        "lerobot": lerobot_status,
        "raw_episodes_modified": False,
    }
    return result
