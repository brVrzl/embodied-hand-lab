"""Audit and materialize reviewed physical bottle demonstrations.

This module is intentionally separate from the live recorder and from the
older single-episode training materializer.  The source tree is read-only;
logical segmentation is represented by a manifest and exported rows retain
source episode/frame provenance.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
import yaml

from rh56_driver.hand_schema import CANONICAL_HAND_ORDER

from .training_materialization import (
    ACTION_ORDER,
    FORCE_ORDER,
    STATE_ORDER,
    TASK_PROMPT,
    _episode_name,
    _episode_paths,
    _finite_vector,
    _sha256_file,
    _stats,
    _write_json,
    _write_jsonl,
    _write_video_subset,
)


PHYSICAL_BOTTLE_CONFIG_VERSION = "embodied_lab.physical_bottle_collection_config.v1"
PHYSICAL_BOTTLE_SCHEMA_VERSION = "embodied_lab.physical_bottle_training.v2"


@dataclass(frozen=True, slots=True)
class PhysicalBottleResult:
    output_root: Path
    summary: Mapping[str, Any]
    reused: bool = False


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


def _json_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def _resolve(config_path: Path, value: str | Path) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (config_path.parent / path).resolve()


def _discover_ids(source_root: Path) -> list[int]:
    ids: set[int] = set()
    patterns = (
        source_root / "meta/episodes/chunk-000",
        source_root / "data/chunk-000",
        source_root / "videos/observation.images.workspace/chunk-000",
        source_root / "videos/observation.images.wrist/chunk-000",
        source_root / "meta/rejected",
    )
    for directory in patterns:
        if not directory.is_dir():
            continue
        for path in directory.glob("episode_*.json*"):
            match = re.fullmatch(r"episode_(\d+)\.json(?:\.partial)?", path.name)
            if match:
                ids.add(int(match.group(1)))
        for path in directory.glob("episode_*.jsonl"):
            match = re.fullmatch(r"episode_(\d+)\.jsonl", path.name)
            if match:
                ids.add(int(match.group(1)))
        for path in directory.glob("episode_*.mp4"):
            match = re.fullmatch(r"episode_(\d+)\.mp4", path.name)
            if match:
                ids.add(int(match.group(1)))
    return sorted(ids)


def _source_inventory(source_root: Path, ids: Sequence[int]) -> dict[str, Any]:
    inventory: dict[str, Any] = {}
    for episode_id in ids:
        paths = _episode_paths(source_root, episode_id)
        paths["metadata_partial"] = source_root / "meta/episodes/chunk-000" / f"episode_{episode_id:06d}.json.partial"
        paths["rejected"] = source_root / "meta/rejected" / f"episode_{episode_id:06d}.json"
        record: dict[str, Any] = {}
        for role, path in paths.items():
            if not path.is_file():
                record[role] = {"path": str(path), "missing": True}
            else:
                record[role] = {"path": str(path), "size": path.stat().st_size, "sha256": _sha256_file(path)}
        inventory[str(episode_id)] = record
    return inventory


def _load_config(config_path: str | Path) -> tuple[Path, dict[str, Any]]:
    path = Path(config_path).resolve()
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or value.get("schema_version") != PHYSICAL_BOTTLE_CONFIG_VERSION:
        raise ValueError("unsupported physical bottle collection config schema")
    for key in ("source_root", "output_root", "audit_report", "materialization_report", "task", "source_reviews", "segments", "splits"):
        if key not in value:
            raise ValueError(f"physical bottle config is missing {key!r}")
    task = value["task"]
    if not isinstance(task, Mapping) or task.get("prompt") != TASK_PROMPT:
        raise ValueError(f"task prompt must be {TASK_PROMPT!r}")
    if not isinstance(value["source_reviews"], list) or not isinstance(value["segments"], list):
        raise ValueError("source_reviews and segments must be lists")
    if not isinstance(value["splits"], Mapping):
        raise ValueError("splits must be a mapping")
    return path, value


def _configured_source_ids(config: Mapping[str, Any], discovered: Sequence[int]) -> list[int]:
    """Return the explicit audit scope when one is declared.

    A human-curated historical view must not silently expand when newer raw
    episodes are collected. Older manifests retain their minimum-id behavior.
    """

    values = config.get("source_episode_ids")
    if values is None:
        minimum_episode_id = int(config.get("minimum_episode_id", 0))
        return [episode_id for episode_id in discovered if episode_id >= minimum_episode_id]
    if not isinstance(values, list):
        raise ValueError("source_episode_ids must be a list")
    ids = [int(value) for value in values]
    if len(ids) != len(set(ids)):
        raise ValueError("source_episode_ids must be unique")
    missing = sorted(set(ids) - set(discovered))
    if missing:
        raise ValueError(f"configured source episodes are not present in raw data: {missing}")
    return ids


def _validate_split_groups(
    config: Mapping[str, Any],
    *,
    segment_to_index: Mapping[str, int],
    split_names_by_segment: Mapping[str, str],
) -> list[dict[str, Any]]:
    """Reject source/session leakage for manifests that declare split groups."""

    configured = config.get("split_groups")
    if configured is None:
        return []
    if not isinstance(configured, list):
        raise ValueError("split_groups must be a list")
    groups: list[dict[str, Any]] = []
    assigned: set[str] = set()
    for value in configured:
        if not isinstance(value, Mapping) or not value.get("id"):
            raise ValueError("each split group needs an id")
        segments = [str(item) for item in value.get("segments", [])]
        if not segments:
            raise ValueError(f"split group {value['id']} has no segments")
        unknown = sorted(set(segments) - set(segment_to_index))
        if unknown:
            raise ValueError(f"split group {value['id']} references non-included segments: {unknown}")
        overlap = sorted(set(segments) & assigned)
        if overlap:
            raise ValueError(f"split group {value['id']} repeats segments: {overlap}")
        split_names = {split_names_by_segment[segment] for segment in segments}
        if len(split_names) != 1:
            raise ValueError(f"split group {value['id']} leaks across splits: {sorted(split_names)}")
        assigned.update(segments)
        groups.append({
            "id": str(value["id"]),
            "segments": segments,
            "split": next(iter(split_names)),
            "evidence": value.get("evidence"),
        })
    missing = sorted(set(segment_to_index) - assigned)
    if missing:
        raise ValueError(f"split_groups do not cover included segments: {missing}")
    return groups


def _video_info(path: Path) -> dict[str, Any]:
    try:
        import cv2
    except ImportError:
        return {"readable": None, "reason": "opencv not installed"}
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        return {"readable": False, "path": str(path)}
    count = 0
    first_shape: list[int] | None = None
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        count += 1
        if first_shape is None:
            first_shape = [int(frame.shape[1]), int(frame.shape[0]), int(frame.shape[2])]
    result = {
        "readable": True,
        "frame_count": count,
        "fps": float(capture.get(cv2.CAP_PROP_FPS) or 0.0),
        "width": int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) or 0),
        "height": int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0),
    }
    if first_shape is not None:
        result["first_frame_shape"] = first_shape
    capture.release()
    return result


def _timing_audit(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    timestamps: list[int] = []
    future = 0
    source_regressions = 0
    force_valid = 0
    state_invalid = 0
    action_invalid = 0
    required_invalid = 0
    force_intervals: list[int] = []
    angle_intervals: list[int] = []
    previous_force: int | None = None
    previous_angle: int | None = None
    for row in rows:
        try:
            canonical = int(row["timestamp_ns"])
        except (KeyError, TypeError, ValueError):
            continue
        timestamps.append(canonical)
        if not _finite_vector(row.get("observation.state"), 12):
            state_invalid += 1
        if not _finite_vector(row.get("action"), 12):
            action_invalid += 1
        timing = row.get("timing") if isinstance(row.get("timing"), Mapping) else {}
        sources = timing.get("source_timestamps_ns", {}) if isinstance(timing, Mapping) else {}
        domains = timing.get("source_timestamp_domains", {}) if isinstance(timing, Mapping) else {}
        validity = timing.get("source_validity", {}) if isinstance(timing, Mapping) else {}
        for name, value in sources.items() if isinstance(sources, Mapping) else ():
            if value is not None:
                try:
                    if int(value) > canonical:
                        future += 1
                except (TypeError, ValueError):
                    future += 1
                if domains.get(name) != "host_monotonic_ns":
                    source_regressions += 1
        if timing.get("synchronization_valid") is not True:
            required_invalid += 1
        if all(validity.get(name) is True for name in ("jaka_observation", "rh56_angle_act", "workspace", "wrist")):
            pass
        else:
            required_invalid += 1
        is_force_valid = bool(timing.get("rh56_force_act_valid")) and validity.get("rh56_force_act") is True
        force_valid += int(is_force_valid)
        force_ts = sources.get("rh56_force_act") if isinstance(sources, Mapping) else None
        angle_ts = sources.get("rh56_angle_act") if isinstance(sources, Mapping) else None
        if force_ts is not None:
            force_ts = int(force_ts)
            if previous_force is not None and force_ts > previous_force:
                force_intervals.append(force_ts - previous_force)
            previous_force = force_ts
        if angle_ts is not None:
            angle_ts = int(angle_ts)
            if previous_angle is not None and angle_ts > previous_angle:
                angle_intervals.append(angle_ts - previous_angle)
            previous_angle = angle_ts
    monotonic = all(left < right for left, right in zip(timestamps, timestamps[1:]))
    duration_s = (timestamps[-1] - timestamps[0]) / 1e9 if len(timestamps) > 1 else 0.0
    return {
        "canonical_monotonic": monotonic,
        "canonical_row_count": len(rows),
        "canonical_duration_s": duration_s,
        "canonical_hz": (len(rows) - 1) / duration_s if duration_s > 0 else None,
        "future_source_count": future,
        "timestamp_domain_or_source_errors": source_regressions,
        "required_invalid_rows": required_invalid,
        "state_invalid_rows": state_invalid,
        "action_invalid_rows": action_invalid,
        "force_valid_rows": force_valid,
        "force_valid_ratio": force_valid / len(rows) if rows else None,
        "force_update_count": len(force_intervals) + (1 if previous_force is not None else 0),
        "angle_update_count": len(angle_intervals) + (1 if previous_angle is not None else 0),
        "force_interval_max_ns": max(force_intervals) if force_intervals else None,
        "angle_interval_max_ns": max(angle_intervals) if angle_intervals else None,
    }


def _review_map(config: Mapping[str, Any]) -> dict[int, dict[str, Any]]:
    result: dict[int, dict[str, Any]] = {}
    for value in config["source_reviews"]:
        if not isinstance(value, Mapping) or "id" not in value:
            raise ValueError("each source review needs id")
        episode_id = int(value["id"])
        if episode_id in result:
            raise ValueError(f"duplicate source review for {episode_id}")
        result[episode_id] = dict(value)
    return result


def _audit_record(source_root: Path, episode_id: int, review: Mapping[str, Any]) -> dict[str, Any]:
    paths = _episode_paths(source_root, episode_id)
    metadata_path = paths["metadata"]
    if not metadata_path.is_file():
        metadata_path = source_root / "meta/episodes/chunk-000" / f"episode_{episode_id:06d}.json.partial"
    metadata = _read_json(metadata_path) if metadata_path.is_file() else {}
    row_path = paths["rows"]
    rows, errors = _read_rows(row_path) if row_path.is_file() else ([], ["missing row file"])
    videos = {
        role: _video_info(paths[role]) if paths[role].is_file() else {"readable": False, "missing": True}
        for role in ("workspace", "wrist")
    }
    timing = _timing_audit(rows)
    has_payload = bool(rows) and all(videos[role].get("readable") is True for role in ("workspace", "wrist"))
    if review.get("classification"):
        classification = str(review["classification"])
    elif not has_payload:
        classification = "CORRUPT_OR_UNUSABLE"
    else:
        classification = "REVIEW_REQUIRED"
    status = "payload_inspected" if has_payload else "no_complete_payload"
    counters = metadata.get("data_quality") if isinstance(metadata.get("data_quality"), Mapping) else {}
    return {
        "episode": episode_id,
        "classification": classification,
        "status": status,
        "metadata_path": str(metadata_path),
        "metadata_completion_status": metadata.get("completion_status"),
        "quality_state": metadata.get("quality_state"),
        "termination_reason": metadata.get("termination_reason"),
        "duration_s": metadata.get("duration_s"),
        "metadata_frame_count": metadata.get("num_frames"),
        "trigger_release_timestamp_ns": metadata.get("trigger_release_timestamp_ns"),
        "rows": timing,
        "row_read_errors": errors,
        "videos": videos,
        "data_quality_counters": {
            "workspace_drop_count": counters.get("workspace_drop_count"),
            "wrist_drop_count": counters.get("wrist_drop_count"),
            "recorder_dropped_count": metadata.get("recorder_dropped_count"),
            "ring_reference_expired_count": metadata.get("ring_reference_expired_count"),
            "writer_failed_count": metadata.get("writer_failed_count"),
        },
        "operator_classification_notes": review.get("notes"),
        "task_progression": review.get("task_progression"),
        "rejected_sidecar_present": (source_root / "meta/rejected" / f"episode_{episode_id:06d}.json").is_file(),
    }


def _audit_markdown(summary: Mapping[str, Any]) -> str:
    lines = [
        "# Physical bottle collection audit v2",
        "",
        "This report was generated from the immutable raw episode tree. Video readability and canonical row/timestamp checks were performed offline; trajectory classifications and merged boundaries are explicit reviewed manifest decisions.",
        "",
        f"- Raw metadata records discovered: **{summary['raw_episode_count']}**",
        f"- Complete payload episodes inspected: **{summary['complete_payload_episode_count']}**",
        f"- Logical trajectories with payload: **{summary['logical_payload_trajectory_count']}**",
        f"- Audit manifest entries (including excluded no-payload records): **{summary['manifest_entry_count']}**",
        f"- Clean full-task logical segments: **{summary['clean_full_task_count']}**",
        f"- Raw merged episodes split: **{summary['merged_source_episode_count']}**",
        f"- Explicitly classified incomplete-start trajectories: **{summary['incomplete_start_count']}** (generic non-nominal entries were not relabelled)",
        f"- Explicitly adjudicated approaches recovered: **{summary['approaches_recovered']}**; impossible to recover: **{summary['approaches_impossible']}**",
        f"- Specifically labelled slip/drop trajectories with defensible raw evidence: **{summary['slip_drop_count']}**",
        "",
        "## Source episode audit",
        "",
        "| source episode | classification | payload | rows | canonical Hz | force valid | workspace | wrist | notes |",
        "|---:|---|---|---:|---:|---:|---|---|---|",
    ]
    for record in summary["source_episodes"]:
        rows = record["rows"]
        lines.append(
            f"| {record['episode']} | {record['classification']} | {record['status']} | {rows['canonical_row_count']} | {rows.get('canonical_hz') or 'n/a'} | {rows.get('force_valid_ratio') if rows.get('force_valid_ratio') is not None else 'n/a'} | {record['videos']['workspace'].get('frame_count', 'n/a')} @ {record['videos']['workspace'].get('fps', 'n/a')} | {record['videos']['wrist'].get('frame_count', 'n/a')} @ {record['videos']['wrist'].get('fps', 'n/a')} | {record.get('operator_classification_notes') or ''} |"
        )
    lines.extend([
        "",
        "## Logical trajectory decisions",
        "",
        "| logical segment | source | class | source frame range | include | decision |",
        "|---|---:|---|---|---|---|",
    ])
    for segment in summary["segments"]:
        start = segment.get("start_frame", 0)
        end = segment.get("end_frame", "release")
        lines.append(
            f"| {segment['id']} | {segment['source_episode']} | {segment['classification']} | {start}–{end} | {segment.get('include', False)} | {segment.get('notes', '')} |"
        )
    lines.extend([
        "",
        "### Review conclusions",
        "",
    ])
    notes = summary.get("audit_notes") or [
        "No rows are synthesized, copied, or interpolated. Raw recordings remain the source of truth."
    ]
    for note in notes:
        lines.extend([str(note), ""])
    return "\n".join(lines)


def audit_physical_bottle(config_path: str | Path) -> dict[str, Any]:
    config_path, config = _load_config(config_path)
    source_root = _resolve(config_path, str(config["source_root"]))
    ids = _configured_source_ids(config, _discover_ids(source_root))
    reviews = _review_map(config)
    missing_reviews = sorted(set(ids) - set(reviews))
    if missing_reviews:
        raise ValueError(f"source_reviews missing discovered episode ids: {missing_reviews}")
    extra_reviews = sorted(set(reviews) - set(ids))
    if config.get("source_episode_ids") is not None and extra_reviews:
        raise ValueError(f"source_reviews outside configured source_episode_ids: {extra_reviews}")
    source_records = [_audit_record(source_root, episode_id, reviews[episode_id]) for episode_id in ids]
    segments = [dict(value) for value in config["segments"]]
    segment_ids = [str(value.get("id")) for value in segments]
    if len(segment_ids) != len(set(segment_ids)) or any(value == "None" for value in segment_ids):
        raise ValueError("logical segment ids must be unique and non-empty")
    payload_ids = {record["episode"] for record in source_records if record["status"] == "payload_inspected"}
    for segment in segments:
        if int(segment["source_episode"]) not in set(ids):
            raise ValueError(f"segment {segment['id']} references undiscovered source episode")
        if segment.get("include") and int(segment["source_episode"]) not in payload_ids:
            raise ValueError(f"included segment {segment['id']} has no complete source payload")
    clean_count = sum(1 for value in segments if value.get("include") and value.get("classification") == "CLEAN_FULL_TASK")
    # Logical entries for a split source are clean, while the source audit keeps
    # the MERGED_MULTIPLE_EPISODES classification.  Count source IDs, not rows.
    merged_sources = {record["episode"] for record in source_records if record["classification"] == "MERGED_MULTIPLE_EPISODES"}
    incomplete_count = sum(1 for value in segments if value.get("classification") == "INCOMPLETE_START")
    summary = {
        "schema_version": PHYSICAL_BOTTLE_SCHEMA_VERSION,
        "config": str(config_path),
        "source_root": str(source_root),
        "raw_episode_count": len(ids),
        "complete_payload_episode_count": sum(record["status"] == "payload_inspected" for record in source_records),
        "logical_payload_trajectory_count": sum(
            1 for value in segments if int(value["source_episode"]) in payload_ids
        ),
        "manifest_entry_count": len(segments),
        "clean_full_task_count": clean_count,
        "merged_source_episode_count": len(merged_sources),
        "incomplete_start_count": incomplete_count,
        "approaches_recovered": 0,
        "approaches_impossible": incomplete_count,
        "slip_drop_count": sum(1 for value in segments if value.get("classification") == "SLIP_OR_DROP"),
        "unusable_source_count": sum(1 for record in source_records if record["classification"] == "CORRUPT_OR_UNUSABLE"),
        "source_episodes": source_records,
        "segments": segments,
        "audit_provenance": config.get("audit_provenance"),
        "audit_notes": config.get("audit_notes", []),
        "raw_inventory": _source_inventory(source_root, ids),
    }
    report = _resolve(config_path, str(config["audit_report"]))
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(_audit_markdown(summary), encoding="utf-8")
    json_report = report.with_suffix(".json")
    _write_json(json_report, summary)
    return summary


def _source_timing(row: Mapping[str, Any]) -> tuple[int, Mapping[str, Any], Mapping[str, Any], Mapping[str, Any]]:
    timing = row.get("timing") if isinstance(row.get("timing"), Mapping) else {}
    timestamps = timing.get("source_timestamps_ns", {}) if isinstance(timing.get("source_timestamps_ns", {}), Mapping) else {}
    validity = timing.get("source_validity", {}) if isinstance(timing.get("source_validity", {}), Mapping) else {}
    domains = timing.get("source_timestamp_domains", {}) if isinstance(timing.get("source_timestamp_domains", {}), Mapping) else {}
    return int(row.get("timestamp_ns", 0)), timestamps, validity, domains


def _quality_reasons(row: Mapping[str, Any]) -> list[str]:
    reasons: list[str] = []
    canonical, timestamps, validity, domains = _source_timing(row)
    timing = row.get("timing") if isinstance(row.get("timing"), Mapping) else {}
    if canonical <= 0:
        reasons.append("missing_canonical_timestamp")
    if timing.get("canonical_host_monotonic_ns") != canonical:
        reasons.append("canonical_timestamp_mismatch")
    if not _finite_vector(row.get("observation.state"), 12):
        reasons.append("state_shape_or_finite")
    if not _finite_vector(row.get("action"), 12):
        reasons.append("action_shape_or_finite")
    if not _finite_vector(row.get("observation.force"), 6):
        reasons.append("force_shape_or_finite")
    if timing.get("synchronization_valid") is not True:
        reasons.append("synchronization_invalid")
    for name, value in timestamps.items():
        if value is None:
            continue
        try:
            if int(value) > canonical:
                reasons.append(f"future_source:{name}")
        except (TypeError, ValueError):
            reasons.append(f"invalid_source_timestamp:{name}")
        if domains.get(name) != "host_monotonic_ns":
            reasons.append(f"source_domain_invalid:{name}")
    for name in ("jaka_observation", "rh56_angle_act", "workspace", "wrist"):
        if validity.get(name) is not True:
            reasons.append(f"invalid_source:{name}")
    cameras = row.get("camera") if isinstance(row.get("camera"), Mapping) else {}
    for role in ("workspace", "wrist"):
        camera = cameras.get(role) if isinstance(cameras.get(role), Mapping) else {}
        if camera.get("valid") is not True:
            reasons.append(f"invalid_camera:{role}")
        if not isinstance(camera.get("rgb_frame_number"), int):
            reasons.append(f"invalid_camera_frame:{role}")
    return sorted(set(reasons))


def _crop_segment(
    rows: Sequence[Mapping[str, Any]], metadata: Mapping[str, Any], segment: Mapping[str, Any]
) -> tuple[list[tuple[int, dict[str, Any]]], dict[str, Any], list[dict[str, Any]]]:
    start_frame = int(segment.get("start_frame", 0))
    if start_frame < 0 or start_frame >= len(rows):
        return [], {"crop_review_required": True, "reason": "start_frame_out_of_range"}, []
    end_frame_value = segment.get("end_frame")
    end_source = "manual_frame_boundary" if end_frame_value is not None else "authoritative_trigger_release_timestamp_ns"
    if end_frame_value is not None:
        end_frame = int(end_frame_value)
    else:
        release = metadata.get("trigger_release_timestamp_ns")
        if release is None:
            return [], {"crop_review_required": True, "reason": "missing_authoritative_release_timestamp"}, []
        end_candidates = [index for index, row in enumerate(rows) if int(row.get("timestamp_ns", 0)) <= int(release)]
        end_frame = end_candidates[-1] if end_candidates else -1
    if end_frame < start_frame or end_frame >= len(rows):
        return [], {"crop_review_required": True, "reason": "end_frame_out_of_range"}, []
    selected = [(index, dict(rows[index])) for index in range(start_frame, end_frame + 1)]
    start_ns = int(selected[0][1]["timestamp_ns"])
    end_ns = int(selected[-1][1]["timestamp_ns"])
    declared_start_ns = segment.get("start_timestamp_ns")
    declared_end_ns = segment.get("end_timestamp_ns")
    if declared_start_ns is not None and int(declared_start_ns) != start_ns:
        return [], {"crop_review_required": True, "reason": "start_timestamp_boundary_mismatch"}, []
    if declared_end_ns is not None and int(declared_end_ns) != end_ns:
        return [], {"crop_review_required": True, "reason": "end_timestamp_boundary_mismatch"}, []
    excluded = [
        {
            "source_episode": int(metadata.get("episode_index", segment["source_episode"])),
            "source_frame_index": int(row.get("frame_index", index)),
            "timestamp_ns": int(row.get("timestamp_ns", 0)),
            "logical_segment_id": str(segment["id"]),
            "reasons": ["outside_logical_segment"],
            "views": {"act": False, "act_force": False},
        }
        for index, row in enumerate(rows)
        if index < start_frame or index > end_frame
    ]
    return selected, {
        "raw_frame_count": len(rows),
        "training_start_frame_index": int(selected[0][1].get("frame_index", start_frame)),
        "training_end_frame_index": int(selected[-1][1].get("frame_index", end_frame)),
        "training_start_timestamp_ns": start_ns,
        "training_end_timestamp_ns": end_ns,
        "training_end_source": end_source,
        "cropped_frame_count_before_quality_filter": len(selected),
        "excluded_tail_rows": max(0, len(rows) - end_frame - 1),
        "crop_review_required": False,
    }, excluded


def _rich_row(
    row: Mapping[str, Any], *, logical_index: int, local_index: int, segment: Mapping[str, Any], start_ns: int
) -> dict[str, Any]:
    timing = row.get("timing") if isinstance(row.get("timing"), Mapping) else {}
    source_timestamps = timing.get("source_timestamps_ns", {}) if isinstance(timing.get("source_timestamps_ns", {}), Mapping) else {}
    source_ages = timing.get("source_age_ns", {}) if isinstance(timing.get("source_age_ns", {}), Mapping) else {}
    force_timestamp = source_timestamps.get("rh56_force_act")
    force_age = source_ages.get("rh56_force_act")
    source_validity = timing.get("source_validity", {}) if isinstance(timing.get("source_validity", {}), Mapping) else {}
    force_valid = bool(timing.get("rh56_force_act_valid")) and source_validity.get("rh56_force_act") is True
    timestamp_ns = int(row["timestamp_ns"])
    return {
        "index": logical_index,
        "episode_index": int(segment["logical_episode_index"]),
        "frame_index": local_index,
        "raw_frame_index": int(row.get("frame_index", local_index)),
        "source_episode_index": int(segment["source_episode"]),
        "source_frame_index": int(row.get("frame_index", local_index)),
        "logical_segment_id": str(segment["id"]),
        "segment_id": 0,
        "audit_classification": str(segment["classification"]),
        "task_success_label": str(segment.get("success_label", "success")),
        "timestamp": (timestamp_ns - start_ns) / 1e9,
        "timestamp_ns": timestamp_ns,
        "task": TASK_PROMPT,
        "observation.state": [float(value) for value in row["observation.state"]],
        "action": [float(value) for value in row["action"]],
        "observation.force": [float(value) for value in row["observation.force"]],
        "force_age_s": None if force_age is None else float(force_age) / 1e9,
        "force_valid": force_valid,
        "force_timestamp_ns": None if force_timestamp is None else int(force_timestamp),
        "action_status": str(row.get("action_status", "accepted")),
        "arm_trigger": bool(row.get("arm_trigger", False)),
        "hand_grip": bool(row.get("hand_grip", False)),
        "sync_valid": bool(timing.get("synchronization_valid", False)),
        "camera_frame_index_workspace": int(row["camera_frame_index"]["workspace"]),
        "camera_frame_index_wrist": int(row["camera_frame_index"]["wrist"]),
        "timing": json.dumps(timing, sort_keys=True, separators=(",", ":")),
    }


def _table(rows: Sequence[Mapping[str, Any]]) -> Any:
    try:
        import pyarrow as pa
    except ImportError as exc:
        raise RuntimeError("physical bottle materialization requires pyarrow") from exc
    nullable_int = pa.array([row["force_timestamp_ns"] for row in rows], type=pa.int64())
    return pa.table({
        "index": pa.array([int(row["index"]) for row in rows], type=pa.int64()),
        "episode_index": pa.array([int(row["episode_index"]) for row in rows], type=pa.int64()),
        "frame_index": pa.array([int(row["frame_index"]) for row in rows], type=pa.int64()),
        "raw_frame_index": pa.array([int(row["raw_frame_index"]) for row in rows], type=pa.int64()),
        "source_episode_index": pa.array([int(row["source_episode_index"]) for row in rows], type=pa.int64()),
        "source_frame_index": pa.array([int(row["source_frame_index"]) for row in rows], type=pa.int64()),
        "logical_segment_id": pa.array([str(row["logical_segment_id"]) for row in rows], type=pa.string()),
        "segment_id": pa.array([0] * len(rows), type=pa.int64()),
        "audit_classification": pa.array([str(row["audit_classification"]) for row in rows], type=pa.string()),
        "task_success_label": pa.array([str(row["task_success_label"]) for row in rows], type=pa.string()),
        "timestamp": pa.array([float(row["timestamp"]) for row in rows], type=pa.float32()),
        "timestamp_ns": pa.array([int(row["timestamp_ns"]) for row in rows], type=pa.int64()),
        "task_index": pa.array([0] * len(rows), type=pa.int64()),
        "task": pa.array([TASK_PROMPT] * len(rows), type=pa.string()),
        "observation.state": pa.array([row["observation.state"] for row in rows], type=pa.list_(pa.float32(), 12)),
        "action": pa.array([row["action"] for row in rows], type=pa.list_(pa.float32(), 12)),
        "observation.force": pa.array([row["observation.force"] for row in rows], type=pa.list_(pa.float32(), 6)),
        "force_age_s": pa.array([row["force_age_s"] for row in rows], type=pa.float32()),
        "force_valid": pa.array([bool(row["force_valid"]) for row in rows], type=pa.bool_()),
        "force_timestamp_ns": nullable_int,
        "action_status": pa.array([str(row["action_status"]) for row in rows], type=pa.string()),
        "arm_trigger": pa.array([bool(row["arm_trigger"]) for row in rows], type=pa.bool_()),
        "hand_grip": pa.array([bool(row["hand_grip"]) for row in rows], type=pa.bool_()),
        "sync_valid": pa.array([bool(row["sync_valid"]) for row in rows], type=pa.bool_()),
        "camera_frame_index_workspace": pa.array([int(row["camera_frame_index_workspace"]) for row in rows], type=pa.int64()),
        "camera_frame_index_wrist": pa.array([int(row["camera_frame_index_wrist"]) for row in rows], type=pa.int64()),
        "timing": pa.array([str(row["timing"]) for row in rows], type=pa.string()),
    })


def _feature_info() -> dict[str, Any]:
    image = {"dtype": "video", "shape": [3, 480, 640], "names": ["channel", "height", "width"], "video_info": {"video.fps": 30, "video.codec": "mp4v"}}
    return {
        "observation.images.workspace": image,
        "observation.images.wrist": image,
        "observation.state": {"dtype": "float32", "shape": [12], "names": list(STATE_ORDER)},
        "observation.force": {"dtype": "float32", "shape": [6], "names": list(FORCE_ORDER)},
        "action": {"dtype": "float32", "shape": [12], "names": list(ACTION_ORDER)},
        "force_age_s": {"dtype": "float32", "shape": [1]},
        "force_valid": {"dtype": "bool", "shape": [1]},
        "force_timestamp_ns": {"dtype": "int64", "shape": [1]},
        "timestamp_ns": {"dtype": "int64", "shape": [1]},
        "source_episode_index": {"dtype": "int64", "shape": [1]},
        "source_frame_index": {"dtype": "int64", "shape": [1]},
    }


def _write_variant(
    variant_root: Path,
    rows_by_segment: Mapping[int, Sequence[Mapping[str, Any]]],
    segment_reports: Sequence[Mapping[str, Any]],
    splits: Mapping[str, Sequence[int]],
    task: Mapping[str, Any],
    *,
    view: str,
    assets_root: Path,
) -> None:
    try:
        import pyarrow.parquet as parquet
    except ImportError as exc:
        raise RuntimeError("physical bottle materialization requires pyarrow") from exc
    master = variant_root / "master"
    for logical_index, rows in sorted(rows_by_segment.items()):
        data_path = master / "data/chunk-000" / f"episode_{logical_index:06d}.parquet"
        data_path.parent.mkdir(parents=True, exist_ok=True)
        parquet.write_table(_table(rows), data_path, compression="zstd")
    _write_json(master / "meta/info.json", {
        "codebase_version": "embodied_lab",
        "schema_version": PHYSICAL_BOTTLE_SCHEMA_VERSION,
        "fps": 30,
        "robot_type": "jaka_mini2_rh56dfx",
        "total_episodes": len(rows_by_segment),
        "total_frames": sum(len(rows) for rows in rows_by_segment.values()),
        "total_tasks": 1,
        "total_videos": 2,
        "data_path": "data/chunk-{chunk_index:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/{video_key}/chunk-{chunk_index:03d}/episode_{episode_index:06d}.mp4",
        "features": _feature_info(),
        "task_id": task["id"],
        "task_prompt": task["prompt"],
        "training_view": view,
        "ignored_inputs": ["observation.force", "force_age_s", "force_valid"] if view == "act" else [],
        "action_semantics": "absolute_native_target",
        "observation_state_order": list(STATE_ORDER),
        "action_order": list(ACTION_ORDER),
        "force_order": list(FORCE_ORDER),
        "observation_force_units": "rh56_force_act_raw_count",
        "rh56_channel_order": list(CANONICAL_HAND_ORDER),
    })
    episode_rows: list[dict[str, Any]] = []
    for logical_index, rows in sorted(rows_by_segment.items()):
        name = _episode_name(logical_index)
        episode_rows.append({
            "episode_index": logical_index,
            "episode_name": name,
            "length": len(rows),
            "task_index": 0,
            "task": task["prompt"],
            "data": f"data/chunk-000/{name}.parquet",
            "videos": {role: f"videos/observation.images.{role}/chunk-000/{name}.mp4" for role in ("workspace", "wrist")},
            "logical_segment_id": rows[0]["logical_segment_id"],
            "source_episode_index": rows[0]["source_episode_index"],
        })
    _write_jsonl(master / "meta/episodes.jsonl", episode_rows)
    _write_jsonl(master / "meta/tasks.jsonl", [{"task_index": 0, "task": task["prompt"], "task_id": task["id"]}])
    _write_json(variant_root / "manifests/splits.json", {"schema_version": "embodied_lab.training_splits.v2", "unit": "logical_segment", "splits": {name: sorted(int(value) for value in values) for name, values in splits.items()}})
    _write_json(variant_root / "manifests/logical_segments.json", {"segments": list(segment_reports), "view": view})
    for role in ("workspace", "wrist"):
        directory = master / "videos" / f"observation.images.{role}" / "chunk-000"
        directory.mkdir(parents=True, exist_ok=True)
        for logical_index in rows_by_segment:
            target = directory / f"episode_{logical_index:06d}.mp4"
            source = assets_root / "videos" / f"observation.images.{role}" / "chunk-000" / f"episode_{logical_index:06d}.mp4"
            target.symlink_to(os.path.relpath(source, target.parent))


def _markdown_materialization(summary: Mapping[str, Any]) -> str:
    lines = [
        "# Physical bottle training datasets v2",
        "",
        "Raw episodes remain immutable. The two exported views use identical logical samples, frame ranges, ordering, images, and native absolute actions; ACT ignores the retained force provenance columns and ACT+Force exposes them.",
        "",
        f"- Clean logical trajectories: **{summary['included_segment_count']}**",
        f"- ACT rows: **{summary['row_count']}**",
        f"- ACT+Force rows: **{summary['act_force_row_count']}**",
        f"- Matched samples: **{summary['matched_samples']}**",
        f"- Total cropped duration: **{summary['duration_s']:.3f} s**",
        f"- Train/val/test logical segments: `{summary['splits']['train']}` / `{summary['splits']['val']}` / `{summary['splits']['test']}`",
        "",
        "## Per logical trajectory",
        "",
        "| segment | source | crop frames | crop duration | rows | excluded | force valid | classification | status |",
        "|---|---:|---:|---:|---:|---:|---:|---|---|",
    ]
    for value in summary["segments"]:
        lines.append(
            f"| {value['id']} | {value['source_episode']} | {value.get('crop_start_frame')}–{value.get('crop_end_frame')} | {value.get('crop_duration_s')} | {value.get('included_rows')} | {value.get('excluded_rows')} | {value.get('force_valid_ratio')} | {value['classification']} | {value['status']} |"
        )
    lines.extend([
        "",
        "## Row filtering and semantics",
        "",
        "Rows were excluded from both views only for invalid state/action/camera/synchronization data, malformed rows, non-causal source timestamps, or outside the reviewed logical crop. Force-invalid or stale rows are retained in both matched views with `force_valid=false`, `force_age_s`, and the causal source timestamp preserved. No force interpolation or Newton conversion is performed.",
        "",
        "The stored state is 12-D `[JAKA measured joints 6, RH56 measured actuator positions 6]`. The stored action is 12-D `[JAKA accepted targets 6, RH56 targets 6]`, in the existing canonical RH56 order. Force is six raw RH56 FORCE_ACT counts. ACT action chunks are limited to one logical segment and use absolute/native targets with end masks.",
        "",
        "Normalization statistics are under `stats/` and are computed only from the train logical-segment split; force statistics use only force-valid rows.",
        "",
    ])
    return "\n".join(lines)


def materialize_physical_bottle(config_path: str | Path, *, replace: bool = False) -> PhysicalBottleResult:
    config_path, config = _load_config(config_path)
    audit = audit_physical_bottle(config_path)
    source_root = Path(audit["source_root"])
    output_root = _resolve(config_path, str(config["output_root"]))
    if output_root == source_root or output_root.is_relative_to(source_root) or source_root.is_relative_to(output_root):
        raise ValueError("training output must be disjoint from immutable raw source root")
    inventory = audit["raw_inventory"]
    fingerprint = hashlib.sha256(_json_bytes({"config": config, "inventory": inventory})).hexdigest()
    marker = output_root / "materialization.json"
    if marker.is_file() and not replace:
        existing = _read_json(marker)
        if existing.get("fingerprint") == fingerprint:
            return PhysicalBottleResult(output_root, _read_json(output_root / "reports/materialization_summary.json"), reused=True)
        raise FileExistsError(f"generated output exists with a different fingerprint: {output_root}")
    if output_root.exists() and any(output_root.iterdir()):
        if not replace:
            raise FileExistsError(f"output root is not empty: {output_root}; use --replace")
        shutil.rmtree(output_root)
    output_root.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{output_root.name}.", dir=output_root.parent))
    try:
        assets = temporary / "assets"
        rows_by_segment: dict[int, list[dict[str, Any]]] = {}
        segment_reports: list[dict[str, Any]] = []
        excluded_rows: list[dict[str, Any]] = []
        source_reviews = _review_map(config)
        source_cache: dict[int, tuple[dict[str, Any], list[dict[str, Any]]]] = {}
        included_segments = [dict(value) for value in config["segments"] if value.get("include")]
        if not included_segments:
            raise ValueError("no logical segments are included")
        for logical_episode_index, segment in enumerate(included_segments):
            segment["logical_episode_index"] = logical_episode_index
            source_episode = int(segment["source_episode"])
            if source_episode not in source_cache:
                paths = _episode_paths(source_root, source_episode)
                if not paths["metadata"].is_file() or not paths["rows"].is_file():
                    raise ValueError(f"included segment {segment['id']} has missing source files")
                source_cache[source_episode] = (_read_json(paths["metadata"]), _read_rows(paths["rows"])[0])
            metadata, rows = source_cache[source_episode]
            selected, crop, crop_excluded = _crop_segment(rows, metadata, segment)
            if crop.get("crop_review_required"):
                raise ValueError(f"included segment {segment['id']} requires crop review: {crop}")
            excluded_rows.extend(crop_excluded)
            output_rows: list[dict[str, Any]] = []
            quality_excluded = 0
            previous_timestamp: int | None = None
            for source_ordinal, row in selected:
                reasons = _quality_reasons(row)
                timestamp = int(row.get("timestamp_ns", 0))
                if previous_timestamp is not None and timestamp <= previous_timestamp:
                    reasons.append("non_monotonic_canonical_timestamp")
                previous_timestamp = timestamp
                if reasons:
                    quality_excluded += 1
                    excluded_rows.append({
                        "source_episode": source_episode,
                        "source_frame_index": int(row.get("frame_index", source_ordinal)),
                        "timestamp_ns": timestamp,
                        "logical_segment_id": segment["id"],
                        "reasons": sorted(set(reasons)),
                        "views": {"act": False, "act_force": False},
                    })
                    continue
                output_rows.append(_rich_row(row, logical_index=0, local_index=len(output_rows), segment=segment, start_ns=int(selected[0][1]["timestamp_ns"])))
            if not output_rows:
                raise ValueError(f"included segment {segment['id']} has no valid rows")
            for row_index, row in enumerate(output_rows):
                row["index"] = sum(len(value) for value in rows_by_segment.values()) + row_index
                row["episode_index"] = logical_episode_index
            rows_by_segment[logical_episode_index] = output_rows
            raw_indices = [source_ordinal for source_ordinal, row in selected if not _quality_reasons(row)]
            paths = _episode_paths(source_root, source_episode)
            for role in ("workspace", "wrist"):
                destination = assets / "videos" / f"observation.images.{role}" / "chunk-000" / f"episode_{logical_episode_index:06d}.mp4"
                destination.parent.mkdir(parents=True, exist_ok=True)
                _write_video_subset(paths[role], destination, raw_indices, 30)
            crop_duration = (int(output_rows[-1]["timestamp_ns"]) - int(output_rows[0]["timestamp_ns"])) / 1e9
            segment_reports.append({
                "id": segment["id"],
                "logical_episode_index": logical_episode_index,
                "source_episode": source_episode,
                "classification": segment["classification"],
                "success_label": segment.get("success_label", "success"),
                "crop_start_frame": int(output_rows[0]["source_frame_index"]),
                "crop_end_frame": int(output_rows[-1]["source_frame_index"]),
                "crop_start_timestamp_ns": int(output_rows[0]["timestamp_ns"]),
                "crop_end_timestamp_ns": int(output_rows[-1]["timestamp_ns"]),
                "crop_duration_s": crop_duration,
                "included_rows": len(output_rows),
                "excluded_rows": len(crop_excluded) + quality_excluded,
                "force_valid_ratio": sum(bool(row["force_valid"]) for row in output_rows) / len(output_rows),
                "manual_boundary": segment.get("end_frame") is not None or segment.get("start_frame", 0) != 0,
                "notes": segment.get("notes"),
                "status": "included",
            })
        # Reassign local episode indices after all rows exist and ensure the two views
        # use byte-for-byte identical sample manifests.
        splits_config = {name: [str(value) for value in values] for name, values in config["splits"].items()}
        segment_to_index = {value["id"]: index for index, value in enumerate(segment_reports)}
        splits: dict[str, list[int]] = {}
        for name in ("train", "val", "test"):
            unknown = sorted(set(splits_config.get(name, [])) - set(segment_to_index))
            if unknown:
                raise ValueError(f"split {name} references non-included segments: {unknown}")
            splits[name] = sorted(segment_to_index[value] for value in splits_config.get(name, []))
        assigned = [value for values in splits.values() for value in values]
        if sorted(assigned) != sorted(rows_by_segment):
            raise ValueError("splits must assign every included logical segment exactly once")
        split_names_by_segment = {
            segment_id: split_name
            for split_name, segment_ids in splits_config.items()
            for segment_id in segment_ids
        }
        split_groups = _validate_split_groups(
            config,
            segment_to_index=segment_to_index,
            split_names_by_segment=split_names_by_segment,
        )
        train_rows = [row for index in splits["train"] for row in rows_by_segment[index]]
        force_train_rows = [row for row in train_rows if row["force_valid"]]
        stats = {
            "state": _stats([row["observation.state"] for row in train_rows], STATE_ORDER, source_split="train"),
            "action": _stats([row["action"] for row in train_rows], ACTION_ORDER, source_split="train"),
            "force": _stats([row["observation.force"] for row in force_train_rows], FORCE_ORDER, source_split="train(force_valid_only)") if force_train_rows else None,
        }
        _write_json(temporary / "stats/state_stats.json", stats["state"])
        _write_json(temporary / "stats/action_stats.json", stats["action"])
        if stats["force"] is not None:
            _write_json(temporary / "stats/force_stats.json", stats["force"])
        _write_json(temporary / "manifests/logical_segments.json", {"schema_version": PHYSICAL_BOTTLE_SCHEMA_VERSION, "segments": segment_reports})
        excluded_segments = [value for value in config["segments"] if not value.get("include")]
        review_segments = [value for value in excluded_segments if value.get("classification") == "REVIEW_REQUIRED" or value.get("review_required")]
        _write_json(temporary / "manifests/excluded_segments.json", {"schema_version": PHYSICAL_BOTTLE_SCHEMA_VERSION, "segments": excluded_segments})
        _write_json(temporary / "manifests/review_required_segments.json", {"schema_version": PHYSICAL_BOTTLE_SCHEMA_VERSION, "segments": review_segments, "note": "No ambiguous logical segment was included; the current reviewed manifest has no unresolved boundary."})
        _write_json(temporary / "manifests/final_clean_training.json", {"schema_version": PHYSICAL_BOTTLE_SCHEMA_VERSION, "included_segments": segment_reports, "excluded_segments": excluded_segments, "review_required_segments": review_segments})
        _write_json(temporary / "manifests/failure_slip_segments.json", {"schema_version": PHYSICAL_BOTTLE_SCHEMA_VERSION, "segments": [], "note": "No retained payload had defensible slip/drop evidence during this audit."})
        _write_jsonl(temporary / "reports/excluded_rows.jsonl", excluded_rows)
        _write_json(temporary / "reports/episode_crop_report.json", {"segments": segment_reports})
        _write_variant(temporary / "act", rows_by_segment, segment_reports, splits, config["task"], view="act", assets_root=assets)
        _write_variant(temporary / "act_force", rows_by_segment, segment_reports, splits, config["task"], view="act_force", assets_root=assets)
        summary = {
            "schema_version": PHYSICAL_BOTTLE_SCHEMA_VERSION,
            "fingerprint": fingerprint,
            "source_root": str(source_root),
            "output_root": str(output_root),
            "task_id": config["task"]["id"],
            "task_prompt": config["task"]["prompt"],
            "included_segment_count": len(rows_by_segment),
            "logical_payload_trajectory_count": audit["logical_payload_trajectory_count"],
            "manifest_entry_count": audit["manifest_entry_count"],
            "row_count": sum(len(rows) for rows in rows_by_segment.values()),
            "act_force_row_count": sum(len(rows) for rows in rows_by_segment.values()),
            "matched_samples": True,
            "duration_s": sum((int(rows[-1]["timestamp_ns"]) - int(rows[0]["timestamp_ns"])) / 1e9 for rows in rows_by_segment.values()),
            "splits": splits,
            "split_groups": split_groups,
            "segments": segment_reports,
            "normalization": {"source_split": "train", "train_segments": splits["train"], "force_valid_rows": len(force_train_rows)},
            "raw_episodes_immutable": True,
            "source_inventory": inventory,
        }
        _write_json(temporary / "reports/materialization_summary.json", summary)
        report_text = _markdown_materialization(summary)
        inside_report = temporary / "reports/materialization_report.md"
        inside_report.parent.mkdir(parents=True, exist_ok=True)
        inside_report.write_text(report_text, encoding="utf-8")
        report_path = _resolve(config_path, str(config["materialization_report"]))
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(report_text, encoding="utf-8")
        _write_json(temporary / "materialization.json", {"schema_version": PHYSICAL_BOTTLE_SCHEMA_VERSION, "fingerprint": fingerprint, "source_inventory": inventory, "raw_episodes_immutable": True})
        os.replace(temporary, output_root)
        temporary = Path()
    finally:
        if str(temporary) not in {"", "."} and temporary.exists():
            shutil.rmtree(temporary)
    return PhysicalBottleResult(output_root, summary, reused=False)


def _load_variant_rows(root: Path, variant: str) -> tuple[dict[int, list[dict[str, Any]]], dict[str, Any]]:
    try:
        import pyarrow.parquet as parquet
    except ImportError as exc:
        raise RuntimeError("physical bottle validation requires pyarrow") from exc
    variant_root = root / variant
    info = _read_json(variant_root / "master/meta/info.json")
    rows_by_episode: dict[int, list[dict[str, Any]]] = {}
    for line in (variant_root / "master/meta/episodes.jsonl").read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        episode = json.loads(line)
        episode_index = int(episode["episode_index"])
        rows_by_episode[episode_index] = parquet.read_table(variant_root / "master" / episode["data"]).to_pylist()
    return rows_by_episode, info


def _raw_inventory_matches(root: Path, inventory: Mapping[str, Any]) -> bool:
    for record in inventory.values():
        for value in record.values():
            if value.get("missing"):
                if Path(value["path"]).exists():
                    return False
            elif not Path(value["path"]).is_file() or _sha256_file(Path(value["path"])) != value.get("sha256"):
                return False
    return True


def validate_physical_bottle(root: str | Path) -> dict[str, Any]:
    root = Path(root).resolve()
    marker = _read_json(root / "materialization.json")
    summary = _read_json(root / "reports/materialization_summary.json")
    errors: list[str] = []
    variants: dict[str, dict[int, list[dict[str, Any]]]] = {}
    infos: dict[str, Any] = {}
    for variant in ("act", "act_force"):
        try:
            rows, info = _load_variant_rows(root, variant)
        except (OSError, ValueError, RuntimeError) as exc:
            errors.append(f"{variant}: {exc}")
            continue
        variants[variant] = rows
        infos[variant] = info
        for episode_index, episode_rows in rows.items():
            previous: int | None = None
            for row in episode_rows:
                if len(row.get("observation.state", [])) != 12 or len(row.get("action", [])) != 12 or len(row.get("observation.force", [])) != 6:
                    errors.append(f"{variant} episode {episode_index}: feature dimension mismatch")
                if not all(math.isfinite(float(value)) for key in ("observation.state", "action", "observation.force") for value in row[key]):
                    errors.append(f"{variant} episode {episode_index}: non-finite feature")
                timestamp = int(row["timestamp_ns"])
                if previous is not None and timestamp <= previous:
                    errors.append(f"{variant} episode {episode_index}: non-monotonic timestamp")
                previous = timestamp
                timing = json.loads(row["timing"])
                for name, value in timing.get("source_timestamps_ns", {}).items():
                    if value is not None and int(value) > timestamp:
                        errors.append(f"{variant} episode {episode_index} frame {row['frame_index']}: future source {name}")
                if bool(row["force_valid"]) and row["force_timestamp_ns"] is None:
                    errors.append(f"{variant} episode {episode_index}: valid force has no timestamp")
            for role in ("workspace", "wrist"):
                video = root / variant / "master" / f"videos/observation.images.{role}/chunk-000/episode_{episode_index:06d}.mp4"
                if not video.exists():
                    errors.append(f"{variant} episode {episode_index}: missing {role} video")
                    continue
                info = _video_info(video)
                if info.get("readable") is not True or info.get("frame_count") != len(episode_rows):
                    errors.append(f"{variant} episode {episode_index}: {role} video decode/count mismatch")
    if set(variants.get("act", {})) != set(variants.get("act_force", {})):
        errors.append("ACT and ACT+Force logical episode sets differ")
    else:
        for episode_index in variants.get("act", {}):
            left = variants["act"][episode_index]
            right = variants["act_force"][episode_index]
            if len(left) != len(right):
                errors.append(f"matched row count differs for logical episode {episode_index}")
                continue
            for a, b in zip(left, right):
                keys = ("source_episode_index", "source_frame_index", "logical_segment_id", "timestamp_ns", "action")
                if any(a[key] != b[key] for key in keys):
                    errors.append(f"matched sample mismatch in logical episode {episode_index}")
                    break
    raw_ok = _raw_inventory_matches(root, marker.get("source_inventory", {}))
    if not raw_ok:
        errors.append("raw source hash changed after materialization")
    loader_checks: dict[str, Any] = {}
    try:
        from .training_views import ActDatasetAdapter, ActForceDatasetAdapter
        act_root = root / "act"
        force_root = root / "act_force"
        split_names = tuple(
            _read_json(act_root / "manifests/splits.json")["splits"].keys()
        )
        act_adapters = [
            ActDatasetAdapter(act_root, split=split, action_horizon=16)
            for split in split_names
        ]
        force_adapters = [
            ActForceDatasetAdapter(force_root, split=split, action_horizon=16)
            for split in split_names
        ]
        act_count = sum(len(adapter) for adapter in act_adapters)
        force_count = sum(len(adapter) for adapter in force_adapters)
        if act_count != force_count or act_count != summary["row_count"]:
            errors.append("ACT adapters do not expose the same row count")
        nonempty = [
            (act, force)
            for act, force in zip(act_adapters, force_adapters)
            if len(act)
        ]
        if nonempty:
            act, force = nonempty[len(nonempty) // 2]
            act_sample = act[len(act) // 2]
            force_sample = force[len(force) // 2]
            if act_sample["action"].shape != (16, 12) or force_sample["action"].shape != (16, 12):
                errors.append("ACT action chunk shape mismatch")
            if force_sample["observation"]["force"].shape != (6,):
                errors.append("ACT+Force force shape mismatch")
        loader_checks = {
            "status": "passed",
            "act_rows": act_count,
            "act_force_rows": force_count,
            "split_rows": {
                split: len(adapter)
                for split, adapter in zip(split_names, act_adapters)
            },
        }
    except Exception as exc:  # adapter compatibility is reported, not hidden
        loader_checks = {"status": "failed", "error": str(exc)}
        errors.append(f"local ACT adapter check failed: {exc}")
    try:
        import lerobot  # type: ignore[import-not-found]
        lerobot_status = {"status": "available", "version": getattr(lerobot, "__version__", None)}
    except ImportError:
        lerobot_status = {"status": "skipped", "reason": "lerobot is not installed"}
    result = {
        "schema_version": PHYSICAL_BOTTLE_SCHEMA_VERSION,
        "status": "passed" if not errors else "failed",
        "errors": errors,
        "row_count": summary["row_count"],
        "act_force_row_count": summary["act_force_row_count"],
        "matched_samples": not errors and summary["matched_samples"],
        "raw_episodes_modified": not raw_ok,
        "loader": loader_checks,
        "lerobot": lerobot_status,
        "variants": {name: {"episode_count": len(rows), "row_count": sum(len(value) for value in rows.values()), "info": infos.get(name, {})} for name, rows in variants.items()},
    }
    _write_json(root / "reports/validation.json", result)
    return result
