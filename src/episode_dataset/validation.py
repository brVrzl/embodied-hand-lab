from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any, Iterable, Mapping
from uuid import UUID

import numpy as np

from embodiment_core.robot_limits import JAKA_MINI2_JOINT_LIMITS_RAD

from .episode import (
    ACTION_ORDER,
    OBSERVATION_STATE_ORDER,
    SCHEMA_VERSION,
    PHYSICAL_SCHEMA_VERSION,
    SUPPORTED_SCHEMA_VERSIONS,
    file_sha256,
)

SUCCESS_LABELS = frozenset({"unlabeled", "success", "failure"})
TRAINING_SUCCESS_LABELS = frozenset({"success", "failure"})
RAW_EPISODE_FORMAT_VERSION = "raw_episode_v1"


def load_canonical_rows(episode: str | Path) -> tuple[list[dict[str, Any]], list[str]]:
    """Load canonical JSONL without accepting a truncated or malformed row."""

    episode_dir = Path(episode).resolve()
    sample_path = episode_dir / "canonical" / "samples.jsonl"
    errors: list[str] = []
    rows: list[dict[str, Any]] = []
    if not sample_path.is_file():
        return rows, ["missing canonical/samples.jsonl"]
    try:
        with sample_path.open(encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, 1):
                if not line.strip():
                    errors.append(f"samples.jsonl:{line_number}: blank row")
                    continue
                try:
                    value = json.loads(line)
                except json.JSONDecodeError as exc:
                    errors.append(f"samples.jsonl:{line_number}: {exc}")
                    continue
                if not isinstance(value, dict):
                    errors.append(
                        f"samples.jsonl:{line_number}: row must be a JSON object"
                    )
                    continue
                rows.append(value)
    except OSError as exc:
        errors.append(f"cannot read canonical/samples.jsonl: {exc}")
    return rows, errors


def load_data_quality_rows(episode: str | Path) -> tuple[list[dict[str, Any]], list[str]]:
    """Load metadata-only canonical slots without treating them as training rows."""

    path = Path(episode).resolve() / "raw" / "data_quality.jsonl"
    rows: list[dict[str, Any]] = []
    errors: list[str] = []
    if not path.exists():
        return rows, errors
    try:
        with path.open(encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, 1):
                if not line.strip():
                    continue
                try:
                    value = json.loads(line)
                except json.JSONDecodeError as exc:
                    errors.append(f"raw/data_quality.jsonl:{line_number}: {exc}")
                    continue
                if not isinstance(value, dict):
                    errors.append(
                        f"raw/data_quality.jsonl:{line_number}: row must be an object"
                    )
                    continue
                rows.append(value)
    except OSError as exc:
        errors.append(f"cannot read raw/data_quality.jsonl: {exc}")
    return rows, errors


def _json_object(path: Path, label: str, errors: list[str]) -> dict[str, Any]:
    if not path.is_file():
        errors.append(f"missing {label}")
        return {}
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        errors.append(f"invalid {label}: {exc}")
        return {}
    if not isinstance(value, dict):
        errors.append(f"invalid {label}: root must be a JSON object")
        return {}
    return value


def _finite_vector(
    value: object,
    *,
    length: int,
    label: str,
    errors: list[str],
) -> tuple[float, ...] | None:
    if not isinstance(value, list) or len(value) != length:
        errors.append(f"{label}: expected {length} values")
        return None
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        errors.append(f"{label}: contains a non-numeric value")
        return None
    if not all(math.isfinite(item) for item in result):
        errors.append(f"{label}: contains NaN or Inf")
        return None
    return result


def _inside_episode(
    episode_dir: Path, relative: object, label: str, errors: list[str]
) -> Path | None:
    if not isinstance(relative, str) or not relative:
        errors.append(f"{label}: missing relative path")
        return None
    candidate = (episode_dir / relative).resolve()
    try:
        candidate.relative_to(episode_dir)
    except ValueError:
        errors.append(f"{label}: path escapes episode directory")
        return None
    if not candidate.is_file():
        errors.append(f"{label}: file does not exist: {relative}")
        return None
    return candidate


def _load_array(
    path: Path,
    *,
    dtype: np.dtype[Any],
    dimensions: int,
    channels: int | None,
    label: str,
    errors: list[str],
) -> np.ndarray | None:
    try:
        value = np.load(path, allow_pickle=False)
    except (OSError, ValueError) as exc:
        errors.append(f"{label}: unreadable NPY: {exc}")
        return None
    if value.dtype != dtype or value.ndim != dimensions:
        errors.append(
            f"{label}: expected {np.dtype(dtype).name}/{dimensions}D, "
            f"got {value.dtype}/{value.ndim}D"
        )
        return None
    if channels is not None and value.shape[-1] != channels:
        errors.append(f"{label}: expected {channels} channels")
        return None
    return value


def _raw_jsonl_errors(episode_dir: Path) -> list[str]:
    errors: list[str] = []
    raw = episode_dir / "raw"
    if not raw.is_dir():
        errors.append("missing raw directory")
        return errors
    for path in sorted(raw.glob("*.jsonl")):
        try:
            with path.open(encoding="utf-8") as handle:
                for line_number, line in enumerate(handle, 1):
                    try:
                        value = json.loads(line)
                    except json.JSONDecodeError as exc:
                        errors.append(
                            f"raw/{path.name}:{line_number}: invalid JSON: {exc}"
                        )
                        continue
                    if not isinstance(value, dict):
                        errors.append(
                            f"raw/{path.name}:{line_number}: row is not an object"
                        )
        except OSError as exc:
            errors.append(f"cannot read raw/{path.name}: {exc}")
    return errors


def _physical_v2_raw_errors(episode_dir: Path) -> list[str]:
    errors: list[str] = []
    for stream in ("jaka_state", "rh56_feedback"):
        path = episode_dir / "raw" / f"{stream}.jsonl"
        try:
            payload = path.read_text(encoding="utf-8")
        except OSError:
            payload = ""
        if not payload.strip():
            errors.append(f"physical v2 requires non-empty raw/{stream}.jsonl")
            continue
        for line_number, line in enumerate(payload.splitlines(), 1):
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            label = f"raw/{stream}.jsonl:{line_number}"
            if not isinstance(row, dict):
                continue
            if stream == "jaka_state":
                for name in (
                    "read_host_monotonic_ns",
                    "record_host_monotonic_ns",
                    "command_host_monotonic_ns",
                ):
                    if not isinstance(row.get(name), int):
                        errors.append(f"{label}: {name} must be int")
                for name, length in (
                    ("accepted_joint_target_rad", 6),
                    ("measured_joint_position_rad", 6),
                    ("estimated_joint_velocity_rad_s", 6),
                    ("commanded_tcp_pose_xyzw", 7),
                ):
                    _finite_vector(
                        row.get(name), length=length, label=f"{label}.{name}", errors=errors
                    )
                continue
            action = row.get("action", {})
            hand_target = action.get("hand_target") if isinstance(action, dict) else None
            if hand_target is not None:
                _finite_vector(
                    hand_target,
                    length=6,
                    label=f"{label}.action.hand_target",
                    errors=errors,
                )
            timestamps = row.get("hand_feedback_register_timestamps_ns", {})
            registers = row.get("rh56_registers", {})
            if hand_target is not None:
                if not isinstance(row.get("hand_command_timestamp"), int):
                    errors.append(f"{label}: hand_command_timestamp must be int when a target was issued")
            elif row.get("hand_command_timestamp") is not None:
                errors.append(
                    f"{label}: hand_command_timestamp must be null when no target was issued"
                )
            if not isinstance(row.get("hand_feedback_timestamp"), int):
                errors.append(f"{label}: hand_feedback_timestamp must be int")
            for name in ("ANGLE_ACT", "CURRENT", "FORCE_ACT", "ERROR", "STATUS"):
                if not isinstance(timestamps, dict) or not isinstance(
                    timestamps.get(name), int
                ):
                    errors.append(f"{label}: missing {name} read timestamp")
                _finite_vector(
                    registers.get(name) if isinstance(registers, dict) else None,
                    length=6,
                    label=f"{label}.rh56_registers.{name}",
                    errors=errors,
                )
    return errors


def _validate_calibration(
    episode_dir: Path, metadata: Mapping[str, Any], errors: list[str]
) -> None:
    snapshot = metadata.get("calibration_snapshot", {})
    if not isinstance(snapshot, dict):
        errors.append("metadata.calibration_snapshot must be an object")
        return
    files = snapshot.get("files", [])
    if not isinstance(files, list):
        errors.append("metadata.calibration_snapshot.files must be a list")
        return
    for index, record in enumerate(files):
        label = f"calibration_snapshot.files[{index}]"
        if not isinstance(record, dict):
            errors.append(f"{label}: must be an object")
            continue
        path = _inside_episode(episode_dir, record.get("path"), label, errors)
        expected = record.get("sha256")
        if path is not None and isinstance(expected, str):
            actual = file_sha256(path)
            if actual != expected:
                errors.append(f"{label}: checksum mismatch")


def _validate_schema_metadata(
    metadata: Mapping[str, Any], errors: list[str], warnings: list[str]
) -> None:
    schema_version = metadata.get("schema_version")
    if schema_version not in SUPPORTED_SCHEMA_VERSIONS:
        errors.append(
            f"unsupported schema_version: {metadata.get('schema_version')!r}"
        )
    if metadata.get("action_order") != list(ACTION_ORDER):
        errors.append("metadata.action_order does not match canonical order")
    if metadata.get("observation_state_order") != list(OBSERVATION_STATE_ORDER):
        errors.append(
            "metadata.observation_state_order does not match canonical order"
        )
    episode_uuid = metadata.get("episode_uuid")
    if not isinstance(episode_uuid, str):
        errors.append("metadata.episode_uuid must be a canonical UUID string")
    else:
        try:
            parsed_uuid = UUID(episode_uuid)
        except ValueError:
            errors.append("metadata.episode_uuid must be a canonical UUID string")
        else:
            if str(parsed_uuid) != episode_uuid:
                errors.append(
                    "metadata.episode_uuid must use lowercase hyphenated UUID form"
                )
    fps = metadata.get("dataset_fps")
    if not isinstance(fps, int) or fps <= 0:
        errors.append("metadata.dataset_fps must be a positive integer")
    if metadata.get("finalized") is not True:
        errors.append("episode is not finalized")
    status = metadata.get("completion_status")
    if status not in {"completed", "aborted", "invalid"}:
        errors.append(f"invalid completion_status: {status!r}")
    elif status != "completed":
        warnings.append(f"episode completion_status is {status!r}")
    success_label = metadata.get("success_label")
    if success_label not in SUCCESS_LABELS:
        errors.append(
            "metadata.success_label must be 'unlabeled', 'success', or 'failure'"
        )
    elif success_label == "unlabeled":
        warnings.append(
            "episode success_label is 'unlabeled'; explicit success/failure "
            "review is required for training"
        )
    units = metadata.get("units", {})
    expected_hand_unit = (
        "normalized_closure_0_to_1"
        if schema_version == PHYSICAL_SCHEMA_VERSION
        else "rad"
    )
    if not isinstance(units, dict) or units.get("hand") != expected_hand_unit:
        errors.append(
            f"{schema_version} requires metadata.units.hand="
            f"{expected_hand_unit!r}"
        )
    if not isinstance(metadata.get("notes", ""), str):
        errors.append("metadata.notes must be a string")
    failure_stage = metadata.get("failure_stage")
    if failure_stage is not None and not isinstance(failure_stage, str):
        errors.append("metadata.failure_stage must be null or a string")
    if success_label == "success" and failure_stage is not None:
        errors.append("successful episode must not declare failure_stage")
    if success_label == "failure" and not failure_stage:
        warnings.append("failed episode has no failure_stage")


def _raw_video_frame_count(path: Path, errors: list[str]) -> int | None:
    """Count decodable frames without trusting a container header alone."""

    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - environment dependent
        errors.append(f"{path.name}: OpenCV is required for deep raw-episode validation: {exc}")
        return None
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        errors.append(f"{path.name}: video cannot be opened")
        return None
    count = 0
    try:
        while True:
            ok, _ = capture.read()
            if not ok:
                break
            count += 1
    finally:
        capture.release()
    return count


def _validate_raw_episode(
    episode_dir: Path,
    *,
    deep: bool,
) -> dict[str, Any]:
    """Validate the compact physical RGB/Parquet episode format."""

    errors: list[str] = []
    warnings: list[str] = []
    metadata = _json_object(episode_dir / "episode.json", "episode.json", errors)
    if metadata.get("format_version") != RAW_EPISODE_FORMAT_VERSION:
        errors.append(
            "episode.json.format_version must be 'raw_episode_v1'"
        )
    episode_uuid = metadata.get("episode_uuid")
    if not isinstance(episode_uuid, str):
        errors.append("episode.json.episode_uuid must be a UUID string")
    else:
        try:
            UUID(episode_uuid)
        except ValueError:
            errors.append("episode.json.episode_uuid must be a UUID string")
    fps = metadata.get("fps")
    if not isinstance(fps, int) or fps <= 0:
        errors.append("episode.json.fps must be a positive integer")
        fps = 0
    status = metadata.get("completion_status")
    if status not in {"completed", "aborted", "invalid"}:
        errors.append(f"invalid completion_status: {status!r}")
    if metadata.get("quest_recorded") is not False:
        errors.append("raw_episode_v1 must not record Quest data")
    if metadata.get("depth_recorded") is not False:
        errors.append("raw_episode_v1 must not record depth data")
    if metadata.get("tcp_recorded") is not False:
        errors.append("raw_episode_v1 must not record TCP in the core table")
    success_label = metadata.get("success_label")
    if success_label not in SUCCESS_LABELS:
        errors.append(
            "episode.json.success_label must be 'unlabeled', 'success', or 'failure'"
        )
    elif success_label == "unlabeled":
        warnings.append("episode success_label is 'unlabeled'; review is required")
    frame_count = metadata.get("num_frames")
    if not isinstance(frame_count, int) or frame_count < 0:
        errors.append("episode.json.num_frames must be a non-negative integer")
        frame_count = 0
    files = metadata.get("files", {})
    if not isinstance(files, dict):
        errors.append("episode.json.files must be an object")
        files = {}

    frame_path: Path | None = None
    video_paths: dict[str, Path | None] = {}
    if frame_count > 0 or status == "completed":
        frame_path = _inside_episode(episode_dir, files.get("frames"), "files.frames", errors)
        for name in ("fixed_camera", "wrist_camera"):
            video_paths[name] = _inside_episode(
                episode_dir, files.get(name), f"files.{name}", errors
            )
    quality_path: Path | None = None
    if files.get("quality") is not None:
        quality_path = _inside_episode(episode_dir, files.get("quality"), "files.quality", errors)

    row_count = 0
    timing_invalid_count = 0
    held_rejected_count = 0
    previous_timestamp: int | None = None
    timing_gap_count = 0
    if frame_path is not None and frame_path.is_file():
        if not deep:
            row_count = frame_count
        else:
            try:
                import pyarrow.parquet as parquet
                table = parquet.read_table(frame_path)
            except (ImportError, OSError, ValueError) as exc:
                errors.append(f"frames.parquet: cannot read: {exc}")
            else:
                expected_columns = [
                    "frame_index",
                    "timestamp_ns",
                    "observation.state",
                    "action",
                ]
                if table.column_names != expected_columns:
                    errors.append(
                        "frames.parquet columns do not match the raw episode contract"
                    )
                row_count = table.num_rows
                if row_count != frame_count:
                    errors.append(
                        f"episode.json.num_frames {frame_count} does not match "
                        f"frames.parquet rows {row_count}"
                    )
                state_type = table.column("observation.state").type if "observation.state" in table.column_names else None
                action_type = table.column("action").type if "action" in table.column_names else None
                if getattr(state_type, "list_size", None) != 12:
                    errors.append("frames.parquet observation.state must be a fixed list of 12")
                if getattr(action_type, "list_size", None) != 12:
                    errors.append("frames.parquet action must be a fixed list of 12")
                if set(expected_columns).issubset(table.column_names):
                    frame_indices = table.column("frame_index").to_pylist()
                    timestamps = table.column("timestamp_ns").to_pylist()
                    states = table.column("observation.state").to_pylist()
                    actions = table.column("action").to_pylist()
                    period_ns = round(1_000_000_000 / fps) if fps else 0
                    for index, (frame_index, timestamp, state, action) in enumerate(
                        zip(frame_indices, timestamps, states, actions, strict=True)
                    ):
                        label = f"frames.parquet row {index}"
                        if frame_index != index:
                            errors.append(f"{label}: frame_index is not contiguous")
                        if not isinstance(timestamp, int):
                            errors.append(f"{label}: timestamp_ns must be int")
                        elif previous_timestamp is not None:
                            delta = timestamp - previous_timestamp
                            if delta <= 0:
                                errors.append(f"{label}: timestamp_ns is not strictly increasing")
                            elif period_ns and abs(delta - period_ns) > 2_000_000:
                                warnings.append(
                                    f"{label}: timestamp spacing differs from the nominal period"
                                )
                        if isinstance(timestamp, int):
                            previous_timestamp = timestamp
                        for name, values in (("observation.state", state), ("action", action)):
                            finite = _finite_vector(values, length=12, label=f"{label}.{name}", errors=errors)
                            if finite is None:
                                continue
                            for joint, (value, (lower, upper)) in enumerate(
                                zip(finite[:6], JAKA_MINI2_JOINT_LIMITS_RAD, strict=True), 1
                            ):
                                if value < lower or value > upper:
                                    errors.append(
                                        f"{label}.{name}: J{joint} outside manufacturer boundary"
                                    )
                            if any(value < 0.0 or value > 1.0 for value in finite[6:]):
                                errors.append(f"{label}.{name}: hand values must be in [0, 1]")
    elif frame_count > 0 or status == "completed":
        errors.append("frames.parquet is required for a non-empty/completed raw episode")

    if deep:
        for name, path in video_paths.items():
            if path is None:
                continue
            actual = _raw_video_frame_count(path, errors)
            if actual is not None and actual != row_count:
                errors.append(
                    f"{name}.mp4 has {actual} decodable frames, expected {row_count}"
                )

        if quality_path is not None and quality_path.is_file():
            try:
                import pyarrow.parquet as parquet
                quality = parquet.read_table(quality_path)
            except (ImportError, OSError, ValueError) as exc:
                errors.append(f"quality.parquet: cannot read: {exc}")
            else:
                required = {
                    "frame_index",
                    "action_status",
                    "timing_valid",
                    "arm_trigger",
                    "hand_grip",
                    "nominal_slot_index",
                    "missed_slots_before",
                    "missed_slots_after",
                }
                if not required.issubset(quality.column_names):
                    errors.append("quality.parquet is missing required quality columns")
                if quality.num_rows != row_count:
                    errors.append("quality.parquet row count does not match frames.parquet")
                if "timing_valid" in quality.column_names:
                    timing_valid = quality.column("timing_valid").to_pylist()
                    timing_invalid_count = sum(value is False for value in timing_valid)
                if "action_status" in quality.column_names:
                    held_rejected_count = sum(
                        value == "held_rejected"
                        for value in quality.column("action_status").to_pylist()
                    )
                if "missed_slots_before" in quality.column_names:
                    timing_gap_count += sum(
                        int(value or 0)
                        for value in quality.column("missed_slots_before").to_pylist()
                    )
                if "missed_slots_after" in quality.column_names:
                    timing_gap_count += sum(
                        int(value or 0)
                        for value in quality.column("missed_slots_after").to_pylist()
                    )
    quality_metadata = metadata.get("quality", {})
    if isinstance(quality_metadata, dict):
        expected_timing_invalid = quality_metadata.get("timing_invalid_frame_count")
        if isinstance(expected_timing_invalid, int) and expected_timing_invalid != timing_invalid_count:
            errors.append("episode.json quality timing-invalid count does not match quality.parquet")
        expected_held = quality_metadata.get("held_rejected_frame_count")
        if isinstance(expected_held, int) and expected_held != held_rejected_count:
            errors.append("episode.json quality held-rejected count does not match quality.parquet")
    if frame_count != row_count and (deep or frame_count == 0):
        errors.append("episode.json.num_frames does not match the table row count")
    valid = not errors
    training_eligible = bool(
        valid
        and status == "completed"
        and success_label in TRAINING_SUCCESS_LABELS
        and row_count > 0
        and timing_invalid_count == 0
        and held_rejected_count == 0
        and timing_gap_count == 0
    )
    return {
        "schema_version": RAW_EPISODE_FORMAT_VERSION,
        "format_version": RAW_EPISODE_FORMAT_VERSION,
        "episode": str(episode_dir),
        "episode_uuid": metadata.get("episode_uuid"),
        "completion_status": status,
        "success_label": success_label,
        "sample_count": row_count,
        "valid": valid,
        "training_eligible": training_eligible,
        "deep_validation": deep,
        "physically_validated": False,
        "errors": errors,
        "warnings": warnings,
        "quality": {
            "timing_invalid_frame_count": timing_invalid_count,
            "held_rejected_frame_count": held_rejected_count,
            "timing_gap_count": timing_gap_count,
            "quest_recorded": False,
            "depth_recorded": False,
        },
    }


def validate_episode(
    episode: str | Path,
    *,
    deep: bool = True,
) -> dict[str, Any]:
    """Validate one finalized episode without touching hardware.

    ``valid`` means the archive is structurally readable. ``training_eligible``
    is stricter: it also requires a completed episode, an explicit
    success/failure label, and no canonical timing gaps. Repeated camera
    payloads are reported as warnings because a static scene can legitimately
    produce identical frames.
    """

    episode_dir = Path(episode).resolve()
    errors: list[str] = []
    warnings: list[str] = []
    if not episode_dir.is_dir():
        return {
            "schema_version": SCHEMA_VERSION,
            "episode": str(episode_dir),
            "valid": False,
            "training_eligible": False,
            "errors": ["episode directory does not exist"],
            "warnings": [],
        }
    if episode_dir.name.endswith(".partial"):
        errors.append("partial episode is not finalized")

    if (episode_dir / "episode.json").is_file() and not (
        episode_dir / "metadata.json"
    ).is_file():
        return _validate_raw_episode(episode_dir, deep=deep)

    metadata = _json_object(episode_dir / "metadata.json", "metadata.json", errors)
    _validate_schema_metadata(metadata, errors, warnings)
    rows, row_errors = load_canonical_rows(episode_dir)
    errors.extend(row_errors)
    quality_rows, quality_errors = load_data_quality_rows(episode_dir)
    errors.extend(quality_errors)
    expected_count = metadata.get("sample_count")
    if not isinstance(expected_count, int) or expected_count != len(rows):
        errors.append(
            f"metadata sample_count {expected_count!r} does not match {len(rows)} rows"
        )

    fps = metadata.get("dataset_fps") if isinstance(metadata.get("dataset_fps"), int) else 0
    period_ns = round(1_000_000_000 / fps) if fps > 0 else 0
    start_ns = metadata.get("start_host_monotonic_ns")
    previous_timestamp: int | None = None
    previous_nominal_slot = -1
    previous_camera_frame: dict[tuple[str, str], int] = {}
    raw_camera_index: dict[tuple[str, int | None, int, int], dict[str, Path]] = {}
    for role in ("workspace", "wrist"):
        raw_path = episode_dir / "raw" / f"camera_{role}.jsonl"
        if not raw_path.is_file():
            continue
        try:
            for line in raw_path.read_text(encoding="utf-8").splitlines():
                record = json.loads(line)
                if not isinstance(record, dict):
                    continue
                rgb_number, depth_number = record.get("rgb_frame_number"), record.get("depth_frame_number")
                if not isinstance(rgb_number, int) or not isinstance(depth_number, int):
                    continue
                paths = {}
                for name in ("rgb", "depth_raw"):
                    relative = record.get(name)
                    if isinstance(relative, str):
                        paths[name] = episode_dir / relative
                raw_camera_index[(role, record.get("ring_sequence"), rgb_number, depth_number)] = paths
        except (OSError, json.JSONDecodeError):
            pass
    previous_rgb_hash: dict[str, tuple[int, str]] = {}
    repeated_camera_selections = 0
    frozen_payload_transitions = 0
    timing_gap_count = 0
    maximum_abs_source_offset_ns = 0
    image_shapes: dict[tuple[str, str], tuple[int, ...]] = {}

    for index, row in enumerate(rows):
        label = f"row {index}"
        if row.get("frame_index") != index:
            errors.append(f"{label}: non-contiguous frame_index")
        timestamp_ns = row.get("timestamp_host_monotonic_ns")
        if not isinstance(timestamp_ns, int):
            errors.append(f"{label}: timestamp_host_monotonic_ns must be int")
        elif previous_timestamp is not None and timestamp_ns <= previous_timestamp:
            errors.append(f"{label}: timestamp is not strictly increasing")
        if isinstance(timestamp_ns, int):
            previous_timestamp = timestamp_ns

        timing = row.get("timing", {})
        if not isinstance(timing, dict):
            errors.append(f"{label}: timing must be an object")
            timing = {}
        nominal_slot = timing.get("nominal_slot_index", index)
        if not isinstance(nominal_slot, int) or nominal_slot <= previous_nominal_slot:
            errors.append(f"{label}: nominal_slot_index is not strictly increasing")
        else:
            previous_nominal_slot = nominal_slot
            if isinstance(start_ns, int) and isinstance(timestamp_ns, int) and period_ns:
                expected_timestamp = start_ns + nominal_slot * period_ns
                if timestamp_ns != expected_timestamp:
                    errors.append(
                        f"{label}: timestamp does not match nominal canonical slot"
                    )
        missed_before = timing.get("missed_slots_before", 0)
        missed_after = timing.get("missed_slots_after", 0)
        if (
            not isinstance(missed_before, int)
            or missed_before < 0
            or not isinstance(missed_after, int)
            or missed_after < 0
        ):
            errors.append(f"{label}: invalid missed canonical slot count")
        else:
            timing_gap_count += missed_after
            if missed_before or missed_after:
                warnings.append(
                    f"{label}: canonical timing gap before={missed_before}, "
                    f"after={missed_after}"
                )
        if timing.get("synchronization_valid") is not True:
            errors.append(f"{label}: synchronization_valid is not true")
        offsets = timing.get("signed_offsets_ns", {})
        if isinstance(offsets, dict):
            for value in offsets.values():
                if isinstance(value, int):
                    maximum_abs_source_offset_ns = max(
                        maximum_abs_source_offset_ns, abs(value)
                    )

        observation = row.get("observation", {})
        state = observation.get("state", {}) if isinstance(observation, dict) else {}
        action = row.get("action", {})
        if not isinstance(state, dict) or not isinstance(action, dict):
            errors.append(f"{label}: state/action must be objects")
            continue
        arm_q = _finite_vector(
            state.get("arm_q_measured"),
            length=6,
            label=f"{label}.arm_q_measured",
            errors=errors,
        )
        _finite_vector(
            state.get("arm_dq_measured"),
            length=6,
            label=f"{label}.arm_dq_measured",
            errors=errors,
        )
        _finite_vector(
            state.get("tcp_pose"),
            length=7,
            label=f"{label}.tcp_pose",
            errors=errors,
        )
        _finite_vector(
            state.get("hand"),
            length=6,
            label=f"{label}.hand",
            errors=errors,
        )
        arm_target = _finite_vector(
            action.get("arm_q_target"),
            length=6,
            label=f"{label}.arm_q_target",
            errors=errors,
        )
        _finite_vector(
            action.get("hand_target"),
            length=6,
            label=f"{label}.hand_target",
            errors=errors,
        )
        for field_name, values in (
            ("arm_q_measured", arm_q),
            ("arm_q_target", arm_target),
        ):
            if values is None:
                continue
            for joint, (value, (lower, upper)) in enumerate(
                zip(values, JAKA_MINI2_JOINT_LIMITS_RAD, strict=True), 1
            ):
                if value < lower or value > upper:
                    errors.append(
                        f"{label}.{field_name}: J{joint} outside manufacturer boundary"
                    )
        if action.get("arm_status") not in {"accepted", "held_rejected"}:
            errors.append(f"{label}: invalid action.arm_status")
        if action.get("arm_source") not in {
            "accepted_target",
            "measured_hold_reference",
        }:
            errors.append(f"{label}: invalid action.arm_source")
        segment_id = state.get("control_segment_id")
        segment_mode = state.get("control_segment_mode")
        if not isinstance(segment_id, int) or segment_id < 0:
            errors.append(f"{label}: invalid control_segment_id")
        if segment_mode not in {
            "both_idle",
            "arm_only",
            "hand_only",
            "arm_and_hand",
        }:
            errors.append(f"{label}: invalid control_segment_mode")

        images = observation.get("images", {}) if isinstance(observation, dict) else {}
        if not isinstance(images, dict):
            errors.append(f"{label}: observation.images must be an object")
            continue
        camera_metadata = row.get("camera", {})
        for role in ("workspace", "wrist"):
            role_images = images.get(role, {})
            role_camera = (
                camera_metadata.get(role, {})
                if isinstance(camera_metadata, dict)
                else {}
            )
            if not isinstance(role_images, dict) or not isinstance(role_camera, dict):
                errors.append(f"{label}: missing {role} image/camera record")
                continue
            for frame_kind in ("rgb", "depth"):
                field = (
                    "rgb_frame_number"
                    if frame_kind == "rgb"
                    else "depth_frame_number"
                )
                frame_number = role_camera.get(field)
                key = (role, frame_kind)
                if not isinstance(frame_number, int):
                    errors.append(f"{label}: {role}.{field} must be int")
                else:
                    previous = previous_camera_frame.get(key)
                    if previous is not None:
                        if frame_number < previous:
                            errors.append(
                                f"{label}: {role}.{field} regressed"
                            )
                        elif frame_number == previous:
                            repeated_camera_selections += 1
                    previous_camera_frame[key] = frame_number

            ring_sequence = role_camera.get("ring_sequence")
            if isinstance(ring_sequence, int):
                rgb_number = role_camera.get("rgb_frame_number")
                depth_number = role_camera.get("depth_frame_number")
                if not isinstance(rgb_number, int) or not isinstance(depth_number, int):
                    errors.append(f"{label}: {role} ring sequence lacks frame numbers")
                    raw_record = None
                else:
                    raw_key = (role, ring_sequence, rgb_number, depth_number)
                    raw_record = raw_camera_index.get(raw_key)
                if raw_record is None:
                    errors.append(f"{label}: {role} ring sequence has no raw record")
                else:
                    for name in ("rgb", "depth_raw"):
                        canonical_path = _inside_episode(
                            episode_dir,
                            role_images.get(name),
                            f"{label}.{role}.{name}",
                            errors,
                        )
                        raw_image = raw_record.get(name)
                        if canonical_path is not None and raw_image is not None:
                            try:
                                if os.stat(canonical_path).st_ino != os.stat(raw_image).st_ino:
                                    warnings.append(
                                        f"{label}: {role}.{name} is a copied hard-link fallback"
                                    )
                            except OSError as exc:
                                errors.append(f"{label}: cannot stat {role}.{name}: {exc}")

            for name, dtype, dimensions, channels in (
                ("rgb", np.dtype(np.uint8), 3, 3),
                ("depth_raw", np.dtype(np.uint16), 2, None),
            ):
                path = _inside_episode(
                    episode_dir,
                    role_images.get(name),
                    f"{label}.{role}.{name}",
                    errors,
                )
                if not deep or path is None:
                    continue
                value = _load_array(
                    path,
                    dtype=dtype,
                    dimensions=dimensions,
                    channels=channels,
                    label=f"{label}.{role}.{name}",
                    errors=errors,
                )
                if value is None:
                    continue
                shape_key = (role, name)
                shape = tuple(int(item) for item in value.shape)
                expected_shape = image_shapes.setdefault(shape_key, shape)
                if shape != expected_shape:
                    errors.append(
                        f"{label}.{role}.{name}: shape changed from "
                        f"{expected_shape} to {shape}"
                    )
                if name == "rgb":
                    frame_number = role_camera.get("rgb_frame_number")
                    digest = file_sha256(path)
                    previous_payload = previous_rgb_hash.get(role)
                    if (
                        previous_payload is not None
                        and isinstance(frame_number, int)
                        and frame_number > previous_payload[0]
                        and digest == previous_payload[1]
                    ):
                        frozen_payload_transitions += 1
                    if isinstance(frame_number, int):
                        previous_rgb_hash[role] = (frame_number, digest)

            aligned = role_images.get("depth_aligned_to_rgb")
            if aligned is not None:
                path = _inside_episode(
                    episode_dir,
                    aligned,
                    f"{label}.{role}.depth_aligned_to_rgb",
                    errors,
                )
                if deep and path is not None:
                    _load_array(
                        path,
                        dtype=np.dtype(np.uint16),
                        dimensions=2,
                        channels=None,
                        label=f"{label}.{role}.depth_aligned_to_rgb",
                        errors=errors,
                    )

    metadata_missed = metadata.get("canonical_missed_slot_count", 0)
    if isinstance(metadata_missed, int) and metadata_missed != timing_gap_count:
        errors.append(
            "metadata canonical_missed_slot_count does not match canonical rows"
        )
    _validate_calibration(episode_dir, metadata, errors)
    if deep:
        errors.extend(_raw_jsonl_errors(episode_dir))
        if metadata.get("schema_version") == PHYSICAL_SCHEMA_VERSION:
            errors.extend(_physical_v2_raw_errors(episode_dir))
    if frozen_payload_transitions:
        warnings.append(
            f"{frozen_payload_transitions} camera frame-number transitions "
            "had identical RGB payloads; inspect for camera freeze"
        )

    valid = not errors
    invalid_quality_slots = sum(
        1
        for row in quality_rows
        if row.get("record_type") == "canonical_data_quality"
        and row.get("metadata_only") is not False
        and (
            row.get("workspace_valid") is False
            or row.get("wrist_valid") is False
            or row.get("reason") in {"ring_reference_expired", "recorder_queue_full"}
        )
    )
    training_eligible = bool(
        valid
        and metadata.get("completion_status") == "completed"
        and metadata.get("success_label") in TRAINING_SUCCESS_LABELS
        and timing_gap_count == 0
        and invalid_quality_slots == 0
        and metadata.get("quality_state", "completed_valid") == "completed_valid"
        and len(rows) > 0
    )
    return {
        "schema_version": metadata.get("schema_version", SCHEMA_VERSION),
        "episode": str(episode_dir),
        "episode_uuid": metadata.get("episode_uuid"),
        "completion_status": metadata.get("completion_status"),
        "success_label": metadata.get("success_label"),
        "sample_count": len(rows),
        "valid": valid,
        "training_eligible": training_eligible,
        "deep_validation": deep,
        "physically_validated": False,
        "errors": errors,
        "warnings": warnings,
        "quality": {
            "canonical_missed_slot_count": timing_gap_count,
            "repeated_camera_selection_count": repeated_camera_selections,
            "identical_rgb_payload_transition_count": frozen_payload_transitions,
            "maximum_absolute_source_offset_ns": maximum_abs_source_offset_ns,
            "image_shapes": {
                f"{role}.{name}": list(shape)
                for (role, name), shape in sorted(image_shapes.items())
            },
            "metadata_only_slot_count": invalid_quality_slots,
            "quality_row_count": len(quality_rows),
        },
    }


def validation_exit_code(reports: Iterable[Mapping[str, Any]]) -> int:
    return 0 if all(bool(report.get("valid")) for report in reports) else 1
