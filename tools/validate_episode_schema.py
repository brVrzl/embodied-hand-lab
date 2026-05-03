from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np

from data_recorder.episode_recorder import FAILURE_MODES, SUPPORTED_SCHEMA_VERSIONS
from rh56_driver.hand_schema import (
    CANONICAL_HAND_ORDER,
    DEFAULT_EE_ROTATION_DELTA_LIMIT_RAD,
    DEFAULT_EE_TRANSLATION_DELTA_LIMIT_M,
    DEFAULT_HAND_DELTA_LIMIT,
)

REQUIRED_METADATA_FIELDS = {
    "schema_version",
    "format_family",
    "control_hz",
    "timestamp_unit",
    "timestamp_clock",
    "frame_index_base",
    "canonical_hand_order",
    "ee_delta_frame",
    "ee_translation_delta_limit_type",
    "ee_translation_delta_limit_m",
    "rotation_delta_type",
    "action_delta_base",
    "embodiment",
}

EXPECTED_EMBODIMENT = "jaka_mini2_rh56_single_arm"
VALID_EE_DELTA_FRAMES = {"base", "ee_local"}
VALID_EE_TRANSLATION_DELTA_LIMIT_TYPES = {"per_axis", "norm"}
VALID_ROTATION_DELTA_TYPES = {"euler_xyz", "rotvec"}
VALID_ACTION_DELTA_BASES = {"command"}


class ValidationError(Exception):
    pass


def _array(value: Any, *, name: str, length: int) -> np.ndarray:
    if value is None:
        raise ValidationError(f"{name} is missing")
    array = np.asarray(value, dtype=np.float32).reshape(-1)
    if array.size != length:
        raise ValidationError(f"{name} shape must be {length}, got {array.size}")
    if not np.all(np.isfinite(array)):
        raise ValidationError(f"{name} contains non-finite values")
    return array


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def _check_metadata(metadata: dict[str, Any], *, context: str) -> None:
    missing = sorted(field for field in REQUIRED_METADATA_FIELDS if field not in metadata)
    _require(not missing, f"{context}: missing metadata fields {missing}")
    _require(metadata["schema_version"] in SUPPORTED_SCHEMA_VERSIONS, f"{context}: unsupported schema_version {metadata['schema_version']!r}")
    _require(metadata["format_family"] == "episode_step_jsonl", f"{context}: invalid format_family")
    _require(metadata["timestamp_unit"] == "seconds", f"{context}: invalid timestamp_unit")
    _require(metadata["timestamp_clock"] in {"unix_time", "ros_time"}, f"{context}: invalid timestamp_clock")
    _require(int(metadata["frame_index_base"]) == 0, f"{context}: frame_index_base must be 0")
    _require(metadata["canonical_hand_order"] == list(CANONICAL_HAND_ORDER), f"{context}: canonical_hand_order mismatch")
    _require(metadata["ee_delta_frame"] in VALID_EE_DELTA_FRAMES, f"{context}: invalid ee_delta_frame")
    _require(
        metadata["ee_translation_delta_limit_type"] in VALID_EE_TRANSLATION_DELTA_LIMIT_TYPES,
        f"{context}: invalid ee_translation_delta_limit_type",
    )
    _require(float(metadata["ee_translation_delta_limit_m"]) > 0.0, f"{context}: ee_translation_delta_limit_m must be positive")
    _require(metadata["rotation_delta_type"] in VALID_ROTATION_DELTA_TYPES, f"{context}: invalid rotation_delta_type")
    _require(metadata["action_delta_base"] in VALID_ACTION_DELTA_BASES, f"{context}: invalid action_delta_base")
    _require(metadata["embodiment"] == EXPECTED_EMBODIMENT, f"{context}: invalid embodiment")
    _require(float(metadata["control_hz"]) > 0.0, f"{context}: control_hz must be positive")
    _require(metadata.get("hand_delta_cmd_clipped") is True, f"{context}: hand_delta_cmd_clipped must be true")
    _require(metadata.get("hand_delta_state_clipped") is True, f"{context}: hand_delta_state_clipped must be true")
    _require(metadata.get("hand_delta_state_raw_available") is True, f"{context}: hand_delta_state_raw_available must be true")


def _check_sample(sample: dict[str, Any], *, eps: float) -> dict[str, float]:
    for field in ("index", "episode_index", "frame_index", "task_index", "timestamp"):
        _require(field in sample, f"sample missing {field}")
    for field in ("is_first", "is_last", "is_terminal"):
        _require(isinstance(sample.get(field), bool), f"{field} must be bool")
    _require(isinstance(sample.get("reward"), (int, float)), "reward must be numeric")
    _require(isinstance(sample.get("discount"), (int, float)), "discount must be numeric")
    _require(0.0 <= float(sample["discount"]) <= 1.0, "discount must be in [0, 1]")
    _check_metadata(sample.get("metadata") or {}, context=f"episode={sample.get('episode_index')} frame={sample.get('frame_index')}")

    observation = sample.get("observation") or {}
    state = observation.get("state") or {}
    action = sample.get("action") or {}
    metadata = sample["metadata"]

    hand_state = _array(state.get("hand_state"), name="observation.state.hand_state", length=6)
    hand_cmd_last = _array(state.get("hand_cmd_last"), name="observation.state.hand_cmd_last", length=6)
    hand_error = _array(state.get("hand_error"), name="observation.state.hand_error", length=6)
    hand_cmd = _array(action.get("hand_cmd") or hand_cmd_last, name="action.hand_cmd", length=6)
    hand_delta_cmd = _array(action.get("hand_delta_cmd"), name="action.hand_delta_cmd", length=6)
    hand_delta_state = _array(action.get("hand_delta_state"), name="action.hand_delta_state", length=6)
    if "hand_delta_state_raw" in action:
        _array(action.get("hand_delta_state_raw"), name="action.hand_delta_state_raw", length=6)
    ee_delta = _array(action.get("ee_delta"), name="action.ee_delta", length=6)
    robot_q_current = _array(state.get("robot_q_current") or action.get("robot_q_current"), name="robot_q_current", length=6)
    robot_q_desired = _array(action.get("robot_q_desired"), name="action.robot_q_desired", length=6)

    _ = robot_q_current, robot_q_desired
    _require(np.all(hand_state >= -eps) and np.all(hand_state <= 1.0 + eps), "hand_state must be in [0, 1]")
    _require(np.all(hand_cmd >= -eps) and np.all(hand_cmd <= 1.0 + eps), "hand_cmd must be in [0, 1]")
    _require(np.all(hand_cmd_last >= -eps) and np.all(hand_cmd_last <= 1.0 + eps), "hand_cmd_last must be in [0, 1]")
    _require(np.allclose(hand_error, hand_cmd_last - hand_state, atol=eps), "hand_error must equal hand_cmd_last - hand_state")
    _require(np.max(np.abs(hand_delta_cmd)) <= DEFAULT_HAND_DELTA_LIMIT + eps, "hand_delta_cmd exceeds limit")
    _require(np.max(np.abs(hand_delta_state)) <= DEFAULT_HAND_DELTA_LIMIT + eps, "hand_delta_state exceeds limit")

    translation_norm = float(np.linalg.norm(ee_delta[:3]))
    max_abs_xyz = float(np.max(np.abs(ee_delta[:3])))
    translation_limit = float(metadata["ee_translation_delta_limit_m"])
    if metadata["ee_translation_delta_limit_type"] == "per_axis":
        _require(max_abs_xyz <= translation_limit + eps, "ee translation delta exceeds per-axis limit")
    else:
        _require(translation_norm <= translation_limit + eps, "ee translation delta exceeds norm limit")
    if metadata["rotation_delta_type"] == "rotvec":
        _require(float(np.linalg.norm(ee_delta[3:])) <= DEFAULT_EE_ROTATION_DELTA_LIMIT_RAD + eps, "ee rotvec delta exceeds limit")
    else:
        _require(np.max(np.abs(ee_delta[3:])) <= DEFAULT_EE_ROTATION_DELTA_LIMIT_RAD + eps, "ee euler_xyz delta exceeds limit")

    success = sample.get("episode_success")
    _require(isinstance(success, bool), "episode_success must be bool")
    failure_mode = sample.get("episode_failure_mode")
    _require(failure_mode in FAILURE_MODES, f"invalid failure_mode {failure_mode!r}")
    if success:
        _require(failure_mode == "none", "success episode must use failure_mode=none")
    else:
        _require(failure_mode != "none", "failure episode must have a non-none failure_mode")
    if sample["is_last"]:
        _require(sample["is_terminal"] is True, "last sample must be terminal")
        _require(float(sample["discount"]) == 0.0, "last sample must have discount=0")
    else:
        _require(sample["is_terminal"] is False, "non-last sample must not be terminal")

    privileged = metadata.get("privileged_observation") or {}
    _require(privileged.get("object_pose") is True, "metadata.privileged_observation.object_pose must be true")
    return {
        "max_abs_ee_delta_xyz": max_abs_xyz,
        "ee_translation_delta_norm": translation_norm,
    }


def validate(export_root: str | Path, *, eps: float = 1e-6) -> dict[str, Any]:
    export_root = Path(export_root).resolve()
    samples_path = export_root / "samples.jsonl"
    manifest_path = export_root / "manifest.json"
    if not samples_path.exists():
        raise ValidationError(f"Missing {samples_path}")
    if not manifest_path.exists():
        raise ValidationError(f"Missing {manifest_path}")

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if "metadata" in manifest:
        _check_metadata(manifest["metadata"], context="manifest.metadata")
    for episode in manifest.get("episodes", []):
        _require(episode.get("failure_mode") in FAILURE_MODES, f"manifest episode has invalid failure_mode: {episode.get('failure_mode')!r}")
        if "metadata" in episode:
            _check_metadata(episode["metadata"], context=f"manifest episode {episode.get('episode_id')}")

    per_episode: dict[int, list[dict[str, Any]]] = defaultdict(list)
    metadata_values: dict[str, set[Any]] = {
        "schema_version": set(),
        "ee_delta_frame": set(),
        "ee_translation_delta_limit_type": set(),
        "ee_translation_delta_limit_m": set(),
        "rotation_delta_type": set(),
        "action_delta_base": set(),
        "embodiment": set(),
    }
    max_abs_ee_delta_xyz = 0.0
    max_ee_translation_delta_norm = 0.0
    sample_count = 0
    expected_global_index = 0
    with samples_path.open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            sample = json.loads(line)
            try:
                sample_metrics = _check_sample(sample, eps=eps)
            except ValidationError as exc:
                raise ValidationError(f"{samples_path}:{line_no}: {exc}") from exc
            sample_count += 1
            _require(int(sample["index"]) == expected_global_index, f"global index gap at sample {sample['index']}")
            expected_global_index += 1
            max_abs_ee_delta_xyz = max(max_abs_ee_delta_xyz, sample_metrics["max_abs_ee_delta_xyz"])
            max_ee_translation_delta_norm = max(max_ee_translation_delta_norm, sample_metrics["ee_translation_delta_norm"])
            per_episode[int(sample["episode_index"])].append(sample)
            for key in metadata_values:
                metadata_values[key].add(sample["metadata"][key])

    _require(sample_count > 0, "samples.jsonl contains no samples")
    for key, values in metadata_values.items():
        _require(len(values) == 1, f"{key} must be consistent across export, got {sorted(values)!r}")

    for episode_index, samples in per_episode.items():
        samples = sorted(samples, key=lambda item: item["frame_index"])
        last_frame = -1
        last_timestamp = -float("inf")
        for offset, sample in enumerate(samples):
            frame_index = int(sample["frame_index"])
            timestamp = float(sample["timestamp"])
            _require(frame_index > last_frame, f"episode {episode_index}: frame_index is not strictly increasing")
            _require(timestamp >= last_timestamp, f"episode {episode_index}: timestamp is not monotonic")
            _require(sample["is_first"] is (offset == 0), f"episode {episode_index}: invalid is_first")
            _require(sample["is_last"] is (offset == len(samples) - 1), f"episode {episode_index}: invalid is_last")
            last_frame = frame_index
            last_timestamp = timestamp

    return {
        "export_root": str(export_root),
        "episodes": len(per_episode),
        "samples": sample_count,
        "schema_version": next(iter(metadata_values["schema_version"])),
        "ee_delta_frame": next(iter(metadata_values["ee_delta_frame"])),
        "ee_translation_delta_limit_type": next(iter(metadata_values["ee_translation_delta_limit_type"])),
        "ee_translation_delta_limit_m": next(iter(metadata_values["ee_translation_delta_limit_m"])),
        "max_abs_ee_delta_xyz": max_abs_ee_delta_xyz,
        "max_ee_translation_delta_norm": max_ee_translation_delta_norm,
        "rotation_delta_type": next(iter(metadata_values["rotation_delta_type"])),
        "action_delta_base": next(iter(metadata_values["action_delta_base"])),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate structured episode schema for JAKA mini2 + RH56 datasets.")
    parser.add_argument("--export-root", required=True)
    parser.add_argument("--eps", type=float, default=1e-6)
    args = parser.parse_args()

    try:
        summary = validate(args.export_root, eps=args.eps)
    except ValidationError as exc:
        print(f"Schema validation failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc

    print("Schema validation passed")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
