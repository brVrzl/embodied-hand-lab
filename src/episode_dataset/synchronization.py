"""Small offline synchronizer for reviewed LeRobot staging episodes."""

from __future__ import annotations

import bisect
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


def synchronize_staging_episode(
    root: str | Path,
    episode: str | int,
    *,
    camera_tolerance_ms: float = 100.0,
    fps: int | None = None,
) -> dict[str, Any]:
    """Build a causal fixed-rate synchronization report without reading images."""

    root_path = Path(root).resolve()
    name = _episode_name(episode)
    rows = _read_jsonl(root_path / "data" / "chunk-000" / f"{name}.jsonl")
    if not rows:
        raise ValueError("cannot synchronize an empty staging episode")
    if not math.isfinite(camera_tolerance_ms) or camera_tolerance_ms < 0.0:
        raise ValueError("camera_tolerance_ms must be finite and non-negative")
    dataset_fps = int(fps or _metadata_fps(root_path, name) or 30)
    if dataset_fps <= 0:
        raise ValueError("fps must be positive")

    audit_root = root_path / "audit" / "chunk-000" / name
    jaka = _jaka_observations(rows, _read_jsonl(audit_root / "jaka_state.jsonl"))
    rh56_position, rh56_force = _rh56_observations(
        rows, _read_jsonl(audit_root / "rh56_feedback.jsonl")
    )
    camera_maps = {
        role: _camera_clock_map(rows, role) for role in ("workspace", "wrist")
    }
    cameras = {
        role: _camera_observations(rows, role, camera_maps[role])
        for role in ("workspace", "wrist")
    }
    start_ns = int(rows[0]["timestamp_ns"])
    end_ns = int(rows[-1]["timestamp_ns"])
    period_ns = round(1_000_000_000 / dataset_fps)
    tolerance_ns = round(camera_tolerance_ms * 1e6)
    timeline = []
    timestamp_ns = start_ns
    while timestamp_ns <= end_ns:
        jaka_value = _latest(jaka, timestamp_ns)
        position_value = _latest(rh56_position, timestamp_ns)
        force_value = _latest(rh56_force, timestamp_ns)
        camera_values = {
            role: _select_camera(
                camera_values_for_role,
                timestamp_ns,
                tolerance_ns,
            )
            for role, camera_values_for_role in cameras.items()
        }
        timeline.append(
            {
                "timestamp_ns": timestamp_ns,
                "jaka": _timed_value(jaka_value, timestamp_ns),
                "rh56_position": _timed_value(position_value, timestamp_ns),
                "rh56_force": _timed_value(force_value, timestamp_ns),
                "camera": camera_values,
                "validity": {
                    "jaka": jaka_value is not None,
                    "rh56_position": position_value is not None,
                    "rh56_force": force_value is not None,
                    **{
                        f"camera.{role}": bool(value["valid"])
                        for role, value in camera_values.items()
                    },
                },
            }
        )
        timestamp_ns += period_ns

    return {
        "format": "embodied_lab.staging_sync_check_v1",
        "episode": name,
        "fps": dataset_fps,
        "camera_tolerance_ms": camera_tolerance_ms,
        "image_interpolation": "none",
        "future_force_samples_used": False,
        "camera_clock_maps": camera_maps,
        "timeline": timeline,
        "validity_counts": _validity_counts(timeline),
    }


def _episode_name(episode: str | int) -> str:
    if isinstance(episode, int):
        return f"episode_{episode:06d}"
    value = str(episode)
    return value if value.startswith("episode_") else f"episode_{int(value):06d}"


def _metadata_fps(root: Path, name: str) -> int | None:
    path = root / "meta" / "episodes" / "chunk-000" / f"{name}.json"
    if not path.exists():
        return None
    value = json.loads(path.read_text(encoding="utf-8")).get("fps")
    return None if value is None else int(value)


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def _camera_clock_map(rows: Sequence[Mapping[str, Any]], role: str) -> dict[str, Any]:
    pairs = []
    domains: set[str] = set()
    for row in rows:
        camera = row.get("camera", {}).get(role, {})
        try:
            device_ms = float(camera["rgb_device_timestamp_ms"])
            host_ns = int(camera["host_monotonic_ns"])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(device_ms):
            pairs.append((device_ms, host_ns))
            domain = camera.get("rgb_timestamp_domain")
            if domain is not None:
                domains.add(str(domain))
    if not pairs:
        return {
            "valid": False,
            "clock": "rgb_device_ms_to_host_monotonic_ns",
            "pair_count": 0,
            "timestamp_domains": sorted(domains),
        }
    xs = [pair[0] for pair in pairs]
    ys = [pair[1] for pair in pairs]
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    denominator = sum((value - mean_x) ** 2 for value in xs)
    slope = 1_000_000.0 if denominator == 0.0 else sum(
        (x - mean_x) * (y - mean_y) for x, y in pairs
    ) / denominator
    intercept = mean_y - slope * mean_x
    residuals = [abs((slope * x + intercept) - y) for x, y in pairs]
    return {
        "valid": len(pairs) >= 2 and math.isfinite(slope) and math.isfinite(intercept),
        "clock": "rgb_device_ms_to_host_monotonic_ns",
        "pair_count": len(pairs),
        "slope_ns_per_ms": slope,
        "intercept_ns": intercept,
        "max_fit_residual_ns": max(residuals),
        "timestamp_domains": sorted(domains),
    }


def _camera_observations(
    rows: Sequence[Mapping[str, Any]], role: str, clock_map: Mapping[str, Any]
) -> list[dict[str, Any]]:
    result = []
    seen: set[tuple[int, int]] = set()
    slope = float(clock_map.get("slope_ns_per_ms", 1_000_000.0))
    intercept = float(clock_map.get("intercept_ns", 0.0))
    for row in rows:
        camera = row.get("camera", {}).get(role, {})
        try:
            frame_number = int(camera["rgb_frame_number"])
            device_ms = float(camera["rgb_device_timestamp_ms"])
            host_ns = int(camera["host_monotonic_ns"])
        except (KeyError, TypeError, ValueError):
            continue
        key = (frame_number, host_ns)
        if key in seen:
            continue
        seen.add(key)
        result.append(
            {
                "timestamp_ns": int(slope * device_ms + intercept),
                "host_timestamp_ns": host_ns,
                "frame_index": int(row["frame_index"]),
                "frame_number": frame_number,
                "device_timestamp_ms": device_ms,
            }
        )
    return sorted(result, key=lambda value: value["timestamp_ns"])


def _jaka_observations(
    rows: Sequence[Mapping[str, Any]], audit_rows: Sequence[Mapping[str, Any]]
) -> list[dict[str, Any]]:
    result = []
    for row in audit_rows:
        timestamp = row.get("read_host_monotonic_ns")
        values = row.get("measured_joint_position_rad")
        if timestamp is not None and values is not None:
            result.append({"timestamp_ns": int(timestamp), "values": list(values)})
    if result:
        return sorted(result, key=lambda value: value["timestamp_ns"])
    for row in rows:
        state = row.get("observation.state")
        if isinstance(state, list) and len(state) >= 6:
            timestamp = row.get("timing", {}).get("source_timestamps_ns", {}).get(
                "jaka_observation", row.get("timestamp_ns")
            )
            result.append({"timestamp_ns": int(timestamp), "values": state[:6]})
    return result


def _rh56_observations(
    rows: Sequence[Mapping[str, Any]], audit_rows: Sequence[Mapping[str, Any]]
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    position = []
    force = []
    for row in audit_rows:
        timestamps = row.get("hand_feedback_register_timestamps_ns", {})
        registers = row.get("rh56_registers", {})
        if timestamps.get("ANGLE_ACT") is not None and registers.get("ANGLE_ACT") is not None:
            position.append({
                "timestamp_ns": int(timestamps["ANGLE_ACT"]),
                "values": list(registers["ANGLE_ACT"]),
            })
        if timestamps.get("FORCE_ACT") is not None and registers.get("FORCE_ACT") is not None:
            force.append({
                "timestamp_ns": int(timestamps["FORCE_ACT"]),
                "values": list(registers["FORCE_ACT"]),
            })
    if position or force:
        return (
            sorted(position, key=lambda value: value["timestamp_ns"]),
            sorted(force, key=lambda value: value["timestamp_ns"]),
        )
    for row in rows:
        timing = row.get("timing", {})
        timestamps = timing.get("source_timestamps_ns", {})
        state = row.get("observation.state")
        if timestamps.get("rh56_angle_act") is not None and isinstance(state, list):
            position.append({
                "timestamp_ns": int(timestamps["rh56_angle_act"]),
                "values": state[6:12],
            })
        if timestamps.get("rh56_force_act") is not None:
            force.append({
                "timestamp_ns": int(timestamps["rh56_force_act"]),
                "values": list(row.get("observation.force", ())),
            })
    return (
        sorted(position, key=lambda value: value["timestamp_ns"]),
        sorted(force, key=lambda value: value["timestamp_ns"]),
    )


def _latest(values: Sequence[Mapping[str, Any]], timestamp_ns: int) -> Mapping[str, Any] | None:
    timestamps = [int(value["timestamp_ns"]) for value in values]
    index = bisect.bisect_right(timestamps, int(timestamp_ns)) - 1
    return None if index < 0 else values[index]


def _timed_value(value: Mapping[str, Any] | None, timestamp_ns: int) -> dict[str, Any]:
    if value is None:
        return {"values": None, "timestamp_ns": None, "age_ns": None, "valid": False}
    return {
        "values": list(value["values"]),
        "timestamp_ns": int(value["timestamp_ns"]),
        "age_ns": int(timestamp_ns) - int(value["timestamp_ns"]),
        "valid": int(value["timestamp_ns"]) <= int(timestamp_ns),
    }


def _select_camera(
    values: Sequence[Mapping[str, Any]], timestamp_ns: int, tolerance_ns: int
) -> dict[str, Any]:
    selected = _latest(values, timestamp_ns)
    if selected is None:
        return {"frame_index": None, "frame_number": None, "age_ns": None, "valid": False}
    age_ns = int(timestamp_ns) - int(selected["timestamp_ns"])
    valid = 0 <= age_ns <= tolerance_ns
    return {
        "frame_index": selected["frame_index"] if valid else None,
        "frame_number": selected["frame_number"] if valid else None,
        "device_timestamp_ms": selected["device_timestamp_ms"] if valid else None,
        "mapped_host_timestamp_ns": selected["timestamp_ns"] if valid else None,
        "age_ns": age_ns,
        "valid": valid,
    }


def _validity_counts(timeline: Iterable[Mapping[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for row in timeline:
        for name, valid in row["validity"].items():
            counts[name] = counts.get(name, 0) + int(bool(valid))
    return counts
