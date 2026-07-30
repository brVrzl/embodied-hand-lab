#!/usr/bin/env python3
"""Reproducible offline coverage report for Quest-to-RH56 retarget logs."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

from motion_input import HtsRawRecordingReader, Side, parse_hts_datagram
from motion_input.hts_protocol import HtsLandmarksPacket
from quest_jaka_sim.hand_retarget import right_hand_palm_local_frame


CANONICAL_ORDER = (
    "index",
    "middle",
    "ring",
    "pinky",
    "thumb_close",
    "thumb_lateral",
)
FINGER_ROWS = {
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
    "thumb_close": (1, 2, 3, 4),
}
PERCENTILES = (1, 5, 50, 95, 99)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze paired HTS, retarget-event, and RH56 telemetry JSONL logs."
    )
    parser.add_argument("--hts", required=True)
    parser.add_argument("--events", required=True)
    parser.add_argument("--telemetry", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--calibration-id", default="quest_rh56_sim_uncalibrated_v1")
    parser.add_argument("--max-close", type=float, default=0.8)
    parser.add_argument("--time-bin-sec", type=float, default=10.0)
    return parser


def _jsonl(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def _angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    left, right = a - b, c - b
    denominator = float(np.linalg.norm(left) * np.linalg.norm(right))
    if denominator <= 1e-9:
        return math.pi
    cosine = float(np.clip(np.dot(left, right) / denominator, -1.0, 1.0))
    return math.acos(cosine)


def _cosine(left: np.ndarray, right: np.ndarray) -> float:
    denominator = float(np.linalg.norm(left) * np.linalg.norm(right))
    if denominator <= 1e-9:
        return 0.0
    return float(np.clip(np.dot(left, right) / denominator, -1.0, 1.0))


def _finger_features(
    points: np.ndarray,
    indices: tuple[int, int, int, int],
    *,
    mcp_weight: float = 0.15,
    mcp_deadband: float = 0.15,
) -> dict[str, float]:
    a, b, c, d = (points[index] for index in indices)
    pip = math.pi - _angle(a, b, c)
    dip = math.pi - _angle(b, c, d)
    distal = float(np.clip((pip + dip) / math.pi, 0.0, 1.0))
    palm_forward = points[9] - points[0]
    mcp_raw = float(
        np.clip(math.acos(_cosine(b - a, palm_forward)) / (math.pi / 2.0), 0.0, 1.0)
    )
    mcp_deadbanded = float(
        np.clip((mcp_raw - mcp_deadband) / (1.0 - mcp_deadband), 0.0, 1.0)
    )
    combined = float(
        np.clip(distal + mcp_weight * mcp_deadbanded * (1.0 - distal), 0.0, 1.0)
    )
    return {
        "raw_curl_rad": pip + dip,
        "mcp_flexion": mcp_raw,
        "pip_flexion_rad": pip,
        "dip_flexion_rad": dip,
        "combined_curl": combined,
    }


def _hts_features(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for datagram in HtsRawRecordingReader(path).datagrams():
        for packet in parse_hts_datagram(datagram.payload):
            if not isinstance(packet, HtsLandmarksPacket) or packet.header.side is not Side.RIGHT:
                continue
            points = np.asarray(packet.positions_wrist_m, dtype=np.float64)
            frame = right_hand_palm_local_frame(points, epsilon_m=1e-5)
            if frame is None:
                continue
            fingers = {
                name: _finger_features(points, indices)
                for name, indices in FINGER_ROWS.items()
            }
            across = np.asarray(frame.across_axis)
            palm_center = np.mean(points[[0, 5, 9, 13, 17]], axis=0)
            thumb_displacement = points[4] - points[1]
            raw_across = float(np.dot(thumb_displacement, across) / frame.palm_width_m)
            palm_length_m = max(float(np.linalg.norm(points[9] - points[0])), 1e-6)
            rows.append(
                {
                    "monotonic_ns": datagram.receive_monotonic_ns,
                    "source_sequence": packet.header.source_sequence,
                    "fingers": fingers,
                    "thumb_tip": points[4].tolist(),
                    "index_mcp": points[5].tolist(),
                    "middle_mcp": points[9].tolist(),
                    "palm_center": palm_center.tolist(),
                    "wrist_origin": points[0].tolist(),
                    "palm_width_m": frame.palm_width_m,
                    "palm_length_m": palm_length_m,
                    "thumb_raw_across_palm": raw_across,
                    "thumb_index_distance_palm": float(
                        np.linalg.norm(points[4] - points[8]) / palm_length_m
                    ),
                    "thumb_middle_distance_palm": float(
                        np.linalg.norm(points[4] - points[12]) / palm_length_m
                    ),
                    "index_middle_distance_palm": float(
                        np.linalg.norm(points[8] - points[12]) / palm_length_m
                    ),
                }
            )
    return rows


def _values(rows: Iterable[dict[str, Any]], path: Sequence[str]) -> list[float]:
    result: list[float] = []
    for row in rows:
        value: Any = row
        for key in path:
            if not isinstance(value, dict):
                value = None
                break
            value = value.get(key)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            result.append(float(value))
    return result


def _stats(values: Sequence[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "min": None, "p01": None, "p05": None, "p50": None, "p95": None, "p99": None, "max": None}
    array = np.asarray(values, dtype=np.float64)
    quantiles = np.percentile(array, PERCENTILES)
    return {
        "count": len(values),
        "min": float(np.min(array)),
        "p01": float(quantiles[0]),
        "p05": float(quantiles[1]),
        "p50": float(quantiles[2]),
        "p95": float(quantiles[3]),
        "p99": float(quantiles[4]),
        "max": float(np.max(array)),
    }


def _fmt(value: float | int | None, digits: int = 4) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, int):
        return str(value)
    return f"{value:.{digits}f}"


def _stats_cells(values: Sequence[float]) -> str:
    stats = _stats(values)
    return " | ".join(
        _fmt(stats[key]) for key in ("min", "p01", "p05", "p50", "p95", "p99", "max")
    )


def _canonical_target(event: dict[str, Any], field: str) -> list[float] | None:
    value = event.get(field)
    if not isinstance(value, list) or len(value) != 6:
        return None
    # Session order is thumb_lateral, thumb_close, index, middle, ring, pinky.
    return [float(value[2]), float(value[3]), float(value[4]), float(value[5]), float(value[1]), float(value[0])]


def _channel_rows(rows: Sequence[dict[str, Any]], path: Sequence[str]) -> dict[str, list[float]]:
    result = {name: [] for name in CANONICAL_ORDER}
    for row in rows:
        value: Any = row
        for key in path:
            value = value.get(key) if isinstance(value, dict) else None
        if not isinstance(value, list) or len(value) != 6:
            continue
        for name, item in zip(CANONICAL_ORDER, value, strict=True):
            if isinstance(item, (int, float)) and math.isfinite(float(item)):
                result[name].append(float(item))
    return result


def _event_targets(events: Sequence[dict[str, Any]], field: str) -> dict[str, list[float]]:
    result = {name: [] for name in CANONICAL_ORDER}
    for event in events:
        target = _canonical_target(event, field)
        if target is None:
            continue
        for name, value in zip(CANONICAL_ORDER, target, strict=True):
            result[name].append(value)
    return result


def _count_transitions(values: Sequence[str], entering: set[str]) -> int:
    return sum(
        current in entering and previous not in entering
        for previous, current in zip(values, values[1:])
    )


def _lag_ms(
    commands: Sequence[tuple[int, float]],
    measured: Sequence[tuple[int, float]],
) -> float | None:
    if len(commands) < 3 or len(measured) < 3:
        return None
    command_time = np.asarray([row[0] for row in commands], dtype=np.float64)
    command_value = np.asarray([row[1] for row in commands], dtype=np.float64)
    measured_time = np.asarray([row[0] for row in measured], dtype=np.float64)
    measured_value = np.asarray([row[1] for row in measured], dtype=np.float64)
    if float(np.ptp(command_value)) < 0.02:
        return None
    scores: list[tuple[float, float]] = []
    for lag_ms in np.arange(0.0, 405.0, 5.0):
        source_time = measured_time - lag_ms * 1e6
        mask = (source_time >= command_time[0]) & (source_time <= command_time[-1])
        if int(np.count_nonzero(mask)) < 3:
            continue
        expected = np.interp(source_time[mask], command_time, command_value)
        mse = float(np.mean((measured_value[mask] - expected) ** 2))
        scores.append((mse, float(lag_ms)))
    return None if not scores else min(scores)[1]


def _time_bins(
    telemetry: Sequence[dict[str, Any]],
    *,
    bin_sec: float,
) -> list[tuple[float, float, dict[str, tuple[float | None, float | None]]]]:
    rows = [row for row in telemetry if isinstance(row.get("monotonic_ns"), int)]
    if not rows:
        return []
    start = int(rows[0]["monotonic_ns"])
    end = int(rows[-1]["monotonic_ns"])
    width_ns = int(round(bin_sec * 1e9))
    result = []
    for lower in range(start, end + 1, width_ns):
        upper = min(end + 1, lower + width_ns)
        selected = [row for row in rows if lower <= int(row["monotonic_ns"]) < upper]
        command = _channel_rows(selected, ("action", "hand_target"))
        measured = _channel_rows(selected, ("observation", "hand_position_normalized"))
        ranges = {
            name: (
                None if not command[name] else max(command[name]),
                None if not measured[name] else max(measured[name]),
            )
            for name in CANONICAL_ORDER
        }
        result.append(((lower - start) / 1e9, (upper - start) / 1e9, ranges))
    return result


def _clutch_segments(events: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    by_cycle: dict[int, list[dict[str, Any]]] = {}
    for event in events:
        cycle = event.get("hand_clutch_cycle_count")
        if isinstance(cycle, int) and cycle > 0 and event.get("hand_reference_captured"):
            by_cycle.setdefault(cycle, []).append(event)
    segments = []
    for cycle, rows in sorted(by_cycle.items()):
        timestamps = [int(row["control_monotonic_ns"]) for row in rows]
        clipped = _event_targets(rows, "hand_clipped_target_rad")
        segments.append(
            {
                "cycle": cycle,
                "duration_sec": (max(timestamps) - min(timestamps)) / 1e9,
                "updates": sum(bool(row.get("hand_command_updated")) for row in rows),
                "max": {name: max(values, default=None) for name, values in clipped.items()},
            }
        )
    return segments


def _render(
    *,
    args: argparse.Namespace,
    hts: Sequence[dict[str, Any]],
    events: Sequence[dict[str, Any]],
    telemetry: Sequence[dict[str, Any]],
) -> str:
    command = _channel_rows(telemetry, ("action", "hand_target"))
    requested = _channel_rows(telemetry, ("action", "requested_hand_target"))
    protocol_raw = _channel_rows(telemetry, ("action", "selected_hand_position_raw"))
    measured = _channel_rows(telemetry, ("observation", "hand_position_normalized"))
    current = _channel_rows(telemetry, ("observation", "hand_current_raw_count"))
    force = _channel_rows(telemetry, ("observation", "hand_current_or_load"))
    event_requested = _event_targets(events, "hand_requested_target_rad")
    event_clipped = _event_targets(events, "hand_clipped_target_rad")

    event_states = [str(row.get("hand_clutch_state")) for row in events]
    valid_flags = [bool(row.get("hand_skeleton_valid")) for row in events]
    tracking_loss = sum(a and not b for a, b in zip(valid_flags, valid_flags[1:]))
    tracking_recovery = sum(not a and b for a, b in zip(valid_flags, valid_flags[1:]))
    reacquisition = sum(bool(row.get("hand_reference_capture")) for row in events)
    grip_reacquisition = _count_transitions(event_states, {"reacquire"})
    duplicate = sum(
        row.get("hand_command_disposition") == "exact_duplicate_suppressed"
        for row in telemetry
    )
    command_rows = [row for row in telemetry if row.get("rh56_scheduled_operation") == "COMMAND"]
    unchanged = 0
    previous: list[float] | None = None
    for row in command_rows:
        value = row.get("action", {}).get("requested_hand_target")
        if isinstance(value, list) and len(value) == 6:
            current_value = [float(item) for item in value]
            unchanged += previous == current_value
            previous = current_value

    lines = [
        "# RH56 retarget baseline analysis",
        "",
        "Validation level: offline analysis of previously recorded physical Quest/RH56 logs. This report does not create a new physical PASS.",
        "",
        "## Inputs and conventions",
        "",
        f"- HTS: `{args.hts}` ({len(hts)} valid right-landmark frames)",
        f"- retarget events: `{args.events}` ({len(events)} control ticks)",
        f"- RH56 telemetry: `{args.telemetry}` ({len(telemetry)} rows)",
        f"- loaded calibration identifier: `{args.calibration_id}`",
        f"- physical software closure ceiling: `{args.max_close:g}` (raw 200 with the current 1000-open/0-close encoding)",
        "- canonical normalized order: index, middle, ring, pinky, thumb_close, thumb_lateral; 0=open and 1=close/opposed.",
        "- The historical event fields ending in `_rad` carry normalized values on the physical output path. RH56 telemetry is used as the authoritative submitted/measured unit source.",
        "",
        "## Quest feature coverage",
        "",
        "`raw curl` is PIP+DIP bend in radians. MCP is the unclipped angle-to-palm-forward feature normalized by pi/2. Combined curl is the production distal feature plus the configured 0.15 deadbanded MCP contribution.",
        "",
        "| finger / feature | min | p01 | p05 | p50 | p95 | p99 | max |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name in FINGER_ROWS:
        for feature in ("raw_curl_rad", "mcp_flexion", "pip_flexion_rad", "dip_flexion_rad", "combined_curl"):
            values = [float(row["fingers"][name][feature]) for row in hts]
            lines.append(f"| {name} / {feature} | {_stats_cells(values)} |")

    lines += [
        "",
        "## Thumb opposition input and mapping",
        "",
        "The old feature uses Quest wrist-local landmarks 1 (thumb metacarpal/base), 4 (thumb tip), 5 (index MCP), 9 (middle MCP), and 17 (pinky MCP). `across = normalize(pinky_MCP - index_MCP)`; palm forward is the component of `middle_MCP - wrist` orthogonal to across; palm normal is `across × forward`. Raw opposition is `dot(thumb_tip - thumb_base, across) / distance(index_MCP, pinky_MCP)`. Thus it is wrist/palm-local and scale-normalized, not a world-frame coordinate. The configured old open/opposed extrema are -0.60/0.25.",
        "",
        "The palm center requested for diagnosis is reported as the mean of wrist plus the four MCP landmarks, but the old production feature does not use that center. Relative grip capture stores the current feature and current measured RH56 target; subsequent mapping uses feature delta, gain 1.0, offset equal to the measured reference, dead zone 0.015, and clips to [0, 0.8]. Releasing grip clears the reference; a release-before-press cycle captures a new one. This avoids an activation jump but can discard available absolute travel and can recapture at a saturated/high thumb-lateral pose.",
        "",
        "| thumb quantity | min | p01 | p05 | p50 | p95 | p99 | max |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
        f"| raw across-palm (HTS) | {_stats_cells([float(row['thumb_raw_across_palm']) for row in hts])} |",
        f"| production raw across-palm (events) | {_stats_cells(_values(events, ('thumb_lateral_debug', 'raw_across_palm')))} |",
        f"| normalized feature (events) | {_stats_cells(_values(events, ('thumb_lateral_debug', 'feature_normalized')))} |",
        f"| captured reference | {_stats_cells(_values(events, ('thumb_lateral_debug', 'captured_feature_reference')))} |",
        f"| feature delta after dead zone | {_stats_cells(_values(events, ('thumb_lateral_debug', 'feature_delta')))} |",
        f"| requested normalized | {_stats_cells(event_requested['thumb_lateral'])} |",
        f"| clipped normalized | {_stats_cells(event_clipped['thumb_lateral'])} |",
        f"| submitted normalized | {_stats_cells(command['thumb_lateral'])} |",
        f"| measured ANGLE_ACT normalized | {_stats_cells(measured['thumb_lateral'])} |",
        "",
        "## Complete channel mapping and coverage",
        "",
        "| channel | calibrated feature min..max | relative reference min..max | delta min..max | gain | requested min..max | clipped min..max | submitted min..max | protocol raw min..max | ANGLE_ACT norm min..max | lag ms* |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    feature_paths = {
        "index": (("four_finger_debug", "feature_rad"), 0),
        "middle": (("four_finger_debug", "feature_rad"), 1),
        "ring": (("four_finger_debug", "feature_rad"), 2),
        "pinky": (("four_finger_debug", "feature_rad"), 3),
        "thumb_close": (("thumb_close_debug", "effective_feature_normalized"), None),
        "thumb_lateral": (("thumb_lateral_debug", "effective_feature_normalized"), None),
    }
    reference_paths = {
        "index": (("four_finger_debug", "captured_feature_reference_rad"), 0),
        "middle": (("four_finger_debug", "captured_feature_reference_rad"), 1),
        "ring": (("four_finger_debug", "captured_feature_reference_rad"), 2),
        "pinky": (("four_finger_debug", "captured_feature_reference_rad"), 3),
        "thumb_close": (("thumb_close_debug", "captured_feature_reference_rad"), None),
        "thumb_lateral": (("thumb_lateral_debug", "captured_feature_reference"), None),
    }
    delta_paths = {
        "index": (("four_finger_debug", "feature_delta_rad"), 0),
        "middle": (("four_finger_debug", "feature_delta_rad"), 1),
        "ring": (("four_finger_debug", "feature_delta_rad"), 2),
        "pinky": (("four_finger_debug", "feature_delta_rad"), 3),
        "thumb_close": (("thumb_close_debug", "feature_delta_rad"), None),
        "thumb_lateral": (("thumb_lateral_debug", "feature_delta"), None),
    }

    def event_component(spec: tuple[Sequence[str], int | None]) -> list[float]:
        path, index = spec
        result: list[float] = []
        for row in events:
            value: Any = row
            for key in path:
                value = value.get(key) if isinstance(value, dict) else None
            if index is not None and isinstance(value, list) and len(value) > index:
                value = value[index]
            if isinstance(value, (int, float)) and math.isfinite(float(value)):
                result.append(float(value))
        return result

    successful = [
        row for row in telemetry
        if row.get("rh56_scheduled_operation") == "COMMAND"
        and row.get("hand_command_disposition") == "serial_write_success"
    ]
    angle_rows = [row for row in telemetry if row.get("rh56_scheduled_operation") == "ANGLE"]
    lags: dict[str, float | None] = {}
    for index, name in enumerate(CANONICAL_ORDER):
        command_series = [
            (int(row["monotonic_ns"]), float(row["action"]["hand_target"][index]))
            for row in successful
            if isinstance(row.get("action", {}).get("hand_target"), list)
        ]
        measured_series = [
            (int(row["monotonic_ns"]), float(row["observation"]["hand_position_normalized"][index]))
            for row in angle_rows
            if isinstance(row.get("observation", {}).get("hand_position_normalized"), list)
        ]
        lags[name] = _lag_ms(command_series, measured_series)
        feature_values = event_component(feature_paths[name])
        reference_values = event_component(reference_paths[name])
        delta_values = event_component(delta_paths[name])
        lines.append(
            f"| {name} | {_fmt(min(feature_values, default=None))}..{_fmt(max(feature_values, default=None))} | "
            f"{_fmt(min(reference_values, default=None))}..{_fmt(max(reference_values, default=None))} | "
            f"{_fmt(min(delta_values, default=None))}..{_fmt(max(delta_values, default=None))} | 1.0 | "
            f"{_fmt(min(event_requested[name], default=None))}..{_fmt(max(event_requested[name], default=None))} | "
            f"{_fmt(min(event_clipped[name], default=None))}..{_fmt(max(event_clipped[name], default=None))} | "
            f"{_fmt(min(command[name], default=None))}..{_fmt(max(command[name], default=None))} | "
            f"{_fmt(min(protocol_raw[name], default=None), 0)}..{_fmt(max(protocol_raw[name], default=None), 0)} | "
            f"{_fmt(min(measured[name], default=None))}..{_fmt(max(measured[name], default=None))} | {_fmt(lags[name], 0)} |"
        )
    lines += [
        "",
        "\\* Lag is the 0--400 ms delay (5 ms grid) minimizing mean squared error between ANGLE samples and the interpolated submitted command. It is an observational command-to-feedback estimate, not a causal transport-only measurement.",
        "",
        "## Event statistics",
        "",
        "| channel | low saturation | high saturation | dead-zone occupancy | submitted unchanged occupancy |",
        "|---|---:|---:|---:|---:|",
    ]
    command_pairs = list(zip(command_rows, command_rows[1:]))
    for index, name in enumerate(CANONICAL_ORDER):
        clipped_values = event_clipped[name]
        deltas = event_component(delta_paths[name])
        low = sum(abs(value) <= 1e-9 for value in clipped_values)
        high = sum(abs(value - args.max_close) <= 1e-9 for value in clipped_values)
        dead = sum(abs(value) <= 1e-12 for value in deltas)
        unchanged_channel = 0
        comparable = 0
        for before, after in command_pairs:
            left = before.get("action", {}).get("requested_hand_target")
            right = after.get("action", {}).get("requested_hand_target")
            if isinstance(left, list) and isinstance(right, list):
                comparable += 1
                unchanged_channel += math.isclose(float(left[index]), float(right[index]), abs_tol=1e-12)
        lines.append(
            f"| {name} | {low}/{len(clipped_values)} ({low / max(1, len(clipped_values)):.3f}) | "
            f"{high}/{len(clipped_values)} ({high / max(1, len(clipped_values)):.3f}) | "
            f"{dead}/{len(deltas)} ({dead / max(1, len(deltas)):.3f}) | "
            f"{unchanged_channel}/{comparable} ({unchanged_channel / max(1, comparable):.3f}) |"
        )
    lines += [
        "",
        f"- exact duplicate suppression: {duplicate} rows",
        f"- all-channel unchanged command occupancy: {unchanged}/{max(0, len(command_rows) - 1)}",
        f"- relative reference captures: {reacquisition}",
        f"- grip reacquisition entries: {grip_reacquisition}",
        f"- tracking losses/recoveries: {tracking_loss}/{tracking_recovery}",
        f"- STATUS nonzero rows: {sum(any(row.get('hand_status') or []) for row in telemetry)} (the observed normal status is 2; fault interpretation is owned by the runtime gate)",
        f"- ERROR fault rows: {sum(any(row.get('hand_error') or []) for row in telemetry)}",
        f"- CURRENT absolute peaks by channel: {', '.join(f'{name}={max((abs(v) for v in current[name]), default=0):.0f}' for name in CANONICAL_ORDER)}",
        f"- FORCE_ACT absolute peaks by channel: {', '.join(f'{name}={max((abs(v) for v in force[name]), default=0):.0f}' for name in CANONICAL_ORDER)}",
        "",
        "## Time distribution",
        "",
        "Each cell is maximum submitted command / maximum measured ANGLE_ACT in that time bin.",
        "",
        "| elapsed s | index | middle | ring | pinky | thumb_close | thumb_lateral |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for lower, upper, ranges in _time_bins(telemetry, bin_sec=args.time_bin_sec):
        cells = [f"{_fmt(ranges[name][0], 3)}/{_fmt(ranges[name][1], 3)}" for name in CANONICAL_ORDER]
        lines.append(f"| {lower:.0f}--{upper:.0f} | " + " | ".join(cells) + " |")

    lines += [
        "",
        "## Major gesture segments",
        "",
        "The capture has no operator gesture labels, so it is not valid to call arbitrary time windows fist, pinch, or grasp. The reproducible segments below are grip clutch cycles; maxima are canonical normalized clipped requests.",
        "",
        "| clutch cycle | duration s | updated ticks | max index/middle/ring/pinky/thumb_close/thumb_lateral |",
        "|---:|---:|---:|---|",
    ]
    for segment in _clutch_segments(events):
        maxima = "/".join(_fmt(segment["max"][name], 3) for name in CANONICAL_ORDER)
        lines.append(f"| {segment['cycle']} | {segment['duration_sec']:.3f} | {segment['updates']} | {maxima} |")

    lines += [
        "",
        "## Baseline findings",
        "",
        "1. The real hand path loads a calibration explicitly named `sim_uncalibrated`; that identity and its generic extrema are unsuitable as the final hardware calibration.",
        "2. The full chain is relative: a grip press captures both Quest features and measured ANGLE_ACT, then applies gain to feature delta. Consequently identical human poses can map differently after reacquisition, and captured high thumb-lateral state consumes remaining opposition travel.",
        "3. Four-finger feature coverage, not the 0.8 ceiling, is the first limiting factor in the latest normal run: submitted maxima remain below the ceiling while raw 200 is still available.",
        "4. Thumb lateral is moving and measured feedback follows it, but the old feature spends substantial time at a clipped feature endpoint and the relative offset repeatedly drives the output near/high saturation. This explains visually weak or stuck-looking motion without a serial-rate hypothesis.",
        "5. The log contains no validated index, middle, tripod, or tissue-contact pose labels. It can quantify mapping and feedback, but cannot establish fingertip contact or tissue-grasp success.",
        "",
        "## Reproduce",
        "",
        "```bash",
        f"PYTHONDONTWRITEBYTECODE=1 .venv/bin/python tools/analyze_rh56_retarget_log.py --hts {args.hts} --events {args.events} --telemetry {args.telemetry} --output {args.output} --calibration-id {args.calibration_id} --max-close {args.max_close:g}",
        "```",
        "",
    ]
    return "\n".join(lines)


def main() -> None:
    args = _parser().parse_args()
    if not 0.0 < args.max_close <= 1.0:
        raise ValueError("--max-close must be in (0, 1]")
    if args.time_bin_sec <= 0.0:
        raise ValueError("--time-bin-sec must be positive")
    hts_path = Path(args.hts)
    events_path = Path(args.events)
    telemetry_path = Path(args.telemetry)
    output_path = Path(args.output)
    report = _render(
        args=args,
        hts=_hts_features(hts_path),
        events=_jsonl(events_path),
        telemetry=_jsonl(telemetry_path),
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(report, encoding="utf-8")
    print(output_path)


if __name__ == "__main__":
    main()
