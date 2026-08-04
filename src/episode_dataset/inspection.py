"""Offline episode inspection summaries, plots, and local playback."""

from __future__ import annotations

from collections import Counter
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np

from .episode import ACTION_ORDER
from .validation import load_canonical_rows, validate_episode


INSPECTION_SCHEMA_VERSION = "embodied_lab.episode_inspection.v1"


def _rate_summary(timestamps_ns: Iterable[int]) -> dict[str, Any]:
    sequence = np.asarray([int(value) for value in timestamps_ns], dtype=np.int64)
    regression_count = int(np.count_nonzero(np.diff(sequence) < 0)) if sequence.size > 1 else 0
    timestamps = np.asarray(sorted(set(sequence.tolist())), dtype=np.int64)
    if timestamps.size < 2:
        return {"sample_count": int(timestamps.size), "actual_hz": None}
    intervals = np.diff(timestamps).astype(np.float64) / 1e9
    median = float(np.median(intervals))
    return {
        "sample_count": int(timestamps.size),
        "actual_hz": float((timestamps.size - 1) / ((timestamps[-1] - timestamps[0]) / 1e9)),
        "median_interval_ms": median * 1000.0,
        "maximum_interval_ms": float(np.max(intervals) * 1000.0),
        "timestamp_regression_count": regression_count,
        "large_gap_count": int(np.count_nonzero(intervals > max(2.5 * median, median + 0.05))),
    }


def _raw_rows(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def _hand_lag_summary(
    target: np.ndarray, measured: np.ndarray, *, fps: float
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    maximum_lag = max(1, int(round(0.5 * fps)))
    for axis in range(target.shape[1]):
        target_delta = np.diff(target[:, axis])
        measured_delta = np.diff(measured[:, axis])
        if np.ptp(target[:, axis]) < 0.02 or np.count_nonzero(np.abs(target_delta) > 1e-4) < 3:
            result[f"H{axis + 1}"] = {"available": False, "reason": "insufficient_motion"}
            continue
        best: tuple[float, int] | None = None
        for lag in range(maximum_lag + 1):
            left = target_delta[: len(target_delta) - lag or None]
            right = measured_delta[lag:]
            if left.size < 5 or np.std(left) == 0.0 or np.std(right) == 0.0:
                continue
            correlation = float(np.corrcoef(left, right)[0, 1])
            if math.isfinite(correlation) and (best is None or correlation > best[0]):
                best = (correlation, lag)
        result[f"H{axis + 1}"] = (
            {"available": False, "reason": "no_finite_correlation"}
            if best is None
            else {
                "available": True,
                "estimated_lag_ms": best[1] * 1000.0 / fps,
                "correlation": best[0],
            }
        )
    return result


def _finite_array(
    rows: Iterable[Mapping[str, Any]],
    *keys: str,
) -> np.ndarray:
    values: list[list[float]] = []
    for row in rows:
        value: Any = row
        for key in keys:
            value = value[key]
        vector = np.asarray(value, dtype=np.float64)
        if vector.ndim != 1 or not np.all(np.isfinite(vector)):
            raise ValueError(f"inspection field {'.'.join(keys)} is invalid")
        values.append(vector.tolist())
    return np.asarray(values, dtype=np.float64)


def inspect_episode(episode: str | Path) -> dict[str, Any]:
    """Return a human-review-oriented summary after full payload validation."""

    episode_dir = Path(episode).resolve()
    metadata = json.loads((episode_dir / "metadata.json").read_text(encoding="utf-8"))
    validation = validate_episode(episode_dir, deep=True)
    report: dict[str, Any] = {
        "schema_version": INSPECTION_SCHEMA_VERSION,
        "episode": str(episode_dir),
        "validation": validation,
        "physically_validated": False,
    }
    if not validation["valid"]:
        report["inspection_available"] = False
        return report

    rows, errors = load_canonical_rows(episode_dir)
    if errors:
        raise ValueError(f"canonical rows changed after validation: {errors}")
    if not rows:
        report.update(
            {
                "inspection_available": False,
                "reason": "episode contains no canonical samples",
            }
        )
        return report

    timestamps = np.asarray(
        [float(row["timestamp"]) for row in rows], dtype=np.float64
    )
    arm_state = _finite_array(
        rows, "observation", "state", "arm_q_measured"
    )
    hand_state = _finite_array(rows, "observation", "state", "hand")
    arm_target = _finite_array(rows, "action", "arm_q_target")
    hand_target = _finite_array(rows, "action", "hand_target")
    arm_error = np.abs(arm_target - arm_state)
    hand_error = np.abs(hand_target - hand_state)
    sensor_rates: dict[str, Any] = {}
    for role in ("workspace", "wrist"):
        camera_rows = _raw_rows(episode_dir / "raw" / f"camera_{role}.jsonl")
        sensor_rates[f"camera_{role}"] = _rate_summary(
            row["host_monotonic_ns"] for row in camera_rows
        )
    jaka_rows = _raw_rows(episode_dir / "raw" / "jaka_state.jsonl")
    if jaka_rows:
        sensor_rates["jaka_state"] = _rate_summary(
            row["read_host_monotonic_ns"] for row in jaka_rows
        )
    rh56_rows = _raw_rows(episode_dir / "raw" / "rh56_feedback.jsonl")
    missing_required_fields = {"jaka_state": 0, "rh56_feedback": 0}
    for row in jaka_rows:
        if any(
            row.get(name) is None
            for name in (
                "read_host_monotonic_ns",
                "accepted_joint_target_rad",
                "measured_joint_position_rad",
                "commanded_tcp_pose_xyzw",
            )
        ):
            missing_required_fields["jaka_state"] += 1
    if rh56_rows:
        register_timestamps: dict[str, list[int]] = {
            name: []
            for name in ("ANGLE_ACT", "CURRENT", "FORCE_ACT", "ERROR", "STATUS")
        }
        for row in rh56_rows:
            values = row.get("hand_feedback_register_timestamps_ns", {})
            registers = row.get("rh56_registers", {})
            if (
                not isinstance(values, Mapping)
                or not isinstance(registers, Mapping)
                or any(
                    values.get(name) is None or registers.get(name) is None
                    for name in register_timestamps
                )
            ):
                missing_required_fields["rh56_feedback"] += 1
            for name in register_timestamps:
                value = values.get(name) if isinstance(values, Mapping) else None
                if isinstance(value, int):
                    register_timestamps[name].append(value)
        for name, values in register_timestamps.items():
            sensor_rates[f"rh56_{name.lower()}"] = _rate_summary(values)

    depth_valid_fraction: dict[str, float] = {}
    for role in ("workspace", "wrist"):
        valid = total = 0
        for row in rows:
            depth = np.load(
                episode_dir / row["observation"]["images"][role]["depth_raw"],
                allow_pickle=False,
            )
            valid += int(np.count_nonzero(depth))
            total += int(depth.size)
        depth_valid_fraction[role] = 0.0 if total == 0 else valid / total

    constant_channels = {
        "arm_q_measured": [
            f"J{index + 1}" for index, value in enumerate(np.ptp(arm_state, axis=0)) if value < 1e-8
        ],
        "arm_q_target": [
            f"J{index + 1}" for index, value in enumerate(np.ptp(arm_target, axis=0)) if value < 1e-8
        ],
        "hand_measured": [
            f"H{index + 1}" for index, value in enumerate(np.ptp(hand_state, axis=0)) if value < 1e-6
        ],
        "hand_target": [
            f"H{index + 1}" for index, value in enumerate(np.ptp(hand_target, axis=0)) if value < 1e-6
        ],
    }

    offsets_by_source: dict[str, list[float]] = {}
    for row in rows:
        offsets = row.get("timing", {}).get("signed_offsets_ns", {})
        if not isinstance(offsets, Mapping):
            continue
        for name, value in offsets.items():
            if isinstance(value, int):
                offsets_by_source.setdefault(str(name), []).append(
                    value / 1e6
                )
    offset_summary = {
        name: {
            "sample_count": len(values),
            "minimum_ms": min(values),
            "maximum_ms": max(values),
            "mean_ms": float(np.mean(values)),
            "p95_absolute_ms": float(np.percentile(np.abs(values), 95)),
        }
        for name, values in sorted(offsets_by_source.items())
        if values
    }
    statuses = Counter(str(row["action"]["arm_status"]) for row in rows)
    segment_modes = Counter(
        str(row["observation"]["state"]["control_segment_mode"])
        for row in rows
    )
    segment_ids = {
        int(row["observation"]["state"]["control_segment_id"])
        for row in rows
    }
    duration_s = (
        0.0 if len(timestamps) == 1 else float(timestamps[-1] - timestamps[0])
    )
    report.update(
        {
            "inspection_available": True,
            "sample_count": len(rows),
            "duration_s": duration_s,
            "action_order": list(ACTION_ORDER),
            "arm_action_status_counts": dict(sorted(statuses.items())),
            "control_segment_count": len(segment_ids),
            "control_segment_mode_sample_counts": dict(sorted(segment_modes.items())),
            "arm_command_state_error_rad": {
                "maximum": float(np.max(arm_error)),
                "mean": float(np.mean(arm_error)),
                "per_joint_maximum": np.max(arm_error, axis=0).tolist(),
            },
            "hand_command_state_error_rad": {
                "maximum": float(np.max(hand_error)),
                "mean": float(np.mean(hand_error)),
                "per_axis_maximum": np.max(hand_error, axis=0).tolist(),
            },
            "source_offset_summary": offset_summary,
            "sensor_rate_summary": sensor_rates,
            "depth_nonzero_fraction": depth_valid_fraction,
            "constant_channels": constant_channels,
            "missing_required_record_counts": missing_required_fields,
            "rh56_target_to_angle_act_lag": _hand_lag_summary(
                hand_target, hand_state, fps=float(metadata["dataset_fps"])
            ),
            "manual_review_required": [
                "confirm workspace and wrist camera role/order",
                "inspect RGB/depth content for occlusion, freeze, and corruption",
                "inspect command/state curves and held-rejected intervals",
                "confirm task outcome and assign an explicit success/failure label",
            ],
        }
    )
    return report


def write_inspection_plot(
    episode: str | Path,
    output: str | Path,
) -> Path:
    """Write an offline arm/hand/synchronization overview plot."""

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "inspection plotting requires matplotlib; install the development extra"
        ) from exc

    episode_dir = Path(episode).resolve()
    report = validate_episode(episode_dir, deep=True)
    if not report["valid"]:
        raise ValueError("cannot plot an invalid episode")
    rows, errors = load_canonical_rows(episode_dir)
    if errors or not rows:
        raise ValueError(f"cannot plot canonical rows: {errors or 'no samples'}")

    timestamps = np.asarray([row["timestamp"] for row in rows], dtype=np.float64)
    arm_state = _finite_array(
        rows, "observation", "state", "arm_q_measured"
    )
    arm_target = _finite_array(rows, "action", "arm_q_target")
    hand_state = _finite_array(rows, "observation", "state", "hand")
    hand_target = _finite_array(rows, "action", "hand_target")

    figure, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    for index in range(6):
        axes[0].plot(
            timestamps,
            arm_state[:, index],
            linewidth=1.0,
            label=f"J{index + 1} measured",
        )
        axes[0].plot(
            timestamps,
            arm_target[:, index],
            linewidth=0.8,
            linestyle="--",
            alpha=0.75,
        )
        axes[1].plot(
            timestamps,
            hand_state[:, index],
            linewidth=1.0,
            label=f"H{index + 1} observed",
        )
        axes[1].plot(
            timestamps,
            hand_target[:, index],
            linewidth=0.8,
            linestyle="--",
            alpha=0.75,
        )
    offset_names = sorted(
        {
            str(name)
            for row in rows
            for name, value in row.get("timing", {})
            .get("signed_offsets_ns", {})
            .items()
            if isinstance(value, int)
        }
    )
    for name in offset_names:
        offsets = [
            row.get("timing", {})
            .get("signed_offsets_ns", {})
            .get(name, math.nan)
            for row in rows
        ]
        axes[2].plot(
            timestamps,
            np.asarray(offsets, dtype=np.float64) / 1e6,
            linewidth=1.0,
            label=name,
        )
    axes[0].set_ylabel("arm q (rad)")
    axes[1].set_ylabel("hand axis (rad)")
    axes[2].set_ylabel("source offset (ms)")
    axes[2].set_xlabel("episode time (s)")
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right", ncol=3, fontsize=7)
    figure.suptitle(episode_dir.name)
    figure.tight_layout()

    path = Path(output).resolve()
    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.stem}.tmp{path.suffix}")
    try:
        figure.savefig(temporary, dpi=140)
        temporary.replace(path)
    finally:
        plt.close(figure)
        temporary.unlink(missing_ok=True)
    return path


def play_episode(
    episode: str | Path,
    *,
    playback_rate: float = 1.0,
) -> None:
    """Play RGB and raw-depth frames in a local OpenCV window."""

    if not math.isfinite(playback_rate) or playback_rate <= 0.0:
        raise ValueError("playback_rate must be finite and positive")
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "episode playback requires OpenCV; install the development extra"
        ) from exc

    episode_dir = Path(episode).resolve()
    validation = validate_episode(episode_dir, deep=True)
    if not validation["valid"]:
        raise ValueError("cannot play an invalid episode")
    rows, errors = load_canonical_rows(episode_dir)
    if errors or not rows:
        raise ValueError(f"cannot play canonical rows: {errors or 'no samples'}")

    depth_values: list[np.ndarray] = []
    for row in rows[:: max(1, len(rows) // 32)]:
        for role in ("workspace", "wrist"):
            path = episode_dir / row["observation"]["images"][role]["depth_raw"]
            depth = np.load(path, allow_pickle=False)
            sampled = depth[depth > 0][::100]
            if sampled.size:
                depth_values.append(sampled)
    if depth_values:
        combined = np.concatenate(depth_values)
        depth_low, depth_high = np.percentile(combined, [2, 98])
        if depth_high <= depth_low:
            depth_high = depth_low + 1.0
    else:
        depth_low, depth_high = 0.0, 1.0

    previous_timestamp: float | None = None
    window = f"Episode playback: {episode_dir.name}"
    try:
        for row in rows:
            panels: list[np.ndarray] = []
            for role in ("workspace", "wrist"):
                paths = row["observation"]["images"][role]
                rgb = np.load(episode_dir / paths["rgb"], allow_pickle=False)
                depth = np.load(
                    episode_dir / paths["depth_raw"], allow_pickle=False
                )
                valid = depth > 0
                scaled = np.zeros(depth.shape, dtype=np.uint8)
                scaled[valid] = np.clip(
                    (depth[valid].astype(np.float32) - depth_low)
                    * (255.0 / (depth_high - depth_low)),
                    0,
                    255,
                ).astype(np.uint8)
                depth_color = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
                rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                if depth_color.shape[:2] != rgb_bgr.shape[:2]:
                    depth_color = cv2.resize(
                        depth_color,
                        (rgb_bgr.shape[1], rgb_bgr.shape[0]),
                    )
                panels.append(np.hstack((rgb_bgr, depth_color)))
            width = max(panel.shape[1] for panel in panels)
            panels = [
                cv2.resize(
                    panel,
                    (width, round(panel.shape[0] * width / panel.shape[1])),
                )
                if panel.shape[1] != width
                else panel
                for panel in panels
            ]
            frame = np.vstack(panels)
            timestamp = float(row["timestamp"])
            cv2.putText(
                frame,
                (
                    f"frame={row['frame_index']} t={timestamp:.3f}s "
                    f"arm={row['action']['arm_status']} "
                    "(Esc/q stops)"
                ),
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(window, frame)
            delta = (
                1.0 / 30.0
                if previous_timestamp is None
                else max(0.001, timestamp - previous_timestamp)
            )
            previous_timestamp = timestamp
            key = cv2.waitKey(max(1, round(1000.0 * delta / playback_rate)))
            if key & 0xFF in {27, ord("q")}:
                break
    finally:
        cv2.destroyWindow(window)
