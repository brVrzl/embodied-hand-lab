"""Offline episode inspection summaries, plots, and local playback."""

from __future__ import annotations

from collections import Counter
import math
from pathlib import Path
import time
from typing import Any, Iterable, Mapping

import numpy as np

from .episode import ACTION_ORDER
from .validation import load_canonical_rows, validate_episode


INSPECTION_SCHEMA_VERSION = "embodied_lab.episode_inspection.v1"


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
