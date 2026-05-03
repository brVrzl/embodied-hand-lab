from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def _array(value: Any, *, length: int | None = None) -> np.ndarray:
    array = np.asarray(value if value is not None else [], dtype=np.float32).reshape(-1)
    if length is not None and array.size < length:
        array = np.pad(array, (0, length - array.size))
    if length is not None:
        array = array[:length]
    return array


def _min_max(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"min": None, "max": None}
    array = np.asarray(values, dtype=np.float32)
    return {"min": float(np.min(array)), "max": float(np.max(array))}


def _min_max_mean(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"min": None, "max": None, "mean": None}
    array = np.asarray(values, dtype=np.float32)
    return {"min": float(np.min(array)), "max": float(np.max(array)), "mean": float(np.mean(array))}


def _object_height(sample: dict[str, Any]) -> float | None:
    state = (sample.get("observation") or {}).get("state") or {}
    object_pose = state.get("object_pose")
    if object_pose is None:
        object_pose = ((sample.get("observation") or {}).get("extra_observation") or {}).get("obj_pose")
    if object_pose is None:
        return None
    array = _array(object_pose)
    if array.size < 3:
        return None
    return float(array[2])


def _episode_quality(*, success: bool, failure_mode: str, final_height: float | None) -> str:
    if not success or failure_mode != "none":
        return "intended_failure"
    if final_height is None:
        return "near_failure"
    if final_height >= 0.08:
        return "strong_success"
    if final_height >= 0.04:
        return "weak_success"
    return "near_failure"


def _load_samples(export_root: Path) -> list[dict[str, Any]]:
    samples_path = export_root / "samples.jsonl"
    if not samples_path.exists():
        raise FileNotFoundError(f"Missing {samples_path}")
    samples: list[dict[str, Any]] = []
    with samples_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                samples.append(json.loads(line))
    if not samples:
        raise RuntimeError(f"No samples found in {samples_path}")
    return samples


def _load_manual_review(path: str | Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    review_path = Path(path).resolve()
    if not review_path.exists():
        raise FileNotFoundError(f"Missing manual review file: {review_path}")
    with review_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    episodes = data.get("episodes")
    if not isinstance(episodes, dict):
        raise ValueError(f"manual review file must contain an 'episodes' mapping: {review_path}")
    return {"path": str(review_path), "episodes": episodes}


def _manual_review_summary(
    *,
    manual_review: dict[str, Any] | None,
    episode_ids: list[str],
    final_heights_by_episode: dict[str, float | None],
) -> dict[str, Any] | None:
    if manual_review is None:
        return None
    review_entries = manual_review["episodes"]
    quality_counts = Counter()
    use_for_bc_episode_list: list[str] = []
    use_for_bc_heights: list[float] = []
    missing_review_entries: list[str] = []

    for episode_id in episode_ids:
        entry = review_entries.get(episode_id)
        if not isinstance(entry, dict):
            missing_review_entries.append(episode_id)
            quality = "unreviewed"
            use_for_bc = False
        else:
            quality = str(entry.get("manual_quality", "unreviewed"))
            use_for_bc = bool(entry.get("use_for_bc", False))
        quality_counts[quality] += 1
        if use_for_bc:
            use_for_bc_episode_list.append(episode_id)
            final_height = final_heights_by_episode.get(episode_id)
            if final_height is not None:
                use_for_bc_heights.append(final_height)

    reviewed_count = sum(count for quality, count in quality_counts.items() if quality != "unreviewed")
    unreviewed_count = len(episode_ids) - reviewed_count
    return {
        "path": manual_review["path"],
        "reviewed_count": reviewed_count,
        "unreviewed_count": unreviewed_count,
        "manual_strong_success_count": quality_counts.get("strong_success", 0),
        "manual_weak_success_count": quality_counts.get("weak_success", 0),
        "manual_near_failure_count": quality_counts.get("near_failure", 0),
        "manual_invalid_count": quality_counts.get("invalid", 0),
        "manual_quality_distribution": dict(quality_counts),
        "use_for_bc_count": len(use_for_bc_episode_list),
        "use_for_bc_episode_list": use_for_bc_episode_list,
        "use_for_bc_final_height": _min_max_mean(use_for_bc_heights),
        "missing_review_entries": missing_review_entries,
    }


def inspect_export(
    export_root: str | Path,
    out_dir: str | Path,
    *,
    success_height_threshold: float = 0.08,
    manual_review: str | Path | None = None,
) -> dict[str, Any]:
    export_root = Path(export_root).resolve()
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    samples = _load_samples(export_root)
    manual_review_data = _load_manual_review(manual_review)
    by_episode: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for sample in samples:
        by_episode[str(sample["episode_id"])].append(sample)

    hand_cmd_values: list[float] = []
    hand_state_values: list[float] = []
    hand_error_values: list[float] = []
    hand_delta_cmd_abs: list[float] = []
    hand_delta_state_abs: list[float] = []
    hand_delta_state_raw_abs: list[float] = []
    ee_translation_delta_norm: list[float] = []
    ee_delta_xyz_abs: list[float] = []
    ee_rotation_delta: list[float] = []
    object_heights: list[float] = []
    object_final_heights: list[float] = []

    episode_frames: dict[str, int] = {}
    success_by_episode: dict[str, bool] = {}
    failure_mode_by_episode: dict[str, str] = {}
    success_final_heights: list[float] = []
    failure_final_heights: list[float] = []
    success_episode_lengths: list[float] = []
    failure_episode_lengths: list[float] = []
    suspicious_success_episodes: list[dict[str, Any]] = []
    final_heights_by_episode: dict[str, float | None] = {}
    episode_quality: dict[str, list[dict[str, Any]]] = {
        "strong_success": [],
        "weak_success": [],
        "near_failure": [],
        "intended_failure": [],
    }

    for episode_id, episode_samples in sorted(by_episode.items()):
        episode_samples.sort(key=lambda item: int(item["frame_index"]))
        episode_frames[episode_id] = len(episode_samples)
        success_by_episode[episode_id] = bool(episode_samples[-1]["episode_success"])
        failure_mode_by_episode[episode_id] = str(episode_samples[-1]["episode_failure_mode"])
        final_height = _object_height(episode_samples[-1])
        final_heights_by_episode[episode_id] = final_height
        if success_by_episode[episode_id]:
            success_episode_lengths.append(float(len(episode_samples)))
        else:
            failure_episode_lengths.append(float(len(episode_samples)))
        if final_height is not None:
            object_final_heights.append(final_height)
            if success_by_episode[episode_id]:
                success_final_heights.append(final_height)
                if final_height < success_height_threshold:
                    suspicious_success_episodes.append(
                        {
                            "episode_id": episode_id,
                            "final_object_height": final_height,
                            "threshold": success_height_threshold,
                        }
                    )
            else:
                failure_final_heights.append(final_height)
        quality = _episode_quality(
            success=success_by_episode[episode_id],
            failure_mode=failure_mode_by_episode[episode_id],
            final_height=final_height,
        )
        episode_quality[quality].append(
            {
                "episode_id": episode_id,
                "final_object_height": final_height,
                "episode_success": success_by_episode[episode_id],
                "failure_mode": failure_mode_by_episode[episode_id],
            }
        )

        for sample in episode_samples:
            state = sample["observation"]["state"]
            action = sample["action"]
            hand_cmd = _array(action.get("hand_cmd"), length=6)
            hand_state = _array(state.get("hand_state"), length=6)
            hand_error = _array(state.get("hand_error"), length=6)
            hand_delta_cmd = _array(action.get("hand_delta_cmd"), length=6)
            hand_delta_state = _array(action.get("hand_delta_state"), length=6)
            hand_delta_state_raw = _array(action.get("hand_delta_state_raw"), length=6)
            ee_delta = _array(action.get("ee_delta"), length=6)

            hand_cmd_values.extend(hand_cmd.tolist())
            hand_state_values.extend(hand_state.tolist())
            hand_error_values.extend(hand_error.tolist())
            hand_delta_cmd_abs.extend(np.abs(hand_delta_cmd).tolist())
            hand_delta_state_abs.extend(np.abs(hand_delta_state).tolist())
            hand_delta_state_raw_abs.extend(np.abs(hand_delta_state_raw).tolist())
            ee_delta_xyz_abs.extend(np.abs(ee_delta[:3]).tolist())
            ee_translation_delta_norm.append(float(np.linalg.norm(ee_delta[:3])))
            ee_rotation_delta.append(float(np.linalg.norm(ee_delta[3:])))
            height = _object_height(sample)
            if height is not None:
                object_heights.append(height)

    failure_mode_distribution = dict(Counter(failure_mode_by_episode.values()))
    success_count = sum(1 for value in success_by_episode.values() if value)
    failure_count = len(success_by_episode) - success_count
    suspicious_quality_episodes = episode_quality["weak_success"] + episode_quality["near_failure"]
    manual_summary = _manual_review_summary(
        manual_review=manual_review_data,
        episode_ids=sorted(by_episode),
        final_heights_by_episode=final_heights_by_episode,
    )
    summary = {
        "export_root": str(export_root),
        "episode_count": len(by_episode),
        "sample_count": len(samples),
        "success_episode_count": success_count,
        "failure_episode_count": failure_count,
        "failure_mode_distribution": failure_mode_distribution,
        "episode_frame_counts": episode_frames,
        "hand_cmd": _min_max(hand_cmd_values),
        "hand_state": _min_max(hand_state_values),
        "hand_error": _min_max(hand_error_values),
        "max_abs_hand_delta_cmd": max(hand_delta_cmd_abs) if hand_delta_cmd_abs else None,
        "max_abs_hand_delta_state": max(hand_delta_state_abs) if hand_delta_state_abs else None,
        "max_abs_hand_delta_state_raw": max(hand_delta_state_raw_abs) if hand_delta_state_raw_abs else None,
        "max_abs_ee_delta_xyz": max(ee_delta_xyz_abs) if ee_delta_xyz_abs else None,
        "max_ee_translation_delta_norm": max(ee_translation_delta_norm) if ee_translation_delta_norm else None,
        "max_ee_rotation_delta": max(ee_rotation_delta) if ee_rotation_delta else None,
        "object_height": {
            **_min_max(object_heights),
            "final_min": min(object_final_heights) if object_final_heights else None,
            "final_max": max(object_final_heights) if object_final_heights else None,
        },
        "success_object_final_height": _min_max_mean(success_final_heights),
        "failure_object_final_height": _min_max_mean(failure_final_heights),
        "success_episode_length": _min_max_mean(success_episode_lengths),
        "failure_episode_length": _min_max_mean(failure_episode_lengths),
        "success_height_threshold": success_height_threshold,
        "suspicious_success_episodes": suspicious_success_episodes,
        "episode_quality_counts": {quality: len(rows) for quality, rows in episode_quality.items()},
        "episode_quality": episode_quality,
        "suspicious_quality_episodes": suspicious_quality_episodes,
        "manual_review": manual_summary,
    }

    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    _write_markdown(summary, out_dir / "summary.md")
    _write_plots(by_episode, out_dir)
    return summary


def _write_markdown(summary: dict[str, Any], path: Path) -> None:
    lines = [
        "# Episode Export Inspection",
        "",
        f"- export_root: `{summary['export_root']}`",
        f"- episodes: {summary['episode_count']}",
        f"- samples: {summary['sample_count']}",
        f"- success episodes: {summary['success_episode_count']}",
        f"- failure episodes: {summary['failure_episode_count']}",
        f"- failure_mode_distribution: `{summary['failure_mode_distribution']}`",
        f"- hand_cmd min/max: `{summary['hand_cmd']}`",
        f"- hand_state min/max: `{summary['hand_state']}`",
        f"- hand_error min/max: `{summary['hand_error']}`",
        f"- max_abs_hand_delta_cmd: {summary['max_abs_hand_delta_cmd']}",
        f"- max_abs_hand_delta_state: {summary['max_abs_hand_delta_state']}",
        f"- max_abs_hand_delta_state_raw: {summary['max_abs_hand_delta_state_raw']}",
        f"- max_abs_ee_delta_xyz: {summary['max_abs_ee_delta_xyz']}",
        f"- max_ee_translation_delta_norm: {summary['max_ee_translation_delta_norm']}",
        f"- max_ee_rotation_delta: {summary['max_ee_rotation_delta']}",
        f"- object_height: `{summary['object_height']}`",
        f"- success_object_final_height: `{summary['success_object_final_height']}`",
        f"- failure_object_final_height: `{summary['failure_object_final_height']}`",
        f"- success_episode_length: `{summary['success_episode_length']}`",
        f"- failure_episode_length: `{summary['failure_episode_length']}`",
        f"- success_height_threshold: {summary['success_height_threshold']}",
        f"- suspicious_success_episodes: `{summary['suspicious_success_episodes']}`",
        f"- episode_quality_counts: `{summary['episode_quality_counts']}`",
        f"- suspicious_quality_episodes: `{summary['suspicious_quality_episodes']}`",
        "",
        "## Manual Replay Review Criteria",
        "",
        "- strong_success: object is held stably by the hand, clearly lifted from the table, and still held at episode end.",
        "- weak_success: object is briefly lifted or slightly raised, but unstable, too low, or unreliable at episode end.",
        "- near_failure: object is not meaningfully lifted, or only slides/gets lightly disturbed near the table.",
        "- invalid: penetration, teleportation, obvious kinematic artifact, or corrupted data.",
        "",
    ]
    if summary.get("manual_review") is not None:
        manual = summary["manual_review"]
        lines.extend(
            [
                "## Manual Review",
                "",
                f"- manual_review_path: `{manual['path']}`",
                f"- reviewed_count: {manual['reviewed_count']}",
                f"- unreviewed_count: {manual['unreviewed_count']}",
                f"- manual_strong_success_count: {manual['manual_strong_success_count']}",
                f"- manual_weak_success_count: {manual['manual_weak_success_count']}",
                f"- manual_near_failure_count: {manual['manual_near_failure_count']}",
                f"- manual_invalid_count: {manual['manual_invalid_count']}",
                f"- use_for_bc_count: {manual['use_for_bc_count']}",
                f"- use_for_bc_episode_list: `{manual['use_for_bc_episode_list']}`",
                f"- use_for_bc_final_height: `{manual['use_for_bc_final_height']}`",
                f"- missing_review_entries: `{manual['missing_review_entries']}`",
                "",
            ]
        )
    lines.extend(
        [
        "## Episode Quality",
        "",
        ]
    )
    for quality, rows in summary["episode_quality"].items():
        lines.append(f"### {quality}")
        for row in rows:
            lines.append(f"- {row['episode_id']}: final_object_height={row['final_object_height']}")
        lines.append("")
    lines.extend(
        [
            "## Episode Frame Counts",
            "",
        ]
    )
    for episode_id, count in summary["episode_frame_counts"].items():
        lines.append(f"- {episode_id}: {count}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_plots(by_episode: dict[str, list[dict[str, Any]]], out_dir: Path) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - optional plotting path
        (out_dir / "plots_skipped.txt").write_text(f"matplotlib unavailable: {exc}\n", encoding="utf-8")
        return

    series_specs = {
        "hand_cmd_mean": lambda sample: float(np.mean(_array(sample["action"].get("hand_cmd"), length=6))),
        "hand_state_mean": lambda sample: float(np.mean(_array(sample["observation"]["state"].get("hand_state"), length=6))),
        "hand_error_mean": lambda sample: float(np.mean(_array(sample["observation"]["state"].get("hand_error"), length=6))),
        "ee_delta_translation_norm": lambda sample: float(np.linalg.norm(_array(sample["action"].get("ee_delta"), length=6)[:3])),
        "object_height": lambda sample: _object_height(sample),
    }
    for name, getter in series_specs.items():
        fig, ax = plt.subplots(figsize=(9, 5))
        for episode_id, episode_samples in sorted(by_episode.items()):
            episode_samples.sort(key=lambda item: int(item["frame_index"]))
            xs = [int(sample["frame_index"]) for sample in episode_samples]
            ys = [getter(sample) for sample in episode_samples]
            xs = [x for x, y in zip(xs, ys, strict=True) if y is not None]
            ys = [y for y in ys if y is not None]
            if xs and ys:
                ax.plot(xs, ys, linewidth=1.0, alpha=0.65, label=episode_id)
        ax.set_title(name)
        ax.set_xlabel("frame_index")
        ax.grid(True, alpha=0.25)
        if len(by_episode) <= 12:
            ax.legend(fontsize=6, loc="best")
        fig.tight_layout()
        fig.savefig(out_dir / f"{name}.png", dpi=140)
        plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect a structured JAKA+RH56 episode export.")
    parser.add_argument("--export-root", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--success-height-threshold", type=float, default=0.08)
    parser.add_argument("--manual-review", default=None)
    args = parser.parse_args()
    summary = inspect_export(
        args.export_root,
        args.out_dir,
        success_height_threshold=args.success_height_threshold,
        manual_review=args.manual_review,
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
