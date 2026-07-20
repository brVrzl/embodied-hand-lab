#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import write_json
from digital_twin.reconstruction.video import (
    calculate_metrics,
    discover_root_videos,
    iter_sampled_frames,
    probe_video,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Audit an iPhone/workspace video for reconstruction quality without extracting every source frame."
    )
    parser.add_argument("--input", type=Path, help="Input .MOV/.mov/.MP4/.mp4 video. Discovers one root video if omitted.")
    parser.add_argument("--search-root", type=Path, default=Path("."), help="Directory searched when --input is omitted.")
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/digital_twin/capture_audit"))
    parser.add_argument("--sample-fps", type=float, default=2.0, help="Audit sample rate; 2-5 fps is recommended.")
    parser.add_argument("--max-dimension", type=int, default=1280, help="Maximum sampled-frame dimension.")
    parser.add_argument("--start-sec", type=float, default=0.0)
    parser.add_argument("--end-sec", type=float)
    parser.add_argument("--contact-sheet-count", type=int, default=24)
    parser.add_argument("--manual-review", type=Path, help="Optional JSON/YAML engineering coverage review appended to report.")
    parser.add_argument("--dry-run", action="store_true", help="Validate/discover inputs and print the intended operation only.")
    return parser


def resolve_input(value: Path | None, root: Path) -> Path:
    if value is not None:
        if not value.is_file():
            raise FileNotFoundError(f"Input video does not exist: {value}")
        return value
    videos = discover_root_videos(root)
    if len(videos) != 1:
        names = ", ".join(str(path) for path in videos) or "none"
        raise ValueError(f"Expected exactly one root video when --input is omitted; found: {names}")
    return videos[0]


def _intervals(rows: list[dict[str, Any]], accepted: bool) -> list[dict[str, Any]]:
    selected = [row for row in rows if bool(row["accepted"]) is accepted]
    if not selected:
        return []
    groups: list[list[dict[str, Any]]] = [[selected[0]]]
    for row in selected[1:]:
        if int(row["sample_index"]) == int(groups[-1][-1]["sample_index"]) + 1:
            groups[-1].append(row)
        else:
            groups.append([row])
    return [
        {
            "start_sec": round(float(group[0]["source_timestamp_sec"]), 3),
            "end_sec": round(float(group[-1]["source_timestamp_sec"]), 3),
            "sample_count": len(group),
            "reasons": sorted({reason for row in group for reason in str(row["rejection_reason"]).split(";") if reason}),
        }
        for group in groups
    ]


def _contact_sheet(frames: list[tuple[float, np.ndarray]], destination: Path, count: int) -> None:
    if not frames:
        raise RuntimeError("No frames were decoded for the contact sheet.")
    indices = np.linspace(0, len(frames) - 1, min(count, len(frames)), dtype=int)
    tiles: list[np.ndarray] = []
    for index in indices:
        timestamp, frame = frames[int(index)]
        tile = cv2.resize(frame, (320, 180), interpolation=cv2.INTER_AREA)
        cv2.rectangle(tile, (0, 0), (105, 24), (0, 0, 0), -1)
        cv2.putText(tile, f"{timestamp:5.1f}s", (7, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        tiles.append(tile)
    columns = 4
    rows = []
    blank = np.zeros_like(tiles[0])
    for start in range(0, len(tiles), columns):
        row = tiles[start : start + columns]
        row.extend([blank] * (columns - len(row)))
        rows.append(np.hstack(row))
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(destination), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 90]):
        raise RuntimeError(f"Failed to write contact sheet: {destination}")


def run_audit(args: argparse.Namespace) -> dict[str, Any]:
    source = resolve_input(args.input, args.search_root)
    if args.sample_fps <= 0 or args.max_dimension < 64:
        raise ValueError("--sample-fps must be positive and --max-dimension must be at least 64.")
    metadata = probe_video(source)
    if args.dry_run:
        print(json.dumps({"input": str(source), "output_dir": str(args.output_dir), "metadata": metadata}, indent=2))
        return {"dry_run": True}
    output = args.output_dir
    sample_dir = output / "sampled_frames"
    sample_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    contact_frames: list[tuple[float, np.ndarray]] = []
    previous: np.ndarray | None = None
    for sample_index, (frame_index, timestamp, frame) in enumerate(
        iter_sampled_frames(
            source,
            sample_fps=args.sample_fps,
            max_dimension=args.max_dimension,
            start_sec=args.start_sec,
            end_sec=args.end_sec,
        )
    ):
        metrics = calculate_metrics(frame, previous)
        filename = f"frame_{sample_index:05d}_{timestamp:09.3f}s.jpg"
        if not cv2.imwrite(str(sample_dir / filename), frame, [cv2.IMWRITE_JPEG_QUALITY, 92]):
            raise RuntimeError(f"Could not write sampled frame: {filename}")
        row = {
            "sample_index": sample_index,
            "frame_filename": filename,
            "source_timestamp_sec": round(timestamp, 6),
            "source_frame_index": frame_index,
            "sharpness_laplacian_var": metrics.sharpness,
            "brightness_mean_0_255": metrics.brightness,
            "underexposed_ratio": metrics.underexposed_ratio,
            "overexposed_ratio": metrics.overexposed_ratio,
            "interframe_motion": metrics.motion,
            "duplicate_score": metrics.duplicate_score,
            "estimated_rotation_deg": metrics.rotation_deg,
            "feature_count": metrics.feature_count,
        }
        rows.append(row)
        contact_frames.append((timestamp, frame))
        previous = frame
    if len(rows) < 3:
        raise RuntimeError("Fewer than three sampled frames were decoded; capture audit is not meaningful.")
    sharpness_values = np.asarray([row["sharpness_laplacian_var"] for row in rows], dtype=float)
    blur_threshold = max(25.0, float(np.percentile(sharpness_values, 10)) * 0.75)
    for index, row in enumerate(rows):
        reasons: list[str] = []
        motion = row["interframe_motion"]
        rotation = row["estimated_rotation_deg"]
        if row["sharpness_laplacian_var"] < blur_threshold:
            reasons.append("low_sharpness")
        if row["underexposed_ratio"] > 0.25:
            reasons.append("underexposed")
        if row["overexposed_ratio"] > 0.25:
            reasons.append("overexposed")
        if motion is not None and motion < 0.002:
            reasons.append("duplicate_or_frozen")
        if motion is not None and motion > 0.35:
            reasons.append("rapid_motion")
        if rotation is not None and abs(rotation) > 15:
            reasons.append("rapid_rotation")
        if index > 0 and abs(row["brightness_mean_0_255"] - rows[index - 1]["brightness_mean_0_255"]) > 35:
            reasons.append("exposure_jump")
        if index > 0:
            ratio = row["sharpness_laplacian_var"] / max(rows[index - 1]["sharpness_laplacian_var"], 1e-9)
            if ratio > 4 or ratio < 0.25:
                reasons.append("major_focus_change")
        if motion is not None and motion > 0.18 and row["sharpness_laplacian_var"] < np.median(sharpness_values) * 0.4:
            reasons.append("possible_motion_smear_or_rolling_shutter")
        row["accepted"] = not reasons
        row["rejection_reason"] = ";".join(reasons)
    accepted_count = sum(bool(row["accepted"]) for row in rows)
    feature_median = float(np.median([row["feature_count"] for row in rows]))
    sharpness_median = float(np.median(sharpness_values))
    motion_values = [float(row["interframe_motion"]) for row in rows if row["interframe_motion"] is not None]
    usable = accepted_count >= max(20, int(0.6 * len(rows))) and feature_median >= 100
    colmap = usable and np.median(motion_values) > 0.01
    visual_reconstruction = colmap and len(rows) >= 60
    rejected = _intervals(rows, accepted=False)
    recommended = _intervals(rows, accepted=True)
    with (output / "quality_metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    write_json(output / "metadata.json", metadata)
    write_json(output / "rejected_intervals.json", {"intervals": rejected, "method": "sampled-frame heuristic"})
    write_json(output / "recommended_frame_ranges.json", {"intervals": recommended, "method": "sampled-frame heuristic"})
    _contact_sheet(contact_frames, output / "contact_sheet.jpg", args.contact_sheet_count)
    rapid_smear_count = sum("possible_motion_smear" in str(row["rejection_reason"]) for row in rows)
    duplicate_count = sum("duplicate_or_frozen" in str(row["rejection_reason"]) for row in rows)
    if args.manual_review:
        from digital_twin.io import load_structured
        review = load_structured(args.manual_review)
        bullets = "\n".join(f"- {item}" for item in review.get("findings", []))
        coverage_section = f"""Engineering review source: `{args.manual_review}`.

{bullets}

Additional capture recommendation: {review.get('additional_capture_recommendation', 'not supplied')}"""
    else:
        coverage_section = """The contact sheet must be reviewed for repeated observation of the robot base/mount, full tabletop and edges, underside or frame constraints, fixed camera support, occlusions, reflections and moving objects. Automated image statistics cannot reliably identify those objects, so their sufficiency is **not determined automatically**. Supply `--manual-review` to make the scene-specific decision reproducible."""
    recommended_text = ", ".join(
        f"{item['start_sec']:.3f}–{item['end_sec']:.3f} s" for item in recommended
    )
    rejected_text = ", ".join(
        f"{item['start_sec']:.3f} s ({'/'.join(item['reasons'])})" for item in rejected
    ) or "none"
    report = f"""# Workspace Capture Quality Audit

Input: `{source.name}`  
Audit sampling: {args.sample_fps:g} fps, maximum dimension {args.max_dimension}px  
Decoded samples: {len(rows)}; accepted by heuristic: {accepted_count}; rejected: {len(rows) - accepted_count}

## Decision

- Usable as-is for an initial reconstruction attempt: **{'yes' if usable else 'no'}**.
- Suitable for a COLMAP sparse-model attempt: **{'yes' if colmap else 'no'}**.
- Likely to support a useful Gaussian Splat/NeRF experiment after pose recovery: **{'yes' if visual_reconstruction else 'uncertain'}**.
- These are screening decisions, not guarantees of successful reconstruction or metric accuracy.

## Metrics

- Median sharpness (Laplacian variance on audit frames): {sharpness_median:.2f}; adaptive rejection threshold: {blur_threshold:.2f}.
- Median tracked scene-change score: {float(np.median(motion_values)):.4f}.
- Median detectable feature count (capped at 500): {feature_median:.0f}.
- Duplicate/frozen samples: {duplicate_count}; possible motion-smear/rolling-shutter samples: {rapid_smear_count}.
- Brightness range: {min(row['brightness_mean_0_255'] for row in rows):.1f}–{max(row['brightness_mean_0_255'] for row in rows):.1f} on an 8-bit scale.

## Intervals

Rejected sampled intervals are recorded in `rejected_intervals.json`; accepted runs are in `recommended_frame_ranges.json`. Rejection reasons include low sharpness, severe exposure, duplicate/frozen frames, rapid motion/rotation, focus changes and exposure jumps. Boundaries have only the audit sampling resolution and should be padded during final extraction.

Rejected sampled instants/runs: **{rejected_text}**. Recommended sampled runs: **{recommended_text}**. Inspect neighboring source frames before excluding padded intervals.

## Coverage and scene-specific review

{coverage_section}

## Known photogrammetry risks

Reflective metal/plastic, textureless tabletop regions, moving cables or people, auto-exposure/focus changes, physical robot motion, and persistent occlusion can break correspondence. Rolling shutter is only weakly screened through rapid-motion plus blur; this audit cannot measure row-wise camera motion. A visually plausible NeRF or Gaussian Splat would remain a rendering layer, not collision geometry or proof of scale/alignment.

## Metadata/privacy

Metadata backend: `{metadata['backend']}`. Location metadata presence: `{metadata['location_metadata_present']}`. No GPS coordinates are emitted. Sampled JPEGs are re-encoded without source EXIF/QuickTime metadata. Metadata limitations: {'; '.join(metadata['metadata_limitations']) or 'none'}.
"""
    (output / "report.md").write_text(report, encoding="utf-8")
    return {"usable": usable, "colmap": colmap, "visual_reconstruction": visual_reconstruction, "rows": rows}


def main() -> None:
    parser = _parser()
    args = parser.parse_args()
    try:
        result = run_audit(args)
    except (FileNotFoundError, ValueError, RuntimeError, cv2.error) as exc:
        parser.error(str(exc))
    if not args.dry_run:
        print(f"Capture audit written to: {args.output_dir}")
        print(f"Usable initial reconstruction input: {result['usable']}")


if __name__ == "__main__":
    main()
