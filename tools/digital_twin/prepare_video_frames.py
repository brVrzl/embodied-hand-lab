#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import cv2

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import write_json
from digital_twin.reconstruction.video import calculate_metrics, iter_sampled_frames, probe_video


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Prepare a conservative, timestamped reconstruction frame set.")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sample-fps", type=float, default=3.0)
    parser.add_argument("--max-dimension", type=int, default=1920)
    parser.add_argument("--start-sec", type=float, default=0.0)
    parser.add_argument("--end-sec", type=float)
    parser.add_argument("--min-sharpness", type=float, default=30.0)
    parser.add_argument("--duplicate-motion-threshold", type=float, default=0.002)
    parser.add_argument("--rapid-motion-threshold", type=float, default=0.35)
    parser.add_argument("--keep-rejected", action="store_true", help="Write rejected images under rejected_frames/.")
    parser.add_argument("--overwrite", action="store_true", help="Replace only this tool's prior manifests/frame_*.jpg outputs.")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    try:
        metadata = probe_video(args.input)
        if args.sample_fps <= 0 or args.max_dimension < 64:
            raise ValueError("Sampling rate must be positive and maximum dimension at least 64.")
        if args.dry_run:
            print(json.dumps({"input": str(args.input), "output_dir": str(args.output_dir), "metadata": metadata}, indent=2))
            return
        accepted_dir = args.output_dir / "images"
        rejected_dir = args.output_dir / "rejected_frames"
        manifest_paths = [args.output_dir / "frame_manifest.csv", args.output_dir / "frame_manifest.json"]
        if any(path.exists() for path in manifest_paths) and not args.overwrite:
            raise ValueError("Frame manifest already exists; choose another output or pass --overwrite.")
        if args.overwrite:
            for directory in (accepted_dir, rejected_dir):
                if directory.is_dir():
                    for path in directory.glob("frame_*.jpg"):
                        path.unlink()
            for path in manifest_paths:
                if path.exists():
                    path.unlink()
        accepted_dir.mkdir(parents=True, exist_ok=True)
        if args.keep_rejected:
            rejected_dir.mkdir(parents=True, exist_ok=True)
        rows = []
        previous = None
        for sample_index, (frame_index, timestamp, frame) in enumerate(
            iter_sampled_frames(args.input, sample_fps=args.sample_fps, max_dimension=args.max_dimension,
                                start_sec=args.start_sec, end_sec=args.end_sec)
        ):
            metrics = calculate_metrics(frame, previous)
            reasons = []
            if metrics.sharpness < args.min_sharpness:
                reasons.append("low_sharpness")
            if metrics.motion is not None and metrics.motion < args.duplicate_motion_threshold:
                reasons.append("duplicate")
            if metrics.motion is not None and metrics.motion > args.rapid_motion_threshold:
                reasons.append("rapid_motion")
            accepted = not reasons
            filename = f"frame_{sample_index:06d}.jpg"
            destination = accepted_dir / filename if accepted else rejected_dir / filename
            if accepted or args.keep_rejected:
                if not cv2.imwrite(str(destination), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                    raise RuntimeError(f"Failed to write: {destination}")
            rows.append({
                "frame_filename": filename,
                "source_timestamp_sec": round(timestamp, 6),
                "source_frame_index": frame_index,
                "sharpness_metric": metrics.sharpness,
                "brightness_metric": metrics.brightness,
                "motion_metric": metrics.motion,
                "accepted": accepted,
                "rejection_reason": ";".join(reasons),
            })
            previous = frame
        if not rows:
            raise RuntimeError("No frames could be decoded.")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        with (args.output_dir / "frame_manifest.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        write_json(args.output_dir / "frame_manifest.json", {
            "source_video": args.input.name,
            "source_metadata": metadata,
            "derived_media_metadata_policy": "re-encoded images contain no copied source EXIF/QuickTime metadata",
            "frames": rows,
        })
        print(f"Prepared {sum(row['accepted'] for row in rows)} accepted frames in: {accepted_dir}")
    except (FileNotFoundError, ValueError, RuntimeError, cv2.error) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
