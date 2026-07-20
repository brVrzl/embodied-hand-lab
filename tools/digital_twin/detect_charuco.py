#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.calibration.charuco import (
    detection_to_dict,
    detect_charuco_instances,
    draw_detections,
    estimate_detection_pose,
    load_board_specs,
)
from digital_twin.io import load_structured, write_json
from digital_twin.reconstruction.video import iter_sampled_frames, probe_video


def contact_sheet(frames: list[tuple[float, np.ndarray]], path: Path, count: int = 24) -> None:
    if not frames:
        raise RuntimeError("No annotated frames are available for a contact sheet.")
    indices = np.linspace(0, len(frames) - 1, min(count, len(frames)), dtype=int)
    tiles = []
    for index in indices:
        timestamp, image = frames[int(index)]
        tile = cv2.resize(image, (400, 225), interpolation=cv2.INTER_AREA)
        cv2.rectangle(tile, (0, 0), (110, 24), (0, 0, 0), -1)
        cv2.putText(tile, f"{timestamp:5.1f}s", (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
        tiles.append(tile)
    blank = np.zeros_like(tiles[0])
    rows = []
    for start in range(0, len(tiles), 4):
        row = tiles[start:start+4] + [blank] * max(0, 4-len(tiles[start:start+4]))
        rows.append(np.hstack(row))
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 92]):
        raise RuntimeError(f"Could not write contact sheet: {path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Detect repeated-ID ChArUco board instances in a video using OpenCV's installed CharucoDetector API.")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--board-config", type=Path, default=Path("digital_twin/configs/charuco_boards.yaml"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sample-fps", type=float, default=3.0)
    parser.add_argument("--max-dimension", type=int, default=1920)
    parser.add_argument("--minimum-markers", type=int, default=6)
    parser.add_argument("--minimum-charuco-corners", type=int, default=8)
    parser.add_argument("--cluster-distance-factor", type=float, default=3.8)
    parser.add_argument("--intrinsics", type=Path, help="Optional JSON/YAML iPhone intrinsics for metric board poses.")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        metadata = probe_video(args.input)
        specs = load_board_specs(args.board_config)
        intrinsics = load_structured(args.intrinsics) if args.intrinsics else None
        camera_matrix = distortion = None
        if intrinsics:
            if "camera_matrix" in intrinsics:
                camera_matrix = np.asarray(intrinsics["camera_matrix"], dtype=float)
            else:
                camera_matrix = np.asarray([
                    [intrinsics["fx"], 0, intrinsics["cx"]],
                    [0, intrinsics["fy"], intrinsics["cy"]],
                    [0, 0, 1],
                ], dtype=float)
            distortion = np.asarray(intrinsics.get("distortion_coefficients", []), dtype=float)
        if args.sample_fps <= 0 or args.max_dimension < 64:
            raise ValueError("Sampling rate must be positive and maximum dimension at least 64.")
        if args.dry_run:
            print(json.dumps({"input": str(args.input), "metadata": metadata, "boards": [spec.name for spec in specs]}, indent=2)); return
        annotated_dir = args.output_dir / "annotated_frames"
        annotated_dir.mkdir(parents=True, exist_ok=True)
        manifest_frames = []
        annotated = []
        for sample_index, (frame_index, timestamp, frame) in enumerate(
            iter_sampled_frames(args.input, sample_fps=args.sample_fps, max_dimension=args.max_dimension)
        ):
            detections, marker_corners, marker_ids, rejected = detect_charuco_instances(
                frame,
                specs,
                minimum_markers=args.minimum_markers,
                minimum_charuco_corners=args.minimum_charuco_corners,
                cluster_distance_factor=args.cluster_distance_factor,
            )
            rendered = draw_detections(frame, detections)
            filename = f"charuco_{sample_index:05d}_{timestamp:09.3f}s.jpg"
            cv2.imwrite(str(annotated_dir / filename), rendered, [cv2.IMWRITE_JPEG_QUALITY, 92])
            annotated.append((timestamp, rendered))
            serialized_detections = []
            for item in detections:
                serialized = detection_to_dict(item)
                if camera_matrix is not None and len(item.candidate_board_names) == 1:
                    spec = next(spec for spec in specs if spec.name == item.candidate_board_names[0])
                    serialized["estimated_pose"] = estimate_detection_pose(item, spec, camera_matrix, distortion)
                serialized_detections.append(serialized)
            manifest_frames.append({
                "video_name": args.input.name,
                "frame_name": filename,
                "source_frame_index": frame_index,
                "timestamp_sec": round(timestamp, 6),
                "raw_detected_markers": 0 if marker_ids is None else len(marker_ids),
                "raw_rejected_marker_candidates": len(rejected),
                "detections": serialized_detections,
            })
        accepted = [detection for frame in manifest_frames for detection in frame["detections"] if detection["accepted"]]
        board_counts = {spec.name: sum(detection["board_name"] == spec.name for detection in accepted) for spec in specs}
        ambiguous = sum(detection["board_name"] is None for detection in accepted)
        complete = sum(detection["detected_charuco_corners"] == specs[0].internal_corner_count for detection in accepted)
        foreshortening = [detection["foreshortening_ratio"] for detection in accepted if detection["foreshortening_ratio"] is not None]
        coverage = [detection["coverage_ratio"] for detection in accepted]
        summary = {
            "video_name": args.input.name,
            "metadata": metadata,
            "opencv_version": cv2.__version__,
            "opencv_api": "cv2.aruco.ArucoDetector + cv2.aruco.CharucoDetector.detectBoard",
            "sample_fps": args.sample_fps,
            "sampled_frames": len(manifest_frames),
            "frames_with_accepted_detection": sum(any(d["accepted"] for d in frame["detections"]) for frame in manifest_frames),
            "accepted_board_observations": len(accepted),
            "complete_24_corner_observations": complete,
            "identity_counts": board_counts,
            "ambiguous_identity_observations": ambiguous,
            "marker_ids_unique_across_boards": False,
            "observed_marker_ids": sorted({identifier for detection in accepted for identifier in detection["marker_ids"]}),
            "corner_coverage": {"min_board_image_ratio": min(coverage) if coverage else None, "max_board_image_ratio": max(coverage) if coverage else None},
            "view_angle_diversity_proxy": {"foreshortening_min": min(foreshortening) if foreshortening else None, "foreshortening_max": max(foreshortening) if foreshortening else None, "note": "homography local-axis ratio; not a calibrated pose angle"},
            "intrinsics_available": camera_matrix is not None,
            "metric_pose_status": "estimated_for_unambiguous_board_candidates" if camera_matrix is not None else "not_estimated_without_iPhone_intrinsics",
            "physical_scale_internal_consistency": "requires SfM or calibrated intrinsics; pixel scale alone is depth-dependent",
        }
        manifest = {"summary": summary, "frames": manifest_frames}
        write_json(args.output_dir / "detection_manifest.json", manifest)
        contact_sheet(annotated, args.output_dir / "annotated_contact_sheet.jpg")
        report = f"""# ChArUco Detection Report — {args.input.name}

- Sampled frames: {summary['sampled_frames']} at {args.sample_fps:g} fps.
- Frames with accepted detections: {summary['frames_with_accepted_detection']}.
- Accepted board observations: {summary['accepted_board_observations']}; complete 24-corner observations: {complete}.
- Identity candidates: {board_counts}; ambiguous observations: {ambiguous}.
- Observed marker IDs: {summary['observed_marker_ids']}.
- OpenCV API: {summary['opencv_api']} on {cv2.__version__}.
- Pose/metric status: {summary['metric_pose_status']}.

All visible patterns reuse IDs 0–16. Identity labels based on same-frame relative square scale are candidates, not independent proof. The annotated contact sheet must be reviewed for cluster merging, fixed-board behavior, occlusion, and whether the apparent third pattern is a second A4 print.
"""
        (args.output_dir / "report.md").write_text(report, encoding="utf-8")
        print(f"ChArUco detections written to: {args.output_dir}")
    except (FileNotFoundError, KeyError, TypeError, ValueError, RuntimeError, cv2.error) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
