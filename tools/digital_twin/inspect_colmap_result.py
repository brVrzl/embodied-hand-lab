#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import load_structured, write_json


def _non_comment_lines(path: Path, *, preserve_empty: bool = False) -> list[str]:
    result = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line.startswith("#"):
            continue
        if line or preserve_empty:
            result.append(line)
    return result


def _parse_cameras(path: Path) -> list[dict]:
    cameras = []
    for line in _non_comment_lines(path):
        fields = line.split()
        cameras.append(
            {
                "camera_id": int(fields[0]),
                "model": fields[1],
                "width": int(fields[2]),
                "height": int(fields[3]),
                "parameters": [float(value) for value in fields[4:]],
            }
        )
    return cameras


def _parse_images(path: Path) -> list[dict]:
    # COLMAP images.txt is strictly two records per image. The second record may
    # be empty, so it must not be discarded while pairing the lines.
    lines = _non_comment_lines(path, preserve_empty=True)
    if len(lines) % 2:
        raise ValueError(f"Malformed COLMAP images.txt (odd data-line count): {path}")
    images = []
    for index in range(0, len(lines), 2):
        pose_fields = lines[index].split()
        if len(pose_fields) < 10:
            raise ValueError(f"Malformed COLMAP image pose line: {lines[index]!r}")
        point_fields = lines[index + 1].split()
        if len(point_fields) % 3:
            raise ValueError(f"Malformed COLMAP image observations for {pose_fields[9]}")
        point_ids = [int(point_fields[offset + 2]) for offset in range(0, len(point_fields), 3)]
        images.append(
            {
                "image_id": int(pose_fields[0]),
                "qvec_wxyz": [float(value) for value in pose_fields[1:5]],
                "tvec": [float(value) for value in pose_fields[5:8]],
                "camera_id": int(pose_fields[8]),
                "name": pose_fields[9],
                "observation_count": len(point_ids),
                "triangulated_observation_count": sum(value >= 0 for value in point_ids),
                "_triangulated_point_ids": {value for value in point_ids if value >= 0},
            }
        )
    return images


def _parse_points(path: Path) -> list[dict]:
    points = []
    for line in _non_comment_lines(path):
        fields = line.split()
        if len(fields) < 8 or (len(fields) - 8) % 2:
            raise ValueError(f"Malformed COLMAP point3D line: {line[:120]!r}")
        points.append(
            {
                "point3D_id": int(fields[0]),
                "xyz": [float(value) for value in fields[1:4]],
                "error_px": float(fields[7]),
                "track_length": (len(fields) - 8) // 2,
            }
        )
    return points


def _frame_number(name: str) -> int | None:
    match = re.search(r"(?:frame[_-]?)(\d+)", Path(name).stem, flags=re.IGNORECASE)
    return int(match.group(1)) if match else None


def _trajectory_gaps(images: list[dict]) -> list[dict]:
    by_folder: dict[str, list[int]] = {}
    for image in images:
        number = _frame_number(image["name"])
        if number is not None:
            by_folder.setdefault(str(Path(image["name"]).parent), []).append(number)
    gaps = []
    for folder, numbers in by_folder.items():
        ordered = sorted(set(numbers))
        for first, second in zip(ordered, ordered[1:]):
            if second > first + 1:
                gaps.append({"folder": folder, "after_frame": first, "before_frame": second, "missing_count": second - first - 1})
    return gaps


def _loop_closure_evidence(images: list[dict]) -> dict:
    ordered = sorted(images, key=lambda item: (_frame_number(item["name"]) is None, _frame_number(item["name"]) or 0))
    if len(ordered) < 4:
        return {"status": "insufficient_images", "shared_sparse_points_first_last_windows": None}
    window = min(5, max(2, len(ordered) // 10))
    first_points = set().union(*(item["_triangulated_point_ids"] for item in ordered[:window]))
    last_points = set().union(*(item["_triangulated_point_ids"] for item in ordered[-window:]))
    shared = len(first_points & last_points)
    return {
        "status": "shared_sparse_structure_detected" if shared >= 20 else "no_strong_first_last_sparse_overlap",
        "window_size": window,
        "shared_sparse_points_first_last_windows": shared,
        "note": "Shared sparse points are loop-closure evidence, not a physical trajectory-closure guarantee.",
    }


def inspect_text_model(model: Path, extracted_image_count: int | None = None, extracted_image_names: list[str] | None = None) -> dict:
    cameras = _parse_cameras(model / "cameras.txt")
    images = _parse_images(model / "images.txt")
    points = _parse_points(model / "points3D.txt")
    track_lengths = np.asarray([item["track_length"] for item in points], dtype=float)
    errors = np.asarray([item["error_px"] for item in points], dtype=float)
    registered = len(images)
    registered_names = [item["name"] for item in images]
    missing_registered = sorted(set(extracted_image_names or []) - set(registered_names))
    filename_gaps = _trajectory_gaps(images)
    return {
        "format": "text",
        "camera_count": len(cameras),
        "cameras": cameras,
        "extracted_image_count": extracted_image_count,
        "registered_image_count": registered,
        "registration_ratio": registered / extracted_image_count if extracted_image_count else None,
        "point3D_count": len(points),
        "registered_images": registered_names,
        "unregistered_extracted_images": missing_registered if extracted_image_names is not None else None,
        "mean_track_length": float(track_lengths.mean()) if len(track_lengths) else None,
        "median_track_length": float(np.median(track_lengths)) if len(track_lengths) else None,
        "total_sparse_observations": int(track_lengths.sum()) if len(track_lengths) else 0,
        "mean_observations_per_registered_image": float(track_lengths.sum() / registered) if registered else None,
        "mean_reprojection_error_px": float(errors.mean()) if len(errors) else None,
        "median_reprojection_error_px": float(np.median(errors)) if len(errors) else None,
        "trajectory_filename_gaps": filename_gaps,
        "trajectory_continuity_status": (
            "all_extracted_images_registered" if extracted_image_names is not None and not missing_registered
            else "some_extracted_images_unregistered" if extracted_image_names is not None
            else "continuous_registered_frame_sequence" if not filename_gaps
            else "registered_sequence_has_filename_gaps"
        ),
        "loop_closure_evidence": _loop_closure_evidence(images),
    }


def inspect_model(model: Path, extracted_image_count: int | None = None, extracted_image_names: list[str] | None = None) -> dict:
    if not model.is_dir():
        raise FileNotFoundError(f"COLMAP model directory does not exist: {model}")
    text_files = all((model / name).is_file() for name in ("cameras.txt", "images.txt", "points3D.txt"))
    binary_files = all((model / name).is_file() for name in ("cameras.bin", "images.bin", "points3D.bin"))
    if text_files:
        return inspect_text_model(model, extracted_image_count, extracted_image_names)
    if binary_files:
        try:
            import pycolmap

            reconstruction = pycolmap.Reconstruction(str(model))
            return {
                "format": "binary",
                "camera_count": len(reconstruction.cameras),
                "registered_image_count": len(reconstruction.images),
                "point3D_count": len(reconstruction.points3D),
                "inspection_backend": "pycolmap",
                "warning": "Convert to text for complete acceptance statistics.",
            }
        except ImportError:
            return {
                "format": "binary",
                "camera_count": None,
                "registered_image_count": None,
                "point3D_count": None,
                "inspection_backend": None,
                "warning": "pycolmap is unavailable; run COLMAP model_converter to text.",
            }
    raise ValueError("Directory does not contain a complete COLMAP text or binary sparse model.")


def _manifest_accepted(path: Path | None) -> tuple[int | None, list[str] | None]:
    if path is None:
        return None, None
    data = load_structured(path)
    rows = data.get("frames", data) if isinstance(data, dict) else data
    accepted = [str(row["frame_filename"]) for row in rows if row.get("accepted")]
    return len(accepted), accepted


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect a COLMAP text/binary sparse model and report acceptance statistics.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--frame-manifest", type=Path, help="Optional frame manifest used to calculate registration ratio.")
    parser.add_argument("--extracted-image-count", type=int, help="Explicit denominator for registration ratio.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        manifest_count, manifest_names = _manifest_accepted(args.frame_manifest)
        extracted = args.extracted_image_count if args.extracted_image_count is not None else manifest_count
        report = inspect_model(args.model, extracted, manifest_names)
        report["model"] = str(args.model)
        report["reconstructed_model_count_in_parent"] = sum(
            path.is_dir() and path.name.isdigit() for path in args.model.parent.iterdir()
        )
        if args.dry_run:
            print(json.dumps(report, indent=2)); return
        write_json(args.output, report)
        print(f"COLMAP inspection written to: {args.output}")
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
