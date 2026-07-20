#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def export_text_points(input_path: Path, output_path: Path, max_error: float | None) -> int:
    if not input_path.is_file():
        raise FileNotFoundError(f"COLMAP points3D text file does not exist: {input_path}")
    points = []
    for line in input_path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) < 8:
            continue
        error = float(fields[7])
        if max_error is not None and error > max_error:
            continue
        points.append((float(fields[1]), float(fields[2]), float(fields[3]), int(fields[4]), int(fields[5]), int(fields[6]), error))
    header = ["ply", "format ascii 1.0", f"element vertex {len(points)}", "property float x", "property float y", "property float z", "property uchar red", "property uchar green", "property uchar blue", "property float reprojection_error", "end_header"]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(header + [" ".join(map(str, point)) for point in points]) + "\n", encoding="utf-8")
    return len(points)


def main() -> None:
    parser = argparse.ArgumentParser(description="Export COLMAP text points3D to a colored rendering/debug PLY.")
    parser.add_argument("--input", type=Path, required=True, help="COLMAP points3D.txt.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-reprojection-error", type=float)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        if args.dry_run:
            if not args.input.is_file():
                raise FileNotFoundError(f"Input does not exist: {args.input}")
            print(f"Would export {args.input} to {args.output}"); return
        count = export_text_points(args.input, args.output, args.max_reprojection_error)
        print(f"Exported {count} points to: {args.output}")
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
