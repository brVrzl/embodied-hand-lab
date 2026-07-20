#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import load_structured, write_json
from digital_twin.registration.transforms import apply_similarity


def main() -> None:
    parser = argparse.ArgumentParser(description="Apply a calibrated T_B_R similarity to point/mesh scene geometry.")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--transform", type=Path, required=True, help="Registration JSON/YAML containing scale, rotation_matrix, translation_m.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        if not args.input.is_file():
            raise FileNotFoundError(f"Scene input does not exist: {args.input}")
        transform = load_structured(args.transform)
        scale, rotation, translation = float(transform["scale"]), np.asarray(transform["rotation_matrix"], float), np.asarray(transform["translation_m"], float)
        if scale <= 0 or rotation.shape != (3, 3) or translation.shape != (3,):
            raise ValueError("Registration transform has invalid dimensions or scale.")
        summary = {"input": str(args.input), "output": str(args.output), "transform": "T_B_R", "scale": scale}
        if args.dry_run:
            print(json.dumps(summary, indent=2)); return
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if args.input.suffix.lower() == ".npy":
            np.save(args.output, apply_similarity(np.load(args.input), scale, rotation, translation))
        elif args.input.suffix.lower() == ".npz":
            data = np.load(args.input); output = {key: data[key] for key in data.files}
            output["points"] = apply_similarity(output["points"], scale, rotation, translation)
            np.savez_compressed(args.output, **output)
        else:
            geometry = trimesh.load(args.input, process=False)
            matrix = np.eye(4); matrix[:3, :3] = scale * rotation; matrix[:3, 3] = translation
            geometry.apply_transform(matrix); geometry.export(args.output)
        write_json(args.output.with_suffix(args.output.suffix + ".alignment.json"), summary)
        print(f"Aligned scene written to: {args.output}")
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
