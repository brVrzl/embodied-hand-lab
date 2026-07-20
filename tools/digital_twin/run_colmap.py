#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import load_structured, write_command_log


def build_commands(args: argparse.Namespace) -> list[list[str]]:
    executable = args.colmap_executable
    database = args.workspace / "database.db"
    sparse = args.workspace / "sparse"
    commands = [
        [executable, "feature_extractor", "--database_path", str(database), "--image_path", str(args.images), "--ImageReader.camera_model", args.camera_model, "--ImageReader.single_camera", "1" if args.single_camera else "0", "--ImageReader.single_camera_per_folder", "1" if args.single_camera_per_folder else "0"],
        [executable, args.matcher, "--database_path", str(database)],
    ]
    if args.loop_closure_exhaustive and args.matcher != "exhaustive_matcher":
        commands.append([executable, "exhaustive_matcher", "--database_path", str(database)])
    commands.append([executable, "mapper", "--database_path", str(database), "--image_path", str(args.images), "--output_path", str(sparse)])
    if args.config:
        config = load_structured(args.config) or {}
        extras = config.get("extra_arguments", {})
        for command in commands:
            stage = command[1]
            for key, value in extras.get(stage, {}).items():
                command.extend([f"--{key}", str(value)])
    return commands


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a reproducible COLMAP sparse reconstruction as explicit subprocess stages.")
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--colmap-executable", default="colmap")
    parser.add_argument("--camera-model", default="OPENCV", choices=["SIMPLE_PINHOLE", "PINHOLE", "SIMPLE_RADIAL", "RADIAL", "OPENCV", "FULL_OPENCV"])
    parser.add_argument("--single-camera", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--single-camera-per-folder", action="store_true", help="Use one camera per image subfolder; intended for joint multi-video workspaces.")
    parser.add_argument("--matcher", choices=["sequential_matcher", "exhaustive_matcher", "vocab_tree_matcher"], default="sequential_matcher")
    parser.add_argument("--loop-closure-exhaustive", action="store_true", help="After sequential matching, add exhaustive pairs for small loop-forming datasets.")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        if not args.images.is_dir() or not any(args.images.iterdir()):
            raise ValueError(f"Image directory is missing or empty: {args.images}")
        if (args.workspace / "database.db").exists() and not args.overwrite:
            raise ValueError("COLMAP database already exists; pass --overwrite or choose another workspace.")
        commands = build_commands(args)
        args.workspace.mkdir(parents=True, exist_ok=True)
        write_command_log(args.workspace / "commands.json", commands)
        if args.dry_run:
            print(json.dumps({"commands": commands}, indent=2)); return
        executable_path = shutil.which(args.colmap_executable)
        if not executable_path:
            raise RuntimeError(f"COLMAP executable not found: {args.colmap_executable}")
        if args.overwrite and (args.workspace / "database.db").exists():
            (args.workspace / "database.db").unlink()
        (args.workspace / "sparse").mkdir(parents=True, exist_ok=True)
        log_path = args.workspace / "colmap.log"
        with log_path.open("w", encoding="utf-8") as log:
            for command in commands:
                process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, text=True)
                if process.returncode:
                    raise RuntimeError(f"COLMAP stage {command[1]} failed with exit code {process.returncode}; see {log_path}")
        print(f"COLMAP workspace written to: {args.workspace}")
    except (FileNotFoundError, KeyError, TypeError, ValueError, RuntimeError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
