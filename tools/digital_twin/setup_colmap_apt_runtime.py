#!/usr/bin/env python3
"""Create an isolated Ubuntu/ARM64 COLMAP runtime without modifying the venv."""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
from pathlib import Path


DEFAULT_PACKAGES = [
    "colmap",
    "libboost-program-options1.83.0",
    "libmetis5",
    "libceres4t64",
    "libgoogle-glog0v6t64",
    "libfreeimage3",
    "libcholmod5",
    "libspqr4",
    "libgflags2.2",
    "libjxr0t64",
    "libamd3",
    "libcamd3",
    "libccolamd3",
]


def run(command: list[str], *, cwd: Path, env: dict[str, str] | None = None) -> None:
    process = subprocess.run(command, cwd=cwd, env=env, text=True)
    if process.returncode:
        raise RuntimeError(f"Command failed ({process.returncode}): {' '.join(command)}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Download and extract an isolated apt COLMAP runtime (tested on Ubuntu ARM64); does not install system packages."
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--package", action="append", dest="packages", help="Override package list; repeat for each apt package.")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    args.output_dir = args.output_dir.resolve()
    packages = args.packages or DEFAULT_PACKAGES
    downloads = args.output_dir / "downloads"
    root = args.output_dir / "root"
    executable = root / "usr/bin/colmap"
    commands = [["apt-get", "download", package] for package in packages]
    manifest = {
        "method": "apt_download_and_dpkg_extract_no_system_install",
        "packages": packages,
        "commands": commands,
        "executable": str(executable),
        "runtime_environment": {
            "LD_LIBRARY_PATH": f"{root}/usr/lib/aarch64-linux-gnu:{root}/lib/aarch64-linux-gnu",
            "QT_QPA_PLATFORM": "offscreen",
        },
    }
    if args.dry_run:
        print(json.dumps(manifest, indent=2)); return
    if executable.exists() and not args.overwrite:
        raise SystemExit(f"Runtime already exists: {executable}; pass --overwrite to rebuild.")
    if shutil.which("apt-get") is None or shutil.which("dpkg-deb") is None:
        raise SystemExit("apt-get and dpkg-deb are required for this isolated route.")
    if args.overwrite and args.output_dir.exists():
        shutil.rmtree(args.output_dir)
    downloads.mkdir(parents=True, exist_ok=True)
    root.mkdir(parents=True, exist_ok=True)
    for command in commands:
        run(command, cwd=downloads)
    debs = sorted(downloads.glob("*.deb"))
    if not debs:
        raise SystemExit("apt-get produced no .deb files.")
    for deb in debs:
        run(["dpkg-deb", "-x", str(deb), str(root)], cwd=args.output_dir)
    if not executable.is_file():
        raise SystemExit(f"Extracted runtime is missing {executable}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "runtime_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    env = os.environ.copy()
    env.update(manifest["runtime_environment"])
    run([str(executable), "-h"], cwd=args.output_dir, env=env)
    print(f"Isolated COLMAP runtime ready: {executable}")


if __name__ == "__main__":
    main()
