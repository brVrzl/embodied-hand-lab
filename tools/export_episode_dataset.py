from __future__ import annotations

import argparse
from pathlib import Path

from episode_dataset_cli import main as dataset_cli_main


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline export of one finalized canonical episode.")
    parser.add_argument("episode", type=Path)
    subparsers = parser.add_subparsers(dest="format", required=True)
    act = subparsers.add_parser("act-hdf5")
    act.add_argument("output", type=Path)
    lerobot = subparsers.add_parser("lerobot-v3")
    lerobot.add_argument("output", type=Path)
    lerobot.add_argument("--repo-id", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    forwarded = ["export", str(args.episode), args.format, str(args.output)]
    if args.format == "lerobot-v3":
        forwarded.extend(["--repo-id", args.repo_id])
    return dataset_cli_main(forwarded)


if __name__ == "__main__":
    raise SystemExit(main())
