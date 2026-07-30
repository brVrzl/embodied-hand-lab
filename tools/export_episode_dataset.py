from __future__ import annotations

import argparse
from pathlib import Path

from episode_dataset.exporters import export_act_hdf5, export_lerobot_v3


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
    if args.format == "act-hdf5":
        result = export_act_hdf5(args.episode, args.output)
    else:
        result = export_lerobot_v3(args.episode, args.output, repo_id=args.repo_id)
    print(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
