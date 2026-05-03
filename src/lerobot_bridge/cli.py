from __future__ import annotations

import argparse

from .exporter import export_to_lerobot_stub


def main() -> None:
    parser = argparse.ArgumentParser(description="Export episodes into a LeRobot-like stub dataset.")
    parser.add_argument("--episodes-root", default="data/episodes")
    parser.add_argument("--output", default="data/exports/lerobot")
    args = parser.parse_args()
    export_root = export_to_lerobot_stub(args.episodes_root, args.output)
    print(f"LeRobot stub exported to: {export_root}")


if __name__ == "__main__":
    main()
