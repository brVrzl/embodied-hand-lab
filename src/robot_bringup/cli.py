from __future__ import annotations

import argparse
import json

from .stack import EmbodiedLabStack


def main() -> None:
    parser = argparse.ArgumentParser(description="Bring up the embodied lab stack.")
    parser.add_argument("--no-arm-hand", action="store_true")
    parser.add_argument("--no-camera", action="store_true")
    args = parser.parse_args()

    stack = EmbodiedLabStack(
        include_arm_hand=not args.no_arm_hand,
        include_camera=not args.no_camera,
    )
    print(json.dumps({"connect_results": stack.connect_all(), "snapshot": stack.snapshot()}, indent=2))


if __name__ == "__main__":
    main()
