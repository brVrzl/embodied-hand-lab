from __future__ import annotations

import argparse

from iphone_mediapipe_hand_teleop import main as teleop_main


def main() -> None:
    parser = argparse.ArgumentParser(description="Compatibility wrapper for iPhone RH56 safety-gated teleop.")
    parser.add_argument("args", nargs=argparse.REMAINDER)
    parsed = parser.parse_args()
    import sys
    sys.argv = [sys.argv[0], *parsed.args]
    teleop_main()


if __name__ == "__main__":
    main()
