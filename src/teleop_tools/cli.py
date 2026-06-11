from __future__ import annotations

import argparse

from jaka_driver_adapter.adapter import JakaDriverAdapter
from rh56_driver.node import RH56Driver

from .session import TeleopSession


def main() -> None:
    parser = argparse.ArgumentParser(description="Minimal CLI teleop.")
    subparsers = parser.add_subparsers(dest="target", required=True)

    arm = subparsers.add_parser("arm")
    arm.add_argument("--dx", type=float, default=0.0)
    arm.add_argument("--dy", type=float, default=0.0)
    arm.add_argument("--dz", type=float, default=0.0)

    hand = subparsers.add_parser("hand")
    hand.add_argument("--command", choices=["open", "close", "pinch", "preset"], required=True)
    hand.add_argument("--preset", default="power_grasp")

    args = parser.parse_args()

    session = TeleopSession(
        arm=JakaDriverAdapter.from_yaml("configs/robot/jaka_mini2.yaml"),
        hand=RH56Driver.from_yaml("configs/hand/rh56.yaml"),
    )
    if session.arm:
        session.arm.connect()
    if session.hand:
        session.hand.connect()

    if args.target == "arm":
        session.nudge_arm_ee(args.dx, args.dy, args.dz)
    elif args.target == "hand":
        session.hand_command(args.command, preset=args.preset)


if __name__ == "__main__":
    main()
