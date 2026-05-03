from __future__ import annotations

import argparse

from jaka_driver_adapter.adapter import JakaDriverAdapter
from quadruped_adapter.adapter import QuadrupedAdapter
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

    dog = subparsers.add_parser("dog")
    dog.add_argument("--linear-x", type=float, default=0.0)
    dog.add_argument("--linear-y", type=float, default=0.0)
    dog.add_argument("--angular-z", type=float, default=0.0)

    args = parser.parse_args()

    session = TeleopSession(
        arm=JakaDriverAdapter.from_yaml("configs/robot/jaka_mini2.yaml"),
        hand=RH56Driver.from_yaml("configs/hand/rh56.yaml"),
        dog=QuadrupedAdapter.from_yaml("configs/quadruped/default.yaml"),
    )
    if session.arm:
        session.arm.connect()
    if session.hand:
        session.hand.connect()
    if session.dog:
        session.dog.connect()

    if args.target == "arm":
        session.nudge_arm_ee(args.dx, args.dy, args.dz)
    elif args.target == "hand":
        session.hand_command(args.command, preset=args.preset)
    else:
        session.dog_teleop(args.linear_x, args.linear_y, args.angular_z)


if __name__ == "__main__":
    main()

