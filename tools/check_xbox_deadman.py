from __future__ import annotations

import argparse
import time

from teleop_tools.xbox_ros2 import PygameXboxController


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Read Xbox RB as a deadman without sourcing ROS2 or publishing commands."
    )
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--hz", type=float, default=20.0)
    args = parser.parse_args()

    controller = PygameXboxController(index=args.index)
    print(f"Xbox controller ready: {controller.name}. Hold and release RB; no commands are published.")
    started = time.monotonic()
    last_state = None
    saw_pressed = False
    saw_released_after_press = False
    try:
        while time.monotonic() - started < max(0.1, args.duration_sec):
            pressed = bool(controller.snapshot().buttons.get("rb", False))
            if pressed != last_state:
                print(f"deadman_rb={str(pressed).lower()}")
                if pressed:
                    saw_pressed = True
                elif saw_pressed:
                    saw_released_after_press = True
                last_state = pressed
            time.sleep(1.0 / max(args.hz, 1.0))
    finally:
        controller.pygame.quit()
    if not saw_pressed or not saw_released_after_press:
        raise SystemExit("RB press-and-release was not observed.")
    print("Xbox RB deadman check passed.")


if __name__ == "__main__":
    main()
