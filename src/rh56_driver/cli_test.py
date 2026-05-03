from __future__ import annotations

import argparse
import json
from pathlib import Path

from .node import RH56Driver


def main() -> None:
    parser = argparse.ArgumentParser(description="Minimal RH56 CLI test.")
    parser.add_argument("--config", default="configs/hand/rh56.yaml")
    parser.add_argument("--command", choices=["open", "close", "pinch", "preset"], default="open")
    parser.add_argument("--preset", default="power_grasp")
    args = parser.parse_args()

    config_path = Path(args.config)
    driver = RH56Driver.from_yaml(config_path)
    driver.connect()
    if args.command == "open":
        driver.open()
    elif args.command == "close":
        driver.close()
    elif args.command == "pinch":
        driver.pinch()
    else:
        driver.preset_grasp(args.preset)
    print(json.dumps(driver.read_state().to_dict(), indent=2))


if __name__ == "__main__":
    main()

