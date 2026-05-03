from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from embodiment_core.config import load_yaml
from jaka_driver_adapter.adapter import JakaDriverAdapter
from jaka_driver_adapter.jaka_sdk_backend import JakaSDKBackend


def main() -> None:
    parser = argparse.ArgumentParser(description="Save a named JAKA joint preset into the robot YAML config.")
    parser.add_argument("--config", default="configs/robot/jaka_mini2.yaml")
    parser.add_argument("--ip", default=None)
    parser.add_argument("--preset-name", required=True)
    parser.add_argument("--joints", nargs=6, type=float, default=None, metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_yaml(config_path)

    result = {
        "config": str(config_path.resolve()),
        "preset_name": args.preset_name,
        "source": "explicit_joints" if args.joints is not None else "current_robot_state",
    }

    if args.joints is not None:
        joints = [float(v) for v in args.joints]
    else:
        config["mode"] = "real"
        config["backend_type"] = "jaka_sdk"
        if args.ip:
            config["ip"] = args.ip
        adapter = JakaDriverAdapter(config)
        backend = adapter.backend
        try:
            adapter.connect()
            joints = [float(v) for v in adapter.get_joint_state().positions]
            result["ip"] = config["ip"]
            result["connect_ok"] = True
        finally:
            if isinstance(backend, JakaSDKBackend):
                try:
                    backend.disconnect()
                except Exception as exc:
                    result["disconnect_error"] = str(exc)

    config = load_yaml(config_path)
    joint_presets = config.setdefault("joint_presets", {})
    previous = joint_presets.get(args.preset_name)
    joint_presets[args.preset_name] = joints
    result["previous"] = previous
    result["saved_joints"] = joints

    if not args.dry_run:
        with config_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(config, handle, sort_keys=False, allow_unicode=True)
        result["written"] = True
    else:
        result["written"] = False

    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
