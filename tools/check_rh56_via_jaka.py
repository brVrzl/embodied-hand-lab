from __future__ import annotations

import argparse
import json
from pathlib import Path

from embodiment_core.config import load_yaml
from rh56_driver.interfaces import HandCommand
from rh56_driver.jaka_tool_backend import RH56JakaToolBackend


def main() -> None:
    parser = argparse.ArgumentParser(description="RH56 smoke test via JAKA tool-end RS485.")
    parser.add_argument("--config", default="configs/hand/rh56.yaml")
    parser.add_argument("--ip", default=None, help="Override JAKA controller IP.")
    parser.add_argument("--hand-id", type=int, default=None, help="Override RH56 node id used in raw frames.")
    parser.add_argument("--enable-vout", action="store_true", help="Enable JAKA tool 24V output before sending commands.")
    parser.add_argument("--set-ai-rs485", action="store_true", help="Set JAKA AI pin mode to RS485L before sending commands.")
    parser.add_argument("--set-channel-mode", action="store_true", help="Set JAKA RS485 channel mode from config.")
    parser.add_argument("--channel-mode", choices=("raw", "modbus"), default=None, help="Override RS485 channel mode.")
    parser.add_argument("--set-comm", action="store_true", help="Set JAKA RS485 Modbus comm parameters from config when using Modbus mode.")
    parser.add_argument("--skip-motion", action="store_true", help="Only connect and report transport status.")
    parser.add_argument("--pause-sec", type=float, default=None, help="Pause between open and close.")
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_yaml(config_path)
    config["mode"] = "real"
    config["backend_type"] = "jaka_tool_rs485"

    transport_cfg = config.setdefault("jaka_tool_rs485", {})
    prepare_cfg = transport_cfg.setdefault("prepare_tio", {})
    if args.hand_id is not None:
        transport_cfg["hand_id"] = args.hand_id
        prepare_cfg.setdefault("comm", {})["slave_id"] = args.hand_id
        config.setdefault("serial", {})["hand_id"] = args.hand_id
    if args.pause_sec is not None:
        transport_cfg["command_pause_sec"] = args.pause_sec
    if args.enable_vout:
        prepare_cfg["enable_vout"] = True
    if args.set_ai_rs485:
        prepare_cfg["set_ai_pin_mode"] = True
    if args.channel_mode is not None:
        prepare_cfg["channel_mode"] = 1 if args.channel_mode == "raw" else 0
    if args.set_channel_mode:
        prepare_cfg["set_channel_mode"] = True
    if args.set_comm:
        prepare_cfg["set_comm"] = True

    robot_cfg_path = Path(transport_cfg.get("robot_config_path", "configs/robot/jaka_mini2.yaml"))
    transport_cfg["robot_config_path"] = str(robot_cfg_path)

    backend = RH56JakaToolBackend(config)
    if args.ip:
        backend.jaka_backend.config["ip"] = args.ip

    result = {
        "config": str(config_path.resolve()),
        "robot_config": str(robot_cfg_path.resolve()),
        "controller_ip": backend.jaka_backend.config.get("ip"),
        "channel_id": backend.channel_id,
        "hand_id": backend.hand_id,
        "requested_channel_mode": transport_cfg.get("prepare_tio", {}).get("channel_mode"),
        "known_open_frame": backend.build_open_frame().hex(" ").upper(),
        "known_close_frame": backend.build_close_frame().hex(" ").upper(),
    }

    try:
        backend.connect()
        result["connect_ok"] = True
        result["transport_status"] = backend.get_transport_status()
        if not args.skip_motion:
            backend.execute(HandCommand(command="close"))
            result["close_sent"] = True
            result["state_after_close"] = backend.read_state().to_dict()
            backend.execute(HandCommand(command="open"))
            result["open_sent"] = True
            result["state_after_open"] = backend.read_state().to_dict()
    except Exception as exc:
        result["connect_ok"] = False
        result["error"] = str(exc)
        raise
    finally:
        try:
            backend.disconnect()
        except Exception as exc:
            result["disconnect_error"] = str(exc)
        print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
