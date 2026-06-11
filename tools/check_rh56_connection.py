from __future__ import annotations

import argparse
import glob
import json
import os
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from rh56_driver.serial_backend import RH56SerialBackend


def _serial_preflight(port: str) -> dict[str, Any]:
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    by_path = sorted(glob.glob("/dev/serial/by-path/*"))
    result: dict[str, Any] = {
        "requested_port_exists": Path(port).exists(),
        "tty_candidates": candidates,
        "serial_by_id": by_id,
        "serial_by_path": by_path,
    }
    try:
        import grp

        group_ids = os.getgroups()
        result["user_groups"] = sorted(grp.getgrgid(group_id).gr_name for group_id in group_ids)
        result["in_dialout"] = "dialout" in result["user_groups"]
    except Exception as exc:
        result["group_error"] = str(exc)

    try:
        import subprocess

        brltty = subprocess.run(
            ["systemctl", "is-active", "brltty-udev.service"],
            check=False,
            text=True,
            capture_output=True,
        )
        result["brltty_udev_active"] = brltty.stdout.strip() == "active"
        result["brltty_udev_status"] = brltty.stdout.strip() or brltty.stderr.strip()
    except Exception as exc:
        result["brltty_check_error"] = str(exc)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Safe RH56 connectivity check without motion.")
    parser.add_argument("--config", default="configs/hand/rh56_real.yaml")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=None)
    parser.add_argument("--preflight-only", action="store_true")
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_yaml(config_path)
    config["mode"] = "real"
    config["backend_type"] = "serial_protocol"
    if args.port:
        config.setdefault("serial", {})["port"] = args.port
    if args.baudrate:
        config.setdefault("serial", {})["baudrate"] = args.baudrate

    backend = RH56SerialBackend(config)
    result = {
        "config": str(config_path.resolve()),
        "port": config.get("serial", {}).get("port"),
        "baudrate": config.get("serial", {}).get("baudrate"),
    }
    result["preflight"] = _serial_preflight(str(result["port"]))

    if args.preflight_only:
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return

    try:
        try:
            import serial  # type: ignore
        except Exception as exc:
            raise RuntimeError("pyserial is required for RH56 connectivity checks.") from exc

        backend.ser = serial.Serial(port=backend.port, baudrate=backend.baudrate, timeout=backend.timeout)
        result["port_open_ok"] = True
        result["angles"] = backend.get_angles()
        result["forces"] = backend.get_forces()
        result["currents"] = backend.get_currents()
        result["status"] = backend.read_register(backend.REG["STATUS"], 6)
        result["errors"] = backend.read_register(backend.REG["ERROR"], 6)
        result["temps"] = backend.read_register(backend.REG["TEMP"], 6)
    except Exception as exc:
        result["port_open_ok"] = False
        result["error"] = str(exc)
        raise
    finally:
        try:
            backend.close_port()
        except Exception as exc:
            result["close_error"] = str(exc)
        print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
