from __future__ import annotations

import argparse
import json
from pathlib import Path

from embodiment_core.config import load_yaml
from rh56_driver.serial_backend import RH56SerialBackend


def main() -> None:
    parser = argparse.ArgumentParser(description="Safe RH56 connectivity check without motion.")
    parser.add_argument("--config", default="configs/hand/rh56.yaml")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=None)
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
