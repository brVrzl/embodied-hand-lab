from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from rh56_driver.hand_schema import CANONICAL_HAND_ORDER, raw_to_canonical
from rh56_driver.serial_backend import RH56SerialBackend

GESTURE_FORCE_CLB_ADDR = 1009


def _read_state(backend: RH56SerialBackend) -> dict[str, Any]:
    angles = backend.get_angles()
    forces = backend.get_forces()
    currents = backend.get_currents()
    return {
        "timestamp_sec": time.time(),
        "protocol_order": list(backend.protocol_order),
        "canonical_order": list(CANONICAL_HAND_ORDER),
        "angles_raw_protocol": angles,
        "forces_raw_protocol": forces,
        "currents_raw_protocol": currents,
        "angles_canonical": raw_to_canonical(angles, raw_order=backend.protocol_order),
        "forces_canonical": raw_to_canonical(forces, raw_order=backend.protocol_order),
        "currents_canonical": raw_to_canonical(currents, raw_order=backend.protocol_order),
        "status_raw_protocol": backend.read_register(backend.REG["STATUS"], 6),
        "errors_raw_protocol": backend.read_register(backend.REG["ERROR"], 6),
        "temps_raw_protocol": backend.read_register(backend.REG["TEMP"], 6),
    }


def _open_backend(args: argparse.Namespace) -> RH56SerialBackend:
    config = load_yaml(args.config)
    config["mode"] = "real"
    config["backend_type"] = "serial_protocol"
    serial_cfg = config.setdefault("serial", {})
    if args.port:
        serial_cfg["port"] = args.port
    if args.baudrate:
        serial_cfg["baudrate"] = args.baudrate
    serial_cfg["timeout_sec"] = args.timeout_sec
    backend = RH56SerialBackend(config)
    try:
        import serial  # type: ignore
    except Exception as exc:
        raise RuntimeError("pyserial is required for RH56 PC-direct calibration.") from exc
    backend.ser = serial.Serial(port=backend.port, baudrate=backend.baudrate, timeout=backend.timeout)
    return backend


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Trigger the official RH56 force-sensor calibration over PC-direct USB-RS485. "
            "Calibration moves the hand automatically; keep all fingers unloaded."
        )
    )
    parser.add_argument("--config", default="configs/hand/rh56_pc_direct_teleop.yaml")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=None)
    parser.add_argument("--timeout-sec", type=float, default=0.2)
    parser.add_argument("--wait-sec", type=float, default=8.0)
    parser.add_argument("--post-polls", type=int, default=5)
    parser.add_argument("--post-poll-sec", type=float, default=0.4)
    parser.add_argument("--out", default="data/reports/rh56_force_calibration/last_calibration.json")
    parser.add_argument("--execute", action="store_true", help="Actually write GESTURE_FORCE_CLB=1.")
    args = parser.parse_args()

    backend = _open_backend(args)
    result: dict[str, Any] = {
        "schema_version": "rh56_force_sensor_calibration_v0.1",
        "config": str(Path(args.config).resolve()),
        "port": backend.port,
        "baudrate": backend.baudrate,
        "protocol_order": list(backend.protocol_order),
        "canonical_order": list(CANONICAL_HAND_ORDER),
        "gesture_force_clb_addr": GESTURE_FORCE_CLB_ADDR,
        "execute": bool(args.execute),
        "wait_sec": args.wait_sec,
    }
    try:
        result["before"] = _read_state(backend)
        if not args.execute:
            result["skipped"] = "Pass --execute to write GESTURE_FORCE_CLB=1."
            return

        backend.clear_error()
        result["clear_error_ok"] = True
        print(
            "About to trigger RH56 force sensor calibration. "
            "Keep the hand unloaded; calibration will move fingers automatically.",
            flush=True,
        )
        time.sleep(2.0)
        result["trigger_ok"] = backend.write_register(GESTURE_FORCE_CLB_ADDR, [1])
        result["trigger_timestamp_sec"] = time.time()
        time.sleep(max(float(args.wait_sec), 0.0))

        post: list[dict[str, Any]] = []
        for _ in range(max(int(args.post_polls), 1)):
            post.append(_read_state(backend))
            time.sleep(max(float(args.post_poll_sec), 0.0))
        result["after_polls"] = post
        result["after"] = post[-1]
    finally:
        try:
            backend.close_port()
        finally:
            out = Path(args.out)
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            print(json.dumps(result, indent=2, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()

