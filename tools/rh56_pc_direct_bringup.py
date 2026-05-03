from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from rh56_driver.hand_schema import CANONICAL_HAND_ORDER, raw_to_canonical
from rh56_driver.serial_backend import RH56SerialBackend


def _read_full_state(backend: RH56SerialBackend) -> dict[str, Any]:
    raw_forces = backend.get_forces()
    raw_currents = backend.get_currents()
    raw_angles = backend.get_angles()
    return {
        "timestamp": time.time(),
        "protocol_order": list(backend.protocol_order),
        "canonical_order": list(CANONICAL_HAND_ORDER),
        "angles_raw": raw_angles,
        "forces_raw": raw_forces,
        "currents_raw": raw_currents,
        "angles_canonical": raw_to_canonical(raw_angles, raw_order=backend.protocol_order),
        "forces_canonical": raw_to_canonical(raw_forces, raw_order=backend.protocol_order),
        "currents_canonical": raw_to_canonical(raw_currents, raw_order=backend.protocol_order),
        "status_raw": backend.read_register(backend.REG["STATUS"], 6),
        "errors_raw": backend.read_register(backend.REG["ERROR"], 6),
        "temps_raw": backend.read_register(backend.REG["TEMP"], 6),
    }


def _summarize_intervals(timestamps: list[float]) -> dict[str, Any]:
    if len(timestamps) < 2:
        return {"count": len(timestamps), "hz_mean": 0.0, "intervals_sec": []}
    intervals = [b - a for a, b in zip(timestamps, timestamps[1:])]
    mean_interval = statistics.fmean(intervals)
    return {
        "count": len(timestamps),
        "hz_mean": 1.0 / mean_interval if mean_interval > 0 else 0.0,
        "interval_sec_mean": mean_interval,
        "interval_sec_min": min(intervals),
        "interval_sec_max": max(intervals),
        "interval_sec_p50": statistics.median(intervals),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "RH56 PC-direct USB-RS485 bring-up. Default mode is read-only and measures "
            "full feedback polling frequency. Add --execute to send open/close/code commands."
        )
    )
    parser.add_argument("--config", default="configs/hand/rh56_real.yaml")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=None)
    parser.add_argument("--polls", type=int, default=20)
    parser.add_argument("--poll-sec", type=float, default=0.0)
    parser.add_argument("--timeout-sec", type=float, default=None)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Allow write commands. Without this flag the script only reads feedback.",
    )
    parser.add_argument(
        "--command-cycles",
        type=int,
        default=0,
        help="Number of open/close command cycles to run. Requires --execute.",
    )
    parser.add_argument(
        "--preset",
        default="",
        help="Optional gesture preset to send after read checks. Requires --execute.",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_yaml(config_path)
    config["mode"] = "real"
    config["backend_type"] = "serial_protocol"
    serial_cfg = config.setdefault("serial", {})
    if args.port:
        serial_cfg["port"] = args.port
    if args.baudrate:
        serial_cfg["baudrate"] = args.baudrate
    if args.timeout_sec is not None:
        serial_cfg["timeout_sec"] = args.timeout_sec

    backend = RH56SerialBackend(config)
    result: dict[str, Any] = {
        "config": str(config_path.resolve()),
        "port": backend.port,
        "baudrate": backend.baudrate,
        "timeout_sec": backend.timeout,
        "protocol_order": list(backend.protocol_order),
        "gesture_order": backend.gesture_order,
        "polls_requested": args.polls,
        "poll_sec": args.poll_sec,
        "execute": args.execute,
        "command_cycles": args.command_cycles,
        "preset": args.preset,
    }

    try:
        try:
            import serial  # type: ignore
        except Exception as exc:
            raise RuntimeError("pyserial is required for RH56 PC-direct bring-up.") from exc

        backend.ser = serial.Serial(port=backend.port, baudrate=backend.baudrate, timeout=backend.timeout)
        result["port_open_ok"] = True

        states: list[dict[str, Any]] = []
        read_errors: list[str] = []
        for _ in range(max(args.polls, 1)):
            try:
                states.append(_read_full_state(backend))
            except Exception as exc:
                read_errors.append(str(exc))
            if args.poll_sec > 0:
                time.sleep(args.poll_sec)

        result["read_error_count"] = len(read_errors)
        result["read_errors"] = read_errors[:5]
        result["feedback_frequency"] = _summarize_intervals([row["timestamp"] for row in states])
        result["first_state"] = states[0] if states else None
        result["last_state"] = states[-1] if states else None

        command_results: list[dict[str, Any]] = []
        if (args.command_cycles > 0 or args.preset) and not args.execute:
            raise RuntimeError("Command tests require --execute.")

        if args.execute:
            backend.clear_error()
            backend.set_canonical_speeds(config.get("speed_default", [800] * 6))
            backend.set_canonical_forces(config.get("force_default", [500] * 6))

            gestures = config.get("gesture_presets", {})
            open_cmd = gestures.get("open", [1000] * 6)
            close_cmd = gestures.get("close", [0] * 6)
            for cycle in range(args.command_cycles):
                t0 = time.time()
                open_ok = backend.set_command_angles(open_cmd)
                t1 = time.time()
                close_ok = backend.set_command_angles(close_cmd)
                t2 = time.time()
                command_results.append(
                    {
                        "cycle": cycle,
                        "open_ok": open_ok,
                        "close_ok": close_ok,
                        "open_latency_sec": t1 - t0,
                        "close_latency_sec": t2 - t1,
                    }
                )

            if args.preset:
                preset = gestures.get(args.preset)
                if preset is None:
                    raise ValueError(f"Unknown RH56 preset: {args.preset}")
                t0 = time.time()
                preset_ok = backend.set_command_angles(preset)
                command_results.append(
                    {
                        "preset": args.preset,
                        "preset_ok": preset_ok,
                        "latency_sec": time.time() - t0,
                    }
                )

        result["command_results"] = command_results
    except Exception as exc:
        result["port_open_ok"] = result.get("port_open_ok", False)
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
