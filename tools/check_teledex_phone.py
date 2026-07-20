from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from embodiment_core.config import load_yaml
from teleop_tools.teledex_phone import TeleDexPhoneClient


DEFAULT_CONFIG = "configs/teleop/teledex_jaka_arm.yaml"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Receive TeleDex iPhone pose/buttons without publishing robot commands."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--no-qr", action="store_true")
    parser.add_argument(
        "--controls-only",
        action="store_true",
        help="Print only Button A/Button B/Toggle state changes for a read-only control probe.",
    )
    parser.add_argument("--jsonl-out", default=None)
    args = parser.parse_args()

    config = load_yaml(args.config)
    teledex_cfg = config.get("teledex", {})
    client = TeleDexPhoneClient(
        port=int(args.port if args.port is not None else teledex_cfg.get("port", 8888)),
        show_qr=bool(teledex_cfg.get("show_qr", True)) and not args.no_qr,
        debug=bool(teledex_cfg.get("debug", False)),
        max_stale_feedback_sec=float(teledex_cfg.get("max_stale_feedback_sec", 0.20)),
        server_start_timeout_sec=float(teledex_cfg.get("server_start_timeout_sec", 3.0)),
        deadman_field=str(teledex_cfg.get("deadman_field", "button")),
        precision_scale=float(teledex_cfg.get("precision_scale", 1.0)),
    )
    output = None
    if args.jsonl_out:
        path = Path(args.jsonl_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        output = path.open("w", encoding="utf-8")

    valid_samples = 0
    last_sequence = -1
    last_invalid_reason: str | None = None
    last_controls: tuple[bool, bool, bool] | None = None
    started = time.monotonic()
    duration_sec = max(0.0, float(args.duration_sec))
    interval_sec = 1.0 / max(float(args.hz), 1e-3)
    try:
        client.connect()
        print(
            f"TeleDex server ready at {client.address}. Open the iPhone app, connect/scan, "
            "then test Button A, Button B, and Toggle one at a time. The JSON raw_inputs "
            "must identify each control before any real-arm run."
        )
        while duration_sec == 0.0 or time.monotonic() - started < duration_sec:
            snapshot = client.read()
            sequence = int(snapshot.raw_inputs.get("sequence", -1))
            controls = (
                bool(snapshot.raw_inputs.get("button", False)),
                bool(snapshot.raw_inputs.get("button_secondary", False)),
                bool(snapshot.raw_inputs.get("toggle", False)),
            )
            if snapshot.valid and controls != last_controls:
                print(
                    "CONTROL_STATE "
                    f"button={controls[0]} button_secondary={controls[1]} toggle={controls[2]}"
                )
                last_controls = controls
            should_print = (
                (snapshot.valid and sequence != last_sequence)
                or (not snapshot.valid and snapshot.reason != last_invalid_reason)
            )
            if should_print and (not args.controls_only or not snapshot.valid):
                payload = {
                    "schema_version": "teledex_phone_pose_v0.1",
                    "snapshot": snapshot.to_dict(),
                    "deadman": snapshot.enabled,
                }
                line = json.dumps(payload, ensure_ascii=False)
                print(line)
                if output is not None:
                    output.write(line + "\n")
                    output.flush()
                last_sequence = sequence
                last_invalid_reason = None if snapshot.valid else snapshot.reason
            if snapshot.valid:
                valid_samples += 1
            time.sleep(interval_sec)
    except KeyboardInterrupt:
        pass
    finally:
        client.close()
        if output is not None:
            output.close()

    if valid_samples == 0:
        raise SystemExit(
            "No valid TeleDex pose was received. Check that phone and PC are on the same LAN, "
            "the displayed IP is reachable, and the selected port is allowed by the firewall."
        )
    print(f"TeleDex read-only check passed with {valid_samples} valid polling samples.")


if __name__ == "__main__":
    main()
