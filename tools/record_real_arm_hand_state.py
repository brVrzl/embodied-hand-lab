from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from jaka_driver_adapter.adapter import JakaDriverAdapter
from jaka_driver_adapter.jaka_sdk_backend import JakaSDKBackend
from rh56_driver.hand_schema import RH56_PROTOCOL_ORDER
from rh56_driver.serial_backend import RH56SerialBackend


SCHEMA_VERSION = "palmhand_lab.real_arm_hand_state.v1"


def _extract_payload(result: Any) -> Any:
    if isinstance(result, tuple) and len(result) >= 2:
        payload = result[1]
    else:
        payload = result
    if isinstance(payload, tuple) and len(payload) == 1:
        return payload[0]
    return payload


def _extract_pose_payload(result: Any) -> dict[str, Any]:
    payload = _extract_payload(result)
    if hasattr(payload, "tran") and hasattr(payload, "rpy"):
        return {
            "frame_id": "jaka_base",
            "translation_m": [
                float(getattr(payload.tran, "x")) / 1000.0,
                float(getattr(payload.tran, "y")) / 1000.0,
                float(getattr(payload.tran, "z")) / 1000.0,
            ],
            "rpy_rad": [
                float(getattr(payload.rpy, "rx")),
                float(getattr(payload.rpy, "ry")),
                float(getattr(payload.rpy, "rz")),
            ],
            "source_units": {"translation": "mm", "rotation": "rad"},
        }
    if isinstance(payload, (list, tuple)) and len(payload) >= 6:
        return {
            "frame_id": "jaka_base",
            "translation_m": [float(v) / 1000.0 for v in payload[:3]],
            "rpy_rad": [float(v) for v in payload[3:6]],
            "source_units": {"translation": "mm", "rotation": "rad"},
        }
    return {"raw_pose": repr(payload)}


def _collect_arm_flags(backend: JakaSDKBackend) -> dict[str, Any]:
    flags: dict[str, Any] = {}
    robot = getattr(backend, "_robot", None)
    if robot is None:
        return flags
    for method_name in (
        "is_in_estop",
        "is_in_collision",
        "is_on_limit",
        "is_in_drag_mode",
        "is_in_servomove",
        "is_in_pos",
    ):
        if hasattr(robot, method_name):
            flags[method_name] = _extract_payload(backend.call_sdk_method(method_name))
    if hasattr(robot, "get_last_error"):
        flags["last_error_raw"] = _extract_payload(backend.call_sdk_method("get_last_error"))
    return flags


def _connect_hand_readonly(config: dict[str, Any], port: str | None) -> RH56SerialBackend:
    config = dict(config)
    config["mode"] = "real"
    config["backend_type"] = "serial_protocol"
    if port:
        config.setdefault("serial", {})["port"] = port
    backend = RH56SerialBackend(config)
    try:
        import serial  # type: ignore
    except Exception as exc:
        raise RuntimeError("pyserial is required for RH56 read-only state recording.") from exc
    backend.ser = serial.Serial(
        port=backend.port,
        baudrate=backend.baudrate,
        timeout=backend.timeout,
    )
    return backend


def _hand_record(backend: RH56SerialBackend) -> dict[str, Any]:
    state = backend.read_state()
    schema_cfg = backend.config.get("hand_schema", {})
    canonical_order = schema_cfg.get(
        "canonical_order",
        ["index", "middle", "ring", "pinky", "thumb_close", "thumb_lateral"],
    )
    protocol_order = schema_cfg.get("protocol_order", RH56_PROTOCOL_ORDER)
    return {
        "frame_id": "rh56_palm",
        "canonical_order": list(canonical_order),
        "protocol_order": list(protocol_order),
        "finger_state": {
            "angle_count_0_1000": state.finger_positions,
            "current_count": state.finger_currents,
            "force_count": state.force_estimate,
            "contact_flags_from_force_positive": state.contact_flags,
        },
        "units": {
            "angle_count_0_1000": "vendor_count_open_1000_close_0",
            "current_count": "vendor_raw_count",
            "force_count": "signed_vendor_raw_count",
        },
        "mode": state.mode,
    }


def _arm_record(adapter: JakaDriverAdapter, backend: JakaSDKBackend) -> dict[str, Any]:
    joint_state = adapter.get_joint_state()
    tcp_pose = None
    robot = getattr(backend, "_robot", None)
    if robot is not None and hasattr(robot, "get_actual_tcp_position"):
        tcp_pose = _extract_pose_payload(backend.call_sdk_method("get_actual_tcp_position"))
    elif robot is not None and hasattr(robot, "get_tcp_position"):
        tcp_pose = _extract_pose_payload(backend.call_sdk_method("get_tcp_position"))
    return {
        "joint_state": {
            "names": joint_state.names,
            "position_rad": joint_state.positions,
            "velocity_rad_s": joint_state.velocities,
            "effort": joint_state.efforts,
        },
        "tcp_pose": tcp_pose,
        "state_flags": _collect_arm_flags(backend),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Record synchronized read-only JAKA+RH56 state samples as JSONL."
    )
    parser.add_argument("--arm-config", default="configs/robot/jaka_mini2_real.yaml")
    parser.add_argument("--hand-config", default="configs/hand/rh56_real.yaml")
    parser.add_argument("--arm-ip", default=None)
    parser.add_argument("--hand-port", default=None)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument(
        "--output",
        default="data/real_arm_hand_state_20260505.jsonl",
        help="JSONL output path.",
    )
    args = parser.parse_args()

    if args.rate_hz <= 0.0:
        raise SystemExit("--rate-hz must be positive.")
    if args.duration_sec <= 0.0:
        raise SystemExit("--duration-sec must be positive.")

    arm_config_path = Path(args.arm_config)
    hand_config_path = Path(args.hand_config)
    arm_config = load_yaml(arm_config_path)
    arm_config["mode"] = "real"
    arm_config["backend_type"] = "jaka_sdk"
    if args.arm_ip:
        arm_config["ip"] = args.arm_ip
    hand_config = load_yaml(hand_config_path)

    arm = JakaDriverAdapter(arm_config)
    arm_backend = arm.backend
    if not isinstance(arm_backend, JakaSDKBackend):
        raise RuntimeError("JAKA real state recording requires the official SDK backend.")
    hand_backend: RH56SerialBackend | None = None
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    summary: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "arm_config": str(arm_config_path.resolve()),
        "hand_config": str(hand_config_path.resolve()),
        "arm_ip": arm_config["ip"],
        "rate_hz": args.rate_hz,
        "duration_sec": args.duration_sec,
        "output": str(output_path.resolve()),
        "samples": 0,
        "sample_errors": [],
    }

    try:
        arm.connect()
        hand_backend = _connect_hand_readonly(hand_config, args.hand_port)
        summary["connect_ok"] = True
        period = 1.0 / args.rate_hz
        start_monotonic = time.monotonic()
        next_tick = start_monotonic
        sample_times: list[float] = []

        with output_path.open("w", encoding="utf-8") as stream:
            sequence = 0
            while True:
                loop_start = time.monotonic()
                elapsed = loop_start - start_monotonic
                if elapsed > args.duration_sec:
                    break
                try:
                    record = {
                        "schema_version": SCHEMA_VERSION,
                        "sequence": sequence,
                        "timestamp": {
                            "unix_sec": time.time(),
                            "monotonic_sec": loop_start,
                        },
                        "arm": _arm_record(arm, arm_backend),
                        "hand": _hand_record(hand_backend),
                    }
                    stream.write(json.dumps(record, ensure_ascii=False) + "\n")
                    sample_times.append(loop_start)
                    sequence += 1
                except Exception as exc:
                    summary["sample_errors"].append(
                        {"sequence": sequence, "error": f"{type(exc).__name__}: {exc}"}
                    )
                    break
                next_tick += period
                sleep_time = next_tick - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)

        intervals = [
            sample_times[i + 1] - sample_times[i] for i in range(len(sample_times) - 1)
        ]
        summary["samples"] = len(sample_times)
        summary["observed_rate_hz"] = (
            (len(sample_times) - 1) / (sample_times[-1] - sample_times[0])
            if len(sample_times) >= 2 and sample_times[-1] > sample_times[0]
            else None
        )
        summary["interval_sec_min"] = min(intervals) if intervals else None
        summary["interval_sec_max"] = max(intervals) if intervals else None
    except Exception as exc:
        summary["connect_ok"] = summary.get("connect_ok", False)
        summary["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if hand_backend is not None:
            try:
                hand_backend.close_port()
            except Exception as exc:
                summary.setdefault("hand_disconnect_error", str(exc))
        try:
            arm_backend.disconnect()
        except Exception as exc:
            summary.setdefault("arm_disconnect_error", str(exc))
        print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
