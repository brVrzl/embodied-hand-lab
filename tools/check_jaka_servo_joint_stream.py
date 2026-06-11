from __future__ import annotations

import argparse
import json
import math
import socket
import time
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from jaka_driver_adapter.adapter import JakaDriverAdapter
from jaka_driver_adapter.jaka_sdk_backend import JakaSDKBackend


ABS_MOVE_MODE = 0


def _resolve_edg_stat_ip(controller_ip: str, configured_ip: str) -> str:
    configured_ip = str(configured_ip).strip()
    if configured_ip and configured_ip.lower() not in {"auto", "default"}:
        return configured_ip
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((controller_ip, 10001))
        return str(sock.getsockname()[0])
    finally:
        sock.close()


def _extract_scalar(result: Any) -> Any:
    payload = result[1] if isinstance(result, tuple) and len(result) >= 2 else result
    if isinstance(payload, tuple) and len(payload) == 1:
        return payload[0]
    return payload


def _max_abs_delta(a: list[float], b: list[float]) -> float:
    return max(abs(x - y) for x, y in zip(a, b))


def _vector_span(values: list[list[float]]) -> list[float]:
    if not values:
        return []
    return [
        max(row[index] for row in values) - min(row[index] for row in values)
        for index in range(len(values[0]))
    ]


def _collect_state_flags(backend: JakaSDKBackend) -> dict[str, Any]:
    robot = getattr(backend, "_robot", None)
    flags: dict[str, Any] = {}
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
            flags[method_name] = _extract_scalar(backend.call_sdk_method(method_name))
    if hasattr(robot, "get_last_error"):
        flags["last_error_raw"] = _extract_scalar(backend.call_sdk_method("get_last_error"))
    return flags


def _find_blockers(flags: dict[str, Any]) -> list[str]:
    blockers: list[str] = []
    if flags.get("is_in_estop", 0) != 0:
        blockers.append("robot_in_estop")
    if flags.get("is_in_collision", 0) != 0:
        blockers.append("robot_in_collision")
    if flags.get("is_on_limit", 0) != 0:
        blockers.append("robot_on_limit")
    if flags.get("is_in_drag_mode", 0) != 0:
        blockers.append("robot_in_drag_mode")
    if flags.get("is_in_servomove", 0) != 0:
        blockers.append("robot_already_in_servomove")
    last_error = flags.get("last_error_raw")
    if isinstance(last_error, list) and last_error and last_error[0] != 0:
        blockers.append("robot_last_error")
    return blockers


def _ensure_success(result: Any, method_name: str) -> None:
    err = result[0] if isinstance(result, tuple) else result
    if err != 0:
        raise RuntimeError(f"JAKA SDK call {method_name} failed with code {err}.")


def _call_servo(
    backend: JakaSDKBackend,
    method: str,
    target: list[float],
    step_num: int,
    robot_index: int,
) -> Any:
    if method == "edg_servo_j":
        return backend.call_sdk_method(method, target, ABS_MOVE_MODE, step_num, robot_index)
    return backend.call_sdk_method(method, target, ABS_MOVE_MODE, step_num)


def _write_output(path: str | None, result: dict[str, Any]) -> None:
    if not path:
        return
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Run a bounded absolute joint servo stream on a JAKA arm and return to the "
            "starting joint state."
        )
    )
    parser.add_argument("--config", default="configs/robot/jaka_mini2_real.yaml")
    parser.add_argument("--ip", default=None)
    parser.add_argument("--method", choices=("servo_j", "edg_servo_j"), default="edg_servo_j")
    parser.add_argument("--joint-index", type=int, default=6, help="1-based joint index.")
    parser.add_argument("--amplitude-rad", type=float, default=0.003)
    parser.add_argument("--max-amplitude-rad", type=float, default=0.01)
    parser.add_argument("--frequency-hz", type=float, default=0.25)
    parser.add_argument("--duration-sec", type=float, default=2.0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--state-sample-hz", type=float, default=10.0)
    parser.add_argument("--step-num", type=int, default=1)
    parser.add_argument("--robot-index", type=int, default=0)
    parser.add_argument("--edg-stat-ip", default="auto")
    parser.add_argument("--settle-sec", type=float, default=0.5)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", default=None)
    args = parser.parse_args()

    if not 1 <= args.joint_index <= 6:
        raise SystemExit("--joint-index must be within [1, 6].")
    if abs(args.amplitude_rad) > args.max_amplitude_rad:
        raise SystemExit("--amplitude-rad exceeds --max-amplitude-rad.")
    if args.duration_sec <= 0.0:
        raise SystemExit("--duration-sec must be positive.")
    if args.rate_hz <= 0.0:
        raise SystemExit("--rate-hz must be positive.")
    if args.step_num < 1:
        raise SystemExit("--step-num must be >= 1.")

    config_path = Path(args.config)
    config = load_yaml(config_path)
    config["mode"] = "real"
    config["backend_type"] = "jaka_sdk"
    config["default_speed_scale"] = 0.02
    if args.ip:
        config["ip"] = args.ip

    adapter = JakaDriverAdapter(config)
    backend = adapter.backend
    result: dict[str, Any] = {
        "config": str(config_path.resolve()),
        "ip": config["ip"],
        "method": args.method,
        "joint_index": args.joint_index,
        "amplitude_rad": args.amplitude_rad,
        "max_amplitude_rad": args.max_amplitude_rad,
        "frequency_hz": args.frequency_hz,
        "duration_sec": args.duration_sec,
        "rate_hz": args.rate_hz,
        "state_sample_hz": args.state_sample_hz,
        "step_num": args.step_num,
        "robot_index": args.robot_index,
        "edg_stat_ip": args.edg_stat_ip,
        "execute": args.execute,
    }

    try:
        adapter.connect()
        result["connect_ok"] = True
        if not isinstance(backend, JakaSDKBackend) or getattr(backend, "_robot", None) is None:
            raise RuntimeError("Servo stream requires the official JAKA SDK backend.")

        flags_before = _collect_state_flags(backend)
        result["state_flags_before"] = flags_before
        blockers = _find_blockers(flags_before)

        start_state = adapter.get_joint_state()
        time.sleep(args.settle_sec)
        confirm_state = adapter.get_joint_state()
        result["start_joint_state"] = start_state.to_dict()
        result["confirm_joint_state"] = confirm_state.to_dict()
        result["stationary_delta_rad"] = _max_abs_delta(
            start_state.positions, confirm_state.positions
        )
        if result["stationary_delta_rad"] > 0.002:
            blockers.append("robot_not_stationary")

        result["precheck_blockers"] = blockers
        result["precheck_ok"] = len(blockers) == 0
        if not args.execute:
            result["action"] = "precheck_only"
            return
        if blockers:
            raise RuntimeError(f"Servo stream blocked by precheck: {blockers}")

        if args.method == "edg_servo_j":
            edg_stat_ip = _resolve_edg_stat_ip(config["ip"], args.edg_stat_ip)
            result["resolved_edg_stat_ip"] = edg_stat_ip
            _ensure_success(backend.call_sdk_method("edg_init", True, edg_stat_ip), "edg_init")
            result["edg_init_ok"] = True

        _ensure_success(backend.call_sdk_method("servo_move_enable", True), "servo_move_enable(True)")
        result["servo_move_enable_ok"] = True
        result["state_flags_after_enable"] = _collect_state_flags(backend)

        base = [float(v) for v in confirm_state.positions]
        period = 1.0 / args.rate_hz
        start_perf = time.perf_counter()
        next_tick = start_perf
        command_times: list[float] = []
        state_samples: list[dict[str, Any]] = []
        failures: list[dict[str, Any]] = []
        sent_count = 0
        sample_period = 1.0 / args.state_sample_hz if args.state_sample_hz > 0.0 else math.inf
        next_sample = start_perf

        while True:
            now = time.perf_counter()
            elapsed = now - start_perf
            if elapsed > args.duration_sec:
                break
            target = list(base)
            target[args.joint_index - 1] += args.amplitude_rad * math.sin(
                2.0 * math.pi * args.frequency_hz * elapsed
            )
            call_started = time.perf_counter()
            sdk_result = _call_servo(backend, args.method, target, args.step_num, args.robot_index)
            command_times.append(call_started)
            err = sdk_result[0] if isinstance(sdk_result, tuple) else sdk_result
            if err != 0:
                failures.append({"command_index": sent_count, "sdk_result": repr(sdk_result)})
                break
            sent_count += 1
            if elapsed >= next_sample - start_perf:
                sample = adapter.get_joint_state()
                state_samples.append(
                    {
                        "elapsed_sec": elapsed,
                        "positions": sample.positions,
                    }
                )
                next_sample += sample_period
            next_tick += period
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

        # Stream a few center commands before leaving servo mode to avoid ending on a sine offset.
        for _ in range(max(3, int(0.1 * args.rate_hz))):
            sdk_result = _call_servo(backend, args.method, base, args.step_num, args.robot_index)
            err = sdk_result[0] if isinstance(sdk_result, tuple) else sdk_result
            if err != 0:
                failures.append({"command_index": sent_count, "sdk_result": repr(sdk_result)})
                break
            sent_count += 1
            time.sleep(period)

        _ensure_success(
            backend.call_sdk_method("servo_move_enable", False), "servo_move_enable(False)"
        )
        result["servo_move_disable_ok"] = True

        end_state = adapter.get_joint_state()
        intervals = [
            command_times[i + 1] - command_times[i] for i in range(len(command_times) - 1)
        ]
        result["action"] = "servo_joint_stream"
        result["command_count"] = sent_count
        result["failure_count"] = len(failures)
        result["failures"] = failures
        result["command_rate_hz_observed"] = (
            (len(command_times) - 1) / (command_times[-1] - command_times[0])
            if len(command_times) >= 2 and command_times[-1] > command_times[0]
            else None
        )
        result["command_interval_sec_min"] = min(intervals) if intervals else None
        result["command_interval_sec_max"] = max(intervals) if intervals else None
        result["state_sample_count"] = len(state_samples)
        result["state_samples"] = state_samples
        result["state_span_rad"] = _vector_span([sample["positions"] for sample in state_samples])
        result["end_joint_state"] = end_state.to_dict()
        result["end_start_error_rad"] = _max_abs_delta(end_state.positions, base)
        result["state_flags_after_disable"] = _collect_state_flags(backend)
        if failures:
            raise RuntimeError(f"Servo stream had SDK failures: {failures}")
    except Exception as exc:
        result["connect_ok"] = result.get("connect_ok", False)
        result["error"] = str(exc)
        raise
    finally:
        if isinstance(backend, JakaSDKBackend):
            try:
                if result.get("servo_move_enable_ok") and not result.get("servo_move_disable_ok"):
                    backend.call_sdk_method("servo_move_enable", False)
            except Exception as exc:
                result.setdefault("servo_disable_error", str(exc))
            try:
                backend.disconnect()
            except Exception as exc:
                result.setdefault("disconnect_error", str(exc))
        _write_output(args.output, result)
        print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
