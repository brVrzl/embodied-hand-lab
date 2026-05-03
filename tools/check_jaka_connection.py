from __future__ import annotations

import argparse
import json
import socket
from pathlib import Path
from typing import Any

from embodiment_core.config import load_yaml
from jaka_driver_adapter.adapter import JakaDriverAdapter
from jaka_driver_adapter.jaka_sdk_backend import JakaSDKBackend


def _extract_pose_payload(result: Any) -> dict[str, Any]:
    if isinstance(result, tuple) and len(result) >= 2:
        payload = result[1]
    else:
        payload = result
    if hasattr(payload, "tran") and hasattr(payload, "rpy"):
        return {
            "translation_mm": {
                "x": getattr(payload.tran, "x", None),
                "y": getattr(payload.tran, "y", None),
                "z": getattr(payload.tran, "z", None),
            },
            "rpy_rad": {
                "rx": getattr(payload.rpy, "rx", None),
                "ry": getattr(payload.rpy, "ry", None),
                "rz": getattr(payload.rpy, "rz", None),
            },
        }
    if isinstance(payload, (list, tuple)):
        return {"raw_pose": list(payload)}
    return {"raw_pose": repr(payload)}


def _extract_tuple_payload(result: Any) -> Any:
    if isinstance(result, tuple) and len(result) >= 2:
        return result[1]
    return result


def _extract_scalar_payload(result: Any) -> Any:
    payload = _extract_tuple_payload(result)
    if isinstance(payload, tuple) and len(payload) == 1:
        return payload[0]
    return payload


def _parse_controller_ip_payload(result: Any) -> dict[str, Any]:
    payload = _extract_scalar_payload(result)
    if not isinstance(payload, str):
        return {"raw": payload}
    stripped = payload.replace("\x00", "").strip()
    parts = stripped.split(";")
    parsed: dict[str, Any] = {"raw": stripped}
    if len(parts) >= 1:
        parsed["controller_serial"] = parts[0]
    if len(parts) >= 4:
        parsed["controller_family"] = parts[3]
    if len(parts) >= 5:
        parsed["python_sdk_build"] = parts[4]
    if ":" in stripped:
        parsed["reported_ip"] = stripped.rsplit(":", 1)[-1].strip()
    return parsed


def main() -> None:
    parser = argparse.ArgumentParser(description="Safe JAKA connectivity check without motion.")
    parser.add_argument("--config", default="configs/robot/jaka_mini2.yaml")
    parser.add_argument("--ip", default=None)
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_yaml(config_path)
    config["mode"] = "real"
    config["backend_type"] = "jaka_sdk"
    if args.ip:
        config["ip"] = args.ip

    adapter = JakaDriverAdapter(config)
    result: dict[str, Any] = {"config": str(config_path.resolve()), "ip": config["ip"]}
    backend = adapter.backend
    result["tcp_ports"] = _probe_ports(config["ip"], [22, 80, 10000, 10001, 10004])
    try:
        adapter.connect()
        result["connect_ok"] = True
        result["joint_state"] = adapter.get_joint_state().to_dict()

        if isinstance(backend, JakaSDKBackend):
            result.update(_collect_sdk_diagnostics(backend))
    except Exception as exc:
        result["connect_ok"] = False
        result["error"] = str(exc)
    finally:
        if isinstance(backend, JakaSDKBackend):
            try:
                backend.disconnect()
            except Exception as exc:
                result["disconnect_error"] = str(exc)
        print(json.dumps(result, indent=2, ensure_ascii=False))
        if result.get("connect_ok") is not True:
            raise SystemExit(1)


def _probe_ports(ip: str, ports: list[int]) -> dict[str, str]:
    statuses: dict[str, str] = {}
    for port in ports:
        sock = socket.socket()
        sock.settimeout(1.0)
        try:
            sock.connect((ip, port))
            statuses[str(port)] = "open"
        except Exception as exc:
            statuses[str(port)] = f"closed: {type(exc).__name__}"
        finally:
            sock.close()
    return statuses


def _collect_sdk_diagnostics(backend: JakaSDKBackend) -> dict[str, Any]:
    diagnostics: dict[str, Any] = {}
    robot = getattr(backend, "_robot", None)
    if robot is None:
        return diagnostics

    if hasattr(robot, "get_sdk_version"):
        diagnostics["sdk_version"] = _extract_scalar_payload(backend._invoke("get_sdk_version"))

    if hasattr(robot, "get_controller_ip"):
        diagnostics["controller_info"] = _parse_controller_ip_payload(
            backend._invoke("get_controller_ip")
        )

    if hasattr(robot, "get_rapidrate"):
        diagnostics["rapidrate"] = _extract_scalar_payload(backend._invoke("get_rapidrate"))

    if hasattr(robot, "get_tcp_position"):
        diagnostics["tcp_pose"] = _extract_pose_payload(backend._invoke("get_tcp_position"))

    if hasattr(robot, "get_actual_tcp_position"):
        diagnostics["actual_tcp_pose"] = _extract_pose_payload(
            backend._invoke("get_actual_tcp_position")
        )

    if hasattr(robot, "get_actual_joint_position"):
        diagnostics["actual_joint_state"] = {
            "names": list(backend._joint_names),
            "positions": list(_extract_scalar_payload(backend._invoke("get_actual_joint_position"))),
        }

    for method_name in (
        "is_in_estop",
        "is_in_collision",
        "is_on_limit",
        "is_in_drag_mode",
        "is_in_servomove",
        "is_in_pos",
    ):
        if hasattr(robot, method_name):
            diagnostics[method_name] = _extract_scalar_payload(backend._invoke(method_name))

    if hasattr(robot, "get_robot_state"):
        diagnostics["robot_state_raw"] = _extract_scalar_payload(backend._invoke("get_robot_state"))

    if hasattr(robot, "get_motion_status"):
        diagnostics["motion_status_raw"] = _extract_scalar_payload(
            backend._invoke("get_motion_status")
        )

    if hasattr(robot, "get_last_error"):
        diagnostics["last_error_raw"] = _extract_scalar_payload(backend._invoke("get_last_error"))

    return diagnostics


if __name__ == "__main__":
    main()
