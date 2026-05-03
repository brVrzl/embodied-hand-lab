from __future__ import annotations

import argparse
import json
import socket
import time
from typing import Any

from rh56_driver.jaka_tio_signal_client import extract_tio_signal_values


RH56_REGISTERS = {
    "angle": 1546,
    "force": 1582,
    "current": 1594,
    "status": 1612,
    "error": 1606,
    "temp": 1618,
}


def _signal_name(prefix: str, index: int) -> str:
    return f"rh56_{prefix}_{index}"


def _decode_json_objects(raw: bytes) -> list[Any]:
    text = raw.decode("utf-8", errors="replace").strip()
    if not text:
        return []
    decoder = json.JSONDecoder()
    values: list[Any] = []
    index = 0
    while index < len(text):
        while index < len(text) and text[index].isspace():
            index += 1
        if index >= len(text):
            break
        value, next_index = decoder.raw_decode(text, index)
        values.append(value)
        index = next_index
    return values


class JakaTcpClient:
    def __init__(self, host: str, port: int, timeout: float, terminator: bytes) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self.terminator = terminator

    def command(self, payload: dict[str, Any]) -> dict[str, Any]:
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8") + self.terminator
        chunks: list[bytes] = []
        with socket.create_connection((self.host, self.port), timeout=self.timeout) as sock:
            sock.settimeout(self.timeout)
            sock.sendall(data)
            try:
                while True:
                    chunk = sock.recv(65536)
                    if not chunk:
                        break
                    chunks.append(chunk)
                    try:
                        decoded = _decode_json_objects(b"".join(chunks))
                    except json.JSONDecodeError:
                        continue
                    if decoded:
                        return {
                            "tx": payload,
                            "rx_raw": b"".join(chunks).decode("utf-8", errors="replace"),
                            "rx": decoded,
                        }
            except socket.timeout:
                pass
        raw = b"".join(chunks)
        decoded: list[Any] = []
        if raw:
            try:
                decoded = _decode_json_objects(raw)
            except json.JSONDecodeError:
                decoded = []
        return {
            "tx": payload,
            "rx_raw": raw.decode("utf-8", errors="replace"),
            "rx": decoded,
        }


def _ok(response: dict[str, Any]) -> bool:
    rx = response.get("rx", [])
    if not rx or not isinstance(rx[0], dict):
        return False
    return str(rx[0].get("errorCode")) == "0"


def _cmd(out: dict[str, Any], client: JakaTcpClient, label: str, payload: dict[str, Any]) -> dict[str, Any]:
    try:
        response = client.command(payload)
    except Exception as exc:
        response = {"tx": payload, "error": str(exc)}
    out[label] = response
    return response


def _signals_from_response(response: dict[str, Any]) -> dict[str, Any]:
    rx = response.get("rx", [])
    if not rx or not isinstance(rx[0], dict):
        return {}
    payload = rx[0]
    signals: dict[str, Any] = {}

    raw_signals = payload.get("signals")
    if isinstance(raw_signals, list):
        for index, item in enumerate(raw_signals):
            if isinstance(item, dict):
                name = (
                    item.get("name")
                    or item.get("sigName")
                    or item.get("sig_name")
                    or item.get("tio_signal_name")
                    or item.get("signalName")
                    or f"signal_{index}"
                )
                signals[str(name)] = item
            elif isinstance(item, list) and len(item) >= 6:
                name, chn_id, sig_type, sig_addr, value, frequency = item[:6]
                signals[str(name)] = {
                    "chnId": chn_id,
                    "sigType": sig_type,
                    "sigAddr": sig_addr,
                    "value": value,
                    "frequency": frequency,
                }

    for key, value in payload.items():
        if key in {"cmdName", "errorCode", "errorMsg", "num", "signals"}:
            continue
        if isinstance(value, dict) and {"sigType", "sigAddr", "chnId", "frequency", "value"} & set(value):
            signals[key] = value

    return signals


def _signal_names_from_response(response: dict[str, Any]) -> list[str]:
    return list(_signals_from_response(response).keys())


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Probe JAKA TIO RS485 semaphores through the official TCP/IP JSON protocol on port 10001."
    )
    parser.add_argument("--host", default="192.168.71.50")
    parser.add_argument("--port", type=int, default=10001)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--terminator", choices=("none", "lf", "crlf"), default="none")
    parser.add_argument("--channel-id", type=int, default=1, help="Official docs: 0 RS485H/channel 1, 1 RS485L/channel 2.")
    parser.add_argument("--slave-id", type=int, default=1)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--databit", type=int, default=8)
    parser.add_argument("--stopbit", type=int, default=1)
    parser.add_argument("--parity", type=int, default=78)
    parser.add_argument("--sig-type", type=int, default=3, help="Modbus function code. 3 means read holding registers.")
    parser.add_argument("--frequency-hz", type=float, default=0.0)
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        default=None,
        help="Override the first signal address, e.g. 512, 773, 0x060A. Applies before --address-stride.",
    )
    parser.add_argument("--address-stride", type=int, default=2)
    parser.add_argument("--groups", nargs="+", default=["angle"], choices=sorted(RH56_REGISTERS))
    parser.add_argument("--single-signal", action="store_true")
    parser.add_argument("--prepare", action="store_true", help="Set TIO power, AI pin mode, RS485 mode, and Modbus comm params.")
    parser.add_argument("--add-signals", action="store_true")
    parser.add_argument("--delete-first", action="store_true")
    parser.add_argument("--delete-existing", action="store_true", help="Delete all currently defined TIO RS485 signals before adding.")
    parser.add_argument("--compact", action="store_true", help="Print only mode/comm and polled signal values.")
    parser.add_argument("--polls", type=int, default=3)
    parser.add_argument("--poll-sec", type=float, default=0.5)
    args = parser.parse_args()

    terminator = {"none": b"", "lf": b"\n", "crlf": b"\r\n"}[args.terminator]
    client = JakaTcpClient(args.host, args.port, args.timeout, terminator)

    signal_specs = [
        (
            group,
            idx,
            _signal_name(group, idx),
            (args.address if args.address is not None else RH56_REGISTERS[group]) + idx * args.address_stride,
        )
        for group in args.groups
        for idx in range(6)
    ]
    if args.single_signal:
        signal_specs = signal_specs[:1]

    out: dict[str, Any] = {
        "host": args.host,
        "port": args.port,
        "channel_id": args.channel_id,
        "slave_id": args.slave_id,
        "notes": [
            "Uses official JAKA TCP/IP JSON protocol, not jkrc.so.",
            "No robot motion commands are sent.",
            "Raw RS485 send can transmit bytes but cannot expose slave responses; this tool tests controller-side semaphores.",
        ],
    }

    _cmd(out, client, "get_tio_vout_param", {"cmdName": "get_tio_vout_param"})
    _cmd(out, client, "get_tio_pin_mode", {"cmdName": "get_tio_pin_mode", "pinType": 2})
    _cmd(out, client, "get_rs485_chn_mode", {"cmdName": "get_rs485_chn_mode", "chnId": args.channel_id})
    _cmd(out, client, "get_rs485_chn_comm", {"cmdName": "get_rs485_chn_comm", "chn_id": args.channel_id})
    _cmd(out, client, "get_rs485_signal_list_initial", {"cmdName": "get_rs485_signal_list"})
    _cmd(out, client, "get_tio_signals_initial", {"cmdName": "get_tio_signals"})

    if args.prepare:
        _cmd(out, client, "set_tio_vout_param", {"cmdName": "set_tio_vout_param", "tio_vout_ena": 1, "tio_vout_vol": 0})
        _cmd(out, client, "set_tio_pin_mode", {"cmdName": "set_tio_pin_mode", "pinType": 2, "pinMode": 1})
        _cmd(out, client, "set_rs485_chn_mode", {"cmdName": "set_rs485_chn_mode", "chnId": args.channel_id, "chnMode": 0})
        _cmd(
            out,
            client,
            "set_rs485_chn_comm",
            {
                "cmdName": "set_rs485_chn_comm",
                "chn_id": args.channel_id,
                "slaveId": args.slave_id,
                "baudrate": args.baudrate,
                "databit": args.databit,
                "stopbit": args.stopbit,
                "parity": args.parity,
            },
        )
        _cmd(out, client, "get_rs485_chn_mode_after_set", {"cmdName": "get_rs485_chn_mode", "chnId": args.channel_id})
        _cmd(out, client, "get_rs485_chn_comm_after_set", {"cmdName": "get_rs485_chn_comm", "chn_id": args.channel_id})

    if args.delete_existing:
        names = set(_signal_names_from_response(out["get_rs485_signal_list_initial"]))
        names.update(_signal_names_from_response(out["get_tio_signals_initial"]))
        for name in sorted(names):
            _cmd(out.setdefault("delete_existing", {}), client, name, {"cmdName": "del_tio_rs_signal", "tio_signal_name": name})

    if args.delete_first:
        for _, _, name, _ in signal_specs:
            _cmd(out.setdefault("delete", {}), client, name, {"cmdName": "del_tio_rs_signal", "tio_signal_name": name})

    if args.add_signals:
        add_results: dict[str, Any] = {}
        for _, _, name, address in signal_specs:
            _cmd(
                add_results,
                client,
                name,
                {
                    "cmdName": "add_tio_rs_signal",
                    "tio_signal_name": name,
                    "tio_signal_chnId": args.channel_id,
                    "tio_signal_sigType": args.sig_type,
                    "tio_signal_sigAddr": address,
                    "frequency": args.frequency_hz,
                },
            )
        out["add_signals"] = add_results

    post_add = _cmd(out, client, "get_rs485_signal_list_post_add", {"cmdName": "get_rs485_signal_list"})
    post_add_tio = _cmd(out, client, "get_tio_signals_post_add", {"cmdName": "get_tio_signals"})
    out["post_add_values"] = {
        "rs485_signal_list": _signals_from_response(post_add),
        "tio_signals": _signals_from_response(post_add_tio),
    }

    polls: list[dict[str, Any]] = []
    for _ in range(args.polls):
        time.sleep(max(args.poll_sec, 0.0))
        try:
            rs485_response = client.command({"cmdName": "get_rs485_signal_list"})
        except Exception as exc:
            rs485_response = {"tx": {"cmdName": "get_rs485_signal_list"}, "error": str(exc)}
        try:
            tio_response = client.command({"cmdName": "get_tio_signals"})
        except Exception as exc:
            tio_response = {"tx": {"cmdName": "get_tio_signals"}, "error": str(exc)}
        polls.append(
            {
                "rs485_signal_list": rs485_response,
                "tio_signals": tio_response,
                "values": {
                    "rs485_signal_list": _signals_from_response(rs485_response),
                    "tio_signals": _signals_from_response(tio_response),
                },
            }
        )
    out["polls"] = polls
    _cmd(out, client, "get_rs485_signal_list_final", {"cmdName": "get_rs485_signal_list"})
    _cmd(out, client, "get_tio_signals_final", {"cmdName": "get_tio_signals"})

    if args.compact:
        compact = {
            "host": args.host,
            "channel_id": args.channel_id,
            "mode": out.get("get_rs485_chn_mode", {}).get("rx", [{}])[0].get("chnMode"),
            "comm": out.get("get_rs485_chn_comm", {}).get("rx", [{}])[0],
            "initial": extract_tio_signal_values(out["get_rs485_signal_list_initial"]),
            "polls": [
                {
                    "rs485": extract_tio_signal_values(poll["rs485_signal_list"]),
                    "tio": extract_tio_signal_values(poll["tio_signals"]),
                }
                for poll in polls
            ],
            "final": extract_tio_signal_values(out["get_rs485_signal_list_final"]),
        }
        print(json.dumps(compact, indent=2, ensure_ascii=False))
    else:
        print(json.dumps(out, indent=2, ensure_ascii=False))
    failed_prepare = any(
        key.startswith("set_") and isinstance(value, dict) and not _ok(value)
        for key, value in out.items()
    )
    raise SystemExit(1 if failed_prepare else 0)


if __name__ == "__main__":
    main()
