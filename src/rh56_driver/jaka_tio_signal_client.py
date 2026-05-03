from __future__ import annotations

import json
import socket
from typing import Any


def decode_json_objects(raw: bytes) -> list[Any]:
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


def extract_tio_signal_values(response: dict[str, Any]) -> dict[str, float]:
    rx = response.get("rx", [])
    if not rx or not isinstance(rx[0], dict):
        return {}

    payload = rx[0]
    values: dict[str, float] = {}

    raw_signals = payload.get("signals")
    if isinstance(raw_signals, list):
        for item in raw_signals:
            if isinstance(item, list) and len(item) >= 5:
                values[str(item[0])] = float(item[4])
            elif isinstance(item, dict):
                name = item.get("name") or item.get("sigName") or item.get("sig_name") or item.get("tio_signal_name")
                if name is not None and "value" in item:
                    values[str(name)] = float(item["value"])

    for key, value in payload.items():
        if key in {"cmdName", "errorCode", "errorMsg", "num", "signals"}:
            continue
        if isinstance(value, dict) and "value" in value:
            values[key] = float(value["value"])

    return values


class JakaTioSignalClient:
    def __init__(self, host: str, port: int = 10001, timeout: float = 0.2, terminator: bytes = b"") -> None:
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
                        decoded = decode_json_objects(b"".join(chunks))
                    except json.JSONDecodeError:
                        continue
                    if decoded:
                        return {"tx": payload, "rx_raw": b"".join(chunks).decode("utf-8", errors="replace"), "rx": decoded}
            except socket.timeout:
                pass

        raw = b"".join(chunks)
        decoded: list[Any] = []
        if raw:
            try:
                decoded = decode_json_objects(raw)
            except json.JSONDecodeError:
                decoded = []
        return {"tx": payload, "rx_raw": raw.decode("utf-8", errors="replace"), "rx": decoded}

    def get_signal_values(self) -> dict[str, float]:
        return extract_tio_signal_values(self.command({"cmdName": "get_rs485_signal_list"}))

    def add_signal(self, name: str, channel_id: int, signal_type: int, address: int, frequency_hz: float = 0.0) -> None:
        response = self.command(
            {
                "cmdName": "add_tio_rs_signal",
                "tio_signal_name": name,
                "tio_signal_chnId": channel_id,
                "tio_signal_sigType": signal_type,
                "tio_signal_sigAddr": address,
                "frequency": frequency_hz,
            }
        )
        rx = response.get("rx", [])
        if not rx or not isinstance(rx[0], dict) or str(rx[0].get("errorCode")) != "0":
            raise RuntimeError(f"JAKA TCP add_tio_rs_signal failed: {response}")

    def set_channel_mode(self, channel_id: int, channel_mode: int) -> None:
        response = self.command({"cmdName": "set_rs485_chn_mode", "chnId": channel_id, "chnMode": channel_mode})
        rx = response.get("rx", [])
        if not rx or not isinstance(rx[0], dict) or str(rx[0].get("errorCode")) != "0":
            raise RuntimeError(f"JAKA TCP set_rs485_chn_mode failed: {response}")

    def get_channel_mode(self, channel_id: int) -> int:
        response = self.command({"cmdName": "get_rs485_chn_mode", "chnId": channel_id})
        rx = response.get("rx", [])
        if not rx or not isinstance(rx[0], dict) or str(rx[0].get("errorCode")) != "0":
            raise RuntimeError(f"JAKA TCP get_rs485_chn_mode failed: {response}")
        return int(rx[0]["chnMode"])

    def send_raw_rs485(self, channel_id: int, data: bytes) -> None:
        response = self.command({"cmdName": "send_tio_rs_command", "chn_id": channel_id, "cmdBuf": list(data)})
        rx = response.get("rx", [])
        if not rx or not isinstance(rx[0], dict) or str(rx[0].get("errorCode")) != "0":
            raise RuntimeError(f"JAKA TCP send_tio_rs_command failed: {response}")
