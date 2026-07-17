"""Input-only UDP transport and raw capture/replay for HTS datagrams."""

from __future__ import annotations

import base64
import binascii
from dataclasses import dataclass
import json
from pathlib import Path
import socket
import time
from typing import Callable, Iterator, TextIO

from .errors import SerializationError, SourceDisconnected
from .hts_protocol import HTS_DEFAULT_UDP_PORT, HTS_MAX_DATAGRAM_BYTES


HTS_RAW_RECORDING_FORMAT = "hts-raw-jsonl"
HTS_RAW_RECORDING_VERSION = "1.0"


@dataclass(frozen=True, slots=True)
class ReceivedHtsDatagram:
    payload: bytes
    source_address: str
    source_port: int
    receive_monotonic_ns: int
    receive_unix_ns: int

    @property
    def source_endpoint(self) -> str:
        return f"{self.source_address}:{self.source_port}"


class HtsUdpReceiver:
    """A bounded datagram transport with no parser or control dependencies."""

    def __init__(
        self,
        bind_address: str = "0.0.0.0",
        port: int = HTS_DEFAULT_UDP_PORT,
        *,
        allowed_sender: str | None = None,
        max_datagram_bytes: int = HTS_MAX_DATAGRAM_BYTES,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        unix_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        if not 1 <= port <= 65535:
            raise ValueError("HTS UDP port must be in [1, 65535]")
        if max_datagram_bytes < 1 or max_datagram_bytes > 65535:
            raise ValueError("HTS max_datagram_bytes must be in [1, 65535]")
        self.bind_address = bind_address
        self.port = port
        self.allowed_sender = allowed_sender
        self.max_datagram_bytes = max_datagram_bytes
        self._monotonic_ns = monotonic_ns
        self._unix_ns = unix_ns
        self._socket: socket.socket | None = None

    @property
    def local_endpoint(self) -> tuple[str, int]:
        if self._socket is None:
            raise SourceDisconnected("HTS UDP receiver is closed")
        address, port = self._socket.getsockname()
        return str(address), int(port)

    def open(self) -> None:
        if self._socket is not None:
            raise RuntimeError("HTS UDP receiver is already open")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.bind_address, self.port))
        self._socket = sock

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def __enter__(self) -> "HtsUdpReceiver":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def receive(self, timeout_s: float | None = None) -> ReceivedHtsDatagram | None:
        if self._socket is None:
            raise SourceDisconnected("HTS UDP receiver is closed")
        if timeout_s is not None and timeout_s < 0:
            raise ValueError("receive timeout must be non-negative")
        self._socket.settimeout(timeout_s)
        try:
            payload, sender = self._socket.recvfrom(self.max_datagram_bytes + 1)
        except socket.timeout:
            return None
        except OSError as exc:
            raise SourceDisconnected(f"HTS UDP receive failed: {exc}") from exc
        receive_monotonic_ns = self._monotonic_ns()
        receive_unix_ns = self._unix_ns()
        if len(payload) > self.max_datagram_bytes:
            raise SerializationError(
                "HTS UDP datagram was truncated/rejected because it exceeds "
                f"{self.max_datagram_bytes} bytes"
            )
        if self.allowed_sender is not None and sender[0] != self.allowed_sender:
            raise SerializationError(
                f"HTS datagram from unapproved sender {sender[0]!r}"
            )
        return ReceivedHtsDatagram(
            payload=payload,
            source_address=str(sender[0]),
            source_port=int(sender[1]),
            receive_monotonic_ns=receive_monotonic_ns,
            receive_unix_ns=receive_unix_ns,
        )


def inspect_datagram(datagram: ReceivedHtsDatagram, *, preview_bytes: int = 160) -> dict[str, object]:
    prefix = datagram.payload[:preview_bytes]
    try:
        utf8_preview: str | None = prefix.decode("utf-8", errors="strict")
    except UnicodeDecodeError:
        utf8_preview = None
    delimiter_counts = {
        "newline": datagram.payload.count(b"\n"),
        "comma": datagram.payload.count(b","),
        "colon": datagram.payload.count(b":"),
        "pipe": datagram.payload.count(b"|"),
    }
    return {
        "source_endpoint": datagram.source_endpoint,
        "datagram_bytes": len(datagram.payload),
        "receive_monotonic_ns": datagram.receive_monotonic_ns,
        "receive_unix_ns": datagram.receive_unix_ns,
        "first_bytes_hex": prefix.hex(" "),
        "utf8_preview": utf8_preview,
        "apparent_delimiters": delimiter_counts,
    }


class HtsRawRecordingWriter:
    def __init__(self, path: str | Path, *, metadata: dict[str, object] | None = None) -> None:
        self.path = Path(path)
        self.metadata = dict(metadata or {})
        self._file: TextIO | None = None
        self.datagram_count = 0

    def open(self) -> None:
        if self._file is not None:
            raise RuntimeError("HTS raw recording is already open")
        self._file = self.path.open("x", encoding="utf-8")
        self._write(
            {
                "record_type": "header",
                "format": HTS_RAW_RECORDING_FORMAT,
                "version": HTS_RAW_RECORDING_VERSION,
                "created_unix_ns": time.time_ns(),
                "metadata": self.metadata,
            }
        )

    def write(self, datagram: ReceivedHtsDatagram) -> None:
        if self._file is None:
            raise RuntimeError("HTS raw recording is closed")
        self._write(
            {
                "record_type": "datagram",
                "payload_base64": base64.b64encode(datagram.payload).decode("ascii"),
                "source_address": datagram.source_address,
                "source_port": datagram.source_port,
                "receive_monotonic_ns": datagram.receive_monotonic_ns,
                "receive_unix_ns": datagram.receive_unix_ns,
            }
        )
        self.datagram_count += 1

    def close(self) -> None:
        if self._file is None:
            return
        self._write(
            {
                "record_type": "footer",
                "datagram_count": self.datagram_count,
                "closed_unix_ns": time.time_ns(),
            }
        )
        self._file.close()
        self._file = None

    def __enter__(self) -> "HtsRawRecordingWriter":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _write(self, record: dict[str, object]) -> None:
        assert self._file is not None
        self._file.write(json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n")
        self._file.flush()


class HtsRawRecordingReader:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)

    def datagrams(self) -> Iterator[ReceivedHtsDatagram]:
        with self.path.open("r", encoding="utf-8") as stream:
            first = stream.readline()
            if not first:
                raise SerializationError("HTS raw recording is empty")
            header = _load_record(first, line_number=1)
            if (
                header.get("record_type") != "header"
                or header.get("format") != HTS_RAW_RECORDING_FORMAT
                or str(header.get("version", "")).split(".", 1)[0]
                != HTS_RAW_RECORDING_VERSION.split(".", 1)[0]
            ):
                raise SerializationError("unsupported HTS raw recording header")
            for line_number, line in enumerate(stream, start=2):
                if not line.strip():
                    continue
                record = _load_record(line, line_number=line_number)
                if record.get("record_type") == "footer":
                    return
                if record.get("record_type") != "datagram":
                    raise SerializationError(
                        f"unexpected HTS raw record at line {line_number}"
                    )
                try:
                    payload = base64.b64decode(
                        str(record["payload_base64"]), validate=True
                    )
                    yield ReceivedHtsDatagram(
                        payload=payload,
                        source_address=str(record["source_address"]),
                        source_port=int(record["source_port"]),
                        receive_monotonic_ns=int(record["receive_monotonic_ns"]),
                        receive_unix_ns=int(record["receive_unix_ns"]),
                    )
                except (binascii.Error, KeyError, TypeError, ValueError) as exc:
                    raise SerializationError(
                        f"invalid HTS raw datagram at line {line_number}: {exc}"
                    ) from exc


def replay_datagrams(
    path: str | Path,
    *,
    as_recorded: bool = False,
    speed: float = 1.0,
    sleep: Callable[[float], None] = time.sleep,
) -> Iterator[ReceivedHtsDatagram]:
    if speed <= 0:
        raise ValueError("replay speed must be positive")
    previous: ReceivedHtsDatagram | None = None
    for datagram in HtsRawRecordingReader(path).datagrams():
        if as_recorded and previous is not None:
            interval_ns = max(
                0, datagram.receive_monotonic_ns - previous.receive_monotonic_ns
            )
            sleep(interval_ns / 1e9 / speed)
        yield datagram
        previous = datagram


def _load_record(line: str, *, line_number: int) -> dict[str, object]:
    try:
        record = json.loads(line)
    except json.JSONDecodeError as exc:
        raise SerializationError(
            f"invalid HTS raw JSON at line {line_number}: {exc}"
        ) from exc
    if not isinstance(record, dict):
        raise SerializationError(f"HTS raw record at line {line_number} is not an object")
    return record
