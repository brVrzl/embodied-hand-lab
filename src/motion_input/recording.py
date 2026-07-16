"""Crash-tolerant, versioned newline-delimited UMIP recordings."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import time
from typing import Any, Iterator, Mapping, TextIO

from .errors import SerializationError
from .model import DeviceDescriptor, JSONValue, MotionInputSample, UMIP_VERSION
from .serialization import device_from_dict, device_to_dict, sample_from_dict, sample_to_dict


RECORDING_FORMAT_VERSION = "1.0"


@dataclass(frozen=True, slots=True)
class RecordingHeader:
    format_version: str
    umip_version: str
    recording_id: str
    created_timestamp_ns: int
    device: DeviceDescriptor
    metadata: Mapping[str, JSONValue]


class MotionRecordingWriter:
    """Streaming writer; every complete line remains recoverable after a crash."""

    def __init__(
        self,
        path: str | Path,
        *,
        recording_id: str,
        device: DeviceDescriptor,
        metadata: Mapping[str, JSONValue] | None = None,
        flush_every_sample: bool = False,
    ) -> None:
        if not recording_id.strip():
            raise ValueError("recording_id must not be empty")
        self.path = Path(path)
        self.header = RecordingHeader(
            format_version=RECORDING_FORMAT_VERSION,
            umip_version=UMIP_VERSION,
            recording_id=recording_id,
            created_timestamp_ns=time.time_ns(),
            device=device,
            metadata=dict(metadata or {}),
        )
        self.flush_every_sample = flush_every_sample
        self._handle: TextIO | None = None
        self._sample_count = 0

    @property
    def sample_count(self) -> int:
        return self._sample_count

    def open(self) -> None:
        if self._handle is not None:
            raise RuntimeError("recording writer is already open")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.path.open("x", encoding="utf-8", newline="\n")
        self._write_record(
            {
                "record_type": "header",
                "format_version": self.header.format_version,
                "umip_version": self.header.umip_version,
                "recording_id": self.header.recording_id,
                "created_timestamp_ns": self.header.created_timestamp_ns,
                "device": device_to_dict(self.header.device),
                "metadata": dict(self.header.metadata),
            }
        )
        self._handle.flush()

    def write(self, sample: MotionInputSample) -> None:
        if self._handle is None:
            raise RuntimeError("recording writer is not open")
        self._write_record({"record_type": "sample", "sample": sample_to_dict(sample)})
        self._sample_count += 1
        if self.flush_every_sample:
            self._handle.flush()

    def close(self) -> None:
        if self._handle is None:
            return
        try:
            self._write_record(
                {
                    "record_type": "footer",
                    "sample_count": self._sample_count,
                    "closed_timestamp_ns": time.time_ns(),
                }
            )
            self._handle.flush()
        finally:
            self._handle.close()
            self._handle = None

    def _write_record(self, record: Mapping[str, Any]) -> None:
        assert self._handle is not None
        self._handle.write(
            json.dumps(
                record,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            )
            + "\n"
        )

    def __enter__(self) -> "MotionRecordingWriter":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


class MotionRecordingReader:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.header: RecordingHeader | None = None
        self.footer_sample_count: int | None = None

    def read_header(self) -> RecordingHeader:
        """Read only far enough to validate and return the header."""

        iterator = self.samples()
        try:
            next(iterator)
        except StopIteration:
            pass
        finally:
            iterator.close()
        if self.header is None:
            raise SerializationError(f"{self.path}: missing recording header")
        return self.header

    def samples(self) -> Iterator[MotionInputSample]:
        self.header = None
        self.footer_sample_count = None
        observed_sample_count = 0
        with self.path.open("r", encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, start=1):
                if not line.strip():
                    continue
                try:
                    record = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise SerializationError(
                        f"{self.path}:{line_number}: invalid JSON: {exc}"
                    ) from exc
                if not isinstance(record, dict):
                    raise SerializationError(f"{self.path}:{line_number}: record must be an object")
                record_type = record.get("record_type")
                if record_type == "header":
                    if self.header is not None:
                        raise SerializationError(f"{self.path}:{line_number}: duplicate header")
                    self.header = self._parse_header(record, line_number)
                elif record_type == "sample":
                    if self.header is None:
                        raise SerializationError(f"{self.path}:{line_number}: sample before header")
                    try:
                        sample = sample_from_dict(record["sample"])
                    except (KeyError, SerializationError) as exc:
                        raise SerializationError(f"{self.path}:{line_number}: {exc}") from exc
                    observed_sample_count += 1
                    yield sample
                elif record_type == "footer":
                    self.footer_sample_count = int(record["sample_count"])
                else:
                    # Same-major minor revisions may add record types. Old readers skip them.
                    continue
        if self.header is None:
            raise SerializationError(f"{self.path}: missing recording header")
        if self.footer_sample_count is not None and self.footer_sample_count != observed_sample_count:
            raise SerializationError(
                f"{self.path}: footer count {self.footer_sample_count} does not match "
                f"{observed_sample_count} sample records"
            )

    def _parse_header(self, record: Mapping[str, Any], line_number: int) -> RecordingHeader:
        try:
            version = str(record["format_version"])
            if version.split(".", 1)[0] != RECORDING_FORMAT_VERSION.split(".", 1)[0]:
                raise SerializationError(f"unsupported recording major version {version!r}")
            return RecordingHeader(
                format_version=version,
                umip_version=str(record["umip_version"]),
                recording_id=str(record["recording_id"]),
                created_timestamp_ns=int(record["created_timestamp_ns"]),
                device=device_from_dict(record["device"]),
                metadata=dict(record.get("metadata", {})),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise SerializationError(f"{self.path}:{line_number}: invalid header: {exc}") from exc
